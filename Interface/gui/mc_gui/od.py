"""Object-dictionary model parsed at runtime from the shared firmware contract.

The single source of truth is ``mc_if_od.h`` (the ``MC_IF_OD_OBJECTS(X)`` X-macro
plus the scaling constants and type/access enums). We parse that header directly so
the GUI can never drift from the firmware. See ``../INTERFACE_SPEC.md``.

CiA-402 standard objects (0x1xxx / 0x6xxx) are scaled integers; manufacturer
objects (0x2xxx) are FLOAT32 already in SI units. ``raw`` is the on-the-wire value,
``si`` is the engineering value the user sees (``si = raw * scale``).
"""
from __future__ import annotations

import re
import struct
from dataclasses import dataclass, field
from pathlib import Path

# --- OD data types (must match MC_IfOdType_t / MC_OdType_t) --------------------
T_U8, T_U16, T_U32, T_I8, T_I16, T_I32, T_F32 = range(7)

_TYPE_BY_NAME = {
    "MC_IF_T_U8": T_U8, "MC_IF_T_U16": T_U16, "MC_IF_T_U32": T_U32,
    "MC_IF_T_I8": T_I8, "MC_IF_T_I16": T_I16, "MC_IF_T_I32": T_I32,
    "MC_IF_T_F32": T_F32,
}
TYPE_NAME = {v: k.split("_")[-1] for k, v in _TYPE_BY_NAME.items()}  # 0 -> "U8"

# struct format, size (bytes), is-float
_TYPE_FMT = {
    T_U8: ("<B", 1, False), T_U16: ("<H", 2, False), T_U32: ("<I", 4, False),
    T_I8: ("<b", 1, False), T_I16: ("<h", 2, False), T_I32: ("<i", 4, False),
    T_F32: ("<f", 4, True),
}

# --- access rights (MC_IfOdAccess_t) ------------------------------------------
A_RO, A_WO, A_RW = 1, 2, 3
ACCESS_NAME = {A_RO: "RO", A_WO: "WO", A_RW: "RW"}
_ACCESS_BY_NAME = {"MC_IF_A_RO": A_RO, "MC_IF_A_WO": A_WO, "MC_IF_A_RW": A_RW}

# --- entry flags ---------------------------------------------------------------
F_NONE, F_PDO, F_PERSIST = 0x00, 0x01, 0x02
_FLAG_BY_NAME = {"MC_IF_F_NONE": F_NONE, "MC_IF_F_PDO": F_PDO, "MC_IF_F_PERSIST": F_PERSIST}

# --- entry owner (MC_IfOdOwner_t, added in protocol v2) ---------------------------
# Which firmware actually handles reads/writes for the entry. Motor-owned entries
# travel over SPI via the CMC's cia402 OD pipeline; CMC-owned entries (0x3xxx,
# axis_manager) are handled locally on the CMC and never reach the motor MCU.
OWNER_MOTOR, OWNER_CMC = 0, 1
OWNER_NAME = {OWNER_MOTOR: "motor", OWNER_CMC: "CMC"}
_OWNER_BY_NAME = {"MC_IF_OWNER_MOTOR": OWNER_MOTOR, "MC_IF_OWNER_CMC": OWNER_CMC}

# Telemetry map object (0x2A00); 16 array sub-entries are U32 map words.
TLM_MAP_INDEX = 0x2A00
TLM_MAX_ENTRIES = 16
TLM_MAX_BYTES = 40

# Fallback scale factors if the header cannot be parsed (matches mc_if_od.h).
_DEFAULT_SCALES = {"POS": 1.0e-5, "VEL": 1.0e-3, "ACC": 1.0e-3, "CUR": 1.0e-3, "TRQ": 1.0e-3}

# Which CiA-402 index uses which scaled quantity.
_QUANTITY_BY_INDEX = {
    0x607A: "POS", 0x6064: "POS",
    0x60FF: "VEL", 0x606C: "VEL", 0x6081: "VEL",
    0x6083: "ACC", 0x6084: "ACC", 0x6085: "ACC",
    0x6071: "CUR", 0x6077: "CUR",
}
_QUANTITY_UNIT = {"POS": "rad", "VEL": "rad/s", "ACC": "rad/s^2", "CUR": "A", "TRQ": "Nm"}

# Unit inference for the float32 manufacturer objects, by name suffix (longest first).
_UNIT_SUFFIXES = [
    ("_rad_per_s", "rad/s"), ("_rad_s", "rad/s"), ("_kg_m2", "kg.m^2"),
    ("_nm_per_a", "Nm/A"), ("_rad", "rad"), ("_nm", "Nm"), ("_ohm", "ohm"),
    ("_hz", "Hz"), ("_v", "V"), ("_a", "A"), ("_h", "H"),
]

# Explicit units for float32 names the suffix heuristic can't catch (accel/jerk knobs etc.).
_UNIT_BY_NAME = {
    "max_accel_rad_s2": "rad/s^2",
    "vel_accel_up": "rad/s^2", "vel_accel_dn": "rad/s^2", "vel_accel_jerk": "rad/s^3",
    "hb_cur_bandwidth": "rad/s",
    "thermal_tau_s": "s",   # thermal_i_cont_a gets "A" from the _a suffix heuristic (ADR-065)
}

# Entries shown as hex (bitfields / CiA error codes), keyed by (index, sub) — NOT whole
# indices: 0x2600 mixes the fault-flags bitfield (sub 1) with F32 engineering values that
# must read as decimals (the position limits, trip current, max vel/accel, bus voltage).
_HEX_KEYS = {(0x6040, 0), (0x6041, 0), (0x603F, 0), (0x1001, 0), (0x2600, 1)}


def map_word(index: int, sub: int, bits: int) -> int:
    """MC_IF_TLM_MAP_ENTRY: (index<<16) | (sub<<8) | bitlen."""
    return ((index & 0xFFFF) << 16) | ((sub & 0xFF) << 8) | (bits & 0xFF)


@dataclass
class OdEntry:
    index: int
    sub: int
    name: str
    type_code: int
    access: int
    flags: int
    scale: float = 1.0
    unit: str = ""
    synthetic: bool = False  # not literally in the X-macro (e.g. 0x2A00 array subs)
    owner: int = OWNER_MOTOR  # MC_IfOdOwner_t (default preserves pre-v2 behaviour)

    @property
    def key(self) -> tuple[int, int]:
        return (self.index, self.sub)

    @property
    def id_str(self) -> str:
        return f"0x{self.index:04X}:{self.sub}"

    @property
    def label(self) -> str:
        return f"{self.id_str}  {self.name}"

    @property
    def type_name(self) -> str:
        return TYPE_NAME[self.type_code]

    @property
    def access_name(self) -> str:
        return ACCESS_NAME[self.access]

    @property
    def owner_name(self) -> str:
        return OWNER_NAME.get(self.owner, "?")

    @property
    def size(self) -> int:
        return _TYPE_FMT[self.type_code][1]

    @property
    def is_float(self) -> bool:
        return _TYPE_FMT[self.type_code][2]

    @property
    def is_pdo(self) -> bool:
        return bool(self.flags & F_PDO)

    @property
    def is_persist(self) -> bool:
        return bool(self.flags & F_PERSIST)

    @property
    def readable(self) -> bool:
        return bool(self.access & A_RO)

    @property
    def writable(self) -> bool:
        return bool(self.access & A_WO)

    @property
    def scaled(self) -> bool:
        return self.scale != 1.0

    # --- value conversion -----------------------------------------------------
    def decode(self, data: bytes) -> int | float:
        """Decode raw little-endian wire bytes into the native raw value."""
        fmt, size, _ = _TYPE_FMT[self.type_code]
        return struct.unpack(fmt, bytes(data[:size]))[0]

    def encode(self, raw: int | float) -> bytes:
        """Encode a raw value to little-endian wire bytes."""
        fmt, _, is_float = _TYPE_FMT[self.type_code]
        return struct.pack(fmt, raw if is_float else int(round(raw)))

    def raw_to_si(self, raw: int | float) -> float:
        return float(raw) * self.scale

    def si_to_raw(self, si: float) -> int | float:
        if self.is_float:
            return float(si)
        return int(round(si / self.scale))

    def format_value(self, raw: int | float) -> str:
        """Human-readable value string (SI decimal for scaled/float, hex for bitfields)."""
        if self.key in _HEX_KEYS:
            return f"0x{int(raw):X}"
        if self.is_float or self.scaled:
            return f"{self.raw_to_si(raw):.6g}"
        return str(raw)


class OdModel:
    """The parsed object dictionary: lookups by (index,sub) and by name."""

    def __init__(self, entries: list[OdEntry], scales: dict[str, float], source: Path):
        self.entries = entries
        self.source = source
        self.scales = scales
        self.by_key: dict[tuple[int, int], OdEntry] = {e.key: e for e in entries}
        self.by_name: dict[str, OdEntry] = {e.name: e for e in entries}

    def get(self, index: int, sub: int) -> OdEntry | None:
        return self.by_key.get((index, sub))

    def lookup_word(self, word: int) -> OdEntry | None:
        return self.get((word >> 16) & 0xFFFF, (word >> 8) & 0xFF)

    @property
    def pdo_entries(self) -> list[OdEntry]:
        return [e for e in self.entries if e.is_pdo]


def _infer_unit(name: str) -> str:
    if name in _UNIT_BY_NAME:
        return _UNIT_BY_NAME[name]
    low = name.lower()
    for suffix, unit in _UNIT_SUFFIXES:
        if low.endswith(suffix):
            return unit
    return ""


def find_header(start: Path | None = None) -> Path:
    """Locate mc_if_od.h by walking up from this file (it lives in Interface/)."""
    here = (start or Path(__file__).resolve()).resolve()
    for base in [here, *here.parents]:
        candidate = base / "mc_if_od.h"
        if candidate.is_file():
            return candidate
        # also check a sibling Interface dir
        alt = base / "Interface" / "mc_if_od.h"
        if alt.is_file():
            return alt
    raise FileNotFoundError(
        "mc_if_od.h not found. Pass --od-header explicitly to point at the contract header."
    )


_SCALE_RE = re.compile(r"#define\s+MC_IF_(\w+)_SCALE\s+\(?\s*([0-9.eE+-]+)f?\s*\)?")
# Protocol v2 X-macro shape (owner column is the 7th and final argument):
#   X(0x1000, 0, device_type, MC_IF_T_U32, MC_IF_A_RO, MC_IF_F_NONE, MC_IF_OWNER_MOTOR)
# The owner column is optional in the regex for backward compatibility with
# pre-v2 headers (entries without it default to MC_IF_OWNER_MOTOR).
_ENTRY_RE = re.compile(
    r"X\(\s*(0x[0-9A-Fa-f]+)\s*,\s*(\d+)\s*,\s*(\w+)\s*,\s*"
    r"(MC_IF_T_\w+)\s*,\s*(MC_IF_A_\w+)\s*,\s*([A-Za-z0-9_|\s]+?)"
    r"(?:\s*,\s*(MC_IF_OWNER_\w+))?\s*\)"
)


def _parse_flags(text: str) -> int:
    flags = 0
    for token in text.split("|"):
        token = token.strip()
        flags |= _FLAG_BY_NAME.get(token, 0)
    return flags


def parse_od_header(path: Path | None = None) -> OdModel:
    """Parse mc_if_od.h into an OdModel (the firmware contract, verbatim)."""
    header = Path(path) if path else find_header()
    text = header.read_text(encoding="utf-8", errors="replace")

    # scale constants
    scales = dict(_DEFAULT_SCALES)
    for m in _SCALE_RE.finditer(text):
        try:
            scales[m.group(1)] = float(m.group(2))
        except ValueError:
            pass

    entries: list[OdEntry] = []
    seen: set[tuple[int, int]] = set()
    for m in _ENTRY_RE.finditer(text):
        index = int(m.group(1), 16)
        sub = int(m.group(2))
        name = m.group(3)
        type_code = _TYPE_BY_NAME[m.group(4)]
        access = _ACCESS_BY_NAME[m.group(5)]
        flags = _parse_flags(m.group(6))
        owner = _OWNER_BY_NAME.get(m.group(7) or "MC_IF_OWNER_MOTOR", OWNER_MOTOR)

        quantity = _QUANTITY_BY_INDEX.get(index)
        if quantity and type_code != T_F32:
            scale = scales.get(quantity, 1.0)
            unit = _QUANTITY_UNIT.get(quantity, "")
        else:
            scale = 1.0
            unit = _infer_unit(name) if type_code == T_F32 else ""

        entries.append(OdEntry(index, sub, name, type_code, access, flags,
                               scale, unit, owner=owner))
        seen.add((index, sub))

    if not entries:
        raise ValueError(f"No OD entries parsed from {header} - is the X-macro intact?")

    # Synthesise the 0x2A00 telemetry-map array members (sub1..16 are U32 map words),
    # which the X-macro declares as an array rather than listing each sub.
    for sub in range(1, TLM_MAX_ENTRIES + 1):
        if (TLM_MAP_INDEX, sub) not in seen:
            entries.append(OdEntry(
                TLM_MAP_INDEX, sub, f"tlm_map_entry_{sub}",
                T_U32, A_RW, F_NONE, synthetic=True,
            ))

    entries.sort(key=lambda e: (e.index, e.sub))
    return OdModel(entries, scales, header)
