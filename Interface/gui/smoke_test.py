"""Headless smoke test: parser, protocol round-trip, telemetry unpack, GUI build.

Run from the gui/ folder:  python smoke_test.py
Uses Qt's offscreen platform so it needs no display.
"""
import os
import struct
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np

from mc_gui import od as odmod
from mc_gui import protocol as proto


def test_od_parse():
    model = odmod.parse_od_header()
    assert model.get(0x6041, 0).name == "statusword"
    assert model.get(0x6041, 0).type_code == odmod.T_U16
    vk = model.by_name["vel_kp"]
    assert vk.is_float and vk.access == odmod.A_RW and vk.is_persist
    pa = model.by_name["position_actual"]
    assert pa.scaled and abs(pa.scale - 1e-5) < 1e-12 and pa.unit == "rad"
    assert pa.is_pdo
    # synthetic 0x2A00 array members exist
    assert model.get(0x2A00, 1) is not None and model.get(0x2A00, 16) is not None
    # protocol v2: owner column; v3: new entries
    assert model.get(0x6041, 0).owner == odmod.OWNER_MOTOR
    axis = model.get(0x3020, 0)  # CMC-owned axis_op_mode
    assert axis is not None and axis.owner == odmod.OWNER_CMC and axis.owner_name == "CMC"
    assert model.get(0x607B, 0) is not None  # v3 target_position_time_ms
    assert proto.PROTOCOL_VERSION == 5
    # value conversion round-trip on a scaled int
    raw = pa.si_to_raw(1.23456)
    assert abs(pa.raw_to_si(raw) - 1.23456) < 1e-4
    # float entry encode/decode
    assert abs(vk.decode(vk.encode(3.5)) - 3.5) < 1e-6
    print(f"OK od_parse: {len(model.entries)} entries, {len(model.pdo_entries)} PDO, "
          f"source={model.source.name}")
    return model


def test_protocol_roundtrip():
    req = proto.build_read_req(7, 0x6041, 0, odmod.T_U16)
    hdr = proto.parse_header(req)
    assert hdr and hdr.type == proto.MSG_OD_READ_REQ and hdr.seq == 7
    idx, sub, etype = struct.unpack_from("<HBB", req, proto.HEADER_SIZE)
    assert (idx, sub, etype) == (0x6041, 0, odmod.T_U16)

    wr = proto.build_write_req(9, 0x2300, 1, odmod.T_F32, struct.pack("<f", 2.5))
    h2 = proto.parse_header(wr)
    assert h2.type == proto.MSG_OD_WRITE_REQ and h2.seq == 9

    # a read response payload
    resp_payload = struct.pack("<HBBBB", 0x6041, 0, odmod.T_U16, proto.OD_OK, 2) + struct.pack("<H", 0x1234)
    rr = proto.parse_read_resp(resp_payload)
    assert rr.index == 0x6041 and rr.result == proto.OD_OK and rr.data == b"\x34\x12"
    print("OK protocol_roundtrip")


def test_telemetry_unpack(model):
    from mc_gui.client import NetworkClient
    client = NetworkClient(model)
    iq = model.by_name["tlm_iq_meas_a"]
    vel = model.by_name["velocity_actual"]
    client._active_map = [iq, vel]  # iq=float32 (4B), vel=I32 scaled (4B) -> 8B blob

    # build one telemetry record: status header (18B, v4) + blob (8B)
    blob = struct.pack("<f", 1.5) + struct.pack("<i", vel.si_to_raw(2.0))
    sh = struct.pack("<HbBHBBIiH",
                     0x0037,   # statusword
                     3,        # mode_display
                     3,        # node_state RUNNING
                     0,        # error_code
                     5,        # map_version
                     len(blob),
                     42,       # status_counter
                     123456,   # position_actual_scaled (v4)
                     0x0003)   # movement_status (v4): MOVING | ON_TARGET
    record = sh + blob
    tlm_payload = struct.pack("<BBBB", 5, 1, len(record), 0) + record
    datagram = proto.pack_header(proto.MSG_TELEMETRY, 1, tlm_payload)

    hdr = proto.parse_header(datagram)
    dg = proto.parse_telemetry(datagram[proto.HEADER_SIZE:proto.HEADER_SIZE + hdr.length])
    samples = client._unpack(dg)
    assert len(samples) == 1
    s = samples[0]
    assert s.layout_ok and s.node_state == 3 and s.counter == 42
    assert abs(s.values["tlm_iq_meas_a"] - 1.5) < 1e-6
    assert abs(s.values["velocity_actual"] - 2.0) < 1e-3
    assert s.values["movement_status"] == float(0x0003)   # v4 header parsed (MOVING | ON_TARGET)
    assert s.values["statusword"] == float(0x0037)
    print("OK telemetry_unpack")


def test_buffer():
    from mc_gui.buffer import TelemetryBuffer
    buf = TelemetryBuffer(capacity=1000, rate_hz=1000.0)
    # contiguous ring caps at capacity, keeps the most recent samples, stays ordered
    for i in range(2500):
        buf._ring("x").append(float(i) * 0.001, float(i))
    t, v = buf.get("x")
    assert len(t) == 1000 and v[0] == 1500.0 and v[-1] == 2499.0
    assert np.all(np.diff(t) > 0)  # monotonic -> windowing is valid
    # windowed query returns only the visible slice (+1 neighbour each side)
    wt, wv = buf.window("x", 2.0, 2.01)  # ~ samples 2000..2010
    assert wv[0] <= 2000.0 and wv[-1] >= 2010.0 and len(wv) < 30
    print(f"OK buffer (ring cap + windowed slice: {len(wv)} pts in window)")


def test_gui_build(model):
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication(sys.argv)
    from mc_gui.main_window import MainWindow
    from mc_gui.graph_window import GraphWindow
    win = MainWindow(model)
    assert win.tree.topLevelItemCount() > 0
    win._load_default_map()
    assert len(win._selected_map_entries()) > 0
    # debug console captures messages (same-thread signal is synchronous)
    win._log("hello console", "INFO")
    assert "hello console" in win.log_console.view.toPlainText()
    gw = GraphWindow(win.buffer, ["statusword", "tlm_iq_meas_a"], initial=["tlm_iq_meas_a"])
    assert "tlm_iq_meas_a" in gw.curves
    gw.close()

    # Watch drives graphing: tick a non-streamed entry, it should graph from polling
    from PySide6.QtCore import Qt
    from mc_gui.main_window import _WATCH_COL
    win.buffer.set_origin()
    item = win.item_by_key[win.od.by_name["vel_kp"].key]
    item.setCheckState(_WATCH_COL, Qt.CheckState.Checked)
    assert "vel_kp" in win._watched_names()
    win.buffer.add_poll("vel_kp", 3.5)  # simulate a polled value reaching the buffer
    t, v = win.buffer.get("vel_kp")
    assert len(v) == 1 and abs(v[0] - 3.5) < 1e-9
    gw2 = win._new_graph()  # no explicit channels -> should use the watched set
    assert "vel_kp" in gw2.curves
    for i in range(50):
        win.buffer.add_poll("vel_kp", float(i))
    gw2._redraw()                     # autoscroll path
    gw2.chk_autoscroll.setChecked(False)
    gw2._dirty = True
    gw2._redraw()                     # manual-view path (reads viewRange)
    gw2.close()

    # Motor Config tab: config rows registered for the gain/model keys,
    # and the cal/store status entries routed to their labels.
    assert win._mcfg_rows, "Motor Config rows not built"
    for key in [(0x2000, 1), (0x2300, 1), (0x2400, 5), (0x2500, 6), (0x2700, 3),
                (0x2200, 4)]:  # velocity_ff_gain (ADR-031)
        assert key in win._mcfg_rows, f"Motor Config missing row {key}"
    assert (0x2700, 2) in win._mcfg_status and (0x2800, 2) in win._mcfg_status
    # status decode (no connection needed): cal_status 1 -> aligning, 0xFFFF -> fault
    win._mcfg_update_status(model.get(0x2700, 2), 1)
    assert "align" in win.lbl_cal_status.text().lower()
    win._mcfg_update_status(model.get(0x2700, 2), 0xFFFF)
    assert "FAULT" in win.lbl_cal_status.text()
    win._mcfg_update_status(model.get(0x2800, 2), 0x0001)   # MC_IF_STORE_VALID, idle
    assert "saved" in win.lbl_store_status.text().lower()
    win._mcfg_update_status(model.get(0x2800, 2), 0x0003)   # VALID | PENDING -> "disable to commit" prompt
    assert "pending" in win.lbl_store_status.text().lower()
    # calibration completeness (0x2700:5 cal_done_flags) parses + decodes to outstanding list
    cdf = model.get(0x2700, 5)
    assert cdf is not None and cdf.name == "cal_done_flags" and cdf.type_code == odmod.T_U16
    assert (0x2700, 5) in win._mcfg_status
    win._mcfg_update_status(cdf, 0x0001)  # only electrical alignment done
    done_html = win.lbl_cal_done.text()
    assert "Electrical alignment" in done_html and "outstanding" in done_html.lower()
    assert "Mechanical zero" in done_html and "Current offset" in done_html
    win._mcfg_update_status(cdf, 0x0007)  # all three done
    assert "all complete" in win.lbl_cal_done.text().lower()
    # a motor-config row read-back routes through the shared _cfg_row plumbing
    vk_entry = model.by_name["vel_kp"]
    win._setup_on_read_done(vk_entry, True, vk_entry.format_value(2.5), "")
    assert win._mcfg_rows[(0x2300, 1)]["current"].text() == vk_entry.format_value(2.5)

    # Motor Command tab: mode combo carries the axis modes including Torque
    # (enabled now that REQ-0012 / CHANGELOG [4.3.0] has landed on the CMC).
    modes = [win.cmd_mode_combo.itemText(i) for i in range(win.cmd_mode_combo.count())]
    assert any("Profile Velocity" in m for m in modes) and any("Profile Position" in m for m in modes)
    torque_idx = next(i for i in range(win.cmd_mode_combo.count())
                      if win.cmd_mode_combo.itemData(i) == 5)  # MC_AXIS_MODE_TORQUE
    assert win.cmd_mode_combo.model().item(torque_idx).isEnabled()  # REQ-0012 done
    # Mode-based entry-point greying: selecting a mode enables only its group.
    win.cmd_mode_combo.setCurrentIndex(torque_idx)
    assert win.cmd_curg.isEnabled() and not win.cmd_posg.isEnabled() and not win.cmd_velg.isEnabled()
    pos_idx = next(i for i in range(win.cmd_mode_combo.count())
                   if win.cmd_mode_combo.itemData(i) == 3)  # MC_AXIS_MODE_PROFILE_POSITION
    win.cmd_mode_combo.setCurrentIndex(pos_idx)
    assert win.cmd_posg.isEnabled() and not win.cmd_curg.isEnabled()
    assert not win.cmd_cur_edit.isEnabled()   # current field greyed in Position mode
    # "Read current mode" readout (axis_op_mode_actual 0x3001 -> label).
    assert hasattr(win, "cmd_mode_actual_lbl")
    oma = model.get(0x3001, 0)
    win._cmd_on_state_read({"entry": oma, "ok": True, "raw": 3, "si": 3})
    assert "Position" in win.cmd_mode_actual_lbl.text()
    # OD must carry the new CMC-owned entry 0x302B axis_target_current (REQ-0012).
    assert "axis_target_current" in model.by_name
    assert model.by_name["axis_target_current"].key == (0x302B, 0)
    # state read-backs route to the command labels (statusword decode + motor actuals)
    pos = model.get(0x6064, 0)
    win._cmd_on_state_read({"entry": pos, "ok": True, "raw": pos.si_to_raw(1.5), "si": 1.5})
    assert "1.5" in win.cmd_fb_pos.text()
    sw = model.get(0x6041, 0)
    win._cmd_on_state_read({"entry": sw, "ok": True, "raw": 0x0427, "si": 0x0427})  # ENABLED+READY+TARGET_REACHED
    assert "TARGET_REACHED" in win.cmd_sw_lbl.text() and win.cmd_fb_reached.text() == "yes"
    ast = model.get(0x3000, 0)
    win._cmd_on_state_read({"entry": ast, "ok": True, "raw": 2, "si": 2})  # RUNNING
    assert win.cmd_state_lbl.text() == "RUNNING"

    # Tuning section (motor-driven 0x2910 generator): widgets, mode list, OD entries, unit relabel.
    for attr in ("cmd_tune_mode", "cmd_tune_amp", "cmd_tune_rate", "cmd_tune_dwell",
                 "cmd_tune_cont", "cmd_tune_active"):
        assert hasattr(win, attr), f"tuning widget {attr} missing"
    tmodes = [win.cmd_tune_mode.itemData(i) for i in range(win.cmd_tune_mode.count())]
    assert tmodes == [0, 1, 2, 3]   # off / velocity / position / current
    # Backend selector (0x2000:6) widget + OD entry.
    assert hasattr(win, "mcfg_backend") and win.mcfg_backend.count() == 2
    assert hasattr(win, "mcfg_backend_lbl")   # reads back motor_backend_sel (0x2000:6) on connect
    assert model.get(0x2000, 6) is not None and model.get(0x2000, 6).name == "motor_backend_sel"
    # Brushed current loop: kp/ki (0x2400:6/7) are directly editable rows (ADR-049, RW PERSIST);
    # the bandwidth (0x2400:8) was removed; measured current is telemetry (0x2410:6).
    assert (0x2400, 6) in win._mcfg_rows and model.get(0x2400, 6).name == "hb_cur_kp"
    assert (0x2400, 7) in win._mcfg_rows and model.get(0x2400, 7).name == "hb_cur_ki"
    assert (0x2400, 8) not in win._mcfg_rows   # bandwidth removed (no longer an editable row)
    assert model.get(0x2410, 6) is not None and model.get(0x2410, 6).name == "tlm_i_arm_a"
    assert hasattr(win, "mcfg_cur_lbl") and hasattr(win, "mcfg_kp_lbl") and hasattr(win, "mcfg_ki_lbl")
    # Motor safety envelope (ADR-040): motor-owned, enforced velocity/accel ceilings as config rows.
    assert (0x2600, 4) in win._mcfg_rows and (0x2600, 5) in win._mcfg_rows
    assert model.get(0x2600, 4).name == "max_velocity_rad_s" and model.get(0x2600, 5).name == "max_accel_rad_s2"
    # Soft position limits (ADR-040): manually-set, motor-owned config rows.
    assert (0x2600, 6) in win._mcfg_rows and (0x2600, 7) in win._mcfg_rows
    assert model.get(0x2600, 6).name == "pos_limit_lo_rad" and model.get(0x2600, 7).name == "pos_limit_hi_rad"
    for key in [(0x2910, 1), (0x2910, 2), (0x2910, 3), (0x2910, 6), (0x2910, 7)]:
        assert model.get(*key) is not None, f"OD missing 0x{key[0]:04X}:{key[1]}"
    assert model.get(0x2910, 1).name == "test_mode" and model.get(0x2910, 7).name == "test_active"
    ts = model.get(0x2910, 8)                       # generator output -> graphable PDO channel
    assert ts is not None and ts.name == "test_signal" and ts.is_pdo
    assert ts in model.pdo_entries                  # appears in the telemetry-map editor's PDO list
    pd = model.get(0x2510, 3)                       # absolute position demand -> graphable PDO (vs 0x6064)
    assert pd is not None and pd.name == "tlm_pos_demand_rad" and pd.is_pdo
    # selecting position tuning relabels the units (offline: not connected -> no OD write)
    pos_idx = next(i for i in range(win.cmd_tune_mode.count()) if win.cmd_tune_mode.itemData(i) == 2)
    win.cmd_tune_mode.setCurrentIndex(pos_idx)
    assert win.cmd_tune_amp_unit.text() == "rad" and win.cmd_tune_rate_unit.text() == "rad/s"
    # test_active readback routes to the label
    win._cmd_on_state_read({"entry": model.get(0x2910, 7), "ok": True, "raw": 1, "si": 1})
    assert win.cmd_tune_active.text() == "RUNNING"
    win._cmd_on_state_read({"entry": model.get(0x2910, 7), "ok": True, "raw": 0, "si": 0})
    assert win.cmd_tune_active.text() == "idle"

    win.close()
    print("OK gui_build (offscreen)")


if __name__ == "__main__":
    m = test_od_parse()
    test_protocol_roundtrip()
    test_telemetry_unpack(m)
    test_buffer()
    test_gui_build(m)
    print("\nALL SMOKE TESTS PASSED")
