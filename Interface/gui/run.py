#!/usr/bin/env python3
"""Convenience launcher so you can run `python run.py` from this folder."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mc_gui.app import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
