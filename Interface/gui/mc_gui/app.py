"""Application entry point: parse the OD contract, launch the GUI."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication, QMessageBox

from .od import parse_od_header


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="CMC Object Dictionary GUI")
    parser.add_argument("--od-header", type=Path, default=None,
                        help="path to mc_if_od.h (auto-discovered if omitted)")
    args = parser.parse_args(argv)

    app = QApplication(sys.argv)
    try:
        od = parse_od_header(args.od_header)
    except (FileNotFoundError, ValueError) as exc:
        QMessageBox.critical(None, "OD load error", str(exc))
        return 2

    # import here so a missing pyqtgraph fails after QApplication exists (nicer error)
    from .main_window import MainWindow

    win = MainWindow(od)
    win.setWindowTitle(f"CMC OD Tool - {od.source.name} ({len(od.entries)} entries)")
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
