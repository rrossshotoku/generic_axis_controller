@echo off
setlocal
rem One-click launcher for the CMC Object Dictionary GUI.
rem Double-click this file. First run installs the Python dependencies.

cd /d "%~dp0"

where python >nul 2>nul
if errorlevel 1 (
    echo Python was not found on PATH. Install Python 3.10+ and tick "Add to PATH".
    echo.
    pause
    exit /b 1
)

rem Ensure dependencies are present (only does work on first run / after updates).
python -c "import PySide6, pyqtgraph, numpy" >nul 2>nul
if errorlevel 1 (
    echo Installing dependencies, please wait...
    python -m pip install -r requirements.txt
    if errorlevel 1 (
        echo Dependency install failed.
        echo.
        pause
        exit /b 1
    )
)

echo Starting CMC OD Tool... ^(close this window to quit the app^)
python run.py
if errorlevel 1 (
    echo.
    echo The app exited with an error - see the message above.
    pause
)
endlocal
