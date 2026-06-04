@echo off
echo ===================================================
echo 1-DOF Robotic Arm - Web Simulator Launcher
echo ===================================================

cd /d "%~dp0"

IF NOT EXIST "venv" (
    echo [1/3] Creating virtual environment...
    python -m venv venv
)

echo [2/3] Activating virtual environment...
call venv\Scripts\activate

echo [3/3] Installing/Checking requirements...
pip install -r requirements.txt

echo.
echo ===================================================
echo Starting Simulator Server...
echo GUI will be available at http://localhost:8080
echo WebSocket at ws://localhost:8765
echo ===================================================
echo Press Ctrl+C to stop.
echo.

start http://localhost:8080
python server.py

pause
