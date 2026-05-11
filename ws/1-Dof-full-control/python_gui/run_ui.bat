@echo off
echo ===================================================
echo 1-DOF Robotic Arm - Python UI Setup ^& Run
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
echo Starting Application...
echo ===================================================
python main.py

pause