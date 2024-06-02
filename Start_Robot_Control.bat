@echo off
echo  _________________________________________


echo  Project RACS - Robotic Arm Control System

echo  _________________________________________

echo  Author: Denis Manolescu, 2024
echo  Supervisors - Prof Dr. Emanuele Secco and Prof Dr. Anuradha Ranasinghe 
echo  Liverpool Hope University, FML401 Robotics Lab
echo  2024.
echo  _______________________________________________________________________

echo  Connecting to Raspberry Pi Zero 2W ...
echo  Connecting to Raspberry Pi Zero 2W ...
echo  ">>> Password: 12345678
echo  ...

python "%~dp0activate_glove.py"
if errorlevel 1 (
    echo Failed to execute activate_glove.py.
    pause
    exit /b 1
)

pause

cmd /k
