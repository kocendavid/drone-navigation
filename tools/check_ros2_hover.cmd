@echo off
call C:\px4-tools\ros_ws\install\setup.bat
if errorlevel 1 exit /b %errorlevel%
python "%~dp0..\ros2_hover.py" --check
if errorlevel 1 exit /b %errorlevel%
python "%~dp0..\tests\test_hover.py" -v
exit /b %errorlevel%
