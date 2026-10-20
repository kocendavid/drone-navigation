@echo off
setlocal
call C:\px4-tools\ros_ws\install\setup.bat
if errorlevel 1 exit /b %errorlevel%
cmake -S "%~dp0..\native_control" -B "%~dp0..\.tools\native_control" -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DPython3_FIND_VIRTUALENV=ONLY
if errorlevel 1 exit /b %errorlevel%
cmake --build "%~dp0..\.tools\native_control" --config Release
if errorlevel 1 exit /b %errorlevel%
call "%~dp0build_native_navigation.cmd"
exit /b %errorlevel%
