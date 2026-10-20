@echo off
setlocal
cmake -S "%~dp0..\native_navigation" -B "%~dp0..\.tools\native_navigation" -G Ninja -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 exit /b %errorlevel%
cmake --build "%~dp0..\.tools\native_navigation" --config Release
exit /b %errorlevel%
