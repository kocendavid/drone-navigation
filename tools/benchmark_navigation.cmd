@echo off
setlocal
rem Offline kernels and ephemeral HTTP servers only; no ROS node is created.
call "%~dp0build_native_navigation.cmd"
if errorlevel 1 exit /b %errorlevel%
python "%~dp0..\benchmarks\navigation_benchmark.py" %*
exit /b %errorlevel%
