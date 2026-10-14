@echo off
setlocal
set "RMW_IMPLEMENTATION=rmw_zenoh_cpp"
set "ROS_DOMAIN_ID=0"
python "%~dp0forward_clearance.py" --output "%~dp0..\logs\forward_clearance.json" %*
exit /b %errorlevel%
