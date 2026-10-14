@echo off
setlocal
title ROS - Isaac Forward Step
call C:\px4-tools\ros_ws\install\setup.bat
if errorlevel 1 exit /b %errorlevel%
set "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
set "ROS_DOMAIN_ID=0"
set "ROS_LOCALHOST_ONLY="
set "ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET"
set "ROS_STATIC_PEERS="
set "ROS_DISCOVERY_SERVER="
set "FASTDDS_ENVIRONMENT_FILE="
set "SKIP_DEFAULT_XML="
set "RMW_FASTRTPS_USE_QOS_FROM_XML="
set "FASTRTPS_DEFAULT_PROFILES_FILE=C:\px4-tools\fastdds.xml"
set "FASTDDS_DEFAULT_PROFILES_FILE=C:\px4-tools\fastdds.xml"
python "%~dp0..\ros2_step.py" %*
exit /b %errorlevel%
