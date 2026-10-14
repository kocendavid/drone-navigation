@echo off
setlocal
call C:\px4-tools\ros_ws\install\setup.bat
if errorlevel 1 exit /b %errorlevel%
set "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
rem Isolated test domain; no simulator or real flight-control traffic.
set "ROS_DOMAIN_ID=97"
set "ROS_LOCALHOST_ONLY="
set "ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET"
set "ROS_STATIC_PEERS="
set "ROS_DISCOVERY_SERVER="
set "FASTDDS_ENVIRONMENT_FILE="
set "SKIP_DEFAULT_XML="
set "RMW_FASTRTPS_USE_QOS_FROM_XML="
set "FASTRTPS_DEFAULT_PROFILES_FILE=C:\px4-tools\fastdds.xml"
set "FASTDDS_DEFAULT_PROFILES_FILE=C:\px4-tools\fastdds.xml"
python -m unittest discover -s "%~dp0..\tests" -p "test_*.py" -v
exit /b %errorlevel%
