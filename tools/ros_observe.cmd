@echo off
setlocal
rem Explicit middleware per invocation; never use or stop the shared ROS daemon.
if /i "%~1"=="dds" (
    set "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    set "FASTRTPS_DEFAULT_PROFILES_FILE=C:\px4-tools\fastdds.xml"
    set "FASTDDS_DEFAULT_PROFILES_FILE=C:\px4-tools\fastdds.xml"
) else if /i "%~1"=="zenoh" (
    set "RMW_IMPLEMENTATION=rmw_zenoh_cpp"
) else (
    echo Usage: ros_observe.cmd dds^|zenoh [lidar_probe arguments]
    exit /b 1
)
set "ROS_DOMAIN_ID=0"
set "ROS_LOCALHOST_ONLY="
set "ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET"
set "ROS_STATIC_PEERS="
set "ROS_DISCOVERY_SERVER="
set "FASTDDS_ENVIRONMENT_FILE="
set "SKIP_DEFAULT_XML="
set "RMW_FASTRTPS_USE_QOS_FROM_XML="
if exist "C:\px4-tools\ros_ws\install\setup.bat" call "C:\px4-tools\ros_ws\install\setup.bat"
python "%~dp0lidar_probe.py" %*
exit /b %errorlevel%
