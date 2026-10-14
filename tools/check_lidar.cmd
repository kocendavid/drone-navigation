@echo off
setlocal
set "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
set "ROS_DOMAIN_ID=0"
set "FASTRTPS_DEFAULT_PROFILES_FILE=C:\px4-tools\fastdds.xml"
set "FASTDDS_DEFAULT_PROFILES_FILE=C:\px4-tools\fastdds.xml"
python -m unittest discover -s "%~dp0..\tests" -p "test*lidar*.py" -v
if errorlevel 1 exit /b %errorlevel%
python "%~dp0..\tests\test_clearance.py" -v
exit /b %errorlevel%
