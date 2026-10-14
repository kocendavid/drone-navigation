@echo off
setlocal
rem Measured from logs/lidar_scene_live.json, 2026-10-12 16:09 Prague time.
rem Mount: sensor XYZ=(0,0,0.0591487379025617) in chassis, aligned axes, unit scale.
rem Re-measure and update this command if the sensor mount or frame names change.
rem One publisher only; stop with Ctrl+C. This sends TF, never flight commands.
set "RMW_IMPLEMENTATION=rmw_zenoh_cpp"
set "ROS_DOMAIN_ID=0"
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.0591487379025617 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id chassis --child-frame-id point_cloud
exit /b %errorlevel%
