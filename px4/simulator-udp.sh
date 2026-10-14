#!/bin/sh
# This replaces only the simulator startup include, not PX4 firmware.
param set IMU_INTEG_RATE 250
param set EKF2_MULTI_IMU 0
param set EKF2_MULTI_MAG 0
param set SENS_IMU_MODE 1
param set SENS_GNSS0_DELAY 0
param set SENS_GNSS1_DELAY 0
param set COM_MODE_ARM_CHK 1
simulator_mavlink start -u 14560
