#!/bin/bash
ros2 bag record \
  --compression-mode file \
  -o calib-$1 \
  /cmd_vel \
  /icp/odom \
  /hardware/imu \
  /hardware/sent_wheel_speeds \
  /hardware/feedback_wheel_speeds

mv calib-$1/calib-$1_0.mcap calib-$1.mcap
rm -rf calib-$1
