#!/usr/bin/env bash
# Record what is needed to tune Stanley's steering from a real drive.
#
#   record_stanley_tuning.sh [name]     ->  ~/rosbags/stanley_tuning_<time>[_name]
#
# Start it BEFORE run_stanley.sh drive and stop it with Ctrl-C (a killed
# recorder leaves a bag without metadata). Small: no camera, no LiDAR.
#
# What each topic answers:
#   /target_angle vs /encoder                 steering-motor delay and speed, and the command ratio
#   desired_steering vs measured_steering     how far the command runs ahead of the wheels
#   cross_track_error, heading_error          the weaving itself (amplitude, period)
#   yaw_rate_error                            whether the yaw damping has the right sign and size
#   /target_speed vs /ekf/odometry            the drive MCU's speed tracking (speed trim)
#   /ekf/innovation, /ublox_gps_node/*,       when and why the ESKF leaves the GPS (2026-09-24:
#   /diagnostics, /lidar_odometry/*           49 m off, every GPS position rejected, RTK Float)

set -euo pipefail
WS="${KROWD_WS:-$HOME/krowd2026_ws}"
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u

OUT="$HOME/rosbags/stanley_tuning_$(date +%m%d_%H%M%S)${1:+_$1}"
mkdir -p "$HOME/rosbags"
echo "기록: $OUT   (Ctrl-C 로 종료)"
exec ros2 bag record -o "$OUT" \
  /stanley_node/status \
  /stanley_node/cross_track_error /stanley_node/heading_error \
  /stanley_node/yaw_rate_error \
  /stanley_node/desired_steering /stanley_node/measured_steering \
  /target_angle /target_speed /brake \
  /encoder /wheel_sensor \
  /ekf/odometry /localization/vehicle_state /planning/trajectory \
  /ekf/global_odometry /ekf/innovation /ekf/heading_verified /ekf/heading_source \
  /ublox_gps_node/fix /ublox_gps_node/fix_velocity /ublox_gps_node/navpvt \
  /imu/data_ekf /lidar_odometry/pose /lidar_odometry/pose_quality \
  /mola/map_localization/odometry /mola/map_localization/heading_verified \
  /traffic_light/detections_json /planning/signal_zones /glane3d/stopline_markers \
  /planning/avoidance_zones /stanley_node/handoff /stanley_node/resume \
  /cone_avoidance_1005/mission_status /avoidance_supervisor/status \
  /localization/map_dir /diagnostics \
  /tf /tf_static
