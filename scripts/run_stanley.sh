#!/usr/bin/env bash
# Stanley waypoint following on the vehicle. The map runs in its own terminal:
#
#   terminal 1:  ros2 launch mola_slam_mapping localization.launch.py \
#                  map_dir:=~/krowd_maps/<run> map_filename:=global_map_corrected.mm
#   terminal 2:  run_stanley.sh check [launch args]   # route + vehicle state, no actuator output
#                run_stanley.sh drive [launch args]   # publishes /target_angle /target_speed /brake !
#
# speed_limit_kmph defaults to 0 = no ceiling (path_tracking.yaml's 20/10 km/h);
# lower it for a cautious run:  run_stanley.sh drive speed_limit_kmph:=5.0
# Record every drive:  scripts/record_stanley_tuning.sh  (in a third terminal)

set -euo pipefail
MODE="${1:-check}"; shift || true
WS="${KROWD_WS:-$HOME/krowd2026_ws}"
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u

pubs() { ros2 topic info "$1" 2>/dev/null | awk '/^Publisher count:/{print $3}'; }

case "$MODE" in
  check|drive) ;;
  *) echo "사용법: $0 check|drive [launch args]"; exit 1 ;;
esac

# The route comes from the map terminal 1 announced.
if ! ros2 node list 2>/dev/null | grep -q '^/selected_map_node$'; then
  echo "중단: 맵이 떠 있지 않습니다. 먼저 다른 터미널에서:"
  echo "  ros2 launch mola_slam_mapping localization.launch.py \\"
  echo "    map_dir:=~/krowd_maps/<run> map_filename:=global_map_corrected.mm"
  exit 1
fi
echo "맵: $(timeout 5 ros2 topic echo --once /localization/map_dir 2>/dev/null | sed -n 's/^data: //p')"
[[ "$(pubs /ekf/odometry)" -ge 1 ]] 2>/dev/null || echo "경고: /ekf/odometry publisher 없음 (all sensor start 확인)"

DRIVE=false
if [[ "$MODE" == drive ]]; then
  DRIVE=true
  echo "=== 실차 구동 모드: /target_angle, /target_speed, /brake 로 명령이 나갑니다 ==="
  for t in /target_angle /target_speed /brake; do
    n="$(pubs "$t")"
    if [[ "${n:-0}" -gt 0 ]]; then
      echo "중단: $t 에 이미 publisher ${n}개. 다른 제어기(MPC, stop_mission, final_control 등)를 먼저 끄세요."
      exit 1
    fi
  done
  if ! ros2 node list 2>/dev/null | grep -q ros_to_can_node; then
    echo "중단: ros_to_can_node 가 없습니다. (all sensor start 는 CAN bridge 를 띄우지 않음)"
    echo "      ros2 launch ros_can_bridge bridge.launch.py can_interface:=can0"
    exit 1
  fi
  # stanley_node brakes without these (STALE_VEHICLE_STATE / NO_STEERING_FEEDBACK).
  if ! [[ "$(pubs /ekf/odometry)" -ge 1 ]] 2>/dev/null; then
    echo "중단: /ekf/odometry publisher 가 없습니다. (all sensor start 확인)"
    exit 1
  fi
  if ! [[ "$(pubs /encoder)" -ge 1 ]] 2>/dev/null; then
    echo "중단: /encoder publisher 가 없습니다. 조향 피드백 없이는 주행하지 않습니다 (CAN bridge 확인)."
    exit 1
  fi
  enc_type="$(ros2 topic type /encoder 2>/dev/null | head -1)"
  if [[ "$enc_type" != "ros_can_bridge_msgs/msg/EncoderAngleStamped" ]]; then
    echo "중단: /encoder 타입이 '$enc_type' 입니다 (ros_can_bridge_msgs/msg/EncoderAngleStamped 이어야 함)."
    exit 1
  fi
  hv="$(timeout 6 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /ekf/heading_verified 2>/dev/null | sed -n 's/^data: //p')"
  if [[ "$hv" != "true" ]]; then
    echo "경고: /ekf/heading_verified='${hv:-없음}'. stanley_node 는 true 가 될 때까지 STOP HEADING_UNVERIFIED 로 서 있습니다."
    echo "      정지 상태로 LiDAR 정합을 기다리거나, 1 m/s 이상으로 10 m 직진한 뒤 다시 확인하세요."
  fi
  echo "E-stop 담당자 대기, 주변 확인, check 모드에서 위치/방향 확인을 마쳤습니까?"
  read -r -p "계속하려면 DRIVE 를 입력: " ans
  [[ "$ans" == "DRIVE" ]] || { echo "취소"; exit 1; }
fi

exec ros2 launch krowd_stanley_control stanley_waypoint_follow.launch.py \
  drive_enabled:="$DRIVE" "$@"
