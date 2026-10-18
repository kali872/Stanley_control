# krowd_stanley_control

맵 주행(`mola_slam_mapping`)이 저장한 `waypoints.csv`를 Stanley 제어기로 따라가는 패키지.
`krowd_slam/krowd_path_tracking`에서 갈라져 나왔고, 2026-09-20부터 Stanley 전용이다
(Pure Pursuit은 `krowd_path_tracking`에 그대로 있다).

```
<map_dir>/waypoints.csv, corridor.csv ─ route_publisher_node ─ /planning/trajectory ─┐
/ekf/odometry (fallback /localization/reference_pose) ─ vehicle_state_node           ├─ stanley_node ─ /target_angle, /target_speed, /brake
                                                       ─ /localization/vehicle_state ┘                  (ros_can_bridge)
```

## 실행

맵은 항상 별도 터미널에서 띄운다. 이 패키지는 그 맵이 `/localization/map_dir`로 알린
run의 경로를 자동으로 따라간다 (맵 이름을 두 번 적지 않는다).

```bash
# 터미널 1 - 맵 (어떤 run이든)
ros2 launch mola_slam_mapping localization.launch.py \
  map_dir:=~/krowd_maps/20260913_103732 map_filename:=global_map_corrected.mm
```

### 실차 주행 — `stanley_waypoint_follow.launch.py`

센서(`all sensor start`)와 CAN bridge(`ros2 launch ros_can_bridge bridge.launch.py can_interface:=can0`)가 떠 있어야 한다.

```bash
# 터미널 2 - 확인 모드: 경로와 차량 상태만, 액추에이터 명령 없음
ros2 launch krowd_stanley_control stanley_waypoint_follow.launch.py
# 터미널 2 - 실차 구동: /target_angle, /target_speed, /brake 발행
ros2 launch krowd_stanley_control stanley_waypoint_follow.launch.py drive_enabled:=true
#   속도 상한 기본 0 = 없음(yaml의 20/10 km/h). 첫 시험은 낮춰서:  ... speed_limit_kmph:=5.0
```

`/encoder`(조향 피드백)가 없으면 주행하지 않는다 — CAN bridge가 반드시 떠 있어야 한다.
매 주행은 세 번째 터미널에서 `scripts/record_stanley_tuning.sh`로 기록한다 (Ctrl-C로 종료).

`scripts/run_stanley.sh check|drive`는 같은 launch를 띄우되, 맵이 떠 있는지 / 액추에이터
토픽에 다른 publisher가 없는지 / CAN bridge가 있는지 검사하고 `DRIVE` 입력을 요구한다.

상태: `ros2 topic echo /stanley_node/status`

| 상태 | 뜻 |
|---|---|
| `TRACKING wp= cte= heading_error= yaw= yaw_damp= steer= steer_fb= cmd= v=실제->목표 cmd_v=` | 정상 추종. `yaw_damp` = yaw-rate 감쇠 보정, `steer_fb` = `/encoder`로 본 실제 road-wheel 각, `cmd_v` = 속도 보정 후 보낸 값 |
| `STOP NO_STEERING_FEEDBACK` / `STALE_STEERING_FEEDBACK age=` | `/encoder`를 못 받음 / 0.25 s 넘게 끊김 — CAN bridge 확인 |
| `WAITING NO_VEHICLE_STATE` | 위치가 아직 안 들어옴 |
| `STOP OUTSIDE_CORRIDOR` | 경로에서 corridor(±3 m) 밖 |
| `STOP HEADING_NOT_ALIGNED` | 경로 방향과 60° 넘게 차이 |
| `STOP STALE_VEHICLE_STATE` / `TRAJECTORY_EXPIRED` / `FRAME_MISMATCH` | 위치·경로 입력 이상 |
| `STOP GOAL_REACHED` | 경로 끝 도착 (`STOP_AT_END`) |

### 폐루프 시뮬레이션 — `stanley_dynamics_sim.launch.py`

센서 스택은 끄고 실행한다 (ESKF도 `base_link` TF를 내기 때문).

```bash
# 터미널 2
ros2 launch krowd_stanley_control stanley_dynamics_sim.launch.py
#   옵션: speed_limit_kmph:=10.0 initial_waypoint_index:=100
#         initial_lateral_offset_m:=1.5 initial_heading_offset_deg:=20
#         heading_prior_error_deg:=20 noise_seed:=3
#         ideal:=true   (아래 불완전성을 모두 끈 이상 모델)
```

Stanley 명령이 자전거 모델을 움직이고 그 위치가 다시 Stanley로 들어간다. 모든 토픽은
`/simulation/*`이고 CAN 토픽은 쓰지 않는다. 차량 박스는 RViz의 기존 "Vehicle"
표시(`/vehicle_marker`)에 나온다. 박스와 TF는 **실제 위치**, Stanley가 받는
`/simulation/odometry`는 **ESKF 추정치**라서 둘의 차이가 RViz에 보인다.

기본값은 실차와 같은 조건이다 (2026-09-23, `stanley_tuning_0922_235531` 실측,
`krowd_stanley_control/sim_sensors.py`):

| 항목 | 기본값 | 근거 |
|---|---|---|
| 조향 지연 / 속도 부족 | 0.24 s / 1.7 km/h | bag의 명령 대비 `/encoder`, 속도 |
| 위치 백색 잡음 | 0.6 cm | `/ekf/odometry` 고주파 성분 |
| 위치 느린 drift | σ 7 cm, 10 s | 실주행 ESKF-RTK 위치 차이 5.4~7.9 cm (2026-09-27, 이전 18 cm는 공분산 상한) |
| yaw / yaw-rate 잡음 | 0.14° / 0.58°/s | `/ekf/odometry` 고주파 성분 |
| `/simulation/encoder` | 50 Hz, 0.01 rad 단위, 0.026 rad 떨림 | bag의 `/encoder` |
| heading 검증 | RTK 코스: 1 m/s 이상, \|yaw rate\| ≤ 0.10 rad/s 표본 3개(5 Hz, 간격 ≤ 1 s) | `gps_imu_eskf.yaml` |
| 검증 전 heading | 저장 prior가 5° 틀림, 위치는 안테나 lever arm 2.5038 m만큼 같이 틀림 | 검증 순간 정답으로 점프 |

그래서 Stanley는 실차 설정 그대로 돈다: 조향 피드백 watchdog, lead 제한, heading 게이트,
미검증 creep(1 s 대기 후 7 km/h 직진)이 모두 켜져 있다 (`config/sim_overrides.yaml`은
피드백 토픽과 LiDAR 일관성 검사만 바꾼다). `ideal:=true`는 `config/sim_ideal.yaml`과 함께
전부 끈다. 위치 튐(jump)은 bag에 한 번도 없어서 모델에 넣지 않았다.

타이어 미끄러짐은 따로 모델링하지 않았다. 조향 실측표 자체가 같은 bag의 yaw-rate/속도로
만든 "실제로 돈 만큼의 각"이라 저속 슬립은 이미 들어가 있다. 속도에 따른 언더스티어는
속도를 바꿔 가며 같은 코너를 돈 bag이 있어야 잴 수 있다.

## 조향 모터 지연 보정 (2026-09-20)

첫 주행들에서 좌우로 크게 흔들렸다. 기본 Stanley 식은 조향 모터가 명령을 따라오는 지연을
모르기 때문이다. 세 가지를 더했다 (`stanley.py`, `stanley_node.py`):

1. **yaw-rate 감쇠** — `gain_s * (속도*경로곡률 - 실제 yaw-rate)`를 조향에 더한다
   (`yaw_rate_damping_gain_s: 0.15`). 직선에서 왼쪽으로 10°/s 돌고 있으면 오른쪽으로 1.5°.
   `/ekf/odometry`의 `twist.angular.z`(rad/s, **좌회전 +**)를 쓴다. 이 값이 0에 고정이거나
   부호가 반대면 0.0으로 끄고 원인을 먼저 찾는다.
2. **조향 lead 제한** — 조향 목표는 `/encoder`로 본 실제 road-wheel 각에서 직선 ±5°,
   커브(R ≤ 15 m) ±12° 이상 앞서지 못한다. 그 사이는 경로 곡률로 섞는다
   (`max_steering_command_lead_deg` / `max_steering_command_lead_curve_deg`,
   `steering_lead_straight_radius_m` / `steering_lead_curve_radius_m`). `/encoder`는 명령 쪽 각도이므로 `vehicle.yaml`의
   실측표를 역으로 적용한다.
3. **피드백 watchdog** — `steering_feedback_required: true`. 시뮬레이션은
   `/simulation/encoder`로 켠 채 돈다 (`ideal:=true`일 때만 끈다).
4. **heading 게이트** — `/ekf/heading_verified`(Bool, latched)가 true가 될 때까지
   `STOP HEADING_UNVERIFIED`. ESKF는 켜진 직후 지난 세션의 저장값으로 heading을 내기
   때문이다(2026-09-21에 124° 틀림). 정지 상태로 LiDAR 정합을 기다리거나 10 m 직진.
5. **조향 영점 오프셋** — `vehicle.yaml`의 `steering_command_offset_rad: -0.21`(2026-09-23
   bag 실측: 직진 시 엔코더 중앙값). Teensy 영점(`/steering_center`)을 다시 잡으면
   직진 bag에서 다시 재야 한다. 조향 실측표도 같은 bag의 yaw-rate/속도로 다시 만들었다
   (우 11.7°/R 6.5 m, 좌 15.2°/R 4.9 m at 99° 명령).

진단 토픽: `/stanley_node/cross_track_error`(m), `heading_error`(rad), `yaw_rate_error`(rad/s),
`desired_steering`(rad, 감쇠 적용 후 요청), `measured_steering`(rad, 실측).

튜닝 순서 (한 번에 하나, 5 km/h에서): ① 직선에서 흔들림 `heading_gain 0.6→0.5`
② cte 부호가 계속 바뀜 `cross_track_gain 0.3→0.25` ③ 복귀가 느리고 흔들림 없음 `0.3→0.4`
④ 직선에서 조향이 앞섬 `max_steering_command_lead_deg 5→3`, 코너에서 바깥으로 밀림 `max_steering_command_lead_curve_deg 12→16` ⑤ 코너 진입이 늦음
`heading_window_m 4→3` ⑥ 조향이 너무 느리고 흔들림 없음 `vehicle.yaml max_steering_rate_deg_s 20→30`.

되돌리기: `yaw_rate_damping_gain_s: 0.0`, `steering_feedback_required: false`,
`max_steering_command_lead_deg: 0.0`이면 기본 Stanley로 동작한다.

## 신호등 정지 (2026-10-02)

`krowd_mission_manager`와 같은 규칙을 `signal_stop.py`에 다시 구현했다 (다른 패키지는 수정·의존하지 않음).

- 맵 폴더의 `traffic_detect_zones.yaml`(구간, waypoint 번호)과 `left_turn_zones.yaml`(좌회전 구간)을
  `route_publisher_node`가 읽어 경로 거리로 바꾸고 `/planning/signal_zones`(latched JSON)로 낸다.
- 구간 안: 속도 명령 7 km/h 제한. 구간 전에는 0.5 m/s²로 미리 줄인다.
- 구간 방향의 빨간불 2프레임 연속(`/traffic_light/detections_json`) → `STOP SIGNAL_RED`, 조향 유지.
  같은 방향의 확정 GO가 와야 풀린다. 구간 끝을 넘어가도 유지된다.
- 신호를 모르거나 검출 노드가 없으면 통과한다. 구간 밖의 빨간불은 무시한다.
- 구간이 없는 맵에서는 아무 일도 하지 않는다. 끄려면 `signal_stop_enabled: false`.

**정지선 (`stopline_enabled: true`)**: 차선인식의 `/glane3d/stopline_markers`를
`stopline.py`(krowd_final_control 추정기의 복사본)로 읽는다. 빨간불일 때 정지선이 보이면 구간
속도로 더 가서 앞범퍼가 정지선 1 m 앞에 서도록 제동한다 (`STOP SIGNAL_RED ... (at the stop line)`).
구간 끝은 넘지 않는다. 정지선이 없거나 2 s보다 오래됐으면 빨간불을 본 자리에서 선다.
시뮬레이션용 가짜 정지선: `fake_stopline.py <map_dir> <waypoint idx>`.

시뮬레이션 시험 (반드시 시험 도메인에서, 실제 토픽 이름으로 발행하므로):

```bash
ROS_DOMAIN_ID=77 ros2 launch krowd_stanley_control stanley_dynamics_sim.launch.py \
    map_dir:=~/krowd_maps/20260913_103732_repaired initial_waypoint_index:=40
ROS_DOMAIN_ID=77 ros2 run krowd_stanley_control fake_traffic_light.py red:40 green
```

## 장애물 회피 인계 (2026-10-07)

맵의 `avoidance_zones.yaml` 구간(RViz 하늘색)에서 차를 세우고 `cone_avoidance_1005`에 넘긴다
(`avoidance_handoff.py`, `avoidance_supervisor_node.py`). cone 패키지는 수정하지 않고 런치만 실행한다.

1. 구간 전: 0.5 m/s²로 5 km/h까지 감속 → 구간 시작점에서 정지 (`STOP AVOIDANCE_HANDOFF`).
2. 정지 확인(0.3 m/s 이하 1 s, 경로 방향 5° 이내; 10 s 넘으면 경고 후 그냥) →
   `/stanley_node/handoff` 발행, 액추에이터 토픽 침묵 (`HANDOFF AVOIDANCE ...`).
3. `avoidance_supervisor_node`가 `ros2 launch cone_avoidance_1005 cone_avoidance_1005.launch.py
   use_can:=false use_rviz:=false`를 띄운다 (launch 인자 `cone_launch_command`).
4. 먼저 오는 조건에서 cone 런치를 끄고 `/stanley_node/resume`: `mission_status` DONE /
   구간 끝 통과 / 첫 명령 10 s 없음 / 120 s 초과. Stanley는 현재 위치부터 다시 추종한다.
5. 같은 구간은 다음 바퀴에 시작 전으로 돌아올 때까지 다시 넘기지 않는다.

시뮬레이션에서는 기본으로 **실제 cone_avoidance_1005 스택**이 뜬다. LiDAR 대신
`fake_lidar_drums.py`가 드럼통 줄(기본 3개씩 두 줄: 구간 시작 10 m 왼쪽, 17 m 오른쪽,
`obstacles:=` 인자)과 지면을 포인트클라우드로 내고, `/simulation/odometry`를 `/ekf/odometry`로
중계한다. cone 설정은 `config/sim_cone_overrides.yaml`(RTK 끔, 포인트 토픽)로 덮어쓴다.
`avoidance_demo_sec:=N`을 주면 대신 가짜 cone 미션(`fake_cone_mission.py`, 직진 7 km/h,
N초 뒤 DONE)이 뜬다. 구간이 있는 맵: 20261003_220932 (#107-135).

cone 단독 런치는 `extra` 파라미터가 적용되지 않아 cone_control이 `/target_angle`·`/target_speed`를
직접 내고, guard는 제어기 입력을 못 받아 `COMMAND_STALE` 브레이크를 낸다 (2026-10-08 실차에서 두 발행자
충돌). 그래서 실차 런치 명령에 `overrides:=config/cone_standalone_overrides.yaml`을 붙여 제어기 출력을
guard 입력 토픽으로 보낸다. 시뮬레이션의 `sim_cone_overrides.yaml`에도 같은 줄이 있다.

cone 단독 런치의 command_guard는 `/cone_avoidance_1005/controller/brake`가 false로 와야
출발하는데 그 토픽은 통합 모드에서만 발행된다. 그래서 감독 노드가 cone이 도는 동안 false를,
끌 때 true를 낸다 (`controller_brake_topic`). 2026-10-07 실차에서 차가 안 움직인 이유 중 하나.

## 파라미터

| 파일 | 내용 |
|---|---|
| `config/path_tracking.yaml` | 경로 속도 프로필, Stanley 게인, 속도 보정, corridor — 맨 위에 증상별 표 |
| `config/vehicle.yaml` | 축거, 조향 한계, 조향 실측표(엔코더 ↔ 타이어각) |
| `config/sim_overrides.yaml` | 폐루프 시뮬레이션 기본(실차 조건) 덮어쓰기 |
| `config/sim_ideal.yaml` | 시뮬레이션 `ideal:=true`용: 피드백·lead·heading 게이트 끔 |
| `config/sim_heading_demo.yaml` | 시뮬레이션 `heading_demo_sec:=N`용: N초 뒤 검증, creep 3 s 대기 |

launch가 소스 폴더의 yaml을 직접 읽으므로 **고치고 launch만 다시 띄우면 된다** (빌드 불필요).
launch 인자 `speed_limit_kmph`(0 = 상한 없음, 값을 주면 yaml 속도를 낮추기만 함)와
`goal_mode`(기본 `STOP_AT_END`)는 yaml보다 우선한다 — 기본값은
`launch/stanley_waypoint_follow.launch.py`의 `default_value`.

## launch 파일

| 파일 | 용도 |
|---|---|
| `stanley_waypoint_follow.launch.py` | 실차 주행 (터미널 2) |
| `stanley_dynamics_sim.launch.py` | 폐루프 시뮬레이션 (터미널 2) |
| `path_tracking.launch.py` | 위 실차 launch가 내부에서 부르는 본체 (경로 발행 + 차량 상태 + `stanley_node`). 지우면 안 된다 |

## 알려진 제약 (2026-09-20)

- 조향 명령 한계 99°: 차량 PC의 `steering_teensy` 소스가 100° 초과 명령을 거부한다.
  실측 road-wheel 우 11.7°(R 6.5 m) / 좌 15.2°(R 4.9 m). 이보다 급한 코너는 어떤 튜닝으로도 못 돈다. 펌웨어가 ±2.5 rad로 올라간 것이 확인되면
  `vehicle.yaml`의 `max_steering_command_deg`를 143.2394488로.
- 조향 실측표: ±1.73/±1.0 rad 4점은 bag 실측, 양 끝단은 09-19 수측정(영점 가정), 그 사이는 선형 보간.
- `/wheel_sensor`는 3.6배 크게 나온다 (CAN 수신부 km/h→m/s 누락). 이 패키지는 쓰지 않는다.
- 속도 보정(`speed_trim_*`)과 Stanley 게인(`cross_track_gain` 0.3, softening 10.8,
  `heading_gain` 0.6), yaw 감쇠 0.15 s, lead 3°는 실측이 아닌 추정값. rosbag(`/target_angle` `/encoder` `/target_speed` `/ekf/odometry`)으로 맞춘다.
- 맵 `20260919_121644`의 `.mm`은 차량 PC의 MOLA가 못 읽는다(직렬화 버전 6) — 경로 추종은 되지만
  LiDAR 정합은 안 된다.

## 테스트

```bash
cd ~/krowd2026_ws/src/krowd_stanley_control && python3 -m pytest test -q
```
