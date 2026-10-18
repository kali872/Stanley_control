# 인계: 제어 런치 시작 시 ESKF 헤딩 검증이 풀리는 문제 (krowd_sensors)

작성 2026-10-12 · 작성 범위: krowd_stanley_control 쪽에서 읽기 전용으로 확인한 내용. krowd_sensors 파일은 수정하지 않았습니다.

## 1. 증상

- 맵 localization + 센서 스택이 떠 있고 `/ekf/heading_verified`가 true인 상태에서
  `ros2 launch krowd_stanley_control path_tracking.launch.py`를 켜면 `/ekf/heading_verified`가 false로 떨어집니다.
- Stanley는 그 Bool을 읽기만 하므로 `STOP/CREEP HEADING_UNVERIFIED`로 출발을 못 하거나, 10 m 직진 재검증을 다시 합니다.
- 2026-10-08 실차에서 cone_avoidance_1005 런치가 뜰 때도 같은 현상이 있었습니다.
- Stanley 런치·노드에는 ESKF를 리셋하는 코드(서비스, 토픽, initialpose, TF)가 없습니다. ESKF가 구독하는 토픽 중 Stanley가 내는 것도 없습니다.

## 2. 원인 (코드 기준, `krowd_sensors/gps_imu_eskf_node.py`)

`/ekf/heading_verified` = `heading_verified` AND `imu_readiness() == "ready"` (`heading_is_usable`, 1144행; 0.1 s 타이머 `publish_heading_gate`, 1147행).
IMU(`/imu/data_ekf`, 100 Hz) 메시지 간격 `dt`에 따라 세 단계로 떨어집니다 (`on_imu`, 2090~2125행).

| 조건 | 코드 동작 | `/ekf/heading_verified` |
|---|---|---|
| `dt > imu_gap_warning_sec` (0.05 s) | 공분산 부풀림, `imu_gap_recovery_hold_sec`(1.0 s) 동안 + 다음 GPS fix(`imu_gap_gps_confirmed`, 2770행)까지 `imu_gap_recovering` | false (헤딩 값은 유지) |
| 위 + yaw σ > `imu_gap_max_yaw_sigma_deg` (12°) | `heading_verified=False`, `heading_source="imu_gap_uncertain"` (2120행) | false, 재검증 필요 |
| `dt > imu_gap_recovery_limit_sec` (0.50 s) | `reset_runtime(keep_origin=True)` (1049행) → 정지 상태 재캘리브레이션부터 다시 | false, 헤딩 완전 초기화 |
| IMU 수신 공백 > `imu_stale_timeout_sec` (0.20 s) | `imu_readiness()="imu_stale"` (1126행) | false |

로그 문구:
```
Short IMU gap 0.xxxs: state retained, covariance inflated; yaw sigma=..deg, control gate held
Long IMU gap 0.xxxs exceeds 0.500s; restarting stationary calibration
```

## 3. 왜 제어 런치가 IMU 간격을 만드는가

- IMU 체인이 전부 Python 단일 스레드입니다: `imu_dual_filter_node.py`(/imu/data → /imu/data_ekf, 10 ms 타이머) → `gps_imu_eskf_node.py`.
- 차량 PC에는 노드 약 61개, UDP 소켓 263개가 떠 있고 Fast-DDS 기본 설정에 Wi-Fi·LiDAR NIC 두 개입니다.
  새 런치가 Python 노드 4개를 동시에 띄우면 DDS 발견(discovery) 트래픽과 CPU 피크가 생기고, 그때 IMU 콜백이 50 ms 밀리면 1단계, 500 ms 밀리면 3단계가 됩니다.
- 같은 PC에서 노드 발견 자체도 느립니다(감독 노드가 cone guard 첫 명령을 보기까지 6.4 s, Stanley가 구간 메시지 받기까지 31 s 관측).

## 4. 아직 확인 못 한 것

- 실차에서 실제로 어느 단계가 걸렸는지. ESKF 로그는 터미널로만 나가고 `~/.ros/log`에는 테스트 로그만 있습니다.
  **다음 실차 시험에서 제어 런치를 켤 때 ESKF 터미널에 위 두 문구 중 무엇이 뜨는지 확인**해 주세요. 간격 값(dt)도 같이.
- `imu_dual_filter_node`가 밀리는지, `/imu/data` 드라이버(CAN/serial) 단계가 밀리는지. `ros2 topic hz /imu/data`와 `/imu/data_ekf`를 런치 순간에 같이 보면 구분됩니다.

## 5. 제안 (krowd_sensors 판단에 맡김)

1. **임계값 완화** (`config/gps_imu_eskf.yaml`): `imu_gap_warning_sec` 0.05 → 0.10~0.15, `imu_gap_recovery_limit_sec` 0.50 → 1.0.
   정지 상태에서만 일어나는 런치 피크라면 공분산 부풀림만으로 충분하고 전체 리셋은 과합니다.
   정지 중(속도 0, ZARU 구간)에는 긴 간격이라도 리셋 대신 부풀림만 하는 분기를 두는 방법도 있습니다.
2. **IMU 노드 우선순위**: 런치에서 `imu_dual_filter_node.py`, `gps_imu_eskf_node.py`를 `nice -n -5`(또는 `chrt`)로 띄우기. 현재 런치(`launch/localization.launch.py` 58·65행)에는 우선순위 설정이 없습니다.
3. **발견 트래픽 축소**: 모든 노드가 한 PC에 있으면 `ROS_LOCALHOST_ONLY=1`을 전 터미널 공통(`~/.bashrc`)에 넣기. 다른 PC나 RViz 원격 접속이 있으면 대신 Fast-DDS XML 프로파일로 Wi-Fi 인터페이스를 빼기.
   어느 쪽이든 전체 스택이 같은 설정이어야 하므로 센서 스택 담당이 결정해야 합니다.
4. 확인 방법: 설정 뒤 센서 스택 + 맵을 띄우고 `/ekf/heading_verified` true 상태에서 `ros2 launch krowd_stanley_control path_tracking.launch.py`를 켜서 `ros2 topic echo /ekf/heading_verified`가 false로 안 떨어지는지, ESKF 터미널에 gap 경고가 없는지 봅니다.

## 6. Stanley 쪽에서 할 수 있는 완화 (미적용)

런치에서 노드 4개를 1~2 s 간격으로 순차 기동해 CPU 피크를 낮추는 것. 요청이 있으면 krowd_stanley_control 안에서만 적용합니다. 근본 해결은 아닙니다.
