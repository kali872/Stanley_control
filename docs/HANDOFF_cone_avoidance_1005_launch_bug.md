# cone_avoidance_1005 단독 런치 버그 인계

작성 2026-10-08 · 발견 경위: Stanley 인계(avoidance_supervisor_node)로 실차에서 cone 런치를 띄웠을 때
차가 움직이지 않고 액추에이터 토픽에 발행자 두 개가 충돌함

## 1. 요약

`cone_avoidance_1005.launch.py`(단독 모드)에서 두 가지가 안 됩니다.

1. **control 노드의 출력 토픽 설정이 적용되지 않습니다.** 런치는 control 출력을 guard 입력
   (`/cone_avoidance_1005/controller/*`)으로 보내도록 `extra` 파라미터를 주지만, 실제 노드는
   설정 파일 기본값인 `/target_angle`, `/target_speed`로 직접 발행합니다.
   - 결과 1: control이 액추에이터 토픽에 7 km/h를 직접 보냅니다.
   - 결과 2: guard는 제어기 입력을 못 받아 `COMMAND_STALE`로 속도 0 + brake를 보냅니다.
   - 같은 토픽에 두 발행자가 번갈아 쓰여 CAN bridge가 "7 km/h"와 "brake"를 동시에 받습니다.
2. **guard의 제어기 brake 입력이 단독 모드에서는 영원히 true입니다.** guard는
   `/cone_avoidance_1005/controller/brake`를 받아야 브레이크를 푸는데, 이 토픽은 통합 모드의
   integration_control만 냅니다. 단독 모드에서는 아무도 내지 않아 초기값 true가 유지되고
   `CONTROLLER_BRAKE`가 됩니다. (1번이 고쳐져도 이것 때문에 출발하지 못합니다.)

Stanley 쪽에서는 임시로 우회해 두었습니다(5절). cone 패키지에서 고쳐 주시면 우회를 걷어내겠습니다.

## 2. 재현

도메인 77에서 런치만 띄우고 파라미터를 읽으면 됩니다. 센서 없이도 재현됩니다.

```bash
ROS_DOMAIN_ID=77 ros2 launch cone_avoidance_1005 cone_avoidance_1005.launch.py use_can:=false use_rviz:=false
# 다른 터미널
ROS_DOMAIN_ID=77 ros2 param get /cone_control topics.target_speed
ROS_DOMAIN_ID=77 ros2 param get /cone_control topics.steer_cmd
ROS_DOMAIN_ID=77 ros2 topic info -v /target_speed | grep -A1 "Node name"
```

관측값 (2026-10-08, 차량 PC, install == source 확인함):

| 실행 | `topics.target_speed` | `topics.steer_cmd` | `/target_speed` 발행자 |
|---|---|---|---|
| 런치 기본 | `/target_speed` | `/target_angle` | cone_control, cone_command_guard |
| `overrides:=<rtk·pointcloud만 바꾼 파일>` | `/cone_avoidance_1005/controller/target_speed` | `/target_angle` (적용 안 됨) | - |
| `overrides:=<topics.steer_cmd·target_speed를 직접 쓴 파일>` | controller 토픽 | controller 토픽 | cone_command_guard만 |

실차 로그 (2026-10-08 21:36~21:38, 인계 후 120 s):
- guard 상태: `{"fault": true, "brake": true, "reason": "COMMAND_STALE", ...}` 계속
- `/target_speed` 발행자: `cone_control`, `cone_command_guard` (+ 침묵 중인 stanley_node 엔티티)
- 차량 속도 0.01 m/s, mission 상태 CRUISE 유지

## 3. 관련 코드 (읽기만 했습니다)

`launch/cone_avoidance_1005.launch.py`:

```python
def node(executable, name, extra=None, **kwargs):
    return Node(package="cone_avoidance_1005", executable=executable, name=name,
                output="screen", parameters=[config, overrides, common] + ([extra] if extra else []), **kwargs)
...
node("control", "cone_control", extra={
    "topics.steer_cmd": "/cone_avoidance_1005/controller/target_angle",
    "topics.target_speed": "/cone_avoidance_1005/controller/target_speed",
}),
```

- `overrides`의 기본값은 `config`와 같은 파일입니다. 즉 기본 실행은 `[config, config, common, extra]`.
- 노드는 `ros_common.declare_config()`에서 `DEFAULTS`를 평탄화한 이름(`topics.target_speed` 등)으로
  `declare_parameter`하고 초기값을 읽습니다. 메커니즘은 표준이라, 왜 `extra` dict가 무시되는지는
  확인하지 못했습니다. 위 표처럼 **파일로 주면 적용되고 dict로 주면 적용되지 않는** 현상만 확인했습니다.
  (`sim.launch.py`도 같은 패턴을 씁니다. 같은 문제일 수 있습니다.)

guard 쪽 (`command_guard_node.py`):

```python
d("input_brake_topic", "/cone_avoidance_1005/controller/brake")
...
self.brake = True                      # 초기값, 메시지가 와야 바뀜
d("require_brake_input", False)        # 오래된 brake는 무시하지만, 한 번도 안 오면 초기값 유지
```

`README.md` 114행에 `/cone_avoidance_1005/controller/brake`는 "(통합 모드)"로 적혀 있습니다. 단독 모드에는
발행자가 없습니다.

## 4. 수정 제안

1. **출력 토픽:** `extra` 파라미터 대신 `Node(remappings=[...])`로 control의 출력 토픽을 바꾸거나,
   control 노드 전용 설정 파일(또는 `config` 자체의 `topics.steer_cmd/target_speed`)을 guard 입력 토픽으로
   두세요. 파일로 주면 적용된다는 것은 확인됐습니다.
2. **guard brake 초기값:** 단독 모드에서는 `brake` 초기값을 false로 두거나, control 노드가
   `/cone_avoidance_1005/controller/brake`를 발행하게 하세요. 통합 모드처럼 외부 노드가 낼 때만 기다리게
   하려면 `require_brake_input`과 묶어서 처리하는 편이 안전합니다.
3. **고친 뒤 확인:** 2절의 명령으로 `/target_speed`, `/target_angle`, `/brake` 발행자가 `cone_command_guard`
   하나인지, guard 상태가 `LIMITED_5_KMH`(또는 `PASS`)인지 보면 됩니다.

## 5. Stanley 쪽 임시 우회 (krowd_stanley_control, 적용 완료)

- `config/cone_standalone_overrides.yaml`: `topics.steer_cmd`, `topics.target_speed`를 guard 입력 토픽으로 지정.
  실차 런치 명령에 `overrides:=` 로 붙여 실행합니다.
- `avoidance_supervisor_node`가 cone이 도는 동안 `/cone_avoidance_1005/controller/brake`에 false를, 끌 때
  true를 20 Hz로 냅니다.
- 시뮬레이션 overrides(`config/sim_cone_overrides.yaml`)에도 같은 토픽 줄이 있습니다.

cone 패키지가 고쳐지면 위 두 우회는 제거해야 합니다. 특히 brake false 발행은 cone 쪽이 brake를 직접 내기
시작하면 서로 겹칩니다.

## 6. 함께 본 것 (참고)

- 실차 PC에서 DDS 노드 발견이 느립니다. 감독 노드가 cone guard의 첫 명령을 보기까지 6.4 s, Stanley가
  경로 구간 메시지를 받기까지 31 s 걸린 적이 있습니다. 노드 61개, UDP 소켓 263개, Fast-DDS 기본 설정,
  Wi-Fi·LiDAR NIC 두 개. `ROS_LOCALHOST_ONLY=1` 또는 Fast-DDS 프로파일로 Wi-Fi 인터페이스를 빼는 방법을
  검토 중입니다 (전체 스택에 영향이 있어 아직 적용하지 않음).
- ESKF는 IMU 간격이 0.2 s를 넘으면 `reset_runtime`으로 heading 검증을 버립니다. cone 스택이 뜰 때의 부하에서
  일어난 것으로 보입니다. krowd_sensors 소관입니다.
