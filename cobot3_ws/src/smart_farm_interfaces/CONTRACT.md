# smart_farm_interfaces 계약

기준 문서: [`docs/02-interfaces.md`](../../../docs/02-interfaces.md).

Isaac Sim 브리지와의 결합도를 낮추기 위해 작업 명령·결과 Topic에는 사용자 정의 ROS 메시지를 만들지 않는다.
모든 Topic payload는 `std_msgs/msg/String`의
UTF-8 JSON 객체다. 이 패키지는 시작 요청에만 `StartCycle.srv`를 제공한다.

## Service

`/start_cycle`은 `smart_farm_interfaces/srv/StartCycle` 타입이다.

| 방향 | 필드 |
| --- | --- |
| Request | `scenario_id: string` |
| Response | `accepted: bool`, `task_id: string`, `reason: string` |

현재 `scenario_id`는 `RACK_CYCLE_01`만 허용한다.
실행 중인 작업이 있으면 `accepted`는 `false`, `reason`은 `BUSY`다.

## Topic JSON

| Topic | 발행자 → 구독자 | JSON 필수 키 |
| --- | --- | --- |
| `/navigation/command` | `task_manager` → `navigation_node` | `command_id`, `task_id`, `destination` |
| `/navigation/result` | `navigation_node` → `task_manager` | `command_id`, `task_id`, `status`, `reason`, `reached_station` |
| `/transport_arm/command` | `task_manager` → `transport_arm_node` | `command_id`, `task_id`, `pallet_id`, `operation`, `station` |
| `/transport_arm/result` | `transport_arm_node` → `task_manager` | `command_id`, `task_id`, `pallet_id`, `operation`, `status`, `reason` |

### Payload examples

```json
{"command_id":"c-01","task_id":"t-01","destination":"RACK_L2"}
```

```json
{"command_id":"c-01","task_id":"t-01","status":"SUCCEEDED","reason":"NONE","reached_station":"RACK_L2"}
```

```json
{"command_id":"c-02","task_id":"t-01","pallet_id":"PALLET_02","operation":"PICK","station":"RACK_L2"}
```

```json
{"command_id":"c-02","task_id":"t-01","pallet_id":"PALLET_02","operation":"PICK","status":"SUCCEEDED","reason":"NONE"}
```

## 값과 처리 규칙

- `pallet_id`: `PALLET_01`~`PALLET_04`, `PALLET_SEED`
- `destination`, `station`: `RACK_L1`~`RACK_L4`, `SEED_PICKUP`, `INSPECT_ZONE`
- `operation`: `PICK`, `PLACE`
- `status`: `SUCCEEDED`, `FAILED`, `CANCELED`, `TIMEOUT`
- `reason`: `NONE`, `INVALID_COMMAND`, `BUSY`, `SIM_NOT_READY`, `NO_FEEDBACK`,
  `NAV_FAILED`, `MOTION_FAILED`, `TIMEOUT`

명령 수신자는 실행 중 새 명령을 수행하지 않고 해당 `command_id`로
`FAILED`/`BUSY` 결과를 발행한다. 완료된 `command_id`가 재전송되면 재실행하지 않는다.
결과 수신자는 `command_id`, `task_id` 및 팔 작업의 경우 `pallet_id`를 대조한 뒤에만 다음 작업을 시작한다.
