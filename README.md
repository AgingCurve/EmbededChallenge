## Premise
- ~~Right Tracking~~ $\rightarrow$ Dynamic switching Tracking
-  초음파 3: 전(dF) · 좌(dL) · 우(dR) — 거리 측정 2 ~ 40cm 각도 넓은 편
-  IR 3: 바닥 · 좌 · 우 (코앞 접촉 직전 범퍼) 3~15 cm 각도 좁은 편
-  모터 2: 좌/우 차동 구동 
- 직진시엔 벽과 parallel하게 직진한다고 가정.
#### Drive Scenario
<img width="1472" height="2710" alt="image" src="https://github.com/user-attachments/assets/29473391-1861-4f0a-98c7-ccb84dcbc591" />


ISLAND 없다고 가정시 T-Stem(5), OPEN(6)은 없는걸로 볼 수 있음.

## Finite State Machine
<img width="1472" height="1030" alt="image" src="https://github.com/user-attachments/assets/cc1a77b3-5687-4951-8b78-e307a2785b40" />


#### Sensing Layer
We maintain a 7-sample window per signal, giving us both the filtered value and its confidence measure. The rate of change is also available.
#### Motor Layer
The robot supports multiple drive modes. We maintain straight-line motion based on the rate of change between sensor readings, while compensating for drift using adaptive coordinate systems that scale with operating time.
#### Control Layer
An FSM handles state transitions and condition checks. Wall-switching logic is also required for dynamic tracking selection.

## Island Premised
- Dynamic Tracking Premised
- Parallel with wall Straight 
- Visited Like -Sparse event log
#### Sensing Layer
#### Motor Layer
#### Control Layer

#### ++Coorder Layer
---
## Functions
#### UltraSonicTask
###### Subfunction
- `int  median7(int *a)` — 7-샘플 슬라이딩 윈도우 중앙값 (insertion sort)
- `int  stddev7(int *a)` — 7-샘플 표준편차 (정수 floor-sqrt, 신뢰도 지표)
- `void angleHistoryFlush(void)` — 각도 보정 ring buffer 초기화 (회전 후/추적측 변경 시)
###### Mainfunction
- `void SensorTask(void *arg)` — 20ms 주기, 초음파 trio 샘플링 + 필터링 + ring buffer push

#### IR_Task
###### Mainfunction
- `void IR_Task(void *arg)` — 20ms 주기, IR ADC 3채널 (좌/우/바닥) 폴링

#### ControlTask
###### Subfunction (Motor Layer)
- `void Motor_Drive(int v_left, int v_right)` — 좌/우 PWM 듀티 설정 (부호 = 방향)
- `void Motor_Stop(void)` — 양쪽 듀티 0으로 coast (PWM 채널은 유지)
- `void rotate_iterative(int degrees, bool left)` — 엔코더 기반 micro-pivot 회전 (rotate primitive)

###### Subfunction (Control Helpers)
- `bool switchTracking(void)` — Dynamic tracking sticky 정책, 추적측 끊기면 swap
- `int  angleCalculate(void)` — arctan 기반 heading 편차 계산 (signed degrees, 소스 자동 선택)
- `void angleAdjusting(void)` — `|θ| >= ANGLE_CORRECT_DEG` 시 이벤트성 회전 보정
- `uint8_t canProgressDirection(void)` — 진행 가능 방향 bitmask (DIR_FORWARD/LEFT/RIGHT)
- `bool isEmergency(void)` — 정면 충돌 임박 판정 (`0 < dF <= EMG_FRONT`)
- `bool emergencyResolved(void)` — 정면 hysteresis 해제 (`dF > EMG_FRONT + EMG_FRONT_HYST`)

###### Mainfunction
- `void ControlTask(void *arg)` — 20ms 주기, 5상태 FSM 실행

---
## Constants & Defines

### `main.h` — 공유 타입
```c
#define READY  3                          /* encoder ISR sentinel */

typedef enum {
    INIT = 0, SEEK, ALIGN_PROGRESS,
    NON_ALIGN_PROGRESS, EMERGENCY
} DriveState;

typedef enum { TRACK_LEFT = 0, TRACK_RIGHT } TrackingSide;

#define DIR_LEFT     0x01                 /* canProgressDirection() bits */
#define DIR_FORWARD  0x02
#define DIR_RIGHT    0x04
```

### Sensing
| 상수 | 값 | 의미 |
|---|---|---|
| `SAMPLE_N` | 7 | 슬라이딩 윈도우 길이 |
| `US_TICKS_PER_CM` | 58 | TIM3 IC diff → cm 변환 분모 |

### Task Timing (ms)
| 상수 | 값 | 의미 |
|---|---|---|
| `CTRL_PERIOD_MS` | 20 | ControlTask 주기 |
| `SENS_PERIOD_MS` | 20 | SensorTask 주기 |
| `IR_PERIOD_MS` | 20 | IR_Task 주기 |
| `TASK_WARMUP_MS` | 200 | Sensor/IR Task 시작 지연 |
| `CTRL_WARMUP_MS` | 300 | ControlTask 시작 지연 |

### Distance Thresholds (cm)
| 상수 | 값 | 역할 |
|---|---|---|
| `D_TARGET` | 20 | 추종 가능 벽 범위 상한 (switchTracking, has_track) |
| `D_MIN` | 10 | 회피 트리거 (SEEK veer-off) |
| `D_OPEN` | 150 | "벽 없음" 임계 (예약, 향후 사용) |
| `EMG_FRONT` | 9 | 정면 EMERGENCY 진입 + 회전 안전 임계 (cm) |
| `EMG_FRONT_HYST` | 2 | EMERGENCY 해제용 +cm 마진 |
| `IR_BUMPER_THRESH` | 1500 | IR 범퍼 raw ADC 트리거 임계 — `ir_left/right > 이 값` 이면 EMERGENCY. 노이즈 floor 30-500, 진짜 근접 1500-2600+ 기준 튜닝 |

### Motor PWM
| 상수 | 값 | 의미 |
|---|---|---|
| `PWM_PERIOD` | 20000 | TIM4/TIM8 Auto-Reload (= 100% duty) |
| `V_CRUISE` | 15000 | 모든 직진 듀티 (SEEK/ALIGN/NON_ALIGN 공통) |
| `V_TURN` | 20000 | 피벗 회전 듀티 (full torque) |

### Iterative Pivot (회전 캘리브)
| 상수 | 값 | 의미 |
|---|---|---|
| `PIVOT_SUBSTEP_TICKS` | 30 | 마이크로 피벗 1회당 엔코더 틱 (~3°) |
| `PIVOT_SUBSTEPS_90_L` | 24 | 90° 좌회전 마이크로 피벗 횟수 (캘리브 2026-05-27 final) |
| `PIVOT_SUBSTEPS_90_R` | 25 | 90° 우회전 마이크로 피벗 횟수 (캘리브 2026-05-27 final) |
| `PIVOT_PAUSE_MS` | 10 | 마이크로 피벗 사이 정지 시간 |
| `PIVOT_SUBSTEP_TIMEOUT_MS` | 200 | 마이크로 피벗당 안전 timeout |
| `POST_TURN_SETTLE_MS` | 300 | 회전 후 median filter 갱신 대기 |

> L/R 분리: 좌/우 바퀴의 effective ticks-per-degree 가 미세하게 달라서 (모터·인코더 비대칭) 분리 상수가 필요. 캘리브는 `CALIB_PIVOT` 모드에서 4×90° 라운드트립으로 측정 → 시작/끝 방향 일치하도록 각 상수만 조정 (틱 수는 고정).

### Angle Correction (arctan 기반)
| 상수 | 값 | 의미 |
|---|---|---|
| `ANGLE_HISTORY_N` | 10 | Ring buffer 길이 (10 × 20ms = 200ms 누적) |
| `ANGLE_CORRECT_DEG` | 10 | 보정 발동 임계 (이하 무시) |
| `MIN_DF_DELTA_CM` | 2 | `|ΔdF|` 이하면 분모 신뢰 불가 → skip |

### EMERGENCY
| 상수 | 값 | 의미 |
|---|---|---|
| `EMERG_RELEASE_TICKS` | 25 | 회전 방향 commit 유지 시간 (~500ms) |

### Hardware
| 상수 | 값 | 의미 |
|---|---|---|
| `TRIG_PULSE` | 2 | HC-SR04 트리거 펄스 폭 |
| `ADC_POLL_TIMEOUT` | 0xFF | IR ADC 폴링 timeout |
