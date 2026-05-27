# CLAUDE.md

이 파일은 Claude Code가 이 저장소에서 작업할 때 참고하는 가이드입니다.
세부 알고리즘/상수 사양은 [`README.md`](./README.md)를 1차 출처(source of truth)로 사용하세요.

---

## 1. 프로젝트 개요

STM32F429 (STM324x9I-EVAL) 기반 임베디드 챌린지 — **벽 추종(wall-following) 자율주행 로봇**.

- 센서: 초음파 3 (전/좌/우), IR 3 (바닥/좌/우)
- 액추에이터: 좌/우 차동 구동 DC 모터 2개 (엔코더 포함)
- RTOS: CMSIS-RTOS (FreeRTOS) — `SensorTask` / `IR_Task` / `ControlTask` 3개 태스크가 20 ms 주기로 협조 구동
- 제어: 5-state FSM (`INIT → SEEK → ALIGN_PROGRESS / NON_ALIGN_PROGRESS / EMERGENCY`)
- 추적 방식: **Dynamic Wall Tracking** (좌/우 sticky swap)

알고리즘 의도/상수/함수 시그니처는 README에 모두 표로 정리되어 있습니다. 코드와 README가 어긋나면 **둘 중 어느 쪽이 stale인지** 먼저 확인하세요 (단순히 코드만 고치지 마세요).

---

## 2. 디렉토리 구조

```
EmbededChallenge/
├── README.md              # 사양 / FSM / 함수 목록 / 상수 표 (1차 출처)
├── CLAUDE.md              # (이 파일)
├── main_good.c            # 작업용 참고 스냅샷 (브랜치 산출물)
├── term_code.c            # 작업용 참고 스냅샷 (브랜치 산출물)
├── project/RTOS/
│   ├── Inc/
│   │   ├── main.h                  # FSM enum, DIR_* 비트마스크, BSP include
│   │   ├── stm32f4xx_hal_conf.h
│   │   └── stm32f4xx_it.h
│   ├── Src/
│   │   ├── main.c                  # 본 작업 대상 — peripherals + 3 task 본체
│   │   ├── stm32f4xx_hal_msp.c
│   │   ├── stm32f4xx_it.c          # ISR (encoder, ultrasonic IC)
│   │   └── system_stm32f4xx.c
│   ├── EWARM/                      # IAR 프로젝트
│   ├── MDK-ARM/                    # Keil µVision 프로젝트
│   └── TrueSTUDIO/                 # Atollic / Eclipse 프로젝트
├── Drivers/                # ST HAL + BSP (STM324x9I_EVAL 등) — 수정 금지
└── Middlewares/            # CMSIS-RTOS / FreeRTOS — 수정 금지
```

**작업 중심은 `project/RTOS/Src/main.c` 와 `project/RTOS/Inc/main.h` 두 파일** 입니다. 나머지 BSP/HAL/Middleware는 ST/ARM 제공 코드라 건드리지 않습니다.

---

## 3. 빌드 / 플래시

CMake/Make 빌드 시스템은 없습니다. 세 가지 IDE 중 하나로 엽니다:

| IDE        | 프로젝트 파일                                  |
| ---------- | ----------------------------------------------- |
| Keil µVision | `project/RTOS/MDK-ARM/Project.uvprojx`         |
| IAR EWARM   | `project/RTOS/EWARM/Project.eww`               |
| TrueSTUDIO  | `project/RTOS/TrueSTUDIO/STM324x9I_EVAL/...`   |

타깃: **STM32F429xx** (STM324x9I-EVAL RevB).
Claude는 빌드/플래시를 직접 수행할 수 없습니다 — 사용자가 IDE에서 실행합니다. 빌드 결과나 거동 확인이 필요하면 사용자에게 요청하세요.

---

## 4. 코드 아키텍처 (README §Functions 요약)

### Sensing Layer
- 신호당 **7-샘플 슬라이딩 윈도우** 유지
- `median7()` → 필터링된 거리값, `stddev7()` → 신뢰도
- ring buffer 가 변화율(rate of change)도 노출

### Motor Layer
- `Motor_Drive(v_left, v_right)` — 부호 = 방향, 절대값 = PWM duty (TIM4/TIM8, ARR=20000)
- `Motor_Stop()` — coast (PWM 채널 유지)
- `rotate_iterative(degrees, left)` — 엔코더 기반 micro-pivot 회전 primitive
  - **회전 캘리브는 `PIVOT_SUBSTEPS_90`만 조정** (틱 수 `PIVOT_SUBSTEP_TICKS=30` 고정)

### Control Layer (FSM)
5 상태: `INIT / SEEK / ALIGN_PROGRESS / NON_ALIGN_PROGRESS / EMERGENCY`
- `switchTracking()` — sticky 정책의 추적측 swap
- `angleCalculate()` — arctan 기반 heading 편차 (signed degrees)
- `angleAdjusting()` — `|θ| >= ANGLE_CORRECT_DEG` 시 이벤트성 회전 보정
- `canProgressDirection()` — `DIR_LEFT|FORWARD|RIGHT` 비트마스크 (`main.h`)
- `isEmergency()` / `emergencyResolved()` — 정면 충돌 hysteresis

### 태스크 주기
세 태스크 모두 **20 ms 주기** (`CTRL_PERIOD_MS`/`SENS_PERIOD_MS`/`IR_PERIOD_MS`).
ControlTask 는 300 ms warm-up 후 시작 (Sensor/IR 은 200 ms).

---

## 5. 상수 / 캘리브 위치

- **공유 타입** (FSM enum, `DIR_*`, `READY`): `project/RTOS/Inc/main.h`
- **튜닝 상수 전부** (`D_TARGET`, `EMG_FRONT`, `PIVOT_*`, `ANGLE_*`, PWM duty …): `project/RTOS/Src/main.c` 상단 *Calibration / tunables* 블록
- **캘리브 모드**: `CALIB_PIVOT` (0/1) — FSM 대신 4×90° 회전 시퀀스 후 정지. / `CALIB_IR` (0/1) — 모터 정지, 100ms 마다 IR raw ADC printf. 둘 다 일반 주행 시엔 0.

상수를 추가하거나 의미를 바꾸면 **README의 상수 표도 같이 업데이트**해야 합니다. 둘이 어긋나면 사람이 헷갈립니다.

### Pivot 캘리브 결과 (2026-05-27)
좌/우 회전이 **비대칭**으로 확정됨 (`PIVOT_SUBSTEPS_90_L = 24`, `PIVOT_SUBSTEPS_90_R = 25`). 모터/엔코더 polarity + 차동구동 미러 때문에 단일 상수로는 양쪽 90° 못 맞춥니다. 향후 모터/바퀴 교체 시 `CALIB_PIVOT` 모드로 재측정 필요. **측정 노이즈 큼** — run-to-run 으로 ±10° 변동 흔함 (배터리 전압/모터 열). 한 자리수 °의 잔여 오차는 `angleAdjusting()` 이 직진 중 보정한다는 전제.

---

## 6. 작업 시 규칙

- **README 와 코드의 단일 출처 정렬**: 함수 시그니처/상수 값이 바뀌면 README 표도 같이 수정하세요. 한쪽만 고치면 안 됩니다.
- **HAL/BSP/Middleware (`Drivers/`, `Middlewares/`) 는 수정 금지** — ST/ARM 제공 코드. 필요하면 wrapper 를 `main.c` 측에 작성합니다.
- 동작 검증은 사용자 손에 있습니다 — Claude 는 컴파일·실행을 할 수 없으므로 "테스트했다"고 말하지 마세요. 대신 변경 사항이 어떤 시나리오에서 어떻게 거동할지 추론을 명시하세요.
- `main_good.c`, `term_code.c` 는 브랜치 산출물 (참고용 스냅샷). `project/RTOS/Src/main.c` 가 진짜 빌드 대상입니다 — 혼동 금지.
- 주석은 *왜* (non-obvious 한 의도/제약/하드웨어 quirk) 만. *무엇* 은 코드가 말합니다.

---

## 7. 자주 참조하는 위치

| 작업 의도                       | 가야 할 곳                                              |
| ------------------------------- | ------------------------------------------------------- |
| FSM 상태 추가/수정              | `main.h` (enum) + `main.c` (`ControlTask` switch)       |
| 거리 임계 튜닝                  | `main.c` 상단 *Distance Thresholds* + README 표         |
| 회전 정확도 캘리브              | `main.c` `PIVOT_SUBSTEPS_90` + README *Iterative Pivot* |
| 새 센서 채널 추가               | `main.c` peripheral init + `SensorTask` ring buffer     |
| 모터 동작 변경                  | `Motor_Drive` / `rotate_iterative`                      |
| ISR (encoder, ultrasonic IC)    | `project/RTOS/Src/stm32f4xx_it.c`                       |
