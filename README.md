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
- `int median7(int *)`
- `int stddev7(int *)`
###### Mainfunction
- `void SensorTask(void)`
#### IR_Task
###### Mainfunction
- `void IR_Task(void)`
#### ControlTask
###### Subfunction
- `void Motor_Drive(int v_left, int v_right)`
- `void Motor_Stop(void)`
- `void switchTracking(void)`
- `void angleAdjusting(void)`
- `int  angleCalculate(void)`
- `uint8_t canProgressDirection(void)`
- `bool isEmergency(void)`
- `bool emergencyResolved(void)`
###### Mainfunction
- `void ControlTask(void)`
