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
<img width="1472" height="940" alt="image" src="https://github.com/user-attachments/assets/c565d8a1-b2dd-49ef-9c51-c8d5a318f502" />

## Non-Island Premised
- Dynamic Tacking Premised
- Parallel with wall Straight 
#### Sensing Layer
We need maintain current 7 signal as confidence of these observed value. also we can know rate of change. 
#### Motor Layer
We can choose drive mode. and need to make straight like based on rate of change, 
but, check some drift.(as far as from operating time, adaptive coorder Systems.)
#### Control Layer
based on FSM(Finite State Machine), transfer logic and checking conditions. also need switching tracking logics.. 

## Island Premised
- Dynamic Tracking Premised
- Parallel with wall Straight 
- Visited Like -Sparse event log
#### Sensing Layer
#### Motor Layer
#### Control Layer

#### ++Coorder Layer
