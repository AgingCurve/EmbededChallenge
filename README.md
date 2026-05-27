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

---

## 결론
**수학적으로 정확합니다.** 짚으신 비율의 arctan이 정확히 "로봇 진행방향과 벽 사이의 각도"가 됩니다. 기하학으로 유도가 깔끔하게 떨어집니다.

## 유도

로봇이 자기 진행방향(heading)으로 Δs만큼 직진했다고 가정. 벽과 진행방향 사이 각도를 θ라 하면:

- **위치 변화**: 벽 방향으로 Δs·cos θ, 벽 수직 방향으로 Δs·sin θ
- **앞 거리 변화**: ΔdF = −Δs (vert 방향이 heading이므로 θ 무관)
- **옆 거리 변화**: ΔdR = −Δs·tan θ (벽에 가까워지는 만큼)

따라서:

$$\frac{\Delta d_R}{\Delta d_F} = \frac{-\Delta s \cdot \tan\theta}{-\Delta s} = \tan\theta$$

$$\boxed{\theta = \arctan\left(\frac{d_{R,\,prev} - d_{R,\,curr}}{d_{F,\,prev} - d_{F,\,curr}}\right)}$$## 부호 처리 (방향 결정)

| 추종 벽 | 사용 비율 | θ &gt; 0 의미 | 보정 방향 |
|---|---|---|---|
| 우측 | `(dR_pre − dR_cur) / (dF_pre − dF_cur)` | 우벽으로 기움 | 좌로 보정 |
| 좌측 | `(dL_pre − dL_cur) / (dF_pre − dF_cur)` | 좌벽으로 기움 | 우로 보정 |

분모·분자 둘 다 정상 직진 시 양수(전·옆 모두 가까워짐). 부호가 음수로 나오면 "벽에서 멀어지는 중"이라는 신호로 해석.

## 함정 (이게 가장 중요)

**1. 전방 벽이 없으면 분모가 0 또는 노이즈 덩어리**
ΔdF가 작거나 측정 불안정하면 division by near-zero → 발산.
- `|dF_pre - dF_cur| < ε` 이면 계산 skip
- 또는 dF가 OPEN 상태(추종 중 전방 멀음)면 이 방법 자체가 무용

**2. Single-cycle Δ는 너무 작아 노이즈에 묻힘**
제어 주기 20ms, 속도 30cm/s → 한 cycle당 Δs ≈ 0.6cm. 센서 해상도 1cm + 노이즈 ±1cm이면 SNR이 박살남.

해결:
- **누적 윈도우** 사용: 10~25 cycle (200~500ms) 동안의 (dR_pre - dR_cur) 합산
- 또는 매 N cycle마다 평가
- 추가 median/EMA로 ratio 자체를 한 번 더 필터링

**3. 작은 각도에서 분해능 한계**
θ = 2°에서 tan θ ≈ 0.035. Δs = 6cm (200ms 누적)이면 ΔdR ≈ 0.2cm. **센서 해상도 이하**.
이 방법은 **5° 이상 큰 기울임만 신뢰 가능**. 미세 정렬은 PD에 맡기는 게 현실적.

**4. 회전 중에는 무효**
로봇 자체가 회전 중이면 ΔdR/ΔdF 관계가 깨짐. WALL_FOLLOW 직진 구간에서만 계산.

**5. 벽이 평행하지 않으면 측정 오류**
복도가 약간 휘어 있거나, 두 벽이 평행 아니면 가정 깨짐. **종이 박스 벽**은 박스 자체가 살짝 기울 수 있어 주의.

## 대안 — Encoder가 있다면 더 robust

엔코더로 Δs를 직접 알 수 있으면:

$$\tan\theta = \frac{-\Delta d_R}{\Delta s_{encoder}}$$

이러면 **전방 벽 불필요**. 분모가 결정론적 값이라 noise 무관, 항상 계산 가능. 게다가 Δs가 충분히 크게 누적 가능 (cycle 무관).

엔코더가 없다면, 모터 출력값(PWM duty)으로 Δs를 거칠게 추정하는 것도 가능. 정확도는 떨어지지만 전방 벽 의존성은 제거.

## arctan 구현은 (지난 답변 참조)

각도 보정 한 cycle에 1번 정도 호출이면 `atan2f` 또는 `atanf`로 충분. STM32F4 FPU 있으면 200~400ns. 굳이 LUT나 다항식 근사 필요 없음.

```c
float tan_theta = (dR_prev - dR_curr) / ((float)(dF_prev - dF_curr));
float theta_rad = atanf(tan_theta);   /* −π/2 ~ +π/2 */
```

**ZeroDiv 방어**:
```c
int denom = dF_prev - dF_curr;
if (abs(denom) < MIN_DF_DELTA) {
    /* 전방 벽 없거나 정지 상태, θ 추정 불가 → 직전 값 유지 또는 skip */
    return last_theta;
}
float theta = atanf((float)(dR_prev - dR_curr) / (float)denom);
```

## 추천 사용 패턴

PD는 그대로 두고, **θ 추정은 보조 신호**로:

```
PD 제어:    error = dR_curr - D_TARGET           ← 거리 오차
보조 θ:     theta = atan(ΔdR / ΔdF)              ← 각도 오차

combined error = K_dist * error + K_angle * theta
```

`K_angle`은 작게 시작 (예: 1/5 of K_dist). PD는 거리 잡고, θ는 자세 잡음. 두 신호가 직교에 가까워 서로 간섭 적음 (cascade control 비슷).

다만 — **이 조합이 PD 단독보다 정말 더 좋은지는 실측 검증 필요**. PD의 D항(`(dR_curr - dR_prev)/Δt`)이 이미 어느 정도 각도 정보를 담고 있기 때문. 굳이 명시적 θ 계산이 필요한지는 의문.

## 정리

| 항목 | 평가 |
|---|---|
| 수학적 정당성 | ✓ 정확 |
| 자이로 없이 가능 | ✓ 작동 |
| 노이즈 robust | ✗ 약함 — 누적 윈도우 필수 |
| 작은 각도 분해능 | ✗ 5° 이하 어려움 |
| 전방 벽 의존 | ⚠️ 직진 통로에서만 작동 |
| PD 대비 우위 | ? 실측 필요 |

**진짜 권장**: 일단 PD만으로 가능한지 보고, 미세 자세 어긋남이 누적되는 게 관찰되면 그때 θ 추정을 보조로 추가. 처음부터 같이 넣으면 디버깅이 어려워집니다.

## 확장 키워드
**Optical flow** (영상에서 같은 원리로 motion 추정), **Wall-relative pose estimation**, **Lateral velocity estimation**, **Heading from differential range measurements**, **Numerical differentiation noise** (작은 차분의 노이즈 증폭), **Signal-to-noise ratio in derivative estimation**, **Sliding window accumulator**, **Cascade control** (inner loop angle, outer loop distance), **Dead reckoning without IMU**, **Visual odometry analogy**, **Time-difference of arrival**, **Sensor fusion at low cost**, **Kalman filter for heading** (encoder + range fusion), **Robustness to division-by-zero in ratio estimators**, **Sample-and-hold for stale estimates**, **Trigonometric identity exploitation**, **Encoder-based odometry**, **Differential drive kinematics**.

핵심 통찰: **사용자가 직감으로 잡은 비율이 정확히 cot(complementary angle) 또는 tan(deviation angle)**입니다. 자이로 없이 벽이라는 reference만으로 자세를 추정하는 영리한 방법이고, 실제로 자율주행 초기 연구에서 같은 방식이 쓰였습니다. 단 noise 처리만 신중하면 실용 가능합니다.
