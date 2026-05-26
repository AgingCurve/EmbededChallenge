/**
  ******************************************************************************
  * @file    Lab8_RTOS/Src/main.c
  * @brief   임베디드 챌린지 - 벽 추종 + 동적 장애물 회피 자율주행
  *
  *  주행 알고리즘 ver3:
  *    1. 충돌하지 않는다 > 완주한다 > 빠르게 움직인다
  *    2. 우측 벽 추종 1순위, 못 잡으면 좌측 → 대칭 적용
  *    3. 모드를 이진으로 나누지 않고 교차점 의심도 S(0~1) 로 속도/신중함 연속 제어
  *
  *  하드웨어 매핑 (term_code.c 와 동일):
  *    초음파  : Right=TIM3_CH2 (uwDiffCapture1)
  *              Forward=TIM3_CH3 (uwDiffCapture2)
  *              Left=TIM3_CH4 (uwDiffCapture3)
  *              cm = uwDiffCaptureN / 58
  *    IR ADC  : Left=ADC3 CH11, Right=ADC2 CH14, Floor=ADC1 CH15
  *    모터    : Right=TIM8 (CH1 전진, CH2 후진)
  *              Left =TIM4 (CH2 전진, CH1 후진)
  *    인코더  : Right=PA15(+PB3), Left=PB4(+PB5)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include "cmsis_os.h"

/* =========================================================================
 *  파라미터 (5번 절: 실측 후 확정)
 * ========================================================================= */
/* 거리(cm) */
#define D_TARGET        15      /* 우측 벽까지 목표 거리 */
#define D_MIN           8       /* 안전 하한 */
#define D_OPEN          35      /* 우측 "열림" 판정 임계 (코너/교차점 의심) */
#define K_FRONT         28      /* 전방 트리거 (회전 결정) — 더 일찍 반응 */
#define EMG_FRONT       6       /* 비상정지 초근접 */
#define W_SAFE          8       /* 회전 가능 절대 측면 여유 — 더 너그럽게 */
#define R_CLEAR         8       /* 제자리 회전에 필요한 사방 최소 여유 */
#define MOVE_TH         2       /* 한 틱당 변화량이 이만큼이면 "접근/이탈" — 더 민감 */

/* PWM 듀티 (0~20000 PERIOD, term_code 기준) */
#define PWM_PERIOD      20000
#define V_MAX           16000   /* 직선 최대 속도 */
#define V_CRUISE        13000   /* 일반 순항 */
#define V_SLOW          9000    /* 신중 주행 */
#define V_TURN          17000   /* 제자리 회전 듀티 — 정지마찰 이기게 강하게 */
#define V_CREEP         8000    /* 정렬/탐색 저속 */

/* 시간(ms) — 20ms 틱 기준 */
#define CTRL_PERIOD_MS  20
#define SENS_PERIOD_MS  20
#define IR_PERIOD_MS    20
#define T_DEADLOCK_MS   2000    /* 정지·양보가 이만큼 안 풀리면 비대칭 우회 */
#define ALIGN_TIMEOUT   1500
#define SEARCH_TIMEOUT  3000

/* IR ADC 임계 (12bit raw, term_code 의 1000 보다 약간 보수적) */
#define IR_BUMP_HIGH    1100    /* 좌/우 IR 이 이 값 이상이면 충돌 직전 */

/* 안정도 분산 임계 */
#define VAR_UNSTABLE    20      /* 표본 표준편차 cm 가 이 이상이면 불안정 */

/* =========================================================================
 *  STM32 핸들 / 전역 (term_code 와 호환되는 이름 유지)
 * ========================================================================= */
TIM_HandleTypeDef    TimHandle1, TimHandle2, TimHandle3, TimHandle4;
TIM_IC_InitTypeDef   sICConfig;
TIM_OC_InitTypeDef   sConfig1, sConfig2, sConfig3;

uint32_t uwPrescalerValue = 0;
uint16_t motorInterrupt1 = 0;     /* 우측 엔코더 */
uint16_t motorInterrupt2 = 0;     /* 좌측 엔코더 */
uint8_t  encoder_right  = READY;
uint8_t  encoder_left   = READY;

uint32_t uwIC2Value1 = 0, uwIC2Value2 = 0, uwDiffCapture1 = 0; /* Right US */
uint32_t uwIC2Value3 = 0, uwIC2Value4 = 0, uwDiffCapture2 = 0; /* Forward US */
uint32_t uwIC2Value5 = 0, uwIC2Value6 = 0, uwDiffCapture3 = 0; /* Left US */
uint32_t uwFrequency = 0;

ADC_HandleTypeDef       AdcHandle1, AdcHandle2, AdcHandle3;
ADC_ChannelConfTypeDef  adcConfig1, adcConfig2, adcConfig3;

__IO uint32_t uhADCxRight;      /* Right IR raw */
__IO uint32_t uhADCxForward;    /* Floor IR raw */
__IO uint32_t uhADCxLeft;       /* Left IR raw */

extern UART_HandleTypeDef UartHandle1, UartHandle2;

/* =========================================================================
 *  주행 상태 / 공유 데이터
 * ========================================================================= */
typedef enum {
    ST_START = 0,
    ST_SCAN,
    ST_SEEK,
    ST_ALIGN,
    ST_WALL_FOLLOW,
    ST_REACQUIRE,
    ST_COR_ROTATE,           /* ③ 정적 코너 회전 */
    ST_COR_ADVANCE,
    ST_BACKUP_FOR_ROTATE,    /* 회전 사전 공간 확보 후진 */
    ST_AVOID_YIELD,          /* ② 정지·양보 */
    ST_AVOID_ROTATE,
    ST_AVOID_ADVANCE,
    ST_AVOID_BYPASS,         /* 교착 해제 비대칭 우회 */
    ST_SEARCH_ROTATE,        /* 벽 상실 폴백 */
    ST_EMERGENCY_STOP
} RobotState;

typedef struct {
    int dF, dL, dR;          /* 필터된 거리 [cm] */
    int dF_prev, dL_prev, dR_prev;
    int rateF, rateL, rateR; /* 1틱당 변화량 [cm], (-)는 접근 */
    int stdF, stdR, stdL;    /* 최근 N틱 표본 표준편차 [cm] */
    uint8_t failF, failR, failL; /* 무반향/튐 누적 카운트 */
    uint8_t bumpL, bumpR;    /* IR 범퍼 트리거 */
} SensorView;

static SensorView sv;

static RobotState  st = ST_START;
static uint8_t     follow_right = 1;   /* 1: 우측벽, 0: 좌측벽 */
static float       S = 0.0f;           /* 교차점 의심도 0~1 */

static int  seq_timer = 0;             /* 시퀀스 진행 ms 누계 */
static int  yield_timer = 0;
static int  search_timer = 0;
static int  align_timer = 0;
static int  bypass_cool = 0;
static int  pd_err_prev = 0;

/* =========================================================================
 *  printf → UART
 * ========================================================================= */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&UartHandle1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

/* =========================================================================
 *  유틸 / 모터 차동 구동
 * ========================================================================= */
static int clampi(int v, int lo, int hi){ return v<lo?lo:(v>hi?hi:v); }
static int absi(int v){ return v<0?-v:v; }

/* 모터 핀들을 항상 4채널 다 켜둔 채로 CCR 만 갱신해서 차동 출력
 *   v_left, v_right: -V_MAX ~ +V_MAX (양수=전진)               */
static void Motor_Drive(int v_left, int v_right)
{
    v_left  = clampi(v_left,  -V_MAX, V_MAX);
    v_right = clampi(v_right, -V_MAX, V_MAX);

    /* Right motor: TIM8 CH1 = forward, CH2 = backward */
    if (v_right >= 0) { TIM8->CCR1 = v_right; TIM8->CCR2 = 0; }
    else              { TIM8->CCR1 = 0;       TIM8->CCR2 = -v_right; }

    /* Left motor: TIM4 CH2 = forward, CH1 = backward */
    if (v_left >= 0)  { TIM4->CCR2 = v_left;  TIM4->CCR1 = 0; }
    else              { TIM4->CCR2 = 0;       TIM4->CCR1 = -v_left; }
}

static void Motor_AllStop(void){ Motor_Drive(0, 0); }

/* term_code 호환용 — 기본 채널들을 항상 살려둔 상태로 유지 */
void Motor_Forward (void){ Motor_Drive( V_CRUISE,  V_CRUISE); }
void Motor_Backward(void){ Motor_Drive(-V_CRUISE, -V_CRUISE); }
void Motor_Left    (void){ Motor_Drive(-V_TURN,    V_TURN ); } /* 좌회전 */
void Motor_Right   (void){ Motor_Drive( V_TURN,   -V_TURN ); } /* 우회전 */
void Motor_Stop    (void){ Motor_AllStop(); }

/* =========================================================================
 *  센서 태스크 — 중앙값 필터 + 변화율 + 안정도
 * ========================================================================= */
#define MED_N 5
static int  hist_F[MED_N], hist_R[MED_N], hist_L[MED_N];
static int  hist_idx = 0;

static int median5(int *a)
{
    int b[MED_N];
    int i, j;
    for (i = 0; i < MED_N; i++) b[i] = a[i];
    for (i = 0; i < MED_N - 1; i++)
        for (j = i + 1; j < MED_N; j++)
            if (b[i] > b[j]) { int t = b[i]; b[i] = b[j]; b[j] = t; }
    return b[MED_N / 2];
}

static int stddev5(int *a, int m)
{
    int i, acc = 0;
    for (i = 0; i < MED_N; i++) { int d = a[i] - m; acc += d * d; }
    /* 간이 sqrt: 0~400 범위에서 충분 */
    int s = acc / MED_N, r = 1;
    while (r * r < s) r++;
    return r;
}

void SensorTask(void)
{
    int i;
    /* 워밍업 */
    osDelay(300);

    for (i = 0; i < MED_N; i++) {
        hist_F[i] = uwDiffCapture2 / 58;
        hist_R[i] = uwDiffCapture1 / 58;
        hist_L[i] = uwDiffCapture3 / 58;
    }

    for (;;)
    {
        int rawF = uwDiffCapture2 / 58;
        int rawR = uwDiffCapture1 / 58;
        int rawL = uwDiffCapture3 / 58;

        /* 무반향/0/과대값 → 직전 유효값 유지 (실패 카운트 가산) */
        if (rawF <= 1 || rawF > 400) { rawF = sv.dF; sv.failF++; } else sv.failF = 0;
        if (rawR <= 1 || rawR > 400) { rawR = sv.dR; sv.failR++; } else sv.failR = 0;
        if (rawL <= 1 || rawL > 400) { rawL = sv.dL; sv.failL++; } else sv.failL = 0;

        hist_F[hist_idx] = rawF;
        hist_R[hist_idx] = rawR;
        hist_L[hist_idx] = rawL;
        hist_idx = (hist_idx + 1) % MED_N;

        int mF = median5(hist_F);
        int mR = median5(hist_R);
        int mL = median5(hist_L);

        sv.dF_prev = sv.dF; sv.dR_prev = sv.dR; sv.dL_prev = sv.dL;
        sv.dF = mF; sv.dR = mR; sv.dL = mL;

        sv.rateF = sv.dF - sv.dF_prev;
        sv.rateR = sv.dR - sv.dR_prev;
        sv.rateL = sv.dL - sv.dL_prev;

        sv.stdF = stddev5(hist_F, mF);
        sv.stdR = stddev5(hist_R, mR);
        sv.stdL = stddev5(hist_L, mL);

        osDelay(SENS_PERIOD_MS);
    }
}

/* =========================================================================
 *  IR 태스크 — 범퍼 / 바닥 라인
 * ========================================================================= */
void IR_Task(void)
{
    osDelay(200);
    for (;;)
    {
        HAL_ADC_Start(&AdcHandle1);
        HAL_ADC_PollForConversion(&AdcHandle1, 0xFF);
        uhADCxLeft = HAL_ADC_GetValue(&AdcHandle1);

        HAL_ADC_Start(&AdcHandle2);
        HAL_ADC_PollForConversion(&AdcHandle2, 0xFF);
        uhADCxRight = HAL_ADC_GetValue(&AdcHandle2);

        HAL_ADC_Start(&AdcHandle3);
        HAL_ADC_PollForConversion(&AdcHandle3, 0xFF);
        uhADCxForward = HAL_ADC_GetValue(&AdcHandle3);

        sv.bumpL = (uhADCxLeft  > IR_BUMP_HIGH) ? 1 : 0;
        sv.bumpR = (uhADCxRight > IR_BUMP_HIGH) ? 1 : 0;
        /* uhADCxForward (바닥 IR) 는 모니터링만, 자율판단에 사용하지 않음 */

        osDelay(IR_PERIOD_MS);
    }
}

/* =========================================================================
 *  S — 교차점 의심도 (0~1)
 * ========================================================================= */
static float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }

static void update_suspicion(void)
{
    float inc = 0.0f;
    int dInner = follow_right ? sv.dR : sv.dL;
    int dOuter = follow_right ? sv.dL : sv.dR;
    int rInner = follow_right ? sv.rateR : sv.rateL;
    int rOuter = follow_right ? sv.rateL : sv.rateR;
    int sInner = follow_right ? sv.stdR : sv.stdL;
    int sOuter = follow_right ? sv.stdL : sv.stdR;

    /* 1) 추종 벽 열림 / 급변 */
    if (dInner > D_OPEN)       inc += 0.25f;
    if (absi(rInner) >= MOVE_TH) inc += 0.15f;

    /* 2) 전방 막힘 */
    if (sv.dF < K_FRONT)       inc += 0.20f;
    if (sv.rateF <= -MOVE_TH)  inc += 0.10f;

    /* 3) 바깥쪽 갑작스런 변화 */
    if (absi(rOuter) >= MOVE_TH) inc += 0.10f;

    /* 4) 다방향 동시 변화(T자/십자) — 크게 가산 */
    int multi = 0;
    if (absi(sv.rateF) >= MOVE_TH) multi++;
    if (absi(sv.rateR) >= MOVE_TH) multi++;
    if (absi(sv.rateL) >= MOVE_TH) multi++;
    if (multi >= 2) inc += 0.20f;

    /* 5) 불안정(무반향/튐/분산) */
    if (sv.stdF >= VAR_UNSTABLE || sInner >= VAR_UNSTABLE || sOuter >= VAR_UNSTABLE)
        inc += 0.15f;
    if (sv.failF >= 2 || sv.failR >= 2 || sv.failL >= 2)
        inc += 0.20f;

    if (inc > 0.0f) {
        S = clampf(S + inc * 0.5f, 0.0f, 1.0f);   /* 가산은 천천히 */
    } else {
        /* 안정 유지 시 천천히 회복 */
        S = clampf(S - 0.05f, 0.0f, 1.0f);
    }
}

/* =========================================================================
 *  공통 안전 게이트 — 전진 직전 진행방향 여유 확인
 * ========================================================================= */
static int safe_to_advance(void){ return sv.dF >= D_MIN + 2 && !sv.bumpL && !sv.bumpR; }

/* =========================================================================
 *  ④ 벽 추종 — PD 차동 + S 캡
 * ========================================================================= */
static void wall_follow_step(void)
{
    int inner = follow_right ? sv.dR : sv.dL;
    int err   = inner - D_TARGET;
    int derr  = err - pd_err_prev;
    pd_err_prev = err;

    /* PD 게인 — 실측 후 튜닝 */
    int Kp = 200, Kd = 600;
    int steer = Kp * err + Kd * derr;       /* 우측 추종 시: +steer = 우회전(벽으로 붙기) */
    if (!follow_right) steer = -steer;
    steer = clampi(steer, -V_MAX, V_MAX);

    /* 기본 속도: S 가 클수록 느려진다 (5번 절 핵심) */
    int base = (int)(V_MAX * (1.0f - S));
    if (base < V_CREEP) base = V_CREEP;

    int vL = base + steer / 16;
    int vR = base - steer / 16;

    /* dR(또는 dL) 안전 하한 보호 */
    if (inner < D_MIN) {
        if (follow_right) { vL = V_SLOW; vR = V_SLOW / 2; }   /* 좌로 살짝 벗어남 */
        else              { vL = V_SLOW / 2; vR = V_SLOW; }
    }

    Motor_Drive(vL, vR);
}

/* =========================================================================
 *  ③ 코너 회전 시퀀스 진입 판정/실행
 * ========================================================================= */
static int rotate_target_left = 0;   /* 1=좌회전, 0=우회전 */

static int can_rotate_inplace(void)
{
    /* 회전 중 사방 긁힘 방지 */
    return (sv.dF >= R_CLEAR && sv.dR >= R_CLEAR && sv.dL >= R_CLEAR);
}

static void enter_corner_rotate(void)
{
    /* 더 트인 쪽 선택, 양쪽 모두 W_SAFE 미만이면 BACKUP 후 재시도 */
    int both_blocked = (sv.dL < W_SAFE && sv.dR < W_SAFE);

    if (both_blocked) {
        st = ST_BACKUP_FOR_ROTATE; seq_timer = 0;
        /* 후진 후 다시 enter_corner_rotate 호출됨 */
        rotate_target_left = (sv.dL >= sv.dR) ? 1 : 0;
        return;
    }

    /* 더 트인 쪽으로 회전 (W_SAFE 미달이라도 트인 쪽이면 시도) */
    rotate_target_left = (sv.dL > sv.dR) ? 1 : 0;
    st = ST_COR_ROTATE;
    seq_timer = 0;

    if (!can_rotate_inplace()) {
        st = ST_BACKUP_FOR_ROTATE;
        seq_timer = 0;
    }
}

/* =========================================================================
 *  ② 동적 회피 판정
 * ========================================================================= */
static int detect_moving_threat(void)
{
    /* 절대 거리 변화량(접근)이 임계 이상이면 동적 의심 */
    if (sv.rateF <= -MOVE_TH && sv.dF < K_FRONT + 6) return 1;
    if (sv.rateL <= -MOVE_TH && sv.dL < W_SAFE + 4)  return 2;
    if (sv.rateR <= -MOVE_TH && sv.dR < W_SAFE + 4)  return 3;
    return 0;
}

/* =========================================================================
 *  최종 속도 캡 — 모든 출력 뒤에 S 로 한 번 더 누름
 * ========================================================================= */
static int is_rotating_state(void)
{
    return (st == ST_COR_ROTATE   || st == ST_AVOID_ROTATE ||
            st == ST_SEARCH_ROTATE || st == ST_BACKUP_FOR_ROTATE);
}

static void apply_S_cap(void)
{
    /* 회전/후진 시퀀스 동안은 토크 유지 — 캡 면제 */
    if (is_rotating_state()) return;
    if (S <= 0.0f) return;
    float scale = 1.0f - 0.5f * S;     /* S=1 이면 절반 이하로 */
    if (scale < 0.3f) scale = 0.3f;
    TIM8->CCR1 = (uint32_t)(TIM8->CCR1 * scale);
    TIM8->CCR2 = (uint32_t)(TIM8->CCR2 * scale);
    TIM4->CCR1 = (uint32_t)(TIM4->CCR1 * scale);
    TIM4->CCR2 = (uint32_t)(TIM4->CCR2 * scale);
}

/* =========================================================================
 *  제어 태스크 — 매 20ms, 상위 게이트 → 시퀀스 → 모드 분기
 * ========================================================================= */
void ControlTask(void)
{
    osDelay(400);  /* 센서 안정화 */

    st = ST_SCAN;
    seq_timer = 0;
    printf("\r\n[CTRL] start\r\n");

    for (;;)
    {
        update_suspicion();

        /* ===== ① 비상정지 (최우선) ===== */
        if (sv.bumpL || sv.bumpR || sv.dF < EMG_FRONT) {
            Motor_AllStop();
            st = ST_EMERGENCY_STOP;
            seq_timer = 0;
            osDelay(CTRL_PERIOD_MS);
            continue;
        }
        if (st == ST_EMERGENCY_STOP) {
            /* 위험 해소 + 안정 N틱 이후 복귀 */
            seq_timer += CTRL_PERIOD_MS;
            if (!sv.bumpL && !sv.bumpR && sv.dF >= EMG_FRONT + 4 && seq_timer >= 200) {
                st = ST_WALL_FOLLOW;
                pd_err_prev = 0;
            }
            osDelay(CTRL_PERIOD_MS);
            continue;
        }

        /* ===== 상태별 처리 ===== */
        switch (st)
        {
        /* --- 7번 절: 시작 단계 --------------------------------------- */
        case ST_SCAN: {
            Motor_AllStop();
            seq_timer += CTRL_PERIOD_MS;
            if (seq_timer >= 400) {                /* 잠시 멈춰 안정값 수집 */
                if (sv.dR < D_OPEN)      { follow_right = 1; st = ST_ALIGN; align_timer = 0; }
                else if (sv.dL < D_OPEN) { follow_right = 0; st = ST_ALIGN; align_timer = 0; }
                else                      { st = ST_SEEK; }
                seq_timer = 0;
                printf("[SCAN→%s] R=%d L=%d F=%d\r\n",
                       st==ST_ALIGN?"ALIGN":"SEEK", sv.dR, sv.dL, sv.dF);
            }
        } break;

        case ST_SEEK: {
            /* 양쪽 다 멀면 저속 직진하며 벽 출현 대기 */
            Motor_Drive(V_CREEP, V_CREEP);
            if (sv.dR < D_OPEN) { follow_right = 1; st = ST_ALIGN; align_timer = 0; }
            else if (sv.dL < D_OPEN) { follow_right = 0; st = ST_ALIGN; align_timer = 0; }
            if (sv.dF < K_FRONT) { st = ST_COR_ROTATE; enter_corner_rotate(); }
        } break;

        case ST_ALIGN: {
            /* 정한 벽으로 비스듬히 합류 — ② 측면 급감 OFF, ① 만 유지 */
            int inner = follow_right ? sv.dR : sv.dL;
            int err   = inner - D_TARGET;
            int turn  = clampi(err * 80, -V_TURN, V_TURN);
            if (!follow_right) turn = -turn;
            Motor_Drive(V_CREEP + turn / 8, V_CREEP - turn / 8);

            align_timer += CTRL_PERIOD_MS;
            if (absi(err) <= 3 || align_timer >= ALIGN_TIMEOUT) {
                st = ST_WALL_FOLLOW;
                pd_err_prev = 0;
                printf("[ALIGN→FOLLOW] inner=%d\r\n", inner);
            }
        } break;

        /* --- 진행 중인 시퀀스 ---------------------------------------- */
        case ST_BACKUP_FOR_ROTATE: {
            Motor_Drive(-V_SLOW, -V_SLOW);
            seq_timer += CTRL_PERIOD_MS;
            if (seq_timer >= 250 && can_rotate_inplace()) {
                st = ST_COR_ROTATE;
                seq_timer = 0;
            } else if (seq_timer >= 800) {
                /* 너무 오래 후진하면 포기하고 양보 */
                st = ST_AVOID_YIELD; yield_timer = 0;
            }
        } break;

        case ST_COR_ROTATE: {
            if (rotate_target_left) Motor_Drive(-V_TURN,  V_TURN);
            else                    Motor_Drive( V_TURN, -V_TURN);
            seq_timer += CTRL_PERIOD_MS;
            /* 종료조건: 전방 트임 + 좌우 여유 OK (안정 유지 시간 짧게) */
            if (sv.dF >= K_FRONT + 2 && sv.dR >= W_SAFE && sv.dL >= W_SAFE) {
                if (seq_timer >= 60) {             /* 안정 유지 60ms */
                    st = ST_COR_ADVANCE; seq_timer = 0;
                }
            } else if (seq_timer >= 1500) {        /* 안전 타임아웃 */
                st = ST_AVOID_YIELD; yield_timer = 0;
            }
        } break;

        case ST_COR_ADVANCE: {
            Motor_Drive(V_SLOW, V_SLOW);
            seq_timer += CTRL_PERIOD_MS;
            /* 측면이 다시 안정 벽이 되거나, 일정 시간 전진하면 종료 */
            int inner = follow_right ? sv.dR : sv.dL;
            if ((inner < D_OPEN && absi(sv.rateR) < MOVE_TH && absi(sv.rateL) < MOVE_TH)
                || seq_timer >= 600) {
                st = (inner < D_OPEN) ? ST_WALL_FOLLOW : ST_REACQUIRE;
                pd_err_prev = 0;
                seq_timer = 0;
            }
            if (sv.dF < K_FRONT) { st = ST_COR_ROTATE; enter_corner_rotate(); }
        } break;

        case ST_AVOID_YIELD: {
            Motor_AllStop();
            yield_timer += CTRL_PERIOD_MS;
            int threat = detect_moving_threat();
            if (!threat && sv.dF >= K_FRONT + 2) {
                st = ST_WALL_FOLLOW;
                pd_err_prev = 0;
            } else if (yield_timer >= T_DEADLOCK_MS && bypass_cool <= 0) {
                /* 교착 해제 — 약간 랜덤 대기 후 비대칭 우회 */
                osDelay((rand() % 200) + 50);
                st = ST_AVOID_BYPASS;
                seq_timer = 0;
                bypass_cool = 4000;
            }
        } break;

        case ST_AVOID_BYPASS: {
            /* 우측 우선 비대칭 우회: 약간 우회전 + 전진 */
            if (sv.dR >= W_SAFE)  Motor_Drive(V_SLOW, V_SLOW / 2);
            else                  Motor_Drive(V_SLOW / 2, V_SLOW);
            seq_timer += CTRL_PERIOD_MS;
            if (seq_timer >= 500 || sv.dF < K_FRONT) {
                st = ST_WALL_FOLLOW;
                pd_err_prev = 0;
            }
        } break;

        case ST_AVOID_ROTATE: {
            if (rotate_target_left) Motor_Drive(-V_TURN,  V_TURN);
            else                    Motor_Drive( V_TURN, -V_TURN);
            seq_timer += CTRL_PERIOD_MS;
            if (sv.dF >= K_FRONT + 4 || seq_timer >= 800) {
                st = ST_AVOID_ADVANCE; seq_timer = 0;
            }
        } break;

        case ST_AVOID_ADVANCE: {
            Motor_Drive(V_SLOW, V_SLOW);
            seq_timer += CTRL_PERIOD_MS;
            if (seq_timer >= 300 || sv.dF < K_FRONT) {
                st = ST_WALL_FOLLOW;
                pd_err_prev = 0;
            }
        } break;

        case ST_REACQUIRE: {
            /* 우측 벽 놓침 → 완만한 호로 우측(or 좌측)으로 */
            int inner = follow_right ? sv.dR : sv.dL;
            if (inner < D_OPEN) {
                st = ST_WALL_FOLLOW; pd_err_prev = 0;
                search_timer = 0;
            } else {
                if (follow_right) Motor_Drive(V_SLOW + 1500, V_SLOW);
                else              Motor_Drive(V_SLOW, V_SLOW + 1500);
                search_timer += CTRL_PERIOD_MS;
                if (search_timer >= SEARCH_TIMEOUT) {
                    st = ST_SEARCH_ROTATE;     /* 벽 상실 폴백 */
                    seq_timer = 0;
                }
            }
            if (sv.dF < K_FRONT) { st = ST_COR_ROTATE; enter_corner_rotate(); }
        } break;

        case ST_SEARCH_ROTATE: {
            /* 저속·고 S 상태로 제자리 탐색 회전, 어느 벽이든 잡으면 추종 재개 */
            S = clampf(S + 0.05f, 0.5f, 1.0f);
            Motor_Drive(-V_CREEP, V_CREEP);    /* 좌로 회전 */
            seq_timer += CTRL_PERIOD_MS;
            if (sv.dR < D_OPEN) { follow_right = 1; st = ST_WALL_FOLLOW; pd_err_prev = 0; }
            else if (sv.dL < D_OPEN) { follow_right = 0; st = ST_WALL_FOLLOW; pd_err_prev = 0; }
            else if (seq_timer >= 4000) {
                /* 한 바퀴 돌아도 못 찾으면 정지 — 폭주 금지 */
                Motor_AllStop();
                st = ST_AVOID_YIELD; yield_timer = 0;
            }
        } break;

        /* --- 기본 모드: 벽 추종 ------------------------------------- */
        case ST_WALL_FOLLOW:
        default: {
            int inner = follow_right ? sv.dR : sv.dL;

            /* ② 동적 회피 우선 */
            int threat = detect_moving_threat();
            if (threat == 1) { st = ST_AVOID_YIELD; yield_timer = 0; break; }
            if (threat == 2 && follow_right) {     /* 좌측 침입자 → 우측으로 */
                rotate_target_left = 0;
                if (sv.dR >= W_SAFE) { st = ST_AVOID_ROTATE; seq_timer = 0; }
                else                  { st = ST_AVOID_YIELD; yield_timer = 0; }
                break;
            }
            if (threat == 3 && !follow_right) {
                rotate_target_left = 1;
                if (sv.dL >= W_SAFE) { st = ST_AVOID_ROTATE; seq_timer = 0; }
                else                  { st = ST_AVOID_YIELD; yield_timer = 0; }
                break;
            }
            if (threat) {                          /* 추종 벽쪽 급감 → 감속·확인 */
                Motor_Drive(V_CREEP, V_CREEP);
                break;
            }

            /* ③ 전방 임계 → 코너 회전 */
            if (sv.dF < K_FRONT) {
                enter_corner_rotate();
                break;
            }

            /* 추종 벽 상실 → REACQUIRE */
            if (inner > D_OPEN) {
                st = ST_REACQUIRE;
                search_timer = 0;
                break;
            }

            /* ④ 벽 추종 */
            wall_follow_step();
        } break;
        } /* switch */

        /* 최종 S 캡 (안전이 마지막에 속도를 누른다) */
        apply_S_cap();
        if (bypass_cool > 0) bypass_cool -= CTRL_PERIOD_MS;

        osDelay(CTRL_PERIOD_MS);
    }
}

/* =========================================================================
 *  디버그 출력 태스크
 * ========================================================================= */
void DebugTask(void)
{
    osDelay(800);
    const char *names[] = {
        "START","SCAN","SEEK","ALIGN","FOLLOW","REACQ","COR_ROT","COR_ADV",
        "BACKUP","YIELD","AV_ROT","AV_ADV","BYPASS","SEARCH","EMG"
    };
    for (;;) {
        printf("[%s] F=%3d L=%3d R=%3d S=%d.%02d IR(L%4d R%4d F%4d)\r\n",
               names[st], sv.dF, sv.dL, sv.dR,
               (int)S, (int)(S * 100) % 100,
               (int)uhADCxLeft, (int)uhADCxRight, (int)uhADCxForward);
        osDelay(300);
    }
}

/* =========================================================================
 *  HW 초기화 / main
 * ========================================================================= */
static void SystemClock_Config(void);
static void EXTILine_Config(void);
static void Error_Handler(void);

int main(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    HAL_Init();
    SystemClock_Config();
    BSP_COM1_Init();

    /***** 모터 *****/
    uwPrescalerValue = (SystemCoreClock / 2) / 1000000;

    __GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);   /* MC_EN */

    sConfig1.OCMode     = TIM_OCMODE_PWM1;
    sConfig1.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig1.OCFastMode = TIM_OCFAST_DISABLE;
    sConfig1.Pulse      = 0;

    TimHandle1.Instance               = TIM8;
    TimHandle1.Init.Prescaler         = uwPrescalerValue;
    TimHandle1.Init.Period            = PWM_PERIOD;
    TimHandle1.Init.ClockDivision     = 0;
    TimHandle1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&TimHandle1);
    HAL_TIM_PWM_ConfigChannel(&TimHandle1, &sConfig1, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&TimHandle1, &sConfig1, TIM_CHANNEL_2);

    sConfig2 = sConfig1;
    TimHandle2.Instance               = TIM4;
    TimHandle2.Init.Prescaler         = uwPrescalerValue;
    TimHandle2.Init.Period            = PWM_PERIOD;
    TimHandle2.Init.ClockDivision     = 0;
    TimHandle2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&TimHandle2);
    HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig2, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig2, TIM_CHANNEL_2);

    /* 4채널 PWM 모두 상시 가동 — CCR 만 0/값 으로 토글하여 차동 출력 */
    HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_2);
    TIM8->CCR1 = TIM8->CCR2 = 0;
    TIM4->CCR1 = TIM4->CCR2 = 0;

    EXTILine_Config();

    /***** 초음파 *****/
    uwPrescalerValue = ((SystemCoreClock / 2) / 1000000) - 1;
    TimHandle3.Instance               = TIM3;
    TimHandle3.Init.Period            = 0xFFFF;
    TimHandle3.Init.Prescaler         = uwPrescalerValue;
    TimHandle3.Init.ClockDivision     = 0;
    TimHandle3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    if (HAL_TIM_IC_Init(&TimHandle3) != HAL_OK) Error_Handler();

    sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
    sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
    sICConfig.ICFilter    = 0;
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_1);
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_2);
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_3);
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_4);
    HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_4);

    /* 초음파 트리거 PWM (TIM10) */
    uwPrescalerValue = (SystemCoreClock / 2 / 131099) - 1;
    TimHandle4.Instance               = TIM10;
    TimHandle4.Init.Prescaler         = uwPrescalerValue;
    TimHandle4.Init.Period            = 0xFFFF;
    TimHandle4.Init.ClockDivision     = 0;
    TimHandle4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&TimHandle4);

    sConfig3.OCMode     = TIM_OCMODE_PWM1;
    sConfig3.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig3.OCFastMode = TIM_OCFAST_DISABLE;
    sConfig3.Pulse      = 2;
    HAL_TIM_PWM_ConfigChannel(&TimHandle4, &sConfig3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TimHandle4, TIM_CHANNEL_1);

    /***** 적외선 (ADC1/2/3) *****/
    AdcHandle1.Instance = ADC3;
    AdcHandle1.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
    AdcHandle1.Init.Resolution            = ADC_RESOLUTION12b;
    AdcHandle1.Init.ScanConvMode          = DISABLE;
    AdcHandle1.Init.ContinuousConvMode    = DISABLE;
    AdcHandle1.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle1.Init.NbrOfDiscConversion   = 0;
    AdcHandle1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    AdcHandle1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle1.Init.NbrOfConversion       = 1;
    AdcHandle1.Init.DMAContinuousRequests = DISABLE;
    AdcHandle1.Init.EOCSelection          = DISABLE;
    HAL_ADC_Init(&AdcHandle1);
    adcConfig1.Channel      = ADC_CHANNEL_11;
    adcConfig1.Rank         = 1;
    adcConfig1.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adcConfig1.Offset       = 0;
    HAL_ADC_ConfigChannel(&AdcHandle1, &adcConfig1);

    AdcHandle2.Instance                   = ADC2;
    AdcHandle2.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
    AdcHandle2.Init.Resolution            = ADC_RESOLUTION12b;
    AdcHandle2.Init.ScanConvMode          = DISABLE;
    AdcHandle2.Init.ContinuousConvMode    = DISABLE;
    AdcHandle2.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle2.Init.NbrOfDiscConversion   = 0;
    AdcHandle2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle2.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    AdcHandle2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle2.Init.NbrOfConversion       = 1;
    AdcHandle2.Init.DMAContinuousRequests = DISABLE;
    AdcHandle2.Init.EOCSelection          = DISABLE;
    HAL_ADC_Init(&AdcHandle2);
    adcConfig2.Channel      = ADC_CHANNEL_14;
    adcConfig2.Rank         = 1;
    adcConfig2.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adcConfig2.Offset       = 0;
    HAL_ADC_ConfigChannel(&AdcHandle2, &adcConfig2);

    AdcHandle3.Instance                   = ADC1;
    AdcHandle3.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
    AdcHandle3.Init.Resolution            = ADC_RESOLUTION12b;
    AdcHandle3.Init.ScanConvMode          = DISABLE;
    AdcHandle3.Init.ContinuousConvMode    = DISABLE;
    AdcHandle3.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle3.Init.NbrOfDiscConversion   = 0;
    AdcHandle3.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle3.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
    AdcHandle3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle3.Init.NbrOfConversion       = 1;
    AdcHandle3.Init.DMAContinuousRequests = DISABLE;
    AdcHandle3.Init.EOCSelection          = DISABLE;
    HAL_ADC_Init(&AdcHandle3);
    adcConfig3.Channel      = ADC_CHANNEL_15;
    adcConfig3.Rank         = 1;
    adcConfig3.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adcConfig3.Offset       = 0;
    HAL_ADC_ConfigChannel(&AdcHandle3, &adcConfig3);

    /***** Task 생성 *****/
    xTaskCreate(SensorTask,  "sensor",  512, NULL, 3, NULL);
    xTaskCreate(IR_Task,     "ir",      512, NULL, 3, NULL);
    xTaskCreate(ControlTask, "control", 1024, NULL, 2, NULL);
    xTaskCreate(DebugTask,   "debug",   512, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1) { }
}

/* =========================================================================
 *  EXTI / IC Callback / 클럭
 * ========================================================================= */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
    case GPIO_PIN_15:
        encoder_right = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
        if      (encoder_right == 0) { motorInterrupt1++; encoder_right = READY; }
        else if (encoder_right == 1) { motorInterrupt1--; encoder_right = READY; }
        break;
    case GPIO_PIN_4:
        encoder_left = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
        if      (encoder_left == 0) { motorInterrupt2++; encoder_left = READY; }
        else if (encoder_left == 1) { motorInterrupt2--; encoder_left = READY; }
        break;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        if ((TIM3->CCER & TIM_CCER_CC2P) == 0) {
            uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            TIM3->CCER |= TIM_CCER_CC2P;
        } else {
            uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            uwDiffCapture1 = (uwIC2Value2 > uwIC2Value1)
                ? (uwIC2Value2 - uwIC2Value1)
                : ((0xFFFF - uwIC2Value1) + uwIC2Value2);
            TIM3->CCER &= ~TIM_CCER_CC2P;
        }
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        if ((TIM3->CCER & TIM_CCER_CC3P) == 0) {
            uwIC2Value3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            TIM3->CCER |= TIM_CCER_CC3P;
        } else {
            uwIC2Value4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            uwDiffCapture2 = (uwIC2Value4 > uwIC2Value3)
                ? (uwIC2Value4 - uwIC2Value3)
                : ((0xFFFF - uwIC2Value3) + uwIC2Value4);
            TIM3->CCER &= ~TIM_CCER_CC3P;
        }
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
        if ((TIM3->CCER & TIM_CCER_CC4P) == 0) {
            uwIC2Value5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            TIM3->CCER |= TIM_CCER_CC4P;
        } else {
            uwIC2Value6 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            uwDiffCapture3 = (uwIC2Value6 > uwIC2Value5)
                ? (uwIC2Value6 - uwIC2Value5)
                : ((0xFFFF - uwIC2Value5) + uwIC2Value6);
            TIM3->CCER &= ~TIM_CCER_CC4P;
        }
    }
}

static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;
    __PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    HAL_PWREx_ActivateOverDrive();
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                   RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

static void EXTILine_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    __GPIOA_CLK_ENABLE();
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin  = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    __GPIOB_CLK_ENABLE();
    GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

static void Error_Handler(void)
{
    BSP_LED_On(LED3);
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { while (1) { } }
#endif

/* prototypes from term_code 호환 */
void Motor_Speed_Up_Config(void)   { /* 차동 PWM 에선 사용 안 함 */ }
void Motor_Speed_Down_Config(void) { /* 차동 PWM 에선 사용 안 함 */ }
