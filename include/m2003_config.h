/*
 * M2003 Hardware Configuration (Pin-Compatible with MS51 Board)
 *
 * This header maps the physical pins of the BL4818 board to M2003 peripherals.
 */

#ifndef M2003_CONFIG_H
#define M2003_CONFIG_H

/* CPU Clock */
#define CLK_HIRC_24M        24000000UL

/* ── PWM Gate Drives (EPWM0) ────────────────────────────────────────────── */
/* EPWM0 channels 0-5 map to the 6 MOSFET gates. */
/* M2003 TSSOP20 pin mapping: */
#define PIN_PWM_U_LOW       PB13      /* PB.13, Pin 13, EPWM0_CH0 */
#define PIN_PWM_U_HIGH      PB12      /* PB.12, Pin 14, EPWM0_CH1 */
#define PIN_PWM_V_HIGH      PB7       /* PB.7, Pin 15, EPWM0_CH2 */
#define PIN_PWM_V_LOW       PB8      /* PB.8, Pin 16, EPWM0_CH3 */
#define PIN_PWM_W_LOW       PB9      /* PB.9, Pin 17, EPWM0_CH4 */
#define PIN_PWM_W_HIGH      PB11      /* PB.11, Pin 19, EPWM0_CH5 */

/* PWM Scaling */
#define PWM_MAX_DUTY        1200    /* 24MHz / 20kHz */

/* ── UART Ring (UART1) ─────────────────────────────────────────────────── */
#define PIN_UART1_TX        PF0     /* Pin 8  */
#define PIN_UART1_RX        PF1     /* Pin 18 */

/* ── ADC Sensing ───────────────────────────────────────────────────────── */
#define ADC_CH_CURRENT      2       /* PB.2, Pin 2 */
#define ADC_CH_VOLTAGE      3       /* PB.3, Pin 3 */

/* ── SSI Encoder (Bit-Banged) ──────────────────────────────────────────── */
#define PIN_SSI_CLK         PB0       /* PB.0, Pin 20 (direct drive) */
#define PIN_SSI_CSN         PB1       /* PB.1, Pin 1 (inverted via N-FET) */
#define PIN_SSI_DAT         PB15      /* PB.15, Pin 11 */

/* ── Commutation ──────────────────────────────────────────────────────── */
#define COMMUTATION_OFFSET  1       /* Advance commutation by 1 sector (matches MS51 tuning) */

/* ── Motor Parameters ─────────────────────────────────────────────────── */
#define MOTOR_POLE_PAIRS    5       /* 10-pole, 12 slot outrunner */
#define HALL_TRANSITIONS_PER_REV (6 * MOTOR_POLE_PAIRS)  /* 30 */
#define ENCODER_COUNTS_PER_REV  16384   /* 14-bit absolute encoder */

/* ── Control Settings ──────────────────────────────────────────────────── */
#define CONTROL_LOOP_HZ     10000
#define PWM_FREQ_HZ         20000
#define PWM_PERIOD          (CLK_HIRC_24M / PWM_FREQ_HZ)
#define DEFAULT_TORQUE_LIMIT_MA 1500
#define CURRENT_LIMIT_MA        5000    /* sustained current limit and max torque-limit setting */
#define CURRENT_PEAK_LIMIT_MA   7000    /* immediate fault ceiling for truly excessive spikes */
#define CURRENT_FILTER_SHIFT    2       /* IIR alpha = 1/(1<<N) for reported/clamped current */
#define OVERCURRENT_FAULT_TICKS 20      /* consecutive 10 kHz control ticks above CURRENT_LIMIT_MA */

/* Velocity PID defaults (Q8 — divide by 256 for real value) */
#define VEL_PID_KP_DEFAULT  1536     /* 0.50 */
#define VEL_PID_KI_DEFAULT  30     /* 0.0625 */
#define VEL_PID_KD_DEFAULT  30       /* off */
#define VEL_FF_DEFAULT      88       /* feedforward tuned */
#define PID_D_FILTER_SHIFT  2       /* D-term IIR alpha = 1/(1<<N): 2→1/4, 3→1/8 */
#define VEL_FILTER_SHIFT    3       /* IIR alpha = 1/(1<<N): 2→1/4, 3→1/8 */

/* Position PID defaults (Q8 — output is velocity RPM)
 * Error is prescaled by POS_ERROR_PRESCALE for finer gain resolution.
 * Effective gain = Q8_value / 256 / POS_ERROR_PRESCALE RPM per encoder count */
#define POS_ERROR_PRESCALE  16      /* divide position error by 16 before PID */
#define POS_PID_KP_DEFAULT  1200     /* ≈0.0625 RPM/count (same as old 16 w/o prescale) */
#define POS_PID_KI_DEFAULT  20
#define POS_PID_KD_DEFAULT  30
#define POS_MAX_VEL_RPM     2000    /* position loop velocity clamp */
#define POS_INT_MAX_RPM     8000    /* position integral velocity clamp (RPM) */
#define POS_LOOP_DIVIDER    (CONTROL_LOOP_HZ / 2000)  /* = 5 → 1 kHz */

/* ── Strike Defaults ──────────────────────────────────────────────────── */
#define STRIKE_HOME_OFFSET_DEFAULT      1024    /* encoder counts above drum surface */
#define STRIKE_COAST_DISTANCE_DEFAULT   256     /* cut power this far from drum (counts) */
#define STRIKE_HOMING_DUTY_DEFAULT      100  /* low duty toward drum (sign = toward drum) */
#define STRIKE_RETURN_VELOCITY_RPM      1000    /* aggressive closed-loop return speed toward home */
#define STRIKE_RETURN_RAMP_RATE         800     /* RPM per strike tick (1 kHz) toward return target */
#define STRIKE_CATCH_ENTRY_WINDOW       512     /* counts: switch from return velocity to position hold */
#define STRIKE_MAX_REBOUND_OVERSHOOT    128     /* counts beyond home allowed before forced recapture */
#define STRIKE_RETRIGGER_DEADBAND       512     /* counts: must be this close to home for predictable retrigger */
#define STRIKE_RETRIGGER_VEL_THRESHOLD  512     /* RPM: must be this slow near home for predictable retrigger */
#define STRIKE_SETTLE_DEADBAND          200      /* counts: position "settled" threshold */
#define STRIKE_SETTLE_TICKS             20     /* 25 ms at 1 kHz */
#define STRIKE_HOMING_STALL_TICKS       200     /* 200 ms no movement = drum contact */
#define STRIKE_HOMING_STALL_THRESHOLD   4       /* counts: less than this = stalled */
#define STRIKE_COAST_TIMEOUT_TICKS      500     /* 500 ms max coast before forced catch */
#define STRIKE_REBOUND_THRESHOLD        5       /* RPM away from drum to confirm rebound */
#define STRIKE_TICK_DIVIDER             10       /* 8 kHz → 1 kHz for strike state machine */

#endif /* M2003_CONFIG_H */
