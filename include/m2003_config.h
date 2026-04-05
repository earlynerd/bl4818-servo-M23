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
#define COMMUTATION_OFFSET  0       /* Advance commutation by 1 sector (matches MS51 tuning) */

/* ── Motor Parameters ─────────────────────────────────────────────────── */
#define MOTOR_POLE_PAIRS    7       /* 14-pole outrunner (typical massage gun motor) */
#define HALL_TRANSITIONS_PER_REV (6 * MOTOR_POLE_PAIRS)  /* 42 */
#define ENCODER_COUNTS_PER_REV  16384   /* 14-bit absolute encoder */

/* ── Control Settings ──────────────────────────────────────────────────── */
#define CONTROL_LOOP_HZ     5000
#define PWM_FREQ_HZ         20000
#define PWM_PERIOD          (CLK_HIRC_24M / PWM_FREQ_HZ)

/* Velocity PID defaults (Q8 — divide by 256 for real value) */
#define VEL_PID_KP_DEFAULT  128     /* 0.50 */
#define VEL_PID_KI_DEFAULT  8      /* 0.0625 */
#define VEL_PID_KD_DEFAULT  0       /* off */
#define VEL_FF_DEFAULT      0       /* feedforward off until tuned */
#define VEL_FILTER_SHIFT    2       /* IIR alpha = 1/(1<<N): 2→1/4, 3→1/8 */

#endif /* M2003_CONFIG_H */
