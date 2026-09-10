#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

/* Motor-specific settings. Update this value when the motor is changed. */
#define FOC_MOTOR_POLE_PAIRS         7u
#define FOC_ENCODER_DIRECTION       (-1)
/* Measured alignment offset: transformed encoder angle - software angle
 * averaged +586.27 mrad in the supplied synchronized capture. */
#define FOC_ENCODER_OFFSET_RAD        (-0.5863f)
#define FOC_VF_TARGET_RPM             100.0f

/* Set to 1 for standalone V/F open-loop (encoder calibration mode). When
 * enabled the EtherCAT CiA402 motion controller no longer overrides the V/F
 * target speed or output enable: the drive spins at FOC_VF_TARGET_RPM on its
 * own, and both the software-simulated and magnetic-encoder electrical angles
 * are streamed over USART3 for alignment calibration. Set to 0 to hand control
 * back to the EtherCAT master. */
#define FOC_VF_STANDALONE_OPEN_LOOP   1u

/* Hardware conversion and diagnostics settings. */
#define FOC_BUS_VOLTAGE_SCALE         ((3.3f / 4095.0f) * 18.727272f)
#define FOC_ADC_VOLTAGE_PER_COUNT     (3.3f / 4095.0f)
#define FOC_CURRENT_OFFSET_V           1.65f
#define FOC_CURRENT_A_PER_V            50.0f
#define FOC_PHASE_VOLTAGE_SCALE        16.0f
#define FOC_DEFAULT_BUS_VOLTAGE       16.0f
#define FOC_PRINT_PERIOD_MS           10u

#endif
 
