#ifndef PLANT_PARAMS_H
#define PLANT_PARAMS_H

/*
 * plant_params.h - yaw plant parameters shared by the simulator and the
 * control design (WBS M1.1). Equations and measurement methods are in
 * docs/plant_model.md; keep the two files in step.
 *
 * Units: rad, rad/s, s, N.m, kg.m^2.
 * Signs: yaw, body rate and wheel speed positive counter-clockwise seen from
 * above. Wheel speed is RELATIVE to the body. Positive motor torque speeds
 * the wheel up and pushes the body towards negative yaw.
 *
 * Status: ESTIMATES copied from v5/teste.c (12 Nov 2026). Nothing has been
 * measured on the real cube yet. Portable C99, no other includes needed.
 */

#define PLANT_PARAMS_VERSION      "1.0"

/* ---- Inertias ---- */
#define PLANT_J_BODY              0.050f    /* kg.m^2  body yaw inertia without the wheel (estimate) */
#define PLANT_J_WHEEL             0.0010f   /* kg.m^2  wheel + rotor about the spin axis (estimate) */

/* ---- Gimbal yaw bearing friction ---- */
#define PLANT_FRIC_VISCOUS        0.002f    /* N.m.s/rad  (estimate) */
#define PLANT_FRIC_COULOMB        0.002f    /* N.m        (estimate) */
#define PLANT_FRIC_STICTION       0.003f    /* N.m  breakaway torque; set equal to COULOMB to disable (assumed) */
#define PLANT_STICK_RATE          1.0e-4f   /* rad/s  below this the body is treated as stuck (numerical) */

/* ---- Gravity torque from imbalance: tau_g = -K * sin(yaw - YAW0) ---- */
#define PLANT_GRAVITY_K           0.0f      /* N.m  0 = perfectly balanced; sweep 0 to 0.005 */
#define PLANT_GRAVITY_YAW0        0.0f      /* rad  yaw at which the heavy side hangs lowest */

/* ---- Wheel actuator (moteus r4.11 + mj5208) ---- */
#define PLANT_TORQUE_MAX          0.2f      /* N.m    same as MAX_TORQUE_COMMAND */
#define PLANT_WHEEL_SPEED_MAX     314.16f   /* rad/s  3000 rpm, same as MAX_WHEEL_SPEED_RAD_S */
#define PLANT_WHEEL_SPEED_SOFT    251.33f   /* rad/s  2400 rpm, same as SOFT_WHEEL_SPEED_RAD_S */
#define PLANT_WHEEL_TAU           0.02f     /* s      velocity-loop time constant (guess, measure on the bench) */
#define PLANT_WHEEL_DRAG          0.0f      /* N.m.s/rad  wheel bearing drag, torque mode only */
#define PLANT_CMD_DELAY           0.0f      /* s      CAN-FD + moteus command delay */
#define PLANT_MOTOR_KT            0.029f    /* N.m/A  from Kv 330; check against the moteus calibration */

/* ---- Derived (do not edit) ---- */
#define PLANT_J_TOTAL             (PLANT_J_BODY + PLANT_J_WHEEL)
#define PLANT_J_REDUCED           ((PLANT_J_BODY * PLANT_J_WHEEL) / PLANT_J_TOTAL)
#define PLANT_WHEEL_KV            (PLANT_J_REDUCED / PLANT_WHEEL_TAU)   /* N.m.s/rad */
#define PLANT_RATE_PER_WHEEL      (PLANT_J_WHEEL / PLANT_J_TOTAL)       /* body rate change per unit wheel speed change, magnitude */
#define PLANT_WHEEL_PER_RATE      (PLANT_J_TOTAL / PLANT_J_WHEEL)       /* rate-loop feed-forward gain, magnitude */

#endif /* PLANT_PARAMS_H */
