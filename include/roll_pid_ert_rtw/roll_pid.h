/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: roll_pid.h
 *
 * Code generated for Simulink model 'roll_pid'.
 *
 * Model version                  : 1.57
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Tue Apr 23 06:47:57 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_roll_pid_h_
#define RTW_HEADER_roll_pid_h_
#ifndef roll_pid_COMMON_INCLUDES_
#define roll_pid_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* roll_pid_COMMON_INCLUDES_ */

#include "roll_pid_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Integrator_DSTATE;            /* '<S33>/Integrator' */
  real_T Filter_DSTATE;                /* '<S28>/Filter' */
} DW_roll_pid_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T u;                            /* '<Root>/u' */
} ExtU_roll_pid_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T y;                            /* '<Root>/y' */
} ExtY_roll_pid_T;

/* Real-time Model Data Structure */
struct tag_RTM_roll_pid_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_roll_pid_T roll_pid_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_roll_pid_T roll_pid_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_roll_pid_T roll_pid_Y;

/* Model entry point functions */
extern void roll_pid_initialize(void);
extern void roll_pid_step(void);
extern void roll_pid_terminate(void);

/* Real-time Model object */
extern RT_MODEL_roll_pid_T *const roll_pid_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('drone_controller_gazebo/roll_pid')    - opens subsystem drone_controller_gazebo/roll_pid
 * hilite_system('drone_controller_gazebo/roll_pid/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'drone_controller_gazebo'
 * '<S1>'   : 'drone_controller_gazebo/roll_pid'
 * '<S2>'   : 'drone_controller_gazebo/roll_pid/Anti-windup'
 * '<S3>'   : 'drone_controller_gazebo/roll_pid/D Gain'
 * '<S4>'   : 'drone_controller_gazebo/roll_pid/Filter'
 * '<S5>'   : 'drone_controller_gazebo/roll_pid/Filter ICs'
 * '<S6>'   : 'drone_controller_gazebo/roll_pid/I Gain'
 * '<S7>'   : 'drone_controller_gazebo/roll_pid/Ideal P Gain'
 * '<S8>'   : 'drone_controller_gazebo/roll_pid/Ideal P Gain Fdbk'
 * '<S9>'   : 'drone_controller_gazebo/roll_pid/Integrator'
 * '<S10>'  : 'drone_controller_gazebo/roll_pid/Integrator ICs'
 * '<S11>'  : 'drone_controller_gazebo/roll_pid/N Copy'
 * '<S12>'  : 'drone_controller_gazebo/roll_pid/N Gain'
 * '<S13>'  : 'drone_controller_gazebo/roll_pid/P Copy'
 * '<S14>'  : 'drone_controller_gazebo/roll_pid/Parallel P Gain'
 * '<S15>'  : 'drone_controller_gazebo/roll_pid/Reset Signal'
 * '<S16>'  : 'drone_controller_gazebo/roll_pid/Saturation'
 * '<S17>'  : 'drone_controller_gazebo/roll_pid/Saturation Fdbk'
 * '<S18>'  : 'drone_controller_gazebo/roll_pid/Sum'
 * '<S19>'  : 'drone_controller_gazebo/roll_pid/Sum Fdbk'
 * '<S20>'  : 'drone_controller_gazebo/roll_pid/Tracking Mode'
 * '<S21>'  : 'drone_controller_gazebo/roll_pid/Tracking Mode Sum'
 * '<S22>'  : 'drone_controller_gazebo/roll_pid/Tsamp - Integral'
 * '<S23>'  : 'drone_controller_gazebo/roll_pid/Tsamp - Ngain'
 * '<S24>'  : 'drone_controller_gazebo/roll_pid/postSat Signal'
 * '<S25>'  : 'drone_controller_gazebo/roll_pid/preSat Signal'
 * '<S26>'  : 'drone_controller_gazebo/roll_pid/Anti-windup/Passthrough'
 * '<S27>'  : 'drone_controller_gazebo/roll_pid/D Gain/Internal Parameters'
 * '<S28>'  : 'drone_controller_gazebo/roll_pid/Filter/Disc. Forward Euler Filter'
 * '<S29>'  : 'drone_controller_gazebo/roll_pid/Filter ICs/Internal IC - Filter'
 * '<S30>'  : 'drone_controller_gazebo/roll_pid/I Gain/Internal Parameters'
 * '<S31>'  : 'drone_controller_gazebo/roll_pid/Ideal P Gain/Passthrough'
 * '<S32>'  : 'drone_controller_gazebo/roll_pid/Ideal P Gain Fdbk/Disabled'
 * '<S33>'  : 'drone_controller_gazebo/roll_pid/Integrator/Discrete'
 * '<S34>'  : 'drone_controller_gazebo/roll_pid/Integrator ICs/Internal IC'
 * '<S35>'  : 'drone_controller_gazebo/roll_pid/N Copy/Disabled'
 * '<S36>'  : 'drone_controller_gazebo/roll_pid/N Gain/Internal Parameters'
 * '<S37>'  : 'drone_controller_gazebo/roll_pid/P Copy/Disabled'
 * '<S38>'  : 'drone_controller_gazebo/roll_pid/Parallel P Gain/Internal Parameters'
 * '<S39>'  : 'drone_controller_gazebo/roll_pid/Reset Signal/Disabled'
 * '<S40>'  : 'drone_controller_gazebo/roll_pid/Saturation/Passthrough'
 * '<S41>'  : 'drone_controller_gazebo/roll_pid/Saturation Fdbk/Disabled'
 * '<S42>'  : 'drone_controller_gazebo/roll_pid/Sum/Sum_PID'
 * '<S43>'  : 'drone_controller_gazebo/roll_pid/Sum Fdbk/Disabled'
 * '<S44>'  : 'drone_controller_gazebo/roll_pid/Tracking Mode/Disabled'
 * '<S45>'  : 'drone_controller_gazebo/roll_pid/Tracking Mode Sum/Passthrough'
 * '<S46>'  : 'drone_controller_gazebo/roll_pid/Tsamp - Integral/Passthrough'
 * '<S47>'  : 'drone_controller_gazebo/roll_pid/Tsamp - Ngain/Passthrough'
 * '<S48>'  : 'drone_controller_gazebo/roll_pid/postSat Signal/Forward_Path'
 * '<S49>'  : 'drone_controller_gazebo/roll_pid/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_roll_pid_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
