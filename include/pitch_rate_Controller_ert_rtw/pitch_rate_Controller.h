/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: pitch_rate_Controller.h
 *
 * Code generated for Simulink model 'pitch_rate_Controller'.
 *
 * Model version                  : 1.57
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Tue Apr 23 06:46:41 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_pitch_rate_Controller_h_
#define RTW_HEADER_pitch_rate_Controller_h_
#ifndef pitch_rate_Controller_COMMON_INCLUDES_
#define pitch_rate_Controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                              /* pitch_rate_Controller_COMMON_INCLUDES_ */

#include "pitch_rate_Controller_types.h"

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
} DW_pitch_rate_Controller_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T u;                            /* '<Root>/u' */
} ExtU_pitch_rate_Controller_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T y;                            /* '<Root>/y' */
} ExtY_pitch_rate_Controller_T;

/* Real-time Model Data Structure */
struct tag_RTM_pitch_rate_Controller_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_pitch_rate_Controller_T pitch_rate_Controller_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_pitch_rate_Controller_T pitch_rate_Controller_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_pitch_rate_Controller_T pitch_rate_Controller_Y;

/* Model entry point functions */
extern void pitch_rate_Controller_initialize(void);
extern void pitch_rate_Controller_step(void);
extern void pitch_rate_Controller_terminate(void);

/* Real-time Model object */
extern RT_MODEL_pitch_rate_Controlle_T *const pitch_rate_Controller_M;

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
 * hilite_system('drone_controller_gazebo/pitch_rate_Controller')    - opens subsystem drone_controller_gazebo/pitch_rate_Controller
 * hilite_system('drone_controller_gazebo/pitch_rate_Controller/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'drone_controller_gazebo'
 * '<S1>'   : 'drone_controller_gazebo/pitch_rate_Controller'
 * '<S2>'   : 'drone_controller_gazebo/pitch_rate_Controller/Anti-windup'
 * '<S3>'   : 'drone_controller_gazebo/pitch_rate_Controller/D Gain'
 * '<S4>'   : 'drone_controller_gazebo/pitch_rate_Controller/Filter'
 * '<S5>'   : 'drone_controller_gazebo/pitch_rate_Controller/Filter ICs'
 * '<S6>'   : 'drone_controller_gazebo/pitch_rate_Controller/I Gain'
 * '<S7>'   : 'drone_controller_gazebo/pitch_rate_Controller/Ideal P Gain'
 * '<S8>'   : 'drone_controller_gazebo/pitch_rate_Controller/Ideal P Gain Fdbk'
 * '<S9>'   : 'drone_controller_gazebo/pitch_rate_Controller/Integrator'
 * '<S10>'  : 'drone_controller_gazebo/pitch_rate_Controller/Integrator ICs'
 * '<S11>'  : 'drone_controller_gazebo/pitch_rate_Controller/N Copy'
 * '<S12>'  : 'drone_controller_gazebo/pitch_rate_Controller/N Gain'
 * '<S13>'  : 'drone_controller_gazebo/pitch_rate_Controller/P Copy'
 * '<S14>'  : 'drone_controller_gazebo/pitch_rate_Controller/Parallel P Gain'
 * '<S15>'  : 'drone_controller_gazebo/pitch_rate_Controller/Reset Signal'
 * '<S16>'  : 'drone_controller_gazebo/pitch_rate_Controller/Saturation'
 * '<S17>'  : 'drone_controller_gazebo/pitch_rate_Controller/Saturation Fdbk'
 * '<S18>'  : 'drone_controller_gazebo/pitch_rate_Controller/Sum'
 * '<S19>'  : 'drone_controller_gazebo/pitch_rate_Controller/Sum Fdbk'
 * '<S20>'  : 'drone_controller_gazebo/pitch_rate_Controller/Tracking Mode'
 * '<S21>'  : 'drone_controller_gazebo/pitch_rate_Controller/Tracking Mode Sum'
 * '<S22>'  : 'drone_controller_gazebo/pitch_rate_Controller/Tsamp - Integral'
 * '<S23>'  : 'drone_controller_gazebo/pitch_rate_Controller/Tsamp - Ngain'
 * '<S24>'  : 'drone_controller_gazebo/pitch_rate_Controller/postSat Signal'
 * '<S25>'  : 'drone_controller_gazebo/pitch_rate_Controller/preSat Signal'
 * '<S26>'  : 'drone_controller_gazebo/pitch_rate_Controller/Anti-windup/Passthrough'
 * '<S27>'  : 'drone_controller_gazebo/pitch_rate_Controller/D Gain/Disabled'
 * '<S28>'  : 'drone_controller_gazebo/pitch_rate_Controller/Filter/Disabled'
 * '<S29>'  : 'drone_controller_gazebo/pitch_rate_Controller/Filter ICs/Disabled'
 * '<S30>'  : 'drone_controller_gazebo/pitch_rate_Controller/I Gain/Internal Parameters'
 * '<S31>'  : 'drone_controller_gazebo/pitch_rate_Controller/Ideal P Gain/Passthrough'
 * '<S32>'  : 'drone_controller_gazebo/pitch_rate_Controller/Ideal P Gain Fdbk/Disabled'
 * '<S33>'  : 'drone_controller_gazebo/pitch_rate_Controller/Integrator/Discrete'
 * '<S34>'  : 'drone_controller_gazebo/pitch_rate_Controller/Integrator ICs/Internal IC'
 * '<S35>'  : 'drone_controller_gazebo/pitch_rate_Controller/N Copy/Disabled wSignal Specification'
 * '<S36>'  : 'drone_controller_gazebo/pitch_rate_Controller/N Gain/Disabled'
 * '<S37>'  : 'drone_controller_gazebo/pitch_rate_Controller/P Copy/Disabled'
 * '<S38>'  : 'drone_controller_gazebo/pitch_rate_Controller/Parallel P Gain/Internal Parameters'
 * '<S39>'  : 'drone_controller_gazebo/pitch_rate_Controller/Reset Signal/Disabled'
 * '<S40>'  : 'drone_controller_gazebo/pitch_rate_Controller/Saturation/Passthrough'
 * '<S41>'  : 'drone_controller_gazebo/pitch_rate_Controller/Saturation Fdbk/Disabled'
 * '<S42>'  : 'drone_controller_gazebo/pitch_rate_Controller/Sum/Sum_PI'
 * '<S43>'  : 'drone_controller_gazebo/pitch_rate_Controller/Sum Fdbk/Disabled'
 * '<S44>'  : 'drone_controller_gazebo/pitch_rate_Controller/Tracking Mode/Disabled'
 * '<S45>'  : 'drone_controller_gazebo/pitch_rate_Controller/Tracking Mode Sum/Passthrough'
 * '<S46>'  : 'drone_controller_gazebo/pitch_rate_Controller/Tsamp - Integral/Passthrough'
 * '<S47>'  : 'drone_controller_gazebo/pitch_rate_Controller/Tsamp - Ngain/Passthrough'
 * '<S48>'  : 'drone_controller_gazebo/pitch_rate_Controller/postSat Signal/Forward_Path'
 * '<S49>'  : 'drone_controller_gazebo/pitch_rate_Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_pitch_rate_Controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
