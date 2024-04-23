/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: yaw_rate_Controller.c
 *
 * Code generated for Simulink model 'yaw_rate_Controller'.
 *
 * Model version                  : 1.57
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Tue Apr 23 06:47:21 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "yaw_rate_Controller.h"

/* Block states (default storage) */
DW_yaw_rate_Controller_T yaw_rate_Controller_DW;

/* External inputs (root inport signals with default storage) */
ExtU_yaw_rate_Controller_T yaw_rate_Controller_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_yaw_rate_Controller_T yaw_rate_Controller_Y;

/* Real-time model */
static RT_MODEL_yaw_rate_Controller_T yaw_rate_Controller_M_;
RT_MODEL_yaw_rate_Controller_T *const yaw_rate_Controller_M =
  &yaw_rate_Controller_M_;

/* Model step function */
void yaw_rate_Controller_step(void)
{
  /* Outport: '<Root>/y' incorporates:
   *  DiscreteIntegrator: '<S33>/Integrator'
   *  Gain: '<S38>/Proportional Gain'
   *  Inport: '<Root>/u'
   *  Sum: '<S42>/Sum'
   */
  yaw_rate_Controller_Y.y = 0.248072499514363 * yaw_rate_Controller_U.u +
    yaw_rate_Controller_DW.Integrator_DSTATE;

  /* Update for DiscreteIntegrator: '<S33>/Integrator' incorporates:
   *  Gain: '<S30>/Integral Gain'
   *  Inport: '<Root>/u'
   */
  yaw_rate_Controller_DW.Integrator_DSTATE += 0.0869721231177393 *
    yaw_rate_Controller_U.u * 0.01;
}

/* Model initialize function */
void yaw_rate_Controller_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void yaw_rate_Controller_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
