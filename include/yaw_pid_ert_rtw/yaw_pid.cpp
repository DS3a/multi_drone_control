/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: yaw_pid.c
 *
 * Code generated for Simulink model 'yaw_pid'.
 *
 * Model version                  : 1.57
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Tue Apr 23 06:48:36 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "yaw_pid.h"
#include "rtwtypes.h"

/* Block states (default storage) */
DW_yaw_pid_T yaw_pid_DW;

/* External inputs (root inport signals with default storage) */
ExtU_yaw_pid_T yaw_pid_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_yaw_pid_T yaw_pid_Y;

/* Real-time model */
static RT_MODEL_yaw_pid_T yaw_pid_M_;
RT_MODEL_yaw_pid_T *const yaw_pid_M = &yaw_pid_M_;

/* Model step function */
void yaw_pid_step(void)
{
  real_T rtb_FilterCoefficient;

  /* Gain: '<S36>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S28>/Filter'
   *  Gain: '<S27>/Derivative Gain'
   *  Inport: '<Root>/u'
   *  Sum: '<S28>/SumD'
   */
  rtb_FilterCoefficient = (0.372004682229386 * yaw_pid_U.u -
    yaw_pid_DW.Filter_DSTATE) * 34.6864643900812;

  /* Outport: '<Root>/y' incorporates:
   *  DiscreteIntegrator: '<S33>/Integrator'
   *  Gain: '<S38>/Proportional Gain'
   *  Inport: '<Root>/u'
   *  Sum: '<S42>/Sum'
   */
  yaw_pid_Y.y = (3.78263033452263 * yaw_pid_U.u + yaw_pid_DW.Integrator_DSTATE)
    + rtb_FilterCoefficient;

  /* Update for DiscreteIntegrator: '<S33>/Integrator' incorporates:
   *  Gain: '<S30>/Integral Gain'
   *  Inport: '<Root>/u'
   */
  yaw_pid_DW.Integrator_DSTATE += 2.56148489944847 * yaw_pid_U.u * 0.01;

  /* Update for DiscreteIntegrator: '<S28>/Filter' */
  yaw_pid_DW.Filter_DSTATE += 0.01 * rtb_FilterCoefficient;
}

/* Model initialize function */
void yaw_pid_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void yaw_pid_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
