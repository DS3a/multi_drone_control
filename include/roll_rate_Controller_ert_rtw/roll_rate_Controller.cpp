/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: roll_rate_Controller.c
 *
 * Code generated for Simulink model 'roll_rate_Controller'.
 *
 * Model version                  : 1.57
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Tue Apr 23 06:45:31 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "roll_rate_Controller.h"

/* Block states (default storage) */
DW_roll_rate_Controller_T roll_rate_Controller_DW;

/* External inputs (root inport signals with default storage) */
ExtU_roll_rate_Controller_T roll_rate_Controller_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_roll_rate_Controller_T roll_rate_Controller_Y;

/* Real-time model */
static RT_MODEL_roll_rate_Controller_T roll_rate_Controller_M_;
RT_MODEL_roll_rate_Controller_T *const roll_rate_Controller_M =
  &roll_rate_Controller_M_;

/* Model step function */
void roll_rate_Controller_step(void)
{
  /* Outport: '<Root>/y' incorporates:
   *  DiscreteIntegrator: '<S33>/Integrator'
   *  Gain: '<S38>/Proportional Gain'
   *  Inport: '<Root>/u'
   *  Sum: '<S42>/Sum'
   */
  roll_rate_Controller_Y.y = 0.141613583408204 * roll_rate_Controller_U.u +
    roll_rate_Controller_DW.Integrator_DSTATE;

  /* Update for DiscreteIntegrator: '<S33>/Integrator' incorporates:
   *  Gain: '<S30>/Integral Gain'
   *  Inport: '<Root>/u'
   */
  roll_rate_Controller_DW.Integrator_DSTATE += 0.049648526279347 *
    roll_rate_Controller_U.u * 0.01;
}

/* Model initialize function */
void roll_rate_Controller_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void roll_rate_Controller_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
