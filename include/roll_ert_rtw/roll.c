/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: roll.c
 *
 * Code generated for Simulink model 'roll'.
 *
 * Model version                  : 1.33
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Mon Apr 15 09:43:00 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "roll.h"

/* Block states (default storage) */
DW_roll_T roll_DW;

/* External inputs (root inport signals with default storage) */
ExtU_roll_T roll_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_roll_T roll_Y;

/* Real-time model */
static RT_MODEL_roll_T roll_M_;
RT_MODEL_roll_T *const roll_M = &roll_M_;

/* Model step function */
void roll_step(void)
{
  /* Outport: '<Root>/y' incorporates:
   *  DiscreteIntegrator: '<S33>/Integrator'
   *  Gain: '<S31>/Proportional Gain'
   *  Inport: '<Root>/u'
   *  Sum: '<S42>/Sum'
   */
  roll_Y.y = (roll_U.u + roll_DW.Integrator_DSTATE) * 0.0752024069168458;

  /* Update for DiscreteIntegrator: '<S33>/Integrator' incorporates:
   *  Gain: '<S30>/Integral Gain'
   *  Inport: '<Root>/u'
   */
  roll_DW.Integrator_DSTATE += 0.174993072998953 * roll_U.u * 0.002;
}

/* Model initialize function */
void roll_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void roll_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
