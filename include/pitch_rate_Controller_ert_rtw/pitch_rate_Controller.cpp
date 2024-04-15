/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: pitch_rate_Controller.c
 *
 * Code generated for Simulink model 'pitch_rate_Controller'.
 *
 * Model version                  : 1.44
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Mon Apr 15 11:52:37 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "pitch_rate_Controller.h"

/* Block states (default storage) */
DW_pitch_rate_Controller_T pitch_rate_Controller_DW;

/* External inputs (root inport signals with default storage) */
ExtU_pitch_rate_Controller_T pitch_rate_Controller_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_pitch_rate_Controller_T pitch_rate_Controller_Y;

/* Real-time model */
static RT_MODEL_pitch_rate_Controlle_T pitch_rate_Controller_M_;
RT_MODEL_pitch_rate_Controlle_T *const pitch_rate_Controller_M =
  &pitch_rate_Controller_M_;

/* Model step function */
void pitch_rate_Controller_step(void)
{
  /* Outport: '<Root>/y' incorporates:
   *  DiscreteIntegrator: '<S33>/Integrator'
   *  Gain: '<S38>/Proportional Gain'
   *  Inport: '<Root>/u'
   *  Sum: '<S42>/Sum'
   */
  pitch_rate_Controller_Y.y = 0.346841783263699 * pitch_rate_Controller_U.u +
    pitch_rate_Controller_DW.Integrator_DSTATE;

  /* Update for DiscreteIntegrator: '<S33>/Integrator' incorporates:
   *  Gain: '<S30>/Integral Gain'
   *  Inport: '<Root>/u'
   */
  pitch_rate_Controller_DW.Integrator_DSTATE += 0.289135541287304 *
    pitch_rate_Controller_U.u * 0.01;
}

/* Model initialize function */
void pitch_rate_Controller_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void pitch_rate_Controller_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
