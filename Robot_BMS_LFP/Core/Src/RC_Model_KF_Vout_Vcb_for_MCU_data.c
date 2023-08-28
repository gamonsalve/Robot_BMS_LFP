/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: RC_Model_KF_Vout_Vcb_for_MCU_data.c
 *
 * Code generated for Simulink model 'RC_Model_KF_Vout_Vcb_for_MCU'.
 *
 * Model version                  : 4.68
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Fri Aug 25 14:07:21 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "RC_Model_KF_Vout_Vcb_for_MCU.h"

/* Constant parameters (default storage) */
const ConstP rtConstP = {
  /* Expression: pInitialization.Qbar
   * Referenced by: '<S24>/Qbar_'
   */
  { 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1 },

  /* Expression: pInitialization.Rbar
   * Referenced by: '<S24>/Rbar_'
   */
  { 0.31622776601683794, 0.0, 0.0, 0.31622776601683794 },

  /* Expression: pInitialization.P0
   * Referenced by: '<S2>/P0'
   */
  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  /* Expression: Rt
   * Referenced by: '<Root>/Rt'
   */
  { 0.062227321814255034, 0.055160369179457346, 0.048742534319501471,
    0.045425137567054907, 0.041935510418071788, 0.03807121444942952,
    0.035456101840728692, 0.034681529183308434, 0.033913554635226231,
    0.033152714449696641, 0.034030271203372944, 0.036412571739175441 },

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<Root>/1-D Lookup Table'
   *   '<Root>/1-D Lookup Table1'
   *   '<Root>/Cbulk'
   *   '<Root>/Csurface'
   *   '<Root>/Rb'
   *   '<Root>/Rs'
   *   '<Root>/Rt'
   */
  { 0.42690086916024478, 0.47690086916024477, 0.52690086916024481,
    0.57690086916024486, 0.62690086916024479, 0.67690086916024472,
    0.72690086916024477, 0.77690086916024481, 0.82690086916024474,
    0.87690086916024479, 0.92690086916024472, 0.97690086916024477 },

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<Root>/Rb'
   *   '<Root>/Rs'
   */
  { 0.91876327196447161, 0.768646175029861, 0.63167706988598016,
    0.55750787003683733, 0.49241378647954087, 0.4470616484656017,
    0.41249213130721851, 0.39380966892443897, 0.38029681834909784,
    0.37237373341734759, 0.37463654006417013, 0.3862543184223643 },

  /* Expression: Cbulk
   * Referenced by: '<Root>/Cbulk'
   */
  { 12217.0, 12217.0, 12217.0, 12217.0, 12217.0, 12217.0, 12217.0, 12217.0,
    12217.0, 12217.0, 12217.0, 12217.0 },

  /* Expression: Csurface
   * Referenced by: '<Root>/Csurface'
   */
  { 100.0, 100.0, 100.0, 100.0, 95.137981581174316, 79.69916174532473,
    63.845873280159786, 47.381908996480064, 34.88150238179454, 26.66678670673744,
    19.387150674915404, 12.966314635500986 },

  /* Pooled Parameter (Expression: Vocv)
   * Referenced by:
   *   '<Root>/1-D Lookup Table'
   *   '<Root>/1-D Lookup Table1'
   */
  { 11.896808, 11.980918001874313, 12.065333309694992, 12.151206877060016,
    12.236338283185747, 12.319855196857679, 12.403140698111921,
    12.486085237571727, 12.568309242451123, 12.649754152189026,
    12.72913185620045, 12.806610987961854 }
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
