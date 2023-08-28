/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: RC_Model_KF_Vout_Vcb_for_MCU.c
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
#include "rtwtypes.h"
#include <string.h>
#include <math.h>
#include <stddef.h>
#define NumBitsPerChar                 8U

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern real_T rt_hypotd_snf(real_T u0, real_T u1);
extern real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u);
extern real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u);
static real_T look1_binlca(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex);
static real_T look1_binlg(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex);
static int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator);

/* Forward declaration for local functions */
static real_T xnrm2(int32_T n, const real_T x[10], int32_T ix0);
static void xgemv(int32_T m, int32_T n, const real_T A[10], int32_T ia0, const
                  real_T x[10], int32_T ix0, real_T y[2]);
static void xgerc(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const real_T
                  y[2], real_T A[10], int32_T ia0);
static void qrFactor(const real_T A[6], const real_T S[9], const real_T Ns[4],
                     real_T b_S[4]);
static void trisolve(const real_T A[4], real_T B_3[6]);
static void linsolve(const real_T A[4], const real_T B_0[6], real_T C[6]);
static void trisolve_o(const real_T A[4], real_T B_4[6]);
static void linsolve_b(const real_T A[4], const real_T B_1[6], real_T C[6]);
static void mtimes(const real_T A[9], const real_T B_2[9], real_T C[9]);
static real_T xnrm2_g(int32_T n, const real_T x[15], int32_T ix0);
static void xgemv_g(int32_T m, int32_T n, const real_T A[15], int32_T ia0, const
                    real_T x[15], int32_T ix0, real_T y[3]);
static void xgerc_a(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
                    real_T y[3], real_T A[15], int32_T ia0);
static void qrFactor_j(const real_T A[9], real_T S[9], const real_T Ns[6]);
static real_T xnrm2_gw(int32_T n, const real_T x[18], int32_T ix0);
static void xgemv_gp(int32_T m, int32_T n, const real_T A[18], int32_T ia0,
                     const real_T x[18], int32_T ix0, real_T y[3]);
static void xgerc_aq(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
                     real_T y[3], real_T A[18], int32_T ia0);
static void qrFactor_jq(const real_T A[9], real_T S[9], const real_T Ns[9]);
static real_T xnrm2_gwq(int32_T n, const real_T x[9], int32_T ix0);
static real_T xdotc(int32_T n, const real_T x[9], int32_T ix0, const real_T y[9],
                    int32_T iy0);
static void xaxpy(int32_T n, real_T a, int32_T ix0, real_T y[9], int32_T iy0);
static real_T xnrm2_gwqu(int32_T n, const real_T x[3], int32_T ix0);
static void xaxpy_c(int32_T n, real_T a, const real_T x[9], int32_T ix0, real_T
                    y[3], int32_T iy0);
static void xaxpy_cs(int32_T n, real_T a, const real_T x[3], int32_T ix0, real_T
                     y[9], int32_T iy0);
static void xswap(real_T x[9], int32_T ix0, int32_T iy0);
static void xrotg(real_T *a, real_T *b, real_T *c, real_T *s);
static void xrot(real_T x[9], int32_T ix0, int32_T iy0, real_T c, real_T s);
static void svd(const real_T A[9], real_T U[9], real_T s[3], real_T V[9]);
static void xgemv_gpk(int32_T m, int32_T n, const real_T A[9], int32_T ia0,
                      const real_T x[9], int32_T ix0, real_T y[3]);
static void xgerc_aq4(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
                      real_T y[3], real_T A[9], int32_T ia0);
static void qr(const real_T A[9], real_T Q[9], real_T R[9]);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);

#define NOT_USING_NONFINITE_LITERALS   1

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static void rt_InitInfAndNaN(size_t realSize);
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
typedef struct {
  struct {
    uint32_T wordH;
    uint32_T wordL;
  } words;
} BigEndianIEEEDouble;

typedef struct {
  struct {
    uint32_T wordL;
    uint32_T wordH;
  } words;
} LittleEndianIEEEDouble;

typedef struct {
  union {
    real32_T wordLreal;
    uint32_T wordLuint;
  } wordL;
} IEEESingle;

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;
static real_T rtGetInf(void);
static real32_T rtGetInfF(void);
static real_T rtGetMinusInf(void);
static real32_T rtGetMinusInfF(void);

/*
 * Initialize rtNaN needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetNaN(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T nan = 0.0;
  if (bitsPerReal == 32U) {
    nan = rtGetNaNF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF80000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    nan = tmpVal.fltVal;
  }

  return nan;
}

/*
 * Initialize rtNaNF needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetNaNF(void)
{
  IEEESingle nanF = { { 0.0F } };

  nanF.wordL.wordLuint = 0xFFC00000U;
  return nanF.wordL.wordLreal;
}

/*
 * Initialize the rtInf, rtMinusInf, and rtNaN needed by the
 * generated code. NaN is initialized as non-signaling. Assumes IEEE.
 */
static void rt_InitInfAndNaN(size_t realSize)
{
  (void) (realSize);
  rtNaN = rtGetNaN();
  rtNaNF = rtGetNaNF();
  rtInf = rtGetInf();
  rtInfF = rtGetInfF();
  rtMinusInf = rtGetMinusInf();
  rtMinusInfF = rtGetMinusInfF();
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  boolean_T result = (boolean_T) 0;
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  if (bitsPerReal == 32U) {
    result = rtIsNaNF((real32_T)value);
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.fltVal = value;
    result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) == 0x7FF00000 &&
                         ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                          (tmpVal.bitVal.words.wordL != 0) ));
  }

  return result;
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  IEEESingle tmp;
  tmp.wordL.wordLreal = value;
  return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                     (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
}

/*
 * Initialize rtInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T inf = 0.0;
  if (bitsPerReal == 32U) {
    inf = rtGetInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0x7FF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    inf = tmpVal.fltVal;
  }

  return inf;
}

/*
 * Initialize rtInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetInfF(void)
{
  IEEESingle infF;
  infF.wordL.wordLuint = 0x7F800000U;
  return infF.wordL.wordLreal;
}

/*
 * Initialize rtMinusInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetMinusInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T minf = 0.0;
  if (bitsPerReal == 32U) {
    minf = rtGetMinusInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    minf = tmpVal.fltVal;
  }

  return minf;
}

/*
 * Initialize rtMinusInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetMinusInfF(void)
{
  IEEESingle minfF;
  minfF.wordL.wordLuint = 0xFF800000U;
  return minfF.wordL.wordLreal;
}

static real_T look1_binlca(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex)
{
  real_T frac;
  real_T y;
  uint32_T iLeft;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Clip'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = 0.0;
  } else if (u0 < bp0[maxIndex]) {
    uint32_T bpIdx;
    uint32_T iRght;

    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex;
    frac = 0.0;
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'on'
     Overflow mode: 'wrapping'
   */
  if (iLeft == maxIndex) {
    y = table[iLeft];
  } else {
    real_T yL_0d0;
    yL_0d0 = table[iLeft];
    y = (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
  }

  return y;
}

static real_T look1_binlg(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex)
{
  uint32_T bpIdx;
  uint32_T iLeft;
  uint32_T iRght;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'on'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'on'
   */
  /* Binary Search */
  bpIdx = maxIndex >> 1U;
  iLeft = 0U;
  iRght = maxIndex;
  while (iRght - iLeft > 1U) {
    if (u0 < bp0[bpIdx]) {
      iRght = bpIdx;
    } else {
      iLeft = bpIdx;
    }

    bpIdx = (iRght + iLeft) >> 1U;
  }

  real_T yL_0d0;

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  yL_0d0 = table[iLeft];
  return (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]) * (table[iLeft + 1U]
    - yL_0d0) + yL_0d0;
}

static int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static real_T xnrm2(int32_T n, const real_T x[10], int32_T ix0)
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        real_T absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T b;
  real_T y;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = sqrt(a * a + 1.0) * b;
  } else if (a > b) {
    b /= a;
    y = sqrt(b * b + 1.0) * a;
  } else if (rtIsNaN(b)) {
    y = (rtNaN);
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void xgemv(int32_T m, int32_T n, const real_T A[10], int32_T ia0, const
                  real_T x[10], int32_T ix0, real_T y[2])
{
  int32_T b_iy;
  int32_T iyend;
  if ((m != 0) && (n != 0)) {
    int32_T b;
    if (n - 1 >= 0) {
      memset(&y[0], 0, (uint32_T)n * sizeof(real_T));
    }

    b = (n - 1) * 5 + ia0;
    for (b_iy = ia0; b_iy <= b; b_iy += 5) {
      real_T c;
      int32_T d;
      c = 0.0;
      d = b_iy + m;
      for (iyend = b_iy; iyend < d; iyend++) {
        c += x[((ix0 + iyend) - b_iy) - 1] * A[iyend - 1];
      }

      iyend = div_nde_s32_floor(b_iy - ia0, 5);
      y[iyend] += c;
    }
  }
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void xgerc(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const real_T
                  y[2], real_T A[10], int32_T ia0)
{
  int32_T ijA;
  int32_T j;
  if (!(alpha1 == 0.0)) {
    int32_T jA;
    jA = ia0;
    for (j = 0; j < n; j++) {
      real_T temp;
      temp = y[j];
      if (temp != 0.0) {
        int32_T b;
        temp *= alpha1;
        b = m + jA;
        for (ijA = jA; ijA < b; ijA++) {
          A[ijA - 1] += A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += 5;
    }
  }
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void qrFactor(const real_T A[6], const real_T S[9], const real_T Ns[4],
                     real_T b_S[4])
{
  real_T b_A[10];
  real_T y[6];
  real_T R[4];
  real_T tau[2];
  real_T work[2];
  int32_T aoffset;
  int32_T c_k;
  int32_T coffset;
  int32_T j;
  int32_T k;
  int32_T lastv;
  for (j = 0; j < 2; j++) {
    coffset = j * 3;
    for (lastv = 0; lastv < 3; lastv++) {
      aoffset = lastv * 3;
      y[coffset + lastv] = (S[aoffset + 1] * A[j + 2] + S[aoffset] * A[j]) +
        S[aoffset + 2] * A[j + 4];
    }
  }

  for (j = 0; j < 2; j++) {
    b_A[5 * j] = y[3 * j];
    b_A[5 * j + 1] = y[3 * j + 1];
    b_A[5 * j + 2] = y[3 * j + 2];
    b_A[5 * j + 3] = Ns[j];
    b_A[5 * j + 4] = Ns[j + 2];
    tau[j] = 0.0;
    work[j] = 0.0;
  }

  for (j = 0; j < 2; j++) {
    real_T atmp;
    real_T s;
    coffset = j * 5 + j;
    atmp = b_A[coffset];
    lastv = coffset + 2;
    tau[j] = 0.0;
    s = xnrm2(4 - j, b_A, coffset + 2);
    if (s != 0.0) {
      s = rt_hypotd_snf(b_A[coffset], s);
      if (b_A[coffset] >= 0.0) {
        s = -s;
      }

      if (fabs(s) < 1.0020841800044864E-292) {
        aoffset = 0;
        k = (coffset - j) + 5;
        do {
          aoffset++;
          for (c_k = lastv; c_k <= k; c_k++) {
            b_A[c_k - 1] *= 9.9792015476736E+291;
          }

          s *= 9.9792015476736E+291;
          atmp *= 9.9792015476736E+291;
        } while ((fabs(s) < 1.0020841800044864E-292) && (aoffset < 20));

        s = rt_hypotd_snf(atmp, xnrm2(4 - j, b_A, coffset + 2));
        if (atmp >= 0.0) {
          s = -s;
        }

        tau[j] = (s - atmp) / s;
        atmp = 1.0 / (atmp - s);
        for (c_k = lastv; c_k <= k; c_k++) {
          b_A[c_k - 1] *= atmp;
        }

        for (lastv = 0; lastv < aoffset; lastv++) {
          s *= 1.0020841800044864E-292;
        }

        atmp = s;
      } else {
        tau[j] = (s - b_A[coffset]) / s;
        atmp = 1.0 / (b_A[coffset] - s);
        aoffset = (coffset - j) + 5;
        for (k = lastv; k <= aoffset; k++) {
          b_A[k - 1] *= atmp;
        }

        atmp = s;
      }
    }

    b_A[coffset] = atmp;
    if (j + 1 < 2) {
      s = b_A[coffset];
      b_A[coffset] = 1.0;
      if (tau[0] != 0.0) {
        lastv = 5;
        aoffset = coffset + 4;
        while ((lastv > 0) && (b_A[aoffset] == 0.0)) {
          lastv--;
          aoffset--;
        }

        aoffset = 1;
        k = coffset;
        int32_T exitg1;
        do {
          exitg1 = 0;
          if (k + 6 <= (coffset + lastv) + 5) {
            if (b_A[k + 5] != 0.0) {
              exitg1 = 1;
            } else {
              k++;
            }
          } else {
            aoffset = 0;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      } else {
        lastv = 0;
        aoffset = 0;
      }

      if (lastv > 0) {
        xgemv(lastv, aoffset, b_A, coffset + 6, b_A, coffset + 1, work);
        xgerc(lastv, aoffset, -tau[0], coffset + 1, work, b_A, coffset + 6);
      }

      b_A[coffset] = s;
    }
  }

  for (j = 0; j < 2; j++) {
    for (coffset = 0; coffset <= j; coffset++) {
      R[coffset + (j << 1)] = b_A[5 * j + coffset];
    }

    if (j + 2 <= 2) {
      R[(j << 1) + 1] = 0.0;
    }
  }

  b_S[0] = R[0];
  b_S[1] = R[2];
  b_S[2] = R[1];
  b_S[3] = R[3];
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void trisolve(const real_T A[4], real_T B_3[6])
{
  int32_T b_k;
  int32_T i;
  int32_T j;
  for (j = 0; j < 3; j++) {
    int32_T jBcol;
    jBcol = j << 1;
    for (b_k = 0; b_k < 2; b_k++) {
      real_T tmp_0;
      int32_T kAcol;
      int32_T tmp;
      kAcol = b_k << 1;
      tmp = b_k + jBcol;
      tmp_0 = B_3[tmp];
      if (tmp_0 != 0.0) {
        B_3[tmp] = tmp_0 / A[b_k + kAcol];
        for (i = b_k + 2; i < 3; i++) {
          B_3[jBcol + 1] -= A[kAcol + 1] * B_3[tmp];
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void linsolve(const real_T A[4], const real_T B_0[6], real_T C[6])
{
  int32_T j;
  for (j = 0; j < 3; j++) {
    int32_T C_tmp;
    C_tmp = j << 1;
    C[C_tmp] = B_0[C_tmp];
    C[C_tmp + 1] = B_0[C_tmp + 1];
  }

  trisolve(A, C);
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void trisolve_o(const real_T A[4], real_T B_4[6])
{
  int32_T i;
  int32_T j;
  int32_T k;
  for (j = 0; j < 3; j++) {
    int32_T jBcol;
    jBcol = j << 1;
    for (k = 1; k >= 0; k--) {
      real_T tmp_0;
      int32_T kAcol;
      int32_T tmp;
      kAcol = k << 1;
      tmp = k + jBcol;
      tmp_0 = B_4[tmp];
      if (tmp_0 != 0.0) {
        B_4[tmp] = tmp_0 / A[k + kAcol];
        for (i = 0; i < k; i++) {
          B_4[jBcol] -= B_4[tmp] * A[kAcol];
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void linsolve_b(const real_T A[4], const real_T B_1[6], real_T C[6])
{
  int32_T j;
  for (j = 0; j < 3; j++) {
    int32_T C_tmp;
    C_tmp = j << 1;
    C[C_tmp] = B_1[C_tmp];
    C[C_tmp + 1] = B_1[C_tmp + 1];
  }

  trisolve_o(A, C);
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void mtimes(const real_T A[9], const real_T B_2[9], real_T C[9])
{
  int32_T i;
  int32_T j;
  for (j = 0; j < 3; j++) {
    int32_T coffset;
    coffset = j * 3;
    for (i = 0; i < 3; i++) {
      int32_T aoffset;
      aoffset = i * 3;
      C[coffset + i] = (A[aoffset + 1] * B_2[j + 3] + A[aoffset] * B_2[j]) +
        A[aoffset + 2] * B_2[j + 6];
    }
  }
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static real_T xnrm2_g(int32_T n, const real_T x[15], int32_T ix0)
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        real_T absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void xgemv_g(int32_T m, int32_T n, const real_T A[15], int32_T ia0, const
                    real_T x[15], int32_T ix0, real_T y[3])
{
  int32_T b_iy;
  int32_T iyend;
  if ((m != 0) && (n != 0)) {
    int32_T b;
    if (n - 1 >= 0) {
      memset(&y[0], 0, (uint32_T)n * sizeof(real_T));
    }

    b = (n - 1) * 5 + ia0;
    for (b_iy = ia0; b_iy <= b; b_iy += 5) {
      real_T c;
      int32_T d;
      c = 0.0;
      d = b_iy + m;
      for (iyend = b_iy; iyend < d; iyend++) {
        c += x[((ix0 + iyend) - b_iy) - 1] * A[iyend - 1];
      }

      iyend = div_nde_s32_floor(b_iy - ia0, 5);
      y[iyend] += c;
    }
  }
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void xgerc_a(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
                    real_T y[3], real_T A[15], int32_T ia0)
{
  int32_T ijA;
  int32_T j;
  if (!(alpha1 == 0.0)) {
    int32_T jA;
    jA = ia0;
    for (j = 0; j < n; j++) {
      real_T temp;
      temp = y[j];
      if (temp != 0.0) {
        int32_T b;
        temp *= alpha1;
        b = m + jA;
        for (ijA = jA; ijA < b; ijA++) {
          A[ijA - 1] += A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += 5;
    }
  }
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void qrFactor_j(const real_T A[9], real_T S[9], const real_T Ns[6])
{
  real_T b_A[15];
  real_T R[9];
  real_T tau[3];
  real_T work[3];
  int32_T coltop;
  int32_T i;
  int32_T ii;
  int32_T knt;
  int32_T lastv;
  mtimes(S, A, R);
  for (i = 0; i < 3; i++) {
    b_A[5 * i] = R[3 * i];
    b_A[5 * i + 1] = R[3 * i + 1];
    b_A[5 * i + 2] = R[3 * i + 2];
    b_A[5 * i + 3] = Ns[i];
    b_A[5 * i + 4] = Ns[i + 3];
    work[i] = 0.0;
  }

  for (i = 0; i < 3; i++) {
    real_T atmp;
    real_T beta1;
    int32_T c_tmp;
    ii = i * 5 + i;
    atmp = b_A[ii];
    lastv = ii + 2;
    tau[i] = 0.0;
    beta1 = xnrm2_g(4 - i, b_A, ii + 2);
    if (beta1 != 0.0) {
      beta1 = rt_hypotd_snf(b_A[ii], beta1);
      if (b_A[ii] >= 0.0) {
        beta1 = -beta1;
      }

      if (fabs(beta1) < 1.0020841800044864E-292) {
        knt = 0;
        c_tmp = (ii - i) + 5;
        do {
          knt++;
          for (coltop = lastv; coltop <= c_tmp; coltop++) {
            b_A[coltop - 1] *= 9.9792015476736E+291;
          }

          beta1 *= 9.9792015476736E+291;
          atmp *= 9.9792015476736E+291;
        } while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));

        beta1 = rt_hypotd_snf(atmp, xnrm2_g(4 - i, b_A, ii + 2));
        if (atmp >= 0.0) {
          beta1 = -beta1;
        }

        tau[i] = (beta1 - atmp) / beta1;
        atmp = 1.0 / (atmp - beta1);
        for (coltop = lastv; coltop <= c_tmp; coltop++) {
          b_A[coltop - 1] *= atmp;
        }

        for (lastv = 0; lastv < knt; lastv++) {
          beta1 *= 1.0020841800044864E-292;
        }

        atmp = beta1;
      } else {
        tau[i] = (beta1 - b_A[ii]) / beta1;
        atmp = 1.0 / (b_A[ii] - beta1);
        coltop = (ii - i) + 5;
        for (knt = lastv; knt <= coltop; knt++) {
          b_A[knt - 1] *= atmp;
        }

        atmp = beta1;
      }
    }

    b_A[ii] = atmp;
    if (i + 1 < 3) {
      beta1 = b_A[ii];
      b_A[ii] = 1.0;
      if (tau[i] != 0.0) {
        boolean_T exitg2;
        lastv = 5 - i;
        knt = (ii - i) + 4;
        while ((lastv > 0) && (b_A[knt] == 0.0)) {
          lastv--;
          knt--;
        }

        knt = 2 - i;
        exitg2 = false;
        while ((!exitg2) && (knt > 0)) {
          int32_T exitg1;
          coltop = ((knt - 1) * 5 + ii) + 5;
          c_tmp = coltop;
          do {
            exitg1 = 0;
            if (c_tmp + 1 <= coltop + lastv) {
              if (b_A[c_tmp] != 0.0) {
                exitg1 = 1;
              } else {
                c_tmp++;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        knt = 0;
      }

      if (lastv > 0) {
        xgemv_g(lastv, knt, b_A, ii + 6, b_A, ii + 1, work);
        xgerc_a(lastv, knt, -tau[i], ii + 1, work, b_A, ii + 6);
      }

      b_A[ii] = beta1;
    }
  }

  for (i = 0; i < 3; i++) {
    for (ii = 0; ii <= i; ii++) {
      R[ii + 3 * i] = b_A[5 * i + ii];
    }

    for (ii = i + 2; ii < 4; ii++) {
      R[(ii + 3 * i) - 1] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    S[3 * i] = R[i];
    S[3 * i + 1] = R[i + 3];
    S[3 * i + 2] = R[i + 6];
  }
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static real_T xnrm2_gw(int32_T n, const real_T x[18], int32_T ix0)
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        real_T absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void xgemv_gp(int32_T m, int32_T n, const real_T A[18], int32_T ia0,
                     const real_T x[18], int32_T ix0, real_T y[3])
{
  int32_T b_iy;
  int32_T iyend;
  if ((m != 0) && (n != 0)) {
    int32_T b;
    if (n - 1 >= 0) {
      memset(&y[0], 0, (uint32_T)n * sizeof(real_T));
    }

    b = (n - 1) * 6 + ia0;
    for (b_iy = ia0; b_iy <= b; b_iy += 6) {
      real_T c;
      int32_T d;
      c = 0.0;
      d = b_iy + m;
      for (iyend = b_iy; iyend < d; iyend++) {
        c += x[((ix0 + iyend) - b_iy) - 1] * A[iyend - 1];
      }

      iyend = div_nde_s32_floor(b_iy - ia0, 6);
      y[iyend] += c;
    }
  }
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void xgerc_aq(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
                     real_T y[3], real_T A[18], int32_T ia0)
{
  int32_T ijA;
  int32_T j;
  if (!(alpha1 == 0.0)) {
    int32_T jA;
    jA = ia0;
    for (j = 0; j < n; j++) {
      real_T temp;
      temp = y[j];
      if (temp != 0.0) {
        int32_T b;
        temp *= alpha1;
        b = m + jA;
        for (ijA = jA; ijA < b; ijA++) {
          A[ijA - 1] += A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += 6;
    }
  }
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void qrFactor_jq(const real_T A[9], real_T S[9], const real_T Ns[9])
{
  real_T b_A[18];
  real_T R[9];
  real_T tau[3];
  real_T work[3];
  int32_T coltop;
  int32_T i;
  int32_T ii;
  int32_T knt;
  int32_T lastv;
  mtimes(S, A, R);
  for (i = 0; i < 3; i++) {
    b_A[6 * i] = R[3 * i];
    b_A[6 * i + 3] = Ns[i];
    b_A[6 * i + 1] = R[3 * i + 1];
    b_A[6 * i + 4] = Ns[i + 3];
    b_A[6 * i + 2] = R[3 * i + 2];
    b_A[6 * i + 5] = Ns[i + 6];
    work[i] = 0.0;
  }

  for (i = 0; i < 3; i++) {
    real_T atmp;
    real_T beta1;
    int32_T c_tmp;
    ii = i * 6 + i;
    atmp = b_A[ii];
    lastv = ii + 2;
    tau[i] = 0.0;
    beta1 = xnrm2_gw(5 - i, b_A, ii + 2);
    if (beta1 != 0.0) {
      beta1 = rt_hypotd_snf(b_A[ii], beta1);
      if (b_A[ii] >= 0.0) {
        beta1 = -beta1;
      }

      if (fabs(beta1) < 1.0020841800044864E-292) {
        knt = 0;
        c_tmp = (ii - i) + 6;
        do {
          knt++;
          for (coltop = lastv; coltop <= c_tmp; coltop++) {
            b_A[coltop - 1] *= 9.9792015476736E+291;
          }

          beta1 *= 9.9792015476736E+291;
          atmp *= 9.9792015476736E+291;
        } while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));

        beta1 = rt_hypotd_snf(atmp, xnrm2_gw(5 - i, b_A, ii + 2));
        if (atmp >= 0.0) {
          beta1 = -beta1;
        }

        tau[i] = (beta1 - atmp) / beta1;
        atmp = 1.0 / (atmp - beta1);
        for (coltop = lastv; coltop <= c_tmp; coltop++) {
          b_A[coltop - 1] *= atmp;
        }

        for (lastv = 0; lastv < knt; lastv++) {
          beta1 *= 1.0020841800044864E-292;
        }

        atmp = beta1;
      } else {
        tau[i] = (beta1 - b_A[ii]) / beta1;
        atmp = 1.0 / (b_A[ii] - beta1);
        coltop = (ii - i) + 6;
        for (knt = lastv; knt <= coltop; knt++) {
          b_A[knt - 1] *= atmp;
        }

        atmp = beta1;
      }
    }

    b_A[ii] = atmp;
    if (i + 1 < 3) {
      beta1 = b_A[ii];
      b_A[ii] = 1.0;
      if (tau[i] != 0.0) {
        boolean_T exitg2;
        lastv = 6 - i;
        knt = (ii - i) + 5;
        while ((lastv > 0) && (b_A[knt] == 0.0)) {
          lastv--;
          knt--;
        }

        knt = 2 - i;
        exitg2 = false;
        while ((!exitg2) && (knt > 0)) {
          int32_T exitg1;
          coltop = ((knt - 1) * 6 + ii) + 6;
          c_tmp = coltop;
          do {
            exitg1 = 0;
            if (c_tmp + 1 <= coltop + lastv) {
              if (b_A[c_tmp] != 0.0) {
                exitg1 = 1;
              } else {
                c_tmp++;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        knt = 0;
      }

      if (lastv > 0) {
        xgemv_gp(lastv, knt, b_A, ii + 7, b_A, ii + 1, work);
        xgerc_aq(lastv, knt, -tau[i], ii + 1, work, b_A, ii + 7);
      }

      b_A[ii] = beta1;
    }
  }

  for (i = 0; i < 3; i++) {
    for (ii = 0; ii <= i; ii++) {
      R[ii + 3 * i] = b_A[6 * i + ii];
    }

    for (ii = i + 2; ii < 4; ii++) {
      R[(ii + 3 * i) - 1] = 0.0;
    }
  }

  for (i = 0; i < 3; i++) {
    S[3 * i] = R[i];
    S[3 * i + 1] = R[i + 3];
    S[3 * i + 2] = R[i + 6];
  }
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static real_T xnrm2_gwq(int32_T n, const real_T x[9], int32_T ix0)
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        real_T absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static real_T xdotc(int32_T n, const real_T x[9], int32_T ix0, const real_T y[9],
                    int32_T iy0)
{
  real_T d;
  int32_T k;
  d = 0.0;
  if (n >= 1) {
    for (k = 0; k < n; k++) {
      d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
    }
  }

  return d;
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void xaxpy(int32_T n, real_T a, int32_T ix0, real_T y[9], int32_T iy0)
{
  int32_T k;
  if ((n >= 1) && (!(a == 0.0))) {
    for (k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += y[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static real_T xnrm2_gwqu(int32_T n, const real_T x[3], int32_T ix0)
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        real_T absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void xaxpy_c(int32_T n, real_T a, const real_T x[9], int32_T ix0, real_T
                    y[3], int32_T iy0)
{
  int32_T k;
  if ((n >= 1) && (!(a == 0.0))) {
    for (k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void xaxpy_cs(int32_T n, real_T a, const real_T x[3], int32_T ix0, real_T
                     y[9], int32_T iy0)
{
  int32_T k;
  if ((n >= 1) && (!(a == 0.0))) {
    for (k = 0; k < n; k++) {
      int32_T tmp;
      tmp = (iy0 + k) - 1;
      y[tmp] += x[(ix0 + k) - 1] * a;
    }
  }
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void xswap(real_T x[9], int32_T ix0, int32_T iy0)
{
  real_T temp;
  temp = x[ix0 - 1];
  x[ix0 - 1] = x[iy0 - 1];
  x[iy0 - 1] = temp;
  temp = x[ix0];
  x[ix0] = x[iy0];
  x[iy0] = temp;
  temp = x[ix0 + 1];
  x[ix0 + 1] = x[iy0 + 1];
  x[iy0 + 1] = temp;
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void xrotg(real_T *a, real_T *b, real_T *c, real_T *s)
{
  real_T absa;
  real_T absb;
  real_T roe;
  real_T scale;
  roe = *b;
  absa = fabs(*a);
  absb = fabs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0) {
    *s = 0.0;
    *c = 1.0;
    *a = 0.0;
    *b = 0.0;
  } else {
    real_T ads;
    real_T bds;
    ads = absa / scale;
    bds = absb / scale;
    scale *= sqrt(ads * ads + bds * bds);
    if (roe < 0.0) {
      scale = -scale;
    }

    *c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      *b = *s;
    } else if (*c != 0.0) {
      *b = 1.0 / *c;
    } else {
      *b = 1.0;
    }

    *a = scale;
  }
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void xrot(real_T x[9], int32_T ix0, int32_T iy0, real_T c, real_T s)
{
  real_T temp;
  real_T temp_tmp;
  temp = x[iy0 - 1];
  temp_tmp = x[ix0 - 1];
  x[iy0 - 1] = temp * c - temp_tmp * s;
  x[ix0 - 1] = temp_tmp * c + temp * s;
  temp = x[ix0] * c + x[iy0] * s;
  x[iy0] = x[iy0] * c - x[ix0] * s;
  x[ix0] = temp;
  temp = x[iy0 + 1];
  temp_tmp = x[ix0 + 1];
  x[iy0 + 1] = temp * c - temp_tmp * s;
  x[ix0 + 1] = temp_tmp * c + temp * s;
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void svd(const real_T A[9], real_T U[9], real_T s[3], real_T V[9])
{
  real_T b_A[9];
  real_T b_s[3];
  real_T e[3];
  real_T work[3];
  real_T nrm;
  real_T rt;
  real_T smm1;
  real_T sqds;
  real_T ztest;
  int32_T kase;
  int32_T m;
  int32_T qjj;
  int32_T qp1;
  int32_T qq;
  b_s[0] = 0.0;
  e[0] = 0.0;
  work[0] = 0.0;
  b_s[1] = 0.0;
  e[1] = 0.0;
  work[1] = 0.0;
  b_s[2] = 0.0;
  e[2] = 0.0;
  work[2] = 0.0;
  for (m = 0; m < 9; m++) {
    b_A[m] = A[m];
    U[m] = 0.0;
    V[m] = 0.0;
  }

  for (m = 0; m < 2; m++) {
    boolean_T apply_transform;
    qp1 = m + 2;
    qq = (3 * m + m) + 1;
    apply_transform = false;
    nrm = xnrm2_gwq(3 - m, b_A, qq);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq - 1] < 0.0) {
        b_s[m] = -nrm;
      } else {
        b_s[m] = nrm;
      }

      if (fabs(b_s[m]) >= 1.0020841800044864E-292) {
        nrm = 1.0 / b_s[m];
        qjj = (qq - m) + 2;
        for (kase = qq; kase <= qjj; kase++) {
          b_A[kase - 1] *= nrm;
        }
      } else {
        qjj = (qq - m) + 2;
        for (kase = qq; kase <= qjj; kase++) {
          b_A[kase - 1] /= b_s[m];
        }
      }

      b_A[qq - 1]++;
      b_s[m] = -b_s[m];
    } else {
      b_s[m] = 0.0;
    }

    for (kase = qp1; kase < 4; kase++) {
      qjj = (kase - 1) * 3 + m;
      if (apply_transform) {
        xaxpy(3 - m, -(xdotc(3 - m, b_A, qq, b_A, qjj + 1) / b_A[m + 3 * m]), qq,
              b_A, qjj + 1);
      }

      e[kase - 1] = b_A[qjj];
    }

    for (qq = m + 1; qq < 4; qq++) {
      kase = (3 * m + qq) - 1;
      U[kase] = b_A[kase];
    }

    if (m + 1 <= 1) {
      nrm = xnrm2_gwqu(2, e, 2);
      if (nrm == 0.0) {
        e[0] = 0.0;
      } else {
        if (e[1] < 0.0) {
          e[0] = -nrm;
        } else {
          e[0] = nrm;
        }

        nrm = e[0];
        if (fabs(e[0]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[0];
          for (qq = qp1; qq < 4; qq++) {
            e[qq - 1] *= nrm;
          }
        } else {
          for (qq = qp1; qq < 4; qq++) {
            e[qq - 1] /= nrm;
          }
        }

        e[1]++;
        e[0] = -e[0];
        for (qq = qp1; qq < 4; qq++) {
          work[qq - 1] = 0.0;
        }

        for (qq = qp1; qq < 4; qq++) {
          xaxpy_c(2, e[qq - 1], b_A, 3 * (qq - 1) + 2, work, 2);
        }

        for (qq = qp1; qq < 4; qq++) {
          xaxpy_cs(2, -e[qq - 1] / e[1], work, 2, b_A, 3 * (qq - 1) + 2);
        }
      }

      for (qq = qp1; qq < 4; qq++) {
        V[qq - 1] = e[qq - 1];
      }
    }
  }

  m = 1;
  b_s[2] = b_A[8];
  e[1] = b_A[7];
  e[2] = 0.0;
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 1.0;
  for (qp1 = 1; qp1 >= 0; qp1--) {
    qq = 3 * qp1 + qp1;
    if (b_s[qp1] != 0.0) {
      for (kase = qp1 + 2; kase < 4; kase++) {
        qjj = ((kase - 1) * 3 + qp1) + 1;
        xaxpy(3 - qp1, -(xdotc(3 - qp1, U, qq + 1, U, qjj) / U[qq]), qq + 1, U,
              qjj);
      }

      for (qjj = qp1 + 1; qjj < 4; qjj++) {
        kase = (3 * qp1 + qjj) - 1;
        U[kase] = -U[kase];
      }

      U[qq]++;
      if (qp1 - 1 >= 0) {
        U[3 * qp1] = 0.0;
      }
    } else {
      U[3 * qp1] = 0.0;
      U[3 * qp1 + 1] = 0.0;
      U[3 * qp1 + 2] = 0.0;
      U[qq] = 1.0;
    }
  }

  for (qp1 = 2; qp1 >= 0; qp1--) {
    if ((qp1 + 1 <= 1) && (e[0] != 0.0)) {
      xaxpy(2, -(xdotc(2, V, 2, V, 5) / V[1]), 2, V, 5);
      xaxpy(2, -(xdotc(2, V, 2, V, 8) / V[1]), 2, V, 8);
    }

    V[3 * qp1] = 0.0;
    V[3 * qp1 + 1] = 0.0;
    V[3 * qp1 + 2] = 0.0;
    V[qp1 + 3 * qp1] = 1.0;
  }

  for (qp1 = 0; qp1 < 3; qp1++) {
    smm1 = e[qp1];
    if (b_s[qp1] != 0.0) {
      rt = fabs(b_s[qp1]);
      nrm = b_s[qp1] / rt;
      b_s[qp1] = rt;
      if (qp1 + 1 < 3) {
        smm1 /= nrm;
      }

      qq = 3 * qp1 + 1;
      for (qjj = qq; qjj <= qq + 2; qjj++) {
        U[qjj - 1] *= nrm;
      }
    }

    if ((qp1 + 1 < 3) && (smm1 != 0.0)) {
      rt = fabs(smm1);
      nrm = rt / smm1;
      smm1 = rt;
      b_s[qp1 + 1] *= nrm;
      qq = (qp1 + 1) * 3 + 1;
      for (qjj = qq; qjj <= qq + 2; qjj++) {
        V[qjj - 1] *= nrm;
      }
    }

    e[qp1] = smm1;
  }

  qp1 = 0;
  nrm = fmax(fmax(fmax(0.0, fmax(fabs(b_s[0]), fabs(e[0]))), fmax(fabs(b_s[1]),
    fabs(e[1]))), fmax(fabs(b_s[2]), fabs(e[2])));
  while ((m + 2 > 0) && (qp1 < 75)) {
    kase = m + 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      qq = kase;
      if (kase == 0) {
        exitg1 = 1;
      } else {
        rt = fabs(e[kase - 1]);
        if (rt <= (fabs(b_s[kase - 1]) + fabs(b_s[kase])) *
            2.2204460492503131E-16) {
          e[kase - 1] = 0.0;
          exitg1 = 1;
        } else if ((rt <= 1.0020841800044864E-292) || ((qp1 > 20) && (rt <=
                     2.2204460492503131E-16 * nrm))) {
          e[kase - 1] = 0.0;
          exitg1 = 1;
        } else {
          kase--;
        }
      }
    } while (exitg1 == 0);

    if (m + 1 == kase) {
      kase = 4;
    } else {
      int32_T k_ii;
      boolean_T exitg2;
      qjj = m + 2;
      k_ii = m + 2;
      exitg2 = false;
      while ((!exitg2) && (k_ii >= kase)) {
        qjj = k_ii;
        if (k_ii == kase) {
          exitg2 = true;
        } else {
          rt = 0.0;
          if (k_ii < m + 2) {
            rt = fabs(e[k_ii - 1]);
          }

          if (k_ii > kase + 1) {
            rt += fabs(e[k_ii - 2]);
          }

          ztest = fabs(b_s[k_ii - 1]);
          if ((ztest <= 2.2204460492503131E-16 * rt) || (ztest <=
               1.0020841800044864E-292)) {
            b_s[k_ii - 1] = 0.0;
            exitg2 = true;
          } else {
            k_ii--;
          }
        }
      }

      if (qjj == kase) {
        kase = 3;
      } else if (m + 2 == qjj) {
        kase = 1;
      } else {
        kase = 2;
        qq = qjj;
      }
    }

    switch (kase) {
     case 1:
      rt = e[m];
      e[m] = 0.0;
      for (qjj = m + 1; qjj >= qq + 1; qjj--) {
        smm1 = e[0];
        xrotg(&b_s[qjj - 1], &rt, &ztest, &sqds);
        if (qjj > qq + 1) {
          rt = -sqds * e[0];
          smm1 = e[0] * ztest;
        }

        xrot(V, 3 * (qjj - 1) + 1, 3 * (m + 1) + 1, ztest, sqds);
        e[0] = smm1;
      }
      break;

     case 2:
      {
        rt = e[qq - 1];
        e[qq - 1] = 0.0;
        for (qjj = qq + 1; qjj <= m + 2; qjj++) {
          real_T emm1;
          xrotg(&b_s[qjj - 1], &rt, &ztest, &sqds);
          emm1 = e[qjj - 1];
          rt = emm1 * -sqds;
          e[qjj - 1] = emm1 * ztest;
          xrot(U, 3 * (qjj - 1) + 1, 3 * (qq - 1) + 1, ztest, sqds);
        }
      }
      break;

     case 3:
      {
        real_T emm1;
        real_T shift;
        sqds = b_s[m + 1];
        ztest = fmax(fmax(fmax(fmax(fabs(sqds), fabs(b_s[m])), fabs(e[m])), fabs
                          (b_s[qq])), fabs(e[qq]));
        rt = sqds / ztest;
        smm1 = b_s[m] / ztest;
        emm1 = e[m] / ztest;
        sqds = b_s[qq] / ztest;
        smm1 = ((smm1 + rt) * (smm1 - rt) + emm1 * emm1) / 2.0;
        emm1 *= rt;
        emm1 *= emm1;
        if ((smm1 != 0.0) || (emm1 != 0.0)) {
          shift = sqrt(smm1 * smm1 + emm1);
          if (smm1 < 0.0) {
            shift = -shift;
          }

          shift = emm1 / (smm1 + shift);
        } else {
          shift = 0.0;
        }

        rt = (sqds + rt) * (sqds - rt) + shift;
        ztest = e[qq] / ztest * sqds;
        for (qjj = qq + 1; qjj <= m + 1; qjj++) {
          xrotg(&rt, &ztest, &sqds, &smm1);
          if (qjj > qq + 1) {
            e[0] = rt;
          }

          emm1 = e[qjj - 1];
          rt = b_s[qjj - 1];
          e[qjj - 1] = emm1 * sqds - rt * smm1;
          ztest = smm1 * b_s[qjj];
          b_s[qjj] *= sqds;
          xrot(V, 3 * (qjj - 1) + 1, 3 * qjj + 1, sqds, smm1);
          b_s[qjj - 1] = rt * sqds + emm1 * smm1;
          xrotg(&b_s[qjj - 1], &ztest, &sqds, &smm1);
          rt = e[qjj - 1] * sqds + smm1 * b_s[qjj];
          b_s[qjj] = e[qjj - 1] * -smm1 + sqds * b_s[qjj];
          ztest = smm1 * e[qjj];
          e[qjj] *= sqds;
          xrot(U, 3 * (qjj - 1) + 1, 3 * qjj + 1, sqds, smm1);
        }

        e[m] = rt;
        qp1++;
      }
      break;

     default:
      if (b_s[qq] < 0.0) {
        b_s[qq] = -b_s[qq];
        qp1 = 3 * qq + 1;
        for (qjj = qp1; qjj <= qp1 + 2; qjj++) {
          V[qjj - 1] = -V[qjj - 1];
        }
      }

      qp1 = qq + 1;
      while ((qq + 1 < 3) && (b_s[qq] < b_s[qp1])) {
        rt = b_s[qq];
        b_s[qq] = b_s[qp1];
        b_s[qp1] = rt;
        xswap(V, 3 * qq + 1, 3 * (qq + 1) + 1);
        xswap(U, 3 * qq + 1, 3 * (qq + 1) + 1);
        qq = qp1;
        qp1++;
      }

      qp1 = 0;
      m--;
      break;
    }
  }

  s[0] = b_s[0];
  s[1] = b_s[1];
  s[2] = b_s[2];
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void xgemv_gpk(int32_T m, int32_T n, const real_T A[9], int32_T ia0,
                      const real_T x[9], int32_T ix0, real_T y[3])
{
  int32_T b_iy;
  int32_T iyend;
  if ((m != 0) && (n != 0)) {
    int32_T b;
    if (n - 1 >= 0) {
      memset(&y[0], 0, (uint32_T)n * sizeof(real_T));
    }

    b = (n - 1) * 3 + ia0;
    for (b_iy = ia0; b_iy <= b; b_iy += 3) {
      real_T c;
      int32_T d;
      c = 0.0;
      d = b_iy + m;
      for (iyend = b_iy; iyend < d; iyend++) {
        c += x[((ix0 + iyend) - b_iy) - 1] * A[iyend - 1];
      }

      iyend = div_nde_s32_floor(b_iy - ia0, 3);
      y[iyend] += c;
    }
  }
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void xgerc_aq4(int32_T m, int32_T n, real_T alpha1, int32_T ix0, const
                      real_T y[3], real_T A[9], int32_T ia0)
{
  int32_T ijA;
  int32_T j;
  if (!(alpha1 == 0.0)) {
    int32_T jA;
    jA = ia0;
    for (j = 0; j < n; j++) {
      real_T temp;
      temp = y[j];
      if (temp != 0.0) {
        int32_T b;
        temp *= alpha1;
        b = m + jA;
        for (ijA = jA; ijA < b; ijA++) {
          A[ijA - 1] += A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += 3;
    }
  }
}

/* Function for MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
static void qr(const real_T A[9], real_T Q[9], real_T R[9])
{
  real_T b_A[9];
  real_T tau[3];
  real_T work[3];
  int32_T b_coltop;
  int32_T b_lastv;
  int32_T coltop;
  int32_T exitg1;
  int32_T ii;
  int32_T itau;
  int32_T knt;
  boolean_T exitg2;
  memcpy(&b_A[0], &A[0], 9U * sizeof(real_T));
  tau[0] = 0.0;
  work[0] = 0.0;
  tau[1] = 0.0;
  work[1] = 0.0;
  tau[2] = 0.0;
  work[2] = 0.0;
  for (itau = 0; itau < 3; itau++) {
    ii = itau * 3 + itau;
    if (itau + 1 < 3) {
      real_T atmp;
      real_T beta1;
      atmp = b_A[ii];
      b_lastv = ii + 2;
      tau[itau] = 0.0;
      beta1 = xnrm2_gwq(2 - itau, b_A, ii + 2);
      if (beta1 != 0.0) {
        beta1 = rt_hypotd_snf(b_A[ii], beta1);
        if (b_A[ii] >= 0.0) {
          beta1 = -beta1;
        }

        if (fabs(beta1) < 1.0020841800044864E-292) {
          knt = 0;
          b_coltop = (ii - itau) + 3;
          do {
            knt++;
            for (coltop = b_lastv; coltop <= b_coltop; coltop++) {
              b_A[coltop - 1] *= 9.9792015476736E+291;
            }

            beta1 *= 9.9792015476736E+291;
            atmp *= 9.9792015476736E+291;
          } while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));

          beta1 = rt_hypotd_snf(atmp, xnrm2_gwq(2 - itau, b_A, ii + 2));
          if (atmp >= 0.0) {
            beta1 = -beta1;
          }

          tau[itau] = (beta1 - atmp) / beta1;
          atmp = 1.0 / (atmp - beta1);
          for (coltop = b_lastv; coltop <= b_coltop; coltop++) {
            b_A[coltop - 1] *= atmp;
          }

          for (b_lastv = 0; b_lastv < knt; b_lastv++) {
            beta1 *= 1.0020841800044864E-292;
          }

          atmp = beta1;
        } else {
          tau[itau] = (beta1 - b_A[ii]) / beta1;
          atmp = 1.0 / (b_A[ii] - beta1);
          knt = (ii - itau) + 3;
          for (b_coltop = b_lastv; b_coltop <= knt; b_coltop++) {
            b_A[b_coltop - 1] *= atmp;
          }

          atmp = beta1;
        }
      }

      b_A[ii] = atmp;
      beta1 = b_A[ii];
      b_A[ii] = 1.0;
      if (tau[itau] != 0.0) {
        b_lastv = 3 - itau;
        knt = (ii - itau) + 2;
        while ((b_lastv > 0) && (b_A[knt] == 0.0)) {
          b_lastv--;
          knt--;
        }

        knt = 2 - itau;
        exitg2 = false;
        while ((!exitg2) && (knt > 0)) {
          b_coltop = ((knt - 1) * 3 + ii) + 3;
          coltop = b_coltop;
          do {
            exitg1 = 0;
            if (coltop + 1 <= b_coltop + b_lastv) {
              if (b_A[coltop] != 0.0) {
                exitg1 = 1;
              } else {
                coltop++;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        b_lastv = 0;
        knt = 0;
      }

      if (b_lastv > 0) {
        xgemv_gpk(b_lastv, knt, b_A, ii + 4, b_A, ii + 1, work);
        xgerc_aq4(b_lastv, knt, -tau[itau], ii + 1, work, b_A, ii + 4);
      }

      b_A[ii] = beta1;
    } else {
      tau[2] = 0.0;
    }
  }

  for (itau = 0; itau < 3; itau++) {
    for (ii = 0; ii <= itau; ii++) {
      R[ii + 3 * itau] = b_A[3 * itau + ii];
    }

    for (ii = itau + 2; ii < 4; ii++) {
      R[(ii + 3 * itau) - 1] = 0.0;
    }

    work[itau] = 0.0;
  }

  for (ii = 2; ii >= 0; ii--) {
    b_lastv = (ii * 3 + ii) + 4;
    if (ii + 1 < 3) {
      b_A[b_lastv - 4] = 1.0;
      if (tau[ii] != 0.0) {
        knt = 3 - ii;
        b_coltop = b_lastv - ii;
        while ((knt > 0) && (b_A[b_coltop - 2] == 0.0)) {
          knt--;
          b_coltop--;
        }

        b_coltop = 2 - ii;
        exitg2 = false;
        while ((!exitg2) && (b_coltop > 0)) {
          coltop = (b_coltop - 1) * 3 + b_lastv;
          itau = coltop;
          do {
            exitg1 = 0;
            if (itau <= (coltop + knt) - 1) {
              if (b_A[itau - 1] != 0.0) {
                exitg1 = 1;
              } else {
                itau++;
              }
            } else {
              b_coltop--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        knt = 0;
        b_coltop = 0;
      }

      if (knt > 0) {
        xgemv_gpk(knt, b_coltop, b_A, b_lastv, b_A, b_lastv - 3, work);
        xgerc_aq4(knt, b_coltop, -tau[ii], b_lastv - 3, work, b_A, b_lastv);
      }

      knt = (b_lastv - ii) - 1;
      for (b_coltop = b_lastv - 2; b_coltop <= knt; b_coltop++) {
        b_A[b_coltop - 1] *= -tau[ii];
      }
    }

    b_A[b_lastv - 4] = 1.0 - tau[ii];
    for (b_coltop = 0; b_coltop < ii; b_coltop++) {
      b_A[(b_lastv - b_coltop) - 5] = 0.0;
    }
  }

  for (itau = 0; itau < 3; itau++) {
    Q[3 * itau] = b_A[3 * itau];
    ii = 3 * itau + 1;
    Q[ii] = b_A[ii];
    ii = 3 * itau + 2;
    Q[ii] = b_A[ii];
  }
}

real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  uint32_T hi;
  uint32_T lo;

  /* Uniform random number generator (random number between 0 and 1)

     #define IA      16807                      magic multiplier = 7^5
     #define IM      2147483647                 modulus = 2^31-1
     #define IQ      127773                     IM div IA
     #define IR      2836                       IM modulo IA
     #define S       4.656612875245797e-10      reciprocal of 2^31-1
     test = IA * (seed % IQ) - IR * (seed/IQ)
     seed = test < 0 ? (test + IM) : test
     return (seed*S)
   */
  lo = *u % 127773U * 16807U;
  hi = *u / 127773U * 2836U;
  if (lo < hi) {
    *u = 2147483647U - (hi - lo);
  } else {
    *u = lo - hi;
  }

  return (real_T)*u * 4.6566128752457969E-10;
}

real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  real_T si;
  real_T sr;
  real_T y;

  /* Normal (Gaussian) random number generator */
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = sqrt(-2.0 * log(si) / si) * sr;
  return y;
}

/* Model step function */
void RC_Model_KF_Vout_Vcb_for_MCU_step(void)
{
  real_T Hnew_0[9];
  real_T Ss[9];
  real_T rtb_A[9];
  real_T rtb_SNew[9];
  real_T rtb_SNew_0[9];
  real_T rtb_Zs[9];
  real_T Hnew[6];
  real_T f[6];
  real_T rtb_C[6];
  real_T rtb_L[6];
  real_T rtb_M[6];
  real_T RsInv[4];
  real_T yCovSqrt[4];
  real_T yCovSqrt_0[4];
  real_T c[3];
  real_T rtb_Add_d[3];
  real_T s[3];
  real_T rtb_Reshapey[2];
  real_T rtb_Reshapey_0[2];
  real_T a_vo;
  real_T a_vo_tmp_tmp;
  real_T absxk;
  real_T b_t;
  real_T r;
  real_T rho;
  real_T rtb_Cbulk;
  real_T rtb_Csurface;
  real_T rtb_Rb;
  real_T rtb_Rt;
  real_T t;
  int32_T Hnew_tmp;
  int32_T Ss_tmp;
  int32_T i;
  int32_T iAcol;
  static const int8_T tmp[6] = { 0, 1, 0, 0, 1, 0 };

  /* Delay: '<S2>/MemoryP' incorporates:
   *  Constant: '<S2>/P0'
   */
  if (rtDW.icLoad) {
    memcpy(&rtDW.MemoryP_DSTATE[0], &rtConstP.P0_Value[0], 9U * sizeof(real_T));
  }

  /* Lookup_n-D: '<Root>/Rt' incorporates:
   *  Delay: '<Root>/Delay'
   */
  rtb_Rt = look1_binlca(rtDW.Delay_DSTATE, rtConstP.pooled3,
                        rtConstP.Rt_tableData, 11U);

  /* Lookup_n-D: '<Root>/Rb' incorporates:
   *  Delay: '<Root>/Delay'
   */
  rtb_Rb = look1_binlca(rtDW.Delay_DSTATE, rtConstP.pooled3, rtConstP.pooled4,
                        11U);

  /* Lookup_n-D: '<Root>/Cbulk' incorporates:
   *  Delay: '<Root>/Delay'
   */
  rtb_Cbulk = look1_binlca(rtDW.Delay_DSTATE, rtConstP.pooled3,
    rtConstP.Cbulk_tableData, 11U);

  /* Lookup_n-D: '<Root>/Csurface' incorporates:
   *  Delay: '<Root>/Delay'
   */
  rtb_Csurface = look1_binlca(rtDW.Delay_DSTATE, rtConstP.pooled3,
    rtConstP.Csurface_tableData, 11U);

  /* MATLAB Function: '<Root>/Matrix Generator' */
  for (i = 0; i < 6; i++) {
    rtb_C[i] = tmp[i];
  }

  b_t = 2.0 * rtb_Cbulk * rtb_Rb;
  a_vo_tmp_tmp = 2.0 * rtb_Csurface * rtb_Rb;
  rho = 1.0 / a_vo_tmp_tmp;
  a_vo = 1.0 / b_t - rho;
  r = exp(-0.1 / (2.0 * rtb_Rb * rtb_Cbulk));
  t = exp(-0.1 / (2.0 * rtb_Rb * rtb_Csurface));
  absxk = exp(-a_vo * 0.1);
  rtb_A[0] = r;
  rtb_A[3] = 1.0 - r;
  rtb_A[6] = 0.0;
  rtb_A[1] = 1.0 - t;
  rtb_A[4] = t;
  rtb_A[7] = 0.0;
  rtb_A[2] = (-1.0 / b_t + rho) * (1.0 / a_vo * (absxk - 1.0));
  rtb_A[5] = 0.0;
  rtb_A[8] = absxk;

  /* Outputs for Atomic SubSystem: '<S2>/CalculatePL' */
  /* MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' incorporates:
   *  Constant: '<S24>/Bbar_'
   *  Constant: '<S24>/Qbar_'
   *  Constant: '<S24>/Rbar_'
   *  Delay: '<S2>/MemoryP'
   */
  qrFactor(rtb_C, rtDW.MemoryP_DSTATE, rtConstP.Rbar_Value, yCovSqrt);
  for (i = 0; i < 3; i++) {
    for (iAcol = 0; iAcol < 3; iAcol++) {
      Hnew_tmp = 3 * iAcol + i;
      Ss[Hnew_tmp] = 0.0;
      Ss[Hnew_tmp] += rtDW.MemoryP_DSTATE[i] * rtDW.MemoryP_DSTATE[iAcol];
      Ss[Hnew_tmp] += rtDW.MemoryP_DSTATE[i + 3] * rtDW.MemoryP_DSTATE[iAcol + 3];
      Ss[Hnew_tmp] += rtDW.MemoryP_DSTATE[i + 6] * rtDW.MemoryP_DSTATE[iAcol + 6];
    }

    for (iAcol = 0; iAcol < 2; iAcol++) {
      Hnew_tmp = 3 * iAcol + i;
      Hnew[Hnew_tmp] = 0.0;
      Hnew[Hnew_tmp] += Ss[i] * rtb_C[iAcol];
      Hnew[Hnew_tmp] += Ss[i + 3] * rtb_C[iAcol + 2];
      Hnew[Hnew_tmp] += Ss[i + 6] * rtb_C[iAcol + 4];
    }
  }

  for (i = 0; i < 2; i++) {
    for (iAcol = 0; iAcol < 3; iAcol++) {
      rtb_L[i + (iAcol << 1)] = (Hnew[3 * i + 1] * rtb_A[iAcol + 3] + Hnew[3 * i]
        * rtb_A[iAcol]) + Hnew[3 * i + 2] * rtb_A[iAcol + 6];
    }
  }

  linsolve(yCovSqrt, rtb_L, rtb_M);
  yCovSqrt_0[0] = yCovSqrt[0];
  yCovSqrt_0[1] = yCovSqrt[2];
  yCovSqrt_0[2] = yCovSqrt[1];
  yCovSqrt_0[3] = yCovSqrt[3];
  linsolve_b(yCovSqrt_0, rtb_M, f);
  for (i = 0; i < 2; i++) {
    rtb_L[3 * i] = f[i];
    rtb_L[3 * i + 1] = f[i + 2];
    rtb_L[3 * i + 2] = f[i + 4];
  }

  for (i = 0; i < 3; i++) {
    Hnew_tmp = i << 1;
    f[Hnew_tmp] = Hnew[i];
    f[Hnew_tmp + 1] = Hnew[i + 3];
  }

  linsolve(yCovSqrt, f, rtb_M);
  yCovSqrt_0[0] = yCovSqrt[0];
  yCovSqrt_0[1] = yCovSqrt[2];
  yCovSqrt_0[2] = yCovSqrt[1];
  yCovSqrt_0[3] = yCovSqrt[3];
  linsolve_b(yCovSqrt_0, rtb_M, f);
  for (i = 0; i < 2; i++) {
    rtb_M[3 * i] = f[i];
    rtb_M[3 * i + 1] = f[i + 2];
    rtb_M[3 * i + 2] = f[i + 4];
  }

  for (i = 0; i < 9; i++) {
    Ss[i] = 0.0;
    rtb_Zs[i] = rtDW.MemoryP_DSTATE[i];
  }

  Ss[0] = 1.0;
  Ss[4] = 1.0;
  Ss[8] = 1.0;
  for (i = 0; i < 3; i++) {
    for (iAcol = 0; iAcol < 3; iAcol++) {
      Hnew_tmp = iAcol << 1;
      Ss_tmp = 3 * iAcol + i;
      rtb_SNew[Ss_tmp] = Ss[Ss_tmp] - (rtb_C[Hnew_tmp + 1] * rtb_M[i + 3] +
        rtb_C[Hnew_tmp] * rtb_M[i]);
    }

    Hnew[i] = 0.0;
    Hnew[i] += rtb_M[i] * 0.31622776601683794;
    r = rtb_M[i + 3];
    Hnew[i] += r * 0.0;
    Hnew[i + 3] = 0.0;
    Hnew[i + 3] += rtb_M[i] * 0.0;
    Hnew[i + 3] += r * 0.31622776601683794;
  }

  qrFactor_j(rtb_SNew, rtb_Zs, Hnew);
  RsInv[0] = 3.1622776601683791;
  RsInv[1] = -0.0;
  RsInv[2] = -0.0;
  RsInv[3] = 3.1622776601683791;
  for (i = 0; i < 3; i++) {
    Hnew[i] = 0.0;
    Hnew[i + 3] = 0.0;
  }

  memcpy(&Ss[0], &rtb_Zs[0], 9U * sizeof(real_T));
  for (i = 0; i < 2; i++) {
    yCovSqrt[i] = 0.0;
    iAcol = i << 1;
    r = RsInv[iAcol];
    yCovSqrt[i] += r * 3.1622776601683791;
    t = RsInv[iAcol + 1];
    yCovSqrt[i] += t * -0.0;
    yCovSqrt[i + 2] = 0.0;
    yCovSqrt[i + 2] += r * -0.0;
    yCovSqrt[i + 2] += t * 3.1622776601683791;
  }

  for (i = 0; i < 3; i++) {
    f[i] = 0.0;
    f[i] += 0.0 * yCovSqrt[0];
    f[i] += 0.0 * yCovSqrt[1];
    f[i + 3] = 0.0;
    f[i + 3] += 0.0 * yCovSqrt[2];
    f[i + 3] += 0.0 * yCovSqrt[3];
    for (iAcol = 0; iAcol < 3; iAcol++) {
      Hnew_tmp = iAcol << 1;
      Ss_tmp = 3 * iAcol + i;
      rtb_SNew[Ss_tmp] = rtb_A[Ss_tmp] - (rtb_C[Hnew_tmp + 1] * f[i + 3] +
        rtb_C[Hnew_tmp] * f[i]);
    }
  }

  qrFactor_jq(rtb_SNew, Ss, rtConstP.Qbar_Value);
  for (i = 0; i < 3; i++) {
    rtb_SNew[3 * i] = Ss[i];
    rtb_SNew[3 * i + 1] = Ss[i + 3];
    rtb_SNew[3 * i + 2] = Ss[i + 6];
  }

  for (Hnew_tmp = 0; Hnew_tmp < 2; Hnew_tmp++) {
    int8_T p;
    boolean_T errorCondition;
    for (i = 0; i < 3; i++) {
      if (1 - i >= 0) {
        memset(&rtb_SNew[(i << 2) + 1], 0, (uint32_T)((1 - i) + 1) * sizeof
               (real_T));
      }
    }

    p = 0;
    errorCondition = (rtb_SNew[0] == 0.0);
    if (!errorCondition) {
      errorCondition = (rtb_SNew[4] == 0.0);
    }

    if (!errorCondition) {
      errorCondition = (rtb_SNew[8] == 0.0);
    }

    if (errorCondition) {
      p = 2;
    } else {
      rtb_Add_d[0] = Hnew[3 * Hnew_tmp];
      rtb_Add_d[1] = Hnew[3 * Hnew_tmp + 1];
      rtb_Add_d[2] = Hnew[3 * Hnew_tmp + 2];
      for (i = 0; i < 3; i++) {
        iAcol = 3 * i;
        r = rtb_Add_d[i];
        for (Ss_tmp = 0; Ss_tmp < i; Ss_tmp++) {
          r -= rtb_SNew[Ss_tmp + iAcol] * rtb_Add_d[Ss_tmp];
        }

        rtb_Add_d[i] = r / rtb_SNew[i + iAcol];
      }

      t = 3.3121686421112381E-170;
      absxk = fabs(rtb_Add_d[0]);
      if (absxk > 3.3121686421112381E-170) {
        r = 1.0;
        t = absxk;
      } else {
        b_t = absxk / 3.3121686421112381E-170;
        r = b_t * b_t;
      }

      absxk = fabs(rtb_Add_d[1]);
      if (absxk > t) {
        b_t = t / absxk;
        r = r * b_t * b_t + 1.0;
        t = absxk;
      } else {
        b_t = absxk / t;
        r += b_t * b_t;
      }

      rho = fabs(rtb_Add_d[2]);
      if (rho > t) {
        b_t = t / rho;
        r = r * b_t * b_t + 1.0;
        t = rho;
      } else {
        b_t = rho / t;
        r += b_t * b_t;
      }

      r = t * sqrt(r);
      if (r >= 1.0) {
        p = 1;
      } else {
        absxk = sqrt(1.0 - r * r);
        if (rho == 0.0) {
          c[2] = 1.0;
          s[2] = 0.0;
        } else {
          t = absxk + rho;
          absxk /= t;
          b_t = rtb_Add_d[2] / t;
          rho = rt_hypotd_snf(absxk, fabs(b_t));
          c[2] = absxk / rho;
          absxk /= absxk;
          s[2] = absxk * b_t / rho;
          absxk *= rho * t;
        }

        rtb_Add_d[2] = 0.0;
        t = fabs(rtb_Add_d[1]);
        if (t == 0.0) {
          c[1] = 1.0;
          s[1] = 0.0;
        } else if (absxk == 0.0) {
          c[1] = 0.0;
          s[1] = 1.0;
          absxk = rtb_Add_d[1];
        } else {
          t += absxk;
          absxk /= t;
          b_t = rtb_Add_d[1] / t;
          rho = rt_hypotd_snf(absxk, fabs(b_t));
          c[1] = absxk / rho;
          absxk /= absxk;
          s[1] = absxk * b_t / rho;
          absxk *= rho * t;
        }

        rtb_Add_d[1] = 0.0;
        r = fabs(absxk);
        t = fabs(rtb_Add_d[0]);
        if (t == 0.0) {
          c[0] = 1.0;
          s[0] = 0.0;
        } else if (r == 0.0) {
          c[0] = 0.0;
          s[0] = 1.0;
        } else {
          t += r;
          absxk /= t;
          b_t = rtb_Add_d[0] / t;
          r = fabs(absxk);
          rho = rt_hypotd_snf(r, fabs(b_t));
          c[0] = r / rho;
          s[0] = absxk / r * b_t / rho;
        }

        rtb_Add_d[0] = 0.0;
        for (i = 0; i < 3; i++) {
          for (iAcol = i + 1; iAcol >= 1; iAcol--) {
            r = rtb_Add_d[i];
            Ss_tmp = (3 * i + iAcol) - 1;
            t = rtb_SNew[Ss_tmp];
            absxk = s[iAcol - 1];
            b_t = c[iAcol - 1];
            rtb_SNew[Ss_tmp] = t * b_t - absxk * r;
            rtb_Add_d[i] = b_t * r + t * absxk;
          }
        }
      }
    }

    if (p != 0) {
      boolean_T exitg2;
      for (i = 0; i < 3; i++) {
        for (iAcol = 0; iAcol < 3; iAcol++) {
          Ss_tmp = 3 * iAcol + i;
          rtb_SNew_0[Ss_tmp] = 0.0;
          rtb_SNew_0[Ss_tmp] += rtb_SNew[3 * i] * rtb_SNew[3 * iAcol];
          rtb_SNew_0[Ss_tmp] += rtb_SNew[3 * i + 1] * rtb_SNew[3 * iAcol + 1];
          rtb_SNew_0[Ss_tmp] += rtb_SNew[3 * i + 2] * rtb_SNew[3 * iAcol + 2];
          Hnew_0[iAcol + 3 * i] = Hnew[3 * Hnew_tmp + iAcol] * Hnew[3 * Hnew_tmp
            + i];
        }
      }

      errorCondition = true;
      for (i = 0; i < 9; i++) {
        r = rtb_SNew_0[i] - Hnew_0[i];
        if (errorCondition && (rtIsInf(r) || rtIsNaN(r))) {
          errorCondition = false;
        }

        Ss[i] = r;
      }

      if (errorCondition) {
        svd(Ss, Hnew_0, s, rtb_SNew_0);
      } else {
        s[0] = (rtNaN);
        s[1] = (rtNaN);
        s[2] = (rtNaN);
        for (i = 0; i < 9; i++) {
          rtb_SNew_0[i] = (rtNaN);
        }
      }

      memset(&Ss[0], 0, 9U * sizeof(real_T));
      Ss[0] = s[0];
      Ss[4] = s[1];
      Ss[8] = s[2];
      for (i = 0; i < 9; i++) {
        Ss[i] = sqrt(Ss[i]);
      }

      for (i = 0; i < 3; i++) {
        for (iAcol = 0; iAcol < 3; iAcol++) {
          Ss_tmp = 3 * iAcol + i;
          rtb_SNew[Ss_tmp] = 0.0;
          rtb_SNew[Ss_tmp] += Ss[3 * i] * rtb_SNew_0[iAcol];
          rtb_SNew[Ss_tmp] += Ss[3 * i + 1] * rtb_SNew_0[iAcol + 3];
          rtb_SNew[Ss_tmp] += Ss[3 * i + 2] * rtb_SNew_0[iAcol + 6];
        }
      }

      errorCondition = true;
      i = 0;
      exitg2 = false;
      while ((!exitg2) && (i < 3)) {
        int32_T exitg1;
        iAcol = i + 1;
        do {
          exitg1 = 0;
          if (iAcol + 1 < 4) {
            if (!(rtb_SNew[3 * i + iAcol] == 0.0)) {
              errorCondition = false;
              exitg1 = 1;
            } else {
              iAcol++;
            }
          } else {
            i++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }

      if (!errorCondition) {
        memcpy(&rtb_SNew_0[0], &rtb_SNew[0], 9U * sizeof(real_T));
        qr(rtb_SNew_0, Hnew_0, rtb_SNew);
      }
    }
  }

  for (i = 0; i < 3; i++) {
    rtb_SNew_0[3 * i] = rtb_SNew[i];
    rtb_SNew_0[3 * i + 1] = rtb_SNew[i + 3];
    rtb_SNew_0[3 * i + 2] = rtb_SNew[i + 6];
  }

  memcpy(&rtb_SNew[0], &rtb_SNew_0[0], 9U * sizeof(real_T));

  /* End of MATLAB Function: '<S4>/Discrete-Time SqrtKF - Calculate SLMZ' */
  /* End of Outputs for SubSystem: '<S2>/CalculatePL' */

  /* Product: '<Root>/Divide9' incorporates:
   *  Constant: '<Root>/Battery Capacity Ah'
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   */
  r = rtDW.DiscreteTimeIntegrator_DSTATE / 5.0;

  /* Reshape: '<S2>/Reshapey' incorporates:
   *  Gain: '<S1>/Output'
   *  Inport: '<Root>/voltage'
   *  Lookup_n-D: '<Root>/1-D Lookup Table1'
   *  RandomNumber: '<S1>/White Noise'
   *  Sum: '<Root>/Add1'
   */
  rtb_Reshapey[0] = rtU.voltage;
  rtb_Reshapey[1] = look1_binlg(0.001 * rtDW.NextOutput + r, rtConstP.pooled3,
    rtConstP.pooled6, 11U);

  /* Delay: '<S2>/MemoryX' incorporates:
   *  Constant: '<S2>/X0'
   */
  if (rtDW.icLoad_o) {
    rtDW.MemoryX_DSTATE[0] = 0.0;
    rtDW.MemoryX_DSTATE[1] = 0.0;
    rtDW.MemoryX_DSTATE[2] = 0.0;
  }

  /* Outputs for Enabled SubSystem: '<S29>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S52>/Enable'
   */
  /* Sum: '<S52>/Add1' incorporates:
   *  Delay: '<S2>/MemoryX'
   *  Inport: '<Root>/current'
   *  MATLAB Function: '<Root>/Matrix Generator'
   *  Product: '<S52>/Product'
   *  Product: '<S52>/Product1'
   */
  for (i = 0; i < 2; i++) {
    rtb_Reshapey_0[i] = (rtb_Reshapey[i] - ((rtb_C[i + 2] * rtDW.MemoryX_DSTATE
      [1] + rtb_C[i] * rtDW.MemoryX_DSTATE[0]) + rtb_C[i + 4] *
      rtDW.MemoryX_DSTATE[2])) - 0.0 * rtU.current;
  }

  /* End of Sum: '<S52>/Add1' */
  for (i = 0; i < 3; i++) {
    /* Sum: '<S29>/Add' incorporates:
     *  Delay: '<S2>/MemoryX'
     *  Product: '<S52>/Product2'
     */
    rtb_Add_d[i] = (rtb_M[i + 3] * rtb_Reshapey_0[1] + rtb_M[i] *
                    rtb_Reshapey_0[0]) + rtDW.MemoryX_DSTATE[i];
  }

  /* End of Outputs for SubSystem: '<S29>/Enabled Subsystem' */

  /* Outputs for Enabled SubSystem: '<S23>/MeasurementUpdate' incorporates:
   *  EnablePort: '<S48>/Enable'
   */
  /* Sum: '<S48>/Sum' incorporates:
   *  Delay: '<S2>/MemoryX'
   *  Inport: '<Root>/current'
   *  MATLAB Function: '<Root>/Matrix Generator'
   *  Product: '<S48>/C[k]*xhat[k|k-1]'
   *  Product: '<S48>/D[k]*u[k]'
   *  Sum: '<S48>/Add1'
   */
  for (i = 0; i < 2; i++) {
    rtb_Reshapey_0[i] = rtb_Reshapey[i] - (((rtb_C[i + 2] * rtDW.MemoryX_DSTATE
      [1] + rtb_C[i] * rtDW.MemoryX_DSTATE[0]) + rtb_C[i + 4] *
      rtDW.MemoryX_DSTATE[2]) + 0.0 * rtU.current);
  }

  /* End of Sum: '<S48>/Sum' */
  for (i = 0; i < 3; i++) {
    /* Product: '<S48>/Product3' */
    c[i] = rtb_L[i + 3] * rtb_Reshapey_0[1] + rtb_L[i] * rtb_Reshapey_0[0];
  }

  /* End of Outputs for SubSystem: '<S23>/MeasurementUpdate' */

  /* Outport: '<Root>/voltage_estimated' incorporates:
   *  Inport: '<Root>/current'
   *  MATLAB Function: '<Root>/Matrix Generator'
   *  Product: '<S5>/Product'
   *  Product: '<S5>/Product1'
   *  Sum: '<S5>/Add1'
   */
  for (i = 0; i < 2; i++) {
    rtY.voltage_estimated[i] = ((rtb_C[i + 2] * rtb_Add_d[1] + rtb_C[i] *
      rtb_Add_d[0]) + rtb_C[i + 4] * rtb_Add_d[2]) + 0.0 * rtU.current;
  }

  /* End of Outport: '<Root>/voltage_estimated' */

  /* Lookup_n-D: '<Root>/1-D Lookup Table' */
  t = look1_binlg(rtb_Add_d[0], rtConstP.pooled6, rtConstP.pooled3, 11U);

  /* Saturate: '<Root>/Saturation' */
  if (t > 1.0) {
    t = 1.0;
  } else if (t < 0.0) {
    t = 0.0;
  }

  /* End of Saturate: '<Root>/Saturation' */

  /* Outport: '<Root>/soc_estimated' */
  rtY.soc_estimated = t;

  /* Outport: '<Root>/soc_measured' */
  rtY.soc_measured = r;

  /* MATLAB Function: '<S46>/SqrtUsedFcn' */
  for (i = 0; i < 3; i++) {
    for (iAcol = 0; iAcol < 3; iAcol++) {
      Ss[i + 3 * iAcol] = (rtb_Zs[i + 3] * rtb_Zs[iAcol + 3] + rtb_Zs[i] *
                           rtb_Zs[iAcol]) + rtb_Zs[i + 6] * rtb_Zs[iAcol + 6];
    }
  }

  /* End of MATLAB Function: '<S46>/SqrtUsedFcn' */

  /* Outport: '<Root>/standard_deviation' incorporates:
   *  Gain: '<Root>/Gain'
   *  Selector: '<Root>/Selector2'
   *  Sqrt: '<Root>/Sqrt'
   */
  rtY.standard_deviation = 3.0 * sqrt(Ss[0]);

  /* Update for Delay: '<S2>/MemoryP' */
  rtDW.icLoad = false;
  memcpy(&rtDW.MemoryP_DSTATE[0], &rtb_SNew[0], 9U * sizeof(real_T));

  /* Update for Delay: '<Root>/Delay' */
  rtDW.Delay_DSTATE = t;

  /* Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' incorporates:
   *  Gain: '<Root>/Convert to Ah'
   *  Inport: '<Root>/current'
   */
  rtDW.DiscreteTimeIntegrator_DSTATE += 0.00027777777777777778 * rtU.current *
    0.1;
  if (rtDW.DiscreteTimeIntegrator_DSTATE >= 5.0) {
    rtDW.DiscreteTimeIntegrator_DSTATE = 5.0;
  } else if (rtDW.DiscreteTimeIntegrator_DSTATE <= 0.0) {
    rtDW.DiscreteTimeIntegrator_DSTATE = 0.0;
  }

  /* End of Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */

  /* Update for RandomNumber: '<S1>/White Noise' */
  rtDW.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf(&rtDW.RandSeed);

  /* Update for Delay: '<S2>/MemoryX' */
  rtDW.icLoad_o = false;

  /* Product: '<S23>/A[k]*xhat[k|k-1]' incorporates:
   *  Delay: '<S2>/MemoryX'
   */
  for (i = 0; i < 3; i++) {
    rtb_Add_d[i] = (rtb_A[i + 3] * rtDW.MemoryX_DSTATE[1] + rtb_A[i] *
                    rtDW.MemoryX_DSTATE[0]) + rtb_A[i + 6] *
      rtDW.MemoryX_DSTATE[2];
  }

  /* End of Product: '<S23>/A[k]*xhat[k|k-1]' */

  /* Update for Delay: '<S2>/MemoryX' incorporates:
   *  Inport: '<Root>/current'
   *  MATLAB Function: '<Root>/Matrix Generator'
   *  Product: '<S23>/B[k]*u[k]'
   *  Sum: '<S23>/Add'
   */
  rtDW.MemoryX_DSTATE[0] = ((1.0 - exp(-0.1 / (2.0 * rtb_Rb * rtb_Cbulk))) *
    rtb_Rb * rtU.current + rtb_Add_d[0]) + c[0];
  rtDW.MemoryX_DSTATE[1] = ((1.0 - exp(-0.1 / (2.0 * rtb_Rb * rtb_Csurface))) *
    rtb_Rb * rtU.current + rtb_Add_d[1]) + c[1];

  /* MATLAB Function: '<Root>/Matrix Generator' */
  rtb_Rt /= a_vo_tmp_tmp;

  /* Update for Delay: '<S2>/MemoryX' incorporates:
   *  Inport: '<Root>/current'
   *  MATLAB Function: '<Root>/Matrix Generator'
   *  Product: '<S23>/B[k]*u[k]'
   *  Sum: '<S23>/Add'
   */
  rtDW.MemoryX_DSTATE[2] = (((1.0 / (2.0 * rtb_Csurface) - rtb_Rt) + rtb_Rt) *
    ((exp(-a_vo * 0.1) - 1.0) * (1.0 / a_vo)) * rtU.current + rtb_Add_d[2]) + c
    [2];
}

/* Model initialize function */
void RC_Model_KF_Vout_Vcb_for_MCU_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* InitializeConditions for Delay: '<S2>/MemoryP' */
  rtDW.icLoad = true;

  /* InitializeConditions for Delay: '<Root>/Delay' */
  rtDW.Delay_DSTATE = 1.0;

  /* InitializeConditions for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' */
  rtDW.DiscreteTimeIntegrator_DSTATE = 5.0;

  /* InitializeConditions for RandomNumber: '<S1>/White Noise' */
  rtDW.RandSeed = 1529675776U;
  rtDW.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf(&rtDW.RandSeed);

  /* InitializeConditions for Delay: '<S2>/MemoryX' */
  rtDW.icLoad_o = true;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
