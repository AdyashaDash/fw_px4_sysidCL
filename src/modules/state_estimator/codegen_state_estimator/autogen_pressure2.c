/*
 * autogen_pressure2.c
 *
 * Code generation for function 'autogen_pressure2'
 *
 * C source code generated on: Wed May 06 16:15:52 2015
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "getAirplane.h"
#include "initStates.h"
#include "magField.h"
#include "propagate.h"
#include "quat2rpy.h"
#include "updateCompass2.h"
#include "updatePosition.h"
#include "updatePressures.h"
#include "updatePressures2.h"
#include "updatePressures_all.h"
#include "updateVelNed.h"
#include "autogen_pressure2.h"
#include "HyEst_rtwutil.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void autogen_pressure2(const real32_T in1[9], const real32_T in2[6], const
  real32_T in3[3], real32_T m, real32_T m0, real32_T rho0, real32_T A, real32_T
  e[4], real32_T E_z[24], real32_T E_x_dash[36])
{
  real32_T t2;
  real32_T t3;
  real32_T t4;
  real32_T t7;
  real32_T t8;
  real32_T t9;
  real32_T t10;
  real32_T t11;
  real32_T t12;
  real32_T t13;
  real32_T t14;
  real32_T t17;
  real32_T t20;
  real32_T t21;
  real32_T t22;
  real32_T t25;
  real32_T t26;
  real32_T t27;
  real32_T t28;
  real32_T t29;
  real32_T t30;
  real32_T t34;
  real32_T t35;
  real32_T t36;
  real32_T t38;
  real32_T t43;
  real32_T x[24];
  real32_T b_x[36];

  /* AUTOGEN_PRESSURE2 */
  /*     [E,E_Z,E_X_DASH] = AUTOGEN_PRESSURE2(IN1,IN2,IN3,M,M0,RHO0,A) */
  /*     This function was generated by the Symbolic Math Toolbox version 5.9. */
  /*     30-Apr-2014 09:59:51 */
  t2 = in2[0] - in1[4];
  t3 = in2[2] - in1[6];
  t4 = 1.0F / in2[3];
  t7 = rt_powf_snf(t2, 2.0F) + rt_powf_snf(t3, 2.0F);
  t8 = 1.0F / m0;
  t9 = (real32_T)sqrt(t7);
  t10 = in2[5] * m * rho0 * t4 * t8 * t9 * 0.0292717703F;
  t11 = 1.0F / in2[5];
  t12 = rt_powf_snf(in1[1], 2.0F);
  t13 = rt_powf_snf(in1[3], 2.0F);
  t14 = 1.0F / (real32_T)sqrt(t7);
  t17 = t2 * in1[1] + t3 * in1[3];
  t20 = in2[0] * 2.0F - in1[4] * 2.0F;
  t21 = 1.0F / (real32_T)sqrt(t10);
  t22 = 1.0F / rt_powf_snf(t7, 1.5F);
  t25 = in2[2] * 2.0F - in1[6] * 2.0F;
  t26 = rt_powf_snf(in2[5], 2.0F);
  t27 = 1.0F / rt_powf_snf(in2[3], 2.0F);
  t28 = 1.0F / rt_powf_snf(t10, 1.5F);
  t29 = in2[5] * in2[4] * t4 * 574.116F;
  t30 = t14 * in1[1];
  t7 = in2[5] * m * rho0 * t4 * t8 * t9;
  t34 = t14 * in1[3];
  t35 = 1.0F / (real32_T)sqrt(t7);
  t36 = 1.0F / rt_powf_snf(t7, 1.5F);
  t7 = t12 + t13;
  t38 = (real32_T)sqrt(t7);
  t7 = 1.0F / (real32_T)sqrt(t7);
  t9 = t11 * (in2[5] + in1[0] * 0.00649F);
  t43 = 1.0F / rt_powf_snf(t9, 5.26388407F);
  e[0] = ((t14 * t17 - in3[2] * (real32_T)sqrt(t10)) - in3[1] * (real32_T)sqrt
          (t29)) - in2[5] * in3[0] * in2[4] * t4 * t21 * 574.116F;
  e[1] = m * (in2[1] - in1[5]) + A * in1[8] * in2[3] * t11 * t38 * in1[2] *
    1.7418083F;
  e[2] = in2[4] * 1000.0F - in2[3] * t11 * ((t12 + t13) + rt_powf_snf(in1[2],
    2.0F)) * 1.7418083F;
  e[3] = in2[3] * 1000.0F - in1[7] * t43 * 1000.0F;
  x[0] = ((t30 - t17 * t20 * t22 * 0.5F) - in2[5] * in3[2] * m * rho0 * t4 * t8 *
          t14 * t20 * t21 * 0.00731794257F) + in3[0] * m * in2[4] * rho0 * t8 *
    t14 * t20 * t26 * t27 * t28 * 4.20134783F;
  x[1] = 0.0F;
  x[2] = 0.0F;
  x[3] = 0.0F;
  x[4] = 0.0F;
  x[5] = m;
  x[6] = 0.0F;
  x[7] = 0.0F;
  x[8] = ((t34 - t17 * t22 * t25 * 0.5F) - in2[5] * in3[2] * m * rho0 * t4 * t8 *
          t14 * t21 * t25 * 0.00731794257F) + in3[0] * m * in2[4] * rho0 * t8 *
    t14 * t25 * t26 * t27 * t28 * 4.20134783F;
  x[9] = 0.0F;
  x[10] = 0.0F;
  x[11] = 0.0F;
  x[12] = 0.0F;
  x[13] = 0.0F;
  x[14] = 0.0F;
  x[15] = 1000.0F;
  x[16] = in2[5] * in3[1] * t4 / (real32_T)sqrt(t29) * -287.058F - in2[5] * in3
    [0] * t4 * t21 * 574.116F;
  x[17] = 0.0F;
  x[18] = 1000.0F;
  x[19] = 0.0F;
  x[20] = 0.0F;
  x[21] = 0.0F;
  x[22] = 0.0F;
  x[23] = in1[7] / rt_powf_snf(in2[5], 2.0F) * in1[0] / rt_powf_snf(in1[0] * t11
    * 0.00649F + 1.0F, 6.26388407F) * -34.1626091F;
  memcpy(&E_z[0], &x[0], 24U * sizeof(real32_T));
  b_x[0] = 0.0F;
  b_x[1] = 0.0F;
  b_x[2] = 0.0F;
  b_x[3] = in1[7] * t11 / rt_powf_snf(t9, 6.26388407F) * 34.1626091F;
  b_x[4] = t2 * t14;
  b_x[5] = A * in1[8] * in2[3] * t11 * t7 * in1[1] * in1[2] * 1.7418083F;
  b_x[6] = in2[3] * t11 * in1[1] * -3.48361659F;
  b_x[7] = 0.0F;
  b_x[8] = 0.0F;
  b_x[9] = A * in1[8] * in2[3] * t11 * t38 * 1.7418083F;
  b_x[10] = in2[3] * t11 * in1[2] * -3.48361659F;
  b_x[11] = 0.0F;
  b_x[12] = t3 * t14;
  b_x[13] = A * in1[8] * in2[3] * t11 * t7 * in1[2] * in1[3] * 1.7418083F;
  b_x[14] = in2[3] * t11 * in1[3] * -3.48361659F;
  b_x[15] = 0.0F;
  b_x[16] = ((-t30 + t17 * t20 * t22 * 0.5F) + in2[5] * in3[2] * m * rho0 * t4 *
             t8 * t14 * t20 * 1.42126384E+8F * 8.30711552E+8F * t35 *
             3.62275976E-19F) - in3[0] * m * in2[4] * rho0 * t8 * t14 * t20 *
    t26 * t27 * 1.42126384E+8F * 8.30711552E+8F * t36 * 7.10542736E-15F;
  b_x[17] = 0.0F;
  b_x[18] = 0.0F;
  b_x[19] = 0.0F;
  b_x[20] = 0.0F;
  b_x[21] = -m;
  b_x[22] = 0.0F;
  b_x[23] = 0.0F;
  b_x[24] = ((-t34 + t17 * t22 * t25 * 0.5F) + in2[5] * in3[2] * m * rho0 * t4 *
             t8 * t14 * t25 * 1.42126384E+8F * 8.30711552E+8F * t35 *
             3.62275976E-19F) - in3[0] * m * in2[4] * rho0 * t8 * t14 * t25 *
    t26 * t27 * 1.42126384E+8F * 8.30711552E+8F * t36 * 7.10542736E-15F;
  b_x[25] = 0.0F;
  b_x[26] = 0.0F;
  b_x[27] = 0.0F;
  b_x[28] = 0.0F;
  b_x[29] = 0.0F;
  b_x[30] = 0.0F;
  b_x[31] = t43 * -1000.0F;
  b_x[32] = 0.0F;
  b_x[33] = A * in2[3] * t11 * t38 * in1[2] * 1.7418083F;
  b_x[34] = 0.0F;
  b_x[35] = 0.0F;
  memcpy(&E_x_dash[0], &b_x[0], 36U * sizeof(real32_T));
}

/* End of code generation (autogen_pressure2.c) */