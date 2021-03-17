/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 3 + 2];

acadoWorkspace.state[18] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[19] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.state[20] = acadoVariables.od[lRun1 * 5];
acadoWorkspace.state[21] = acadoVariables.od[lRun1 * 5 + 1];
acadoWorkspace.state[22] = acadoVariables.od[lRun1 * 5 + 2];
acadoWorkspace.state[23] = acadoVariables.od[lRun1 * 5 + 3];
acadoWorkspace.state[24] = acadoVariables.od[lRun1 * 5 + 4];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 3] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 3 + 3];
acadoWorkspace.d[lRun1 * 3 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 3 + 4];
acadoWorkspace.d[lRun1 * 3 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 3 + 5];

acadoWorkspace.evGx[lRun1 * 9] = acadoWorkspace.state[3];
acadoWorkspace.evGx[lRun1 * 9 + 1] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 9 + 2] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 9 + 3] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 9 + 4] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 9 + 5] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 9 + 6] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 9 + 7] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 9 + 8] = acadoWorkspace.state[11];

acadoWorkspace.evGu[lRun1 * 6] = acadoWorkspace.state[12];
acadoWorkspace.evGu[lRun1 * 6 + 1] = acadoWorkspace.state[13];
acadoWorkspace.evGu[lRun1 * 6 + 2] = acadoWorkspace.state[14];
acadoWorkspace.evGu[lRun1 * 6 + 3] = acadoWorkspace.state[15];
acadoWorkspace.evGu[lRun1 * 6 + 4] = acadoWorkspace.state[16];
acadoWorkspace.evGu[lRun1 * 6 + 5] = acadoWorkspace.state[17];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 3;
const real_t* od = in + 5;

/* Compute outputs: */
out[0] = (xd[0]-od[0]);
out[1] = (xd[1]-od[1]);
out[2] = (xd[2]-od[2]);
out[3] = (u[0]-od[3]);
out[4] = (u[1]-od[4]);
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 3;

/* Compute outputs: */
out[0] = (xd[0]-od[0]);
out[1] = (xd[1]-od[1]);
out[2] = (xd[2]-od[2]);
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[5];
tmpQ1[4] = + tmpQ2[6];
tmpQ1[5] = + tmpQ2[7];
tmpQ1[6] = + tmpQ2[10];
tmpQ1[7] = + tmpQ2[11];
tmpQ1[8] = + tmpQ2[12];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[15];
tmpR2[1] = +tmpObjS[16];
tmpR2[2] = +tmpObjS[17];
tmpR2[3] = +tmpObjS[18];
tmpR2[4] = +tmpObjS[19];
tmpR2[5] = +tmpObjS[20];
tmpR2[6] = +tmpObjS[21];
tmpR2[7] = +tmpObjS[22];
tmpR2[8] = +tmpObjS[23];
tmpR2[9] = +tmpObjS[24];
tmpR1[0] = + tmpR2[3];
tmpR1[1] = + tmpR2[4];
tmpR1[2] = + tmpR2[8];
tmpR1[3] = + tmpR2[9];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 60; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 2 + 1];
acadoWorkspace.objValueIn[5] = acadoVariables.od[runObj * 5];
acadoWorkspace.objValueIn[6] = acadoVariables.od[runObj * 5 + 1];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 5 + 2];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 5 + 3];
acadoWorkspace.objValueIn[9] = acadoVariables.od[runObj * 5 + 4];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 5] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 5 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 5 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 5 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 5 + 4] = acadoWorkspace.objValueOut[4];

acado_setObjQ1Q2( acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 9 ]), &(acadoWorkspace.Q2[ runObj * 15 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj * 4 ]), &(acadoWorkspace.R2[ runObj * 10 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[180];
acadoWorkspace.objValueIn[1] = acadoVariables.x[181];
acadoWorkspace.objValueIn[2] = acadoVariables.x[182];
acadoWorkspace.objValueIn[3] = acadoVariables.od[300];
acadoWorkspace.objValueIn[4] = acadoVariables.od[301];
acadoWorkspace.objValueIn[5] = acadoVariables.od[302];
acadoWorkspace.objValueIn[6] = acadoVariables.od[303];
acadoWorkspace.objValueIn[7] = acadoVariables.od[304];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5];
Gu2[2] = + Gx1[3]*Gu1[0] + Gx1[4]*Gu1[2] + Gx1[5]*Gu1[4];
Gu2[3] = + Gx1[3]*Gu1[1] + Gx1[4]*Gu1[3] + Gx1[5]*Gu1[5];
Gu2[4] = + Gx1[6]*Gu1[0] + Gx1[7]*Gu1[2] + Gx1[8]*Gu1[4];
Gu2[5] = + Gx1[6]*Gu1[1] + Gx1[7]*Gu1[3] + Gx1[8]*Gu1[5];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 240) + (iCol * 2)] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4];
acadoWorkspace.H[(iRow * 240) + (iCol * 2 + 1)] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5];
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 2)] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4];
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 2 + 1)] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 242] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + R11[0];
acadoWorkspace.H[iRow * 242 + 1] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + R11[1];
acadoWorkspace.H[iRow * 242 + 120] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + R11[2];
acadoWorkspace.H[iRow * 242 + 121] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + R11[3];
acadoWorkspace.H[iRow * 242] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 242 + 121] += 1.0000000000000000e-10;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[3]*Gu1[2] + Gx1[6]*Gu1[4];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[3]*Gu1[3] + Gx1[6]*Gu1[5];
Gu2[2] = + Gx1[1]*Gu1[0] + Gx1[4]*Gu1[2] + Gx1[7]*Gu1[4];
Gu2[3] = + Gx1[1]*Gu1[1] + Gx1[4]*Gu1[3] + Gx1[7]*Gu1[5];
Gu2[4] = + Gx1[2]*Gu1[0] + Gx1[5]*Gu1[2] + Gx1[8]*Gu1[4];
Gu2[5] = + Gx1[2]*Gu1[1] + Gx1[5]*Gu1[3] + Gx1[8]*Gu1[5];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[2] + Q11[2]*Gu1[4] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[3] + Q11[2]*Gu1[5] + Gu2[1];
Gu3[2] = + Q11[3]*Gu1[0] + Q11[4]*Gu1[2] + Q11[5]*Gu1[4] + Gu2[2];
Gu3[3] = + Q11[3]*Gu1[1] + Q11[4]*Gu1[3] + Q11[5]*Gu1[5] + Gu2[3];
Gu3[4] = + Q11[6]*Gu1[0] + Q11[7]*Gu1[2] + Q11[8]*Gu1[4] + Gu2[4];
Gu3[5] = + Q11[6]*Gu1[1] + Q11[7]*Gu1[3] + Q11[8]*Gu1[5] + Gu2[5];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[3]*w11[1] + Gx1[6]*w11[2] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[4]*w11[1] + Gx1[7]*w11[2] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[5]*w11[1] + Gx1[8]*w11[2] + w12[2];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[2]*w11[1] + Gu1[4]*w11[2];
U1[1] += + Gu1[1]*w11[0] + Gu1[3]*w11[1] + Gu1[5]*w11[2];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + w12[0];
w13[1] = + Q11[3]*w11[0] + Q11[4]*w11[1] + Q11[5]*w11[2] + w12[1];
w13[2] = + Q11[6]*w11[0] + Q11[7]*w11[1] + Q11[8]*w11[2] + w12[2];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2];
w12[1] += + Gx1[3]*w11[0] + Gx1[4]*w11[1] + Gx1[5]*w11[2];
w12[2] += + Gx1[6]*w11[0] + Gx1[7]*w11[1] + Gx1[8]*w11[2];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2];
w12[1] += + Gx1[3]*w11[0] + Gx1[4]*w11[1] + Gx1[5]*w11[2];
w12[2] += + Gx1[6]*w11[0] + Gx1[7]*w11[1] + Gx1[8]*w11[2];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1];
w12[1] += + Gu1[2]*U1[0] + Gu1[3]*U1[1];
w12[2] += + Gu1[4]*U1[0] + Gu1[5]*U1[1];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 240) + (iCol * 2)] = acadoWorkspace.H[(iCol * 240) + (iRow * 2)];
acadoWorkspace.H[(iRow * 240) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 240 + 120) + (iRow * 2)];
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 2)] = acadoWorkspace.H[(iCol * 240) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 240 + 120) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 240 + 120) + (iRow * 2 + 1)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4];
RDy1[1] = + R2[5]*Dy1[0] + R2[6]*Dy1[1] + R2[7]*Dy1[2] + R2[8]*Dy1[3] + R2[9]*Dy1[4];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4];
QDy1[1] = + Q2[5]*Dy1[0] + Q2[6]*Dy1[1] + Q2[7]*Dy1[2] + Q2[8]*Dy1[3] + Q2[9]*Dy1[4];
QDy1[2] = + Q2[10]*Dy1[0] + Q2[11]*Dy1[1] + Q2[12]*Dy1[2] + Q2[13]*Dy1[3] + Q2[14]*Dy1[4];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun2 = 0; lRun2 < 60; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 121)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 6 ]), &(acadoWorkspace.E[ lRun3 * 6 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 60; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (3)) * (3)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (3)) * (2)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (3)) * (2)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (60)) - (1)) * (3)) * (2)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 59; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 6 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 9 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 9 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (3)) * (2)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 4 ]), &(acadoWorkspace.evGu[ lRun2 * 6 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

for (lRun1 = 0; lRun1 < 180; ++lRun1)
acadoWorkspace.sbar[lRun1 + 3] = acadoWorkspace.d[lRun1];

acadoWorkspace.lb[0] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[39];
acadoWorkspace.lb[40] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[40];
acadoWorkspace.lb[41] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[41];
acadoWorkspace.lb[42] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[42];
acadoWorkspace.lb[43] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[43];
acadoWorkspace.lb[44] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[44];
acadoWorkspace.lb[45] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[45];
acadoWorkspace.lb[46] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[46];
acadoWorkspace.lb[47] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[47];
acadoWorkspace.lb[48] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[48];
acadoWorkspace.lb[49] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[49];
acadoWorkspace.lb[50] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[50];
acadoWorkspace.lb[51] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[51];
acadoWorkspace.lb[52] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[52];
acadoWorkspace.lb[53] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[53];
acadoWorkspace.lb[54] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[54];
acadoWorkspace.lb[55] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[55];
acadoWorkspace.lb[56] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[56];
acadoWorkspace.lb[57] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[57];
acadoWorkspace.lb[58] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[58];
acadoWorkspace.lb[59] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[59];
acadoWorkspace.lb[60] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[60];
acadoWorkspace.lb[61] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[61];
acadoWorkspace.lb[62] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[62];
acadoWorkspace.lb[63] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[63];
acadoWorkspace.lb[64] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[64];
acadoWorkspace.lb[65] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[65];
acadoWorkspace.lb[66] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[66];
acadoWorkspace.lb[67] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[67];
acadoWorkspace.lb[68] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[68];
acadoWorkspace.lb[69] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[69];
acadoWorkspace.lb[70] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[70];
acadoWorkspace.lb[71] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[71];
acadoWorkspace.lb[72] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[72];
acadoWorkspace.lb[73] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[73];
acadoWorkspace.lb[74] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[74];
acadoWorkspace.lb[75] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[75];
acadoWorkspace.lb[76] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[76];
acadoWorkspace.lb[77] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[77];
acadoWorkspace.lb[78] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[78];
acadoWorkspace.lb[79] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[79];
acadoWorkspace.lb[80] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[80];
acadoWorkspace.lb[81] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[81];
acadoWorkspace.lb[82] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[82];
acadoWorkspace.lb[83] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[83];
acadoWorkspace.lb[84] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[84];
acadoWorkspace.lb[85] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[85];
acadoWorkspace.lb[86] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[86];
acadoWorkspace.lb[87] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[87];
acadoWorkspace.lb[88] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[88];
acadoWorkspace.lb[89] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[89];
acadoWorkspace.lb[90] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[90];
acadoWorkspace.lb[91] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[91];
acadoWorkspace.lb[92] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[92];
acadoWorkspace.lb[93] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[93];
acadoWorkspace.lb[94] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[94];
acadoWorkspace.lb[95] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[95];
acadoWorkspace.lb[96] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[96];
acadoWorkspace.lb[97] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[97];
acadoWorkspace.lb[98] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[98];
acadoWorkspace.lb[99] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[99];
acadoWorkspace.lb[100] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[100];
acadoWorkspace.lb[101] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[101];
acadoWorkspace.lb[102] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[102];
acadoWorkspace.lb[103] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[103];
acadoWorkspace.lb[104] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[104];
acadoWorkspace.lb[105] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[105];
acadoWorkspace.lb[106] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[106];
acadoWorkspace.lb[107] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[107];
acadoWorkspace.lb[108] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[108];
acadoWorkspace.lb[109] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[109];
acadoWorkspace.lb[110] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[110];
acadoWorkspace.lb[111] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[111];
acadoWorkspace.lb[112] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[112];
acadoWorkspace.lb[113] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[113];
acadoWorkspace.lb[114] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[114];
acadoWorkspace.lb[115] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[115];
acadoWorkspace.lb[116] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[116];
acadoWorkspace.lb[117] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[117];
acadoWorkspace.lb[118] = (real_t)-6.9999999999999996e-01 - acadoVariables.u[118];
acadoWorkspace.lb[119] = (real_t)-2.4434609527920612e+00 - acadoVariables.u[119];
acadoWorkspace.ub[0] = (real_t)6.9999999999999996e-01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)2.4434609527920612e+00 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)6.9999999999999996e-01 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)2.4434609527920612e+00 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)6.9999999999999996e-01 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)2.4434609527920612e+00 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)6.9999999999999996e-01 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)2.4434609527920612e+00 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)6.9999999999999996e-01 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)2.4434609527920612e+00 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)6.9999999999999996e-01 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)2.4434609527920612e+00 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)6.9999999999999996e-01 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)2.4434609527920612e+00 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)6.9999999999999996e-01 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)2.4434609527920612e+00 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)6.9999999999999996e-01 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)2.4434609527920612e+00 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)6.9999999999999996e-01 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)2.4434609527920612e+00 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)6.9999999999999996e-01 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)2.4434609527920612e+00 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)6.9999999999999996e-01 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)2.4434609527920612e+00 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)6.9999999999999996e-01 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)2.4434609527920612e+00 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)6.9999999999999996e-01 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)2.4434609527920612e+00 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)6.9999999999999996e-01 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)2.4434609527920612e+00 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)6.9999999999999996e-01 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)2.4434609527920612e+00 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)6.9999999999999996e-01 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)2.4434609527920612e+00 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)6.9999999999999996e-01 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)2.4434609527920612e+00 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)6.9999999999999996e-01 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)2.4434609527920612e+00 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)6.9999999999999996e-01 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)2.4434609527920612e+00 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)6.9999999999999996e-01 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)2.4434609527920612e+00 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)6.9999999999999996e-01 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)2.4434609527920612e+00 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)6.9999999999999996e-01 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)2.4434609527920612e+00 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)6.9999999999999996e-01 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)2.4434609527920612e+00 - acadoVariables.u[47];
acadoWorkspace.ub[48] = (real_t)6.9999999999999996e-01 - acadoVariables.u[48];
acadoWorkspace.ub[49] = (real_t)2.4434609527920612e+00 - acadoVariables.u[49];
acadoWorkspace.ub[50] = (real_t)6.9999999999999996e-01 - acadoVariables.u[50];
acadoWorkspace.ub[51] = (real_t)2.4434609527920612e+00 - acadoVariables.u[51];
acadoWorkspace.ub[52] = (real_t)6.9999999999999996e-01 - acadoVariables.u[52];
acadoWorkspace.ub[53] = (real_t)2.4434609527920612e+00 - acadoVariables.u[53];
acadoWorkspace.ub[54] = (real_t)6.9999999999999996e-01 - acadoVariables.u[54];
acadoWorkspace.ub[55] = (real_t)2.4434609527920612e+00 - acadoVariables.u[55];
acadoWorkspace.ub[56] = (real_t)6.9999999999999996e-01 - acadoVariables.u[56];
acadoWorkspace.ub[57] = (real_t)2.4434609527920612e+00 - acadoVariables.u[57];
acadoWorkspace.ub[58] = (real_t)6.9999999999999996e-01 - acadoVariables.u[58];
acadoWorkspace.ub[59] = (real_t)2.4434609527920612e+00 - acadoVariables.u[59];
acadoWorkspace.ub[60] = (real_t)6.9999999999999996e-01 - acadoVariables.u[60];
acadoWorkspace.ub[61] = (real_t)2.4434609527920612e+00 - acadoVariables.u[61];
acadoWorkspace.ub[62] = (real_t)6.9999999999999996e-01 - acadoVariables.u[62];
acadoWorkspace.ub[63] = (real_t)2.4434609527920612e+00 - acadoVariables.u[63];
acadoWorkspace.ub[64] = (real_t)6.9999999999999996e-01 - acadoVariables.u[64];
acadoWorkspace.ub[65] = (real_t)2.4434609527920612e+00 - acadoVariables.u[65];
acadoWorkspace.ub[66] = (real_t)6.9999999999999996e-01 - acadoVariables.u[66];
acadoWorkspace.ub[67] = (real_t)2.4434609527920612e+00 - acadoVariables.u[67];
acadoWorkspace.ub[68] = (real_t)6.9999999999999996e-01 - acadoVariables.u[68];
acadoWorkspace.ub[69] = (real_t)2.4434609527920612e+00 - acadoVariables.u[69];
acadoWorkspace.ub[70] = (real_t)6.9999999999999996e-01 - acadoVariables.u[70];
acadoWorkspace.ub[71] = (real_t)2.4434609527920612e+00 - acadoVariables.u[71];
acadoWorkspace.ub[72] = (real_t)6.9999999999999996e-01 - acadoVariables.u[72];
acadoWorkspace.ub[73] = (real_t)2.4434609527920612e+00 - acadoVariables.u[73];
acadoWorkspace.ub[74] = (real_t)6.9999999999999996e-01 - acadoVariables.u[74];
acadoWorkspace.ub[75] = (real_t)2.4434609527920612e+00 - acadoVariables.u[75];
acadoWorkspace.ub[76] = (real_t)6.9999999999999996e-01 - acadoVariables.u[76];
acadoWorkspace.ub[77] = (real_t)2.4434609527920612e+00 - acadoVariables.u[77];
acadoWorkspace.ub[78] = (real_t)6.9999999999999996e-01 - acadoVariables.u[78];
acadoWorkspace.ub[79] = (real_t)2.4434609527920612e+00 - acadoVariables.u[79];
acadoWorkspace.ub[80] = (real_t)6.9999999999999996e-01 - acadoVariables.u[80];
acadoWorkspace.ub[81] = (real_t)2.4434609527920612e+00 - acadoVariables.u[81];
acadoWorkspace.ub[82] = (real_t)6.9999999999999996e-01 - acadoVariables.u[82];
acadoWorkspace.ub[83] = (real_t)2.4434609527920612e+00 - acadoVariables.u[83];
acadoWorkspace.ub[84] = (real_t)6.9999999999999996e-01 - acadoVariables.u[84];
acadoWorkspace.ub[85] = (real_t)2.4434609527920612e+00 - acadoVariables.u[85];
acadoWorkspace.ub[86] = (real_t)6.9999999999999996e-01 - acadoVariables.u[86];
acadoWorkspace.ub[87] = (real_t)2.4434609527920612e+00 - acadoVariables.u[87];
acadoWorkspace.ub[88] = (real_t)6.9999999999999996e-01 - acadoVariables.u[88];
acadoWorkspace.ub[89] = (real_t)2.4434609527920612e+00 - acadoVariables.u[89];
acadoWorkspace.ub[90] = (real_t)6.9999999999999996e-01 - acadoVariables.u[90];
acadoWorkspace.ub[91] = (real_t)2.4434609527920612e+00 - acadoVariables.u[91];
acadoWorkspace.ub[92] = (real_t)6.9999999999999996e-01 - acadoVariables.u[92];
acadoWorkspace.ub[93] = (real_t)2.4434609527920612e+00 - acadoVariables.u[93];
acadoWorkspace.ub[94] = (real_t)6.9999999999999996e-01 - acadoVariables.u[94];
acadoWorkspace.ub[95] = (real_t)2.4434609527920612e+00 - acadoVariables.u[95];
acadoWorkspace.ub[96] = (real_t)6.9999999999999996e-01 - acadoVariables.u[96];
acadoWorkspace.ub[97] = (real_t)2.4434609527920612e+00 - acadoVariables.u[97];
acadoWorkspace.ub[98] = (real_t)6.9999999999999996e-01 - acadoVariables.u[98];
acadoWorkspace.ub[99] = (real_t)2.4434609527920612e+00 - acadoVariables.u[99];
acadoWorkspace.ub[100] = (real_t)6.9999999999999996e-01 - acadoVariables.u[100];
acadoWorkspace.ub[101] = (real_t)2.4434609527920612e+00 - acadoVariables.u[101];
acadoWorkspace.ub[102] = (real_t)6.9999999999999996e-01 - acadoVariables.u[102];
acadoWorkspace.ub[103] = (real_t)2.4434609527920612e+00 - acadoVariables.u[103];
acadoWorkspace.ub[104] = (real_t)6.9999999999999996e-01 - acadoVariables.u[104];
acadoWorkspace.ub[105] = (real_t)2.4434609527920612e+00 - acadoVariables.u[105];
acadoWorkspace.ub[106] = (real_t)6.9999999999999996e-01 - acadoVariables.u[106];
acadoWorkspace.ub[107] = (real_t)2.4434609527920612e+00 - acadoVariables.u[107];
acadoWorkspace.ub[108] = (real_t)6.9999999999999996e-01 - acadoVariables.u[108];
acadoWorkspace.ub[109] = (real_t)2.4434609527920612e+00 - acadoVariables.u[109];
acadoWorkspace.ub[110] = (real_t)6.9999999999999996e-01 - acadoVariables.u[110];
acadoWorkspace.ub[111] = (real_t)2.4434609527920612e+00 - acadoVariables.u[111];
acadoWorkspace.ub[112] = (real_t)6.9999999999999996e-01 - acadoVariables.u[112];
acadoWorkspace.ub[113] = (real_t)2.4434609527920612e+00 - acadoVariables.u[113];
acadoWorkspace.ub[114] = (real_t)6.9999999999999996e-01 - acadoVariables.u[114];
acadoWorkspace.ub[115] = (real_t)2.4434609527920612e+00 - acadoVariables.u[115];
acadoWorkspace.ub[116] = (real_t)6.9999999999999996e-01 - acadoVariables.u[116];
acadoWorkspace.ub[117] = (real_t)2.4434609527920612e+00 - acadoVariables.u[117];
acadoWorkspace.ub[118] = (real_t)6.9999999999999996e-01 - acadoVariables.u[118];
acadoWorkspace.ub[119] = (real_t)2.4434609527920612e+00 - acadoVariables.u[119];

}

void acado_condenseFdb(  )
{
int lRun1;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
for (lRun1 = 0; lRun1 < 300; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 10 ]), &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 20 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 30 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 40 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 50 ]), &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 60 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 70 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 80 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 90 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 100 ]), &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 110 ]), &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 120 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 130 ]), &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 140 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 150 ]), &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 160 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 170 ]), &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 180 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 190 ]), &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.g[ 38 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 200 ]), &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 210 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 220 ]), &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 230 ]), &(acadoWorkspace.Dy[ 115 ]), &(acadoWorkspace.g[ 46 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 240 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 250 ]), &(acadoWorkspace.Dy[ 125 ]), &(acadoWorkspace.g[ 50 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 260 ]), &(acadoWorkspace.Dy[ 130 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 270 ]), &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 280 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 290 ]), &(acadoWorkspace.Dy[ 145 ]), &(acadoWorkspace.g[ 58 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 300 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 310 ]), &(acadoWorkspace.Dy[ 155 ]), &(acadoWorkspace.g[ 62 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 320 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.g[ 64 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 330 ]), &(acadoWorkspace.Dy[ 165 ]), &(acadoWorkspace.g[ 66 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 340 ]), &(acadoWorkspace.Dy[ 170 ]), &(acadoWorkspace.g[ 68 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 350 ]), &(acadoWorkspace.Dy[ 175 ]), &(acadoWorkspace.g[ 70 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 360 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 370 ]), &(acadoWorkspace.Dy[ 185 ]), &(acadoWorkspace.g[ 74 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 380 ]), &(acadoWorkspace.Dy[ 190 ]), &(acadoWorkspace.g[ 76 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 390 ]), &(acadoWorkspace.Dy[ 195 ]), &(acadoWorkspace.g[ 78 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 400 ]), &(acadoWorkspace.Dy[ 200 ]), &(acadoWorkspace.g[ 80 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 410 ]), &(acadoWorkspace.Dy[ 205 ]), &(acadoWorkspace.g[ 82 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 420 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.g[ 84 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 430 ]), &(acadoWorkspace.Dy[ 215 ]), &(acadoWorkspace.g[ 86 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 440 ]), &(acadoWorkspace.Dy[ 220 ]), &(acadoWorkspace.g[ 88 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 450 ]), &(acadoWorkspace.Dy[ 225 ]), &(acadoWorkspace.g[ 90 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 460 ]), &(acadoWorkspace.Dy[ 230 ]), &(acadoWorkspace.g[ 92 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 470 ]), &(acadoWorkspace.Dy[ 235 ]), &(acadoWorkspace.g[ 94 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 480 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.g[ 96 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 490 ]), &(acadoWorkspace.Dy[ 245 ]), &(acadoWorkspace.g[ 98 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 500 ]), &(acadoWorkspace.Dy[ 250 ]), &(acadoWorkspace.g[ 100 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 510 ]), &(acadoWorkspace.Dy[ 255 ]), &(acadoWorkspace.g[ 102 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 520 ]), &(acadoWorkspace.Dy[ 260 ]), &(acadoWorkspace.g[ 104 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 530 ]), &(acadoWorkspace.Dy[ 265 ]), &(acadoWorkspace.g[ 106 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 540 ]), &(acadoWorkspace.Dy[ 270 ]), &(acadoWorkspace.g[ 108 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 550 ]), &(acadoWorkspace.Dy[ 275 ]), &(acadoWorkspace.g[ 110 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 560 ]), &(acadoWorkspace.Dy[ 280 ]), &(acadoWorkspace.g[ 112 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 570 ]), &(acadoWorkspace.Dy[ 285 ]), &(acadoWorkspace.g[ 114 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 580 ]), &(acadoWorkspace.Dy[ 290 ]), &(acadoWorkspace.g[ 116 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 590 ]), &(acadoWorkspace.Dy[ 295 ]), &(acadoWorkspace.g[ 118 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 15 ]), &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.QDy[ 3 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 30 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 45 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 9 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 60 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 75 ]), &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.QDy[ 15 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 90 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 105 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.QDy[ 21 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 120 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 135 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.QDy[ 27 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 150 ]), &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 165 ]), &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.QDy[ 33 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 180 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 195 ]), &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.QDy[ 39 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 210 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 42 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 225 ]), &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.QDy[ 45 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 240 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 255 ]), &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.QDy[ 51 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 270 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 54 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 285 ]), &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.QDy[ 57 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 300 ]), &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 315 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.QDy[ 63 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 330 ]), &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.QDy[ 66 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 345 ]), &(acadoWorkspace.Dy[ 115 ]), &(acadoWorkspace.QDy[ 69 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 360 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 375 ]), &(acadoWorkspace.Dy[ 125 ]), &(acadoWorkspace.QDy[ 75 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 390 ]), &(acadoWorkspace.Dy[ 130 ]), &(acadoWorkspace.QDy[ 78 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 405 ]), &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.QDy[ 81 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 420 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 435 ]), &(acadoWorkspace.Dy[ 145 ]), &(acadoWorkspace.QDy[ 87 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 450 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 465 ]), &(acadoWorkspace.Dy[ 155 ]), &(acadoWorkspace.QDy[ 93 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 480 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 495 ]), &(acadoWorkspace.Dy[ 165 ]), &(acadoWorkspace.QDy[ 99 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 510 ]), &(acadoWorkspace.Dy[ 170 ]), &(acadoWorkspace.QDy[ 102 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 525 ]), &(acadoWorkspace.Dy[ 175 ]), &(acadoWorkspace.QDy[ 105 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 540 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 555 ]), &(acadoWorkspace.Dy[ 185 ]), &(acadoWorkspace.QDy[ 111 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 570 ]), &(acadoWorkspace.Dy[ 190 ]), &(acadoWorkspace.QDy[ 114 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 585 ]), &(acadoWorkspace.Dy[ 195 ]), &(acadoWorkspace.QDy[ 117 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 600 ]), &(acadoWorkspace.Dy[ 200 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 615 ]), &(acadoWorkspace.Dy[ 205 ]), &(acadoWorkspace.QDy[ 123 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 630 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.QDy[ 126 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 645 ]), &(acadoWorkspace.Dy[ 215 ]), &(acadoWorkspace.QDy[ 129 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 660 ]), &(acadoWorkspace.Dy[ 220 ]), &(acadoWorkspace.QDy[ 132 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 675 ]), &(acadoWorkspace.Dy[ 225 ]), &(acadoWorkspace.QDy[ 135 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 690 ]), &(acadoWorkspace.Dy[ 230 ]), &(acadoWorkspace.QDy[ 138 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 705 ]), &(acadoWorkspace.Dy[ 235 ]), &(acadoWorkspace.QDy[ 141 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 720 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.QDy[ 144 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 735 ]), &(acadoWorkspace.Dy[ 245 ]), &(acadoWorkspace.QDy[ 147 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 750 ]), &(acadoWorkspace.Dy[ 250 ]), &(acadoWorkspace.QDy[ 150 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 765 ]), &(acadoWorkspace.Dy[ 255 ]), &(acadoWorkspace.QDy[ 153 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 780 ]), &(acadoWorkspace.Dy[ 260 ]), &(acadoWorkspace.QDy[ 156 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 795 ]), &(acadoWorkspace.Dy[ 265 ]), &(acadoWorkspace.QDy[ 159 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 810 ]), &(acadoWorkspace.Dy[ 270 ]), &(acadoWorkspace.QDy[ 162 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 825 ]), &(acadoWorkspace.Dy[ 275 ]), &(acadoWorkspace.QDy[ 165 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 840 ]), &(acadoWorkspace.Dy[ 280 ]), &(acadoWorkspace.QDy[ 168 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 855 ]), &(acadoWorkspace.Dy[ 285 ]), &(acadoWorkspace.QDy[ 171 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 870 ]), &(acadoWorkspace.Dy[ 290 ]), &(acadoWorkspace.QDy[ 174 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 885 ]), &(acadoWorkspace.Dy[ 295 ]), &(acadoWorkspace.QDy[ 177 ]) );

acadoWorkspace.QDy[180] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[181] = + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[182] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 3 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.sbar[ 3 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 9 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 15 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 21 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 33 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.sbar[ 33 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 39 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.sbar[ 42 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 51 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.sbar[ 51 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 57 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.sbar[ 57 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 189 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 66 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 198 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.sbar[ 69 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 207 ]), &(acadoWorkspace.sbar[ 69 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 75 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.sbar[ 75 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 234 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 87 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 261 ]), &(acadoWorkspace.sbar[ 87 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 270 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 93 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 279 ]), &(acadoWorkspace.sbar[ 93 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 297 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 102 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 306 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.sbar[ 105 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 315 ]), &(acadoWorkspace.sbar[ 105 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 111 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 333 ]), &(acadoWorkspace.sbar[ 111 ]), &(acadoWorkspace.sbar[ 114 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 342 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 351 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 123 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 369 ]), &(acadoWorkspace.sbar[ 123 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 378 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 129 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 387 ]), &(acadoWorkspace.sbar[ 129 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 138 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 414 ]), &(acadoWorkspace.sbar[ 138 ]), &(acadoWorkspace.sbar[ 141 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 423 ]), &(acadoWorkspace.sbar[ 141 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 147 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 441 ]), &(acadoWorkspace.sbar[ 147 ]), &(acadoWorkspace.sbar[ 150 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 450 ]), &(acadoWorkspace.sbar[ 150 ]), &(acadoWorkspace.sbar[ 153 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 459 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 159 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 477 ]), &(acadoWorkspace.sbar[ 159 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 165 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 495 ]), &(acadoWorkspace.sbar[ 165 ]), &(acadoWorkspace.sbar[ 168 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.sbar[ 171 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 513 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.sbar[ 174 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 522 ]), &(acadoWorkspace.sbar[ 174 ]), &(acadoWorkspace.sbar[ 177 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 531 ]), &(acadoWorkspace.sbar[ 177 ]), &(acadoWorkspace.sbar[ 180 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[182] + acadoWorkspace.QDy[180];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[182] + acadoWorkspace.QDy[181];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[182] + acadoWorkspace.QDy[182];
acado_macBTw1( &(acadoWorkspace.evGu[ 354 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 118 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 531 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 177 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 531 ]), &(acadoWorkspace.sbar[ 177 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 348 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 116 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 522 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 174 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 522 ]), &(acadoWorkspace.sbar[ 174 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 342 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 114 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 513 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 171 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 513 ]), &(acadoWorkspace.sbar[ 171 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 336 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 112 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 504 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 168 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 504 ]), &(acadoWorkspace.sbar[ 168 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 330 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 110 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 495 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 165 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 495 ]), &(acadoWorkspace.sbar[ 165 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 108 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 486 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 162 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 486 ]), &(acadoWorkspace.sbar[ 162 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 318 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 106 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 477 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 159 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 477 ]), &(acadoWorkspace.sbar[ 159 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 312 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 104 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 468 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 156 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 468 ]), &(acadoWorkspace.sbar[ 156 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 306 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 102 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 459 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 153 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 459 ]), &(acadoWorkspace.sbar[ 153 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 300 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 100 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 450 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 150 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 450 ]), &(acadoWorkspace.sbar[ 150 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 294 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 98 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 441 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 147 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 441 ]), &(acadoWorkspace.sbar[ 147 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 96 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.sbar[ 144 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 282 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 94 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 423 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 141 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 423 ]), &(acadoWorkspace.sbar[ 141 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 276 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 92 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 414 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 138 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 414 ]), &(acadoWorkspace.sbar[ 138 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 270 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 90 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 405 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 135 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.sbar[ 135 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 264 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 88 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 396 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 132 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 396 ]), &(acadoWorkspace.sbar[ 132 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 258 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 86 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 387 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 129 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 387 ]), &(acadoWorkspace.sbar[ 129 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 252 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 84 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 378 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 126 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 378 ]), &(acadoWorkspace.sbar[ 126 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 246 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 82 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 369 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 123 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 369 ]), &(acadoWorkspace.sbar[ 123 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 80 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 360 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 120 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 360 ]), &(acadoWorkspace.sbar[ 120 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 234 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 78 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 351 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 117 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 351 ]), &(acadoWorkspace.sbar[ 117 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 228 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 76 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 342 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 114 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 342 ]), &(acadoWorkspace.sbar[ 114 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 222 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 74 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 333 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 111 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 333 ]), &(acadoWorkspace.sbar[ 111 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 72 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 210 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 70 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 315 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 105 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 315 ]), &(acadoWorkspace.sbar[ 105 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 204 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 68 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 306 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 102 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 306 ]), &(acadoWorkspace.sbar[ 102 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 198 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 66 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 297 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 99 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 297 ]), &(acadoWorkspace.sbar[ 99 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 192 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 64 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 96 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.sbar[ 96 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 186 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 62 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 279 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 93 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 279 ]), &(acadoWorkspace.sbar[ 93 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 180 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 60 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 270 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 270 ]), &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 174 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 58 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 261 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 87 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 261 ]), &(acadoWorkspace.sbar[ 87 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 168 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 56 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 252 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 84 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 252 ]), &(acadoWorkspace.sbar[ 84 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 54 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 243 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 81 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 243 ]), &(acadoWorkspace.sbar[ 81 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 156 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 52 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 234 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 78 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 234 ]), &(acadoWorkspace.sbar[ 78 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 150 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 50 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 75 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.sbar[ 75 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 216 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 138 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 46 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 207 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 69 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 207 ]), &(acadoWorkspace.sbar[ 69 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 132 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 44 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 198 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 66 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 198 ]), &(acadoWorkspace.sbar[ 66 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 126 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 42 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 189 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 63 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 189 ]), &(acadoWorkspace.sbar[ 63 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 120 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 40 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 180 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 180 ]), &(acadoWorkspace.sbar[ 60 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 114 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 38 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 171 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 57 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.sbar[ 57 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.sbar[ 54 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 102 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 34 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 153 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 51 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.sbar[ 51 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 48 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 135 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 45 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.sbar[ 45 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 84 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 126 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 42 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.sbar[ 42 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 78 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 26 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 117 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 39 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.sbar[ 39 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 66 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 22 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 99 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 33 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.sbar[ 33 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 90 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.sbar[ 30 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 54 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 27 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.sbar[ 27 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 42 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 14 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 63 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 21 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.sbar[ 21 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 54 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 30 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 10 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 45 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 15 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.sbar[ 15 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 27 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 9 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 27 ]), &(acadoWorkspace.sbar[ 9 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 18 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 18 ]), &(acadoWorkspace.sbar[ 6 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 6 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 2 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 9 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 3 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 9 ]), &(acadoWorkspace.sbar[ 3 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );


}

void acado_expand(  )
{
int lRun1;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoVariables.u[60] += acadoWorkspace.x[60];
acadoVariables.u[61] += acadoWorkspace.x[61];
acadoVariables.u[62] += acadoWorkspace.x[62];
acadoVariables.u[63] += acadoWorkspace.x[63];
acadoVariables.u[64] += acadoWorkspace.x[64];
acadoVariables.u[65] += acadoWorkspace.x[65];
acadoVariables.u[66] += acadoWorkspace.x[66];
acadoVariables.u[67] += acadoWorkspace.x[67];
acadoVariables.u[68] += acadoWorkspace.x[68];
acadoVariables.u[69] += acadoWorkspace.x[69];
acadoVariables.u[70] += acadoWorkspace.x[70];
acadoVariables.u[71] += acadoWorkspace.x[71];
acadoVariables.u[72] += acadoWorkspace.x[72];
acadoVariables.u[73] += acadoWorkspace.x[73];
acadoVariables.u[74] += acadoWorkspace.x[74];
acadoVariables.u[75] += acadoWorkspace.x[75];
acadoVariables.u[76] += acadoWorkspace.x[76];
acadoVariables.u[77] += acadoWorkspace.x[77];
acadoVariables.u[78] += acadoWorkspace.x[78];
acadoVariables.u[79] += acadoWorkspace.x[79];
acadoVariables.u[80] += acadoWorkspace.x[80];
acadoVariables.u[81] += acadoWorkspace.x[81];
acadoVariables.u[82] += acadoWorkspace.x[82];
acadoVariables.u[83] += acadoWorkspace.x[83];
acadoVariables.u[84] += acadoWorkspace.x[84];
acadoVariables.u[85] += acadoWorkspace.x[85];
acadoVariables.u[86] += acadoWorkspace.x[86];
acadoVariables.u[87] += acadoWorkspace.x[87];
acadoVariables.u[88] += acadoWorkspace.x[88];
acadoVariables.u[89] += acadoWorkspace.x[89];
acadoVariables.u[90] += acadoWorkspace.x[90];
acadoVariables.u[91] += acadoWorkspace.x[91];
acadoVariables.u[92] += acadoWorkspace.x[92];
acadoVariables.u[93] += acadoWorkspace.x[93];
acadoVariables.u[94] += acadoWorkspace.x[94];
acadoVariables.u[95] += acadoWorkspace.x[95];
acadoVariables.u[96] += acadoWorkspace.x[96];
acadoVariables.u[97] += acadoWorkspace.x[97];
acadoVariables.u[98] += acadoWorkspace.x[98];
acadoVariables.u[99] += acadoWorkspace.x[99];
acadoVariables.u[100] += acadoWorkspace.x[100];
acadoVariables.u[101] += acadoWorkspace.x[101];
acadoVariables.u[102] += acadoWorkspace.x[102];
acadoVariables.u[103] += acadoWorkspace.x[103];
acadoVariables.u[104] += acadoWorkspace.x[104];
acadoVariables.u[105] += acadoWorkspace.x[105];
acadoVariables.u[106] += acadoWorkspace.x[106];
acadoVariables.u[107] += acadoWorkspace.x[107];
acadoVariables.u[108] += acadoWorkspace.x[108];
acadoVariables.u[109] += acadoWorkspace.x[109];
acadoVariables.u[110] += acadoWorkspace.x[110];
acadoVariables.u[111] += acadoWorkspace.x[111];
acadoVariables.u[112] += acadoWorkspace.x[112];
acadoVariables.u[113] += acadoWorkspace.x[113];
acadoVariables.u[114] += acadoWorkspace.x[114];
acadoVariables.u[115] += acadoWorkspace.x[115];
acadoVariables.u[116] += acadoWorkspace.x[116];
acadoVariables.u[117] += acadoWorkspace.x[117];
acadoVariables.u[118] += acadoWorkspace.x[118];
acadoVariables.u[119] += acadoWorkspace.x[119];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
for (lRun1 = 0; lRun1 < 180; ++lRun1)
acadoWorkspace.sbar[lRun1 + 3] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 3 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.evGu[ 6 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.sbar[ 3 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 9 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.evGu[ 18 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 15 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.evGu[ 30 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 21 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.evGu[ 42 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.evGu[ 54 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 33 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.evGu[ 66 ]), &(acadoWorkspace.x[ 22 ]), &(acadoWorkspace.sbar[ 33 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 39 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.evGu[ 78 ]), &(acadoWorkspace.x[ 26 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.sbar[ 42 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.evGu[ 84 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.evGu[ 90 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 51 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.evGu[ 102 ]), &(acadoWorkspace.x[ 34 ]), &(acadoWorkspace.sbar[ 51 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 57 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.evGu[ 114 ]), &(acadoWorkspace.x[ 38 ]), &(acadoWorkspace.sbar[ 57 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.x[ 40 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 189 ]), &(acadoWorkspace.evGu[ 126 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 66 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 198 ]), &(acadoWorkspace.evGu[ 132 ]), &(acadoWorkspace.x[ 44 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.sbar[ 69 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 207 ]), &(acadoWorkspace.evGu[ 138 ]), &(acadoWorkspace.x[ 46 ]), &(acadoWorkspace.sbar[ 69 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 75 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.evGu[ 150 ]), &(acadoWorkspace.x[ 50 ]), &(acadoWorkspace.sbar[ 75 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 234 ]), &(acadoWorkspace.evGu[ 156 ]), &(acadoWorkspace.x[ 52 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.evGu[ 162 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.evGu[ 168 ]), &(acadoWorkspace.x[ 56 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 87 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 261 ]), &(acadoWorkspace.evGu[ 174 ]), &(acadoWorkspace.x[ 58 ]), &(acadoWorkspace.sbar[ 87 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 270 ]), &(acadoWorkspace.evGu[ 180 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 93 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 279 ]), &(acadoWorkspace.evGu[ 186 ]), &(acadoWorkspace.x[ 62 ]), &(acadoWorkspace.sbar[ 93 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGu[ 192 ]), &(acadoWorkspace.x[ 64 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 297 ]), &(acadoWorkspace.evGu[ 198 ]), &(acadoWorkspace.x[ 66 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 102 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 306 ]), &(acadoWorkspace.evGu[ 204 ]), &(acadoWorkspace.x[ 68 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.sbar[ 105 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 315 ]), &(acadoWorkspace.evGu[ 210 ]), &(acadoWorkspace.x[ 70 ]), &(acadoWorkspace.sbar[ 105 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 111 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 333 ]), &(acadoWorkspace.evGu[ 222 ]), &(acadoWorkspace.x[ 74 ]), &(acadoWorkspace.sbar[ 111 ]), &(acadoWorkspace.sbar[ 114 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 342 ]), &(acadoWorkspace.evGu[ 228 ]), &(acadoWorkspace.x[ 76 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 351 ]), &(acadoWorkspace.evGu[ 234 ]), &(acadoWorkspace.x[ 78 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.evGu[ 240 ]), &(acadoWorkspace.x[ 80 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 123 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 369 ]), &(acadoWorkspace.evGu[ 246 ]), &(acadoWorkspace.x[ 82 ]), &(acadoWorkspace.sbar[ 123 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 378 ]), &(acadoWorkspace.evGu[ 252 ]), &(acadoWorkspace.x[ 84 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 129 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 387 ]), &(acadoWorkspace.evGu[ 258 ]), &(acadoWorkspace.x[ 86 ]), &(acadoWorkspace.sbar[ 129 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.evGu[ 264 ]), &(acadoWorkspace.x[ 88 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.evGu[ 270 ]), &(acadoWorkspace.x[ 90 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 138 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 414 ]), &(acadoWorkspace.evGu[ 276 ]), &(acadoWorkspace.x[ 92 ]), &(acadoWorkspace.sbar[ 138 ]), &(acadoWorkspace.sbar[ 141 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 423 ]), &(acadoWorkspace.evGu[ 282 ]), &(acadoWorkspace.x[ 94 ]), &(acadoWorkspace.sbar[ 141 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.evGu[ 288 ]), &(acadoWorkspace.x[ 96 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 147 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 441 ]), &(acadoWorkspace.evGu[ 294 ]), &(acadoWorkspace.x[ 98 ]), &(acadoWorkspace.sbar[ 147 ]), &(acadoWorkspace.sbar[ 150 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 450 ]), &(acadoWorkspace.evGu[ 300 ]), &(acadoWorkspace.x[ 100 ]), &(acadoWorkspace.sbar[ 150 ]), &(acadoWorkspace.sbar[ 153 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 459 ]), &(acadoWorkspace.evGu[ 306 ]), &(acadoWorkspace.x[ 102 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.evGu[ 312 ]), &(acadoWorkspace.x[ 104 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 159 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 477 ]), &(acadoWorkspace.evGu[ 318 ]), &(acadoWorkspace.x[ 106 ]), &(acadoWorkspace.sbar[ 159 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.evGu[ 324 ]), &(acadoWorkspace.x[ 108 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 165 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 495 ]), &(acadoWorkspace.evGu[ 330 ]), &(acadoWorkspace.x[ 110 ]), &(acadoWorkspace.sbar[ 165 ]), &(acadoWorkspace.sbar[ 168 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.evGu[ 336 ]), &(acadoWorkspace.x[ 112 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.sbar[ 171 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 513 ]), &(acadoWorkspace.evGu[ 342 ]), &(acadoWorkspace.x[ 114 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.sbar[ 174 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 522 ]), &(acadoWorkspace.evGu[ 348 ]), &(acadoWorkspace.x[ 116 ]), &(acadoWorkspace.sbar[ 174 ]), &(acadoWorkspace.sbar[ 177 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 531 ]), &(acadoWorkspace.evGu[ 354 ]), &(acadoWorkspace.x[ 118 ]), &(acadoWorkspace.sbar[ 177 ]), &(acadoWorkspace.sbar[ 180 ]) );
for (lRun1 = 0; lRun1 < 183; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 60; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 3];
acadoWorkspace.state[1] = acadoVariables.x[index * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 3 + 2];
acadoWorkspace.state[18] = acadoVariables.u[index * 2];
acadoWorkspace.state[19] = acadoVariables.u[index * 2 + 1];
acadoWorkspace.state[20] = acadoVariables.od[index * 5];
acadoWorkspace.state[21] = acadoVariables.od[index * 5 + 1];
acadoWorkspace.state[22] = acadoVariables.od[index * 5 + 2];
acadoWorkspace.state[23] = acadoVariables.od[index * 5 + 3];
acadoWorkspace.state[24] = acadoVariables.od[index * 5 + 4];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 3 + 3] = acadoWorkspace.state[0];
acadoVariables.x[index * 3 + 4] = acadoWorkspace.state[1];
acadoVariables.x[index * 3 + 5] = acadoWorkspace.state[2];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 60; ++index)
{
acadoVariables.x[index * 3] = acadoVariables.x[index * 3 + 3];
acadoVariables.x[index * 3 + 1] = acadoVariables.x[index * 3 + 4];
acadoVariables.x[index * 3 + 2] = acadoVariables.x[index * 3 + 5];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[180] = xEnd[0];
acadoVariables.x[181] = xEnd[1];
acadoVariables.x[182] = xEnd[2];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[180];
acadoWorkspace.state[1] = acadoVariables.x[181];
acadoWorkspace.state[2] = acadoVariables.x[182];
if (uEnd != 0)
{
acadoWorkspace.state[18] = uEnd[0];
acadoWorkspace.state[19] = uEnd[1];
}
else
{
acadoWorkspace.state[18] = acadoVariables.u[118];
acadoWorkspace.state[19] = acadoVariables.u[119];
}
acadoWorkspace.state[20] = acadoVariables.od[300];
acadoWorkspace.state[21] = acadoVariables.od[301];
acadoWorkspace.state[22] = acadoVariables.od[302];
acadoWorkspace.state[23] = acadoVariables.od[303];
acadoWorkspace.state[24] = acadoVariables.od[304];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[180] = acadoWorkspace.state[0];
acadoVariables.x[181] = acadoWorkspace.state[1];
acadoVariables.x[182] = acadoWorkspace.state[2];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 59; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[118] = uEnd[0];
acadoVariables.u[119] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79] + acadoWorkspace.g[80]*acadoWorkspace.x[80] + acadoWorkspace.g[81]*acadoWorkspace.x[81] + acadoWorkspace.g[82]*acadoWorkspace.x[82] + acadoWorkspace.g[83]*acadoWorkspace.x[83] + acadoWorkspace.g[84]*acadoWorkspace.x[84] + acadoWorkspace.g[85]*acadoWorkspace.x[85] + acadoWorkspace.g[86]*acadoWorkspace.x[86] + acadoWorkspace.g[87]*acadoWorkspace.x[87] + acadoWorkspace.g[88]*acadoWorkspace.x[88] + acadoWorkspace.g[89]*acadoWorkspace.x[89] + acadoWorkspace.g[90]*acadoWorkspace.x[90] + acadoWorkspace.g[91]*acadoWorkspace.x[91] + acadoWorkspace.g[92]*acadoWorkspace.x[92] + acadoWorkspace.g[93]*acadoWorkspace.x[93] + acadoWorkspace.g[94]*acadoWorkspace.x[94] + acadoWorkspace.g[95]*acadoWorkspace.x[95] + acadoWorkspace.g[96]*acadoWorkspace.x[96] + acadoWorkspace.g[97]*acadoWorkspace.x[97] + acadoWorkspace.g[98]*acadoWorkspace.x[98] + acadoWorkspace.g[99]*acadoWorkspace.x[99] + acadoWorkspace.g[100]*acadoWorkspace.x[100] + acadoWorkspace.g[101]*acadoWorkspace.x[101] + acadoWorkspace.g[102]*acadoWorkspace.x[102] + acadoWorkspace.g[103]*acadoWorkspace.x[103] + acadoWorkspace.g[104]*acadoWorkspace.x[104] + acadoWorkspace.g[105]*acadoWorkspace.x[105] + acadoWorkspace.g[106]*acadoWorkspace.x[106] + acadoWorkspace.g[107]*acadoWorkspace.x[107] + acadoWorkspace.g[108]*acadoWorkspace.x[108] + acadoWorkspace.g[109]*acadoWorkspace.x[109] + acadoWorkspace.g[110]*acadoWorkspace.x[110] + acadoWorkspace.g[111]*acadoWorkspace.x[111] + acadoWorkspace.g[112]*acadoWorkspace.x[112] + acadoWorkspace.g[113]*acadoWorkspace.x[113] + acadoWorkspace.g[114]*acadoWorkspace.x[114] + acadoWorkspace.g[115]*acadoWorkspace.x[115] + acadoWorkspace.g[116]*acadoWorkspace.x[116] + acadoWorkspace.g[117]*acadoWorkspace.x[117] + acadoWorkspace.g[118]*acadoWorkspace.x[118] + acadoWorkspace.g[119]*acadoWorkspace.x[119];
kkt = fabs( kkt );
for (index = 0; index < 120; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 5 */
real_t tmpDy[ 5 ];

/** Row vector of size: 3 */
real_t tmpDyN[ 3 ];

for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[5] = acadoVariables.od[lRun1 * 5];
acadoWorkspace.objValueIn[6] = acadoVariables.od[lRun1 * 5 + 1];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 5 + 2];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 5 + 3];
acadoWorkspace.objValueIn[9] = acadoVariables.od[lRun1 * 5 + 4];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 5] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 5];
acadoWorkspace.Dy[lRun1 * 5 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 5 + 1];
acadoWorkspace.Dy[lRun1 * 5 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 5 + 2];
acadoWorkspace.Dy[lRun1 * 5 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 5 + 3];
acadoWorkspace.Dy[lRun1 * 5 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 5 + 4];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[180];
acadoWorkspace.objValueIn[1] = acadoVariables.x[181];
acadoWorkspace.objValueIn[2] = acadoVariables.x[182];
acadoWorkspace.objValueIn[3] = acadoVariables.od[300];
acadoWorkspace.objValueIn[4] = acadoVariables.od[301];
acadoWorkspace.objValueIn[5] = acadoVariables.od[302];
acadoWorkspace.objValueIn[6] = acadoVariables.od[303];
acadoWorkspace.objValueIn[7] = acadoVariables.od[304];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 5]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 5 + 1]*acadoVariables.W[6];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 5 + 2]*acadoVariables.W[12];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 5 + 3]*acadoVariables.W[18];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 5 + 4]*acadoVariables.W[24];
objVal += + acadoWorkspace.Dy[lRun1 * 5]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 5 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 5 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 5 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 5 + 4]*tmpDy[4];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[4];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[8];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2];

objVal *= 0.5;
return objVal;
}

