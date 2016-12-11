/************************************************************************************
GENERAL FILTER DESIGN
----------------------
Descr.:		Filter design functions
Author:		Wataru Ohnishi, The University of Tokyo
*************************************************************************************/
#include "ctrl_design.h"
#include "system_math.h"
#include "data/system_data.h"


void ctrl_FiltInit1(float b1, float b0, float a1, float a0, float Ts, float *A, float *B, float *C, float *D)
{
	a0 = (double)a0;
	a1 = (double)a1;
	b0 = (double)b0;
	b1 = (double)b1;
	Ts = (double)Ts;

	*A = (float)((2 * a1 - Ts*a0) / (2 * a1 + Ts*a0));
	*B = (float)(1 / (2 * a1 + Ts*a0) / (2 * a1 + Ts*a0));
	*C = (float)(-4 * Ts*(a0*b1 - a1*b0));
	*D = (float)((2 * b1 + Ts*b0) / (2 * a1 + Ts*a0));

}


void ctrl_FiltInit2(float b2, float b1, float b0, float a2, float a1, float a0, float Ts, float *A, float *B, float *C, float *D)
{
	a0 = (double)a0;
	a1 = (double)a1;
	a2 = (double)a2;
	b0 = (double)b0;
	b1 = (double)b1;
	b2 = (double)b2;
	Ts = (double)Ts;

	double a2_d = (a0*Ts*Ts + 2 * a1*Ts + 4 * a2);

	A[0] = (float)0;
	A[2] = (float)1;
	A[1] = (float)(-(a0*Ts*Ts - 2 * a1*Ts + 4 * a2) / a2_d);
	A[3] = (float)(-2 * a0*Ts*Ts + 8 * a2) / a2_d;

	B[0] = (float)0;
	B[1] = (float)(1 / a2_d / a2_d);

	C[0] = (float)(4 * Ts*(4 * a1*b2 - 4 * a2*b1 - Ts*Ts * a0*b1 + Ts*Ts * a1*b0));
	C[1] = (float)(-4 * Ts*(4 * a1*b2 - 4 * a2*b1 + 4 * Ts*a0*b2 - 4 * Ts*a2*b0 + Ts*Ts * a0*b1 - Ts*Ts * a1*b0));

	*D = (float)((b0*Ts*Ts + 2 * b1*Ts + 4 * b2) / a2_d);

}


void ctrl_LPFInit1(float freqHz, float Ts, float *A, float *B, float *C, float *D)
{
	float wp = PI(2)*freqHz;
	ctrl_FiltInit1(0, wp, 1, wp, Ts, A, B, C, D);
}

void ctrl_HPFInit1(float freqHz, float Ts, float *A, float *B, float *C, float *D)
{
	double wp = PI(2)*(double)freqHz;
	ctrl_FiltInit1(1, 0, 1, wp, Ts, &*A, B, C, D);
}

void ctrl_LPFInit2(float freqHz, float dp, float Ts, float *A, float *B, float *C, float *D)
{
	float wp = PI(2)*freqHz;
	ctrl_FiltInit2(0, 0, wp*wp, 1, 2*dp*wp, wp*wp, Ts, A, B, C, D);
}


void ctrl_PID(float mn, float bn, float kn, float Ktn, float fp, float Ts, float *A, float *B, float *C, float *D)
{
	float b2 = -(-bn*bn + 8.0 * PI(1)*bn*fp*mn - 24.0 * PI(1)* PI(1) * fp* fp * mn* mn + kn*mn) / (Ktn*mn);
	float b1 = (32.0 * PI(1)* PI(1)* PI(1) * fp* fp* fp * mn* mn - 8.0 * kn*PI(1)*fp*mn + bn*kn) / (Ktn*mn);
	float b0 = (16.0 * fp * fp * fp * fp * mn*PI(1)*PI(1)*PI(1)*PI(1)) / Ktn;

	float a1 = -(bn - 8.0 * PI(1)*fp*mn) / mn;

	ctrl_FiltInit2(b2, b1, b0, 1, a1, 0, Ts, A, B, C, D);
}



