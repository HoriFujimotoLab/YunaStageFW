/************************************************************************************
MATH MODULE
-----------
Descr.:		mathematic module for algebra calculations
Boards:		PE-Expert3, C6713A DSP (floating point calculation)
Author:		Thomas Beauduin, University of Tokyo, 2016
*************************************************************************************/
#include "system_math.h"

#define	NMAX	(2)					// max amount of states

void math_state(float A[], float x[], float B[], float u[], float *dx, int nrofs, int nrofi)
{
	float Ax[NMAX] = { 0.0 }; float Bu[NMAX] = { 0.0 };
	switch (nrofs)
	{
	case 1:		vec_scale(A, x, Ax, nrofs);								break;		// SS
	default:	mat_mul(A, nrofs, nrofs, x, 1, Ax);						break;		// MS
	}
	switch (nrofi)
	{
	case 1:		vec_scale(B, u, Bu, nrofs);								break;		// SI
	default:	mat_mul(B, nrofs, nrofi, u, 1, Bu);						break;		// MI
	}
	vec_add(Ax, Bu, &*dx, nrofs);
}


//DSPF_sp_
void math_output(float C[], float x[], float D[], float u[], float *y, int nrofs, int nrofi, int nrofo)
{
	float Cx[NMAX] = { 0.0 }; float Du[NMAX] = { 0.0 };
	switch (nrofo)
	{
	case 1:		vec_dot(C, x, &Cx[0], nrofs);
				switch (nrofi)
				{
				case 1:		Du[0] = D[0] * u[0];						break;		// SISO
				default:	vec_dot(D, u, &Du[0], nrofi);				break;		// MISO
				}
	break;
	default:	mat_mul(C, nrofo, nrofs, x, 1, Cx);
				switch (nrofi)
				{
				case 1:		vec_scale(D, u, Du, nrofo);					break;		// SIMO
				default:	mat_mul(D, nrofo, nrofi, u, 1, Du);			break;		// MIMO
				}
	break;
	}
	vec_add(Cx, Du, y, nrofs);
}


void vec_add(float *x, float *y, float *r, int nr)
{
	int i;
	for (i = 0; i < nr; i++) {
		r[i] = x[i] + y[i];
	}
}
void vec_mul(float *x, float *y, float *r, int nr)
{
	int i;
	for (i = 0; i < nr; i++) {
		r[i] = x[i] * y[i];
	}
}
void vec_scale(float *x, float *w, float *r, int nr)
{
	int i;
	for (i = 0; i < nr; i++) {
		r[i] = *w * x[i];
	}
}
void vec_dot(float *x, float *y, float *r, int nr)
{
	int i; *r = 0.0;
	for (i = 0; i < nr; i++) {
		*r += x[i] * y[i];
	}
}


// temp
void mat_mul(float *x, int r1, int c1, float *y, int c2, float *r)
{
	int i, k;
	float sum;
	for (i = 0; i < r1; i++) {
		sum = 0.0;
		for (k = 0; k < c1; k++) { sum += x[i*c1 + k] * y[k]; }
		r[i] = sum;
	}
}


