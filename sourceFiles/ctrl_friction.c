/************************************************************************************
FRICTION MODULE
----------------
Descr.:		Module for identification/compensation of friction
Boards:		PE-Expert3, C6713A DSP (float calculation)
Author:		Thomas Beauduin, University of Tokyo, December 2016
*************************************************************************************/
#include	"ctrl_friction.h"
#include	"ctrl_motion.h"
#include	"ctrl_traject.h"
#include	"system_math.h"
#include	"data/system_data.h"
#include	"data/ctrl_friction_ref.h"

// MODULE VAR
// Global: reference generation
float time_msr = 2.0;
float Afric = 1.0;
int vel = 0;
// Local:
double tfric = 0.0;
int rtn = 0;
float e_lpf = 0.0;
float res = 0.1;
float vo = 0.0;
//lpf model
float	Avlpf[2][2] = {
	{ 9.519775726340529e-01,9.220139303001995e-02 },
	{ -9.220139303002040e-02,7.675747865740119e-01 }
};
float	Bvlpf[2][1] = {
	{ -2.359999863025425e-01 },
	{ -1.934214687506972e-01 }
};
float	Cvlpf[1][2] = {
	{ -2.359999863025426e-01,1.934214687506974e-01 }
};
float	Dvlpf[1][1] = {
	{ 4.915679951610148e-03 }
};
float	xvlpf[2] = { 0.0e+00,0.0e+00 };


void ctrl_friction_vlpf(float u_org, float *u_lpf);
void ctrl_friction_reset(void);


void ctrl_friction_stribeck(float xm, float xh, float *xo)
{
	tfric += (TS*1.0e-6);
	if (tfric < timevec[vel] && rtn == 0)				// phase 1: constant vel ctrl
	{
		ctrl_friction_vlpf(Afric*velvec[vel], &v_ref);
		*xo = xm; vo = v_ref;
	}
	if (tfric > timevec[vel] && rtn == 0)				// phase 2: switch to pos ctrl
	{
		rtn = 1;
		ctrl_friction_reset();	//lpf-vel
	}
	if (rtn == 1)										// phase 3: return to init pos
	{
		ctrl_traject_lpf(xh - *xo, &p_ref);
		ctrl_motion_ppi(p_ref, xm, *xo, &v_ref);
		v_ref = v_ref + vo;
	}
	if (xm < xh + res && xm > xh - res && rtn == 1)		// phase 4: switch to vel ctrl
	{
		tfric = 0.0; rtn = 0; v_ref = 0.0;
		ctrl_traject_reset();	// lpf-pos
		ctrl_motion_reset(3);	// vpi
		ctrl_motion_reset(4);	// ppi
		if (vel < (NROFS - 1)) { vel++; }
		else				   { vel = 0; }
	}
}


// PPI: 4Hz, zeta 1.2/sqrt(2)
// VPI: 40Hz, zeta 1.2/sqrt(2)
// LPF: 0.75Hz, zeta 1
void ctrl_friction_hyster(float theta_m, float theta_h)
{
	ctrl_traject_ref(reftype_e, Aref, Fref, &r_lpf);
	ctrl_traject_lpf(r_lpf, &p_ref);
	ctrl_motion_ppi(r_lpf, theta_m, theta_h, &v_ref);
}


void ctrl_friction_vlpf(float u_org, float *u_lpf)
{
	float u[1] = { 0.0 };
	u[0] = u_org;
	math_output(Cvlpf[0], xvlpf, Dvlpf[0], u, u_lpf, 2, 1, 1);
	math_state(Avlpf[0], xvlpf, Bvlpf[0], u, xvlpf, 2, 1);
}

void ctrl_friction_reset(void)
{
	xvlpf[0] = 0.0; xvlpf[1] = 0.0;
}


// consider making friction ref functions here for g-code saw
// consider having friction controllers design here

