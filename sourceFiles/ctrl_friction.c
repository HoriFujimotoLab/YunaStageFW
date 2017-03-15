/************************************************************************************
FRICTION MODULE
----------------
Descr.:		Module for identification/compensation of friction
Boards:		PE-Expert3, C6713A DSP (float calculation)
Author:		Thomas Beauduin, University of Tokyo, December 2016
**************************************************************************************
Notes:		include controllers in friction functions
*/
#include	"ctrl_friction.h"
#include	"ctrl_motion.h"
#include	"ctrl_traject.h"
#include	"system_math.h"
#include	"data/system_data.h"

//#include	"data/ctrl_stribeck_ref.h"
#include	"data/Rgmsfric_d.h"
#include	"data/Rsawtooth2.h"

// MODULE VAR
// Global: reference generation
float time_msr = 2.0;
float Afric = 1.0;
int vel = 0; 
float iq_fric = 0.0;
// Local:
double tfric = 0.0;
int rtn = 0;
float e_lpf = 0.0;
float res = 0.1;
float vo = 0.0;
int rf = 0;

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

/*
// PPI: 2Hz, zeta 1.2/sqrt(2)
// VPI: 20Hz, zeta 1.2/sqrt(2)
// LPF: 1Hz, zeta 1
// increase vpi to 40 now that the max velocity has changed
void ctrl_friction_stribeck(float xm, float xh, float *xo)
{
	tfric += (TS*1.0e-6);
	if (tfric < timevec[vel] && rtn == 0)					// phase 1: constant vel ctrl
	{
		ctrl_friction_vlpf(Afric*velvec[vel], &v_ref);
		*xo = xm; vo = v_ref;
	}
	if (tfric > timevec[vel] && rtn == 0)					// phase 2: switch to pos ctrl
	{
		rtn = 1;
		ctrl_friction_reset();	//lpf-vel
	}
	if (rtn == 1)											// phase 3: return to init pos
	{
		ctrl_traject_lpf(xh - *xo, &p_ref);
		ctrl_motion_ppi(p_ref, xm, *xo, &v_ref);
		v_ref = v_ref + vo;
	}
	if (xm < xh + res && xm > xh - res && rtn == 1)			// phase 4: switch to vel ctrl
	{
		tfric = 0.0; rtn = 0; v_ref = 0.0;
		ctrl_traject_reset();	// lpf-pos
		ctrl_motion_reset(3);	// vpi
		ctrl_motion_reset(4);	// ppi
		if (vel < (NROFS - 1)) { vel++; }
		else				   { vel = 0; }
	}
}


void ctrl_friction_lag(float xm, float xh, float *xo)
{
	float vr;
	tfric += (TS*1.0e-6);
	if (tfric < timevec[vel] && rtn == 0)				// phase 1: constant vel ctrl
	{
		vr = velvec[vel] + Afric*velvec[vel]*sindp(Fref*PI(2)*tfric);
		ctrl_friction_vlpf(vr, &v_ref);
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
		else { vel = 0; }
	}
}
*/

// PPI: 10Hz, zeta 1.2/sqrt(2)
// VPI: 50Hz, zeta 1.2/sqrt(2)
// LPF: 50Hz, zeta 1
void ctrl_friction_hyster(float theta_m, float theta_h)
{
	ctrl_traject_ref(reftype_e, Aref, Fref, &p_ref);
	//ctrl_traject_lpf(r_lpf, &p_ref);
	ctrl_motion_ppi(p_ref, theta_m, theta_h, &v_ref);

	// remove traject module connection, not logic
}


void ctrl_friction_drift(float *iq_ref)
{
	ctrl_traject_ref(reftype_e, 1, 1, &*iq_ref);
	// make the reference directly in here instead
	// no connection with trajectory module
	// trajectory module is only for poly/bezier/...
}


void ctrl_friction_gms(float *p_ref, float *iq_fric)
{
	*iq_fric = iqvec[rf];
	*p_ref = refvec[rf];
	if (rf < (NROFS - 1)) { rf++; }
	else { rf = 0; }
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

