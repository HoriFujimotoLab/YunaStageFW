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


void ctrl_friction_stribeck(float theta_m, float theta_h, float *theta_mo)
{
	int state;

	tfric += (TS*1.0e-6);
	if (tfric > timevec[vel] && rtn == 0)		 { state = 1; }		// return moment
	if (tfric < timevec[vel] && rtn == 0)		 { state = 2; }		// constant vel
	if (rtn == 1)								 { state = 3; }		// return step
	if (theta_m < theta_h + 0.1 && theta_m > theta_h - 0.1 && rtn == 1) { state = 4; }		// start vel
	switch (state)
	{
	case 1: rtn = 1; v_ref = 0.0;								break;
	case 2: v_ref = Afric*fricvec[vel];
			*theta_mo = theta_m;								break;
	case 3: e_lpf = theta_h - *theta_mo;
			ctrl_traject_lpf(e_lpf, &p_ref);
			ctrl_motion_ppi(p_ref, theta_m, *theta_mo, &v_ref);	break;
	case 4: ctrl_traject_reset();	// lpf
			ctrl_motion_reset(3);	// vpi
			ctrl_motion_reset(4);	// ppi
			if (vel < (NROFS - 1)) { vel++; }
			else				   { vel = 0; }
			tfric = 0.0; rtn = 0; v_ref = 0.0;					break;
	}
}


void ctrl_friction_hyster(float theta_m, float theta_h)
{
	ctrl_traject_ref(reftype_e, Aref, Fref, &r_lpf);
	ctrl_traject_lpf(r_lpf, &p_ref);
	ctrl_motion_ppi(r_lpf, theta_m, theta_h, &v_ref);
}


// consider making friction ref functions here
