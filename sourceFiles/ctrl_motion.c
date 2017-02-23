/************************************************************************************
MOTION CONTROL MODULE
----------------------
Descr.:		Control module for stage motion control
Boards:		PE-Expert3, C6713A DSP (float calculation)
Author:		Thomas Beauduin, Wataru Ohnishi
			Hori-Fujimoto Lab, University of Tokyo, 2016
*************************************************************************************/
#include "ctrl_motion.h"
#include "system_math.h"
#include "data/system_data.h"
#include "data/ctrl_motion_par.h"

// MODULE VAR
// Global: motion control variables
float p_ref = 0.0;
float v_ref = 0.0, r_lpf = 0.0;
float i_shp = 0.0, iq_dob = 0.0;
enum ctrl ctrltype_e = LPF;


void ctrl_motion_shp(float u_org, float *u_sh)
{
	float u[1] = { 0.0 };
	u[0] = u_org;
	math_output(Cshp[0], xshp, Dshp[0], u, u_sh, 2, 1, 1);
	math_state(Ashp[0], xshp, Bshp[0], u, xshp, 2, 1);
	if (fabsf(*u_sh) > I_PK) { *u_sh = sign(*u_sh) * I_PK; }		// limit torque
}


void ctrl_motion_vpi(float v_ref, float v_msr, float *i_ref)
{
	float v_err[1] = { 0.0 };
	v_err[0] = v_ref - v_msr;
	math_output(Cvpi[0], xvpi, Dvpi[0], v_err, i_ref, 1, 1, 1);
	math_state(Avpi[0], xvpi, Bvpi[0], v_err, xvpi, 1, 1);
	if (fabsf(*i_ref) > I_PK) { *i_ref = sign(*i_ref) * I_PK; }		// limit torque
}


void ctrl_motion_ppi(float p_ref, float p_msr, float p_off, float *v_ref)
{
	float p_err[1] = { 0.0 };
	p_err[0] = p_off + p_ref - p_msr;
	math_output(Cppi[0], xppi, Dppi[0], p_err, v_ref, 1, 1, 1);
	math_state(Appi[0], xppi, Bppi[0], p_err, xppi, 1, 1);
	if (fabsf(*v_ref) > V_PK) { *v_ref = sign(*v_ref) * V_PK; }		// velocity limit
}

/*
void ctrl_motion_pd(float p_ref, float p_msr, float p_off, float *i_ref)
{
	float p_err[1] = { 0.0 };
	p_err[0] = p_off + p_ref - p_msr;
	math_output(Cpd[0], xpd, Dpd[0], p_err, i_ref, 1, 1, 1);
	math_state(Apd[0], xpd, Bpd[0], p_err, xpd, 1, 1);
	if (fabsf(*i_ref) > I_PK) { *i_ref = sign(*i_ref) * I_PK; }
}
*/

void ctrl_motion_pid(float p_ref, float p_msr, float p_off, float *i_ref)
{
	float p_err[1] = { 0.0 };
	p_err[0] = p_off + p_ref - p_msr;
	math_output(Cpid[0], xpid, Dpid[0], p_err, i_ref, 2, 1, 1);
	math_state(Apid[0], xpid, Bpid[0], p_err, xpid, 2, 1);
	if (fabsf(*i_ref) > I_PK) { *i_ref = sign(*i_ref) * I_PK; }
}


void ctrl_motion_hom(float v_ref, float v_msr, float *i_ref)
{
	float v_err[1] = { 0.0 };
	v_err[0] = v_ref - v_msr;
	math_output(Chom[0], xhom, Dhom[0], v_err, i_ref, 1, 1, 1);
	math_state(Ahom[0], xhom, Bhom[0], v_err, xhom, 1, 1);
	if (fabsf(*i_ref) > I_PK) { *i_ref = sign(*i_ref) * I_PK; }
}


void ctrl_motion_dob(float i_ref, float v_msr, float *i_dob)
{
	float vm[1] = { 0.0 };
	float ir[1] = { 0.0 };
	float i1[1] = { 0.0 };
	float i2[1] = { 0.0 };
	vm[0] = v_msr;
	math_output(Cdob[0], xdob, Ddob[0], vm, i1, 2, 1, 1);
	math_state(Adob[0], xdob, Bdob[0], vm, xdob, 2, 1);
	ir[0] = i_ref;
	math_output(Clpf[0], xlpf, Dlpf[0], ir, i2, 2, 1, 1);
	math_state(Alpf[0], xlpf, Blpf[0], ir, xlpf, 2, 1);
	*i_dob = i2[0] - i1[0];
	if (fabsf(*i_dob) > I_PK) { *i_dob = sign(*i_dob) * I_PK; }
}


void ctrl_motion_reset(int ctrltype_e)
{
	switch (ctrltype_e)
	{
	case 1: xhom[0] = 0.0;					break;
	case 2: xlpf[0] = 0.0; xlpf[1] = 0.0;	break;
	case 3: xvpi[0] = 0.0;					break;
	case 4: xppi[0] = 0.0;					break;
	case 5: xshp[0] = 0.0; xshp[1] = 0.0;	break;
//	case 6: xpd[0] = 0.0;					break;
	default:
		xhom[0] = 0.0;
		xlpf[0] = 0.0; xlpf[1] = 0.0;
		xvpi[0] = 0.0; xppi[0] = 0.0;
		xshp[0] = 0.0; xshp[1] = 0.0;
//		xpd[0] = 0.0;						
	break;
	}
}

