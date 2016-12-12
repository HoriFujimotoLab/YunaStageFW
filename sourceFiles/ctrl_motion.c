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
float pos_err = 0.0;
float v_ref = 0.0, v_lpf = 0.0;
float i_shp = 0.0;
float yd = 0.0, time_ff = 0.0;
int yd_nano = 0, pos_ref_nano = 0;
float Kff = 1.0;


void ctrl_motion_lpf(float u_org, float *u_lpf)
{
	float u[1] = { 0.0 };
	u[0] = u_org;
	math_output(Clpf[0], xlpf, Dlpf[0], u, u_lpf, 2, 1, 1);
	math_state(Alpf[0], xlpf, Blpf[0], u, xlpf, 2, 1);
	if (fabsf(*u_lpf) > I_PK) { *u_lpf = sign(*u_lpf) * I_PK; }		// limit torque
}


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


/*
void ctrl_motion_pid_nano(int ix_ref, int ix_msr, float *iq_ref)
{
	float x_err[1] = { 0.0 };
	x_err[0] = ((float)(ix_ref - ix_msr))*1e-9;
	math_output(Cpid[0], xpid, Dpid[0], x_err, iq_ref, 2, 1, 1);
	math_state(Apid[0], xpid, Bpid[0], x_err, xpid, 2, 1);
	if (fabsf(*iq_ref) > I_PK) { *iq_ref = sign(*iq_ref) * I_PK; }		// limit torque
}
*/

void ctrl_motion_reset(void)
{
	xlpf[0] = 0.0; xlpf[1] = 0.0;
	xvpi[0] = 0.0;
	xshp[0] = 0.0; xshp[1] = 0.0;
}
