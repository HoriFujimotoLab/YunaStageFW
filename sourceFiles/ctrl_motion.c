/************************************************************************************
MOTION CONTROL MODULE
----------------------
Descr.:		Control module for stage motion control
Boards:		PE-Expert3, C6713A DSP (float calculation)
Author:		Thomas Beauduin, Wataru Ohnishi
			Hori-Fujimoto Lab, University of Tokyo, 2016
*************************************************************************************/
#include "ctrl_motion.h"
#include "ctrl_math.h"
#include "system_data.h"
#include "data/ctrl_motion_par.h"


// MODULE VAR
// Global: motion control variables
float pos_err = 0.0;
float yd = 0.0, time_ff = 0.0;
int yd_nano = 0, pos_ref_nano = 0;
float Kff = 1.0;


void ctrl_motion_lpf1(float pos_ref_org, float *pos_ref_sh)
{
	float u[1] = { 0.0 };
	u[0] = pos_ref_org;
	ctrl_math_output(Clpf1[0], xlpf1, Dlpf1[0], u, pos_ref_sh, 1, 1, 1);
	ctrl_math_state(Alpf1[0], xlpf1, Blpf1[0], u, xlpf1, 1, 1);
	/*
	if (*pos_ref_sh < OVP_MIN) {
		*pos_ref_sh = OVP_MIN;
	 }
	if (*pos_ref_sh > OVP_MAX) {
		*pos_ref_sh = OVP_MAX;
	}
	*/
}

void ctrl_motion_sh1(float u_org, float *u_sh)
{
	float u[1] = { 0.0 };
	u[0] = u_org;
	ctrl_math_output(Csh1[0], xsh1, Dsh1[0], u, u_sh, 2, 1, 1);
	ctrl_math_state(Ash1[0], xsh1, Bsh1[0], u, xsh1, 2, 1);
	if (fabsf(*u_sh) > I_PK) { *u_sh = sign(*u_sh) * I_PK; }		// limit torque
}


void ctrl_motion_pid_nano(int ix_ref, int ix_msr, float *iq_ref)
{
	float x_err[1] = { 0.0 };
	x_err[0] = ((float)(ix_ref - ix_msr))*1e-9;
	ctrl_math_output(Cpid[0], xpid, Dpid[0], x_err, iq_ref, 2, 1, 1);
	ctrl_math_state(Apid[0], xpid, Bpid[0], x_err, xpid, 2, 1);
	if (fabsf(*iq_ref) > I_PK) { *iq_ref = sign(*iq_ref) * I_PK; }		// limit torque
}


void ctrl_motion_reset(void)
{
	xlpf1[0] = 0.0;
	xpid[0] = 0.0; xpid[1] = 0.0;
	xsh1[0] = 0.0; xsh1[1] = 0.0;
}
