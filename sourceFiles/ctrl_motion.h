/************************************************************************************
MOTION CONTROL MODULE
----------------------
Descr.:		Control module for stage motion control
Boards:		PE-Expert3, C6713A DSP (float calculation)
Author:		Thomas Beauduin, Wataru Ohnishi
			Hori-Fujimoto Lab, University of Tokyo, 2016
*************************************************************************************/
#ifndef	CTRL_MOTION_H
#define	CTRL_MOTION_H

// MODULE VAR
// Global: motion control variables
extern float p_ref, v_ref;
extern float i_shp, r_lpf;
extern float iq_dob;
extern enum ctrl {
	LPF = 2,
	SHP = 5,							// 
	VPI = 3,							// 
	PPI = 4,							// 
	HOM = 1
} ctrltype_e;


/*	1ST ORDER LPF FOR INPUT SHAPING
**	------------------------
**	DES:	LPF for shaped stepped reference
**	INP:	pos_ref_org	: original step reference
**	OUT:	pos_ref_sh	: shaped step reference
*/
void ctrl_motion_lpf(float u_org, float *u_lpf);
void ctrl_motion_dob(float i_ref, float v_msr, float *i_dob);

/*	PLANT SHAPING FILTER
**	--------------------
**	DES:	2nd order plant shaping filetr for 1st mode
**	INP:	u_org	: original current reference
**	OUT:	u_sh	: shaped current reference
*/
void ctrl_motion_shp(float u_org, float *u_sh);


/*	PID POSITION CTRL
**	-----------------
**	DES:	pid control of stage position (pole placement)
**	INP:	xm_ref	: reference position
**			x_msr	: measured position
**	OUT:	iq_ref	: calculated current reference
*/
void ctrl_motion_vpi(float v_ref, float v_msr, float *i_ref);
void ctrl_motion_ppi(float p_ref, float p_msr, float p_off, float *v_ref);
void ctrl_motion_hom(float v_ref, float v_msr, float *i_ref);
//void ctrl_motion_pid_nano(int x_ref, int x_msr, float *iq_ref);


/*	RESET CONTROL MODULE
**	--------------------
**	DES:	resets module internal counters and variables
**			necessary at measurement init for good startup
*/
void ctrl_motion_reset(int ctrl);


#endif
