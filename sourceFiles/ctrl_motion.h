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
extern float pos_ref, pos_err;			// [m]
extern float yd, time_ff;
extern float Kff;						// [-]
extern int yd_nano, pos_ref_nano;		// [nm]


/*	1ST ORDER LPF FOR INPUT SHAPING
**	------------------------
**	DES:	LPF for shaped stepped reference
**	INP:	pos_ref_org	: original step reference
**	OUT:	pos_ref_sh	: shaped step reference
*/
void ctrl_motion_lpf1(float pos_ref_org, float *pos_ref_sh);


/*	PLANT SHAPING FILTER
**	--------------------
**	DES:	2nd order plant shaping filetr for 1st mode
**	INP:	u_org	: original current reference
**	OUT:	u_sh	: shaped current reference
*/
void ctrl_motion_sh1(float u_org, float *u_sh);


/*	PID POSITION CTRL
**	-----------------
**	DES:	pid control of stage position (pole placement)
**	INP:	xm_ref	: reference position
**			x_msr	: measured position
**	OUT:	iq_ref	: calculated current reference
*/
void ctrl_motion_pid_nano(int x_ref, int x_msr, float *iq_ref);


/*	RESET CONTROL MODULE
**	--------------------
**	DES:	resets module internal counters and variables
**			necessary at measurement init for good startup
*/
void ctrl_motion_reset(void);


#endif
