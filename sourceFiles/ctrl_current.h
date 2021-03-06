/************************************************************************************
CURRENT CONTROL MODULE
----------------------
Descr.:		Control module for drive current control
Boards:		PE-Expert3, C6713A DSP (float calculation)
Author:		Thomas Beauduin, Wataru Ohnishi
			Hori-Fujimoto lab, University of Tokyo, 2016
*************************************************************************************/
#ifndef	CTRL_CURRENT_H
#define	CTRL_CURRENT_H

// MODULE PAR
// Global: inverter control values
extern float	vd_ref, vq_ref, id_ref, iq_ref;
extern float	va_ref, vb_ref, vu_ref, vv_ref, vw_ref;
extern float	iq_ref_ff, iq_ref_fb, iq_pid;


/*	POLE-ZERO CANCEL PI
**	-------------------
**	DES:	d/q-axis PI current control by pole zero cancel PI
**	INP:	[A,B,C,D] : state space matrix of the current controller
**			iq_ref	: q-axis reference current
**			id/q_ad	: dq-axis measured current
**	OUT:	vd/q_ref: dq-axis reference voltage
*/
//void ctrl_current_pzcpi(float iq_ref, float id_ad, float iq_ad, float *vd_ref, float *vq_ref);

/*	POLE-ZERO CANCEL PI [A,B,C,D] design
**	-------------------
**	DES:	pole zero cancel PI current controller online design
**	INP:	Fbw[Hz]	  : desired bandwidwh (closed loop pole) of the current controller
**						typical value Fbw = 500 [Hz]
**	OUT:	[A,B,C,D] : state space matrix of the current controller
*/
//void ctrl_current_pzcpi_init(float Fbw, float *Ai_pzcpi, float *Bi_pzcpi, float *Ci_pzcpi, float *Di_pzcpi);


/*	ZERO-CANCEL PI CTRL
**	-------------------
**	DES:	d/q-axis PI current control with zero-cancel ff
**	INP:	iq_ref	: q-axis reference current (id_ref = 0.0)
**			id/q_ad	: dq-axis measured current 
**	OUT:	vd/q_ref: dq-axis reference voltage 
*/
void ctrl_current_zcpi(float iq_ref, float id_ad, float iq_ad, float *vd_ref, float *vq_ref);


/*	ZERO-CANCEL PI CTRL DESIGN
**	-------------------
**	DES:	design of d/q-axis PI current control with zero-cancel ff
**			Rs, Ls, and pi are pre-calculated
**			code is generated by I_currentCtrl_sym.m
**	INP:	Fbw		: closed loop of current fb control
**			zeta	: damping factor of the closed loop denominator
**	OUT:	[Affi,Bffi,Cffi,Dffi], [Afbi,Bfbi,Cfbi,Dfbi]
*/
//void ctrl_current_zcpi_init(float Fbw, float zeta, float *Affi, float *Bffi, float *Cffi, float *Dffi, float *Afbi, float *Bfbi, float *Cfbi, float *Dfbi);


/*	DQ DECOUPLING CTRL
**	------------------
**	DES:	feedforward dq-axis decoupling ctrl
**	INP:	omega_m : measured motor angular velocity
**			id/q_ad	: dq-axis measured current
**	OUT:	vd/q_ref: decoupled dq-axis reference voltage 
*/
void ctrl_current_dec(float omega_m, float id_ad, float iq_ad, float *vd_ref, float *vq_ref);


/*	DEADTIME COMPENSATION CTRL
**	--------------------------
**	DES:	hysteresis compensation of inverter deadtime distortion
**	INP:	iq_ref	  : q-axis reference current (hysteresis input) 
**			theta_e	  : motor electrical angle (axis-transform)
**			vdc_ad	  : measured dc-bus voltage
**	OUT:	vu/v/w_ref: calculated phase voltage references
*/
void ctrl_current_dtc(float iq_ref, float theta_e, float vdc_ad, float *vu_ref, float *vv_ref, float *vw_ref);


/*	REFERENCE FRAMES TRANFORM
**	-------------------------
**	DES:	motor reference frame transformation for FOC
**	INP:	uvw-axis: phase reference frame
**			ab-axis	: stationary reference frame
**	OUT:	dq-axis	: rotating reference frame
*/
void ctrl_current_uw2ab(float u, float w, float *a, float *b);
void ctrl_current_ul2ab(float uv, float vw, float *a, float *b);
void ctrl_current_ab2dq(float a, float b, float theta_e, float *d, float *q);
void ctrl_current_dq2ab(float d, float q, float theta_e, float *a, float *b);
void ctrl_current_ab2uvw(float a, float b, float *u, float *v, float *w);


/*	RESET CONTROL MODULE
**	--------------------
**	DES:	resets module internal counters and variables
**			necessary at measurement init for good startup
*/
void ctrl_current_reset(void);

#endif
