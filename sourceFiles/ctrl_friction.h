/************************************************************************************
FRICTION MODULE
----------------
Descr.:		Module for identification/compensation of friction
Boards:		PE-Expert3, C6713A DSP (float calculation)
Author:		Thomas Beauduin, University of Tokyo, December 2016
*************************************************************************************/
#ifndef	CTRL_FRICTION_H
#define	CTRL_FRICTION_H

// COMMAND VAR
extern float time_msr;
extern int vel;
extern float Afric;
extern float iq_fric;

/*	FRIC IDENT
**	----------
**	DES:	
*/
void ctrl_friction_stribeck(float xm, float xh, float *xo);
void ctrl_friction_hyster(float theta_m, float theta_h);
void ctrl_friction_lag(float xm, float xh, float *xo);
void ctrl_friction_gms(float *p_ref, float *iq_fric);
void ctrl_friction_drift(float *iq_ref);


#endif

