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

/*	FRIC IDENT
**	----------
**	DES:	
*/
void ctrl_friction_stribeck(float theta_m, float theta_h, float *theta_mo);
void ctrl_friction_hyster(float theta_m, float theta_h);


#endif

