/************************************************************************************
TRAJECTORY GENERATION MODULE
----------------------------
Descr.:		Control module for reference trajectory design
Boards:		PE-Expert3, C6713A DSP (float calculation)
Author:		Thomas Beauduin, University of Tokyo, 2016
*************************************************************************************/
#ifndef	CTRL_TRAJECT_H
#define	CTRL_TRAJECT_H

// COMMAND VAR
extern float Aref, Fref;
extern enum ref {
	REF_OFF = 0, REF_CST = 1, REF_SIN = 2,
	REF_FDI = 3, REF_TDI = 4, REF_EXT = 5
} reftype_e;
extern int nroft;


/*	MOTION REF GENERATION
**	---------------------
**	DES:	generate reference trajectory
**	INP:	iqref_e	: reference type (enumerate)
**			Aref	: amplitude of reference signal
**	OUT:	x_ref	: calculated motion reference
*/
void ctrl_traject_ref(int reftype_e, float Aref, float Fref, float *x_ref);


#endif

