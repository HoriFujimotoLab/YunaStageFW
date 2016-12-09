/************************************************************************************
TRAJECTORY GENERATION MODULE
----------------------------
Descr.:		Control module for reference trajectory design
Boards:		PE-Expert3, C6713A DSP (float calculation)
Author:		Thomas Beauduin, Wataru Ohnishi
			Hori-Fujimoto Lab, University of Tokyo, 2016
*************************************************************************************/
#ifndef	CTRL_TRAJECT_H
#define	CTRL_TRAJECT_H

#define		NROFB	(40)			// number of element in time delay buffer

// COMMAND VAR
extern float Aref, Fref;
extern enum ref {
	REF_OFF = 0, REF_CST = 1, REF_SIN = 2,
	REF_FDI = 3, REF_TDI = 4, REF_EXT = 5
} reftype_e;
extern int nroft;

// DELAY VAR
extern int buffer_m[NROFB], buffer_l[NROFB];
extern int in_idx, m_idx, l_idx;
extern int delay_m, delay_l;


/*	MOTION REF GENERATION
**	---------------------
**	DES:	generate reference trajectory
**	INP:	iqref_e	: reference type (enumerate)
**			Aref	: amplitude of reference signal
**	OUT:	x_ref	: calculated motion reference
*/
void ctrl_traject_ref(int reftype_e, float pos_fb, float Aref, float Fref, float *x_ref);

void ctrl_traject_init(int vel_m_nano, int vel_l_nano);

#endif

