/************************************************************************************
TRAJECTORY GENERATION MODULE
----------------------------
Descr.:		Control module for reference trajectory design
Boards:		PE-Expert3, C6713A DSP (float calculation)
Author:		Thomas Beauduin, Wataru Ohnishi
			Hori-Fujimoto Lab, University of Tokyo, 2016
*************************************************************************************/
#include	"ctrl_traject.h"
#include	"ctrl_math.h"
#include	"system_data.h"
//#include	"data/Rqlog1_1k.h"
//#include	"data/Rqlog1_120.h"
#include	"data/Rqlog121_1k.h"
//#include	"data/Rqlog400_1k.h"

// MODULE VAR
// Global: reference generation
float Aref = 0.0, Fref = 0.0;
enum ref reftype_e = REF_OFF;
int nroft = NROFS;

// Global: time delay
int buffer_m[NROFB], buffer_l[NROFB];
int in_idx = 0, m_idx = 0, l_idx = 0;
int delay_m = 8, delay_l = 11;
// Local:
int ref = 0;
double t = 0.0;


void ctrl_traject_ref(int reftype_e, float pos_fb, float Aref, float Fref, float *x_ref)
{
	switch (reftype_e)
	{
	case 0:	*x_ref = pos_fb;						break;
	case 1:	*x_ref = Aref;							break;
	case 2:	*x_ref = Aref*sindp(Fref*PI(2)*t);
			t += (TS*1.0e-6);						break;
	case 3: ctrl_motion_lpf1(Aref, &*x_ref);		break;
	case 4: if (t < 0.5) { *x_ref = 0.0; }
			else { *x_ref = Aref; }
			t += (TS*1.0e-6);						break;
	case 5: *x_ref = Aref*refvec[ref];
			if (ref < (NROFS - 1))	{ ref++; }
			else					{ ref = 0; }	break;
	}
}


void ctrl_traject_init(int vel_m_nano, int vel_l_nano)
{
	int k;

	for (k = 0; k < NROFB; k++) {
		buffer_m[k] = vel_m_nano;
		buffer_l[k] = vel_l_nano;
	}
}

