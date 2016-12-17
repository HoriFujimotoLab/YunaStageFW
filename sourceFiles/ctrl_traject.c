/************************************************************************************
TRAJECTORY GENERATION MODULE
----------------------------
Descr.:		Control module for reference trajectory design
Boards:		PE-Expert3, C6713A DSP (float calculation)
Author:		Thomas Beauduin, University of Tokyo, 2016
*************************************************************************************/
#include	"ctrl_traject.h"
#include	"system_math.h"
#include	"data/system_data.h"
#include	"data/Rqlog1_250.h"
#include	"data/ctrl_traject_par.h"

// MODULE VAR
// Global: reference generation
float Aref = 0.0, Fref = 0.0;
enum ref reftype_e = REF_OFF;
int nroft = NROFS;
// Local:
int ref = 0;
double t = 0.0;


void ctrl_traject_ref(int reftype_e, float Aref, float Fref, float *x_ref)
{
	switch (reftype_e)
	{
	case 0:	*x_ref = 0.0;							break;
	case 1:	*x_ref = Aref;							break;
	case 2:	*x_ref = Aref*sindp(Fref*PI(2)*t);
			t += (TS*1.0e-6);						break;
	case 3: ctrl_traject_lpf(Aref, &*x_ref);		break;
	case 4: if (t < 0.5) { *x_ref = 0.0; }
			else { *x_ref = Aref; }
			t += (TS*1.0e-6);						break;
	case 5: *x_ref = Aref*refvec[ref];
			if (ref < (NROFS - 1))	{ ref++; }
			else					{ ref = 0; }	break;
	}
}


void ctrl_traject_lpf(float u_org, float *u_lpf)
{
	float u[1] = { 0.0 };
	u[0] = u_org;
	math_output(Ctlpf[0], xtlpf, Dtlpf[0], u, u_lpf, 2, 1, 1);
	math_state(Atlpf[0], xtlpf, Btlpf[0], u, xtlpf, 2, 1);
}


void ctrl_traject_reset()
{
	xtlpf[0] = 0.0; xtlpf[1] = 0.0;
}
