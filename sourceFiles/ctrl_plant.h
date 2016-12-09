/************************************************************************************
SYSTEM PLANT MODULE
-------------------
Descr.:		Plant simulation module for control validation
Author:		Thomas Beauduin, University of Tokyo, 2015
*************************************************************************************/
#ifndef	CTRL_PLANT_H
#define	CTRL_PLANT_H

#include	"system_data.h"

//	MODULE VARIABLES
extern float iq0_ad;
extern float pos_m;
extern float pos_l;
extern float pos_fb;
extern int msr;


/*	MECHANICAL PLANT SIM
**	--------------------
**	DES:	simulate mechanical plant
**	INP:	/
**			/
**	OUT:	/
*/
void ctrl_plant_mech(float iq_ad, float *pos_m, float *pos_l);


/*	ELECTRICAL PLANT SIM
**	--------------------
**	DES:	simulate mechanical plant
**	INP:	/
**			/
**	OUT:	/
*/
void ctrl_plant_elec(float iq_ref, float *iq_ad);


#endif