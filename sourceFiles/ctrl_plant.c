/************************************************************************************
SYSTEM PLANT MODULE
-------------------
Descr.:		Plant simulation module for control validation
Author:		Thomas Beauduin, University of Tokyo, 2015
*************************************************************************************/
#include "ctrl_plant.h"
#include "ctrl_math.h"
#include "data/ctrl_plant_par.h"

// MODULE VARIABLES
float iq0_ad = 0.0;
float pos_m = 0.0;
float pos_l = 0.0;
float pos_fb = 0.0;
int	msr = -1;


void ctrl_plant_mech(float iq_ad, float *pos_m, float *pos_l)
{
	float input[1] = { 0.0 };
	float output[2] = { 0.0 };
	input[0] = iq_ad;
	ctrl_math_output(Cmech[0], xmech, Dmech[0], input, output, 8, 1, 2);
	ctrl_math_state(Amech[0], xmech, Bmech[0], input, xmech, 8, 1);
	*pos_m = output[0];
	*pos_l = output[1];
}


void ctrl_plant_elec(float iq_ref, float *iq_ad)
{
	float input[1] = { 0.0 };
	float output[1] = { 0.0 };
	input[0] = iq_ref;
	ctrl_math_output(Celec[0], xelec, Delec[0], input, output, 2, 1, 1);
	ctrl_math_state(Aelec[0], xelec, Belec[0], input, xelec, 2, 1);
	*iq_ad = output[0];
}



/*
void file_init(FILE *fp0)
{
	fprintf(fp0, "%d\n", NROFD);
	fprintf(fp0, "time,");
	fprintf(fp0, header);
	fprintf(fp0, "\n");
}

void file_write(FILE *fp0, double t_sim)
{
	fprintf(fp0, "%f,", t_sim);
	for (int i = 0; i < NROFD; i++) {
		fprintf(fp0, "%f,", *data[i]);
	}
	fprintf(fp0, "\n");
}

*/