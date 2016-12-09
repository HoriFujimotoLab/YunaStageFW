/************************************************************************************
C# CONTROL VALIDATION
---------------------
Content:		simulate developed c# code for numeric validation
System:			windows console application	(win 32)
Author:			Thomas Beauduin, Wataru Ohnishi
				Hori-Fujimoto Lab, University of Tokyo, 2015
*************************************************************************************/
#include	"ctrl_motion.h"
#include	"ctrl_traject.h"
#include	"ctrl_plant.h"
#include	"ctrl_math.h"
#include	"system_data.h"
#include	<stdio.h>
#include	<string.h>


float A[2][2] = { { 1.0, 2.0 },{ 3.0, 4.0 } };
float B[2][1] = { { 1.0 },{ 2.0 } };
float C[1][2] = { {1.0, 1.0} };
float D[1][1] = { {2.0} };
float x[2] = { 1.0, 0.0 };
float u[1] = { 1.0 };
float y[1] = { 0.0 };
float r = 0.0;
float time = 0.0;

// SIMULATION PARAMETERS
#define NROFD	12
char header[] = "";
float *data[] = { &A };
char *fname = "data/Csim.csv";
#define STARTTIME 0.0
#define ENDTIME   4.0
#define STEPTIME  2.0


void main(void)
{
	FILE *fp0;
	fp0 = fopen(fname, "w");
	fprintf(fp0, "%d\n", NROFD);
	fprintf(fp0, "time,");
	fprintf(fp0, header);
	fprintf(fp0, "\n");

	for (double t_sim = STARTTIME; t_sim < ENDTIME;) {

		/*SIMUALTION-CODE------------------------------------------------*/




		/*---------------------------------------------------------------*/

		fprintf(fp0, "%f,", t_sim);
		for (int i = 0; i < NROFD; i++) {
			fprintf(fp0, "%f,", *data[i]);
		}
		fprintf(fp0, "\n");
		t_sim += (TS*1.0e-6);
	}
	fclose(fp0);
	printf("simulation finished\n");
}



/*FF-SIMULATION ************************************************************
#include "data/nofar/NPZI_force_ff.h"
#include "data/nofar/NPZI_time.h"
#include "data/nofar/NPZI_yd.h"
#include "data/nofar/NPZI_yref.h"
#include "data/nofar/ZPETC_force_ff.h"
#include "data/nofar/ZPETC_time.h"
#include "data/nofar/ZPETC_yd.h"
#include "data/nofar/ZPETC_yref.h"
#include "data/nofar/ZMETC_force_ff.h"
#include "data/nofar/ZMETC_time.h"
#include "data/nofar/ZMETC_yd.h"
#include "data/nofar/ZMETC_yref.h"
#include "data/nofar/PPTC_force_ff.h"
#include "data/nofar/PPTC_time.h"
#include "data/nofar/PPTC_yd.h"
#include "data/nofar/PPTC_yref.h"

char header[] = "pos_ref, yd, iq0_ref_ff, iq0_ref_fb, iq0_ref, iq0_pid, iq0_ad, pos_m, pos_l, pos_i, time_ff, pos_fb";
float *data[] = { &pos_ref, &yd, &iq0_ref_ff, &iq0_ref_fb, &iq0_ref, &iq0_pid, &iq0_ad, &pos_m, &pos_l, &pos_i, &time_ff, &pos_fb };

// imaginary measurement
float pos_i = 0.0;
float Lm_i = 0.30; // 0.17; // Encoder height for imaginary measurement
#define		Lm_l			(0.085)								// Encoder height for load side


if (STEPTIME - (TS*1.0e-6) / 2 < t_sim && t_sim < STEPTIME + (TS*1.0e-6) / 2) {
msr = 0;
}

if (msr == -1) {
ctrl_traject_ref(3, 0.0, Aref, 1.0, &yd);
pos_ref = yd;
iq0_ref_ff = 0.0;
}
else if (msr >= 0 && msr < NROFS) {

switch (posctrltype_e)
{
case 1:
iq0_ref_ff = Kff*NPZI_force_ff[msr];
yd = NPZI_yd[msr];
time_ff = NPZI_time[msr];
pos_ref = NPZI_yref[msr];
break;
case 2:
iq0_ref_ff = Kff*ZPETC_force_ff[msr];
yd = ZPETC_yd[msr];
time_ff = ZPETC_time[msr];
pos_ref = ZPETC_yref[msr];
break;
case 3:
iq0_ref_ff = Kff*ZMETC_force_ff[msr];
yd = ZMETC_yd[msr];
time_ff = ZMETC_time[msr];
pos_ref = ZMETC_yref[msr];
break;
case 4:
iq0_ref_ff = Kff*PPTC_force_ff[msr];
yd = PPTC_yd[msr];
time_ff = PPTC_time[msr];
pos_ref = PPTC_yref[msr];
break;
}

pos_ref = pos_ref + Aref; // table assumes initial position is 0
yd = yd + Aref;
msr++;
}
else {
pos_ref = PPTC_yd[NROFS - 1] + Aref; // table assumes initial position is 0
yd = PPTC_yd[NROFS - 1] + Aref;
iq0_ref_ff = 0.0;
}

pos_err = yd - pos_fb;
//		ctrl_motion_pid(yd, pos_i, &iq0_ref_fb);
ctrl_motion_pid(yd, pos_i, &iq0_pid);
ctrl_motion_sh1(iq0_pid, &iq0_ref_fb); // limit off
iq0_ref = iq0_ref_ff + iq0_ref_fb;

// plant
ctrl_plant_elec(iq0_ref, &iq0_ad);
ctrl_plant_mech(iq0_ad, &pos_m, &pos_l);
pos_i = pos_m + (Lm_i / Lm_l)*(pos_l - pos_m);
pos_fb = pos_i;
*************************************************************************/
/*
void main(void)
{
	ctrl_math_state(A[0], x, B[0], u, x, 2, 1);
	ctrl_math_output(C[0], x, D[0], u, y, 2, 1, 1);
	print_matrix(x, 2, 1);
	print_matrix(y, 1, 1);
	getchar();
}


void print_matrix(float X[], int row, int col)
{
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) { printf("%f \t", X[i*col + j]); }
		printf("\n");
	}
	printf("\n");
}
*/
