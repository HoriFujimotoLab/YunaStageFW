/************************************************************************************
PROCESSOR-IN-THE-LOOP
---------------------
Descr.:		
System:		
Author:		Thomas Beauduin, University of Tokyo, 2015
*************************************************************************************/
#include "system_fsm.h"

// TIMER COMP
#define ALPHA (0.2)
#define	PEV_BDN	0	
#define	INV_CH	0
float t0 = 0.0, t0_a = 0.0;

// TEMP
float A[3][3] = { { 1.0, 2.0, 3.0 },{ 3.0, 4.0, 5.0 },{ 6.0, 7.0, 8.0 } };
float B[3][1] = { { 3.0 },{ 4.0 },{ 5.0 } };
float C[1][3] = { { 5.0, 2.0, 1.0 } };
float D[1][1] = { { 1.0 } };

float u[1] = { 5.0 };
float v[1] = { 5.0 };
float x[3] = { 2.0, 7.0, 1.0 };
float y[3] = { 0.0 };

float z[1] = { 2.0 };
float Cx[4] = { 0.0 };
float Du[4] = { 0.0 };
float Ax[3] = { 0.0 };

float T[2][2] = {0.0};

int nrofs = 3;
int nrofi = 1;
int nrofo = 1;
float r = 2.0;

float test1 = 0.0;
float test2 = 0.0;
float test3 = 0.0;
float test4 = 0.0;

interrupt void system_tint0(void);


void main(void)
{
	int_disable();

	// TIMER INIT
	timer0_init(TS);
	timer0_init_vector(system_tint0);
	timer0_start();
	timer0_enable_int();
	watch_init();
	watch_data();

	// TEST CODE INIT
	system_fsm_init();

	// INVERTER INIT
	pev_inverter_init(PEV_BDN, INV_CH, FC, DT);
	pev_inverter_set_uvw(PEV_BDN, INV_CH, 0.0, 0.0, 0.0);
	wait(TC);
	pev_inverter_start_pwm(PEV_BDN, INV_CH);

	int_enable();

	//SYSTEM RUN
	while (1) { system_fsm_mode(); }
}


void system_tint0(void)
{
	time += (TS*1.0e-6);
	// TEST CODE
	// ---------
	if (msr >= 0 && msr < nroft) {

		ctrl_motion_dob(iq_ref, omega_m, &iq_dob);
		//mat_mul(A[0], 3, 3, x, 1, Ax);

		//math_state(A[0], x, B[0], u, y, nrofs, nrofi);
		//math_output(C[0], x, D[0], u, z, nrofs, nrofi, nrofo);

		msr++;
	}

	// TIMER COUNT
	// -----------
	t0 = timer0_read();
	t0_a = ALPHA * (t0 * 17.7777e-3) + (1 - ALPHA) * t0_a;
	//if (msr >= 0 && msr < nroft) { msr++; }

	//test1 = y[0];
	test2 = y[1];
	test3 = Ax[0];
	test4 = Ax[1];

	watch_data_8ch();
}



