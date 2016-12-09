/************************************************************************************
MOTOR ROTARY-ENCODER MODULE
---------------------------
Descr.:		Hardware module for motor encoder (00672)
Boards:		MWPE-Expert3, MWPE-PIOS (PIO expansion board)
Sensor:		20bit absolute rotary encoder (Tamagawa)
Author:		Thomas Beauduin, University of Tokyo, April 2015
*************************************************************************************/

#include "motor_enc.h"
#include <math.h>
#include <mwio3.h>

// MODULE PAR
#define DAT_ADDR		0xA0070000					// myway defined data address
#define RST_ADDR		0xA0071000					// myway defined reset address
#define PIOS_BDN		0							// PIOS board dip switch number
#define ENC_OFF			(3.169)					    // setup encoder-rotor offset (M1:3.2427)
#define	ENC_RES			(1048576.0)					// 20bit resolution [cnt/turn]
#define	ENC_DIR			(-1.0)						// encoder rotation direction
#define ALPHA			(0.05)						// recursive IIR MAF factor

// MODULE VAR
static int nrofr = 0;								// number of turns [-]
static double theta_msr;							// measured theta  [rad]

void motor_enc_init()
{
	pios_pio_set_bit(PIOS_BDN, 0);											// bit0 DS40
	pios_pio_clr_bit(PIOS_BDN, 1);											// bit1 INRQ
	pios_pio_set_bit(PIOS_BDN, 2);											// bit2 RQC0
	pios_pio_set_bit(PIOS_BDN, 3);											// bit3 RQC1
	pios_pio_clr_bit(PIOS_BDN, 5);											// bit5 ABSMD (20bit)
	pios_pio_clr_bit(PIOS_BDN, 6);											// bit6 RESET
}


void motor_enc_elec(float *theta_e)
{
	// LOCAL VAR
	unsigned int data_msr;													// measured data   [cnt]

	// READ DATA
	pios_pio_set_bit(PIOS_BDN, 4);											// bit4 RQSTB
	(*(volatile int*)(RST_ADDR + ((PIOS_BDN) << 14))); wait(3);				// reset flag
	pios_pio_clr_bit(PIOS_BDN, 4);											// bit4 RQSTB
	data_msr = (*(volatile int*)(DAT_ADDR + ((PIOS_BDN) << 14)) & 0x000FFFFF);
	theta_msr = (double)data_msr / ENC_RES * PI(2) * ENC_DIR;				// [cnt] to [rad]

	// ELEC
	*theta_e = (float)(theta_msr * Pp) - ENC_OFF;							// mech to elec
	while (*theta_e > PI(2)) { *theta_e -= PI(2); }							// value limitation:
	while (*theta_e < 0)	 { *theta_e += PI(2); }							// {0, 2pi}
}


void motor_enc_read(float *theta_m, float *omega_m, float *omega_ma)
{
	// LOCAL VAR
	double diff = 0.0;														// position diff   [rad]
	int i = 0;																// rev jump index  [-]

	// MECH
	diff = theta_msr - (double)(*theta_m - nrofr * PI(2));					// theta time difference
	if (diff >  PI(1)){ nrofr--; i--; }										// revolution calc
	if (diff < -PI(1)){ nrofr++; i++; }										// note: max vel pi*fs
	*theta_m = (float)(theta_msr + nrofr * PI(2));							// full screw pos calc
	*omega_m = (float)((diff + i*PI(2)) * FS);								// full screw vel calc

	// FILT
	*omega_ma = ALPHA * *omega_m + (1.0 - ALPHA) * *omega_ma;				// resursive iir maf
}


void motor_enc_reset(float *theta_m)
{
	*theta_m = *theta_m - nrofr * PI(2);									// remove rev from data
	nrofr = 0;																// reset number of rev
}


void motor_enc_status(unsigned int *status)
{
	*status = (((*(volatile int*)(DAT_ADDR + ((PIOS_BDN) << 14))) & 0x80000000) >> 31);
}


/* NOTES:
** ------
** consider an extension for glitch detection on theta_msr
** simplest solution is to check if diff is physically possible
** better is estimate next data point check difference, if to big
** use estimate and not sensor data
** combining both time diff and estimate diff is probably more robust
*/
