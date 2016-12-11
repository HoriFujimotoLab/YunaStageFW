/************************************************************************************
MOTOR ROTARY-ENCODER MODULE
---------------------------
Descr.:		Hardware module for motor encoder (00672)
Boards:		MWPE-Expert3, MWPE-PIOS (PIO expansion board)
Sensor:		20bit absolute rotary encoder (Tamagawa)
Author:		Thomas Beauduin, University of Tokyo, December 2016
*************************************************************************************/
#include "hardw_menc.h"
#include "system_math.h"
#include "data/system_data.h"
#include <mwio3.h>

// MODULE PAR
#define DAT_ADDR		0xA0070000					// myway defined data address
#define RST_ADDR		0xA0071000					// myway defined reset address
#define PIOS_BDN		0							// PIOS board dip switch number
#define ENC_OFF			(3.169)					    // setup encoder-rotor offset (M1:3.2427)
#define	ENC_RES			(1048576.0)					// 20bit resolution [cnt/turn]
#define	ENC_DIR			(-1.0)						// rotor electrical direction

// MODULE VAR
// global
float theta_m = 0.0, theta_e, omega_m = 0.0;
int theta_m_nano = 0, omega_m_nano = 0;

// local
static int nrofr = 0;
static int theta_nano_temp = 0;
static double theta_db_temp = 0.0;
static float theta_home = 0.0;
static float fs_m, alpha = 0.0;

void hardw_menc_init(int fs, int fc)
{
	unsigned int data_msr;

	// LAUNCH
	pios_pio_set_bit(PIOS_BDN, 0);					// bit0 DS40
	pios_pio_clr_bit(PIOS_BDN, 1);					// bit1 INRQ
	pios_pio_set_bit(PIOS_BDN, 2);					// bit2 RQC0
	pios_pio_set_bit(PIOS_BDN, 3);					// bit3 RQC1
	pios_pio_clr_bit(PIOS_BDN, 5);					// bit5 ABSMD (20bit)
	pios_pio_clr_bit(PIOS_BDN, 6);					// bit6 RESET

	// PAR
	fs_m = (float)fs;
	alpha = expsp(-PI(2)*(float)fc / fs_m);

	// READ 1e
	pios_pio_set_bit(PIOS_BDN, 4);											// bit4 RQSTB
	(*(volatile int*)(RST_ADDR + ((PIOS_BDN) << 14))); wait(3);				// reset flag
	pios_pio_clr_bit(PIOS_BDN, 4);											// bit4 RQSTB
	data_msr = (*(volatile int*)(DAT_ADDR + ((PIOS_BDN) << 14)) & 0x000FFFFF);
	theta_nano_temp = (int)data_msr;
	theta_db_temp = (double)(theta_nano_temp) / ENC_RES * PI(2);			// [cnt] to [rad]

}


void hardw_menc_read(int *theta_m_nano, float *theta_m, int *omega_m_nano, float *omega_m, float *theta_e)
{
	int i = 0, diff;
	unsigned int data_msr;
	double theta_db;
	float omega_temp = 0.0;

	// NANO
	pios_pio_set_bit(PIOS_BDN, 4);											// bit4 RQSTB
	(*(volatile int*)(RST_ADDR + ((PIOS_BDN) << 14))); wait(3);				// reset flag
	pios_pio_clr_bit(PIOS_BDN, 4);											// bit4 RQSTB
	data_msr = (*(volatile int*)(DAT_ADDR + ((PIOS_BDN) << 14)) & 0x000FFFFF);
	*theta_m_nano = (int)data_msr;
	diff = *theta_m_nano - theta_nano_temp;
	if (diff >  ENC_RES/2.0) { nrofr--; i--; }
	if (diff < -ENC_RES/2.0) { nrofr++; i++; }
	theta_nano_temp = *theta_m_nano;

	// ELEC
	theta_db = (double)(*theta_m_nano) / ENC_RES * PI(2);					// [cnt] to [rad]
	*theta_e = (float)(theta_db * ENC_DIR * Pp - ENC_OFF);					// mech to elec
	while (*theta_e > PI(2)) { *theta_e -= PI(2); }							// value limitation: {0, 2pi}
	while (*theta_e < 0)	 { *theta_e += PI(2); }

	// MECH
	omega_temp = *omega_m;
	*theta_m_nano = *theta_m_nano + nrofr * ENC_RES;
	*theta_m = (float)(theta_db) + nrofr * PI(2) - theta_home;				// full screw calc
	*omega_m_nano = (diff + i*ENC_RES) * fs_m;
	*omega_m = (float)((theta_db - theta_db_temp + i*PI(2)) * fs_m);
	*omega_m = *omega_m * (1.0 - alpha) + omega_temp * alpha;				// resursive iir maf
	theta_db_temp = theta_db;
}


void hardw_menc_home(void)
{
	nrofr = 0;
	theta_home = (float)theta_db_temp;
}


void hardw_menc_status(unsigned int *status)
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
