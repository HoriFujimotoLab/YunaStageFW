/************************************************************************************
STAGE ROTARY-ENCODER MODULE
---------------------------
Descr.:		Hardware module for stage encoder
Boards:		MWPE-Expert3, MWPE-FPGAA (custom board)
Sensor:		20bit absolute rotary encoder (nikon)
Author:		Thomas Beauduin, University of Tokyo, December 2016
*************************************************************************************/
#include	"hardw_senc.h"
#include	"system_math.h"
#include	"data/system_data.h"
#include	<mwio3.h>

// MODULE PAR
#define	FPGA_BDN	0						// FPGAA board number
#define	ENC_RES		(1048576.0)				// 20bit resolution [cnt/turn]
#define ENC_OFF		(2.125)					// setup encoder-rotor offset

// MODULE VAR
// global
float theta_s = 0.0, omega_s = 0.0;
int theta_s_nano = 0, omega_s_nano = 0;
// local
static int nrofr = 0;
static int theta_nano_temp;
static double theta_db_temp = 0.0;
static float theta_home = 0.0;
static float fs_s, alpha = 0.0;

void hardw_senc_init(int fs, int fc)
{
	// LAUNCH
	int reset = 0;															// reset counter
	*(FPGA_addr + ((FPGA_BDN) << 12) + 0xA04) = 0x00000000;					// set clear register
	for (reset = 0; reset < 8; reset++)										// iterate over registers
	{
		*(FPGA_addr + ((FPGA_BDN) << 12) + 0xA00) = 0xC0000008;				// clear enc registers
	}		
	*(FPGA_addr + ((FPGA_BDN) << 12) + 0xA00) = 0xC0000F00;					// read bit for init
	
	// PAR
	fs_s = (float)fs;
	alpha = expsp(-PI(2)*(float)fc / fs_s);

	// READ 1e
	*(FPGA_addr + ((FPGA_BDN) << 12) + 0xA00) = 0xC0000F00;
	theta_nano_temp = (*(FPGA_addr + ((FPGA_BDN) << 12) + 0x804) & 0xFFFFF);
	theta_db_temp = (double)(theta_nano_temp) / ENC_RES * PI(2);
}


void hardw_senc_read(int *theta_s_nano, float *theta_s, int *omega_s_nano, float *omega_s)
{
	int i = 0, diff;
	double theta_db;
	float omega_temp;

	// NANO
	*(FPGA_addr + ((FPGA_BDN) << 12) + 0xA00) = 0xC0000F00;
	*theta_s_nano = (*(FPGA_addr + ((FPGA_BDN) << 12) + 0x804) & 0xFFFFF);
	diff = *theta_s_nano - theta_nano_temp;
	if (diff >  ENC_RES/2.0) { nrofr--; i--; }
	if (diff < -ENC_RES/2.0) { nrofr++; i++; }
	theta_nano_temp = *theta_s_nano;

	// MECH
	omega_temp = *omega_s;
	theta_db = (double)(*theta_s_nano) / ENC_RES * PI(2);
	*theta_s_nano = *theta_s_nano + nrofr * ENC_RES;
	*theta_s = (float)theta_db + nrofr * PI(2) - theta_home;
	*omega_s_nano = (diff + i*ENC_RES) * fs_s;
	*omega_s = (float)((theta_db - theta_db_temp + i*PI(2)) * fs_s);
	*omega_s = *omega_s * (1.0 - alpha) + omega_temp * alpha;				// resursive iir maf
	theta_db_temp = theta_db;
}


void hardw_senc_home(void)
{
	nrofr = 0;
	theta_home = (float)theta_db_temp;
}


void hardw_senc_status(int *status)
{
	*status = ((*(FPGA_addr + ((FPGA_BDN) << 12) + 0x808) >> 20) & 0xFF);	// read error status
}

