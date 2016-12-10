/************************************************************************************
STAGE ROTARY-ENCODER MODULE
---------------------------
Descr.:		Hardware module for stage encoder
Boards:		MWPE-Expert3, MWPE-FPGAA (custom board)
Sensor:		20bit absolute rotary encoder (nikon)
Author:		Thomas Beauduin, University of Tokyo, December 2016
*************************************************************************************/
#include	"hardw_senc.h"
#include	"system_data.h"
#include	<mwio3.h>

// MODULE PAR
#define	FPGA_BDN	0						// FPGAA board number
#define	ENC_RES		(1048576.0)				// 20bit resolution [cnt/turn]
#define ENC_DIR		(-1.0)					// direction rel to motor uvw
#define ENC_OFF		(2.125)					// setup encoder-rotor offset
#define ALPHA		(0.50)					// recusive iir maf factor

// MODULE VAR
// global
float theta_s, theta_e, omega_s;
int theta_s_nano, omega_s_nano;
// local
static int nrofr = 0;						// number of turns [-]


void hardw_senc_init(void)
{
	int reset = 0;															// reset counter
	*(FPGA_addr + ((FPGA_BDN) << 12) + 0xA04) = 0x00000000;					// set clear register
	for (reset = 0; reset < 8; reset++)										// iterate over registers
	{
		*(FPGA_addr + ((FPGA_BDN) << 12) + 0xA00) = 0xC0000008;				// clear enc registers
	}		
	*(FPGA_addr + ((FPGA_BDN) << 12) + 0xA00) = 0xC0000F00;					// read bit for init
}


void hardw_senc_read(float *theta_s, float *omega_s)
{
	// LOCAL VAR
	int data_ms = 0;														// measured data  [cnt]
	double theta_ms = 0.0;													// measured theta [rad]
	double diff = 0.0;														// position diff  [rad]
	int i = 0;																// rev jump index [-]

	// READ DATA
	*(FPGA_addr + ((FPGA_BDN) << 12) + 0xA00) = 0xC0000F00;					// send read bit
	data_ms = (*(FPGA_addr + ((FPGA_BDN) << 12) + 0x804) & 0xFFFFF);		// read register
	theta_ms = ((double)data_ms / ENC_RES * PI(2) * ENC_DIR) - ENC_OFF;		// [cnt] to [rad]

	// ANGLE
	diff = theta_ms - (*theta_s - nrofr * PI(2));							// theta time difference
	if (diff >  PI(1)){ nrofr--; i--; }										// revolution calc
	if (diff < -PI(1)){ nrofr++; i++; }										// note: max vel pi*fs
	*theta_s = theta_ms + nrofr * PI(2);									// full screw pos [rad]
	*omega_s = (diff + i*PI(2)) * FS;										// full screw vel [rad/s]
	//*omega_sa = ALPHA * *omega_s + (1 - ALPHA) * *omega_sa;				// resursive maf  [rad/s]
}


void hardw_senc_reset(void)
{
	nrofr = 0;																// set start position
}


void hardw_senc_status(int *status)
{
	*status = ((*(FPGA_addr + ((FPGA_BDN) << 12) + 0x808) >> 20) & 0xFF);	// read error status
}

