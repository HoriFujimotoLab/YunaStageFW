/************************************************************************************
LINEAR-SCALE MODULE
-------------------
Descr.:		Hardware module for linear-scale of table
Boards:		MWPE-Expert3, MWPE-FPGAA (custom board: 08934-C2-xxx)
Sensor:		1nm incremental linear scale (magnescale)
Author:		Thomas Beauduin, University of Tokyo, December 2016
*************************************************************************************/
#include	"hardw_lin.h"
#include	"system_math.h"
#include	<mwio3.h>

// FPGA ADDR
#define	INI_ADDR	((volatile unsigned int*)0xA002400C)		// init address     (RW bit0)
#define	ERR_ADDR	((volatile unsigned int*)0xA0024000)		// error address    (R bit6..0)
#define	RST_ADDR	((volatile unsigned int*)0xA0024008)		// reset address    (RW bit0)
#define	CLR_ADDR	((volatile unsigned int*)0xA0024020)		// clear address    (W bit6..2)
#define	CON_ADDR	((volatile unsigned int*)0xA0024024)		// convert address  (W only 1 write)
#define	DAT_ADDR	((volatile unsigned int*)0xA002402C)		// data address		(R 32bit)
#define	STA_ADDR	((volatile unsigned int*)0xA0024024)		// status address	(R bit6..0:busy, bit7:err)

// MODULE VAR
// Global:
int pos_t_nano = 0, vel_t_nano = 0, error_in = 0;
float pos_t = 0, vel_t = 0;
// Local:
int read_err = 0, hard_err = 0;
float fs_t, alpha = 0.0;

void hardw_lin_init(int fs, int fc)
{
	*RST_ADDR = 0; *RST_ADDR = 1;								// reset registers
	*RST_ADDR = 1; *RST_ADDR = 0;								// reset hardware  
	*INI_ADDR = 0; *CLR_ADDR = 0x04;							// counter register init
	fs_t = (float)fs;
	alpha = expsp(-PI(2)*(float)fc / fs_t);
}


void hardw_lin_read(int *pos_nano, float *pos,  int *vel_nano, float *vel)
{
	int i = 0;
	int pos_nano_temp;
	float vel_temp;

	// READ DATA
	pos_nano_temp = *pos_nano;
	*CON_ADDR = 1;												// conversion start
	while (((*STA_ADDR & 0x02) == 1) && (i <= 5)) { i++; }		// wait for rdy status
	if (i >= 5) { read_err = 1; }								// over-time error
	*pos_nano = *DAT_ADDR;										// read data register

	// POS & VEL
	vel_temp = *vel;
	*vel_nano = (*pos_nano - pos_nano_temp) * fs_t;
	*pos = (float)*pos_nano * 1.0e-9;							// table position [m]
	*vel = (float)*vel_nano * 1.0e-9;							// table velocity [m/s]
	*vel = *vel * (1.0 - alpha) + vel_temp * alpha;				// resursive maf  [m/s]
}


void hardw_lin_status(int *status)
{
	hard_err = *ERR_ADDR;										// hardware error
	if (read_err == 1 || hard_err == 1) { *status = 1; }		// fpga read error
	else								{ *status = 0; }
}


void hardw_lin_home(void)
{
	*CLR_ADDR = 0x04;											// set data_addr to 0
}

