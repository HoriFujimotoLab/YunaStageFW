/************************************************************************************
STAGE LINEAR-SCALE MODULE
-------------------------
Descr.:		Hardware module for linear-scale of table
Boards:		MWPE-Expert3, MWPE-FPGAA (custom board: 08934-C2-xxx)
Sensor:		1nm incremental linear scale (magnescale)
Author:		Thomas Beauduin, University of Tokyo, April 2015
*************************************************************************************/

#include	"stage_lin.h"
#include	<mwio3.h>

// MODULE PAR
#define ALPHA		(0.10)										// recusive iir maf factor
#define LIN_DIR		(1.0)										// linear scale direction

// FPGA ADDR
#define	INI_ADDR	((volatile unsigned int*)0xA002400C)		// init address     (RW bit0)
#define	ERR_ADDR	((volatile unsigned int*)0xA0024000)		// error address    (R bit6..0)
#define	RST_ADDR	((volatile unsigned int*)0xA0024008)		// reset address    (RW bit0)
#define	CLR_ADDR	((volatile unsigned int*)0xA0024020)		// clear address    (W bit6..2)
#define	CON_ADDR	((volatile unsigned int*)0xA0024024)		// convert address  (W only 1 write)
#define	DAT_ADDR	((volatile unsigned int*)0xA002402C)		// data address		(R 32bit)
#define	STA_ADDR	((volatile unsigned int*)0xA0024024)		// status address	(R bit6..0:busy, bit7:err)

// MODULE VAR
int read_err = 0, hard_err = 0;


void stage_lin_init(void)
{
	*RST_ADDR = 0; *RST_ADDR = 1;								// reset registers
	*RST_ADDR = 1; *RST_ADDR = 0;								// reset hardware  
	*INI_ADDR = 0; *CLR_ADDR = 0x04;							// counter register init
}


void stage_lin_read(int *pos_nano, float *pos,  int vel_nano, float *vel)
{
	// LOCAL VAR
	int data_cnt = 0;											// msr counter [cnt]
	int i = 0;													// loop index  [-]
	float temp = 0;												// temp data buffer

	// READ DATA
	*CON_ADDR = 1;												// conversion start
	while (((*STA_ADDR & 0x02) == 1) && (i <= 5)) { i++; }		// wait for rdy status
	if (i >= 5) { read_err = 1; }								// over-time error
	data_cnt = *DAT_ADDR;										// read data register

	// POS & VEL
	temp = *pos_t;												// previous msr   [mm]
	*pos_t = LIN_DIR * data_cnt * 1.0e-6;						// table position [mm]
	*vel_t = (*pos_t - temp) * FS;								// table velocity [mm/s]
	*vel_ta = ALPHA * *vel_t + (1 - ALPHA) * *vel_ta;			// resursive maf  [mm/s]

	// NANO

}


void stage_lin_status(int *status)
{
	hard_err = *ERR_ADDR;										// hardware error
	if (read_err == 1 || hard_err == 1) { *status = 1; }		// fpga read error
	else								{ *status = 0; }
}


void stage_lin_reset(void)
{
	*CLR_ADDR = 0x04;											// set data_addr to 0
}
