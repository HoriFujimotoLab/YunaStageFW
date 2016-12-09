/************************************************************************************
STAGE ROTARY-ENCODER MODULE
---------------------------
Descr.:		Hardware module for load-side encoder
Boards:		MWPE-Expert3, MWPE-FPGAA (custom board)
Sensor:		20bit absolute rotary encoder (nikon)
Author:		Thomas Beauduin, University of Tokyo, April 2015
*************************************************************************************/

#ifndef	STAGE_ENC_H
#define	STAGE_ENC_H

#include	"system_data.h"

/*	INIT ENCODER & BOARD
**	--------------------
**	DES:	initiate fpga board and encoder by register set/clear
**			necessary at program init before encoder reading
*/
void stage_enc_init(void);


/*	READ STAGE ENCODER DATA
**	-----------------------
**	DES:	returns the read and processed motor enc data from pios board
**	OUT:	theta_s:	theta mechanic for position control		[rad]	{-LR/2, LR/2}
**			omega_s:	omega mechanic for decoupling control	[rad/s] {-2pi*fs, 2pi*fs}
*/
void stage_enc_read(float *theta_s, float *omega_s, float *omega_sa);


/*	RESET STAGE ENCODER MODULE
**	--------------------------
**	DES:	resets encoder module internal counters and variables
**			necessary at measurement init to calibrate setup encoders
*/
void stage_enc_reset(void);


/*	CHECK ENCODER STATUS
**	--------------------
**	DES:	check run/error status of stage encoder
**	OUT:	encoder status for error handling	[-]	{RUN=0, ERROR=1}
*/
void stage_enc_status(int *status);

#endif
