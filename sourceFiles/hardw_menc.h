/************************************************************************************
MOTOR ROTARY-ENCODER MODULE
---------------------------
Descr.:		Hardware module for motor encoder (00672)
Boards:		MWPE-Expert3, MWPE-PIOS (PIO expansion board)
Sensor:		20bit absolute rotary encoder (Tamagawa)
Author:		Thomas Beauduin, University of Tokyo, December 2016
*************************************************************************************/
#ifndef	HARDW_MENC_H
#define	HARDW_MENC_H

// MODULE VAR
extern float theta_m, theta_e, omega_m;
extern int theta_m_nano, omega_m_nano;


/*	INIT ENCODER & BOARD
**	--------------------
**	DES:	initiate pios board and encoder by register set/clear
**			necessary at program init before encoder reading
*/
void hardw_menc_init();


/*	READ MOTOR ENCODER DATA
**	-----------------------
**	DES:	returns the read and processed motor enc data from pios board
**	OUT:	theta_e:	theta electric for park transformation	[rad]	{0,2pi}
**			theta_m:	theta mechanic for position control		[rad]	{-LR/2, LR/2}
**			omega_m:	omega mechanic for decoupling control	[rad/s] {-2pi*fs, 2pi*fs}
**			omega_ma:	omega averaged (time) for vel control	[rad/s]	{-2pi*fs, 2pi*fs}
**	DSP:	calctime:	avgerage calclation time				8 [ms]  (vers. 29/11/15)
*/
void hardw_menc_elec(float *theta_e);
void hardw_menc_read(float *theta_m, float *omega_m);


/*	RESET ENCODER MODULE
**	--------------------
**	DES:	resets encoder module internal counters and variables
**			necessary at measurement init to calibrate setup encoders
*/
void hardw_menc_reset(float *theta_m);


/*	CHECK ENCODER STATUS
**	--------------------
**	DES:	check run/error status of motor encoder
**	OUT:	encoder status for error handling	[-]	{RUN=1, ERROR=0}
*/
void hardw_menc_status(unsigned int *status);


#endif
