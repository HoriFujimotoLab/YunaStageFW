/************************************************************************************
MOTOR ROTARY-ENCODER MODULE
---------------------------
Descr.:		Hardware module for motor encoder (00672)
Boards:		MWPE-Expert3, MWPE-PIOS (PIO expansion board)
Sensor:		20bit absolute rotary encoder (Tamagawa)
Author:		Thomas Beauduin, University of Tokyo, April 2015
*************************************************************************************/

#ifndef	MOTOR_ENC_H
#define	MOTOR_ENC_H

#include "system_data.h"

/*	INIT ENCODER & BOARD
**	--------------------
**	DES:	initiate pios board and encoder by register set/clear
**			necessary at program init before encoder reading
*/
void motor_enc_init();


/*	READ MOTOR ENCODER DATA
**	-----------------------
**	DES:	returns the read and processed motor enc data from pios board
**	OUT:	theta_e:	theta electric for park transformation	[rad]	{0,2pi}
**			theta_m:	theta mechanic for position control		[rad]	{-LR/2, LR/2}
**			omega_m:	omega mechanic for decoupling control	[rad/s] {-2pi*fs, 2pi*fs}
**			omega_ma:	omega averaged (time) for vel control	[rad/s]	{-2pi*fs, 2pi*fs}
**	DSP:	calctime:	avgerage calclation time				8 [ms]  (vers. 29/11/15)
*/
void motor_enc_elec(float *theta_e);
void motor_enc_read(float *theta_m, float *omega_m, float *omega_ma);


/*	RESET ENCODER MODULE
**	--------------------
**	DES:	resets encoder module internal counters and variables
**			necessary at measurement init to calibrate setup encoders
*/
void motor_enc_reset(float *theta_m);


/*	CHECK ENCODER STATUS
**	--------------------
**	DES:	check run/error status of motor encoder
**	OUT:	encoder status for error handling	[-]	{RUN=1, ERROR=0}
*/
void motor_enc_status(unsigned int *status);


#endif
