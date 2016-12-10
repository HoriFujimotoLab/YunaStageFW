/************************************************************************************
LINEAR-SCALE MODULE
-------------------
Descr.:		Hardware module for linear-scale of table
Boards:		MWPE-Expert3, MWPE-FPGAA (custom board)
Sensor:		1nm incremental linear scale (magnescale)
Author:		Thomas Beauduin, University of Tokyo, December 2016
*************************************************************************************/

#ifndef	HARDW_LIN_H
#define	HARDW_LIN_H

/*	GLOBAL VARIABLES
**	----------------
**	DES:	table-side linear scale
*/
extern int pos_t_nano, vel_t_nano, error_in;
extern float pos_t, vel_t;


/*	INIT ENCODER & BOARD
**	--------------------
**	DES:	initiate fpga board and encoder by register set/clear
**			necessary at program init before encoder reading
*/
void hardw_lin_init(void);


/*	READ LINEAR-SCALE DATA
**	-----------------------
**	DES:	returns the processed incremental data from linear scale
**			lower 32 bit data in [nm]
**			32bit int: -2^31 < int < 2^31
**			2^31 = 2.147483647*1e9 [nm]
**	OUT:	pos_t:	linear table position	[mm]
**			vel_t:	linear table velocity	[mm/s]
**			vel_ta:	averaged velocity (lpf)	[mm/s]
*/
void hardw_lin_read(float *pos_t, float *vel_t, float *vel_ta);


/*	READ LINEAR-SCALE STATUS
**	------------------------
**	DES:	returns the status of the sensor data acquisition
**	OUT:	status:	sensor error status [-] {ERR = 1}
*/
void hardw_lin_status(int *status);


/*	READ LINEAR-SCALE STATUS
**	------------------------
**	DES:	returns the data count registers to 0
**			necessary at stage home referencing 
*/
void hardw_lin_reset(void);


#endif

/*	NOTES:
**	home:	sticker close to motor (analog msr)
**			home position resolution: 0.4 um
**			max. home sensing speed: 150mm/s
*/
