/************************************************************************************
SETUP SENSOR DAC MODULE
-----------------------
Descr.:		Hardware module dac of stage sensors
Boards:		MWPE3-C6713A, MWPE3-ADC (14 bit)
Sensor:		inverter I/V sensors (myway), torque sensor (unipulse)
Author:		Thomas Beauduin, University of Tokyo, March 2016
*************************************************************************************/
#ifndef	HARDW_DAC_H
#define	HARDW_DAC_H


/*	INIT CONVERSION & BOARD
**	-----------------------
**	DES:	initiate dac board & set range for all channels
**			necessary at program init before output
*/
void hardw_dac_init(int bdn_ad);




#endif
