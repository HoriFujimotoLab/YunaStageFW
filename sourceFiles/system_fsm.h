/************************************************************************************
SYSTEM FINIT-STATE-MACHINE MODULE
---------------------------------
Descr.:		finit-state-machine (fsm) for system protection
Boards:		MWPE-Expert3, MWPE-DIO (MWPE-PEV dio connect)
Sensor:		Digital input from inverter & emergency switches
Author:		Thomas Beauduin, University of Tokyo, 2015
*************************************************************************************/
#ifndef	SYSTEM_FSM_H
#define	SYSTEM_FSM_H

extern enum mode {
	SYS_STP = 0x01,				// LED (0001)
	SYS_INI = 0x02,				// LED (0010)
	SYS_RUN = 0x04,				// LED (0100)
	SYS_ERR = 0x0f				// LED (1111)
} sysmode_e;
extern int msr, cnt;			// measurement counters
extern float v_home;
extern int time;

#include	"system_math.h"
#include	"data/system_data.h"

/*	HARDWARE MODULES INCLUSION
**	--------------------------
**	DES:	include the required hardware modules headers
**			necessary for the particular setup configuration
*/
#include	"hardw_pev.h"
#include	"hardw_adc.h"
#include	"hardw_lin.h"
#include	"hardw_menc.h"
#include	"hardw_senc.h"
#include	<mwio3.h>


/*	CONTROL MODULES INCLUSION
**	-------------------------
**	DES:	include the required control modules headers
**			necessary for the particular experiment configuration
*/
#include	"ctrl_friction.h"
#include	"ctrl_current.h"
#include	"ctrl_motion.h"
#include	"ctrl_traject.h"


/*	INIT SYSTEM PROTECTION
**	----------------------
**	DES:	initiate dio board & protection flags
**			necessary at program start before interrupt init
*/
void system_fsm_init(void);


/*	PROCES SYSTEM MODE
**	------------------
**	DES:	finit-state-machine mode processing 
**			defines the system state and sets mode flag
*/
void system_fsm_mode(void);

#endif
