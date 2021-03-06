/************************************************************************************
SYSTEM FINIT-STATE-MACHINE MODULE
---------------------------------
Descr.:		finit-state-machine (fsm) for system protection
Boards:		MWPE-Expert3, MWPE-DIO (MWPE-PEV dio connect)
Sensor:		Digital input from inverter & emergency switches
Author:		Thomas Beauduin, University of Tokyo, 2015
*************************************************************************************/
#include "system_fsm.h"

// SYSTEM CNT
#define		HWE_DI		0x0001			// hardware error			(DI0)
#define		THE_DI		0x0002			// thermal error			(DI1)
#define		ESW_DI		0x0010			// emergency switch			(DI4)
#define		INI_SW		0x0100			// system init switch		(DI8)
#define		RUN_SW		0x0200			// system run switch		(DI9)
#define		ERR_SW		0x0400			// error override switch	(DI10)

#define		LCF_SW		0x2000			// LC-filter switch			(DI13)
#define		OIL_SW		0x4000			// oil pump switch			(DI14)
#define		HOM_SW		0x8000			// stage homing switch		(DI15)

#define		OIL_DO		0x0001			// oil pump relay output	(DO0)
#define		RST_DO		0x0002			// inv reset relay output	(DO1)
#define		LCF_DO		0x0004			// LC filter output			(DO2)

#define		FWE_LED		0x0100			// 1. firmware error		(DO8)
#define		OVC_LED		0x0200			// 2. overcurrent error		(DO9)
#define		OVV_LED		0x0400			// 3. overvoltage error		(DO10)
#define		OVS_LED		0x0800			// 4. overspeed error		(DO11)
#define		HWE_LED		0x1000			// 5. inv hardware error	(DO12)
#define		SSE_LED		0x2000			// 6. setup sensor error	(DO13)
#define		ESW_LED		0x4000			// 7. emergency switch		(DO14)
#define		LSW_LED		0x8000			// 8. limit switch			(DO15)

// MODULE PAR
#define		PEV_BDN		0				// pev board number
#define		INV_CH		0				// pev inverter channel
#define		SWT_PRT		0.2				// switch protection time

// MODULE VAR
//global
enum mode sysmode_e = SYS_STP;
int msr = -1, cnt = 0;					// measurement counters
int time = 0;
float v_home = -18;
// local
int din, don, err = 0;
int firmerr = 0, senserr = 0;
int set = 0, calib = 0;
void system_fsm_err(void);
void system_fsm_reset(void);
void system_fsm_home(void);


void system_fsm_mode(void)
{
	if (time / FS >= SWT_PRT) {
		din = pev_pio_in(PEV_BDN); don = 0;	time = 0;							// DI read, DO reset
	}
	switch (sysmode_e)
	{
	case SYS_STP:
		if (INI_SW == (din & INI_SW)){											// init switch on
			pev_pio_out(PEV_BDN, don);											// reset relays				
			pev_inverter_start_pwm(PEV_BDN, INV_CH); 
			sysmode_e = SYS_INI;
		}
		if (INI_SW == (din & INI_SW) && RUN_SW == (din & RUN_SW)){				// avoid direct init
			err = err | FWE_LED; 
			sysmode_e = SYS_ERR; 
		}
		break;

	case SYS_INI:
		system_fsm_err();														// check for errors
		if (OIL_SW == (din & OIL_SW)) { don = don | OIL_DO; }					// oil pump on
		if (LCF_SW == (din & LCF_SW)) { don = don | LCF_DO; }					// lc filter on
		if (HOM_SW == (din & HOM_SW)) { system_fsm_home(); }
		if (HOM_SW != (din & HOM_SW)) { v_ref = 0.0; ctrl_motion_reset(1); }
		pev_pio_out(PEV_BDN, don);												// switch relays
		if (err != 0) {															// error detected	
			pev_inverter_stop_pwm(PEV_BDN, INV_CH); sysmode_e = SYS_ERR; }		// error mode change
		if (INI_SW != (din & INI_SW)) {											// ini switch off
			system_fsm_reset();													// reset firmware
			pev_inverter_stop_pwm(PEV_BDN, INV_CH); sysmode_e = SYS_STP; }		// revert to stp mode
		if (RUN_SW == (din & RUN_SW)) {				
			ctrl_motion_reset(1);
			sysmode_e = SYS_RUN; 
		}
		break;

	case SYS_RUN:
		system_fsm_err();														// check for errors
		//if (OIL_SW == (din & OIL_SW)) { don = don | OIL_DO; }					// oil pump on
		//if (LCF_SW == (din & LCF_SW)) { don = don | LCF_DO; }					// lc filter on
		pev_pio_out(PEV_BDN, don);												// switch relays
		if (err != 0) {															// error detected
			pev_inverter_stop_pwm(PEV_BDN, INV_CH); sysmode_e = SYS_ERR; }		// error mode change
		if (RUN_SW != (din & RUN_SW)) { sysmode_e = SYS_INI; }					// firmware reset
		break;

	case SYS_ERR:
		pev_pio_out(PEV_BDN, err);												// system error LED
		if (RUN_SW != (din & RUN_SW) && INI_SW != (din & INI_SW))				// hardware reset
		{
			system_fsm_reset();	err = 0; sysmode_e = SYS_STP;					// firmware reset
			don = don | RST_DO; pev_pio_out(PEV_BDN, don);						// reset relay on
		}
		break;
	}
	led_out(sysmode_e);															// system mode LED
}


void system_fsm_err(void)
{	
	hardw_pev_read(0, &vu_ex, &vw_ex, &iu_ex, &iw_ex);
	hardw_pev_read(1, &lsw_s, &lsw_m, &home_ad, &temp_m);
	if (firmerr != 0)				{ err = err | FWE_LED; }				// 1. firmware error
	if (fabsf(iu_ex) > OVC_LIM)		{ err = err | OVC_LED; }				// 2. overcurrent
	if (fabsf(iw_ex) > OVC_LIM)		{ err = err | OVC_LED; }				// 2.
	if (fabsf(idc_ad) > OVC_LIM)	{ err = err | OVC_LED; }				// 2.
	if (fabsf(vu_ex) > OVV_LIM)		{ err = err | OVV_LED; }				// 3. overvoltage
	if (fabsf(vw_ex) > OVV_LIM)		{ err = err | OVV_LED; }				// 3.
	if (fabsf(vdc_ad) > OVV_LIM)	{ err = err | OVV_LED; }				// 3.
	if (fabsf(omega_m) > OVS_LIM)	{ err = err | OVS_LED; }				// 4. motor overspeed
	if (sysmode_e == SYS_RUN)
	{
		if (HWE_DI == (din & HWE_DI))	{ err = err | HWE_LED; }			// 5. hardware error
		if (THE_DI == (din & THE_DI))	{ err = err | HWE_LED; }			// 5.
	}
	if (ESW_DI == (din & ESW_DI))	{ err = err | ESW_LED; }				// 7. emergency switch
	if (ERR_SW != (din & ERR_SW))											// error override
	{
		if (senserr != 0)				{ err = err | SSE_LED; }			// 6. sensor error
		if (lsw_s < 0.8 || lsw_m < 0.8)	{ err = err | LSW_LED; }			// 8. limit switch
	}
}


void system_fsm_reset(void)
{
	ctrl_current_reset();
	ctrl_motion_reset(100);
	Aref = 0.0;
	calib = 0;
}


void system_fsm_init(void)
{
	pev_inverter_stop_pwm(PEV_BDN, INV_CH); 
	sysmode_e = SYS_STP;
}


void system_fsm_home(void)
{
	if (home_ad < 0.5 && calib == 0) { v_ref = +v_home; }
	if (home_ad > 0.5 && calib == 0) { v_ref = -v_home;
		hardw_lin_home(); 
		hardw_menc_home(); 
		hardw_senc_home();
		calib = 1; 
	}
	if (pos_t < 0.0 && calib == 1)	 { v_ref = -v_home; }
	if (pos_t > 0.5 && calib == 1)   { v_ref = +v_home; }
}


/* NOTES:
** ------
*/

