/************************************************************************************
POWER-ELEC MODULE
-----------------
Descr.:		hardware module for motor drive
Boards:		MWPE-Expert3, MWPE-PEV (PEV expansion board)
PWelec:		MWINV-9R122B, Yuna ball-screw setup
Author:		Thomas Beauduin, University of Tokyo, December 2016
*************************************************************************************/
#ifndef	HARDW_PEV_H
#define	HARDW_PEV_H

// MODULE VAR
extern float lsw_m, lsw_s, home_ad, temp_m;
extern float vu_ex, vw_ex, iu_ex, iw_ex;

/*	INIT PEV BOARD ADC
**	------------------
**	DES:	initiate pev board and adc settings
**			sensor ranges set and offset calculated
*/
void hardw_pev_init(void);


/*	INIT INVERTER
**	-------------
**	DES:	initiate inverter settings and gate signal
**			modulation type set & pwm function started
*/
void hardw_inv_init(void);


/*	READ MOTOR ADC DATA
**	-------------------
**	DES:	returns the read and processed motor adc data from pev board
**	INP:	grp_ad: ad convertion group number [-] {0 or 1}
**	OUT:	ad0:	grp0: vu [V] {0,400}, grp1: lsw1 [%] {0,1}
**			ad1:	grp0: vw [V] {0,400}, grp1: lsw2 [%] {0,1}
**			ad2:	grp0: iu [A] {0, 50}, grp1: home [%] {0,1}
**			ad3:	grp0: iw [A] {0, 50}, grp1: temp [%] {0,1}
*/
void hardw_pev_read(int grp_ad, float *ad0, float *ad1, float *ad2, float *ad3);


/*	SET INVERTER OUTPUT
**	-------------------
**	DES:	modulates reference signal and creates gate signal
**			modulation type: S-PWM for speed & SVM for power
**	INP:	vx_ref:		phase reference voltage	[V]	{-200, 200}
**			vdc_ad:		measured dc-bus voltage	[V] {   0, 400}
*/
void hardw_inv_pwm(float vu_ref, float vv_ref, float vw_ref, float vdc_ad);


#endif
