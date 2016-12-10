/************************************************************************************
SENSOR ADC MODULE
-----------------
Descr.:		Hardware module adc of stage sensors
Boards:		MWPE3-C6713A, MWPE3-ADC (14 bit)
Sensor:		hioki,kyowa,keyence: Yuna ball-screw setup
Author:		Thomas Beauduin, University of Tokyo, December 2016
*************************************************************************************/
#ifndef	HARDW_ADC_H
#define	HARDW_ADC_H

//	GLOBAL VAR
// inverter measured values
extern float vu_ad, vw_ad, iu_ad, iw_ad;
extern float va_ad, vb_ad, ia_ad, ib_ad;
extern float vd_ad, vq_ad, id_ad, iq_ad;
extern float vdc_ad, idc_ad;
// sensor level
extern float disp_s1, disp_s2, disp_m1, disp_m2;
extern float load_m, load_s, servo_ad, temp_s;
extern float acc_mx, acc_tx, acc_tz, acc_sx;


/*	INIT CONVERSION & BOARD
**	-----------------------
**	DES:	initiate adc board & calculate offsets
**			necessary at program init before reading
*/
void hardw_adc_init(void);


/*	READ SENSOR ADC DATA
**	---------------------
**	DES:	returns the processed adc sensor data
**	OUT:	ad0: grp0: ec1 [mm] {0,1}, grp1: lc1 [N] {0,5k}, grp2: lin [%] {0,1}
**			ad1: grp0: ec2 [mm] {0,1}, grp1: lc2 [N] {0,5k}, grp2: tp1 [V] {0,5}
**			ad2: grp0: ec3 [mm] {0,1}, grp1: ex1 [A] {0,25}, grp2: tp2 [V] {0,5}
**			ad3: grp0: ec4 [mm] {0,1}, grp1: ex2 [A] {0,25}, grp2: tp3 [V] {0,5}
*/
void hardw_adc_read(int grp_ad, float *ad0, float *ad1, float *ad2, float *ad3);


#endif

/* NOTES:
** ecs1: screw-side eddy-current #16	(bnc1)
** ecs2: screw_side eddy-current #17	(bnc2)
** ecs3: motor_side eddy-current #18	(bnc3)
** ecs4: motor_side eddy-current #19	(bnc4)
** lcs1: motor-side load-cell			(bnc5)
** lcs2: screw-side load_cell			(bnc6)
*/

