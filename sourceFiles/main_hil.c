/************************************************************************************
MULTI INTERRUPT FIRMWARE
-------------------------
Descr.:		main file for multiple interrupt experiments
Boards:		PE-Expert3, C6713A DSP + System Hardware
System:		Main-file of modular firmware structure
Author:		Thomas Beauduin, University of Tokyo, 2016
*************************************************************************************
INITIALIZATION AND EXPERIMENT
-------------------------
1. switch_e = 1 (HOM_SW)
homing active
2. SLOWLY and GENTLY move the stage by hand to the home sensor in the left side
and move back to the center position
3. switch_e = 2 (INI_SW)
Initialize and activate current controller
4. switch_e = 3 (RUN_SW)
Activate motion
A. current controller redesign
flag_CRD = 1 during SYS_STP
*************************************************************************************
MULTI INTERRUPT CHECKLIST FOR SYSTEM IDENTIFICATION
-------------------------
1. main_exp.c
interrupt void system_tint0(void);
void system_cint5(void)
// REFERENCE GENERATION
watch_data_8ch();
void system_tint0(void)
// REFERENCE GENERATION
watch_data_8ch();
void system_init(void)
// MOTION CTRL
2. system_data.h  nroft
3. sensor read location
4. change WAVE settings refer system_data.h
*************************************************************************************/
#include	"system_fsm.h"

void system_init(void);
//interrupt void system_tint0(void);
interrupt void system_cint5(void);

void main(void)
{
	// SYSTEM INIT
	int_disable();
	system_init();
	int_enable();

	//SYSTEM RUN
	while (1) { system_fsm_mode(); }
}

void system_cint5(void)
{
	// SENSOR READ
	hardw_adc_read(BDN0, 0, &vu0_ad, &iu0_ad, &iw0_ad, &vdc0_ad);
	hardw_lin_read(BDN0, &pos_m_nano, &pos_m, &vel_m_nano, &vel_m);
	hardw_lin_read(BDN1, &pos_l_nano, &pos_l, &vel_l_nano, &vel_l);
	hardw_lin_imagi(pos_m_nano, pos_l_nano, Lm_i, &pos_i_nano);
	hardw_lin_imagi(vel_m_nano, vel_l_nano, Lm_i, &vel_i_nano);
	hardw_lin_imagf(pos_m, pos_l, Lm_i, &pos_i);
	hardw_lin_imagf(vel_m, vel_l, Lm_i, &vel_i);
	hardw_lin_elec(pos_m, &theta_e);

	// REFERENCE GENERATION
	if (sysmode_e == SYS_RUN)
	{
		if (msr >= 0 && msr < nroft) {
			ctrl_traject_ref(reftype_e, pos_i, Aref, Fref, &iq0_ref);
			msr++;
		}
		else {
			iq0_ref = 0.0;
		}
	}

	// CURRENT CONTROL - MAIN
	if (sysmode_e == SYS_INI || sysmode_e == SYS_RUN)
	{
		ctrl_current_uw2ab(iu0_ad, iw0_ad, &ia0_ad, &ib0_ad);
		ctrl_current_ab2dq(ia0_ad, ib0_ad, theta_e, &id0_ad, &iq0_ad);
		ctrl_current_zcpi(Affi, Bffi, Cffi, Dffi, Afbi, Bfbi, Cfbi, Dfbi, iq0_ref, id0_ad, iq0_ad, &vd0_ref, &vq0_ref);
		ctrl_current_dec(vel_ma, id0_ad, iq0_ad, &vd0_ref, &vq0_ref);
		ctrl_current_dq2ab(vd0_ref, vq0_ref, theta_e, &va0_ref, &vb0_ref);
		ctrl_current_ab2uvw(va0_ref, vb0_ref, &vu0_ref, &vv0_ref, &vw0_ref);
		hardw_inv_pwm(0, vu0_ref, vv0_ref, vw0_ref, vdc0_ad);
	}
	else {
		hardw_inv_pwm(0, 0, 0, 0, vdc0_ad);
	}
	watch_data_8ch();
}

/*
void system_tint0(void)
{
	unsigned int regs[2];

	// SENSOR READ
//	hardw_lin_read(BDN0, &pos_m_nano, &pos_m, &vel_m, &vel_ma);
//	hardw_lin_read(BDN1, &pos_l_nano, &pos_l, &vel_l, &vel_la);
//	hardw_lin_imag(pos_m, pos_l, Lm_i, &pos_i);
//	hardw_lin_imag(vel_m, vel_l, Lm_i, &vel_i);
//	hardw_lin_elec(pos_m, &theta_e);

	// MULTI-INT ON
	regs[0] = CSR;
	regs[1] = IRP;
	int_enable();


	// MULTI-INT OFF
	int_disable();
	CSR = regs[0];
	IRP = regs[1];

	//watch_data_8ch();
}
*/

void system_init(void)
{
	// SENSORS
	watch_init();
	hardw_lin_init();
	hardw_pev_init();
	hardw_adc_init(BDN0);
	hardw_dac_init(BDN0);

	// DRIVE CTRL
	int5_init_vector(system_cint5);
	hardw_inv_init();
	int5_enable_int();

	/*
	// MOTION CTRL
	timer0_init(TS);
	timer0_init_vector(system_tint0);
	timer0_start();
	timer0_enable_int();
	*/

	system_fsm_init();
}




