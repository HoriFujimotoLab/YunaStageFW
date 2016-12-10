/************************************************************************************
MULTI INTERRUPT FIRMWARE
-------------------------
Descr.:		main file for multiple interrupt experiments
Boards:		PE-Expert3, C6713A DSP + System Hardware
System:		Main-file of modular firmware structure
Author:		Thomas Beauduin, University of Tokyo, December 2016
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
	time = time + TC;

	// SENSOR READ
	hardw_adc_read(0, &iu_ad, &iw_ad, &vdc_ad, &idc_ad);
	hardw_adc_read(1, &load_m, &load_s, &acc_tx, &acc_tz);
	hardw_lin_read(&pos_t_nano, &pos_t, &vel_t_nano, &vel_t);
	hardw_menc_read(&theta_m, &omega_m);
	hardw_menc_read(&theta_s, &omega_s);

	// MOTOR OUTPUT
	hardw_inv_pwm(0, 0, 0, vdc_ad);

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
	hardw_adc_init();
	hardw_menc_init();
	hardw_senc_init();

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




