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
interrupt void system_tint0(void);
interrupt void system_cint5(void);

int test = 0;

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
	hardw_adc_read(0, &vdc_ad, &idc_ad, &iu_ad, &iw_ad);

	// CURRENT CONTROL
	if (sysmode_e == SYS_INI || sysmode_e == SYS_RUN)
	{
		ctrl_current_uw2ab(iu_ad, iw_ad, &ia_ad, &ib_ad);
		ctrl_current_ab2dq(ia_ad, ib_ad, theta_e, &id_ad, &iq_ad);
		ctrl_current_zcpi(iq_ref, id_ad, iq_ad, &vd_ref, &vq_ref);
		ctrl_current_dec(omega_m, id_ad, iq_ad, &vd_ref, &vq_ref);
		ctrl_current_dq2ab(vd_ref, vq_ref, theta_e, &va_ref, &vb_ref);
		ctrl_current_ab2uvw(va_ref, vb_ref, &vu_ref, &vv_ref, &vw_ref);
		hardw_inv_pwm(vu_ref, vv_ref, vw_ref, vdc_ad);
	}
	else {
		hardw_inv_pwm(0, 0, 0, vdc_ad);
	}
}


void system_tint0(void)
{
	unsigned int regs[2];

	// SENSOR READ
	hardw_adc_read(1, &load_m, &load_s, &acc_mx, &acc_tx);
	load = load_m - load_s;
	hardw_lin_read(&pos_t_nano, &pos_t, &vel_t_nano, &vel_t);
	hardw_menc_read(&theta_m_nano, &theta_m, &omega_m_nano, &omega_m, &theta_e);

	// MULTI-INT ON
	regs[0] = CSR;
	regs[1] = IRP;
	int_enable();

	// REFERENCE GENERATION
	if (sysmode_e == SYS_STP) {}
	if (sysmode_e == SYS_INI) { 
		ctrl_motion_hom(v_ref, omega_m, &iq_ref);
		theta_h = theta_m;
	}
	if (sysmode_e == SYS_RUN) {
		if (msr >= 0 && msr < nroft) {
			//ctrl_friction_stribeck(theta_m, theta_h, &theta_mo);
			//ctrl_friction_hyster(theta_m, theta_h);
			//ctrl_friction_lag(theta_m, theta_h, &theta_mo);
			//if (msr == 0)	 { v_ref = +v_home; }
			//if (pos_t < 0.0) { v_ref = -v_home; }
			//if (pos_t > 0.5) { v_ref = +v_home; }
			//ctrl_motion_vpi(v_ref, omega_m, &i_shp);
			//ctrl_motion_shp(i_shp, &iq_ref);
			//Perr = (theta_h + p_ref - theta_m);
			//v_ref = Ppos*Perr;
			//ctrl_traject_ref(reftype_e, 1, 1, &iq_ref);

			// PI-PI
			// ctrl_motion_ppi(p_ref, theta_m, theta_h, &v_ref);
			//ctrl_motion_vpi(v_ref, omega_m, &i_shp);
			//ctrl_motion_shp(i_shp, &iq_ref);
			
			
			// PID
			ctrl_friction_gms(&p_ref, &iq_fric);
			ctrl_motion_pid(p_ref, theta_m, theta_h, &iq_ref);
			ctrl_motion_dob(iq_ref, omega_m, &iq_dob);
			//iq_ref += iq_dob;
			//iq_ref += iq_fric;
			
			msr++;
		}
		else { 
			iq_ref = 0.0;
			v_ref = 0.0;
			ctrl_motion_reset(3);
			ctrl_traject_reset();
		}
	}
	
	// MULTI-INT OFF
	int_disable();
	CSR = regs[0];
	IRP = regs[1];

	watch_data_8ch();
	time++;
}


void system_init(void)
{
	// SENSORS
	watch_init();
	hardw_pev_init();
	hardw_adc_init();
	hardw_lin_init(FS, 500);
	hardw_menc_init(FS, 500);
	hardw_senc_init(FS, 500);

	// DRIVE CTRL
	int5_init_vector(system_cint5);
	hardw_inv_init();
	int5_enable_int();

	// MOTION CTRL
	timer0_init(TS);
	timer0_init_vector(system_tint0);
	timer0_start();
	timer0_enable_int();
	system_fsm_init();
}

