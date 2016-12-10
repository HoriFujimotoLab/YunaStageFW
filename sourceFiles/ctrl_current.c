/************************************************************************************
CURRENT CONTROL MODULE
----------------------
Descr.:		Control module for drive current control
Boards:		PE-Expert3, C6713A DSP (float calculation)
Author:		Thomas Beauduin, Wataru Ohnishi
			Hori-Fujimoto lab, University of Tokyo, 2016
*************************************************************************************/
#include	"ctrl_current.h"
#include	"ctrl_math.h"
#include	"system_data.h"
#include	"data/ctrl_current_par.h"

// MODULE VAR
//Local: Dead-Time Compensation
float	ihys = 5.0000000000e-02;
float	vhys = 2.8800000000e-02;
// Local: State-Space Control state vectors
double xff_q = 0.0, xfb_d = 0.0, xfb_q = 0.0;
float x_id = 0.0, x_iq = 0.0;

// Global: inverter control values
float vd_ref = 0.0, vq_ref = 0.0, id_ref = 0.0, iq_ref = 0.0;
float va_ref = 0.0, vb_ref = 0.0, vu_ref = 0.0, vv_ref = 0.0, vw_ref = 0.0;
float iq_ref_ff = 0.0, iq_ref_fb = 0.0, iq_pid = 0.0;
// Global: Pole-Zero Cancel PI
float	Ai_pzcpi = 0.0, Bi_pzcpi = 0.0;
float	Ci_pzcpi = 0.0, Di_pzcpi = 0.0;
float	Fbw_pzcpi = 500;
// Global: Zero-Cancel PI
float	Affi = 0.0, Bffi = 0.0, Cffi = 0.0, Dffi = 0.0;
float	Afbi = 0.0, Bfbi = 0.0, Cfbi = 0.0, Dfbi = 0.0;
float	Fbw_zcpi = 1000; // [Hz]
float	zeta_zcpi = 0.848528137423857; // 1.2/sqrt(2)


void ctrl_current_pzcpi(float Ai_pzcpi, float Bi_pzcpi, float Ci_pzcpi, float Di_pzcpi, float iq_ref, float id_ad, float iq_ad, float *vd_ref, float *vq_ref)
{
	float e_id, e_iq;

	e_id = 0 - id_ad;
	e_iq = iq_ref - iq_ad;
	*vd_ref = Ci_pzcpi * x_id + Di_pzcpi * e_id;
	*vq_ref = Ci_pzcpi * x_iq + Di_pzcpi * e_iq;
	x_id = Ai_pzcpi * x_id + Bi_pzcpi * e_id;
	x_iq = Ai_pzcpi * x_iq + Bi_pzcpi * e_iq;
}


void ctrl_current_pzcpi_init(float Fbw, float *Ai_pzcpi, float *Bi_pzcpi, float *Ci_pzcpi, float *Di_pzcpi)
{
	float Ti = (1 / (PI(2) * Fbw));

	*Ai_pzcpi = 1.0;
	*Bi_pzcpi = 1.0;
	*Ci_pzcpi = Rs / (Ti * FC);
	*Di_pzcpi = (2 * Ls * FC + Rs) / (2 * Ti * FC);
}


void ctrl_current_zcpi(float Affi, float Bffi, float Cffi, float Dffi, float Afbi, float Bfbi, float Cfbi, float Dfbi, float iq_ref, float id_ad, float iq_ad, float *vd_ref, float *vq_ref)
{
	double iq_ff, id_er, iq_er;
	
	iq_ff = Cffi * xff_q + Dffi * (double)iq_ref;						// input shaping
	xff_q = Affi * xff_q + Bffi * (double)iq_ref;

	id_er = 0.0 - (double)id_ad;										// dq-axis feedback
	iq_er = iq_ff - (double)iq_ad;
	*vd_ref = (float)(Cfbi * xfb_d + Dfbi * id_er);
	xfb_d = Afbi * xfb_d + Bfbi * id_er;
	*vq_ref = (float)(Cfbi * xfb_q + Dfbi * iq_er);
	xfb_q = Afbi * xfb_q + Bfbi * iq_er;
}


void ctrl_current_zcpi_init(float Fbw, float zeta, float *Affi, float *Bffi, float *Cffi, float *Dffi, float *Afbi, float *Bfbi, float *Cfbi, float *Dfbi)
{
	// I_currentCtrl_sym.m
	*Affi = -(1.0*((0.17765287921960845513902083799777*Fbw * Fbw) / FC - 0.11309733552923255658465516179806*Fbw*zeta + 10.5)) / (0.11309733552923255658465516179806*Fbw*zeta + (0.17765287921960845513902083799777*Fbw * Fbw) / FC - 10.5);
	*Bffi = 1.0;
	*Cffi = (0.17765287921960845513902083799777*Fbw * Fbw) / (FC*(0.11309733552923255658465516179806*Fbw*zeta + (0.17765287921960845513902083799777*Fbw * Fbw) / FC - 10.5)) - (0.17765287921960845513902083799777*Fbw * Fbw * ((0.17765287921960845513902083799777*Fbw * Fbw) / FC - 0.11309733552923255658465516179806*Fbw*zeta + 10.5)) / (FC*(0.11309733552923255658465516179806*Fbw*zeta + (0.17765287921960845513902083799777*Fbw * Fbw) / FC - 10.5) * (0.11309733552923255658465516179806*Fbw*zeta + (0.17765287921960845513902083799777*Fbw * Fbw) / FC - 10.5));
	*Dffi = (0.17765287921960845513902083799777*Fbw * Fbw) / (FC*(0.11309733552923255658465516179806*Fbw*zeta + (0.17765287921960845513902083799777*Fbw * Fbw) / FC - 10.5));

	*Afbi = 1.0;
	*Bfbi = 1.0;
	*Cfbi = (0.35530575843921691027804167599554*Fbw * Fbw) / FC;
	*Dfbi = 0.11309733552923255658465516179806*Fbw*zeta + (0.17765287921960845513902083799777*Fbw * Fbw) / FC - 10.5;
}


void ctrl_current_dec(float omega_m, float id_ad, float iq_ad, float *vd_ref, float *vq_ref)
{
	*vd_ref -= (omega_m * Ls * iq_ad);								// coupling compensation
	*vq_ref += (omega_m * Ke);										// back-emf compensation
}


void ctrl_current_dtc(float iq_ref, float theta_e, float vdc_ad, float *vu_ref, float *vv_ref, float *vw_ref)
{
	double dtc_err;														// dtc comp voltage
	float ia_ref, ib_ref, iu_ref, iv_ref, iw_ref;						// stationary frame ref

	dtc_err = vhys * vdc_ad;											// deadtime calc
	ctrl_current_dq2ab(0.0, iq_ref, theta_e, &ia_ref, &ib_ref);			// frame transform
	ctrl_current_ab2uvw(ia_ref, ib_ref, &iu_ref, &iv_ref, &iw_ref);		// rot to stationary
	if		(iu_ref >  ihys)	{ *vu_ref += dtc_err; }					// dt comp hysteresis
	else if (iu_ref < -ihys)	{ *vu_ref -= dtc_err; }
	if		(iv_ref >  ihys)	{ *vv_ref += dtc_err; }
	else if (iv_ref < -ihys)	{ *vv_ref -= dtc_err; }
	if		(iw_ref >  ihys)	{ *vw_ref += dtc_err; }
	else if (iw_ref < -ihys)	{ *vw_ref -= dtc_err; }
}


void ctrl_current_reset(void)
{
	xff_q = 0.0; xfb_d = 0.0; xfb_q = 0.0;
	x_id = 0.0, x_iq = 0.0;
}


void ctrl_current_uw2ab(float u, float w, float *a, float *b)
{
	*a = C_SQRT3_2 * u;
	*b = -C_SQRT1_2 * u - C_SQRT2 * w;
}


void ctrl_current_ul2ab(float uv, float vw, float *a, float *b)
{
	*a = C_SQRT2_3 * uv + C_SQRT1_6 * vw;
	*b = C_SQRT1_2 * vw;
}


void ctrl_current_ab2dq(float a, float b, float theta_e, float *d, float *q)
{
	*d =  cossp(theta_e) * a + sinsp(theta_e) * b;
	*q = -sinsp(theta_e) * a + cossp(theta_e) * b;
}


void ctrl_current_dq2ab(float d, float q, float theta_e, float *a, float *b)
{
	*a = cossp(theta_e) * d - sinsp(theta_e) * q;
	*b = sinsp(theta_e) * d + cossp(theta_e) * q;
}


void ctrl_current_ab2uvw(float a, float b, float *u, float *v, float *w)
{
	*u = C_SQRT2_3 * a;
	*v = -C_SQRT1_6 * a + C_SQRT1_2 * b;
	*w = -C_SQRT1_6 * a - C_SQRT1_2 * b;
}
