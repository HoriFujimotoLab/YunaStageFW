/************************************************************************************
STAGE SENSOR ADC MODULE
-----------------------
Descr.:		Hardware module adc of stage sensors
Boards:		MWPE3-C6713A, MWPE3-ADC (14 bit)
Sensor:		hioki,kyowa,keyence: Yuna ball-screw setup
Author:		Thomas Beauduin, University of Tokyo, December 2016
*************************************************************************************/
#include	"hardw_adc.h"
#include	"system_math.h"
#include	"data/system_data.h"
#include	<mwio3.h>

// MODULE PAR
#define		ADC_BDN	0					// adc board number
#define		NROFM	(500.0)				// number of offset msr

// ADC RANGES
#define		R_EC	(1000.0)			// Eddy-Current sensors	[um] (1mm/5V)
#define		R_LC	(2000.0)			// Load-Cell sensors	[N] (2kN/5V)
#define		R_IS	(31.25)				// Hioki current sensor [A] (31.25A/5V)
#define		R_VS	(400.0)				// DC voltage sensor	[V] (400V/5V)
#define		R_DC	(50.0)				// DC current sensor	[A] (50A/5V)
#define		R_SV	(1.0)				// Servo Analyzer		[%] (100%/5V)

#define		R_AM	(94.1620)			// AccMotor A17sn91989z	[m/s2] (53.1mV/m/s^2)
#define		R_AS	(92.4214)			// AccScrew A17sn91991z	[m/s2] (54.1mV/m/s^2)
#define		R_AT	(106.610)			// AccTable A17sn24104y	[m/s2] (46.9mV/m/s^2)
#define		R_AP	(500.501)			// AccPitch A32sn36789y	[m/s2] (9.99mV/m/s^2)

// ADC OFFSETS (see notes)
#define		O_VDC	(-2.65)				// ch2 adc offset: vdc
#define		O_LC1	(-15.0)				// ch5 adc offset: load-cell
#define		O_LC2	(-36.5)				// ch6 adc offset: load-cell

// MODULE VAR
// inverter
float vu_ad, vw_ad, iu_ad, iw_ad;
float va_ad, vb_ad, ia_ad, ib_ad;
float vd_ad, vq_ad, id_ad, iq_ad;
float vdc_ad, idc_ad;
// sensor level
float disp_s1, disp_s2, disp_m1, disp_m2;
float load_m, load_s, servo_ad, temp_s;
float acc_mx, acc_tx, acc_tz, acc_sx;
// local
float ad_avg[12] = { 0.0 };				// MW-ADC average offsets

void hardw_adc_init(void)
{
	// LOCAL VAR
	int i = 0, j = 0;													// loop counters
	float ad0, ad1, ad2, ad3;											// measured values
	float sum0 = 0.0, sum1 = 0.0, sum2 = 0.0, sum3 = 0.0;				// value summation

	// RANGE SET						
	adc_ad_init(ADC_BDN);												// init ADC board
	adc_ad_set_range(ADC_BDN, 0, R_VS, R_DC, R_IS, R_IS);				// grp 0 range settings
	adc_ad_set_range(ADC_BDN, 1, R_LC, R_LC, R_AM, R_AT);				// grp 1 range settings
	adc_ad_set_range(ADC_BDN, 2, R_EC, R_EC, R_EC, R_EC);				// grp 2 range settings
	
	// AVG CALC
	for (i = 0; i < 3; i++){
		adc_ad_set_offset(ADC_BDN, i, 0.0, 0.0, 0.0, 0.0);				// initial offset
		for (j = 0; j < NROFM; j++){
			adc_ad_in_grp(ADC_BDN, i, &ad0, &ad1, &ad2, &ad3);			// ADC group x read
			sum0 -= ad0; sum1 -= ad1; sum2 -= ad2; sum3 -= ad3;			// offset neg sum calc
		}
		ad_avg[4*i] = sum0 / NROFM; ad_avg[4*i+1] = sum1 / NROFM;		// channel avg calc
		ad_avg[4*i+2] = sum2 / NROFM; ad_avg[4*i+3] = sum3 / NROFM;
		sum0 = 0.0; sum1 = 0.0; sum2 = 0.0; sum3 = 0.0; j = 0;
	}
	
	// OFFSET SET
	adc_ad_set_offset(ADC_BDN, 0, O_VDC, ad_avg[1], ad_avg[2], ad_avg[3]);
	adc_ad_set_offset(ADC_BDN, 1, O_LC1, O_LC2, ad_avg[6], ad_avg[7]);
	adc_ad_set_offset(ADC_BDN, 2, ad_avg[8], ad_avg[9], ad_avg[10], ad_avg[11]);
}


void hardw_adc_read(int grp_ad, float *ad0, float *ad1, float *ad2, float *ad3)
{
	adc_ad_in_grp(ADC_BDN, grp_ad, &*ad0, &*ad1, &*ad2, &*ad3);
	if (grp_ad == 0) { *ad2 = -*ad2; *ad3 = -*ad3; }					// hioki cabling inverse
}


/* OFFSET PROCEDURE:
** offset cause: a) sensors offset, b) adc-board offset
** offset cases: 
** case 1)	remove both offsets (init data=0):
**			msr & calculate offset at init (ec, ls, ex sensors)
** case 2)  remove board offset only, info in sensor offset:
**			dedicated experiment for adc-board (lc sensor)
**			sampling (30min), removal of bnc cables, offset msr
*/

