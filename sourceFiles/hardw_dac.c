/************************************************************************************
STAGE SENSOR ADC MODULE
-----------------------
Descr.:		Hardware module adc of setup sensors
Boards:		
Author:		Wataru Ohnishi, University of Tokyo, June 2016
*************************************************************************************/
#include	"hardw_dac.h"
#include	"system_data.h"
#include	<mwio3.h>


#define		RNG		(10.0)	//	DAC Range
#define		NROFCH	(8)		//	number of channels


void hardw_dac_init(int bdn_ad)
{
	// LOCAL VAR
	int i = 0;													// loop counters
	
	dac_da_init(bdn_ad);

	for (i = 0; i < NROFCH; i++) {
		dac_da_set_range(bdn_ad, i, RNG);
	}

}


/* void dac_da_set_range( inc bdn, int ch, float rng );
void daca_da_set_range( inc bdn, int ch, float rng );
bdn: ボード番号 0 ≦ bdn ≦ 3
ch: DA 出力チャンネル番号 0 ≦ ch ≦ 7 (MWPE3-DAC)
ch: DA 出力チャンネル番号 0 ≦ ch ≦ 11 (MWPE3-DACA)
rng: レンジDA コンバータの最大値10V 出力する時の値

本システムでは、AD とDA の基準となる出力、入力の電圧が異なる
ので注意してください。
AD は5V を基準としています。
DA は10V を基準としています。
AD から5V を入力した時、内部で5 の値に換算し、DA から5V と
して出力できるプログラムのソースの例を以下に示します。(例は
MWPE3-DACA 用)
pev_ad_set_range( 0, 0, 5, 5, 5, 5 ); //  5V で5 と換算する 
daca_da_set_range(0, 0, 10); // 10 で10V 出力する 

pev_ad_in_grp(0, 0, &c0, &c1, &c2, &c3);
daca_da_out(0, 0, c0);
MWPE3 - DAC ボード(dac_da_set_range())では、DA 出力チャンネ
ル8 ~11 を利用することができません。

*/



