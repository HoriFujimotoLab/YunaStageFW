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
bdn: �{�[�h�ԍ� 0 �� bdn �� 3
ch: DA �o�̓`�����l���ԍ� 0 �� ch �� 7 (MWPE3-DAC)
ch: DA �o�̓`�����l���ԍ� 0 �� ch �� 11 (MWPE3-DACA)
rng: �����WDA �R���o�[�^�̍ő�l10V �o�͂��鎞�̒l

�{�V�X�e���ł́AAD ��DA �̊�ƂȂ�o�́A���͂̓d�����قȂ�
�̂Œ��ӂ��Ă��������B
AD ��5V ����Ƃ��Ă��܂��B
DA ��10V ����Ƃ��Ă��܂��B
AD ����5V ����͂������A������5 �̒l�Ɋ��Z���ADA ����5V ��
���ďo�͂ł���v���O�����̃\�[�X�̗���ȉ��Ɏ����܂��B(���
MWPE3-DACA �p)
pev_ad_set_range( 0, 0, 5, 5, 5, 5 ); //  5V ��5 �Ɗ��Z���� 
daca_da_set_range(0, 0, 10); // 10 ��10V �o�͂��� 

pev_ad_in_grp(0, 0, &c0, &c1, &c2, &c3);
daca_da_out(0, 0, c0);
MWPE3 - DAC �{�[�h(dac_da_set_range())�ł́ADA �o�̓`�����l
��8 ~11 �𗘗p���邱�Ƃ��ł��܂���B

*/



