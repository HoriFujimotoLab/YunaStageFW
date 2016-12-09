/************************************************************************************
SYSTEM GLOBAL DATA MODULE
-------------------------
Descr.:		Header with experimental setup global data
Boards:		PE-Expert3, MWPE-C6713A DSP, MW-WAVE2 acquisition
Author:		Thomas Beauduin, Wataru Ohnishi
			Hori-Fujimoto lab, University of Tokyo, 2016
*************************************************************************************/
#ifndef	SYSTEM_DATA_H
#define	SYSTEM_DATA_H

// BOARD NUMBER for ENCODER
#define	BDN0	(0)							// Board number 0
#define	BDN1	(1)							// Board number 1

// MOTOR PAR
#define		Ke		(26.5)					// Voltage constant		[Nm/V]
#define		Rs		(10.5)					// Stator resistance	[Ohm]
#define		Ls		(9.0e-3)				// Stator inductance	[H]
#define		OVC_LIM	(4.2)					// overcurrent limit	[A]  OVC_LIM	(4.0)
#define		OVV_LIM	(380.0)					// overvorltage limit	[V]
#define		OVS_LIM	(0.80)					// overspeed limit		[m/s]
#define		OVP_MIN (0.05)					// overposition limit (min) [m] // temporary
#define		OVP_MAX (0.60)					// overposition limit (max) [m] // temporary
#define		I_PK	(4.0)					// ctrl out saturation	[A]  I_PK	(3.0)
#define		POS_HOM	(0.3)					// home position of the stage
/* MOTOR SPEC (S200T)
** rated force		25 [N] *2
** rated current	0.54 [A] *2
** peak force		102 [N] *2
** peak current		2.16 [A] *2
** force constant	15.0 [N/A]
** back EMF			16 [V/m/s] ?
** resistance		43 [Ohm] / 2
** inductance		29 [mH] / 2
*/

// INV PAR
#define		FC		(12500)				// carrier frequency	[Hz] : 12.5kHz->12500
#define		TC		(80)                // carrier sampling	[us] : 12.5kHz->80		
#define		DT		(3500)				// inv dead time		[ns]

// STAGE PAR
#define		FS		(2000)	    	// system frequency		[Hz]
#define		TS		(500)   		// system sampling		[us]

// MATH PAR
#define		PI(n)	(3.14159265358979323846 * (n))
#define		sign(a) (((a)<0) ? -1 : ((a)>0))


#endif

