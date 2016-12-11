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

// MOTOR SPECS
#define		Pp		(4.0)					// pole pairs amount	[-]
#define		Kt		(0.564)					// Torque constant		[Nm/A]
#define		Ke		(0.141)					// Voltage constant		[Nm/V]
#define		Rs		(0.38)					// Stator resistance	[Ohm]
#define		Ls		(0.003)                 // Stator inductance	[H]
#define		OVC_LIM	(27.0)					// overcurrent limit	[A]
#define		OVV_LIM	(380.0)					// overvorltage limit	[V]
#define		OVS_LIM	(400.0)					// overspeed limit		[rad/s]
#define		I_PK	(20.0)					// ctrl out saturation	[A]

// INV PAR
#define		FC		(12500.0)				// carrier frequency	[Hz] : 12.5kHz->12500
#define		TC		(80.0)		            // carrier sampling	[us] : 12.5kHz->80		
#define		DT		(3500.0)	 			// inv dead time		[ns]

// STAGE PAR
#define		FS		(2000.0)	    		// system frequency		[Hz]
#define		TS		(500.0)   				// system sampling		[us]


#endif

