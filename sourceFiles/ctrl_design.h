/************************************************************************************
GENERAL FILTER DESIGN
----------------------
Descr.:		Filter design functions
Author:		Wataru Ohnishi, The University of Tokyo
*************************************************************************************/
#ifndef	CTRL_DESIGN_H
#define	CTRL_DESIGN_H

/*	DESIGN 1ST ORDER FILTER
**	-------------------
**	DES:	Discretize by Tustin and realize it as control canonical form
**	INP:	(b1*s + b0)/(a1*s + a0)
**			Ts		: Sampling period [s]
**	OUT:	[A,B,C,D]
**  REF:    c2d_tustin_sym.m in SymbolicMathTools
*/
void ctrl_FiltInit1(float b1, float b0, float a1, float a0, float Ts, float *A, float *B, float *C, float *D);


/*	DESIGN 2ND ORDER FILTER
**	-------------------
**	DES:	Discretize by Tustin and realize it as control canonical form
**	INP:	(b2*s^2 + b1*s + b0)/(a2*s^2 + a1*s + a0)
**			Ts		: Sampling period [s]
**	OUT:	[A,B,C,D]
**  REF:    c2d_tustin_sym.m in SymbolicMathTools
*/
void ctrl_FiltInit2(float b2, float b1, float b0, float a2, float a1, float a0, float Ts, float *A, float *B, float *C, float *D);


/*	DESIGN 1ST ORDER LOW PASS FILTER
**	-------------------
**	DES:	Design 1st order low pass filter with discretization and realization
**	INP:	freqHz	: Cut off frequency [Hz]
**			Ts		: Sampling period [s]
**	OUT:	[A,B,C,D]
**  REF:    ctrlLPF1_sym.m in SymbolicMathTools
*/
void ctrl_LPFInit1(float freqHz, float Ts, float *A, float *B, float *C, float *D);


/*	DESIGN 1ST ORDER HIGH PASS FILTER
**	-------------------
**	DES:	Design 1st order high pass filter with discretization and realization
**	INP:	freqHz	: Cut off frequency [Hz]
**			Ts		: Sampling period [s]
**	OUT:	[A,B,C,D]
**  REF:    ctrlHPF1_sym.m in SymbolicMathTools
*/
void ctrl_HPFInit1(float freqHz, float Ts, float *A, float *B, float *C, float *D);


/*	DESIGN 2ND ORDER LOW PASS FILTER
**	-------------------
**	DES:	Design 2nd order high pass filter with discretization and realization
**	INP:	freqHz	: Cut off frequency [Hz]
**			dp		: Damping
**			Ts		: Sampling period [s]
**	OUT:	[A,B,C,D]
**  REF:    ctrlLPF2_sym.m in SymbolicMathTools
*/
void ctrl_LPFInit2(float freqHz, float dp, float Ts, float *A, float *B, float *C, float *D);


/*	DESIGN PID CONTROLLER BY POLE PLACEMENT
**	-------------------
**	DES:	Design PID controller with discretization and realization by Sylvester matrix
**			Ref. G. C. Goodwin, S. F. Graebe, and M. E. Salgado, Control System Design. 2000.
**	INP:	Nominal plant:	Pn = Ktn/(mn*s^2 + bn*s + kn) 
**			fp		: Desired closed loop pole
**			Ts		: Sampling period [s]
**	OUT:	[A,B,C,D]
**  REF:    ctrlPID_sym.m in SymbolicMathTools
*/
void ctrl_PID(float mn, float bn, float kn, float Ktn, float fp, float Ts, float *A, float *B, float *C, float *D);


#endif

