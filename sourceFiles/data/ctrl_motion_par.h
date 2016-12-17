// STATE-SPACE PARAMETERS HEADER 
// automatically generated by acqtools repository 
//hom model
float	Ahom[1][1]	={
 	{1.000000000000000e+00}
 	};
float	Bhom[1][1]	={
 	{2.500000000000000e-01}
 	};
float	Chom[1][1]	={
 	{-1.356714655665146e-01}
 	};
float	Dhom[1][1]	={
 	{-1.769665149356074e+00}
 	};
float	xhom[1] 	={ 	0.0e+00	};

//vpi model
float	Avpi[1][1]	={
 	{1.000000000000000e+00}
 	};
float	Bvpi[1][1]	={
 	{5.000000000000000e-01}
 	};
float	Cvpi[1][1]	={
 	{-4.800546306742266e-01}
 	};
float	Dvpi[1][1]	={
 	{-5.379725575141412e+00}
 	};
float	xvpi[1] 	={ 	0.0e+00	};

//shp model
float	Ashp[2][2]	={
 	{9.604973063316178e-01,-2.342607652511368e-01},
 	{2.342607652511370e-01,9.573932063715052e-01}
 	};
float	Bshp[2][1]	={
 	{8.689754397809768e-02},
 	{-7.312864914987596e-02}
 	};
float	Cshp[1][2]	={
 	{8.689754397809750e-02,7.312864914987619e-02}
 	};
float	Dshp[1][1]	={
 	{9.454079195155637e-01}
 	};
float	xshp[2] 	={ 	0.0e+00,0.0e+00	};

//ppi model
float	Appi[1][1]	={
 	{1.000000000000000e+00}
 	};
float	Bppi[1][1]	={
 	{5.000000000000000e-01}
 	};
float	Cppi[1][1]	={
 	{3.744722939308076e-01}
 	};
float	Dppi[1][1]	={
 	{4.274529427980302e+01}
 	};
float	xppi[1] 	={ 	0.0e+00	};

//lpf model
float	Alpf[2][2]	={
 	{9.501267215752560e-01,1.014265738816710e-01},
 	{-1.014265738816707e-01,8.040724551856497e-01}
 	};
float	Blpf[2][1]	={
 	{-2.459525068384190e-01},
 	{-2.041233945480092e-01}
 	};
float	Clpf[1][2]	={
 	{-2.459525068384189e-01,2.041233945480091e-01}
 	};
float	Dlpf[1][1]	={
 	{5.014724720744154e-03}
 	};
float	xlpf[2] 	={ 	0.0e+00,0.0e+00	};

//dob model
float	Adob[1][1]	={
 	{-1.000000000000000e+00}
 	};
float	Bdob[1][1]	={
 	{1.128360130745570e+01}
 	};
float	Cdob[1][1]	={
 	{1.165930379744041e+01}
 	};
float	Ddob[1][1]	={
 	{-6.613096499103821e+01}
 	};
float	xdob[1] 	={ 	0.0e+00	};

// https://github.com/HoriFujimotoLab/AcqTools 
// Thomas Beauduin, HFlab (University of Tokyo) 
