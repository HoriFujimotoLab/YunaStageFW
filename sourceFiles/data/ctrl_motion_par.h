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
 	{-7.496479599688914e-01}
 	};
float	Dvpi[1][1]	={
 	{-6.849926187990117e+00}
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
 	{1.000000000000000e+00}
 	};
float	Cppi[1][1]	={
 	{1.150277423616814e+00}
 	};
float	Dppi[1][1]	={
 	{1.072043292276092e+02}
 	};
float	xppi[1] 	={ 	0.0e+00	};

//lpf model
float	Alpf[2][2]	={
 	{9.519775726340529e-01,9.220139303001995e-02},
 	{-9.220139303002040e-02,7.675747865740119e-01}
 	};
float	Blpf[2][1]	={
 	{-2.359999863025425e-01},
 	{-1.934214687506972e-01}
 	};
float	Clpf[1][2]	={
 	{-2.359999863025426e-01,1.934214687506974e-01}
 	};
float	Dlpf[1][1]	={
 	{4.915679951610148e-03}
 	};
float	xlpf[2] 	={ 	0.0e+00,0.0e+00	};

//dob model
float	Adob[2][2]	={
 	{7.297613969228740e-01,-1.300147826811592e-01},
 	{1.300147826811576e-01,9.897909622851909e-01}
 	};
float	Bdob[2][1]	={
 	{7.504097986399105e-01},
 	{2.587268299980754e-02}
 	};
float	Cdob[1][2]	={
 	{-7.504097986399102e-01,2.587268299980361e-02}
 	};
float	Ddob[1][1]	={
 	{-3.250786587870792e-01}
 	};
float	xdob[2] 	={ 	0.0e+00,0.0e+00	};

// https://github.com/HoriFujimotoLab/AcqTools 
// Thomas Beauduin, HFlab (University of Tokyo) 
