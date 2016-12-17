// CREST FACTOR = 1.391371
// SIGNAL LENGTH = 200 [ms]

/* HARMONICS PARAMETERS 
** ----------------------- 
** fs = 2500.000000 [Hz]: sampling frequency 
** df = 5.000000 [Hz]: frequency resolution 
** fl = 1.000000 [Hz]: lowest frequency 
** fh = 250.000000 [Hz]: highest frequency 
** fr = 1.020000 : frequency log ratio 
*/ 

/* DESIGN OPTIONS 
** ----------------------- 
** itp = s : init phase type:  s=schroeder/r=random 
** ctp = c : compression type: c=comp/n=no_comp 
** dtp = f : signal type:      f=full/ O=odd-odd 
**                             o=odd / O2=special odd-odd
** gtp = q : grid type: l=linear/q=quasi-logarithmic 
*/ 

/* AMPLITUDE SPECTRUM 
** ----------------------- 
** tf([num,den]) 
** num = [ 3.132015e-12 4.198996e-08 3.343448e-04 9.996844e-01 ]
** den = [ 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 ]
*/ 

#define NROFS 500 
far float refvec [NROFS] = { 
-0.8735588581, -0.1802603113, 0.5632251304, 1.1406436023, 1.3889311826, 1.2478472386, 0.7755886071, 0.1265873604, -0.4997347331, -0.9214065932, 
-1.0270093616, -0.8031839173, -0.3296988877, 0.2537079190, 0.7956341299, 1.1795542181, 1.3538280031, 1.3373234924, 1.2017397171, 1.0394260280, 
0.9294129473, 0.9134726246, 0.9890807748, 1.1193029863, 1.2535233442, 1.3497207875, 1.3895818969, 1.3815396758, 1.3521143050, 1.3305526314, 
1.3339898781, 1.3595113219, 1.3861338692, 1.3853615201, 1.3354023510, 1.2327581359, 1.0961623424, 0.9611289458, 0.8672855363, 0.8436055177, 
0.8974710622, 1.0119150274, 1.1520896549, 1.2783378373, 1.3606596900, 1.3888736382, 1.3745279284, 1.3439034809, 1.3249589405, 1.3334374323, 
1.3636823853, 1.3878999602, 1.3643820366, 1.2518137958, 1.0245151259, 0.6831328402, 0.2569847693, -0.2027256511, -0.6364091759, -0.9916873595, 
-1.2360097199, -1.3625650382, -1.3883100416, -1.3456739550, -1.2714077892, -1.1965113608, -1.1401686640, -1.1086985304, -1.0985361212, -1.1009988292, 
-1.1064777630, -1.1066286624, -1.0945636128, -1.0642356887, -1.0105827299, -0.9313842485, -0.8305021463, -0.7208801089, -0.6250732454, -0.5715936396, 
-0.5869186976, -0.6850449026, -0.8581174762, -1.0721261464, -1.2705405155, -1.3862913426, -1.3594895372, -1.1558187172, -0.7796347312, -0.2769444260, 
0.2736246644, 0.7799346972, 1.1601997068, 1.3644980416, 1.3873610847, 1.2675743668, 1.0753336232, 0.8908538722, 0.7811032341, 0.7816831527, 
0.8889444361, 1.0639465181, 1.2460796785, 1.3713729599, 1.3895861597, 1.2752998110, 1.0307992220, 0.6815346605, 0.2672027200, -0.1677253818, 
-0.5810235770, -0.9361648040, -1.2031229338, -1.3586696902, -1.3882140322, -1.2898812036, -1.0795918918, -0.7943427498, -0.4905556792, -0.2356061911, 
-0.0931302301, -0.1055230636, -0.2790148472, -0.5768857484, -0.9244454901, -1.2258328717, -1.3886577914, -1.3494609737, -1.0921025463, -0.6529875450, 
-0.1110314127, 0.4348218577, 0.8935661446, 1.2067045134, 1.3615969525, 1.3894841445, 1.3492511005, 1.3028247623, 1.2909154829, 1.3176193632, 
1.3492485154, 1.3277379052, 1.1938379175, 0.9118342891, 0.4869498149, -0.0309247963, -0.5588250033, -1.0040182021, -1.2913926822, -1.3859066793, 
-1.3027470230, -1.1019712135, -0.8695387724, -0.6910218176, -0.6264669509, -0.6941997744, -0.8681533783, -1.0887411768, -1.2829679124, -1.3868119399, 
-1.3627106780, -1.2071545878, -0.9470558039, -0.6273662526, -0.2950742606, 0.0146092091, 0.2854335581, 0.5190054903, 0.7268640529, 0.9195322425, 
1.0971319223, 1.2452295824, 1.3374048439, 1.3434616463, 1.2401702357, 1.0206151353, 0.6988337272, 0.3081568401, -0.1061597924, -0.4975645120, 
-0.8287072413, -1.0786298631, -1.2446901151, -1.3390086472, -1.3809211426, -1.3880432608, -1.3688333763, -1.3189646746, -1.2226297583, -1.0584657252, 
-0.8084868083, -0.4675459031, -0.0505891075, 0.4046403789, 0.8408259669, 1.1902365331, 1.3883103269, 1.3888082490, 1.1768550306, 0.7761847303, 
0.2478899300, -0.3201354532, -0.8314376502, -1.2029488864, -1.3850983038, -1.3744005970, -1.2143674312, -0.9835672817, -0.7733454266, -0.6610268120, 
-0.6862616182, -0.8377924611, -1.0552307751, -1.2460912182, -1.3136164218, -1.1873299311, -0.8470220857, -0.3325395912, 0.2640801314, 0.8221643743, 
1.2247338931, 1.3898500649, 1.2914876093, 0.9637769547, 0.4878082220, -0.0342185932, -0.5087523544, -0.8748266009, -1.1147570168, -1.2489936770, 
-1.3181387952, -1.3595944674, -1.3880260457, -1.3872106388, -1.3164701004, -1.1293369260, -0.7973779988, -0.3299311516, 0.2182868451, 0.7555552855, 
1.1756640412, 1.3875504293, 1.3422624096, 1.0480992643, 0.5686675561, 0.0043686832, -0.5365290783, -0.9675819994, -1.2448543149, -1.3721238704, 
-1.3896851144, -1.3516340156, -1.3005210128, -1.2490264451, -1.1755800623, -1.0355973554, -0.7841269125, -0.4013091205, 0.0892736505, 0.6170222550, 
1.0781280193, 1.3632597866, 1.3906009187, 1.1335144744, 0.6335289511, -0.0061140330, -0.6449606480, -1.1408152646, -1.3857384190, -1.3321444361, 
-1.0018626673, -0.4764191287, 0.1275298967, 0.6895497962, 1.1141394746, 1.3490300592, 1.3892139542, 1.2675923304, 1.0371078461, 0.7512328061, 
0.4492580714, 0.1503640302, -0.1430237883, -0.4353298965, -0.7260651481, -1.0010633628, -1.2301047301, -1.3718642691, -1.3846002081, -1.2392028072, 
-0.9306172903, -0.4842545644, 0.0445364021, 0.5780683781, 1.0294990355, 1.3189835093, 1.3896658365, 1.2204197032, 0.8327355443, 0.2899955481, 
-0.3114619159, -0.8586569892, -1.2432620671, -1.3840423807, -1.2457160973, -0.8495628546, -0.2723723143, 0.3672745849, 0.9333244110, 1.3021458978, 
1.3910392202, 1.1781964315, 0.7087346224, 0.0848409659, -0.5577853696, -1.0813324806, -1.3771565187, -1.3891178659, -1.1234742743, -0.6436287845, 
-0.0521178248, 0.5346304124, 1.0119150296, 1.3066507207, 1.3877850773, 1.2665627802, 0.9886402258, 0.6215661497, 0.2412115957, -0.0803344451, 
-0.2843819948, -0.3328976776, -0.2138410783, 0.0555195822, 0.4283308363, 0.8325238264, 1.1810908394, 1.3880632722, 1.3877892645, 1.1530333002, 
0.7064533398, 0.1208531598, -0.4936806508, -1.0137624356, -1.3320278146, -1.3837154782, -1.1626422189, -0.7215753567, -0.1567396849, 0.4187618046, 
0.9042508940, 1.2343486037, 1.3886480731, 1.3861840793, 1.2681672646, 1.0766048315, 0.8376793860, 0.5565652320, 0.2254886897, -0.1588856500, 
-0.5769965894, -0.9762483430, -1.2758731167, -1.3877296247, -1.2472557310, -0.8436882774, -0.2373199035, 0.4451529772, 1.0390567014, 1.3872594296, 
1.3891381797, 1.0366618954, 0.4241619049, -0.2737675802, -0.8514025530, -1.1337906207, -1.0320273862, -0.5736817615, 0.1024998899, 0.7871873812, 
1.2643874942, 1.3788022436, 1.0851888608, 0.4637534937, -0.3036251290, -0.9890025045, -1.3882780237, -1.3833976294, -0.9774976817, -0.2924838077, 
0.4694201532, 1.0873711295, 1.3872457139, 1.2906603855, 0.8336833947, 0.1513067777, -0.5655704280, -1.1260618532, -1.3908523462, -1.3050482224, 
-0.9050328378, -0.3005467015, 0.3597518015, 0.9275169906, 1.2877257753, 1.3784450601, 1.1956568723, 0.7859731757, 0.2323496719, -0.3623767332, 
-0.8913168721, -1.2581833202, -1.3913705261, -1.2574541609, -0.8721678969, -0.3049220752, 0.3276240336, 0.8815630393, 1.2177847311, 1.2394129673, 
0.9239833865, 0.3388745422, -0.3681037848, -1.0039483663, -1.3837714831, -1.3871176200, -0.9992697391, -0.3225666274, 0.4486245042, 1.0839453131, 
1.3878757023, 1.2629826003, 0.7443195550, -0.0078099482, -0.7561527929, -1.2614383248, -1.3599471558, -1.0179751529, -0.3444529979, 0.4433695850, 
1.0904807980, 1.3876995712, 1.2402879358, 0.6983056067, -0.0617328808, -0.7977223105, -1.2845113807, -1.3897170605, -1.1166535669, -0.5996150683, 
-0.0533719115, 0.3052099379, 0.3356908938, 0.0248903311, -0.5080718396, -1.0544533140, -1.3912857374, -1.3627910184, -0.9388271754, -0.2293447060, 
0.5507064444, 1.1540756808, 1.3862139055, 1.1728037988, 0.5860627665, -0.1795294621, -0.8737068675, -1.2759422526, -1.2702299642, -0.8831688958, 
-0.2723170466, 0.3307695873, 0.7026014584, 0.7060983363, 0.3379385590, -0.2712637123, -0.9034788237, -1.3296984554, -1.3892733132, -1.0444994547, 
-0.3918656446, 0.3731773287, 1.0247568111, 1.3838446536, 1.3770880995, 1.0570672332, 0.5778308575, 0.1354638323, -0.1043967636, -0.0659862314, 
0.2112083691, 0.5896279262, 0.8866069263, 0.9436992822, 0.6873453633, 0.1591856188, -0.4947470531, -1.0721584208, -1.3831100217, -1.3152985013, 
}; 