clear all;
close all;
clc;

X_car_0 = [4.0856,4.0856,4.0856,4.0856,4.0856,4.0856,4.0856,4.0856,4.0845,4.0809,4.077,4.073,4.0688,4.0643,4.0597,4.0549,4.0498,4.045,4.045,4.0353,4.0247,4.0132,4.0008,3.9876,3.9735,3.9586,3.9429,3.9264,3.9092,3.8911,3.8723,3.8528,3.85,3.85,3.8315,3.8112,3.7891,3.7654,3.7403,3.714,3.6866,3.6582,3.6292,3.6118,3.6118,3.5819,3.5521,3.5222,3.4923,3.4624,3.4324,3.4025,3.3725,3.3425,3.3361,3.3361,3.3061,3.2761,3.2461,3.2161,3.1861,3.1561,3.1261,3.0961,3.0661,3.0361,3.0061,2.9761,2.9461,2.9161,2.8861,2.8561,2.8261,2.7961,2.7661,2.7361,2.7061,2.6761,2.6461,2.6161,2.5861,2.5561,2.5261,2.4961,2.4661,2.4361,2.4061,2.3761,2.3461,2.3161,2.2861,2.2561,2.2261,2.1961,2.1661,2.1361,2.1061,2.0761,2.0461,2.0161,1.9861,1.9561,1.9261,1.8961,1.8661,1.8361,1.8061,1.7761,1.7461,1.7161,1.6861,1.6561,1.6261,1.5961,1.5661,1.5361,1.5061,1.4761,1.4461,1.4161,1.3861,1.3561,1.3261,1.2961,1.2661,1.2361,1.2061,1.1761,1.1461,1.1161,1.0861,1.0561,1.0261,0.9961,0.9661,0.9361,0.9061,0.8761,0.8461,0.8161,0.7861,0.7561,0.7261,0.6961,0.6661,0.6361,0.6061,0.5761,0.5461,0.5161,0.4861,0.4561,0.4261,0.3961,0.3661,0.3361,0.3061,0.2761,0.2461,0.2161,0.1861,0.1561,0.1261,0.0961,0.0661,0.0361,0.0061,-0.0239,-0.0539,-0.0839,-0.0983,-0.0983,-0.1283,-0.15829,-0.18827,-0.21824,-0.24819,-0.27813,-0.30803,-0.33791,-0.36776,-0.3811,-0.3811,-0.41079,-0.44045,-0.47009,-0.49969,-0.52927,-0.55882,-0.58835,-0.61784,-0.64729,-0.67672,-0.70611,-0.73546,-0.757,-0.757,-0.78655,-0.81588,-0.84496,-0.87375,-0.9022,-0.93028,-0.95794,-0.9657,-0.9657,-0.98725,-1.007,-1.0247,-1.0404,-1.0539,-1.0618,-1.0618,-1.0711,-1.0799,-1.0881,-1.0957,-1.1028,-1.1093,-1.1152,-1.1205,-1.1253,-1.1295,-1.1331,-1.1361,-1.1367,-1.1367,-1.1397,-1.1424,-1.145,-1.1473,-1.1494,-1.1512,-1.1529,-1.1543,-1.1555,-1.1564,-1.1571,-1.1571,-1.1564,-1.1555,-1.1543,-1.1529,-1.1512,-1.1494,-1.1473,-1.145,-1.1424,-1.1397,-1.1367,-1.1367,-1.1361,-1.1331,-1.1295,-1.1253,-1.1205,-1.1152,-1.1093,-1.1028,-1.0957,-1.0881,-1.0799,-1.0711,-1.0618,-1.0618,-1.0539,-1.0404,-1.0247,-1.007,-0.98725,-0.9657,-0.9657,-0.95794,-0.93028,-0.9022,-0.87375,-0.84496,-0.81588,-0.78655,-0.757,-0.757,-0.73546,-0.70611,-0.67672,-0.64729,-0.61784,-0.58835,-0.55882,-0.52927,-0.49969,-0.47009,-0.44045,-0.41079,-0.3811,-0.3811,-0.36776,-0.33791,-0.30803,-0.27813,-0.24819,-0.21824,-0.18827,-0.15829,-0.1283,-0.0983,-0.0983,-0.0839,-0.0539,-0.0239,0.0061,0.0361,0.0661,0.0961,0.1261,0.1561,0.1861,0.2161,0.2461,0.2761,0.3061,0.3361,0.3661,0.3961,0.4261,0.4561,0.4861,0.5161,0.5461,0.5761,0.6061,0.6361,0.6661,0.6961,0.7261,0.7561,0.7861,0.8161,0.8461,0.8761,0.9061,0.9361,0.9661,0.9961,1.0261,1.0561,1.0861,1.1161,1.1461,1.1761,1.2061,1.2361,1.2661,1.2961,1.3261,1.3561,1.3861,1.4161,1.4461,1.4761,1.5061,1.5361,1.5661,1.5961,1.6261,1.6561,1.6861,1.7161,1.7461,1.7761,1.8061,1.8361,1.8661,1.8961,1.9261,1.9561,1.9861,2.0161,2.0461,2.0761,2.1061,2.1361,2.1661,2.1961,2.2261,2.2561,2.2861,2.3161,2.3461,2.3761,2.4061,2.4361,2.4661,2.4961,2.5261,2.5561,2.5861,2.6161,2.6461,2.6761,2.7061,2.7361,2.7661,2.7961,2.8261,2.8561,2.8861,2.9161,2.9461,2.9761,3.0061,3.0361,3.0661,3.0961,3.1261,3.1561,3.1861,3.2161,3.2461,3.2761,3.3061,3.3361,3.3361,3.3425,3.3725,3.4025,3.4324,3.4624,3.4923,3.5222,3.5521,3.5819,3.6118,3.6118,3.6292,3.6582,3.6866,3.714,3.7403,3.7654,3.7891,3.8112,3.8315,3.85,3.85,3.8528,3.8723,3.8911,3.9092,3.9264,3.9429,3.9586,3.9735,3.9876,4.0008,4.0132,4.0247,4.0353,4.045,4.045,4.0498,4.0549,4.0597,4.0643,4.0688,4.073,4.077,4.0809,4.0845,4.0856,4.0856,4.0856,4.0856,4.0856,4.0856,4.0856,4.0856 ];
Y_car_0 = [0,0.03,0.06,0.09,0.12,0.15,0.18,0.1983,0.1983,0.22808,0.25784,0.28756,0.31726,0.34693,0.37657,0.40618,0.43575,0.4629,0.4629,0.49128,0.51934,0.54705,0.57438,0.60131,0.62781,0.65385,0.67942,0.70447,0.729,0.75296,0.77635,0.79913,0.8023,0.8023,0.82591,0.84794,0.86823,0.88666,0.9031,0.91744,0.92958,0.93945,0.94698,0.9503,0.9503,0.95326,0.95603,0.95859,0.96095,0.96311,0.96507,0.96683,0.96838,0.96974,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.97,0.9704,0.97003,0.96934,0.96831,0.96696,0.96527,0.96326,0.96092,0.95825,0.95525,0.9538,0.9538,0.94948,0.94499,0.94033,0.9355,0.9305,0.92533,0.91999,0.91448,0.9088,0.90295,0.89694,0.89075,0.8861,0.8861,0.88092,0.87465,0.86728,0.85884,0.84933,0.83877,0.82717,0.8237,0.8237,0.80284,0.78025,0.75608,0.73052,0.70373,0.6848,0.6848,0.65629,0.6276,0.59874,0.56973,0.54057,0.51128,0.48187,0.45235,0.42273,0.39302,0.36324,0.33339,0.3263,0.3263,0.29645,0.26657,0.23668,0.20677,0.17684,0.1469,0.11695,0.086979,0.057003,0.027019,8.3267e-16,-8.3267e-16,-0.027019,-0.057003,-0.086979,-0.11695,-0.1469,-0.17684,-0.20677,-0.23668,-0.26657,-0.29645,-0.3263,-0.3263,-0.33339,-0.36324,-0.39302,-0.42273,-0.45235,-0.48187,-0.51128,-0.54057,-0.56973,-0.59874,-0.6276,-0.65629,-0.6848,-0.6848,-0.70373,-0.73052,-0.75608,-0.78025,-0.80284,-0.8237,-0.8237,-0.82717,-0.83877,-0.84933,-0.85884,-0.86728,-0.87465,-0.88092,-0.8861,-0.8861,-0.89075,-0.89694,-0.90295,-0.9088,-0.91448,-0.91999,-0.92533,-0.9305,-0.9355,-0.94033,-0.94499,-0.94948,-0.9538,-0.9538,-0.95525,-0.95825,-0.96092,-0.96326,-0.96527,-0.96696,-0.96831,-0.96934,-0.97003,-0.9704,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.97,-0.96974,-0.96838,-0.96683,-0.96507,-0.96311,-0.96095,-0.95859,-0.95603,-0.95326,-0.9503,-0.9503,-0.94698,-0.93945,-0.92958,-0.91744,-0.9031,-0.88666,-0.86823,-0.84794,-0.82591,-0.8023,-0.8023,-0.79913,-0.77635,-0.75296,-0.729,-0.70447,-0.67942,-0.65385,-0.62781,-0.60131,-0.57438,-0.54705,-0.51934,-0.49128,-0.4629,-0.4629,-0.43575,-0.40618,-0.37657,-0.34693,-0.31726,-0.28756,-0.25784,-0.22808,-0.1983,-0.1983,-0.18,-0.15,-0.12,-0.09,-0.06,-0.03,-0 ];
X_car_1 = [4.2856,4.2856,4.2856,4.2856,4.2856,4.2856,4.2856,4.2856,4.2831,4.2795,4.2757,4.2717,4.2675,4.2631,4.2585,4.2537,4.2487,4.2435,4.2418,4.2352,4.2255,4.2152,4.204,4.1922,4.1797,4.1664,4.1525,4.1378,4.1225,4.1065,4.0899,4.0726,4.0547,4.0362,4.017,3.9992,4.0124,3.9942,3.9748,3.9542,3.9326,3.9098,3.886,3.8613,3.8358,3.8094,3.7824,3.7546,3.7264,3.6976,3.6684,3.6446,3.6322,3.6024,3.5725,3.5426,3.5127,3.4828,3.4529,3.4229,3.393,3.363,3.3442,3.3361,3.3061,3.2761,3.2461,3.2161,3.1861,3.1561,3.1261,3.0961,3.0661,3.0361,3.0061,2.9761,2.9461,2.9161,2.8861,2.8561,2.8261,2.7961,2.7661,2.7361,2.7061,2.6761,2.6461,2.6161,2.5861,2.5561,2.5261,2.4961,2.4661,2.4361,2.4061,2.3761,2.3461,2.3161,2.2861,2.2561,2.2261,2.1961,2.1661,2.1361,2.1061,2.0761,2.0461,2.0161,1.9861,1.9561,1.9261,1.8961,1.8661,1.8361,1.8061,1.7761,1.7461,1.7161,1.6861,1.6561,1.6261,1.5961,1.5661,1.5361,1.5061,1.4761,1.4461,1.4161,1.3861,1.3561,1.3261,1.2961,1.2661,1.2361,1.2061,1.1761,1.1461,1.1161,1.0861,1.0561,1.0261,0.9961,0.9661,0.9361,0.9061,0.8761,0.8461,0.8161,0.7861,0.7561,0.7261,0.6961,0.6661,0.6361,0.6061,0.5761,0.5461,0.5161,0.4861,0.4561,0.4261,0.3961,0.3661,0.3361,0.3061,0.2761,0.2461,0.2161,0.1861,0.1561,0.1261,0.0961,0.0661,0.0361,0.0061,-0.0239,-0.0539,-0.0839,-0.0983,-0.099638,-0.12964,-0.15963,-0.18961,-0.21959,-0.24954,-0.27948,-0.3094,-0.3393,-0.36917,-0.399,-0.40317,-0.40933,-0.43902,-0.46868,-0.49832,-0.52793,-0.55752,-0.58707,-0.6166,-0.6461,-0.67557,-0.70501,-0.73441,-0.76378,-0.79312,-0.79962,-0.78785,-0.81742,-0.84682,-0.87603,-0.90503,-0.93378,-0.96227,-0.99046,-1.0183,-1.0459,-1.0484,-1.0735,-1.0987,-1.1205,-1.1413,-1.1608,-1.1791,-1.196,-1.2117,-1.2259,-1.2387,-1.2485,-1.2513,-1.2606,-1.2695,-1.2779,-1.2857,-1.2931,-1.2999,-1.3063,-1.3121,-1.3175,-1.3223,-1.3266,-1.3304,-1.3336,-1.336,-1.3356,-1.3386,-1.3414,-1.3439,-1.3463,-1.3484,-1.3503,-1.352,-1.3535,-1.3548,-1.3559,-1.3567,-1.3571,-1.3571,-1.3567,-1.3559,-1.3548,-1.3535,-1.352,-1.3503,-1.3484,-1.3463,-1.3439,-1.3414,-1.3386,-1.3356,-1.336,-1.3336,-1.3304,-1.3266,-1.3223,-1.3175,-1.3121,-1.3063,-1.2999,-1.2931,-1.2857,-1.2779,-1.2695,-1.2606,-1.2513,-1.2485,-1.2387,-1.2259,-1.2117,-1.196,-1.1791,-1.1608,-1.1413,-1.1205,-1.0987,-1.0735,-1.0484,-1.0459,-1.0183,-0.99046,-0.96227,-0.93378,-0.90503,-0.87603,-0.84682,-0.81742,-0.78785,-0.79962,-0.79312,-0.76378,-0.73441,-0.70501,-0.67557,-0.6461,-0.6166,-0.58707,-0.55752,-0.52793,-0.49832,-0.46868,-0.43902,-0.40933,-0.40317,-0.399,-0.36917,-0.3393,-0.3094,-0.27948,-0.24954,-0.21959,-0.18961,-0.15963,-0.12964,-0.099638,-0.0983,-0.0839,-0.0539,-0.0239,0.0061,0.0361,0.0661,0.0961,0.1261,0.1561,0.1861,0.2161,0.2461,0.2761,0.3061,0.3361,0.3661,0.3961,0.4261,0.4561,0.4861,0.5161,0.5461,0.5761,0.6061,0.6361,0.6661,0.6961,0.7261,0.7561,0.7861,0.8161,0.8461,0.8761,0.9061,0.9361,0.9661,0.9961,1.0261,1.0561,1.0861,1.1161,1.1461,1.1761,1.2061,1.2361,1.2661,1.2961,1.3261,1.3561,1.3861,1.4161,1.4461,1.4761,1.5061,1.5361,1.5661,1.5961,1.6261,1.6561,1.6861,1.7161,1.7461,1.7761,1.8061,1.8361,1.8661,1.8961,1.9261,1.9561,1.9861,2.0161,2.0461,2.0761,2.1061,2.1361,2.1661,2.1961,2.2261,2.2561,2.2861,2.3161,2.3461,2.3761,2.4061,2.4361,2.4661,2.4961,2.5261,2.5561,2.5861,2.6161,2.6461,2.6761,2.7061,2.7361,2.7661,2.7961,2.8261,2.8561,2.8861,2.9161,2.9461,2.9761,3.0061,3.0361,3.0661,3.0961,3.1261,3.1561,3.1861,3.2161,3.2461,3.2761,3.3061,3.3361,3.3442,3.363,3.393,3.4229,3.4529,3.4828,3.5127,3.5426,3.5725,3.6024,3.6322,3.6446,3.6684,3.6976,3.7264,3.7546,3.7824,3.8094,3.8358,3.8613,3.886,3.9098,3.9326,3.9542,3.9748,3.9942,4.0124,3.9992,4.017,4.0362,4.0547,4.0726,4.0899,4.1065,4.1225,4.1378,4.1525,4.1664,4.1797,4.1922,4.204,4.2152,4.2255,4.2352,4.2418,4.2435,4.2487,4.2537,4.2585,4.2631,4.2675,4.2717,4.2757,4.2795,4.2831,4.2856,4.2856,4.2856,4.2856,4.2856,4.2856,4.2856,4.2856];
Y_car_1 = [0,0.03,0.06,0.09,0.12,0.15,0.18,0.1983,0.22178,0.25156,0.28132,0.31105,0.34075,0.37043,0.40007,0.42969,0.45927,0.48882,0.49845,0.52475,0.55316,0.5813,0.60917,0.63673,0.66398,0.69089,0.71745,0.74363,0.76943,0.79482,0.81978,0.8443,0.86836,0.89195,0.91505,0.93546,0.91909,0.94297,0.96585,0.98769,1.0084,1.0279,1.0462,1.0633,1.079,1.0933,1.1062,1.1177,1.1277,1.1362,1.1431,1.1476,1.1493,1.1522,1.155,1.1576,1.16,1.1622,1.1642,1.166,1.1676,1.169,1.1698,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.1704,1.17,1.1694,1.1684,1.1671,1.1656,1.1637,1.1615,1.159,1.1562,1.153,1.1526,1.1518,1.1475,1.143,1.1384,1.1335,1.1286,1.1234,1.1181,1.1127,1.1071,1.1013,1.0953,1.0892,1.0829,1.0815,1.0837,1.0786,1.0727,1.0659,1.0582,1.0496,1.0402,1.03,1.0189,1.0069,1.0058,0.98945,0.9731,0.95256,0.93088,0.90811,0.88433,0.85959,0.83399,0.80758,0.78044,0.75656,0.74883,0.72033,0.69167,0.66286,0.63391,0.60482,0.57562,0.5463,0.51687,0.48735,0.45774,0.42805,0.39829,0.36847,0.34349,0.34688,0.31703,0.28716,0.25727,0.22736,0.19744,0.1675,0.13755,0.10759,0.077612,0.047631,0.017644,0.0043543,-0.0043543,-0.017644,-0.047631,-0.077612,-0.10759,-0.13755,-0.1675,-0.19744,-0.22736,-0.25727,-0.28716,-0.31703,-0.34688,-0.34349,-0.36847,-0.39829,-0.42805,-0.45774,-0.48735,-0.51687,-0.5463,-0.57562,-0.60482,-0.63391,-0.66286,-0.69167,-0.72033,-0.74883,-0.75656,-0.78044,-0.80758,-0.83399,-0.85959,-0.88433,-0.90811,-0.93088,-0.95256,-0.9731,-0.98945,-1.0058,-1.0069,-1.0189,-1.03,-1.0402,-1.0496,-1.0582,-1.0659,-1.0727,-1.0786,-1.0837,-1.0815,-1.0829,-1.0892,-1.0953,-1.1013,-1.1071,-1.1127,-1.1181,-1.1234,-1.1286,-1.1335,-1.1384,-1.143,-1.1475,-1.1518,-1.1526,-1.153,-1.1562,-1.159,-1.1615,-1.1637,-1.1656,-1.1671,-1.1684,-1.1694,-1.17,-1.1704,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.1698,-1.169,-1.1676,-1.166,-1.1642,-1.1622,-1.16,-1.1576,-1.155,-1.1522,-1.1493,-1.1476,-1.1431,-1.1362,-1.1277,-1.1177,-1.1062,-1.0933,-1.079,-1.0633,-1.0462,-1.0279,-1.0084,-0.98769,-0.96585,-0.94297,-0.91909,-0.93546,-0.91505,-0.89195,-0.86836,-0.8443,-0.81978,-0.79482,-0.76943,-0.74363,-0.71745,-0.69089,-0.66398,-0.63673,-0.60917,-0.5813,-0.55316,-0.52475,-0.49845,-0.48882,-0.45927,-0.42969,-0.40007,-0.37043,-0.34075,-0.31105,-0.28132,-0.25156,-0.22178,-0.1983,-0.18,-0.15,-0.12,-0.09,-0.06,-0.03,-0];

car_0 = [X_car_0;Y_car_0];
car_1 = [X_car_1;Y_car_1];

X = [];
Y = [];
for i =0:249
    for j = 0:249
        x = 12.45 - 0.1 * i;
        y = 12.45 - 0.1 * j;
        X = [X,x];
        Y = [Y,y];
    end
end

index_L = 333;

for index_plan = 2
    fid = fopen('E:\Experiments\hybrid_A_star-master\val\plan_HybridAstar.txt', 'rb');
    request = 1;
    log_list = 0;
    log_idx = 0;
    
    path = zeros(300,5);
    while ~feof(fid)
        
        log_list = log_list + 1;
        tline = fgetl(fid);
        
        if log_list == (request-1)*index_L+10+log_idx
            num_str = regexp(tline,'\-?\d*\.?\d*','match');
            num = str2double(num_str);
            P0 = num(2:3);
        elseif log_list == (request-1)*index_L+11+log_idx
            num_str = regexp(tline,'\-?\d*\.?\d*','match');
            num = str2double(num_str);
            P1 = num(2:3);
        elseif log_list == (request-1)*index_L+12+log_idx
            num_str = regexp(tline,'\-?\d*\.?\d*','match');
            num = str2double(num_str);
            P2 = num(2:3);
        elseif log_list == (request-1)*index_L+13+log_idx
            num_str = regexp(tline,'\-?\d*\.?\d*','match');
            num = str2double(num_str);
            P3 = num(2:3);
        elseif log_list == (request-1)*index_L+16+log_idx
            num_str = regexp(tline,'\-?\d*\.?\d*','match');
            num = str2double(num_str);
            START = num;
        elseif log_list == (request-1)*index_L+17+log_idx
            num_str = regexp(tline,'\-?\d*\.?\d*','match');
            num = str2double(num_str);
            END = num;
        elseif log_list == (request-1)*index_L+21+log_idx
            num_str = regexp(tline,'\-?\d*\.?\d*','match');
            num = str2double(num_str);
            map = num;
        elseif (log_list >= (request-1)*index_L+32+log_idx) && (log_list <= (request-1)*index_L+331+log_idx)
            num_str = regexp(tline,'\-?\d*\.?\d*','match');
            num = str2double(num_str);
            path(log_list-((request-1)*index_L+32+log_idx)+1,:) = num;
        end
        
        if (mod(log_list,index_L) == 0)%||(mod(log_list,327) == 25)
            status = reshape(map,250,250);
            Map = status';
            X_car = reshape(X,250,250);
            X_car = X_car';
            Y_car = reshape(Y,250,250);
            Y_car = Y_car';
            
            park = [P0;P1;P2;P3;P0];
            
            figure(request)
            for index_x = 1:250
                for index_y = 1:250
                    if Map(index_x,index_y) == 0 
                        Car_r = [X_car(index_x,index_y)-0.05,Y_car(index_x,index_y)-0.05;X_car(index_x,index_y)+0.05,Y_car(index_x,index_y)-0.05;X_car(index_x,index_y)+0.05,Y_car(index_x,index_y)+0.05;X_car(index_x,index_y)-0.05,Y_car(index_x,index_y)+0.05;X_car(index_x,index_y)-0.05,Y_car(index_x,index_y)-0.05];
                        plot(Car_r(:,1),Car_r(:,2),'r');hold on;
                        plot(X_car(index_x,index_y),Y_car(index_x,index_y),'.r');hold on;
                    elseif Map(index_x,index_y) == 3
                        plot(X_car(index_x,index_y),Y_car(index_x,index_y),'.b');hold on;
                    elseif Map(index_x,index_y) == 4
                        plot(X_car(index_x,index_y),Y_car(index_x,index_y),'.k');hold on;
                    elseif Map(index_x,index_y) == 5
                        plot(X_car(index_x,index_y),Y_car(index_x,index_y),'.y');hold on;
                    end
                end
            end
            CAR_0s = [cos(START(3)),-sin(START(3));sin(START(3)),cos(START(3))]*car_0+[START(1);START(2)];
            CAR_1s = [cos(START(3)),-sin(START(3));sin(START(3)),cos(START(3))]*car_1+[START(1);START(2)];
            
            CAR_0 = [cos(END(3)),-sin(END(3));sin(END(3)),cos(END(3))]*car_0+[END(1);END(2)];
            CAR_1 = [cos(END(3)),-sin(END(3));sin(END(3)),cos(END(3))]*car_1+[END(1);END(2)];
            plot(START(1),START(2),'ob');
            plot(END(1),END(2),'oy');
            plot(CAR_0s(1,:),CAR_0s(2,:),'.b');hold on;
            plot(CAR_1s(1,:),CAR_1s(2,:),'.b');hold on;
            plot(CAR_0(1,:),CAR_0(2,:),'.y');hold on;
            plot(CAR_1(1,:),CAR_1(2,:),'.y');hold on;
            plot(park(:,1),park(:,2),'g');hold on;
            for pathplan_index = 1:300
                if path(pathplan_index,5) == 0
                    break;
                end
            end
            plot(path(1:pathplan_index-1,1)*0.01,path(1:pathplan_index-1,2)*0.01,'k');
            path = zeros(300,5);
            axis equal
            xlim([-12.5,12.5]);ylim([-12.5,12.5]);
            drawnow;
            request = request +1;
        end
    end
    if (mod(log_list,index_L) ~= 0)
        status = reshape(map,250,250);
        Map = status';
        X_car = reshape(X,250,250);
        X_car = X_car';
        Y_car = reshape(Y,250,250);
        Y_car = Y_car';

        park = [P0;P1;P2;P3;P0];

        figure(request)
        for index_x = 1:250
            for index_y = 1:250
                if Map(index_x,index_y) == 0 
                    Car_r = [X_car(index_x,index_y)-0.05,Y_car(index_x,index_y)-0.05;X_car(index_x,index_y)+0.05,Y_car(index_x,index_y)-0.05;X_car(index_x,index_y)+0.05,Y_car(index_x,index_y)+0.05;X_car(index_x,index_y)-0.05,Y_car(index_x,index_y)+0.05;X_car(index_x,index_y)-0.05,Y_car(index_x,index_y)-0.05];
                    plot(Car_r(:,1),Car_r(:,2),'r');hold on;
                    plot(X_car(index_x,index_y),Y_car(index_x,index_y),'.r');hold on;
                elseif Map(index_x,index_y) == 3
                    plot(X_car(index_x,index_y),Y_car(index_x,index_y),'.b');hold on;
                elseif Map(index_x,index_y) == 4
                    plot(X_car(index_x,index_y),Y_car(index_x,index_y),'.k');hold on;
                elseif Map(index_x,index_y) == 5
                    plot(X_car(index_x,index_y),Y_car(index_x,index_y),'.y');hold on;
                end
            end
        end
        %把车身坐标系下的坐标转换为以起点坐标为原点的坐标系下的坐标,表示整个车在起点处的位置
        CAR_0s = [cos(START(3)),-sin(START(3));sin(START(3)),cos(START(3))]*car_0+[START(1);START(2)];
        CAR_1s = [cos(START(3)),-sin(START(3));sin(START(3)),cos(START(3))]*car_1+[START(1);START(2)];
        %把车身坐标系下的坐标转换为以终点坐标为原点的坐标系下的坐标，表示整个车在终点处的位置
        CAR_0 = [cos(END(3)),-sin(END(3));sin(END(3)),cos(END(3))]*car_0+[END(1);END(2)];
        CAR_1 = [cos(END(3)),-sin(END(3));sin(END(3)),cos(END(3))]*car_1+[END(1);END(2)];
        plot(START(1),START(2),'ob');
        plot(END(1),END(2),'oy');
        plot(CAR_0s(1,:),CAR_0s(2,:),'.b');hold on;
        plot(CAR_1s(1,:),CAR_1s(2,:),'.b');hold on;
        plot(CAR_0(1,:),CAR_0(2,:),'.y');hold on;
        plot(CAR_1(1,:),CAR_1(2,:),'.y');hold on;
        plot(park(:,1),park(:,2),'g');hold on;

        for pathplan_index = 1:300
            if path(pathplan_index,5) == 0
                break;
            end
        end
        plot(path(1:pathplan_index-1,1)*0.01,path(1:pathplan_index-1,2)*0.01,'k');
        axis equal
        xlim([-12.5,12.5]);ylim([-12.5,12.5]);
    end
    fclose(fid);
 end