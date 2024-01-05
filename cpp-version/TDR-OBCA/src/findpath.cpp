#include "findpath.h"
//#include "util_time.h"
#define OBCA
// #define voronoi

//////////////new//////////////////////
// bai
double Rect_x[512] = { 4.2856,4.2856,4.2856,4.2856,4.2856,4.2856,4.2856,4.2856,4.2831,4.2795,4.2757,4.2717,4.2675,4.2631,4.2585,4.2537,4.2487,4.2435,4.2418,4.2352,4.2255,4.2152,4.204,4.1922,4.1797,4.1664,4.1525,4.1378,4.1225,4.1065,4.0899,4.0726,4.0547,4.0362,4.017,3.9992,4.0124,3.9942,3.9748,3.9542,3.9326,3.9098,3.886,3.8613,3.8358,3.8094,3.7824,3.7546,3.7264,3.6976,3.6684,3.6446,3.6322,3.6024,3.5725,3.5426,3.5127,3.4828,3.4529,3.4229,3.393,3.363,3.3442,3.3361,3.3061,3.2761,3.2461,3.2161,3.1861,3.1561,3.1261,3.0961,3.0661,3.0361,3.0061,2.9761,2.9461,2.9161,2.8861,2.8561,2.8261,2.7961,2.7661,2.7361,2.7061,2.6761,2.6461,2.6161,2.5861,2.5561,2.5261,2.4961,2.4661,2.4361,2.4061,2.3761,2.3461,2.3161,2.2861,2.2561,2.2261,2.1961,2.1661,2.1361,2.1061,2.0761,2.0461,2.0161,1.9861,1.9561,1.9261,1.8961,1.8661,1.8361,1.8061,1.7761,1.7461,1.7161,1.6861,1.6561,1.6261,1.5961,1.5661,1.5361,1.5061,1.4761,1.4461,1.4161,1.3861,1.3561,1.3261,1.2961,1.2661,1.2361,1.2061,1.1761,1.1461,1.1161,1.0861,1.0561,1.0261,0.9961,0.9661,0.9361,0.9061,0.8761,0.8461,0.8161,0.7861,0.7561,0.7261,0.6961,0.6661,0.6361,0.6061,0.5761,0.5461,0.5161,0.4861,0.4561,0.4261,0.3961,0.3661,0.3361,0.3061,0.2761,0.2461,0.2161,0.1861,0.1561,0.1261,0.0961,0.0661,0.0361,0.0061,-0.0239,-0.0539,-0.0839,-0.0983,-0.099638,-0.12964,-0.15963,-0.18961,-0.21959,-0.24954,-0.27948,-0.3094,-0.3393,-0.36917,-0.399,-0.40317,-0.40933,-0.43902,-0.46868,-0.49832,-0.52793,-0.55752,-0.58707,-0.6166,-0.6461,-0.67557,-0.70501,-0.73441,-0.76378,-0.79312,-0.79962,-0.78785,-0.81742,-0.84682,-0.87603,-0.90503,-0.93378,-0.96227,-0.99046,-1.0183,-1.0459,-1.0484,-1.0735,-1.0987,-1.1205,-1.1413,-1.1608,-1.1791,-1.196,-1.2117,-1.2259,-1.2387,-1.2485,-1.2513,-1.2606,-1.2695,-1.2779,-1.2857,-1.2931,-1.2999,-1.3063,-1.3121,-1.3175,-1.3223,-1.3266,-1.3304,-1.3336,-1.336,-1.3356,-1.3386,-1.3414,-1.3439,-1.3463,-1.3484,-1.3503,-1.352,-1.3535,-1.3548,-1.3559,-1.3567,-1.3571,-1.3571,-1.3567,-1.3559,-1.3548,-1.3535,-1.352,-1.3503,-1.3484,-1.3463,-1.3439,-1.3414,-1.3386,-1.3356,-1.336,-1.3336,-1.3304,-1.3266,-1.3223,-1.3175,-1.3121,-1.3063,-1.2999,-1.2931,-1.2857,-1.2779,-1.2695,-1.2606,-1.2513,-1.2485,-1.2387,-1.2259,-1.2117,-1.196,-1.1791,-1.1608,-1.1413,-1.1205,-1.0987,-1.0735,-1.0484,-1.0459,-1.0183,-0.99046,-0.96227,-0.93378,-0.90503,-0.87603,-0.84682,-0.81742,-0.78785,-0.79962,-0.79312,-0.76378,-0.73441,-0.70501,-0.67557,-0.6461,-0.6166,-0.58707,-0.55752,-0.52793,-0.49832,-0.46868,-0.43902,-0.40933,-0.40317,-0.399,-0.36917,-0.3393,-0.3094,-0.27948,-0.24954,-0.21959,-0.18961,-0.15963,-0.12964,-0.099638,-0.0983,-0.0839,-0.0539,-0.0239,0.0061,0.0361,0.0661,0.0961,0.1261,0.1561,0.1861,0.2161,0.2461,0.2761,0.3061,0.3361,0.3661,0.3961,0.4261,0.4561,0.4861,0.5161,0.5461,0.5761,0.6061,0.6361,0.6661,0.6961,0.7261,0.7561,0.7861,0.8161,0.8461,0.8761,0.9061,0.9361,0.9661,0.9961,1.0261,1.0561,1.0861,1.1161,1.1461,1.1761,1.2061,1.2361,1.2661,1.2961,1.3261,1.3561,1.3861,1.4161,1.4461,1.4761,1.5061,1.5361,1.5661,1.5961,1.6261,1.6561,1.6861,1.7161,1.7461,1.7761,1.8061,1.8361,1.8661,1.8961,1.9261,1.9561,1.9861,2.0161,2.0461,2.0761,2.1061,2.1361,2.1661,2.1961,2.2261,2.2561,2.2861,2.3161,2.3461,2.3761,2.4061,2.4361,2.4661,2.4961,2.5261,2.5561,2.5861,2.6161,2.6461,2.6761,2.7061,2.7361,2.7661,2.7961,2.8261,2.8561,2.8861,2.9161,2.9461,2.9761,3.0061,3.0361,3.0661,3.0961,3.1261,3.1561,3.1861,3.2161,3.2461,3.2761,3.3061,3.3361,3.3442,3.363,3.393,3.4229,3.4529,3.4828,3.5127,3.5426,3.5725,3.6024,3.6322,3.6446,3.6684,3.6976,3.7264,3.7546,3.7824,3.8094,3.8358,3.8613,3.886,3.9098,3.9326,3.9542,3.9748,3.9942,4.0124,3.9992,4.017,4.0362,4.0547,4.0726,4.0899,4.1065,4.1225,4.1378,4.1525,4.1664,4.1797,4.1922,4.204,4.2152,4.2255,4.2352,4.2418,4.2435,4.2487,4.2537,4.2585,4.2631,4.2675,4.2717,4.2757,4.2795,4.2831,4.2856,4.2856,4.2856,4.2856,4.2856,4.2856,4.2856,4.2856};
double Rect_y[512] = { 0,0.03,0.06,0.09,0.12,0.15,0.18,0.1983,0.22178,0.25156,0.28132,0.31105,0.34075,0.37043,0.40007,0.42969,0.45927,0.48882,0.49845,0.52475,0.55316,0.5813,0.60917,0.63673,0.66398,0.69089,0.71745,0.74363,0.76943,0.79482,0.81978,0.8443,0.86836,0.89195,0.91505,0.93546,0.91909,0.94297,0.96585,0.98769,1.0084,1.0279,1.0462,1.0633,1.079,1.0933,1.1062,1.1177,1.1277,1.1362,1.1431,1.1476,1.1493,1.1522,1.155,1.1576,1.16,1.1622,1.1642,1.166,1.1676,1.169,1.1698,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.1704,1.17,1.1694,1.1684,1.1671,1.1656,1.1637,1.1615,1.159,1.1562,1.153,1.1526,1.1518,1.1475,1.143,1.1384,1.1335,1.1286,1.1234,1.1181,1.1127,1.1071,1.1013,1.0953,1.0892,1.0829,1.0815,1.0837,1.0786,1.0727,1.0659,1.0582,1.0496,1.0402,1.03,1.0189,1.0069,1.0058,0.98945,0.9731,0.95256,0.93088,0.90811,0.88433,0.85959,0.83399,0.80758,0.78044,0.75656,0.74883,0.72033,0.69167,0.66286,0.63391,0.60482,0.57562,0.5463,0.51687,0.48735,0.45774,0.42805,0.39829,0.36847,0.34349,0.34688,0.31703,0.28716,0.25727,0.22736,0.19744,0.1675,0.13755,0.10759,0.077612,0.047631,0.017644,0.0043543,-0.0043543,-0.017644,-0.047631,-0.077612,-0.10759,-0.13755,-0.1675,-0.19744,-0.22736,-0.25727,-0.28716,-0.31703,-0.34688,-0.34349,-0.36847,-0.39829,-0.42805,-0.45774,-0.48735,-0.51687,-0.5463,-0.57562,-0.60482,-0.63391,-0.66286,-0.69167,-0.72033,-0.74883,-0.75656,-0.78044,-0.80758,-0.83399,-0.85959,-0.88433,-0.90811,-0.93088,-0.95256,-0.9731,-0.98945,-1.0058,-1.0069,-1.0189,-1.03,-1.0402,-1.0496,-1.0582,-1.0659,-1.0727,-1.0786,-1.0837,-1.0815,-1.0829,-1.0892,-1.0953,-1.1013,-1.1071,-1.1127,-1.1181,-1.1234,-1.1286,-1.1335,-1.1384,-1.143,-1.1475,-1.1518,-1.1526,-1.153,-1.1562,-1.159,-1.1615,-1.1637,-1.1656,-1.1671,-1.1684,-1.1694,-1.17,-1.1704,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.1698,-1.169,-1.1676,-1.166,-1.1642,-1.1622,-1.16,-1.1576,-1.155,-1.1522,-1.1493,-1.1476,-1.1431,-1.1362,-1.1277,-1.1177,-1.1062,-1.0933,-1.079,-1.0633,-1.0462,-1.0279,-1.0084,-0.98769,-0.96585,-0.94297,-0.91909,-0.93546,-0.91505,-0.89195,-0.86836,-0.8443,-0.81978,-0.79482,-0.76943,-0.74363,-0.71745,-0.69089,-0.66398,-0.63673,-0.60917,-0.5813,-0.55316,-0.52475,-0.49845,-0.48882,-0.45927,-0.42969,-0.40007,-0.37043,-0.34075,-0.31105,-0.28132,-0.25156,-0.22178,-0.1983,-0.18,-0.15,-0.12,-0.09,-0.06,-0.03,-0};
// 20
double Rect_x10[512] = { 4.2856,4.2856,4.2856,4.2856,4.2856,4.2856,4.2856,4.2856,4.2831,4.2795,4.2757,4.2717,4.2675,4.2631,4.2585,4.2537,4.2487,4.2435,4.2418,4.2352,4.2255,4.2152,4.204,4.1922,4.1797,4.1664,4.1525,4.1378,4.1225,4.1065,4.0899,4.0726,4.0547,4.0362,4.017,3.9992,4.0124,3.9942,3.9748,3.9542,3.9326,3.9098,3.886,3.8613,3.8358,3.8094,3.7824,3.7546,3.7264,3.6976,3.6684,3.6446,3.6322,3.6024,3.5725,3.5426,3.5127,3.4828,3.4529,3.4229,3.393,3.363,3.3442,3.3361,3.3061,3.2761,3.2461,3.2161,3.1861,3.1561,3.1261,3.0961,3.0661,3.0361,3.0061,2.9761,2.9461,2.9161,2.8861,2.8561,2.8261,2.7961,2.7661,2.7361,2.7061,2.6761,2.6461,2.6161,2.5861,2.5561,2.5261,2.4961,2.4661,2.4361,2.4061,2.3761,2.3461,2.3161,2.2861,2.2561,2.2261,2.1961,2.1661,2.1361,2.1061,2.0761,2.0461,2.0161,1.9861,1.9561,1.9261,1.8961,1.8661,1.8361,1.8061,1.7761,1.7461,1.7161,1.6861,1.6561,1.6261,1.5961,1.5661,1.5361,1.5061,1.4761,1.4461,1.4161,1.3861,1.3561,1.3261,1.2961,1.2661,1.2361,1.2061,1.1761,1.1461,1.1161,1.0861,1.0561,1.0261,0.9961,0.9661,0.9361,0.9061,0.8761,0.8461,0.8161,0.7861,0.7561,0.7261,0.6961,0.6661,0.6361,0.6061,0.5761,0.5461,0.5161,0.4861,0.4561,0.4261,0.3961,0.3661,0.3361,0.3061,0.2761,0.2461,0.2161,0.1861,0.1561,0.1261,0.0961,0.0661,0.0361,0.0061,-0.0239,-0.0539,-0.0839,-0.0983,-0.099638,-0.12964,-0.15963,-0.18961,-0.21959,-0.24954,-0.27948,-0.3094,-0.3393,-0.36917,-0.399,-0.40317,-0.40933,-0.43902,-0.46868,-0.49832,-0.52793,-0.55752,-0.58707,-0.6166,-0.6461,-0.67557,-0.70501,-0.73441,-0.76378,-0.79312,-0.79962,-0.78785,-0.81742,-0.84682,-0.87603,-0.90503,-0.93378,-0.96227,-0.99046,-1.0183,-1.0459,-1.0484,-1.0735,-1.0987,-1.1205,-1.1413,-1.1608,-1.1791,-1.196,-1.2117,-1.2259,-1.2387,-1.2485,-1.2513,-1.2606,-1.2695,-1.2779,-1.2857,-1.2931,-1.2999,-1.3063,-1.3121,-1.3175,-1.3223,-1.3266,-1.3304,-1.3336,-1.336,-1.3356,-1.3386,-1.3414,-1.3439,-1.3463,-1.3484,-1.3503,-1.352,-1.3535,-1.3548,-1.3559,-1.3567,-1.3571,-1.3571,-1.3567,-1.3559,-1.3548,-1.3535,-1.352,-1.3503,-1.3484,-1.3463,-1.3439,-1.3414,-1.3386,-1.3356,-1.336,-1.3336,-1.3304,-1.3266,-1.3223,-1.3175,-1.3121,-1.3063,-1.2999,-1.2931,-1.2857,-1.2779,-1.2695,-1.2606,-1.2513,-1.2485,-1.2387,-1.2259,-1.2117,-1.196,-1.1791,-1.1608,-1.1413,-1.1205,-1.0987,-1.0735,-1.0484,-1.0459,-1.0183,-0.99046,-0.96227,-0.93378,-0.90503,-0.87603,-0.84682,-0.81742,-0.78785,-0.79962,-0.79312,-0.76378,-0.73441,-0.70501,-0.67557,-0.6461,-0.6166,-0.58707,-0.55752,-0.52793,-0.49832,-0.46868,-0.43902,-0.40933,-0.40317,-0.399,-0.36917,-0.3393,-0.3094,-0.27948,-0.24954,-0.21959,-0.18961,-0.15963,-0.12964,-0.099638,-0.0983,-0.0839,-0.0539,-0.0239,0.0061,0.0361,0.0661,0.0961,0.1261,0.1561,0.1861,0.2161,0.2461,0.2761,0.3061,0.3361,0.3661,0.3961,0.4261,0.4561,0.4861,0.5161,0.5461,0.5761,0.6061,0.6361,0.6661,0.6961,0.7261,0.7561,0.7861,0.8161,0.8461,0.8761,0.9061,0.9361,0.9661,0.9961,1.0261,1.0561,1.0861,1.1161,1.1461,1.1761,1.2061,1.2361,1.2661,1.2961,1.3261,1.3561,1.3861,1.4161,1.4461,1.4761,1.5061,1.5361,1.5661,1.5961,1.6261,1.6561,1.6861,1.7161,1.7461,1.7761,1.8061,1.8361,1.8661,1.8961,1.9261,1.9561,1.9861,2.0161,2.0461,2.0761,2.1061,2.1361,2.1661,2.1961,2.2261,2.2561,2.2861,2.3161,2.3461,2.3761,2.4061,2.4361,2.4661,2.4961,2.5261,2.5561,2.5861,2.6161,2.6461,2.6761,2.7061,2.7361,2.7661,2.7961,2.8261,2.8561,2.8861,2.9161,2.9461,2.9761,3.0061,3.0361,3.0661,3.0961,3.1261,3.1561,3.1861,3.2161,3.2461,3.2761,3.3061,3.3361,3.3442,3.363,3.393,3.4229,3.4529,3.4828,3.5127,3.5426,3.5725,3.6024,3.6322,3.6446,3.6684,3.6976,3.7264,3.7546,3.7824,3.8094,3.8358,3.8613,3.886,3.9098,3.9326,3.9542,3.9748,3.9942,4.0124,3.9992,4.017,4.0362,4.0547,4.0726,4.0899,4.1065,4.1225,4.1378,4.1525,4.1664,4.1797,4.1922,4.204,4.2152,4.2255,4.2352,4.2418,4.2435,4.2487,4.2537,4.2585,4.2631,4.2675,4.2717,4.2757,4.2795,4.2831,4.2856,4.2856,4.2856,4.2856,4.2856,4.2856,4.2856,4.2856};
double Rect_y10[512] = { 0,0.03,0.06,0.09,0.12,0.15,0.18,0.1983,0.22178,0.25156,0.28132,0.31105,0.34075,0.37043,0.40007,0.42969,0.45927,0.48882,0.49845,0.52475,0.55316,0.5813,0.60917,0.63673,0.66398,0.69089,0.71745,0.74363,0.76943,0.79482,0.81978,0.8443,0.86836,0.89195,0.91505,0.93546,0.91909,0.94297,0.96585,0.98769,1.0084,1.0279,1.0462,1.0633,1.079,1.0933,1.1062,1.1177,1.1277,1.1362,1.1431,1.1476,1.1493,1.1522,1.155,1.1576,1.16,1.1622,1.1642,1.166,1.1676,1.169,1.1698,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.17,1.1704,1.17,1.1694,1.1684,1.1671,1.1656,1.1637,1.1615,1.159,1.1562,1.153,1.1526,1.1518,1.1475,1.143,1.1384,1.1335,1.1286,1.1234,1.1181,1.1127,1.1071,1.1013,1.0953,1.0892,1.0829,1.0815,1.0837,1.0786,1.0727,1.0659,1.0582,1.0496,1.0402,1.03,1.0189,1.0069,1.0058,0.98945,0.9731,0.95256,0.93088,0.90811,0.88433,0.85959,0.83399,0.80758,0.78044,0.75656,0.74883,0.72033,0.69167,0.66286,0.63391,0.60482,0.57562,0.5463,0.51687,0.48735,0.45774,0.42805,0.39829,0.36847,0.34349,0.34688,0.31703,0.28716,0.25727,0.22736,0.19744,0.1675,0.13755,0.10759,0.077612,0.047631,0.017644,0.0043543,-0.0043543,-0.017644,-0.047631,-0.077612,-0.10759,-0.13755,-0.1675,-0.19744,-0.22736,-0.25727,-0.28716,-0.31703,-0.34688,-0.34349,-0.36847,-0.39829,-0.42805,-0.45774,-0.48735,-0.51687,-0.5463,-0.57562,-0.60482,-0.63391,-0.66286,-0.69167,-0.72033,-0.74883,-0.75656,-0.78044,-0.80758,-0.83399,-0.85959,-0.88433,-0.90811,-0.93088,-0.95256,-0.9731,-0.98945,-1.0058,-1.0069,-1.0189,-1.03,-1.0402,-1.0496,-1.0582,-1.0659,-1.0727,-1.0786,-1.0837,-1.0815,-1.0829,-1.0892,-1.0953,-1.1013,-1.1071,-1.1127,-1.1181,-1.1234,-1.1286,-1.1335,-1.1384,-1.143,-1.1475,-1.1518,-1.1526,-1.153,-1.1562,-1.159,-1.1615,-1.1637,-1.1656,-1.1671,-1.1684,-1.1694,-1.17,-1.1704,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.17,-1.1698,-1.169,-1.1676,-1.166,-1.1642,-1.1622,-1.16,-1.1576,-1.155,-1.1522,-1.1493,-1.1476,-1.1431,-1.1362,-1.1277,-1.1177,-1.1062,-1.0933,-1.079,-1.0633,-1.0462,-1.0279,-1.0084,-0.98769,-0.96585,-0.94297,-0.91909,-0.93546,-0.91505,-0.89195,-0.86836,-0.8443,-0.81978,-0.79482,-0.76943,-0.74363,-0.71745,-0.69089,-0.66398,-0.63673,-0.60917,-0.5813,-0.55316,-0.52475,-0.49845,-0.48882,-0.45927,-0.42969,-0.40007,-0.37043,-0.34075,-0.31105,-0.28132,-0.25156,-0.22178,-0.1983,-0.18,-0.15,-0.12,-0.09,-0.06,-0.03,-0};
int Rect_index10 = 512;
int Rect_index = 512;
///////////////////////////////////////
double nseconds = 0;

double g_px = 0, g_py = 0, g_pth = 0;

Node g_tnode;
Node g_tnode_two;  

int g_xidx = 0, g_yidx = 0, g_thidx = 0;
int CloseSizeMax = 0;

int flag_CCS = 0;
int flag_CCC = 0;
int flag_CSC = 0;
int flag_SCS = 0;
int flag_CCCC = 0;
int flag_CCSC = 0;
int flag_CCSCC = 0;

/* priority */
std::priority_queue<std::pair<std::string, double>, std::vector<std::pair<std::string, double>>, cmp> pq;
std::unordered_map<std::string, Node> g_openset, g_closeset;
std::string str_idx;

std::priority_queue<std::pair<std::string, double>, std::vector<std::pair<std::string, double>>, cmp> pq_two;
std::unordered_map<std::string, Node> g_openset_two, g_closeset_two;
std::string str_idx_two;

DynamicVoronoi voronoiDiagram;

std::vector<Node> g_open, g_close;
std::vector<path_point> pathpoint;
std::vector<path_point> pathpoint_two;
Path_config pathfind_parameters;

std::string get_str_idx(Node &node);
std::string get_par_str_idx(Node &node);
// map_set map_parameters;
Vehicle_config vehicle_parameters;

std::vector<double> find_steer_degree;

double TotalCost(Node wknode, double End[3]);
int inNodes(Node node, std::vector<Node>* nodes);
bool CalcIdx(double x, double y, double theta, double& xidx, double& yidx, double& thidx);
void Update_one(Node wknode, double End[3]);
void Update_two(Node wknode1, Node wknode2, double End[3], double End_two[3]);
void PopNode(std::vector<Node>* nodes);
bool CalcNextNode(Node wknode, double D, double delta);
RsPath AnalysticExpantion(Node wknode, double End[3]);
void getFinalPath_one(RsPath path, std::string stridx);
void getFinalPath_two(std::string stridx_two);
void getFinalPath_RS(RsPath path);
void End2End_HybridA(double s[3], double e[3]);

void get_voronoi();
///////类定义/////////////////

Node::Node()
{

}
Node::Node(int xidx1, int yidx1, int yawidx1, double D1, double delta1, double x1, double y1, double theta1, int  parent1[3], double  cost1, double fcost1)
{
	xidx = xidx1;
	yidx = yidx1;
	yawidx = yawidx1;
	D = D1;
	delta = delta1;
	x = x1;
	y = y1;
	theta = theta1;

	parent[0] = parent1[0];
	parent[1] = parent1[1];
	parent[2] = parent1[2];
	cost = cost1;
	fcost = fcost1;
}

////////////函数定义//////////////////////////////////////////////////
void get_voronoi()
{
    int width=250;
	int height=250;
	bool** binMap;//二维数组
	binMap = new bool*[width];
	for (int x = 0; x < width; x++) 
	{ 
		binMap[x] = new bool[height];
	}

	for (int x = 0; x < width; ++x) 
	{
		for (int y = 0; y < height; ++y) 
		{
			binMap[x][y] = (obstmap[y * width + x].Status==0) ? true : false;
    	}
	}//转化为二值地图
	voronoiDiagram.initializeMap(width, height, binMap);//注意这里传入到DynamicVoronoi里并进行保存，所以没有delete
	voronoiDiagram.update();
	voronoiDiagram.visualize();//将Voronoi Diagram初始化、更新并显示
}

double TotalCost(Node wknode, double End[3])
{
	// 将起点转换到原点计算轨迹，变换坐标系了
	double rmin = vehicle_parameters.MIN_CIRCLE;
	double x = End[0] - wknode.x;
	double y = End[1] - wknode.y;
	double ph = End[2] - wknode.theta;
	double phi = mod2pi(wknode.theta);
	// 起点start坐标系在基坐标系下的方向余弦矩阵(Z轴旋转，因为点在Z = 0平面，因此绕Z旋转仍然在Z = 0平面)
	// dcm*x 表示将基坐标中的x表示到旋转后的坐标系中，即计算坐标旋转后各向量在新坐标中的表示
	double x1 = x*cos(phi) + y*sin(phi);
	double y1 = y*cos(phi) - x*sin(phi);
	//看是否从当前点到目标点存在无碰撞的Reeds - Shepp轨迹，前面pvec = End - Start; 的意义就在这里，注意！这里的x, y, prev(3)是把起点转换成以start为原点坐标系下的坐标
	double cost = 0;
	#ifdef voronoi
	cost += 0.5*voronoiDiagram.getDistance(wknode.yidx, wknode.xidx);
	#endif
	
	if (fusion.parkingSpaceInfo.ParkingSpaceType == 2)
	{
		if ((Theta_se < 0.2793) && (Y_se > 1))
		{
			RsPath path;
			ErrorCode = FindRSPath(x1, y1, ph, path);
			if (ErrorCode == 0)
			{
				cost += pathfind_parameters.H_COST * rmin * path.lenth + wknode.cost;
			}
			else
			{
				LOG_WARNING("ErrorCodeTotalCost=%d", ErrorCode);
				cost += 1000;
			}
		}
		else
		{
			RsPath path = FindRSPath(x1, y1, ph);
			cost += pathfind_parameters.H_COST * rmin * path.lenth + wknode.cost;
		}
	}
	else
	{
		if (SEorES == 0)
		{
			RsPath path;
			ErrorCode = FindRSPath(x1, y1, ph, path);
			if (ErrorCode == 0)
			{
				cost += pathfind_parameters.H_COST * rmin * path.lenth + wknode.cost;
			}
			else
			{
				LOG_WARNING("ErrorCodeTotalCost=%d", ErrorCode);
				cost += 1000;
			}
		}
		else
		{
			RsPath path = FindRSPath(x1, y1, ph);
			cost += pathfind_parameters.H_COST * rmin * path.lenth + wknode.cost;
		}
	}

	return cost;
}

bool CalcIdx(double x, double y, double theta)
{
	float gres = pathfind_parameters.XY_GRID_RESOLUTION; //[m]世界坐标生成栅格坐标的分辨率（2m一个栅格）
	float yawres = pathfind_parameters.YAW_GRID_RESOLUTION; // 栅格角度分辨率
	g_xidx = round((x - pathfind_parameters.MINX) / gres); // 计算栅格索引
	g_yidx = round((y - pathfind_parameters.MINY) / gres); // 计算栅格索引
	theta = mod2pi(theta); // 控制theta范围在[-pi, pi]区间
	g_thidx = round((theta - pathfind_parameters.MINYAW) / (yawres / 180 * M_PI)); // 计算栅格索引
	double middle_x = (pathfind_parameters.MAXX - pathfind_parameters.MINX) / gres; // 地图边界栅格索引
	double middle_y = (pathfind_parameters.MAXY - pathfind_parameters.MINY) / gres; // 地图边界栅格索引
	if (g_xidx <= 0 || g_xidx > ceil(middle_x)) // 判断是否在地图内
	{
		return false;
	}
	else if (g_yidx <= 0 || g_yidx > ceil(middle_y)) // 判断是否在地图内
	{
		return false;
	}

	return true;
}

int inNodes(Node node, std::vector<Node>  *nodes) // 判断节点node是否在nodes里面
{
	int i, fan = -1;
	for (i = 0; i < nodes->size(); i++)
	{
		if (node.xidx == (*nodes)[i].xidx
			&& node.yidx == (*nodes)[i].yidx
			&& node.yawidx == (*nodes)[i].yawidx)
		{
			fan = i;
		}
	}
	return fan;
}

void VehicleDynamic(double x, double y, double theta, double D, double delta) //根据当前位姿和输入, 计算下一位置的位姿
{
	double mid_R;
	
	if (delta == 0)
	{
		g_px = x + D * cos(theta); // 运动学公式： x_dot = v_x * cos(theta); x_dot * t = v_x * t * cos(theta), 在采样时间t内, 则有x = x + v_x * t * cos(theta)，其中v_x * t = D
		g_py = y + D * sin(theta); // 运动学公式
		g_pth = theta + D / vehicle_parameters.WB * tan(delta); // L是轴距, 航向变化, theta_dot = v / R, R = L / tan(delta)
		g_pth = mod2pi(g_pth);
	}
	else
	{
		mid_R = vehicle_parameters.WB / tan(delta);
		g_px = x - mid_R * sin(theta) + mid_R * sin(theta + D / mid_R); // 运动学公式： x_dot = v_x * cos(theta); x_dot * t = v_x * t * cos(theta), 在采样时间t内, 则有x = x + v_x * t * cos(theta)，其中v_x * t = D
		g_py = y + mid_R * cos(theta) - mid_R * cos(theta + D / mid_R); // 运动学公式
		g_pth = theta + D / mid_R; // L是轴距, 航向变化, theta_dot = v / R, R = L / tan(delta)
		g_pth = mod2pi(g_pth);
	}
}

bool CalcNextNode(Node wknode, double D, double delta)//根据当前点及距离及方向盘角度计算下一点
{
	double cost = 0;
	int fa[3] = { wknode.xidx, wknode.yidx, wknode.yawidx };
	g_px = wknode.x;
	g_py = wknode.y;
	g_pth = wknode.theta;
	int i, nlist = 3;
	for (i = 0; i < nlist; i++)
	{
		VehicleDynamic(g_px, g_py, g_pth, D, delta);//更新全局变量的g_px、g_py、g_pth
		
		if (VehicleCollisionGrid(g_px, g_py, g_pth) == true)
		{
			return false;
		}
	}

	if (CalcIdx(g_px, g_py, g_pth) == false) // 把路径末端点的实际坐标转换为栅格坐标，（全局变量的g_xidx, g_yidx, g_thidx）

	{
		return false;
	}
	else
	{
		cost = wknode.cost;
	}

	if (D > 0) //前进
	{
		cost = cost + 0.3; // 每条轨迹大概是2米的长度。这里乘以1.5是确保下一个末端状态肯定在另一个栅格中，不会还在一个栅格中！在地图栅格中子结点拓展。比如对角线长度是1.4，此时还是在同一个栅格中
	}
	else // 后退
	{
		cost = cost + pathfind_parameters.BACK_COST*0.3;
	}

	if (D != wknode.D)
	{
		cost = cost + pathfind_parameters.SB_COST;
	}
	else if (delta != wknode.delta)
	{
//		cost = cost + pathfind_parameters.STEER_CHANGE_COST;
		cost = cost + pathfind_parameters.STEER_CHANGE_COST * fabs(delta - wknode.delta)/ vehicle_parameters.MAX_STEER;
	}
//	cost = cost + pathfind_parameters.STEER_COST*fabs(delta);
//	cost = cost + pathfind_parameters.STEER_CHANGE_COST*fabs(delta - wknode.delta);
	g_tnode = Node(g_xidx, g_yidx, g_thidx, D, delta, g_px, g_py, g_pth, fa, cost, 0); //tnode是路径的末端点，cost为到当前状态到此路径末端状态的成本

	return true;
}

bool VehicleCollisionGrid(double cpx, double cpy, double cph)
{   
	bool isCollision = 0;
	double phi = mod2pi(cph); // 构造旋转矩阵
	double cosphi = cos(-phi);
	double sinphi = sin(-phi);
	double rect_x;
	double rect_y;
	if (fusion.parkingSpaceInfo.ParkingSpaceType == 2)
	{
		double level_parking_space = 5.8 + front_D * 0.1 + back_D * 0.1;
		if (level_parking_space < 6.66)
		{
			Rect_index = Rect_index10;
		}
	}
	
	for (int i = 0; i < Rect_index; i++)
	{	
		if (i % 9 == 0)
		{
			if (Rect_index == Rect_index10)
			{
				rect_x = Rect_x10[i] * cosphi + Rect_y10[i] * sinphi + cpx;
				rect_y = Rect_y10[i] * cosphi - Rect_x10[i] * sinphi + cpy;
			}
			else
			{
				rect_x = Rect_x[i] * cosphi + Rect_y[i] * sinphi + cpx;
				rect_y = Rect_y[i] * cosphi - Rect_x[i] * sinphi + cpy;
			}
			
			//栅格化
			int xidx = ceil((pathfind_parameters.MAXX - rect_x) / pathfind_parameters.XY_GRID_RESOLUTION);
			int yidx = ceil((pathfind_parameters.MAXY - rect_y) / pathfind_parameters.XY_GRID_RESOLUTION);
			if (xidx == 0)
			{
				xidx = 1;
			}
			if (yidx == 0)
			{
				yidx = 1;
			}
			int idx_xy = (xidx - 1) * 250 + (yidx - 1);
			if ((xidx <= 0 || xidx > pathfind_parameters.XIDX) || (yidx <= 0 || yidx > pathfind_parameters.XIDY))
			{
				isCollision = true;
				return isCollision;
			}
			/*else if ((obstmap[idx_xy].Status == 0) || (obstmap[idx_xy].Status == 3) || (obstmap[idx_xy].Status == 4))
			{
				isCollision = true;
				return isCollision;
			}*/
			else 
			{	
				if (no_imag_map == 0)
				{
					if ((obstmap[idx_xy].Status == 0) || (obstmap[idx_xy].Status == 3) || (obstmap[idx_xy].Status == 4))
					{
						isCollision = true;
						return isCollision;
					}
				}
				else
				{
					if ((obstmap[idx_xy].Status == 0) || (obstmap[idx_xy].Status == 4))
					{
						isCollision = true;
						return isCollision;
					}
				}
			}
		}	
	}
	return isCollision;
}

void Update_one(Node wknode, double End[3]) //扩展点 ，并记录点，扩展close、OPEen集合
{
	int minidx = 0;                              //最小值下标
	char ilr = 0;                                   //左右转
	double sres = vehicle_parameters.MAX_STEER / pathfind_parameters.N_STEER;     //转向分辨率
	double delta = -vehicle_parameters.MAX_STEER;                 //右转最小转角
	double mres = pathfind_parameters.MOTION_RESOLUTION;           // motino resolution    运动分辨率

	// all possible control input，所有可能的控制输入

	for (ilr = 0; ilr < find_steer_degree.size(); ilr++)
	{
		delta = find_steer_degree[ilr];
		if (CalcNextNode(wknode, -mres, delta) == false)  //输出全局变量的g_tnode，fcost=0；
		{
			continue;
		}
		str_idx = get_str_idx(g_tnode);
		if (g_closeset.find(str_idx) != g_closeset.end())  // exist
			continue;
		
		g_tnode.fcost = TotalCost(g_tnode, End);
		/*if (Gears == 2)
		{
			g_tnode.fcost = g_tnode.fcost + 10;
		}*/
		if (g_openset.find(str_idx) != g_openset.end())
		{
			if (g_openset[str_idx].fcost > g_tnode.fcost)
			{
				g_openset[str_idx]=g_tnode;
				pq.push({str_idx, g_tnode.fcost});
			}
		}
		else
		{
			g_openset[str_idx]=g_tnode;
			pq.push({str_idx, g_tnode.fcost});
		}
	}

	for (ilr = 0; ilr < find_steer_degree.size(); ilr++)
	{
		delta = find_steer_degree[ilr];
		if (CalcNextNode(wknode, mres, delta) == false)  //输出全局变量的g_tnode，fcost=0；
		{
			continue;
		}
		str_idx = get_str_idx(g_tnode);
		if (g_closeset.find(str_idx) != g_closeset.end())  // exist
			continue;
		
		g_tnode.fcost = TotalCost(g_tnode, End);
		/*if (Gears == 1)
		{
			g_tnode.fcost = g_tnode.fcost + 10;
		}*/

		if (g_openset.find(str_idx) != g_openset.end())
		{
			if (g_openset[str_idx].fcost > g_tnode.fcost)
			{
				g_openset[str_idx]=g_tnode;
				pq.push({str_idx, g_tnode.fcost});
			}
		}
		else
		{
			g_openset[str_idx]=g_tnode;
			pq.push({str_idx, g_tnode.fcost});
		}
	}
}

void Update_two(Node wknode1, Node wknode2, double End[3], double End_two[3]) //扩展点 ，并记录点，扩展close、OPEen集合
{
	int minidx = 0;                              //最小值下标
	char ilr = 0;                                   //左右转
	double sres = vehicle_parameters.MAX_STEER / pathfind_parameters.N_STEER;     //转向分辨率
	double delta = -vehicle_parameters.MAX_STEER;                 //右转最小转角
	double mres = pathfind_parameters.MOTION_RESOLUTION;           // motino resolution    运动分辨率

	// all possible control input，所有可能的控制输入

	for (ilr = 0; ilr < find_steer_degree.size(); ilr++)
	{
		delta = find_steer_degree[ilr];
		if (CalcNextNode(wknode1, -mres, delta) == false)  //输出全局变量的g_tnode，fcost=0；
		{
			continue;
		}
		str_idx = get_str_idx(g_tnode);
		if (g_closeset.find(str_idx) != g_closeset.end())  // exist
			continue;
		
		g_tnode.fcost = TotalCost(g_tnode, End);
		/*if (Gears == 2)
		{
			g_tnode.fcost = g_tnode.fcost + 10;
		}*/
		if (g_openset.find(str_idx) != g_openset.end())
		{
			if (g_openset[str_idx].fcost > g_tnode.fcost)
			{
				g_openset[str_idx]=g_tnode;
				pq.push({str_idx, g_tnode.fcost});
			}
		}
		else
		{
			g_openset[str_idx]=g_tnode;
			pq.push({str_idx, g_tnode.fcost});
		}
	}

	for (ilr = 0; ilr < find_steer_degree.size(); ilr++)
	{
		delta = find_steer_degree[ilr];
		if (CalcNextNode(wknode1, mres, delta) == false)  //输出全局变量的g_tnode，fcost=0；
		{
			continue;
		}
		str_idx = get_str_idx(g_tnode);
		if (g_closeset.find(str_idx) != g_closeset.end())  // exist
			continue;
		
		g_tnode.fcost = TotalCost(g_tnode, End);
		/*if (Gears == 1)
		{
			g_tnode.fcost = g_tnode.fcost + 10;
		}*/

		if (g_openset.find(str_idx) != g_openset.end())
		{
			if (g_openset[str_idx].fcost > g_tnode.fcost)
			{
				g_openset[str_idx]=g_tnode;
				pq.push({str_idx, g_tnode.fcost});
			}
		}
		else
		{
			g_openset[str_idx]=g_tnode;
			pq.push({str_idx, g_tnode.fcost});
		}
	}

	for (ilr = 0; ilr < find_steer_degree.size(); ilr++)
	{
		delta = find_steer_degree[ilr];
		if (CalcNextNode(wknode2, -mres, delta) == false)  //输出全局变量的g_tnode，fcost=0；
		{
			continue;
		}
		str_idx = get_str_idx(g_tnode);
		if (g_closeset_two.find(str_idx) != g_closeset_two.end())  // exist
			continue;
		
		g_tnode.fcost = TotalCost(g_tnode, End_two);
		/*if (Gears == 2)
		{
			g_tnode.fcost = g_tnode.fcost + 10;
		}*/
		if (g_openset_two.find(str_idx) != g_openset_two.end())
		{
			if (g_openset_two[str_idx].fcost > g_tnode.fcost)
			{
				g_openset_two[str_idx]=g_tnode;
				pq_two.push({str_idx, g_tnode.fcost});
			}
		}
		else
		{
			g_openset_two[str_idx]=g_tnode;
			pq_two.push({str_idx, g_tnode.fcost});
		}
	}

	for (ilr = 0; ilr < find_steer_degree.size(); ilr++)
	{
		delta = find_steer_degree[ilr];
		if (CalcNextNode(wknode2, mres, delta) == false)  //输出全局变量的g_tnode，fcost=0；
		{
			continue;
		}
		str_idx = get_str_idx(g_tnode);
		if (g_closeset_two.find(str_idx) != g_closeset_two.end())  // exist
			continue;
		
		g_tnode.fcost = TotalCost(g_tnode, End_two);
		/*if (Gears == 2)
		{
			g_tnode.fcost = g_tnode.fcost + 10;
		}*/
		if (g_openset_two.find(str_idx) != g_openset_two.end())
		{
			if (g_openset_two[str_idx].fcost > g_tnode.fcost)
			{
				g_openset_two[str_idx]=g_tnode;
				pq_two.push({str_idx, g_tnode.fcost});
			}
		}
		else
		{
			g_openset_two[str_idx]=g_tnode;
			pq_two.push({str_idx, g_tnode.fcost});
		}
	}
}

RsPath AnalysticExpantion(Node wknode, double End[3]) // 参数节点的RS曲线是否发生碰撞
{
	// printf("RSstartpoint_End = %lf,%lf,%lf\n",End[0],End[1],End[2]);
	// printf("RSendpoint_wknode = %lf,%lf,%lf\n",wknode.x,wknode.y,wknode.theta);
	bool isCollision = false;
	// 将起点转换到原点计算轨迹，变换坐标系了
	double rmin = vehicle_parameters.MIN_CIRCLE;
	double x = End[0] - wknode.x;
	double y = End[1] - wknode.y;
	double ph = End[2] - wknode.theta;
	double phi = mod2pi(wknode.theta);
	// 起点start坐标系在基坐标系下的方向余弦矩阵(Z轴旋转，因为点在Z = 0平面，因此绕Z旋转仍然在Z = 0平面)
	// dcm*x 表示将基坐标中的x表示到旋转后的坐标系中，即计算坐标旋转后各向量在新坐标中的表示
	double x1 = x * cos(phi) + y * sin(phi);
	double y1 = y * cos(phi) - x * sin(phi);
	// 看是否从当前点到目标点存在无碰撞的Reeds - Shepp轨迹，前面pvec = End - Start; 的意义就在这里，注意！这里的x, y, prev(3)是把起点转换成以start为原点坐标系下的坐标
	RsPath path;

	if (fusion.parkingSpaceInfo.ParkingSpaceType == 2)
	{
		if ((Theta_se < 0.2793) && (Y_se > 1))
		{
			ErrorCode = FindRSPath(x1, y1, ph, path);
		}
		else
		{
			path = FindRSPath(x1, y1, ph);
		}
	}
	else
	{
		if (SEorES == 0)
		{
			ErrorCode = FindRSPath(x1, y1, ph, path);
		}
		else
		{
			path = FindRSPath(x1, y1, ph);
		}
	}
	// printf("path = %lf,%lf,%lf,%lf\n",rmin * path.t,rmin * path.u,rmin * path.v,path.lenth);
	if (((ErrorCode == 2) && (SEorES == 0)) || (path.lenth > 999))
	{
		path.lenth = 0;
	}
	else
	{
		// 给全局变量赋初值
		g_px = wknode.x;
		g_py = wknode.y;
		g_pth = wknode.theta;

		// 以下是根据路径点和车辆运动学模型计算位置，检测是否会产生碰撞，返回isok的值。对每段路径从起点到终点按顺序进行处理，这一个线段的终点pvec是下一个线段的起点px, py, pth，

		double segs[5] = {rmin * path.pathlength[0], rmin * path.pathlength[1], rmin * path.pathlength[2],
		rmin * path.pathlength[3], rmin * path.pathlength[4]};
		int i = 0;
		double D = 0;
		double delta = 0;
		int idx = 0;
		int num = path.pathtype.size();
		for (i = 0; i < num; i++)
		{
			if (segs[i] == 0)
			{
				continue;
			}

			D = (segs[i] > 0) ? pathfind_parameters.MOTION_RESOLUTION : -pathfind_parameters.MOTION_RESOLUTION;
			
			if (path.pathtype[i] == 'S')
			{
				delta = 0;
			}

			else if (path.pathtype[i] == 'L')
			{
				delta = vehicle_parameters.MAX_STEER;
			}

			else if (path.pathtype[i] == 'R')
			{
				delta = -vehicle_parameters.MAX_STEER;
			}

			// 把此段的路径离散成为路点，即栅格索引, 然后为路点，然后检测是否存在障碍物碰撞问题
			for (idx = 0; idx < round(fabs(segs[i]) / pathfind_parameters.MOTION_RESOLUTION); idx++) // round()四舍五入
																									 // D和delta是固定，说明转弯的时候是按固定半径的圆转弯
			{
				VehicleDynamic(g_px, g_py, g_pth, D, delta); // 根据当前位姿和输入, 计算下一位置的位姿
				isCollision = VehicleCollisionGrid(g_px, g_py, g_pth);
				if (isCollision)
				{
					path.lenth = 0;
					break;
				}
			}
			if (isCollision)
			{
				path.lenth = 0;
				break;
			}
		}
		// 终点位姿小于期望阈值也舍弃
		if (fabs(mod2pi(g_pth) - End[2]) > M_PI / 180 * 3)
		{
			path.lenth = 0;
		}
	}
	// printf("path = %lf,%lf,%lf,%lf\n",rmin * path.t,rmin * path.u,rmin * path.v,rmin * path.lenth);
	return path;
}

void getFinalPath_RS(RsPath path)
{
	int i = 1, n = 0, idx = 0; // 临时循环变量，i=1是确保第一个循环可以走下去
	std::vector<Node> nodes;

	path_point mid;
	mid.x = pathpoint[pathpoint.size() - 1].x;
	mid.y = pathpoint[pathpoint.size() - 1].y;
	mid.th = pathpoint[pathpoint.size() - 1].th;
	mid.D = pathpoint[pathpoint.size() - 1].D;
	mid.delta = pathpoint[pathpoint.size() - 1].delta;

	double rmin = vehicle_parameters.MIN_CIRCLE;
	double segs[5] = {rmin * path.pathlength[0], rmin * path.pathlength[1], rmin * path.pathlength[2],
		rmin * path.pathlength[3], rmin * path.pathlength[4]};
	double D = 0;
	double delta = 0;
	int num = path.pathtype.size();
	for (i = 0; i < num; i++)
	{
		if (segs[i] == 0)
		{
			continue;
		}

		D = (segs[i] > 0) ? pathfind_parameters.MOTION_RESOLUTION : -pathfind_parameters.MOTION_RESOLUTION;

		if (path.pathtype[i] == 'S')
		{
			delta = 0;
		}

		else if (path.pathtype[i] == 'L')
		{
			delta = vehicle_parameters.MAX_STEER;
		}

		else if (path.pathtype[i] == 'R')
		{
			delta = -vehicle_parameters.MAX_STEER;
		}

		// 把此段的路径离散成为路点，即栅格索引, 然后为路点，然后检测是否存在障碍物碰撞问题
		for (idx = 0; idx < round(fabs(segs[i]) / pathfind_parameters.MOTION_RESOLUTION); idx++) // round()四舍五入
																								 // D和delta是固定，说明转弯的时候是按固定半径的圆转弯
		{
			VehicleDynamic(mid.x, mid.y, mid.th, D, delta); // 根据当前位姿和输入, 计算下一位置的位姿
			mid.x = g_px;
			mid.y = g_py;
			mid.th = g_pth;
			mid.D = D;
			mid.delta = delta;
			pathpoint.push_back(mid);
		}
	}
}

void getFinalPath_one(RsPath path, std::string stridx)
{
	int i = 1, n = 0, idx = 0; // 临时循环变量，i=1是确保第一个循环可以走下去
	path_point mid;
	Node wknode = g_closeset[stridx]; // RS曲线中最后一个元素是目标点
	std::vector<Node> nodes;
	nodes.push_back(wknode);
	double rmin = vehicle_parameters.MIN_CIRCLE;
	// 找目标点wknode的parent, 回溯，直到Close集合为空
	while (wknode.xidx != wknode.parent[0] || wknode.yidx != wknode.parent[1] || wknode.yawidx != wknode.parent[2]) // 如果父节点不是自己才继续查找
	{
		stridx = get_par_str_idx(wknode);
		wknode = g_closeset[stridx];
		nodes.push_back(wknode);
		g_closeset.erase(stridx);
	}

	mid.x = nodes[nodes.size() - 1].x;
	mid.y = nodes[nodes.size() - 1].y;
	mid.th = nodes[nodes.size() - 1].theta;
	mid.D = nodes[nodes.size() - 1].D;
	mid.delta = nodes[nodes.size() - 1].delta;
	pathpoint.push_back(mid); // 先把起点的坐标放到路径中

	char nlist = 3;

	// 路径要么是纯RS路径，要么是由RS路径和混合A*组合一起来的路径，先处理混合A*的结点，最后处理RS路径，肯定有RS路径
	if (nodes.size() >= 2)
	{
		for (i = nodes.size() - 1; i >= 1; i--)
		{
			for (idx = 0; idx < 3; idx++)
			{
				VehicleDynamic(mid.x, mid.y, mid.th, nodes[i - 1].D, nodes[i - 1].delta);
				mid.x = g_px;
				mid.y = g_py;
				mid.th = g_pth;
				mid.D = nodes[i - 1].D;
				mid.delta = nodes[i - 1].delta;
				pathpoint.push_back(mid);
			}
		}
	}

	double segs[5] = {rmin * path.pathlength[0], rmin * path.pathlength[1], rmin * path.pathlength[2],
		rmin * path.pathlength[3], rmin * path.pathlength[4]};
	double D = 0;
	double delta = 0;
	int num = path.pathtype.size();
	for (i = 0; i < num; i++)
	{
		if (segs[i] == 0)
		{
			continue;
		}

		D = (segs[i] > 0) ? pathfind_parameters.MOTION_RESOLUTION : -pathfind_parameters.MOTION_RESOLUTION;

		if (path.pathtype[i] == 'S')
		{
			delta = 0;
		}

		else if (path.pathtype[i] == 'L')
		{
			delta = vehicle_parameters.MAX_STEER;
		}

		else if (path.pathtype[i] == 'R')
		{
			delta = -vehicle_parameters.MAX_STEER;
		}

		// 把此段的路径离散成为路点，即栅格索引, 然后为路点，然后检测是否存在障碍物碰撞问题
		for (idx = 0; idx < round(fabs(segs[i]) / pathfind_parameters.MOTION_RESOLUTION); idx++) // round()四舍五入
																								 // D和delta是固定，说明转弯的时候是按固定半径的圆转弯
		{
			VehicleDynamic(mid.x, mid.y, mid.th, D, delta); // 根据当前位姿和输入, 计算下一位置的位姿
			mid.x = g_px;
			mid.y = g_py;
			mid.th = g_pth;
			mid.D = D;
			mid.delta = delta;
			pathpoint.push_back(mid);
		}
	}
}

void getFinalPath_two(std::string stridx_two)
{
	int i = 1, n = 0, idx = 0; // 临时循环变量，i=1是确保第一个循环可以走下去
	path_point mid;
	Node wknode = g_closeset_two[stridx_two]; // RS曲线中最后一个元素是目标点
	std::vector<Node> nodes;
	nodes.push_back(wknode);
	double rmin = vehicle_parameters.MIN_CIRCLE;
	// 找目标点wknode的parent, 回溯，直到Close集合为空
	while (wknode.xidx != wknode.parent[0] || wknode.yidx != wknode.parent[1] || wknode.yawidx != wknode.parent[2]) // 如果父节点不是自己才继续查找
	{
		stridx_two = get_par_str_idx(wknode);
		wknode = g_closeset_two[stridx_two];
		nodes.push_back(wknode);
		g_closeset_two.erase(stridx_two);
	}

	mid.x = nodes[nodes.size() - 1].x;
	mid.y = nodes[nodes.size() - 1].y;
	mid.th = nodes[nodes.size() - 1].theta;
	mid.D = nodes[nodes.size() - 1].D;
	mid.delta = nodes[nodes.size() - 1].delta;
	pathpoint_two.push_back(mid); // 先把起点的坐标放到路径中

	char nlist = 3;

	// 路径要么是纯RS路径，要么是由RS路径和混合A*组合一起来的路径，先处理混合A*的结点，最后处理RS路径，肯定有RS路径
	if (nodes.size() >= 2)
	{
		for (i = nodes.size() - 1; i >= 1; i--)
		{
			for (idx = 0; idx < 3; idx++)
			{
				VehicleDynamic(mid.x, mid.y, mid.th, nodes[i - 1].D, nodes[i - 1].delta);
				mid.x = g_px;
				mid.y = g_py;
				mid.th = g_pth;
				mid.D = nodes[i - 1].D;
				mid.delta = nodes[i - 1].delta;
				pathpoint_two.push_back(mid);
			}
		}
	}
	std::reverse(pathpoint_two.begin(), pathpoint_two.end());
	pathpoint.insert(pathpoint.end(), pathpoint_two.begin(), pathpoint_two.end());
}

std::string get_str_idx(Node &node)
{
	std::string str = "xy_yaw"+std::to_string(node.xidx)+"_"+std::to_string(node.yidx)+"_"+std::to_string(node.yawidx);
	return str;
}

std::string get_par_str_idx(Node &node)
{
	std::string str = "xy_yaw"+std::to_string(node.parent[0])+"_"+std::to_string(node.parent[1])+"_"+std::to_string(node.parent[2]);
	return str;
}

void End2End_HybridA(double s[3], double e[3])
{	
	Node tnode;
	if (CalcIdx(s[0], s[1], s[2]))
	{	
		int mid[3]={g_xidx, g_yidx, g_thidx};
		Node node = Node(g_xidx, g_yidx, g_thidx, 0, 0, s[0], s[1], s[2], mid, 0, 0);
		node.fcost = TotalCost(node, e); // 起始点f
		
		g_openset.clear();    // open 集合清空
		g_closeset.clear();   // close 集合清空
		while (!pq.empty())
			pq.pop();

		str_idx = get_str_idx(node); // hybrid A*的Open集合
		pq.push({str_idx, node.fcost});
		g_openset[str_idx]=node;

		while (!g_openset.empty())
		{
			while (1)
			{
				str_idx = pq.top().first; // 选出代价最小的OPEN节点
				if (g_openset.find(str_idx) == g_openset.end())
					pq.pop();
				else
					break;	
			}

			tnode = g_openset[str_idx];
			pq.pop();
			g_openset.erase(str_idx);
			g_closeset[str_idx]=tnode;

			RsPath path = AnalysticExpantion(tnode, e);

			if (path.lenth)
			{
				getFinalPath_one(path, str_idx);
				return;
			}
			Update_one(tnode, End);
		}
	}
}

bool HybridAStar(double Start1[3], double End1[3])
{	
	#ifdef voronoi
	get_voronoi();
	#endif

	double End[3];
	double Start[3];
	double Start_two[3];
    double End_two[3];
	double temp[3];

	double time_max = 0;
	printf("SEorES = %d\n",SEorES);
	Rect_index10 = 512;
	Rect_index = 512;

	time_max = 8000;

	Start[0] = Start1[0];
	Start[1] = Start1[1];
	Start[2] = Start1[2];
	End[0] = End1[0];
	End[1] = End1[1];
	End[2] = End1[2];

	Start_two[0] = End1[0];
	Start_two[1] = End1[1];
	Start_two[2] = End1[2];
	End_two[0] = Start1[0];
	End_two[1] = Start1[1];
	End_two[2] = Start1[2];

	//printf("Start = %lf,%lf,%lf\n",Start[0],Start[1],Start[2]);
	//printf("End = %lf,%lf,%lf\n",End[0],End[1],End[2]);
	LOG_WARNING("Start=%lf,%lf,%lf", Start[0],Start[1],Start[2]);
	LOG_WARNING("End=%lf,%lf,%lf", End[0],End[1],End[2]);
	fflush(bydapa::common::Log::GetInstance()->fileptr());
	Theta_se = fabs(Start[2] - End[2]);
	Y_se = fabs(Start[1] - End[1]);
	//printf("Theta_se = %lf\n",Theta_se);
	//printf("Y_se = %lf\n",Y_se);

	RsPath path; //函数临时变量
	pathpoint.clear(); // 路 集合清空

	Node tnode;//函数临时变量
    g_openset.clear();    // open 集合清空
	g_closeset.clear();   // close 集合清空
	while (!pq.empty())
		pq.pop();

	Node tnode_two;//函数临时变量
	g_openset_two.clear();    // open 集合清空
	g_closeset_two.clear();   // close 集合清空
	while (!pq_two.empty())
		pq_two.pop();

	//////// 起点和终点发生碰撞返回“0”////////
	bool isCollision_Start = VehicleCollisionGrid(Start[0], Start[1], Start[2]);
	bool isCollision_End = VehicleCollisionGrid(End[0], End[1], End[2]);
	if ((isCollision_Start) || (isCollision_End))
	{
		nseconds = 0;
	}
	///////////////////////////////////////////
	else if (CalcIdx(Start[0], Start[1], Start[2]))
	{
		int mid[3] = { g_xidx, g_yidx, g_thidx };// 函数临时变量
		tnode = Node(g_xidx, g_yidx, g_thidx, 0, 0, Start[0], Start[1], Start[2], mid, 0, 0);
		tnode.fcost = TotalCost(tnode, End); // 起始点f
		
		str_idx = get_str_idx(tnode); // hybrid A*的Open集合
		pq.push({str_idx, tnode.fcost});
		g_openset[str_idx]=tnode;

		if (CalcIdx(Start_two[0], Start_two[1], Start_two[2]))
		{
			int mid_two[3] = { g_xidx, g_yidx, g_thidx };// 函数临时变量
			tnode_two = Node(g_xidx, g_yidx, g_thidx, 0, 0, Start_two[0], Start_two[1], Start_two[2], mid_two, 0, 0);
			tnode_two.fcost = TotalCost(tnode_two, End_two); // 起始点f
			
			str_idx_two = get_str_idx(tnode_two); // hybrid A*的Open集合
			pq_two.push({str_idx_two, tnode_two.fcost});
			g_openset_two[str_idx_two]=tnode_two;
		}

		CloseSizeMax = 0;
		bydapa::common::TicToc tictoc;
		tictoc.tic();
		
		while (!g_openset.empty() && !g_openset_two.empty())
		{
			nseconds = tictoc.toc();
			nseconds = nseconds * 0.000000001;
			
			if (nseconds > time_max)
			{
				ErrorCode = 3;
				LOG_WARNING("ErrorCodenseconds=%d", ErrorCode);
				fflush(bydapa::common::Log::GetInstance()->fileptr());
				return false;
			}

			while (1)
			{
				str_idx = pq.top().first; // 选出代价最小的OPEN节点
				if (g_openset.find(str_idx) == g_openset.end())
					pq.pop();
				else
					break;	
			}

			while (1)
			{
				str_idx_two = pq_two.top().first; // 选出代价最小的OPEN节点
				if (g_openset_two.find(str_idx_two) == g_openset_two.end())
					pq_two.pop();
				else
					break;	
			}
			
			g_tnode = g_openset[str_idx];
			pq.pop();
			g_openset.erase(str_idx);
			g_closeset[str_idx]=g_tnode;

			g_tnode_two = g_openset_two[str_idx_two];
			pq_two.pop();
			g_openset_two.erase(str_idx_two);
			g_closeset_two[str_idx_two]=g_tnode_two;

			temp[0]=g_tnode_two.x;
			temp[1]=g_tnode_two.y;
			temp[2]=g_tnode_two.theta;
			
			path = AnalysticExpantion(g_tnode, temp); // 代价最小的OPEN节点的RS是否发生碰撞，如果碰撞则下一点，不碰撞则搜路结束
			if (path.lenth)
			{	
				double temp2[3];
				temp2[0]=g_tnode.x;
				temp2[1]=g_tnode.y;
				temp2[2]=g_tnode.theta;

				// 重新对第一段曲线使用HA
				End2End_HybridA(Start1, temp2);

				// 获取第二段RS曲线
				getFinalPath_RS(path);

				// std::ofstream outfile("output.txt");
				// // 检查文件是否成功打开
				// if (outfile.is_open()) {
				// 	// 将每个 path_point 结构体的数据写入文件
				// 	for (const auto& point : pathpoint) {
				// 		outfile << point.x << " " << point.y << " " << point.th << " " << point.D << " " << point.delta << std::endl;
				// 	}
				// 	// 关闭文件流
				// 	outfile.close();
				// 	std::cout << "Data written to output.txt" << std::endl;
				// } else {
				// 	std::cerr << "Unable to open the file." << std::endl;
				// }
				#ifdef OBCA
				std::vector<std::vector<std::pair<double, double>>> obst;
				std::vector<std::pair<double, double>> tmp= {{-12.5, 12.5}, {-12.5, 4.95}, {12.5, 4.95}, {12.5, 12.5}};
				obst.push_back(tmp);
				tmp= {{-12.5, 0.15}, {-12.5, -5.0}, {-2.45, -5.0}, {-2.45, 0.15}};
				obst.push_back(tmp);
				tmp= {{1.35, 0.15}, {1.35, -5.55}, {9.05, -5.55}, {9.05, 0.15}};
				obst.push_back(tmp);
				// tmp= {{11.85, 5.0}, {11.85, -5.5}, {12.5, -5.5}, {12.5, 5.0}};
				// obst.push_back(tmp);

				// std::vector<path_point> pt(pathpoint.begin(), pathpoint.begin() + 10);

				// std::ofstream outfile("obst.txt");

				// // 检查文件是否成功打开
				// if (outfile.is_open()) {
				// 	// 将每个 std::vector<std::pair<double, double>> 写入文件
				// 	for (const auto& row : obst) {
				// 		for (const auto& pair : row) {
				// 			outfile << pair.first << " " << pair.second << " ";
				// 		}
				// 		outfile << std::endl;
				// 	}

				// 	// 关闭文件流
				// 	outfile.close();
				// 	std::cout << "Data written to obst.txt" << std::endl;
				// } else {
				// 	std::cerr << "Unable to open the file." << std::endl;
				// }

				Smoother smo(vehicle_parameters,pathpoint,obst,temp);
				smo.define_parameter_and_variable();
				smo.get_warm_start_dual_variables();
				smo.generate_constrain();
				smo.generate_variable();
				smo.generate_object();
				pathpoint = smo.solve();
				#endif

				// 直接取第三段曲线作为最终结果
				getFinalPath_two(str_idx_two);
				
				// 重新对第三段曲线使用HA，以RS终点为起点
				// temp[0]=pathpoint[pathpoint.size() - 1].x;
				// temp[1]=pathpoint[pathpoint.size() - 1].y;
				// temp[2]=pathpoint[pathpoint.size() - 1].th;
				// End2End_HybridA(temp, End);
				
				// pathpoint.erase(pathpoint.begin(), pathpoint.begin() + pathpoint.size() - 300);
				if (aaa == 1)
				{
					LOG_WARNING("Start=%lf,%lf,%lf", Start[0], Start[1], Start[2]);
					LOG_WARNING("End=%lf,%lf,%lf", End[0], End[1], End[2]);
					fflush(bydapa::common::Log::GetInstance()->fileptr());
				}
				nseconds = tictoc.toc();
				nseconds = nseconds * 0.000000001;
				return true; 
			}

			Update_two(g_tnode, g_tnode_two, End, End_two);

			if (CloseSizeMax < (int)g_closeset.size())
			{
				CloseSizeMax = (int)g_closeset.size();
			}
		}
	}
	else
	{
		ErrorCode = 1;
		LOG_ERROR("ErrorCodeCalcIdxStart=%d", ErrorCode);  // 起始点不在地图范围内
	}
	g_openset.clear();	 // open集合清空
	g_closeset.clear(); // close 集合清空
	g_openset_two.clear();	 // open集合清空
	g_closeset_two.clear(); // close 集合清空
	return false;	 // 搜不到路
}

//////////////////////////RS函数定义////////////////////////////////////////////////////////////////////////
RsPath FindRSPath(double x, double y, double phi)
{
	RsPath path;
	double rmin = vehicle_parameters.MIN_CIRCLE;
	path.lenth = 1000;
	x = x / rmin;
	y = y / rmin;
	// CSC(x, y, phi, path);
	CCC(x, y, phi, path);
	// CCS(x, y, phi, path);

	return path;
}

int FindRSPath(double x, double y, double phi, RsPath &path)
{
	//	RsPath path;
	double rmin = vehicle_parameters.MIN_CIRCLE;
	path.lenth = 1000;
	x = x / rmin;
	y = y / rmin;

	int cnt = 0;
	double tmp = path.lenth;
	CCS(x, y, phi, path);
	if (path.lenth < tmp)
	{
		tmp = path.lenth;
		cnt = 1;
	}
	// CCC(x, y, phi, path);
	// if (path.lenth < tmp)
	// {
	// 	tmp = path.lenth;
	// 	cnt = 2;
	// }
	// CSC(x, y, phi, path);
	// if (path.lenth < tmp)
	// {
	// 	tmp = path.lenth;
	// 	cnt = 3;
	// }
	// SCS(x, y, phi, path);
	// if (path.lenth < tmp)
	// {
	// 	tmp = path.lenth;
	// 	cnt = 4;
	// }
	// CCCC(x, y, phi, path);
	// if (path.lenth < tmp)
	// {
	// 	tmp = path.lenth;
	// 	cnt = 5;
	// }
	// CCSC(x, y, phi, path);
	// if (path.lenth < tmp)
	// {
	// 	tmp = path.lenth;
	// 	cnt = 6;
	// }
	// CCSCC(x, y, phi, path);
	// if (path.lenth < tmp)
	// {
	// 	tmp = path.lenth;
	// 	cnt = 7;
	// }

	switch (cnt)
	{
	case 1:
		flag_CCS += 1;
		break;
	case 2:
		flag_CCC += 1;
		break;
	case 3:
		flag_CSC += 1;
		break;
	case 4:
		flag_SCS += 1;
		break;
	case 5:
		flag_CCCC += 1;
		break;
	case 6:
		flag_CCSC += 1;
		break;
	case 7:
		flag_CCSCC += 1;
		break;
	}

	if (path.lenth < 1000)
	{
		return 0;
	}
	else
	{
		return 2;
	}
}

double mod2pi(double x)
{
	double v = fmod(x, twopi);
	if (v < -M_PI)
		v += twopi;
	else if (v > M_PI)
		v -= twopi;
	return v;
}

void polar(double x, double y, double &r, double &theta)
{
	r = sqrt(x * x + y * y);
	theta = atan2(y, x);
}

std::pair<double, double> calc_tau_omega(const double u,
										 const double v,
										 const double xi,
										 const double eta,
										 const double phi)
{
	double delta = mod2pi(u - v);
	double A = sin(u) - sin(delta);
	double B = cos(u) - cos(delta) - 1.0;

	double t1 = atan2(eta * A - xi * B, xi * A + eta * B);
	double t2 = 2.0 * (cos(delta) - cos(v) - cos(u)) + 3.0;
	double tau = 0.0;
	if (t2 < 0)
	{
		tau = mod2pi(t1 + M_PI);
	}
	else
	{
		tau = mod2pi(t1);
	}
	double omega = mod2pi(tau - u + v - phi);
	return std::make_pair(tau, omega);
}

bool LSL(double x, double y, double phi, double &t, double &u, double &v)
{
	polar(x - sin(phi), y - 1. + cos(phi), u, t);
	if (t >= 0)
	{
		v = mod2pi(phi - t);
		if (v >= 0)
		{
			return true;
		}
	}
	return false;
}

bool LSR(double x, double y, double phi, double &t, double &u, double &v)
{
	double t1, u1;
	polar(x + sin(phi), y - 1. - cos(phi), u1, t1);
	u1 = u1 * u1;
	if (u1 >= 4)
	{
		double theta;
		u = sqrt(u1 - 4);
		theta = atan2(2, u);
		t = mod2pi(t1 + theta);
		v = mod2pi(t - phi);
		return t >= 0 && v >= 0;
	}
	return false;
}

bool LRL(double x, double y, double phi, double &t, double &u, double &v)
{
	double xi = x - sin(phi), eta = y - 1. + cos(phi), u1, theta;
	polar(xi, eta, u1, theta);
	if (u1 <= 4.)
	{
		u = -2. * asin(.25 * u1);
		t = mod2pi(theta + .5 * u + pi);
		v = mod2pi(phi - t + u);

		return t >= 0 && u <= 0;
	}
	return false;
}

bool SLS(double x, double y, double phi, double &t, double &u, double &v)
{
	double phi_mod = mod2pi(phi);
	double epsilon = 1e-1;
	double xd = 0.0;
	if (y > 0.0 && phi_mod > epsilon && phi_mod < M_PI)
	{
		xd = -y / tan(phi_mod) + x;
		t = xd + tan(phi_mod / 2.0);
		u = -phi_mod;
		v = sqrt((x - xd) * (x - xd) + y * y) + tan(phi_mod / 2.0);
		return true;
	}
	else if (y < 0.0 && phi_mod > epsilon && phi_mod < M_PI)
	{
		xd = -y / tan(phi_mod) + x;
		t = xd + tan(phi_mod / 2.0);
		u = -phi_mod;
		v = -sqrt((x - xd) * (x - xd) + y * y) + tan(phi_mod / 2.0);
		return true;
	}
	return false;
}

bool LRLRn(double x, double y, double phi, double &t, double &u, double &v)
{
	double xi = x + sin(phi), eta = y - 1. + cos(phi);
	double rho = 0.25 * (2.0 + sqrt(xi * xi + eta * eta));
	if (rho <= 1.0 && rho >= 0.0)
	{
		u = acos(rho);
		if (u >= 0 && u <= 0.5 * M_PI)
		{
			std::pair<double, double> tau_omega = calc_tau_omega(u, -u, xi, eta, phi);
			if (tau_omega.first >= 0.0 && tau_omega.second <= 0.0)
			{
				t = tau_omega.first;
				v = tau_omega.second;
				return true;
			}
		}
	}
	return false;
}

bool LRLRp(double x, double y, double phi, double &t, double &u, double &v)
{
	double xi = x + sin(phi), eta = y - 1. - cos(phi);
	double rho = (20.0 - xi * xi - eta * eta) / 16.0;
	if (rho <= 1.0 && rho >= 0.0)
	{
		u = -acos(rho);
		if (u >= 0 && u <= 0.5 * M_PI)
		{
			std::pair<double, double> tau_omega = calc_tau_omega(u, u, xi, eta, phi);
			if (tau_omega.first >= 0.0 && tau_omega.second >= 0.0)
			{
				t = tau_omega.first;
				v = tau_omega.second;
				return true;
			}
		}
	}
	return false;
}

bool LRSR(double x, double y, double phi, double &t, double &u, double &v)
{
	double rho, theta;
	polar(x + sin(phi), -(y - 1. - cos(phi)), rho, theta);
	if (rho >= 2.0)
	{
		t = theta;
		u = 2.0 - rho;
		v = mod2pi(t + 0.5 * M_PI - phi);
		if (t >= 0.0 && u <= 0.0 && v <= 0.0)
		{
			return true;
		}
	}
	return false;
}

bool LRSL(double x, double y, double phi, double &t, double &u, double &v)
{
	double rho, theta;
	polar(x - sin(phi), y - 1. + cos(phi), rho, theta);
	double r = 0.0;
	if (rho >= 2.0)
	{
		r = sqrt(rho * rho - 4.0);
		u = 2.0 - r;
		t = mod2pi(theta + atan2(r, -2.0));
		v = mod2pi(phi - 0.5 * M_PI - t);
		if (t >= 0.0 && u <= 0.0 && v <= 0.0)
		{
			return true;
		}
	}
	return false;
}

bool LRSLR(double x, double y, double phi, double &t, double &u, double &v)
{
	double rho, theta;
	double xi = x + sin(phi);
	double eta = y - 1. - cos(phi);
	polar(xi, eta, rho, theta);
	if (rho >= 2.0)
	{
		u = 4.0 - sqrt(rho * rho - 4.0);
		if (u <= 0.0)
		{
			t = mod2pi(atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
			v = mod2pi(t - phi);
			if (t >= 0.0 && v >= 0.0)
			{
				return true;
			}
		}
	}
	return false;
}

bool LpRpSp(double x, double y, double phi, double &t, double &u, double &v)
{
	double xi = (1 + x * sin(phi) - y * cos(phi) + cos(phi));

	if (fabs(xi) <= 2)
	{
		t = mod2pi(acos(xi / 2) + phi);
		u = mod2pi(acos(xi / 2));
		if ((fabs(phi) < pi / 4) || (fabs(phi) > 3 * pi / 4))
		{
			v = (x + sin(phi) - 2 * sin(t)) / cos(phi);
		}
		else
		{
			v = (y + 2 * cos(t) - 1 - cos(phi)) / sin(phi);
		}
		return true;
	}
	return false;
}

void CSC(double x, double y, double phi, RsPath &path)
{
	double t, u, v, Lmin = path.lenth, L;
	if (LSL(x, y, phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v)))
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = u;
		path.pathlength[2] = v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "LSL";
	}
	if (LSL(-x, y, -phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = -u;
		path.pathlength[2] = -v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "LSL";
	}
	if (LSL(x, -y, -phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = u;
		path.pathlength[2] = v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "RSR";
	}
	if (LSL(-x, -y, phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = -u;
		path.pathlength[2] = -v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "RSR";
	}
	if (LSR(x, y, phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v)))
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = u;
		path.pathlength[2] = v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "LSR";
	}
	if (LSR(-x, y, -phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = -u;
		path.pathlength[2] = -v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "LSR";
	}
	if (LSR(x, -y, -phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = u;
		path.pathlength[2] = v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "RSL";
	}
	if (LSR(-x, -y, phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = -u;
		path.pathlength[2] = -v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "RSL";
	}
}

void CCC(double x, double y, double phi, RsPath &path)
{
	double t, u, v, Lmin = path.lenth, L;
	if (LRL(x, y, phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v)))
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = u;
		path.pathlength[2] = v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "LRL";
	}
	if (LRL(-x, y, -phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = -u;
		path.pathlength[2] = -v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "LRL";
	}
	if (LRL(x, -y, -phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = u;
		path.pathlength[2] = v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "RLR";
	}
	if (LRL(-x, -y, phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = -u;
		path.pathlength[2] = -v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "RLR";
	}

	double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
	if (LRL(xb, yb, phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v)))
	{
		path.lenth = L;
		path.pathlength[0] = v;
		path.pathlength[1] = u;
		path.pathlength[2] = t;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "LRL";
	}
	if (LRL(-xb, yb, -phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = -v;
		path.pathlength[1] = -u;
		path.pathlength[2] = -t;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "LRL";
	}
	if (LRL(xb, -yb, -phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
	{
		path.lenth = L;
		path.pathlength[0] = v;
		path.pathlength[1] = u;
		path.pathlength[2] = t;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "RLR";
	}
	if (LRL(-xb, -yb, phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
	{
		path.lenth = L;
		path.pathlength[0] = -v;
		path.pathlength[1] = -u;
		path.pathlength[2] = -t;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "RLR";
	}
}

void CCS(double x, double y, double phi, RsPath &path)
{
	double t, u, v, Lmin = path.lenth, L;
	if (LpRpSp(x, y, phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v)))
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = u;
		path.pathlength[2] = v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "LRS";
	}
	if (LpRpSp(-x, y, -phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = -u;
		path.pathlength[2] = -v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "LRS";
	}
	if (LpRpSp(x, -y, -phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = u;
		path.pathlength[2] = v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "RLS";
	}
	if (LpRpSp(-x, -y, phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = -u;
		path.pathlength[2] = -v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "RLS";
	}
}

void SCS(double x, double y, double phi, RsPath &path)
{
	double t, u, v, Lmin = path.lenth, L;
	if (SLS(x, y, phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v)))
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = u;
		path.pathlength[2] = v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "SRS";
	}
	if (SLS(x, -y, -phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = u;
		path.pathlength[2] = v;
		path.pathlength[3] = 0;
		path.pathlength[4] = 0;
		path.pathtype = "SLS";
	}
}

void CCCC(double x, double y, double phi, RsPath &path)
{
	double t, u, v, Lmin = path.lenth, L;
	if (LRLRn(x, y, phi, t, u, v) && path.lenth > (L = fabs(t) + 2 * fabs(u) + fabs(v)))
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = u;
		path.pathlength[2] = -u;
		path.pathlength[3] = v;
		path.pathlength[4] = 0;
		path.pathtype = "LRLR";
	}
	if (LRLRn(-x, y, -phi, t, u, v) && path.lenth > (L = fabs(t) + 2 * fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = -u;
		path.pathlength[2] = u;
		path.pathlength[3] = -v;
		path.pathlength[4] = 0;
		path.pathtype = "LRLR";
	}
	if (LRLRn(x, -y, -phi, t, u, v) && path.lenth > (L = fabs(t) + 2 * fabs(u) + fabs(v)))
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = u;
		path.pathlength[2] = -u;
		path.pathlength[3] = v;
		path.pathlength[4] = 0;
		path.pathtype = "RLRL";
	}
	if (LRLRn(-x, -y, phi, t, u, v) && path.lenth > (L = fabs(t) + 2 * fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = -u;
		path.pathlength[2] = u;
		path.pathlength[3] = -v;
		path.pathlength[4] = 0;
		path.pathtype = "RLRL";
	}

	if (LRLRp(x, y, phi, t, u, v) && path.lenth > (L = fabs(t) + 2 * fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = u;
		path.pathlength[2] = u;
		path.pathlength[3] = v;
		path.pathlength[4] = 0;
		path.pathtype = "LRLR";
	}
	if (LRLRp(-x, y, -phi, t, u, v) && path.lenth > (L = fabs(t) + 2 * fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = -u;
		path.pathlength[2] = -u;
		path.pathlength[3] = -v;
		path.pathlength[4] = 0;
		path.pathtype = "LRLR";
	}
	if (LRLRp(x, -y, -phi, t, u, v) && path.lenth > (L = fabs(t) + 2 * fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = u;
		path.pathlength[2] = u;
		path.pathlength[3] = v;
		path.pathlength[4] = 0;
		path.pathtype = "RLRL";
	}
	if (LRLRp(-x, -y, phi, t, u, v) && path.lenth > (L = fabs(t) + 2 * fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = -u;
		path.pathlength[2] = -u;
		path.pathlength[3] = -v;
		path.pathlength[4] = 0;
		path.pathtype = "RLRL";
	}
}

void CCSC(double x, double y, double phi, RsPath &path)
{
	double t, u, v, Lmin = path.lenth, L;
	if (LRSL(x, y, phi, t, u, v) && path.lenth > (L = fabs(t) + 0.5 * M_PI + fabs(u) + fabs(v)))
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = -0.5 * M_PI;
		path.pathlength[2] = u;
		path.pathlength[3] = v;
		path.pathlength[4] = 0;
		path.pathtype = "LRSL";
	}
	if (LRSL(-x, y, -phi, t, u, v) && path.lenth > (L = fabs(t) + 0.5 * M_PI + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = 0.5 * M_PI;
		path.pathlength[2] = -u;
		path.pathlength[3] = -v;
		path.pathlength[4] = 0;
		path.pathtype = "LRSL";
	}
	if (LRSL(x, -y, -phi, t, u, v) && path.lenth > (L = fabs(t) + 0.5 * M_PI + fabs(u) + fabs(v)))
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = -0.5 * M_PI;
		path.pathlength[2] = u;
		path.pathlength[3] = v;
		path.pathlength[4] = 0;
		path.pathtype = "RLSR";
	}
	if (LRSL(-x, -y, phi, t, u, v) && path.lenth > (L = fabs(t) + 0.5 * M_PI + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = 0.5 * M_PI;
		path.pathlength[2] = -u;
		path.pathlength[3] = -v;
		path.pathlength[4] = 0;
		path.pathtype = "RLSR";
	}

	if (LRSR(x, y, phi, t, u, v) && path.lenth > (L = fabs(t) + 0.5 * M_PI + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = -0.5 * M_PI;
		path.pathlength[2] = u;
		path.pathlength[3] = v;
		path.pathlength[4] = 0;
		path.pathtype = "LRSR";
	}
	if (LRSR(-x, y, -phi, t, u, v) && path.lenth > (L = fabs(t) + 0.5 * M_PI + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = 0.5 * M_PI;
		path.pathlength[2] = -u;
		path.pathlength[3] = -v;
		path.pathlength[4] = 0;
		path.pathtype = "LRSR";
	}
	if (LRSR(x, -y, -phi, t, u, v) && path.lenth > (L = fabs(t) + 0.5 * M_PI + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = -0.5 * M_PI;
		path.pathlength[2] = u;
		path.pathlength[3] = v;
		path.pathlength[4] = 0;
		path.pathtype = "RLSL";
	}
	if (LRSR(-x, -y, phi, t, u, v) && path.lenth > (L = fabs(t) + 0.5 * M_PI + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = 0.5 * M_PI;
		path.pathlength[2] = -u;
		path.pathlength[3] = -v;
		path.pathlength[4] = 0;
		path.pathtype = "RLSL";
	}

	double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
	if (LRSL(xb, yb, phi, t, u, v) && path.lenth > (L = fabs(t) + 0.5 * M_PI + fabs(u) + fabs(v)))
	{
		path.lenth = L;
		path.pathlength[0] = v;
		path.pathlength[1] = u;
		path.pathlength[2] = -0.5 * M_PI;
		path.pathlength[3] = t;
		path.pathlength[4] = 0;
		path.pathtype = "LSRL";
	}
	if (LRSL(-xb, yb, -phi, t, u, v) && path.lenth > (L = fabs(t) + 0.5 * M_PI + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = -v;
		path.pathlength[1] = -u;
		path.pathlength[2] = 0.5 * M_PI;
		path.pathlength[3] = -t;
		path.pathlength[4] = 0;
		path.pathtype = "LSRL";
	}
	if (LRSL(xb, -yb, -phi, t, u, v) && path.lenth > (L = fabs(t) + 0.5 * M_PI + fabs(u) + fabs(v)))
	{
		path.lenth = L;
		path.pathlength[0] = v;
		path.pathlength[1] = u;
		path.pathlength[2] = -0.5 * M_PI;
		path.pathlength[3] = t;
		path.pathlength[4] = 0;
		path.pathtype = "RSLR";
	}
	if (LRSL(-xb, -yb, phi, t, u, v) && path.lenth > (L = fabs(t) + 0.5 * M_PI + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = -v;
		path.pathlength[1] = -u;
		path.pathlength[2] = 0.5 * M_PI;
		path.pathlength[3] = -t;
		path.pathlength[4] = 0;
		path.pathtype = "RSLR";
	}

	if (LRSR(xb, yb, phi, t, u, v) && path.lenth > (L = fabs(t) + 0.5 * M_PI + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = v;
		path.pathlength[1] = u;
		path.pathlength[2] = -0.5 * M_PI;
		path.pathlength[3] = t;
		path.pathlength[4] = 0;
		path.pathtype = "RSRL";
	}
	if (LRSR(-xb, yb, -phi, t, u, v) && path.lenth > (L = fabs(t) + 0.5 * M_PI + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = -v;
		path.pathlength[1] = -u;
		path.pathlength[2] = 0.5 * M_PI;
		path.pathlength[3] = -t;
		path.pathlength[4] = 0;
		path.pathtype = "RSRL";
	}
	if (LRSR(xb, -yb, -phi, t, u, v) && path.lenth > (L = fabs(t) + 0.5 * M_PI + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = v;
		path.pathlength[1] = u;
		path.pathlength[2] = -0.5 * M_PI;
		path.pathlength[3] = t;
		path.pathlength[4] = 0;
		path.pathtype = "LSLR";
	}
	if (LRSR(-xb, -yb, phi, t, u, v) && path.lenth > (L = fabs(t) + 0.5 * M_PI + fabs(u) + fabs(v))) // timeflip
	{
		path.lenth = L;
		path.pathlength[0] = -v;
		path.pathlength[1] = -u;
		path.pathlength[2] = 0.5 * M_PI;
		path.pathlength[3] = -t;
		path.pathlength[4] = 0;
		path.pathtype = "LSLR";
	}
}

void CCSCC(double x, double y, double phi, RsPath &path)
{
	double t, u, v, Lmin = path.lenth, L;
	if (LRSLR(x, y, phi, t, u, v) && path.lenth > (L = fabs(t) + M_PI + fabs(u) + fabs(v)))
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = -0.5 * M_PI;
		path.pathlength[2] = u;
		path.pathlength[3] = -0.5 * M_PI;
		path.pathlength[4] = v;
		path.pathtype = "LRSLR";
	}
	if (LRSLR(-x, y, -phi, t, u, v) && path.lenth > (L = fabs(t) + M_PI + fabs(u) + fabs(v)))
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = 0.5 * M_PI;
		path.pathlength[2] = -u;
		path.pathlength[3] = 0.5 * M_PI;
		path.pathlength[4] = -v;
		path.pathtype = "LRSLR";
	}
	if (LRSLR(x, -y, -phi, t, u, v) && path.lenth > (L = fabs(t) + M_PI + fabs(u) + fabs(v)))
	{
		path.lenth = L;
		path.pathlength[0] = t;
		path.pathlength[1] = -0.5 * M_PI;
		path.pathlength[2] = u;
		path.pathlength[3] = -0.5 * M_PI;
		path.pathlength[4] = v;
		path.pathtype = "RLSRL";
	}
	if (LRSLR(-x, -y, phi, t, u, v) && path.lenth > (L = fabs(t) + M_PI + fabs(u) + fabs(v)))
	{
		path.lenth = L;
		path.pathlength[0] = -t;
		path.pathlength[1] = 0.5 * M_PI;
		path.pathlength[2] = -u;
		path.pathlength[3] = 0.5 * M_PI;
		path.pathlength[4] = -v;
		path.pathtype = "RLSRL";
	}
}