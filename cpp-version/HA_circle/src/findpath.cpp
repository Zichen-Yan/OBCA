#include "findpath.h"
// #include "util_time.h"
// #define OBCA
//////////////new//////////////////////
// bai
double Rect_x[478] = { 4.0443,4.0438,4.043,4.042,4.0407,4.0393,4.0376,4.0362,4.0348,4.0328,4.0302,4.0271,4.0234,4.0191,4.0144,4.0089,4.0011,3.9926,3.9835,3.9737,3.9633,3.9522,3.9405,3.9282,3.9277,3.9151,3.9013,3.8865,3.8706,3.8536,3.8357,3.8167,3.7968,3.776,3.7856,3.7675,3.7486,3.729,3.7086,3.6876,3.6659,3.6435,3.6206,3.605,3.5894,3.5633,3.5364,3.5087,3.4805,3.4518,3.4226,3.393,3.3633,3.3333,3.326,3.296,3.2661,3.2361,3.2062,3.1762,3.1462,3.1388,3.1089,3.0789,3.0489,3.0189,2.9889,2.9589,2.9289,2.8989,2.8689,2.8492,2.8192,2.7892,2.7592,2.7292,2.6992,2.6692,2.6392,2.6092,2.5792,2.5493,2.5193,2.4893,2.4593,2.4293,2.3993,2.3693,2.3532,2.3371,2.3071,2.2772,2.2472,2.2172,2.1872,2.1573,2.1273,2.0973,2.0673,2.0373,2.0073,1.9773,1.9473,1.9173,1.8873,1.8573,1.8273,1.7973,1.7673,1.7373,1.7073,1.6836,1.6536,1.6236,1.5936,1.5636,1.5336,1.5036,1.4736,1.4436,1.4136,1.3836,1.3536,1.3236,1.2936,1.2636,1.2336,1.2036,1.1736,1.1436,1.1136,1.0836,1.0536,1.0236,0.99358,0.96358,0.93358,0.90359,0.87359,0.84359,0.8136,0.7836,0.7536,0.72361,0.69361,0.66362,0.63363,0.60363,0.57364,0.59531,0.56545,0.53557,0.50569,0.47579,0.44588,0.41595,0.38602,0.35608,0.32613,0.29617,0.2662,0.23623,0.20625,0.17627,0.14628,0.11629,0.08629,0.056293,0.026294,-0.003706,-0.033706,-0.063705,-0.093703,-0.1237,-0.15369,-0.18368,-0.21367,-0.24365,-0.27362,-0.29313,-0.3231,-0.35305,-0.38296,-0.41282,-0.44263,-0.47237,-0.48756,-0.50274,-0.53226,-0.5617,-0.59105,-0.6203,-0.64946,-0.67851,-0.69302,-0.70754,-0.73521,-0.76225,-0.78857,-0.81413,-0.83883,-0.86262,-0.88544,-0.90723,-0.91063,-0.93249,-0.95333,-0.9731,-0.99174,-1.0092,-1.0255,-1.0405,-1.0479,-1.0553,-1.0672,-1.0784,-1.089,-1.0988,-1.1081,-1.116,-1.124,-1.1316,-1.1389,-1.1458,-1.1522,-1.1575,-1.1624,-1.1671,-1.1716,-1.1759,-1.1801,-1.1824,-1.1846,-1.1874,-1.1898,-1.1919,-1.1936,-1.195,-1.1961,-1.1958,-1.1979,-1.198,-1.1982,-1.1967,-1.1954,-1.1941,-1.1929,-1.1926,-1.1909,-1.1887,-1.1859,-1.1826,-1.1788,-1.1744,-1.1722,-1.1681,-1.1635,-1.1583,-1.1525,-1.1462,-1.1392,-1.137,-1.1298,-1.122,-1.1137,-1.1048,-1.0954,-1.0911,-1.0815,-1.0706,-1.0585,-1.045,-1.0304,-1.0145,-0.99744,-0.99088,-0.97171,-0.95177,-0.93111,-0.90973,-0.88766,-0.86493,-0.84156,-0.82037,-0.7951,-0.76899,-0.74211,-0.71455,-0.68638,-0.6577,-0.62858,-0.59911,-0.59928,-0.57,-0.5406,-0.51107,-0.48145,-0.45173,-0.42194,-0.39863,-0.36871,-0.33878,-0.30884,-0.27888,-0.24892,-0.21894,-0.18896,-0.15897,-0.12898,-0.098983,-0.068984,-0.038985,-0.0089849,0.017873,0.047771,0.077678,0.10759,0.13752,0.16745,0.19739,0.22733,0.25728,0.28724,0.3172,0.34717,0.37714,0.40712,0.4371,0.46709,0.49708,0.52707,0.55706,0.58706,0.61705,0.64705,0.67705,0.70705,0.73705,0.76705,0.79705,0.82705,0.85704,0.88704,0.91703,0.94702,0.97701,1.007,1.037,1.0669,1.0969,1.116,1.146,1.176,1.206,1.236,1.266,1.296,1.326,1.356,1.386,1.416,1.446,1.476,1.506,1.536,1.566,1.596,1.626,1.656,1.686,1.716,1.746,1.776,1.806,1.836,1.866,1.896,1.926,1.956,1.9654,1.9953,2.0253,2.0553,2.0853,2.1152,2.1452,2.1752,2.2052,2.2352,2.2652,2.2951,2.3251,2.3551,2.3851,2.4151,2.4451,2.4751,2.5051,2.535,2.565,2.5906,2.6206,2.6506,2.6806,2.7106,2.7406,2.7706,2.8006,2.8306,2.8606,2.8906,2.9206,2.9506,2.9806,3.0106,3.0405,3.0705,3.0854,3.1154,3.1453,3.1753,3.2053,3.2162,3.2462,3.2761,3.3061,3.3359,3.3657,3.3955,3.4251,3.4533,3.4822,3.5104,3.538,3.5648,3.5908,3.6157,3.6395,3.6622,3.6836,3.7036,3.7084,3.7278,3.7472,3.7667,3.7863,3.8059,3.8256,3.8429,3.8603,3.8763,3.8915,3.9061,3.9199,3.9329,3.9451,3.9566,3.9673,3.9706,3.9812,3.9908,3.9994,4.0069,4.0135,4.0189,4.0234,4.0268,4.0283,4.0308,4.033,4.0351,4.037,4.0388,4.0403,4.0413,4.0427,4.0437,4.0442};
double Rect_y[478] = { -0.0029061,-0.032901,-0.062891,-0.092874,-0.12285,-0.15281,-0.18277,-0.2034,-0.23336,-0.2633,-0.29319,-0.32302,-0.35279,-0.38249,-0.41211,-0.4395,-0.46846,-0.49723,-0.52581,-0.55417,-0.5823,-0.61018,-0.6378,-0.66514,-0.666,-0.69321,-0.71988,-0.74594,-0.77138,-0.79613,-0.82016,-0.84342,-0.86588,-0.88749,-0.87434,-0.89824,-0.92155,-0.94424,-0.9663,-0.98769,-1.0084,-1.0284,-1.0477,-1.059,-1.0703,-1.0851,-1.0983,-1.11,-1.1201,-1.1286,-1.1355,-1.1408,-1.1444,-1.1463,-1.1463,-1.1481,-1.1498,-1.1514,-1.1528,-1.154,-1.1551,-1.1553,-1.1567,-1.1579,-1.159,-1.16,-1.1608,-1.1614,-1.162,-1.1624,-1.1626,-1.1627,-1.163,-1.1631,-1.1632,-1.1632,-1.1631,-1.1629,-1.1626,-1.1623,-1.1618,-1.1613,-1.1607,-1.16,-1.1592,-1.1584,-1.1575,-1.1564,-1.1558,-1.1552,-1.1535,-1.1519,-1.1504,-1.149,-1.1477,-1.1466,-1.1455,-1.1446,-1.1438,-1.1431,-1.1425,-1.1421,-1.1417,-1.1415,-1.1413,-1.1413,-1.1414,-1.1417,-1.142,-1.1424,-1.143,-1.1435,-1.1435,-1.1436,-1.1436,-1.1436,-1.1436,-1.1436,-1.1435,-1.1434,-1.1433,-1.1432,-1.1431,-1.1429,-1.1428,-1.1426,-1.1424,-1.1422,-1.1419,-1.1416,-1.1413,-1.141,-1.1407,-1.1404,-1.14,-1.1396,-1.1392,-1.1388,-1.1383,-1.1379,-1.1374,-1.1369,-1.1364,-1.1358,-1.1353,-1.1347,-1.1341,-1.1335,-1.1328,-1.1319,-1.1347,-1.1375,-1.1401,-1.1425,-1.1448,-1.147,-1.149,-1.1508,-1.1525,-1.1541,-1.1555,-1.1568,-1.1579,-1.1589,-1.1598,-1.1604,-1.161,-1.1614,-1.1616,-1.1617,-1.1617,-1.1615,-1.1612,-1.1607,-1.16,-1.1593,-1.1583,-1.1573,-1.156,-1.1552,-1.1539,-1.1521,-1.1497,-1.1469,-1.1435,-1.1396,-1.1373,-1.135,-1.1297,-1.1239,-1.1177,-1.111,-1.1039,-1.0965,-1.0919,-1.0874,-1.0759,-1.0629,-1.0485,-1.0328,-1.0158,-0.99749,-0.97802,-0.9574,-0.95311,-0.93257,-0.91099,-0.88843,-0.86493,-0.84055,-0.81536,-0.7894,-0.77485,-0.7603,-0.73275,-0.70493,-0.67685,-0.64852,-0.61998,-0.59337,-0.56446,-0.53545,-0.50634,-0.47714,-0.44784,-0.42181,-0.39221,-0.36258,-0.33293,-0.30324,-0.27353,-0.25601,-0.2385,-0.20862,-0.17872,-0.14879,-0.11884,-0.088875,-0.058896,-0.053784,-0.023857,-0.0072265,0.0094037,0.03937,0.06934,0.099313,0.12929,0.13662,0.16658,0.19649,0.22637,0.25618,0.28594,0.31562,0.32873,0.35846,0.3881,0.41765,0.44708,0.4764,0.50558,0.51427,0.54339,0.57237,0.60119,0.62985,0.65833,0.67032,0.69875,0.7267,0.75412,0.78094,0.8071,0.83256,0.85724,0.86588,0.88895,0.91137,0.93311,0.95415,0.97448,0.99405,1.0129,1.0286,1.0448,1.0596,1.0729,1.0847,1.095,1.1038,1.111,1.1166,1.1158,1.1224,1.1283,1.1336,1.1384,1.1425,1.146,1.1482,1.1505,1.1525,1.1543,1.1559,1.1574,1.1586,1.1597,1.1605,1.1612,1.1616,1.1619,1.162,1.1618,1.161,1.1585,1.1562,1.1539,1.1518,1.1498,1.1478,1.146,1.1443,1.1427,1.1412,1.1398,1.1385,1.1374,1.1363,1.1354,1.1345,1.1338,1.1332,1.1327,1.1323,1.132,1.1318,1.1317,1.1317,1.1319,1.1321,1.1325,1.133,1.1335,1.1342,1.135,1.1359,1.1369,1.138,1.1393,1.1406,1.1414,1.1416,1.1419,1.1421,1.1424,1.1426,1.1427,1.1429,1.1431,1.1432,1.1433,1.1434,1.1435,1.1435,1.1436,1.1436,1.1436,1.1436,1.1435,1.1435,1.1434,1.1433,1.1432,1.1431,1.1429,1.1428,1.1426,1.1424,1.1421,1.1418,1.1431,1.1443,1.1455,1.1466,1.1478,1.1489,1.15,1.151,1.152,1.153,1.154,1.1549,1.1558,1.1567,1.1576,1.1584,1.1592,1.16,1.1607,1.1614,1.162,1.1624,1.1627,1.163,1.1631,1.1632,1.1632,1.1631,1.1629,1.1626,1.1622,1.1618,1.1612,1.1606,1.1599,1.159,1.1581,1.1577,1.1563,1.1551,1.1538,1.1527,1.1522,1.1515,1.1501,1.148,1.1454,1.1421,1.1381,1.1336,1.1282,1.1199,1.1099,1.0981,1.0847,1.0696,1.0529,1.0347,1.015,0.994,0.97168,0.96581,0.94289,0.92003,0.89722,0.87448,0.85179,0.82916,0.8078,0.78644,0.76105,0.73523,0.70898,0.68233,0.65531,0.62793,0.60021,0.57218,0.5626,0.53454,0.50612,0.47738,0.44835,0.41907,0.38958,0.35991,0.3301,0.31096,0.28106,0.25114,0.22121,0.19128,0.16132,0.13136,0.11028,0.080315,0.050331,0.020336};
int Rect_index = 478;
// 20
double Rect_x10[68] = {4.0443,4.0362,4.0089,3.9277,3.776,3.7856,3.6875,3.5894,3.3333,3.326,3.1388,2.8689,2.8492,2.5792,2.3371,2.0673,1.7973,1.6836,1.4136,1.1436,0.87359,0.60363,0.59531,0.32613,0.056293,-0.21367,-0.29313,-0.50274,-0.70754,-0.91063,-1.0553,-1.116,-1.1575,-1.1846,-1.1958,-1.1982,-1.1926,-1.1722,-1.137,-1.0911,-0.99088,-0.82037,-0.59928,-0.39863,-0.12898,0.017873,0.28724,0.55706,0.82705,1.0969,1.116,1.386,1.656,1.926,1.9654,2.2352,2.5051,2.5906,2.8606,3.0854,3.2162,3.4533,3.6836,3.7084,3.8603,3.9706,4.0283,4.0413};
double Rect_y10[68] = {-0.0029061,-0.2034,-0.4395,-0.666,-0.88749,-0.87434,-0.97234,-1.0703,-1.1463,-1.1463,-1.1553,-1.1626,-1.1627,-1.1618,-1.1552,-1.1438,-1.1417,-1.1435,-1.1433,-1.1416,-1.1383,-1.1335,-1.1319,-1.1525,-1.1614,-1.1583,-1.1552,-1.135,-1.0874,-0.95311,-0.7603,-0.59337,-0.42181,-0.2385,-0.053784,0.0094037,0.13662,0.32873,0.51427,0.67032,0.86588,1.0286,1.1158,1.1482,1.1612,1.161,1.1427,1.1332,1.1325,1.1406,1.1414,1.1432,1.1435,1.1424,1.1418,1.152,1.16,1.162,1.1626,1.1577,1.1522,1.1282,0.994,0.96581,0.78644,0.5626,0.31096,0.11028};
int Rect_index10 = 68;
///////////////////////////////////////
double nseconds = 0;
double g_px = 0, g_py = 0, g_pth = 0;
Node g_tnode;
int g_xidx = 0, g_yidx = 0, g_thidx = 0;
int CloseSizeMax = 0;

int flag_CCS = 0;
int flag_CCC = 0;
int flag_CSC = 0;
int flag_SCS = 0;
int flag_CCCC = 0;
int flag_CCSC = 0;
int flag_CCSCC = 0;

int flag_1=0;
int flag_2=0;
int flag_3=0;
int flag_5=0;
int flag_8=0;
int flag_10=0;


/* priority */
std::priority_queue<std::pair<std::string, double>, std::vector<std::pair<std::string, double>>, cmp> pq;
std::unordered_map<std::string, Node> g_openset, g_closeset;
std::string str_idx;

std::vector<path_point> pathpoint;
Path_config pathfind_parameters;

std::string get_str_idx(Node &node);
std::string get_par_str_idx(Node &node);
// map_set map_parameters;
Vehicle_config vehicle_parameters;

std::vector<double> find_steer_degree;

double TotalCost(Node wknode, double End[3]);
int inNodes(Node node, std::vector<Node> *nodes);
bool CalcIdx(double x, double y, double theta, double &xidx, double &yidx, double &thidx);
void Update(Node wknode, double End[3]);
void PopNode(std::vector<Node> *nodes);
bool CalcNextNode(Node wknode, double D, double delta);
RsPath AnalysticExpantion(Node wknode, double End[3]);
void getFinalPath(RsPath path, std::string stridx);

///////类定义/////////////////

Node::Node()
{
}
Node::Node(int xidx1, int yidx1, int yawidx1, double D1, double delta1, double x1, double y1, double theta1, int parent1[3], double cost1, double fcost1)
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
	double x1 = x * cos(phi) + y * sin(phi);
	double y1 = y * cos(phi) - x * sin(phi);
	// 看是否从当前点到目标点存在无碰撞的Reeds - Shepp轨迹，前面pvec = End - Start; 的意义就在这里，注意！这里的x, y, prev(3)是把起点转换成以start为原点坐标系下的坐标
	double cost = 0;

	// double dis_to_midline = wknode.x-End[0]; // 负表示在中线左侧，正表示中线右侧
	// if (fabs(dis_to_midline)<1)
	// {
	// 	in_parking_lot=1;
	// }
	// else
	// {
	// 	in_parking_lot=0;
	// }

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
	float gres = pathfind_parameters.XY_GRID_RESOLUTION;							//[m]世界坐标生成栅格坐标的分辨率（2m一个栅格）
	float yawres = pathfind_parameters.YAW_GRID_RESOLUTION;							// 栅格角度分辨率
	g_xidx = round((x - pathfind_parameters.MINX) / gres);							// 计算栅格索引
	g_yidx = round((y - pathfind_parameters.MINY) / gres);							// 计算栅格索引
	theta = mod2pi(theta);															// 控制theta范围在[-pi, pi]区间
	g_thidx = round((theta - pathfind_parameters.MINYAW) / (yawres / 180 * pi));	// 计算栅格索引
	double middle_x = (pathfind_parameters.MAXX - pathfind_parameters.MINX) / gres; // 地图边界栅格索引
	double middle_y = (pathfind_parameters.MAXY - pathfind_parameters.MINY) / gres; // 地图边界栅格索引
	if (g_xidx <= 0 || g_xidx > ceil(middle_x))										// 判断是否在地图内
	{
		return false;
	}
	else if (g_yidx <= 0 || g_yidx > ceil(middle_y)) // 判断是否在地图内
	{
		return false;
	}

	return true;
}

int inNodes(Node node, std::vector<Node> *nodes) // 判断节点node是否在nodes里面
{
	int i, fan = -1;
	for (i = 0; i < nodes->size(); i++)
	{
		if (node.xidx == (*nodes)[i].xidx && node.yidx == (*nodes)[i].yidx && node.yawidx == (*nodes)[i].yawidx)
		{
			fan = i;
		}
	}
	return fan;
}

void VehicleDynamic(double x, double y, double theta, double D, double delta) // 根据当前位姿和输入, 计算下一位置的位姿
{
	double mid_R;

	if (delta == 0)
	{
		g_px = x + D * cos(theta);								// 运动学公式： x_dot = v_x * cos(theta); x_dot * t = v_x * t * cos(theta), 在采样时间t内, 则有x = x + v_x * t * cos(theta)，其中v_x * t = D
		g_py = y + D * sin(theta);								// 运动学公式
		g_pth = theta + D / vehicle_parameters.WB * tan(delta); // L是轴距, 航向变化, theta_dot = v / R, R = L / tan(delta)
		g_pth = mod2pi(g_pth);
	}
	else
	{
		mid_R = vehicle_parameters.WB / tan(delta);
		g_px = x - mid_R * sin(theta) + mid_R * sin(theta + D / mid_R); // 运动学公式： x_dot = v_x * cos(theta); x_dot * t = v_x * t * cos(theta), 在采样时间t内, 则有x = x + v_x * t * cos(theta)，其中v_x * t = D
		g_py = y + mid_R * cos(theta) - mid_R * cos(theta + D / mid_R); // 运动学公式
		g_pth = theta + D / mid_R;										// L是轴距, 航向变化, theta_dot = v / R, R = L / tan(delta)
		g_pth = mod2pi(g_pth);
	}
}

bool CalcNextNode(Node wknode, double D, double delta) // 根据当前点及距离及方向盘角度计算下一点
{
	double cost = 0;
	int fa[3] = {wknode.xidx, wknode.yidx, wknode.yawidx};
	g_px = wknode.x;
	g_py = wknode.y;
	g_pth = wknode.theta;
	int i, nlist = 3;
	for (i = 0; i < nlist; i++)
	{
		VehicleDynamic(g_px, g_py, g_pth, D, delta); // 更新全局变量的g_px、g_py、g_pth
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

	if (D > 0) // 前进
	{
		cost = cost + 0.3; // 每条轨迹大概是2米的长度。这里乘以1.5是确保下一个末端状态肯定在另一个栅格中，不会还在一个栅格中！在地图栅格中子结点拓展。比如对角线长度是1.4，此时还是在同一个栅格中
	}
	else // 后退
	{
		cost = cost + pathfind_parameters.BACK_COST * 0.3;
	}

	if (D != wknode.D)
	{
		cost = cost + pathfind_parameters.SB_COST;
	}
	else if (delta != wknode.delta)
	{
		//		cost = cost + pathfind_parameters.STEER_CHANGE_COST;
		cost = cost + pathfind_parameters.STEER_CHANGE_COST * fabs(delta - wknode.delta) / vehicle_parameters.MAX_STEER;
	}
	cost = cost + pathfind_parameters.STEER_COST*fabs(delta);
	//	cost = cost + pathfind_parameters.STEER_CHANGE_COST*fabs(delta - wknode.delta);
	g_tnode = Node(g_xidx, g_yidx, g_thidx, D, delta, g_px, g_py, g_pth, fa, cost, 0); // tnode是路径的末端点，cost为到当前状态到此路径末端状态的成本

	return true;
}

bool VehicleCollisionGrid(double cpx, double cpy, double cph)
{
	double phi = mod2pi(cph); // 构造旋转矩阵
	double cosphi = cos(phi);
	double sinphi = sin(phi);
	std::pair<double,double> p0={fusion.parkingSpaceInfo.P0_X/100.0,fusion.parkingSpaceInfo.P0_Y/100.0}; 
	std::pair<double,double> p1={fusion.parkingSpaceInfo.P2_X/100.0,fusion.parkingSpaceInfo.P2_Y/100.0};
	int in_parking_lot;
	if (((p0.first<=cpx && cpx<=p1.first) || (p1.first<=cpx && cpx<=p0.first)) && ((p0.second<=cpy && cpy<=p1.second) || (p1.second<=cpy && cpy<=p0.second)))
		in_parking_lot=1;
	else
		in_parking_lot=0;
	
	// bydapa::common::TicToc tictoc_;
	// tictoc_.tic();
	
	if (in_parking_lot)
	{
		double rect_x;
		double rect_y;
		double rect_index = Rect_index10;

		for (int i = 0; i < rect_index; i++)
		{
			if (rect_index != Rect_index)
			{
				rect_x = Rect_x10[i] * cosphi - Rect_y10[i] * sinphi + cpx;
				rect_y = Rect_y10[i] * cosphi + Rect_x10[i] * sinphi + cpy;
			}
			else
			{
				rect_x = Rect_x[i] * cosphi - Rect_y[i] * sinphi + cpx;
				rect_y = Rect_y[i] * cosphi + Rect_x[i] * sinphi + cpy;
			}

			// 栅格化
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
				return true;
			}
			else
			{
				if (no_imag_map == 0)
				{
					if ((obstmap[idx_xy].Status == 0) || (obstmap[idx_xy].Status == 3) || (obstmap[idx_xy].Status == 4))
					{
						return true;
					}
				}
				else
				{
					if ((obstmap[idx_xy].Status == 0) || (obstmap[idx_xy].Status == 4))
					{
						return true;
					}
				}
			}
		}
	}
	else
	{
		double length=vehicle_parameters.LB+vehicle_parameters.LF+0.4;
		double wid = vehicle_parameters.W+0.4;
		// return VehicleCollisionTreeSearch(cpx,cpy,cph,0,cosphi,sinphi,length,wid);
		int result = VehicleCollisionTreeSearch3(cpx, cpy, cph, cosphi, sinphi, length, wid);
		if (result)
		{
			if(VehicleCollisionTreeSearch8(cpx, cpy, cph, result, cosphi, sinphi, length, wid))
				return true;
		}
	}
	
	// nseconds = tictoc_.toc();
	// nseconds = nseconds * 0.000000001;
	// std::cout << "obs time:" << nseconds << std::endl;
	return false;
}

bool VehicleCollisionTreeSearch1(double cpx, double cpy, double cph, double cosphi, double sinphi, double length, double wid)
{
	double offset; //圆心间距离
	double radius; //圆半径

	flag_1++;
	offset=0;
	radius = sqrt(pow(wid/2,2)+pow(length/2,2));
	
	double rect_x=length/2-(vehicle_parameters.LB+0.2); //车辆中心坐标
	double rect_y=0;
	double center_x= rect_x * cosphi - rect_y * sinphi + cpx; //圆心坐标
	double center_y= rect_x * sinphi + rect_y * cosphi + cpy;
	std::pair<double,double> tmp1={center_x,center_y};
	
	std::vector<std::pair<double,double>> circle_list={tmp1};
	std::vector<double> radius_list={radius-0.2};

	for (const auto& circle : circle_list) {
		center_x=circle.first;
		center_y=circle.second;
		// 栅格化
		int xidx = ceil((pathfind_parameters.MAXX - center_x) / pathfind_parameters.XY_GRID_RESOLUTION);
		int yidx = ceil((pathfind_parameters.MAXY - center_y) / pathfind_parameters.XY_GRID_RESOLUTION);
		if (xidx == 0){
			xidx = 1;
		}
		if (yidx == 0){
			yidx = 1;
		}
		int idx_xy = (xidx - 1) * 250 + (yidx - 1);
		if ((xidx <= 0 || xidx > pathfind_parameters.XIDX) || (yidx <= 0 || yidx > pathfind_parameters.XIDY))
		{
			return true;
		}
		else {
			double dis = dis_map[(yidx-1)][249-(xidx-1)]*pathfind_parameters.XY_GRID_RESOLUTION;
			double radius = radius_list[0];
			if (dis<=radius)
			{
				return true;
			}
		}
	}
	return false;
}

int VehicleCollisionTreeSearch2(double cpx, double cpy, double cph, double cosphi, double sinphi, double length, double wid)
{
	int result = 0; // 0无碰撞，1前车碰撞，2后车碰撞，3前后都碰撞

	flag_2++;
	double offset=length/4; //圆心间距离
	double radius = sqrt(pow(wid/2,2)+pow(length/4,2))-0.2; //圆半径
	
	double rect_x=length/2-(vehicle_parameters.LB+0.2); //车辆中心坐标
	double rect_y=0;
	double center_x= (rect_x+offset-0.15) * cosphi + cpx; //圆心坐标
	double center_y= (rect_x+offset-0.15) * sinphi + cpy;
	// std::pair<double,double> tmp1={center_x,center_y};

	int xidx = ceil((pathfind_parameters.MAXX - center_x) / pathfind_parameters.XY_GRID_RESOLUTION);
	int yidx = ceil((pathfind_parameters.MAXY - center_y) / pathfind_parameters.XY_GRID_RESOLUTION);
	if (xidx == 0){
		xidx = 1;
	}
	if (yidx == 0){
		yidx = 1;
	}
	int idx_xy = (xidx - 1) * 250 + (yidx - 1);
	double dis;
	if ((xidx <= 0 || xidx > pathfind_parameters.XIDX) || (yidx <= 0 || yidx > pathfind_parameters.XIDY))
	{
		result=1;
	}
	else {
		dis = dis_map[(yidx-1)][249-(xidx-1)]*pathfind_parameters.XY_GRID_RESOLUTION;
		if (dis<=radius)
		{
			result=1;
		}
	}

	center_x = (rect_x-offset+0.15) * cosphi + cpx;
	center_y = (rect_x-offset+0.15) * sinphi + cpy;

	xidx = ceil((pathfind_parameters.MAXX - center_x) / pathfind_parameters.XY_GRID_RESOLUTION);
	yidx = ceil((pathfind_parameters.MAXY - center_y) / pathfind_parameters.XY_GRID_RESOLUTION);
	if (xidx == 0){
		xidx = 1;
	}
	if (yidx == 0){
		yidx = 1;
	}
	idx_xy = (xidx - 1) * 250 + (yidx - 1);
	if ((xidx <= 0 || xidx > pathfind_parameters.XIDX) || (yidx <= 0 || yidx > pathfind_parameters.XIDY))
	{
		if (result==0)
			result=2;
		else
			result=3;
	}
	else {
		dis = dis_map[(yidx-1)][249-(xidx-1)]*pathfind_parameters.XY_GRID_RESOLUTION;
		if (dis<=radius)
		{
			if (result==0)
				result=2;
			else
				result=3;
		}
	}
	return result;
}

int VehicleCollisionTreeSearch3(double cpx, double cpy, double cph, double cosphi, double sinphi, double length, double wid)
{
	int result = 0; // 0无碰撞，1前车碰撞，2后车碰撞，3前后都碰撞
	double offset; //圆心间距离
	double radius; //圆半径

	flag_3++;
	offset=length/3;
	radius = sqrt(pow(wid/2,2)+pow(length/6,2))-0.1; 
	
	double rect_x=length/2-(vehicle_parameters.LB+0.2); //车辆中心坐标
	double rect_y=0;

	double center_x1= (rect_x+offset-0.25) * cosphi + cpx; //圆心坐标
	double center_y1= (rect_x+offset-0.25) * sinphi + cpy;
	// std::pair<double,double> tmp1={center_x,center_y};

	double center_x2 = (rect_x) * cosphi + cpx;
	double center_y2 = (rect_x) * sinphi + cpy;
	// std::pair<double,double> tmp2={center_x,center_y};

	double center_x3 = (rect_x-offset+0.25) * cosphi + cpx;
	double center_y3 = (rect_x-offset+0.25) * sinphi + cpy;
	// std::pair<double,double> tmp3={center_x,center_y};

	// std::vector<std::pair<double,double>> circle_list={{center_x1,center_y1}, {center_x2,center_y2}, {center_x3,center_y3}};
	// std::vector<double> radius_list={radius-0.08,radius-0.13,radius-0.08};
	
	double center_x;
	double center_y; 

	center_x=center_x1;
	center_y=center_y1;
	// 栅格化
	int xidx = ceil((pathfind_parameters.MAXX - center_x) / pathfind_parameters.XY_GRID_RESOLUTION);
	int yidx = ceil((pathfind_parameters.MAXY - center_y) / pathfind_parameters.XY_GRID_RESOLUTION);
	if (xidx == 0){
		xidx = 1;
	}
	if (yidx == 0){
		yidx = 1;
	}
	int idx_xy = (xidx - 1) * 250 + (yidx - 1);
	if ((xidx <= 0 || xidx > pathfind_parameters.XIDX) || (yidx <= 0 || yidx > pathfind_parameters.XIDY))
	{
		result = 1;
	}
	else {
		double dis = dis_map[(yidx-1)][249-(xidx-1)]*pathfind_parameters.XY_GRID_RESOLUTION;
		if (dis<=radius)
		{
			result = 1;
		}
	}

	center_x=center_x2;
	center_y=center_y2;
	// 栅格化
	xidx = ceil((pathfind_parameters.MAXX - center_x) / pathfind_parameters.XY_GRID_RESOLUTION);
	yidx = ceil((pathfind_parameters.MAXY - center_y) / pathfind_parameters.XY_GRID_RESOLUTION);
	if (xidx == 0){
		xidx = 1;
	}
	if (yidx == 0){
		yidx = 1;
	}
	idx_xy = (xidx - 1) * 250 + (yidx - 1);
	if ((xidx <= 0 || xidx > pathfind_parameters.XIDX) || (yidx <= 0 || yidx > pathfind_parameters.XIDY))
	{
		return 3;
	}
	else {
		double dis = dis_map[(yidx-1)][249-(xidx-1)]*pathfind_parameters.XY_GRID_RESOLUTION;
		if (dis<=radius)
		{
			return 3;
		}
	}

	center_x=center_x3;
	center_y=center_y3;
	// 栅格化
	xidx = ceil((pathfind_parameters.MAXX - center_x) / pathfind_parameters.XY_GRID_RESOLUTION);
	yidx = ceil((pathfind_parameters.MAXY - center_y) / pathfind_parameters.XY_GRID_RESOLUTION);
	if (xidx == 0){
		xidx = 1;
	}
	if (yidx == 0){
		yidx = 1;
	}
	idx_xy = (xidx - 1) * 250 + (yidx - 1);
	if ((xidx <= 0 || xidx > pathfind_parameters.XIDX) || (yidx <= 0 || yidx > pathfind_parameters.XIDY))
	{
		if (result==1)
			return 3;
		else
			result=2;
	}
	else {
		double dis = dis_map[(yidx-1)][249-(xidx-1)]*pathfind_parameters.XY_GRID_RESOLUTION;
		if (dis<=radius)
		{
			if (result==1)
				return 3;
			else
				result=2;
		}
	}

	return result;
}

bool VehicleCollisionTreeSearch8(double cpx, double cpy, double cph, int result, double cosphi, double sinphi, double length, double wid)
{
	double offset; //圆心间距离
	double radius; //圆半径

	flag_8++;
	offset=length/8;
	radius = sqrt(pow(wid/4,2)+pow(length/8,2)); 
	
	double center_x1; //圆心坐标
	double center_y1;
	double center_x2;
	double center_y2;
	double center_x3; 
	double center_y3;
	double center_x4; 
	double center_y4;

	double rect_x; //车辆中心坐标
	double rect_y;
	std::vector<std::pair<double,double>> circle_list1;
	std::vector<std::pair<double,double>> circle_list2;
	std::vector<double> radius_list1;
	std::vector<double> radius_list2;
	
	rect_x=length/2-(vehicle_parameters.LB+0.2);
	if (result==1 || result==3)
	{
		rect_y=wid/4;

		center_x1 = (rect_x+3*offset-0.2) * cosphi - (rect_y-0.28) * sinphi + cpx;
		center_y1 = (rect_x+3*offset-0.2) * sinphi + (rect_y-0.28) * cosphi + cpy;
		// std::pair<double,double> tmp1={center_x,center_y};

		center_x2 = (rect_x+offset) * cosphi - (rect_y-0.15) * sinphi + cpx;
		center_y2 = (rect_x+offset) * sinphi + (rect_y-0.15) * cosphi + cpy;
		// std::pair<double,double> tmp2={center_x,center_y};

		rect_y=-wid/4;

		center_x3 = (rect_x+3*offset-0.2) * cosphi - (rect_y+0.28) * sinphi + cpx;
		center_y3 = (rect_x+3*offset-0.2) * sinphi + (rect_y+0.28) * cosphi + cpy;
		// std::pair<double,double> tmp3={center_x,center_y};

		center_x4 = (rect_x+offset) * cosphi - (rect_y+0.15) * sinphi + cpx;
		center_y4 = (rect_x+offset) * sinphi + (rect_y+0.15) * cosphi + cpy;
		// std::pair<double,double> tmp4={center_x,center_y};

		circle_list1={{center_x1,center_y1},{center_x2,center_y2},{center_x3,center_y3},{center_x4,center_y4}};
		radius_list1 = {radius+0.15,radius,radius+0.15,radius};
	}
	if (result==2 || result==3)
	{
		rect_y=wid/4;

		center_x1 = (rect_x-offset) * cosphi - (rect_y-0.15) * sinphi + cpx;
		center_y1 = (rect_x-offset) * sinphi + (rect_y-0.15) * cosphi + cpy;
		// std::pair<double,double> tmp5={center_x,center_y};

		center_x2 = (rect_x-3*offset+0.2) * cosphi - (rect_y-0.28) * sinphi + cpx;
		center_y2 = (rect_x-3*offset+0.2) * sinphi + (rect_y-0.28) * cosphi + cpy;
		// std::pair<double,double> tmp6={center_x,center_y};

		rect_y=-wid/4;

		center_x3 = (rect_x-offset) * cosphi - (rect_y+0.15) * sinphi + cpx;
		center_y3 = (rect_x-offset) * sinphi + (rect_y+0.15) * cosphi + cpy;
		// std::pair<double,double> tmp7={center_x,center_y};

		center_x4 = (rect_x-3*offset+0.2) * cosphi - (rect_y+0.28) * sinphi + cpx;
		center_y4 = (rect_x-3*offset+0.2) * sinphi + (rect_y+0.28) * cosphi + cpy;
		// std::pair<double,double> tmp8={center_x,center_y};

		circle_list2={{center_x1,center_y1},{center_x2,center_y2},{center_x3,center_y3},{center_x4,center_y4}};
		radius_list2 = {radius,radius+0.15,radius,radius+0.15};
	}
	circle_list1.insert(circle_list1.end(), circle_list2.begin(), circle_list2.end());
	radius_list1.insert(radius_list1.end(), radius_list2.begin(), radius_list2.end());
	
	size_t idx = 0;
	double center_x;
	double center_y;
	for (const auto& circle : circle_list1) {
		center_x=circle.first;
		center_y=circle.second;
		// 栅格化
		int xidx = ceil((pathfind_parameters.MAXX - center_x) / pathfind_parameters.XY_GRID_RESOLUTION);
		int yidx = ceil((pathfind_parameters.MAXY - center_y) / pathfind_parameters.XY_GRID_RESOLUTION);
		if (xidx == 0){
			xidx = 1;
		}
		if (yidx == 0){
			yidx = 1;
		}
		int idx_xy = (xidx - 1) * 250 + (yidx - 1);
		if ((xidx <= 0 || xidx > pathfind_parameters.XIDX) || (yidx <= 0 || yidx > pathfind_parameters.XIDY))
		{
			return true;
		}
		else {
			double dis = dis_map[(yidx-1)][249-(xidx-1)]*pathfind_parameters.XY_GRID_RESOLUTION;
			double radius = radius_list1[idx++];
			if (dis<=radius)
			{
				return true;
			}
		}
	}
	return false;
}

bool VehicleCollisionTreeSearch(double cpx, double cpy, double cph, int circle_idx, 
	double cosphi, double sinphi, double length, double wid)
{
	double offset; //圆心间距离
	double radius; //圆半径

	std::vector<int> circle_num_list={2,8};
	if (circle_idx>=circle_num_list.size()){
		return true;
	}
	int circle_num = circle_num_list[circle_idx];

	if (circle_num==1) // 1个圆
	{
		flag_1++;
		offset=0;
		radius = sqrt(pow(wid/2,2)+pow(length/2,2)); 
	}
	else if (circle_num==2)
	{
		flag_2++;
		offset=length/4;
		radius = sqrt(pow(wid/2,2)+pow(length/4,2)); 
	}
	else if (circle_num==3)
	{
		flag_3++;
		offset=length/3;
		radius = sqrt(pow(wid/2,2)+pow(length/6,2)); 
	}
	else if (circle_num==5)
	{
		flag_5++;
		offset=length/5;
		radius = sqrt(pow(wid/2,2)+pow(length/10,2)); 
	}
	else if (circle_num==8)
	{
		flag_8++;
		offset=length/8;
		radius = sqrt(pow(wid/4,2)+pow(length/8,2)); 
	}
	else if (circle_num==10)
	{
		flag_10++;
		offset=length/5;
		radius = sqrt(pow(wid/4,2)+pow(length/10,2)); 
	}
	
	double center_x; //圆心坐标
	double center_y;
	double rect_x; //车辆中心坐标
	double rect_y;
	std::vector<std::pair<double,double>> circle_list;
	std::vector<double> radius_list;
	if (circle_num==1){
		rect_x=length/2-(vehicle_parameters.LB+0.2);
		rect_y=0;

		center_x = rect_x * cosphi - rect_y * sinphi + cpx;
		center_y = rect_x * sinphi + rect_y * cosphi + cpy;
		std::pair<double,double> tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius-0.2);
	}
	else if (circle_num==2){
		rect_x=length/2-(vehicle_parameters.LB+0.2);
		rect_y=0;
		
		center_x = (rect_x+offset-0.15) * cosphi - rect_y * sinphi + cpx;
		center_y = (rect_x+offset-0.15) * sinphi + rect_y * cosphi + cpy;
		std::pair<double,double> tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius-0.2);

		center_x = (rect_x-offset+0.15) * cosphi - rect_y * sinphi + cpx;
		center_y = (rect_x-offset+0.15) * sinphi + rect_y * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius-0.2);
	}
	else if (circle_num==3){
		rect_x=length/2-(vehicle_parameters.LB+0.2);
		rect_y=0;

		center_x = rect_x * cosphi - rect_y * sinphi + cpx;
		center_y = rect_x * sinphi + rect_y * cosphi + cpy;
		std::pair<double,double> tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius);
		
		center_x = (rect_x+offset-0.45) * cosphi - rect_y * sinphi + cpx;
		center_y = (rect_x+offset-0.45) * sinphi + rect_y * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.insert(circle_list.begin(), tmp1);
		radius_list.push_back(radius);

		center_x = (rect_x-offset+0.45) * cosphi - rect_y * sinphi + cpx;
		center_y = (rect_x-offset+0.45) * sinphi + rect_y * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius);
	}
	else if (circle_num==8){
		rect_x=length/2-(vehicle_parameters.LB+0.2);
		rect_y=wid/4;
		
		center_x = (rect_x+3*offset-0.18) * cosphi - (rect_y-0.28) * sinphi + cpx;
		center_y = (rect_x+3*offset-0.18) * sinphi + (rect_y-0.28) * cosphi + cpy;
		std::pair<double,double> tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius+0.05);

		center_x = (rect_x+offset) * cosphi - rect_y * sinphi + cpx;
		center_y = (rect_x+offset) * sinphi + rect_y * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius);

		center_x = (rect_x-offset) * cosphi - rect_y * sinphi + cpx;
		center_y = (rect_x-offset) * sinphi + rect_y * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius);

		center_x = (rect_x-3*offset+0.18) * cosphi - (rect_y-0.28) * sinphi + cpx;
		center_y = (rect_x-3*offset+0.18) * sinphi + (rect_y-0.28) * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius+0.05);

		rect_y=-wid/4;

		center_x = (rect_x+3*offset-0.18) * cosphi - (rect_y+0.28) * sinphi + cpx;
		center_y = (rect_x+3*offset-0.18) * sinphi + (rect_y+0.28) * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius+0.05);

		center_x = (rect_x+offset) * cosphi - rect_y * sinphi + cpx;
		center_y = (rect_x+offset) * sinphi + rect_y * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius);

		center_x = (rect_x-offset) * cosphi - rect_y * sinphi + cpx;
		center_y = (rect_x-offset) * sinphi + rect_y * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius);

		center_x = (rect_x-3*offset+0.18) * cosphi - (rect_y+0.28) * sinphi + cpx;
		center_y = (rect_x-3*offset+0.18) * sinphi + (rect_y+0.28) * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius+0.05);
	}
	else{
		rect_x=length/2-(vehicle_parameters.LB+0.2);
		rect_y=wid/4;
		
		center_x = (rect_x+2*offset-0.15) * cosphi - (rect_y-0.2) * sinphi + cpx;
		center_y = (rect_x+2*offset-0.15) * sinphi + (rect_y-0.2) * cosphi + cpy;
		std::pair<double,double> tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius);
		
		center_x = (rect_x+offset+0.1) * cosphi - (rect_y) * sinphi + cpx;
		center_y = (rect_x+offset+0.1) * sinphi + (rect_y) * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius);

		center_x = rect_x * cosphi - (rect_y) * sinphi + cpx;
		center_y = rect_x * sinphi + (rect_y) * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius);

		center_x = (rect_x-offset-0.1) * cosphi - (rect_y) * sinphi + cpx;
		center_y = (rect_x-offset-0.1) * sinphi + (rect_y) * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius);

		center_x = (rect_x-2*offset+0.15) * cosphi - (rect_y-0.2) * sinphi + cpx;
		center_y = (rect_x-2*offset+0.15) * sinphi + (rect_y-0.2) * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius);

		rect_y=-wid/4;
		
		center_x = (rect_x+2*offset-0.15) * cosphi - (rect_y+0.2) * sinphi + cpx;
		center_y = (rect_x+2*offset-0.15) * sinphi + (rect_y+0.2) * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius);

		center_x = (rect_x+offset+0.1) * cosphi - (rect_y) * sinphi + cpx;
		center_y = (rect_x+offset+0.1) * sinphi + (rect_y) * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius);

		center_x = rect_x * cosphi - (rect_y) * sinphi + cpx;
		center_y = rect_x * sinphi + (rect_y) * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius);

		center_x = (rect_x-offset-0.1) * cosphi - (rect_y) * sinphi + cpx;
		center_y = (rect_x-offset-0.1) * sinphi + (rect_y) * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius);

		center_x = (rect_x-2*offset+0.15) * cosphi - (rect_y+0.2) * sinphi + cpx;
		center_y = (rect_x-2*offset+0.15) * sinphi + (rect_y+0.2) * cosphi + cpy;
		tmp1={center_x,center_y};
		circle_list.push_back(tmp1);
		radius_list.push_back(radius);
	}
	
	size_t idx = 0;
	for (const auto& circle : circle_list) {
		center_x=circle.first;
		center_y=circle.second;
		// 栅格化
		int xidx = ceil((pathfind_parameters.MAXX - center_x) / pathfind_parameters.XY_GRID_RESOLUTION);
		int yidx = ceil((pathfind_parameters.MAXY - center_y) / pathfind_parameters.XY_GRID_RESOLUTION);
		if (xidx == 0){
			xidx = 1;
		}
		if (yidx == 0){
			yidx = 1;
		}
		int idx_xy = (xidx - 1) * 250 + (yidx - 1);
		if ((xidx <= 0 || xidx > pathfind_parameters.XIDX) || (yidx <= 0 || yidx > pathfind_parameters.XIDY))
		{
			return true;
		}
		else {
			double dis = dis_map[(yidx-1)][249-(xidx-1)]*pathfind_parameters.XY_GRID_RESOLUTION;
			double radius = radius_list[idx++];
			if (dis<=radius)
			{
				return VehicleCollisionTreeSearch(cpx,cpy,cph,circle_idx+1,cosphi,sinphi,length,wid);
			}
		}
	}
	return false;
}

void Update(Node wknode, double End[3]) // 扩展点 ，并记录点，扩展close、OPEen集合
{
	// int minidx = 0;															  // 最小值下标
	char ilr = 0;															  // 左右转
	// double sres = vehicle_parameters.MAX_STEER / pathfind_parameters.N_STEER; // 转向分辨率
	double delta = -vehicle_parameters.MAX_STEER;							  // 右转最小转角
	double mres = pathfind_parameters.MOTION_RESOLUTION;					  // motino resolution    运动分辨率

	// all possible control input，所有可能的控制输入

	for (ilr = 0; ilr < find_steer_degree.size(); ilr++)
	{
		delta = find_steer_degree[ilr];
		if (CalcNextNode(wknode, -mres, delta) == false) // 输出全局变量的g_tnode，fcost=0；
		{
			continue;
		}
		str_idx = get_str_idx(g_tnode);
		if (g_closeset.find(str_idx) != g_closeset.end()) // exist
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
				g_openset[str_idx] = g_tnode;
				pq.push({str_idx, g_tnode.fcost});
			}
		}
		else
		{
			g_openset[str_idx] = g_tnode;
			pq.push({str_idx, g_tnode.fcost});
		}
	}

	for (ilr = 0; ilr < find_steer_degree.size(); ilr++)
	{
		delta = find_steer_degree[ilr];
		if (CalcNextNode(wknode, mres, delta) == false) // 输出全局变量的g_tnode，fcost=0；
		{
			continue;
		}
		str_idx = get_str_idx(g_tnode);
		if (g_closeset.find(str_idx) != g_closeset.end()) // exist
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
				g_openset[str_idx] = g_tnode;
				pq.push({str_idx, g_tnode.fcost});
			}
		}
		else
		{
			g_openset[str_idx] = g_tnode;
			pq.push({str_idx, g_tnode.fcost});
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
			ErrorCode = FindRSPath_CCS(x1, y1, ph, path);
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
			ErrorCode = FindRSPath_CCS(x1, y1, ph, path);
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
		if (fabs(mod2pi(g_pth) - End[2]) > pi / 180 * 3)
		{
			path.lenth = 0;
		}
	}
	// printf("path = %lf,%lf,%lf,%lf\n",rmin * path.t,rmin * path.u,rmin * path.v,rmin * path.lenth);
	return path;
}

void getFinalPath(RsPath path, std::string stridx)
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

std::string get_str_idx(Node &node)
{
	std::string str = "xy_yaw" + std::to_string(node.xidx) + "_" + std::to_string(node.yidx) + "_" + std::to_string(node.yawidx);
	return str;
}

std::string get_par_str_idx(Node &node)
{
	std::string str = "xy_yaw" + std::to_string(node.parent[0]) + "_" + std::to_string(node.parent[1]) + "_" + std::to_string(node.parent[2]);
	return str;
}

bool HybridAStar(double Start1[3], double End1[3])
{
	double End[3];
	double Start[3];
	double time_max = 0;
	printf("SEorES = %d\n", SEorES);

	if (SEorES == 0)
	{
		time_max = 100;
		Start[0] = Start1[0];
		Start[1] = Start1[1];
		Start[2] = Start1[2];
		End[0] = End1[0];
		End[1] = End1[1];
		End[2] = End1[2];
	}
	else
	{
		time_max = 800;
		Start[0] = End1[0];
		Start[1] = End1[1];
		Start[2] = End1[2];
		End[0] = Start1[0];
		End[1] = Start1[1];
		End[2] = Start1[2];
	}
	// printf("Start = %lf,%lf,%lf\n",Start[0],Start[1],Start[2]);
	// printf("End = %lf,%lf,%lf\n",End[0],End[1],End[2]);
	LOG_WARNING("Start=%lf,%lf,%lf", Start[0], Start[1], Start[2]);
	LOG_WARNING("End=%lf,%lf,%lf", End[0], End[1], End[2]);
	fflush(bydapa::common::Log::GetInstance()->fileptr());
	Theta_se = fabs(Start[2] - End[2]);
	Y_se = fabs(Start[1] - End[1]);
	// printf("Theta_se = %lf\n",Theta_se);
	// printf("Y_se = %lf\n",Y_se);
	Node tnode;			// 函数临时变量
	RsPath path;		// 函数临时变量
	g_openset.clear();	// open 集合清空
	g_closeset.clear(); // close 集合清空
	pathpoint.clear();	// 路 集合清空
	while (!pq.empty())
		pq.pop();

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
		int mid[3] = {g_xidx, g_yidx, g_thidx}; // 函数临时变量
		tnode = Node(g_xidx, g_yidx, g_thidx, 0, 0, Start[0], Start[1], Start[2], mid, 0, 0);
		tnode.fcost = TotalCost(tnode, End); // 起始点f

		str_idx = get_str_idx(tnode); // hybrid A*的Open集合
		pq.push({str_idx, tnode.fcost});
		g_openset[str_idx] = tnode;

		CloseSizeMax = 0;
		bydapa::common::TicToc tictoc;
		tictoc.tic();

		while (!g_openset.empty())
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

			g_tnode = g_openset[str_idx];
			pq.pop();
			g_openset.erase(str_idx);
			g_closeset[str_idx] = g_tnode;

			path = AnalysticExpantion(g_tnode, End); // 代价最小的OPEN节点的RS是否发生碰撞，如果碰撞则下一点，不碰撞则搜路结束
			if (path.lenth>0)
			{
				getFinalPath(path, str_idx);

				#ifdef OBCA
				std::vector<path_point> sampled_path = sample_path(pathpoint, 2);

				//=====================================================
				// std::ofstream file1("../path-ori.txt");
				// // 检查文件是否成功打开
				// if (file1.is_open()) {
				// 	// 循环遍历每个矩阵
				// 	for (const auto& pt : sampled_path) {
				// 		// 将矩阵的每一行写入文件
				// 		file1 << pt.x << ' ' << pt.y << ' ' << pt.th<< ' ' << pt.D << ' ' << pt.delta << std::endl;
				// 	}
				// 	// 关闭文件
				// 	file1.close();
				// 	std::cout << "Path written to path.txt" << std::endl;
				// } else {
				// 	std::cerr << "Unable to open file: path.txt" << std::endl;
				// }
				//=====================================================

				std::vector<Eigen::Vector3d> statelist;
				statelist.reserve(sampled_path.size()); // 提前分配内存

				for (const auto &point : sampled_path)
				{
					Eigen::Vector3d tmp(point.x, point.y, point.th);
					statelist.push_back(tmp);
				}
				std::vector<Eigen::MatrixXd> corridor = getRectangle(statelist);

				//=====================================================
				// std::ofstream file("../corridor.txt");
				// // 检查文件是否成功打开
				// if (file.is_open()) {
				// 	// 循环遍历每个矩阵
				// 	for (const auto& matrix : corridor) {
				// 		// 将矩阵的每一行写入文件
				// 		for (int i = 0; i < matrix.rows(); ++i) {
				// 			file << matrix.row(i) << std::endl;
				// 		}
				// 		file << std::endl;
				// 	}
				// 	// 关闭文件
				// 	file.close();
				// 	std::cout << "Matrices written to corridor.txt" << std::endl;
				// } else {
				// 	std::cerr << "Unable to open file: corridor.txt" << std::endl;
				// }
				//=====================================================

				Smoother smo(vehicle_parameters, sampled_path);
				smo.define_parameter_and_variable();
				smo.generate_constrain(corridor);
				smo.generate_variable();
				smo.generate_object();
				pathpoint = smo.solve();

				//=====================================================
				// std::ofstream file2("../path-opt.txt");
				// // 检查文件是否成功打开
				// if (file2.is_open()) {
				// 	// 循环遍历每个矩阵
				// 	for (const auto& pt : pathpoint) {
				// 		// 将矩阵的每一行写入文件
				// 		file2 << pt.x << ' ' << pt.y << ' ' << pt.th<< ' ' << pt.D << ' ' << pt.delta << std::endl;
				// 	}
				// 	// 关闭文件
				// 	file2.close();
				// 	std::cout << "Path written to path.txt" << std::endl;
				// } else {
				// 	std::cerr << "Unable to open file: path.txt" << std::endl;
				// }
				//=====================================================
				#endif

				// pathpoint.erase(pathpoint.begin(), pathpoint.begin() + pathpoint.size() - 300);
				if (aaa == 1)
				{
					LOG_WARNING("Start=%lf,%lf,%lf", Start[0], Start[1], Start[2]);
					LOG_WARNING("End=%lf,%lf,%lf", End[0], End[1], End[2]);
					fflush(bydapa::common::Log::GetInstance()->fileptr());
				}
				nseconds = tictoc.toc();
				nseconds = nseconds * 0.000000001;
				std::cout << "flag_CCS:" << flag_CCS << std::endl;
				std::cout << "flag_CCC:" << flag_CCC << std::endl;
				std::cout << "flag_CSC:" << flag_CSC << std::endl;
				std::cout << "flag_SCS:" << flag_SCS << std::endl;
				std::cout << "flag_CCCC:" << flag_CCCC << std::endl;
				std::cout << "flag_CCSC:" << flag_CCSC << std::endl;
				std::cout << "flag_CCSCC:" << flag_CCSCC << std::endl;

				std::cout << "flag_1:" << flag_1 << std::endl;
				std::cout << "flag_2:" << flag_2 << std::endl;
				std::cout << "flag_3:" << flag_3 << std::endl;
				std::cout << "flag_5:" << flag_5 << std::endl;
				std::cout << "flag_8:" << flag_8 << std::endl;
				std::cout << "flag_10:" << flag_10 << std::endl;
				std::cout << pathpoint.size() << std::endl;
				return true;
			}

			Update(g_tnode, End);

			if (CloseSizeMax < (int)g_closeset.size())
			{
				CloseSizeMax = (int)g_closeset.size();
			}
			/*
			if (CloseSizeMax > 20000)
			{
				return false;
			}*/
		}
	}
	else
	{
		ErrorCode = 1;
		LOG_ERROR("ErrorCodeCalcIdxStart=%d", ErrorCode); // 起始点不在地图范围内
	}
	
	g_openset.clear();	// open集合清空
	g_closeset.clear(); // close 集合清空
	return false;		// 搜不到路
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
	CCC(x, y, phi, path);
	if (path.lenth < tmp)
	{
		tmp = path.lenth;
		cnt = 2;
	}
	
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

int FindRSPath_CCS(double x, double y, double phi, RsPath &path)
{
	//	RsPath path;
	double rmin = vehicle_parameters.MIN_CIRCLE;
	path.lenth = 1000;
	x = x / rmin;
	y = y / rmin;

	double tmp = path.lenth;
	CCS(x, y, phi, path);
	if (path.lenth < tmp)
	{
		tmp = path.lenth;
	}
	flag_CCS += 1;
	
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
	if (v < -pi)
		v += twopi;
	else if (v > pi)
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
	double xi = x + sin(phi);
	double eta = y - 1. - cos(phi);
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
	double xi = x + sin(phi);
	double eta = y - 1. - cos(phi);
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