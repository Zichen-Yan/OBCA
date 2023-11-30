#include "findpath.h"
////////////全局变量定义/////////////////////////////////////////////

int RoadCollision_CC(double Start[3], double End[3], double road_yaw[6]);
void getroadpath_CC(double Start[3], double road_yaw[6]);
char angleget_CC(double sin_result, double cos_result, double* last_angle_result);
char road_find_two_CC(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_two[6]);
int CC_road(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_two[6]);
///////类定义/////////////////

void getroadpath_CC(double Start[3], double road_yaw[6])
{
	path_point mid_L;
	mid_L.x = Start[0];
	mid_L.y = Start[1];
	mid_L.th = mod2pi(Start[2]);
	mid_L.D = 0;
	mid_L.delta = 0;
	pathpoint.push_back(mid_L);
	double D_L = 0;
	double D_r = 0;
	double D_s = 0;
	double delta_L = 0;
	double delta_r = 0;
	double delta_s = 0;
	if (road_yaw[1] > 0)
	{
		D_L = 0.1;
	}
	else if (road_yaw[1] < 0)
	{
		D_L = -0.1;
	}
	if (road_yaw[3] > 0)
	{
		D_r = 0.1;
	}
	else if (road_yaw[3] < 0)
	{
		D_r = -0.1;
	}
	if (road_yaw[5] > 0)
	{
		D_s = 0.1;
	}
	else if (road_yaw[5] < 0)
	{
		D_s = -0.1;
	}
	if (fabs(road_yaw[0]) > 100000 || fabs(road_yaw[0]) < R_C)
	{
		delta_L = 0;
	}
	else
	{
		delta_L = atan(vehicle_parameters.WB / road_yaw[0]);
	}
	if (fabs(road_yaw[2]) > 100000 || fabs(road_yaw[2]) < R_C)
	{
		delta_r = 0;
	}
	else
	{
		delta_r = atan(vehicle_parameters.WB / road_yaw[2]);
	}
	if (fabs(road_yaw[4]) > 100000 || fabs(road_yaw[4]) < R_C)
	{
		delta_s = 0;
	}
	else
	{
		delta_s = atan(vehicle_parameters.WB / road_yaw[4]);
	}
	for (int idx_L = 0; idx_L < round(fabs(road_yaw[1]) / pathfind_parameters.MOTION_RESOLUTION); idx_L++) // round()四舍五入
	{
		VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D_L, delta_L); // 根据当前位姿和输入, 计算下一位置的位姿
		mid_L.x = g_px;
		mid_L.y = g_py;
		mid_L.th = g_pth;
		mid_L.D = D_L;
		mid_L.delta = delta_L;
		pathpoint.push_back(mid_L);
	}
	for (int idx_r = 0; idx_r < round(fabs(road_yaw[3]) / pathfind_parameters.MOTION_RESOLUTION); idx_r++) // round()四舍五入
	{
		VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D_r, delta_r); // 根据当前位姿和输入, 计算下一位置的位姿
		mid_L.x = g_px;
		mid_L.y = g_py;
		mid_L.th = g_pth;
		mid_L.D = D_r;
		mid_L.delta = delta_r;
		pathpoint.push_back(mid_L);
	}
	for (int idx_s = 0; idx_s < round(fabs(road_yaw[5]) / pathfind_parameters.MOTION_RESOLUTION); idx_s++) // round()四舍五入
	{
		VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D_s, delta_s); // 根据当前位姿和输入, 计算下一位置的位姿
		mid_L.x = g_px;
		mid_L.y = g_py;
		mid_L.th = g_pth;
		mid_L.D = D_s;
		mid_L.delta = delta_s;
		pathpoint.push_back(mid_L);
	}
	//vehicle_parameters.MIN_CIRCLE = vehicle_parameters.WB / fabs(tan(delta_L));
}

int RoadCollision_CC(double Start[3], double End[3], double road_yaw[6])
{
	path_point mid_L;
	mid_L.x = Start[0];
	mid_L.y = Start[1];
	mid_L.th = mod2pi(Start[2]);
	double D_L = 0;
	double D_r = 0;
	double D_s = 0;
	double delta_L = 0;
	double delta_r = 0;
	double delta_s = 0;
	if (road_yaw[1] > 0)
	{
		D_L = 0.1;
	}
	else if(road_yaw[1] < 0)
	{
		D_L = -0.1;
	}
	if (road_yaw[3] > 0)
	{
		D_r = 0.1;
	}
	else if (road_yaw[3] < 0)
	{
		D_r = -0.1;
	}
	if (road_yaw[5] > 0)
	{
		D_s = 0.1;
	}
	else if (road_yaw[5] < 0)
	{
		D_s = -0.1;
	}
	if (fabs(road_yaw[0]) < R_C)
	{
		delta_L = 0;
	}
	else
	{
		delta_L = atan(vehicle_parameters.WB / road_yaw[0]);
	}
	if (fabs(road_yaw[2]) < R_C)
	{
		delta_r = 0;
	}
	else
	{
		delta_r = atan(vehicle_parameters.WB / road_yaw[2]);
	}
	if (fabs(road_yaw[4]) < R_C)
	{
		delta_s = 0;
	}
	else
	{
		delta_s = atan(vehicle_parameters.WB / road_yaw[4]);
	}
	int roadisCollision = 0;
	for (int idx_L = 0; idx_L < round(fabs(road_yaw[1]) / pathfind_parameters.MOTION_RESOLUTION); idx_L++) // round()四舍五入
	{
		VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D_L, delta_L); // 根据当前位姿和输入, 计算下一位置的位姿
		mid_L.x = g_px;
		mid_L.y = g_py;
		mid_L.th = g_pth;
		mid_L.D = D_L;
		mid_L.delta = delta_L;
		roadisCollision = VehicleCollisionGrid(g_px, g_py, g_pth);
		if (roadisCollision)
		{
			return 1;
		}
	}
	for (int idx_r = 0; idx_r < round(fabs(road_yaw[3]) / pathfind_parameters.MOTION_RESOLUTION); idx_r++) // round()四舍五入
	{
		VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D_r, delta_r); // 根据当前位姿和输入, 计算下一位置的位姿
		mid_L.x = g_px;
		mid_L.y = g_py;
		mid_L.th = g_pth;
		mid_L.D = D_r;
		mid_L.delta = delta_r;
		roadisCollision = VehicleCollisionGrid(g_px, g_py, g_pth);
		if (roadisCollision)
		{
			return 1;
		}
	}
	for (int idx_s = 0; idx_s < round(fabs(road_yaw[5]) / pathfind_parameters.MOTION_RESOLUTION); idx_s++) // round()四舍五入
	{
		VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D_s, delta_s); // 根据当前位姿和输入, 计算下一位置的位姿
		mid_L.x = g_px;
		mid_L.y = g_py;
		mid_L.th = g_pth;
		mid_L.D = D_s;
		mid_L.delta = delta_s;
		roadisCollision = VehicleCollisionGrid(g_px, g_py, g_pth);
		if (roadisCollision)
		{
			return 1;
		}
	}

	return 0;
}

char angleget_CC(double sin_result, double cos_result, double* last_angle_result)
{
	if (sin_result > 1 || cos_result > 1)//如果正弦余弦值大于1，则返回1，超界
	{
		return 1;
	}
	if (cos_result >= 0)
	{
		last_angle_result[0] = asin(sin_result);
	}
	else if (cos_result < 0 && sin_result <= 0)
	{
		last_angle_result[0] = -1 * pi - asin(sin_result);
	}
	else
	{
		last_angle_result[0] = acos(cos_result);
	}
	return 0;
}

////////////////////////////////////////求解两端路径////////////////////////////////////////////////////////////////////////////////
//x_now  y_now  thea_now  车辆当前点x，y，thea坐标
//x_aim  y_aim  thea_aim  车辆终点x，y，thea坐标
//car_r_limit         最小转弯半径，小于此值无解
//road_two[0] 第一段转弯半径，road_two[1] 第一段距离，前进为正后退为负，road_two[2] 第二段转弯半径，road_two[3] 第二段距离，前进为正后退为负，
//return 0代表有解 
char road_find_two_CC(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_two[6])
{
	//////////////////////////////车辆坐标转换处理/////////////////////////////////////////////////////////////////////
	double y = (y_aim - y_now) * cos(theta_now) - (x_aim - x_now) * sin(theta_now);// 坐标转换后 Y向的实际偏差
	double x = (x_aim - x_now) * cos(theta_now) + (y_aim - y_now) * sin(theta_now);
	double theta = 0;
	theta = mod2pi(theta_aim - theta_now);

	//////////////////////////////距离太短，不做输出求解,保护后面数据不越界//////////////////////////////////////////////////////////
	if (fabs(x) < 0.05 || fabs(y) > 1000 || fabs(x) > 1000)
	{
		return 2;//距离目标小于5cm
	}
	double a;
	double b;
	double c;
	double sqr;
	//double car_r_limit = 5.5;
	double R_1 = 0;
	double sin_theta_first_1 = 0;//车辆第一个旋转角度的sin值
	double cos_theta_first_1 = 0;//车辆第一个旋转角度的cos值
	double angle_1[1] = { 0 };
	double distance1 = 0;
	double R_2 = 0;
	double sin_theta_first_2 = 0;//车辆第一个旋转角度的sin值
	double cos_theta_first_2 = 0;//车辆第一个旋转角度的cos值
	double angle_2[1] = { 0 };
	double distance2 = 0;
	///////////////////计算公式///////////
	a = pow(cos(theta) + 1, 2) + pow(sin(theta), 2) - 4;
	b = 2 * x * sin(theta) - 2 * y * (cos(theta) + 1);
	c = x * x + y * y;
	sqr = pow(b, 2) - 4 * a * c;
	if (fabs(a) < 0.0001)
	{
		a = 0;
	}
	///////////////////////////////////////均方根为负值则无解//////////////////////////////////////////////
	if (sqr < 0)
	{
		return 1;
	}
	///////////////////////////////////////a为0，b为0则无解//////////////////////////////////////////////
	if (a == 0 && b == 0)
	{
		return 1;
	}
	/////////////////////////////////////a=0，单独计算求解//////////////////////////////////////
	if (a == 0)
	{
		if (fabs(c * 0.000001) >= fabs(b))   //保护除数
		{
			R_1 = 1000000;
		}
		else
		{
			R_1 = -c / b;
		}
		if (fabs(R_1) < car_r_limit)
		{
			return 1;//无解
		}
		sin_theta_first_1 = x / 2 / (R_1)+sin(theta) / 2;
		cos_theta_first_1 = -y / 2 / (R_1)+cos(theta) / 2 + 0.5;
		if (angleget_CC(sin_theta_first_1, cos_theta_first_1, angle_1) != 0)//无解
		{
			return 1;
		}
		////////////////////可删除下面两行验证用/////////////////////////////////////////
		double aaa_x1 = 2 * R_1 * sin(angle_1[0]) - R_1 * sin(0.0);
		double aaa_y1 = R_1 - 2 * R_1 * cos(angle_1[0]) + R_1 * cos(0.0);
		if (angle_1[0] * (theta - angle_1[0]) > 0)//两端路必须为同一方向
		{
			return 1;//无解
		}
		double distance1 = fabs(R_1 * angle_1[0]) + fabs(R_1 * (theta - angle_1[0]));//可以删除，验证用
		road_two[0] = R_1;
		road_two[1] = R_1 * angle_1[0];
		road_two[2] = -R_1;
		road_two[3] = -R_1 * (theta - angle_1[0]);
		return 0;
	}

	////一元二次方程两个解，求解1
	if (fabs((-b + pow(sqr, 0.5)) * 0.000002) >= fabs(a))   //保护除数
	{
		R_1 = 1000000;
	}
	else
	{
		R_1 = (-b + pow(sqr, 0.5)) / 2 / a;
	}
	if (fabs(R_1) >= car_r_limit)
	{
		sin_theta_first_1 = x / 2 / (R_1)+sin(theta) / 2;
		cos_theta_first_1 = -y / 2 / (R_1)+cos(theta) / 2 + 0.5;
		if (angleget_CC(sin_theta_first_1, cos_theta_first_1, angle_1) != 0)
		{
			return 1;
		}
		double distance1 = fabs(R_1 * angle_1[0]) + fabs(R_1 * (theta - angle_1[0]));
	}
	////一元二次方程两个解，求解2
	if (fabs((-b - pow(sqr, 0.5)) * 0.000002) >= fabs(a))   //保护除数
	{
		R_2 = 1000000;
	}
	else
	{
		R_2 = (-b - pow(sqr, 0.5)) / 2 / a;
	}
	if (fabs(R_2) >= car_r_limit)
	{
		sin_theta_first_2 = x / 2 / (R_2)+sin(theta) / 2;
		cos_theta_first_2 = -y / 2 / (R_2)+cos(theta) / 2 + 0.5;
		if (angleget_CC(sin_theta_first_2, cos_theta_first_2, angle_2) != 0)
		{
			return 1;
		}
		double distance2 = fabs(R_2 * angle_2[0]) + fabs(R_2 * (theta - angle_2[0]));
	}
	if (fabs(R_1) >= car_r_limit && fabs(R_2) >= car_r_limit)//两个解都有，取同一方向
	{
		if ((angle_1[0]) * (theta - angle_1[0]) <= 0)
		{
			road_two[0] = R_1;
			road_two[1] = R_1 * angle_1[0];
			road_two[2] = -R_1;
			road_two[3] = -R_1 * (theta - angle_1[0]);
		}
		else if ((angle_2[0]) * (theta - angle_2[0]) <= 0)
		{
			road_two[0] = R_2;
			road_two[1] = R_2 * angle_2[0];
			road_two[2] = -R_2;
			road_two[3] = -R_2 * (theta - angle_2[0]);
		}
		else
		{
			return 1;
		}
	}
	else if (fabs(R_1) < car_r_limit && fabs(R_2) < car_r_limit)
	{
		return 1;
	}

	else if (fabs(R_1) >= car_r_limit)
	{
		if ((angle_1[0]) * (theta - angle_1[0]) <= 0)
		{
			road_two[0] = R_1;
			road_two[1] = R_1 * angle_1[0];
			road_two[2] = -R_1;
			road_two[3] = -R_1 * (theta - angle_1[0]);
		}
		else
		{
			return 1;
		}
	}
	else
	{
		if ((angle_2[0]) * (theta - angle_2[0]) <= 0)
		{
			road_two[0] = R_2;
			road_two[1] = R_2 * angle_2[0];
			road_two[2] = -R_2;
			road_two[3] = -R_2 * (theta - angle_2[0]);
		}
		else
		{
			return 1;
		}
	}
	return 0;
}

int CC_road(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_two[6])
{
	double S_distance = 0.3;
	double y = (y_now - y_aim) * cos(theta_aim) - (x_now - x_aim) * sin(theta_aim);// 坐标转换后 Y向的实际偏差
	double x = (x_now - x_aim) * cos(theta_aim) + (y_now - y_aim) * sin(theta_aim);
	double theta = 0;
	theta = mod2pi(theta_now - theta_aim);
	double End_before[3] = { S_distance,0,0 };

	double road_CC[6] = { 0,0,0,0,0,0 };
	int road_id = 1;
	double Start_imag[3] = { 0,0,0 };
	double End_imag[3] = { 0,0,0 };
	Start_imag[0] = x_now;
	Start_imag[1] = y_now;
	Start_imag[2] = theta_now;
	End_imag[0] = x_aim;
	End_imag[1] = y_aim;
	End_imag[2] = theta_aim;
	double End_diff[7] = { 0, 0.02, -0.02, 0.04, -0.04, 0.06, -0.06 };
	for (int idx = 0; idx < 7; idx++)
	{
		road_id = road_find_two_CC(x, y, theta, End_before[0], End_before[1]+ End_diff[idx], End_before[2], R_, road_CC);
		if ((road_id == 0) && (road_CC[1] < 0) && (road_CC[3] < 0))
		{
			road_CC[4] = 1000000;
			road_CC[5] = -1*S_distance;
			road_id = RoadCollision_CC(Start_imag, End_imag, road_CC);
			if (road_id == 0)
			{
				road_two[0] = road_CC[0];
				road_two[1] = road_CC[1];
				road_two[2] = road_CC[2];
				road_two[3] = road_CC[3];
				road_two[4] = road_CC[4];
				road_two[5] = road_CC[5];
				return 0;
			}
		}
		else
		{
			road_CC[0] = 0;
			road_CC[1] = 0;
			road_CC[2] = 0;
			road_CC[3] = 0;
			road_CC[4] = 0;
			road_CC[5] = 0;
		}
	}
	
	return 1;
}

int  dynamic_CC_road(double Start1[3], double End1[3])
{
	pathpoint.clear();

	double End[3];
	double Start[3];

	Start[0] = Start1[0];
	Start[1] = Start1[1];
	Start[2] = Start1[2];
	End[0] = End1[0];
	End[1] = End1[1];
	End[2] = End1[2];
	
	double road_yaw[6] = { 0,0,0,0,0,0 };
	
	int road_ID = 1;
	road_ID = CC_road(Start[0], Start[1], Start[2], End[0], End[1], End[2], R_, road_yaw);
	if (road_ID == 0)
	{
		road_ID = RoadCollision_CC(Start, End, road_yaw);
		if (road_ID == 0)
		{
			LOG_WARNING("road_ID=%d", road_ID);
			fflush(bydapa::common::Log::GetInstance()->fileptr());
			getroadpath_CC(Start, road_yaw);
			return 1;
		}
	}

	return 0;
}
