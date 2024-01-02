#include "findpath.h"
////////////全局变量定义/////////////////////////////////////////////

int RoadCollision_multi(double Start[3], double End[3], double road_yaw[8]);
void getroadpath_multi(double Start[3], double road_yaw[8]);
char angleget_multi(double sin_result, double cos_result, double* last_angle_result);

double road_find_first_multi(double x_now, double y_now, double thea_now, double x_aim, double y_aim, double thea_aim, double y_tolerance, double min_r, double yaw_tolerance, double road_yaw[8]);
char road_find_two_multi(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_two[6]);
char road_find_three_cs_multi(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double bias, double road_R_S[8], double road_R_S2[8]);


void getroadpath_multi(double Start[3], double road_yaw[8])
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
	double D_a = 0;
	double delta_L = 0;
	double delta_r = 0;
	double delta_s = 0;
	double delta_a = 0;
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
	if (road_yaw[7] > 0)
	{
		D_a = 0.1;
	}
	else if (road_yaw[7] < 0)
	{
		D_a = -0.1;
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
	if (fabs(road_yaw[6]) > 100000 || fabs(road_yaw[6]) < R_C)
	{
		delta_a = 0;
	}
	else
	{
		delta_a = atan(vehicle_parameters.WB / road_yaw[6]);
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
	for (int idx_a = 0; idx_a < round(fabs(road_yaw[7]) / pathfind_parameters.MOTION_RESOLUTION); idx_a++) // round()四舍五入
	{
		VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D_a, delta_a); // 根据当前位姿和输入, 计算下一位置的位姿
		mid_L.x = g_px;
		mid_L.y = g_py;
		mid_L.th = g_pth;
		mid_L.D = D_a;
		mid_L.delta = delta_a;
		pathpoint.push_back(mid_L);
	}
}

int RoadCollision_multi(double Start[3], double End[3], double road_yaw[8])
{
	path_point mid_L;
	mid_L.x = Start[0];
	mid_L.y = Start[1];
	mid_L.th = mod2pi(Start[2]);
	double D_L = 0;
	double D_r = 0;
	double D_s = 0;
	double D_a = 0;
	double delta_L = 0;
	double delta_r = 0;
	double delta_s = 0;
	double delta_a = 0;
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
	if (road_yaw[7] > 0)
	{
		D_a = 0.1;
	}
	else if (road_yaw[7] < 0)
	{
		D_a = -0.1;
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
	if (fabs(road_yaw[4]) < R_C) //
	{
		delta_s = 0;
	}
	else
	{
		delta_s = atan(vehicle_parameters.WB / road_yaw[4]);
	}
	if (fabs(road_yaw[6]) < R_C)
	{
		delta_a = 0;
	}
	else
	{
		delta_a = atan(vehicle_parameters.WB / road_yaw[6]);
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
	for (int idx_a = 0; idx_a < round(fabs(road_yaw[7]) / pathfind_parameters.MOTION_RESOLUTION); idx_a++) // round()四舍五入
	{
		VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D_a, delta_a); // 根据当前位姿和输入, 计算下一位置的位姿
		mid_L.x = g_px;
		mid_L.y = g_py;
		mid_L.th = g_pth;
		mid_L.D = D_a;
		mid_L.delta = delta_a;
		roadisCollision = VehicleCollisionGrid(g_px, g_py, g_pth);
		if (roadisCollision)
		{
			return 1;
		}
	}

	return 0;
}

double road_find_first_multi(double x_now, double y_now, double thea_now, double x_aim, double y_aim, double thea_aim, double y_tolerance, double min_r, double yaw_tolerance, double road_yaw[8])  //根据车辆当前点及目标点计算转弯半径
{
	double tolerance_y = 0;//纠正偏航角偏差后横向距离误差
	double tolerance_y_a = 0;
	double tolerance_y_b = 0;
	double r_yaw = 0;//纠正偏航角需要的转弯半径
	double r_yaw_a = 0;
	double r_yaw_b = 0;
	double mid_thea;
	double r = 0;
	//double angle_out = 0;//输出的方向盘角度
	yaw_tolerance = yaw_tolerance * M_PI / 180;
	//double aim_r;//当前目标方向盘角度转弯半径单位 cm
	//double aim_y;       //当前目标方向盘角度y向偏差
	//double aim_yaw;
	//////////////////////////////////////////坐标转换，输出车辆当前坐标为原点的x坐标y坐标及角度////////////////////////////////////////////////////////////
	double y = (y_aim - y_now) * cos(thea_now) - (x_aim - x_now) * sin(thea_now);// 坐标转换后 Y向的实际偏差
	double x = (x_aim - x_now) * cos(thea_now) + (y_aim - y_now) * sin(thea_now);
	double aim_now_thea_tolerance = 0;
	aim_now_thea_tolerance = mod2pi(thea_aim - thea_now);
	////////如果距离目标太近，则无需调整//////////////////////////////////////////////////////////////////////////////////////////////////////
	if (fabs(x) < 0.05)
	{
		return 2;//距离目标小于5cm
	}
	//////////////////////////////////////////纠正偏航角计算方向盘角度////////////////////////////////////////////////////////////
	mid_thea = aim_now_thea_tolerance;
	if (fabs(x) * 0.000001 >= fabs(sin(mid_thea)))
	{
		r_yaw = 1000000;
	}
	else
	{
		r_yaw = x / sin(mid_thea);
	}
	tolerance_y = r_yaw - r_yaw * cos(mid_thea) - y;
	//////////////////////////////////////////纠正偏航角计算方向盘角度,-0.5///////////////////////////////////////////////////////
	mid_thea = aim_now_thea_tolerance - yaw_tolerance;
	if (fabs(x) * 0.000001 >= fabs(sin(mid_thea)))
	{
		r_yaw_a = 1000000;
	}
	else
	{
		r_yaw_a = x / sin(mid_thea);
	}
	tolerance_y_a = r_yaw_a - r_yaw_a * cos(mid_thea) - y;
	//////////////////////////////////////////纠正偏航角计算方向盘角度,+0.5///////////////////////////////////////////////////////
	mid_thea = aim_now_thea_tolerance + yaw_tolerance;
	if (fabs(x) * 0.000001 >= fabs(sin(mid_thea)))
	{
		r_yaw_b = 1000000;
	}
	else
	{
		r_yaw_b = x / sin(mid_thea);
	}
	tolerance_y_b = r_yaw_b - r_yaw_b * cos(mid_thea) - y;
	//////////////////////方便后面运算//////////////////////////////////////////////////////////////////////
	if (fabs(r_yaw) < min_r)
	{
		tolerance_y = 10;
	}
	if (fabs(r_yaw_a) < min_r)
	{
		tolerance_y_a = 10;
	}
	if (fabs(r_yaw_b) < min_r)
	{
		tolerance_y_b = 10;
	}
	/////////////////////////////////////如果横向误差都大于yaw_tolerance，则无解//////////////////////////////
	if (fabs(tolerance_y) > y_tolerance && fabs(tolerance_y_a) > y_tolerance && fabs(tolerance_y_b) > y_tolerance)
	{
		return 1;//y向误差太大或者转弯半径小于车辆要求
	}
	//////////////////////////////////////////选择最小的y向误差////////////////////////////////////////////////
	if (fabs(tolerance_y) <= fabs(tolerance_y_a) && fabs(tolerance_y) <= fabs(tolerance_y_b))
	{
		road_yaw[0] = r_yaw;
		if (r_yaw == 1000000)
		{
			road_yaw[1] = x;
		}
		else if (x > 0)
		{
			road_yaw[1] = fabs(r_yaw) * fabs(aim_now_thea_tolerance);//车需要后退
		}
		else
		{
			road_yaw[1] = -fabs(r_yaw) * fabs(aim_now_thea_tolerance);//车需要后退
		}
	}
	else if (fabs(tolerance_y_a) <= fabs(tolerance_y) && fabs(tolerance_y_a) <= fabs(tolerance_y_b))
	{
		road_yaw[0] = r_yaw_a;
		if (r_yaw_a == 1000000)
		{
			road_yaw[1] = x;
		}
		else if (x > 0)
		{
			road_yaw[1] = fabs(r_yaw_a) * fabs(aim_now_thea_tolerance - yaw_tolerance);//车需要后退
		}
		else
		{
			road_yaw[1] = -fabs(r_yaw_a) * fabs(aim_now_thea_tolerance - yaw_tolerance);//车需要后退
		}
	}
	else
	{
		road_yaw[0] = r_yaw_b;
		if (r_yaw_b == 1000000)
		{
			road_yaw[1] = x;
		}
		else if (x > 0)
		{
			road_yaw[1] = fabs(r_yaw_b) * fabs(aim_now_thea_tolerance + yaw_tolerance);//车需要后退
		}
		else
		{
			road_yaw[1] = -fabs(r_yaw_b) * fabs(aim_now_thea_tolerance + yaw_tolerance);//车需要后退
		}
	}
	return 0;
}

char angleget_multi(double sin_result, double cos_result, double* last_angle_result)
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
		last_angle_result[0] = -1 * M_PI - asin(sin_result);
	}
	else
	{
		last_angle_result[0] = acos(cos_result);
	}
	return 0;
}

char road_find_two_multi(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_two[6])
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
		if (angleget_multi(sin_theta_first_1, cos_theta_first_1, angle_1) != 0)//无解
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
		if (angleget_multi(sin_theta_first_1, cos_theta_first_1, angle_1) != 0)
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
		if (angleget_multi(sin_theta_first_2, cos_theta_first_2, angle_2) != 0)
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

char road_find_three_cs_multi(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double bias, double *road_R_S, double *road_R_S2)
{
	double l_back = 0;  //直线后退的值
	double y_1 = 0;     //走完第一段弧的y值
	double x_1 = 0;     //走完第一段弧的y值
	//////////////////////////////车辆坐标转换处理/////////////////////////////////////////////////////////////////////
	double thea_aim1 = theta_aim / 180 * M_PI;
	//double theta_now1 = (theta_now - theta_aim) / 180 * 3.1415926;
	double y = (y_now - y_aim)*cos(thea_aim1) - (x_now - x_aim)*sin(thea_aim1);// 坐标转换后 Y向的实际偏差
	double x = (x_now - x_aim)*cos(thea_aim1) + (y_now - y_aim)*sin(thea_aim1);
	double theta_ang = theta_now - theta_aim;
	//////
	theta_ang = fmod(theta_ang, 360);
	if (theta_ang > 180)
		theta_ang = theta_ang - 360;
	else if (theta_ang <-180)
		theta_ang = theta_ang + 360;
	double theta_rad = theta_ang / 180 * M_PI;
	// 车辆方向与车位夹角
	if (fabs(theta_ang)>90)  //当前角度与目标角度差值较大，cs不一定是最优解,60参数可以调整
	{
		return 10;
	}
	////////////////////////////////允许y存在误差，查看是否有解？？？？？？////////////////////////////////////////////////
	//偏差允许有解：方向盘先向左打，然后把车辆调成和目标一致，查看圆弧直线是否需要换挡
	double car_r_limit_m = car_r_limit;
	x_1 = x - car_r_limit_m*sin(theta_rad);
	y_1 = y + car_r_limit_m*cos(theta_rad) - car_r_limit_m;   //把方向盘角度调整为和目标一致！
	if (car_r_limit_m*(-theta_rad)*x_1<0)//圆弧的解无需换挡
	{
		if (y*y_1 < 0)   //最小转弯半径也照样调整不到位,一个在y=0的左边，一个在右边
		{
			if (fabs(y_1) < bias)//是否在误差允许范围内
			{
				road_R_S[0] = car_r_limit_m;
				road_R_S[1] = car_r_limit_m*(-theta_rad);
				road_R_S[2] = 1000000;
				road_R_S[3] = -x_1;
				road_R_S2[0] = car_r_limit_m;
				road_R_S2[1] = car_r_limit_m*(-theta_rad);
				double road_yaw_cs[8] = { 0,0,0,0,0,0,0,0 };
				int roadthird_multi_CS = road_find_two_multi(x_1, y_1, 0, 0, 0, 0, R_, road_yaw_cs);
				if (roadthird_multi_CS == 0)
				{
					road_R_S2[2] = road_yaw_cs[0];
					road_R_S2[3] = road_yaw_cs[1];
					road_R_S2[4] = road_yaw_cs[2];
					road_R_S2[5] = road_yaw_cs[3];
				}
				else
				{
					road_R_S2[2] = car_r_limit_m;
					road_R_S2[3] = car_r_limit_m * -asin(x_1*0.5 / car_r_limit_m);
					road_R_S2[4] = -car_r_limit_m;
					road_R_S2[5] = car_r_limit_m * -asin(x_1*0.5 / car_r_limit_m);
				}
				return 1;
			}
			else
			{
				return 2;//无解
			}
		}
	}
	else
	{
		//无需处理
	}
	//偏差允许有解：方向盘向右打，然后把车辆调成和目标一致，查看圆弧直线是否需要换挡
	car_r_limit_m = -car_r_limit;
	x_1 = x - car_r_limit_m*sin(theta_rad);
	y_1 = y + car_r_limit_m*cos(theta_rad) - car_r_limit_m;   //把方向盘角度调整为和目标一致！
	if (car_r_limit_m*(-theta_rad)*x_1 <0)//圆弧的解无需换挡
	{
		if (y*y_1 < 0)   //最小转弯半径也照样调整不到位
		{
			if (fabs(y_1) < bias)//是否在误差允许范围内
			{
				road_R_S[0] = car_r_limit_m;
				road_R_S[1] = car_r_limit_m*(-theta_rad);
				road_R_S[2] = 1000000;
				road_R_S[3] = -x_1;
				road_R_S2[0] = car_r_limit_m;
				road_R_S2[1] = car_r_limit_m*(-theta_rad);
				double road_yaw_cs[8] = { 0,0,0,0,0,0,0,0 };
				int roadthird_multi_CS = road_find_two_multi(x_1, y_1, 0, 0, 0, 0, R_, road_yaw_cs);
				if (roadthird_multi_CS == 0)
				{
					road_R_S2[2] = road_yaw_cs[0];
					road_R_S2[3] = road_yaw_cs[1];
					road_R_S2[4] = road_yaw_cs[2];
					road_R_S2[5] = road_yaw_cs[3];
				}
				else
				{
					road_R_S2[2] = car_r_limit_m;
					road_R_S2[3] = car_r_limit_m * -asin(x_1*0.5 / car_r_limit_m);
					road_R_S2[4] = -car_r_limit_m;
					road_R_S2[5] = car_r_limit_m * -asin(x_1*0.5 / car_r_limit_m);
				}
				return 1;
			}
			else
			{
				return 2;//无解
			}
		}
	}
	else
	{
		//无需处理
	}
	if (x<0.05 && fabs(y)>19)         //越界保护考虑
	{
		return 2;//越界不在求解
	}
	if (cos(theta_rad)>0.999999) //如果车辆当前角度与目标角度一致，那么其他公式有解，保护不越界
	{
		return 2;//无解
	}

	road_R_S[0] = y / (1 - cos(theta_rad));//计算转弯半径

	if (fabs(road_R_S[0]) < car_r_limit)//最小转弯半径小于车辆实际转弯半径
	{
		return 2;//无解
	}
	x_1 = x - road_R_S[0] * sin(theta_rad);
	y_1 = y + road_R_S[0] * cos(theta_rad) - road_R_S[0];   //把方向盘角度调整为和目标一致！
	if (road_R_S[0] * (-theta_rad)*x_1 >0)//圆弧的解如果需要换挡，则无解
	{
		return 2;//无解
	}

	road_R_S[1] = road_R_S[0] * (-theta_rad);
	road_R_S[2] = 1000000;
	road_R_S[3] = -x_1;
	return 0;
}


int  dynamic_vertical_multi(double Start1[3], double End1[3])
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
	
	double road_yaw[8] = { 0,0,0,0,0,0,0,0 };
	double road_yaw2[8] = { 0,0,0,0,0,0,0,0 };

	//C-
	int roadfirst_multi = 1;
	roadfirst_multi = road_find_first_multi(Start[0], Start[1], Start[2], End[0], End[1], End[2], 0.05, R_, 0.3, road_yaw); // 2022.12.06:5.8修改为5.3
	if (roadfirst_multi == 0)
	{
		roadfirst_multi = RoadCollision_multi(Start, End, road_yaw);
		if (roadfirst_multi == 0)
		{
			LOG_WARNING("roadfirst_multi=%d", roadfirst_multi);
			//fflush(bydapa::common::Log::GetInstance()->fileptr());
			getroadpath_multi(Start, road_yaw);
			return 1;
		}
	}
	//C-S-
	int roadsecond_multi = 1;
	roadsecond_multi = road_find_three_cs_multi(Start[0], Start[1], Start[2] * 180 / M_PI, End[0], End[1], End[2] * 180 / M_PI, R_, 0.1, road_yaw, road_yaw2); // 2022.12.06:5.8修改为5.3
	if (roadsecond_multi == 0)
	{
		roadsecond_multi = RoadCollision_multi(Start, End, road_yaw);
		if (roadsecond_multi == 0)
		{
			LOG_WARNING("roadsecond_multi=%d", roadsecond_multi);
			//fflush(bydapa::common::Log::GetInstance()->fileptr());
			getroadpath_multi(Start, road_yaw);
			return 1;
		}
	}
	else if (roadsecond_multi == 1)
	{
		roadsecond_multi = RoadCollision_multi(Start, End, road_yaw2);
		if (roadsecond_multi == 0)
		{
			LOG_WARNING("roadsecond_multi=%d", roadsecond_multi);
			//fflush(bydapa::common::Log::GetInstance()->fileptr());
			getroadpath_multi(Start, road_yaw2);
			return 1;
		}
		else 
		{
			roadsecond_multi = RoadCollision_multi(Start, End, road_yaw);
			if (roadsecond_multi == 0)
			{
				LOG_WARNING("roadsecond_multi=%d", roadsecond_multi);
				//fflush(bydapa::common::Log::GetInstance()->fileptr());
				getroadpath_multi(Start, road_yaw);
				return 1;
			}
		}
	}
	//C-C-
	/*
	if (fabs(mod2pi(End[2] - Start[2])) < 0.0873)// 2022.12.02:增加mod2pi
	{
		int roadthird_multi = 1;
		roadthird_multi = road_find_two_multi(Start[0], Start[1], Start[2], End[0], End[1], End[2], 5.3, road_yaw);  // 2022.12.06:最小转弯半径5.8修改为5.3
		if (roadthird_multi == 0)
		{
			roadthird_multi = RoadCollision_multi(Start, End, road_yaw);
			if (roadthird_multi == 0)
			{
				LOG_WARNING("roadthird_multi=%d", roadthird_multi);
				//fflush(bydapa::common::Log::GetInstance()->fileptr());
				getroadpath_multi(Start, road_yaw);
				return 1;
			}
		}
	}
    */
    return 0;
}