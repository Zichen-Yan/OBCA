#include "findpath.h"
////////////全局变量定义/////////////////////////////////////////////

double road_find_first_Level(double x_now, double y_now, double thea_now, double x_aim, double y_aim, double thea_aim, double y_tolerance, double min_r, double yaw_tolerance, double road_yaw[6]);
int RoadCollision_Level(double Start[3], double End[3], double road_yaw[6]);
void getroadpath_Level(double Start[3], double road_yaw[6]);
char road_find_three_Level(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_R_S[6]);
char angleget_Level(double sin_result, double cos_result, double* last_angle_result);
char road_find_two_Level(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_two[6]);
char road_find_four_Level(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double max_x, double road_yaw[6]);
int CCS_five_Level(double x, double y, double phi, RsPath &path_five, int theta_car);
char road_find_five_Level(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_yaw[6], double road_yaw2[6]);
int CCS_six_Level(double x, double y, double phi, RsPath &path_six, int theta_car);
char road_find_six_Level(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_yaw[6], double road_yaw2[6]);
char road_find_three_cs_Level(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double bias, double *road_R_S);
int CCS_seven_Level(double x, double y, double phi, RsPath &path_seven, int theta_car);
int CCC_seven_Level(double x, double y, double phi, RsPath &path_seven);
char road_find_seven1_Level(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_yaw[6]);
char road_find_seven2_Level(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit);
///////类定义/////////////////

void getroadpath_Level(double Start[3], double road_yaw[6])
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
}

int RoadCollision_Level(double Start[3], double End[3], double road_yaw[6])
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

int CCS_five_Level(double x, double y, double phi, RsPath &path_five, int theta_car)
{
	double t, u, v;
	if (LpRpSp(x, y, phi, t, u, v) && (((t >= 0) && (u >= 0) && (v <= 0) && (theta_car == 1)) || ((t <= 0) && (u <= 0) && (v >= 0) && (theta_car == -1))))
	{
		path_five.t = t;
		path_five.u = u;
		path_five.v = v;
		path_five.lenth = fabs(t) + fabs(u) + fabs(v);
		path_five.rspath_type[0] = Le;
		path_five.rspath_type[1] = Ri;
		path_five.rspath_type[2] = St;
		return 0;
	}
	if (LpRpSp(-x, y, -phi, t, u, v) && (((t <= 0) && (u <= 0) && (v >= 0) && (theta_car == 1)) || ((t >= 0) && (u >= 0) && (v <= 0) && (theta_car == -1))))
		//	if (LpRpSp(-x, y, -phi, t, u, v) && (t <= 0) && (u <= 0) && (v >= 0)) // timeflip
	{
		path_five.t = -t;
		path_five.u = -u;
		path_five.v = -v;
		path_five.lenth = fabs(t) + fabs(u) + fabs(v);
		path_five.rspath_type[0] = Le;
		path_five.rspath_type[1] = Ri;
		path_five.rspath_type[2] = St;
		return 0;
	}
	if (LpRpSp(x, -y, -phi, t, u, v) && (((t >= 0) && (u >= 0) && (v <= 0) && (theta_car == 1)) || ((t <= 0) && (u <= 0) && (v >= 0) && (theta_car == -1))))
		//	if (LpRpSp(x, -y, -phi, t, u, v) && (t >= 0) && (u >= 0) && (v <= 0)) // reflect
	{
		path_five.t = t;
		path_five.u = u;
		path_five.v = v;
		path_five.lenth = fabs(t) + fabs(u) + fabs(v);
		path_five.rspath_type[0] = Ri;
		path_five.rspath_type[1] = Le;
		path_five.rspath_type[2] = St;
		return 0;
	}
	if (LpRpSp(-x, -y, phi, t, u, v) && (((t <= 0) && (u <= 0) && (v >= 0) && (theta_car == 1)) || ((t >= 0) && (u >= 0) && (v <= 0) && (theta_car == -1))))
		//	if (LpRpSp(-x, -y, phi, t, u, v) && (t <= 0) && (u <= 0) && (v >= 0)) // timeflip + reflect
	{
		path_five.t = -t;
		path_five.u = -u;
		path_five.v = -v;
		path_five.lenth = fabs(t) + fabs(u) + fabs(v);
		path_five.rspath_type[0] = Ri;
		path_five.rspath_type[1] = Le;
		path_five.rspath_type[2] = St;
		return 0;
	}

	return 2;
}

int CCS_six_Level(double x, double y, double phi, RsPath &path_six, int theta_car)
{
	double t, u, v;
	if (LpRpSp(x, y, phi, t, u, v) && (((t >= 0) && (u <= 0) && (v <= 0) && (theta_car == 1)) || ((t <= 0) && (u >= 0) && (v >= 0) && (theta_car == -1))))
		//	if (LpRpSp(x, y, phi, t, u, v) && (t >= 0) && (u <= 0) && (v <= 0))
	{
		path_six.t = t;
		path_six.u = u;
		path_six.v = v;
		path_six.lenth = fabs(t) + fabs(u) + fabs(v);
		path_six.rspath_type[0] = Le;
		path_six.rspath_type[1] = Ri;
		path_six.rspath_type[2] = St;
		return 0;
	}
	if (LpRpSp(-x, y, -phi, t, u, v) && (((t <= 0) && (u >= 0) && (v >= 0) && (theta_car == 1)) || ((t >= 0) && (u <= 0) && (v <= 0) && (theta_car == -1))))
		//	if (LpRpSp(-x, y, -phi, t, u, v) && (t <= 0) && (u >= 0) && (v >= 0)) // timeflip
	{
		path_six.t = -t;
		path_six.u = -u;
		path_six.v = -v;
		path_six.lenth = fabs(t) + fabs(u) + fabs(v);
		path_six.rspath_type[0] = Le;
		path_six.rspath_type[1] = Ri;
		path_six.rspath_type[2] = St;
		return 0;
	}
	if (LpRpSp(x, -y, -phi, t, u, v) && (((t >= 0) && (u <= 0) && (v <= 0) && (theta_car == 1)) || ((t <= 0) && (u >= 0) && (v >= 0) && (theta_car == -1))))
		//	if (LpRpSp(x, -y, -phi, t, u, v) && (t >= 0) && (u <= 0) && (v <= 0)) // reflect
	{
		path_six.t = t;
		path_six.u = u;
		path_six.v = v;
		path_six.lenth = fabs(t) + fabs(u) + fabs(v);
		path_six.rspath_type[0] = Ri;
		path_six.rspath_type[1] = Le;
		path_six.rspath_type[2] = St;
		return 0;
	}
	if (LpRpSp(-x, -y, phi, t, u, v) && (((t <= 0) && (u >= 0) && (v >= 0) && (theta_car == 1)) || ((t >= 0) && (u <= 0) && (v <= 0) && (theta_car == -1))))
		//	if (LpRpSp(-x, -y, phi, t, u, v) && (t <= 0) && (u >= 0) && (v >= 0)) // timeflip + reflect
	{
		path_six.t = -t;
		path_six.u = -u;
		path_six.v = -v;
		path_six.lenth = fabs(t) + fabs(u) + fabs(v);
		path_six.rspath_type[0] = Ri;
		path_six.rspath_type[1] = Le;
		path_six.rspath_type[2] = St;
		return 0;
	}

	return 2;

}

int CCS_seven_Level(double x, double y, double phi, RsPath &path_seven, int theta_car)
{
	double t, u, v;
	if ((LpRpSp(x, y, phi, t, u, v)) && (t * u < 0) && ((Gears * t) < 0))
	{
		path_seven.t = t;
		path_seven.u = u;
		path_seven.v = v;
		path_seven.lenth = fabs(t) + fabs(u) + fabs(v);
		path_seven.rspath_type[0] = Le;
		path_seven.rspath_type[1] = Ri;
		path_seven.rspath_type[2] = St;
		return 0;
	}
	if ((LpRpSp(-x, y, -phi, t, u, v)) && (t * u < 0) && ((Gears * t) > 0))
	{
		path_seven.t = -t;
		path_seven.u = -u;
		path_seven.v = -v;
		path_seven.lenth = fabs(t) + fabs(u) + fabs(v);
		path_seven.rspath_type[0] = Le;
		path_seven.rspath_type[1] = Ri;
		path_seven.rspath_type[2] = St;
		return 0;
	}
	if ((LpRpSp(x, -y, -phi, t, u, v)) && (t * u < 0) && ((Gears * t) < 0))
	{
		path_seven.t = t;
		path_seven.u = u;
		path_seven.v = v;
		path_seven.lenth = fabs(t) + fabs(u) + fabs(v);
		path_seven.rspath_type[0] = Ri;
		path_seven.rspath_type[1] = Le;
		path_seven.rspath_type[2] = St;
		return 0;
	}
	if ((LpRpSp(-x, -y, phi, t, u, v)) && (t * u < 0) && ((Gears * t) > 0))
	{
		path_seven.t = -t;
		path_seven.u = -u;
		path_seven.v = -v;
		path_seven.lenth = fabs(t) + fabs(u) + fabs(v);
		path_seven.rspath_type[0] = Ri;
		path_seven.rspath_type[1] = Le;
		path_seven.rspath_type[2] = St;
		return 0;
	}

	return 2;

}

int CCC_seven_Level(double x, double y, double phi, RsPath &path_seven)
{
	double t, u, v;
	if ((LpRmL(x, y, phi, t, u, v)) && (t * u <= 0) && (u * v <= 0) && ((Gears * t) < 0))
	{
		path_seven.t = t;
		path_seven.u = u;
		path_seven.v = v;
		path_seven.lenth = fabs(t) + fabs(u) + fabs(v);
		path_seven.rspath_type[0] = Le;
		path_seven.rspath_type[1] = Ri;
		path_seven.rspath_type[2] = Le;
		return 0;
	}
	if ((LpRmL(-x, y, -phi, t, u, v)) && (t * u <= 0) && (u * v <= 0) && ((Gears * t) > 0)) // timeflip
	{
		path_seven.t = -t;
		path_seven.u = -u;
		path_seven.v = -v;
		path_seven.lenth = fabs(t) + fabs(u) + fabs(v);
		path_seven.rspath_type[0] = Le;
		path_seven.rspath_type[1] = Ri;
		path_seven.rspath_type[2] = Le;
		return 0;
	}
	if ((LpRmL(x, -y, -phi, t, u, v)) && (t * u <= 0) && (u * v <= 0) && ((Gears * t) < 0)) // reflect
	{
		path_seven.t = t;
		path_seven.u = u;
		path_seven.v = v;
		path_seven.lenth = fabs(t) + fabs(u) + fabs(v);
		path_seven.rspath_type[0] = Ri;
		path_seven.rspath_type[1] = Le;
		path_seven.rspath_type[2] = Ri;
		return 0;
	}
	if ((LpRmL(-x, -y, phi, t, u, v)) && (t * u <= 0) && (u * v <= 0) && ((Gears * t) > 0)) // timeflip + reflect
	{
		path_seven.t = -t;
		path_seven.u = -u;
		path_seven.v = -v;
		path_seven.lenth = fabs(t) + fabs(u) + fabs(v);
		path_seven.rspath_type[0] = Ri;
		path_seven.rspath_type[1] = Le;
		path_seven.rspath_type[2] = Ri;
		return 0;
	}

	double xb = x*cos(phi) + y*sin(phi), yb = x*sin(phi) - y*cos(phi);
	if ((LpRmL(xb, yb, phi, t, u, v)) && (v * u <= 0) && (u * t <= 0) && ((Gears * v) < 0))
	{
		path_seven.t = v;
		path_seven.u = u;
		path_seven.v = t;
		path_seven.lenth = fabs(t) + fabs(u) + fabs(v);
		path_seven.rspath_type[0] = Le;
		path_seven.rspath_type[1] = Ri;
		path_seven.rspath_type[2] = Le;
		return 0;
	}
	if ((LpRmL(-xb, yb, -phi, t, u, v)) && (v * u <= 0) && (u * t <= 0) && ((Gears * v) > 0)) // timeflip
	{
		path_seven.t = -v;
		path_seven.u = -u;
		path_seven.v = -t;
		path_seven.lenth = fabs(t) + fabs(u) + fabs(v);
		path_seven.rspath_type[0] = Le;
		path_seven.rspath_type[1] = Ri;
		path_seven.rspath_type[2] = Le;
		return 0;
	}
	if ((LpRmL(xb, -yb, -phi, t, u, v)) && (v * u <= 0) && (u * t <= 0) && ((Gears * v) < 0)) // reflect
	{
		path_seven.t = v;
		path_seven.u = u;
		path_seven.v = t;
		path_seven.lenth = fabs(t) + fabs(u) + fabs(v);
		path_seven.rspath_type[0] = Ri;
		path_seven.rspath_type[1] = Le;
		path_seven.rspath_type[2] = Ri;
		return 0;
	}
	if ((LpRmL(-xb, -yb, phi, t, u, v)) && (v * u <= 0) && (u * t <= 0) && ((Gears * v) > 0)) // timeflip + reflect
	{
		path_seven.t = -v;
		path_seven.u = -u;
		path_seven.v = -t;
		path_seven.lenth = fabs(t) + fabs(u) + fabs(v);
		path_seven.rspath_type[0] = Ri;
		path_seven.rspath_type[1] = Le;
		path_seven.rspath_type[2] = Ri;
		return 0;
	}

	return 2;
}

double road_find_first_Level(double x_now, double y_now, double thea_now, double x_aim, double y_aim, double thea_aim, double y_tolerance, double min_r, double yaw_tolerance, double road_yaw[6])  //根据车辆当前点及目标点计算转弯半径
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
	yaw_tolerance = yaw_tolerance * pi / 180;
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

char road_find_three_Level(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_R_S[6])
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
	double R = 0;
	if (cos(theta) == 1)
	{
		return 1;
	}
	else if (fabs(x * sin(theta) - y * cos(theta)) * 0.000001 >= fabs(1 - cos(theta)))
	{
		R = 1000000;
	}
	else
	{
		R = (x * sin(theta) - y * cos(theta)) / (1 - cos(theta));
	}
	if (fabs(R) < car_r_limit)
	{
		return 1;
	}
	double L = x * cos(theta) + y * sin(theta) - R * sin(theta);
	if (R * theta > 0 && L > 0)
	{
		road_R_S[0] = R;
		road_R_S[1] = R * theta;
		road_R_S[2] = 1000000;
		road_R_S[3] = L;
	}
	else if (R * theta < 0 && L < 0)
	{
		road_R_S[0] = R;
		road_R_S[1] = R * theta;
		road_R_S[2] = 1000000;
		road_R_S[3] = L;
	}
	else if (R * theta == 0 || fabs(L) < 0.01)
	{
		road_R_S[0] = R;
		road_R_S[1] = R * theta;
		road_R_S[2] = 1000000;
		road_R_S[3] = L;
	}
	else
	{
		return 1;
	}
	if (fabs(road_R_S[0]) < car_r_limit)
	{
		return 1;
	}
	return 0;
}

char angleget_Level(double sin_result, double cos_result, double* last_angle_result)
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

char road_find_three_cs_Level(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double bias, double *road_R_S)
{
	double l_back = 0;  //直线后退的值
	double y_1 = 0;     //走完第一段弧的y值
	double x_1 = 0;     //走完第一段弧的y值
						//////////////////////////////车辆坐标转换处理/////////////////////////////////////////////////////////////////////
	double thea_aim1 = theta_aim / 180 * pi;
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
	double theta_rad = theta_ang / 180 * pi;
	// 车辆方向与车位夹角
	if (fabs(theta_ang)>60)  //当前角度与目标角度差值较大，cs不一定是最优解,60参数可以调整
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
				return 0;
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
				return 0;
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
		return 1;//越界不在求解
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

////////////////////////////////////////求解两端路径////////////////////////////////////////////////////////////////////////////////
//x_now  y_now  thea_now  车辆当前点x，y，thea坐标
//x_aim  y_aim  thea_aim  车辆终点x，y，thea坐标
//car_r_limit         最小转弯半径，小于此值无解
//road_two[0] 第一段转弯半径，road_two[1] 第一段距离，前进为正后退为负，road_two[2] 第二段转弯半径，road_two[3] 第二段距离，前进为正后退为负，
//return 0代表有解 
char road_find_two_Level(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_two[6])
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
		if (angleget_Level(sin_theta_first_1, cos_theta_first_1, angle_1) != 0)//无解
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
		if (angleget_Level(sin_theta_first_1, cos_theta_first_1, angle_1) != 0)
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
		if (angleget_Level(sin_theta_first_2, cos_theta_first_2, angle_2) != 0)
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

char road_find_four_Level(double x_now, double y_now, double theta_now,
	double x_aim, double y_aim, double theta_aim,
	double car_r_limit, double bias, double *road_R_S)
{
	double l_back = 0;  //直线后退的值
	double y_1 = 0;     //走完第一段弧的y值
	double x_1 = 0;     //走完第一段弧的y值
						//////////////////////////////车辆坐标转换处理/////////////////////////////////////////////////////////////////////
	double thea_aim1 = theta_aim / 180 * pi;
	//double theta_now1 = (theta_now - theta_aim) / 180 * 3.1415926;
	double y = (y_now - y_aim)*cos(thea_aim1) - (x_now - x_aim)*sin(thea_aim1);// 坐标转换后 Y向的实际偏差
	double x = (x_now - x_aim)*cos(thea_aim1) + (y_now - y_aim)*sin(thea_aim1);
	double theta_ang = theta_now - theta_aim;
	//////把角度调整为-180到180之间
	theta_ang = fmod(theta_ang, 360);
	if (theta_ang > 180)
		theta_ang = theta_ang - 360;
	else if (theta_ang <-180)
		theta_ang = theta_ang + 360;
	double theta_rad = theta_ang / 180 * pi;
	if (fabs(theta_ang)>60)  //当前角度与目标角度差值较大，cs不一定是最优解,60参数可以调整
	{
		return 10;
	}
	////////////////////////////////允许y存在误差，查看是否有解？？？？？？////////////////////////////////////////////////
	//偏差允许有解：方向盘先向左打，然后把车辆调成和目标一致，查看圆弧直线是否需要换挡
	double car_r_limit_m = car_r_limit;
	x_1 = x - car_r_limit_m*sin(theta_rad);
	y_1 = y + car_r_limit_m*cos(theta_rad) - car_r_limit_m;   //把方向盘角度调整为和目标一致！
	if (car_r_limit_m*(-theta_rad)*x_1 >0)//圆弧的解需换挡
	{
		if (fabs(y_1) < bias && fabs(x_1) <= 2)//是否在误差允许范围内 并且未超出对向限制空间
		{
			road_R_S[0] = car_r_limit_m;
			road_R_S[1] = car_r_limit_m*(-theta_rad);
			road_R_S[2] = 1000000;
			road_R_S[3] = -x_1;
			return 0;
		}
		else
		{
			return 2;//无解
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
	if (car_r_limit_m*(-theta_rad)*x_1 >0)//圆弧的解需换挡
	{
		if (fabs(y_1) < bias && fabs(x_1) <= 2)//是否在误差允许范围内 并且未超出对向限制空间
		{
			road_R_S[0] = car_r_limit_m;
			road_R_S[1] = car_r_limit_m*(-theta_rad);
			road_R_S[2] = 1000000;
			road_R_S[3] = -x_1;
			return 0;
		}
		else
		{
			return 2;//无解
		}
	}
	else
	{
		//无需处理
	}
	
	return 2;
}

char road_find_five_Level(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_yaw[6], double road_yaw2[6])
{
	//////////////////////////////车辆坐标转换处理/////////////////////////////////////////////////////////////////////
	double rmin = car_r_limit;
	double ph = mod2pi(theta_aim - theta_now);
	double phi = mod2pi(theta_now);
	// 起点start坐标系在基坐标系下的方向余弦矩阵(Z轴旋转，因为点在Z = 0平面，因此绕Z旋转仍然在Z = 0平面)
	// dcm*x 表示将基坐标中的x表示到旋转后的坐标系中，即计算坐标旋转后各向量在新坐标中的表示
	double x1 = (x_aim - x_now)*cos(phi) + (y_aim - y_now)*sin(phi);
	double y1 = (y_aim - y_now)*cos(phi) - (x_aim - x_now)*sin(phi);
	if (fabs(ph) < 45 * pi / 180)
	{
		RsPath path_five;
		double x1_five = x1 / rmin;
		double y1_five = y1 / rmin;
		int ErrorCode = 100;
		int theta_car = 0;
		if (fusion.ParkInMode == 1)
		{
			theta_car = -1;
		}
		else
		{
			theta_car = 1;
		}
		ErrorCode = CCS_five_Level(x1_five, y1_five, ph, path_five, theta_car);
		if (ErrorCode == 0)
		{
			for (int index_five_CCS = 0;index_five_CCS < 3;index_five_CCS++)
			{
				if (path_five.rspath_type[index_five_CCS] == St)
				{
					road_yaw[index_five_CCS * 2] = 1000000;
				}

				else if (path_five.rspath_type[index_five_CCS] == Le)
				{
					road_yaw[index_five_CCS * 2] = rmin;
				}

				else if (path_five.rspath_type[index_five_CCS] == Ri)
				{
					road_yaw[index_five_CCS * 2] = -rmin;
				}
			}

			road_yaw[1] = path_five.t * rmin;
			if (road_yaw[1] < 0.15)
			{
				road_yaw[1] = 0;
			}
			road_yaw[3] = path_five.u * rmin;
			road_yaw[5] = path_five.v * rmin;
			path_five.lenth = path_five.lenth * rmin;
			if (fabs(road_yaw[5]) > 2)
			{
				return 2;
			}

		}
		else
		{
			return 2;
		}

		double x_midpoint = 2;
		double y_midpoint = 0;
		double theta_midpoint = 0;

		double x = x_midpoint * cos(ph) - y_midpoint * sin(ph) + x1;
		double y = y_midpoint * cos(ph) + x_midpoint * sin(ph) + y1;
		double theta = mod2pi(ph - theta_midpoint);

		double R_s = 1000000;
		double L_s = -x_midpoint * theta_car;

		double road_yaw_R[6] = { 0,0,0,0,0,0 };
		int roadthird_five = 1;
		roadthird_five = road_find_two_Level(0, 0, 0, x, y, theta, R_L, road_yaw_R);
		if (roadthird_five == 0)
		{
			road_yaw2[0] = road_yaw_R[0];
			road_yaw2[1] = road_yaw_R[1];
			road_yaw2[2] = road_yaw_R[2];
			road_yaw2[3] = road_yaw_R[3];
			road_yaw2[4] = R_s;
			road_yaw2[5] = L_s;
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 2;
	}

	return 2;
}

char road_find_six_Level(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_yaw[6], double road_yaw2[6])
{
	//////////////////////////////车辆坐标转换处理/////////////////////////////////////////////////////////////////////
	double rmin = car_r_limit;
	double ph = mod2pi(theta_aim - theta_now);
	double phi = mod2pi(theta_now);
	// 起点start坐标系在基坐标系下的方向余弦矩阵(Z轴旋转，因为点在Z = 0平面，因此绕Z旋转仍然在Z = 0平面)
	// dcm*x 表示将基坐标中的x表示到旋转后的坐标系中，即计算坐标旋转后各向量在新坐标中的表示
	double x1 = (x_aim - x_now)*cos(phi) + (y_aim - y_now)*sin(phi);
	double y1 = (y_aim - y_now)*cos(phi) - (x_aim - x_now)*sin(phi);

	RsPath path_six;
	double x1_six = x1 / rmin;
	double y1_six = y1 / rmin;
	int ErrorCode = 100;
	int theta_car = 0;
	if (fusion.ParkInMode == 1)
	{
		theta_car = -1;
	}
	else
	{
		theta_car = 1;
	}
	ErrorCode = CCS_six_Level(x1_six, y1_six, ph, path_six, theta_car);
	if (ErrorCode == 0)
	{
		for (int index_six_CCS = 0;index_six_CCS < 3;index_six_CCS++)
		{
			if (path_six.rspath_type[index_six_CCS] == St)
			{
				road_yaw[index_six_CCS * 2] = 1000000;
			}

			else if (path_six.rspath_type[index_six_CCS] == Le)
			{
				road_yaw[index_six_CCS * 2] = rmin;
			}

			else if (path_six.rspath_type[index_six_CCS] == Ri)
			{
				road_yaw[index_six_CCS * 2] = -rmin;
			}
		}

		road_yaw[1] = path_six.t * rmin;
		if (road_yaw[1] < 0.15)
		{
			road_yaw[1] = 0;
		}
		road_yaw[3] = path_six.u * rmin;
		road_yaw[5] = path_six.v * rmin;
		path_six.lenth = path_six.lenth * rmin;
		if (fabs(road_yaw[3] + road_yaw[5]) > 2)
		{
			return 2;
		}

	}
	else
	{
		return 2;
	}
	/////////////////////////////////////////////////////////////////////////////////////
	double delta_six1 = 0;
	if (fabs(road_yaw[0]) > 100000 || fabs(road_yaw[0]) < R_C)
	{
		delta_six1 = 0;
	}
	else
	{
		delta_six1 = atan(vehicle_parameters.WB / road_yaw[0]);
	}
	road_yaw2[0] = road_yaw[0];
	road_yaw2[1] = road_yaw[1];
	road_yaw2[2] = road_yaw[2];
	road_yaw2[3] = road_yaw[3];
	road_yaw2[4] = road_yaw[4];
	road_yaw2[5] = road_yaw[5];
	double road_yaw_siximag[6] = { 0,0,0,0,0,0 };
	double road_yaw_siximagthree[6] = { 0,0,0,0,0,0 };
	double Start_imag[3] = { 0,0,0 };
	double End_imag[3] = { 0,0,0 };
	Start_imag[0] = x_now;
	Start_imag[1] = y_now;
	Start_imag[2] = theta_now;
	End_imag[0] = x_aim;
	End_imag[1] = y_aim;
	End_imag[2] = theta_aim;

	for (int index_D_six = 30; index_D_six > 0; index_D_six--)
	{
		double g_x = 0;
		double g_y = 0;
		double g_th = 0;
		VehicleDynamic(g_x, g_y, g_th, road_yaw[1] + index_D_six * 0.1, delta_six1);

		int roadsecond_six = 1;
		roadsecond_six = road_find_three_Level(g_px, g_py, g_pth, x1, y1, ph, R_L, road_yaw_siximagthree);
		if (roadsecond_six == 0)
		{
			road_yaw_siximag[0] = road_yaw[0];
			road_yaw_siximag[1] = road_yaw[1] + index_D_six * 0.1;
			road_yaw_siximag[2] = road_yaw_siximagthree[0];
			road_yaw_siximag[3] = road_yaw_siximagthree[1];
			road_yaw_siximag[4] = road_yaw_siximagthree[2];
			road_yaw_siximag[5] = road_yaw_siximagthree[3];

			roadsecond_six = RoadCollision_Level(Start_imag, End_imag, road_yaw_siximag);

			if ((roadsecond_six == 0) && (fabs(road_yaw_siximag[5]) > 1))
			{
				road_yaw2[0] = road_yaw_siximag[0];
				road_yaw2[1] = road_yaw_siximag[1];
				if (road_yaw2[1] < 0.15)
				{
					road_yaw2[1] = 0;
				}
				road_yaw2[2] = road_yaw_siximag[2];
				road_yaw2[3] = road_yaw_siximag[3];
				road_yaw2[4] = road_yaw_siximag[4];
				road_yaw2[5] = road_yaw_siximag[5];
				return 1;
			}

		}
	}

	return 0;
}

char road_find_seven1_Level(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_yaw[6])
{
	//////////////////////////////车辆坐标转换处理/////////////////////////////////////////////////////////////////////
	double rmin = car_r_limit;
	////////////////////////////不是水平小车位///////////////////////////////////////
	double level_parking_space = 5.8 + front_D * 0.1 + back_D * 0.1;
	if (level_parking_space > 6.66)
	{
		return 1;
	}
	////////////////////////////////////揉库////////////////////////////////////////////////
	double y_seven = (y_now - y_aim)*cos(theta_aim) - (x_now - x_aim)*sin(theta_aim); // 终点坐标系
	double x_seven = (x_now - x_aim)*cos(theta_aim) + (y_now - y_aim)*sin(theta_aim);
	double theta_ang_seven = theta_now - theta_aim;
	if (theta_ang_seven >= 0)
	{
		if (fabs(theta_ang_seven) < 1 * pi / 180) //判断车头方向是否超调，如果超调证明空间足够，请求A*
		{
			if ((fabs(y_seven) < 0.3) && (fabs(x_seven) > 0.15))  // 完成条件角度小于1°，y向偏差30cm
			{
				road_yaw[0] = 1000000;
				road_yaw[1] = -x_seven;
				road_yaw[2] = 0;
				road_yaw[3] = 0;
				road_yaw[4] = 0;
				road_yaw[5] = 0;
				return 0;
			}
			else
			{
				return 2;     // 角度小于1°，y向偏差大于30cm，失败交给A*
			}
		}
	}
	else
	{
		if (fabs(theta_ang_seven) < 1 * pi / 180) //判断车头方向是否超调，如果超调证明空间足够，请求A*
		{
			if ((fabs(y_seven) < 0.3) && (fabs(x_seven) > 0.15))  // 完成条件角度小于1°，y向偏差30cm
			{
				printf("x_seven=%lf\n",x_seven);
				printf("**************************\n");
				road_yaw[0] = 1000000;
				road_yaw[1] = -x_seven;
				road_yaw[2] = 0;
				road_yaw[3] = 0;
				road_yaw[4] = 0;
				road_yaw[5] = 0;
				return 0;
			}
			else
			{
				return 2;     // 角度小于1°，y向偏差大于30cm，失败交给A*
			}
		}
	}

	return 2;
}

char road_find_seven2_Level(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit)
{
	//////////////////////////////车辆坐标转换处理/////////////////////////////////////////////////////////////////////
	double rmin = car_r_limit;
	////////////////////////////不是水平小车位///////////////////////////////////////
	double level_parking_space = 5.8 + front_D * 0.1 + back_D * 0.1;
	if (level_parking_space > 6.66)
	{
		return 1;
	}
	////////////////////////////////////揉库////////////////////////////////////////////////
	double y_seven = (y_now - y_aim)*cos(theta_aim) - (x_now - x_aim)*sin(theta_aim); // 终点坐标系
	double x_seven = (x_now - x_aim)*cos(theta_aim) + (y_now - y_aim)*sin(theta_aim);
	double theta_ang_seven = theta_now - theta_aim;
	if (theta_ang_seven >= 0)
	{
		if (fabs(theta_ang_seven) < 1 * pi / 180) //判断车头方向是否超调，如果超调证明空间足够，请求A*
		{
			return 2;     // 角度小于1°，y向偏差大于30cm，失败交给A*
		}
		else if (theta_ang_seven > 60 * pi / 180)
		{
			return 1;        // 夹角大于60°，失败
		}

		int point_rk = round(fabs(theta_ang_seven) / 0.0182);  // 需要多少点才能将车辆角度调正

		if ((control.ObsUssInfo == 6u) || (control.ObsUssInfo == 7u) || (control.ObsUssInfo == 8u) || (control.ObsUssInfo == 9u) || (Gears == 2)) // 后退被逼停
		{
			/*if (y_seven > 0.3)
			{
				return 2; // 揉正外移大于30cm
			}*/

			double D_adv_ = front_path - x_seven; // 车辆前进最多走的距离
			double D_adv = rmin*(fabs(theta_ang_seven)- asin(sin(fabs(theta_ang_seven))- D_adv_/rmin));
			int point_adv;                       // 揉库最少的点数，每个点10cm
			if (D_adv < 0.2)
			{
				return 3; // 车辆停在可揉库范围外
			}
			else
			{
				point_adv = round(D_adv / 0.1);
			}
			int point_adv_min = round(point_rk / 6);
			if (point_adv_min <= 2) // 最少走20cm
			{
				point_adv_min = 2;
			}
			if (point_adv_min > point_adv)
			{
				return 3; // 揉库次数不够
			}

			double diff_y = 100; // 记录y向偏差
			int rk_idx = 0;        // 记录揉库次数
			int point_large = 0;

			double delta_idx_D = -atan(vehicle_parameters.WB / rmin);
			double D_idx_D = 0.1;
			double delta_idx_R = atan(vehicle_parameters.WB / rmin);
			double D_idx_R = -0.1;

			for (int idx = point_adv; idx > point_adv_min - 1; idx--) // 遍历所有可能的步长
			{
				double rk_x = x_seven; // 记录揉库后的终点
				double rk_y = y_seven;
				double rk_theta = theta_ang_seven;

				int mod_point = point_rk; // 记录剩余的点数

				for (int index = 0; index < ceil(point_rk * 1.0 / point_adv); index++)
				{
					int gears_DR = index % 2; // 取余来判断挡位
					mod_point = mod_point - idx;
					int idx_max = idx;
					if (mod_point >= 0)
					{
						idx_max = idx_max;
					}
					else
					{
						idx_max = mod_point + idx;
					}

					if (gears_DR == 0) // 前进
					{
						rk_idx = rk_idx + 1;
						//double D_lth_point = round((fabs(rk_theta) - asin((rmin * sin(fabs(rk_theta))- idx_max * 0.1)/ rmin)) * 10 * rmin);
						VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_D*idx_max, delta_idx_D);

						rk_x = g_px; // 记录揉库后的终点
						rk_y = g_py;
						rk_theta = g_pth;
					}
					else  // 后退
					{
						rk_idx = rk_idx + 1;

						VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_R*idx_max, delta_idx_R);

						rk_x = g_px; // 记录揉库后的终点
						rk_y = g_py;
						rk_theta = g_pth;

					}
				}

				if ((fabs(rk_y) < 0.3) && (rk_idx <= (6 - small_level_park))) // 如果y向偏差在30cm输出
				{
					diff_y = rk_y;
					point_large = idx;
					break;
				}
				else
				{
					if ((fabs(diff_y) > fabs(rk_y)) && (rk_idx <= (6 - small_level_park)))
					{
						diff_y = rk_y;
						point_large = idx;
					}
				}
			}
			// 给路
			if (fabs(diff_y) < 0.3)
			{
				path_point mid_LR;
				mid_LR.x = Start[0];
				mid_LR.y = Start[1];
				mid_LR.th = mod2pi(Start[2]);
				mid_LR.D = 0;
				mid_LR.delta = 0;
				pathpoint.push_back(mid_LR);

				double rk_x = Start[0]; // 记录揉库后的终点
				double rk_y = Start[1];
				double rk_theta = mod2pi(Start[2]);

				int mod_point = point_rk; // 记录剩余的点数

				for (int index = 0; index < ceil(point_rk * 1.0 / point_large); index++)
				{
					int gears_DR = index % 2; // 取余来判断挡位
					mod_point = mod_point - point_large;
					int idx_max = point_large;
					if (mod_point >= 0)
					{
						idx_max = idx_max;
					}
					else
					{
						idx_max = mod_point + point_large;
					}

					if (gears_DR == 0) // 前进
					{
						for (int index_1 = 0; index_1 < idx_max; index_1++)
						{
							VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_D, delta_idx_D);
							mid_LR.x = g_px;
							mid_LR.y = g_py;
							mid_LR.th = g_pth;
							mid_LR.D = D_idx_D;
							mid_LR.delta = delta_idx_D;
							pathpoint.push_back(mid_LR);
							rk_x = g_px; // 记录揉库后的终点
							rk_y = g_py;
							rk_theta = g_pth;
						}
					}
					else  // 后退
					{
						for (int index_1 = 0; index_1 < idx_max; index_1++)
						{
							VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_R, delta_idx_R);
							mid_LR.x = g_px;
							mid_LR.y = g_py;
							mid_LR.th = g_pth;
							mid_LR.D = D_idx_R;
							mid_LR.delta = delta_idx_R;
							pathpoint.push_back(mid_LR);
							rk_x = g_px; // 记录揉库后的终点
							rk_y = g_py;
							rk_theta = g_pth;
						}
					}
				}
				return 0;
			}
			else
			{
				return 2;
			}

		}
		else if ((control.ObsUssInfo == 0u) || (control.ObsUssInfo == 1u) || (control.ObsUssInfo == 2u) || (control.ObsUssInfo == 3u) || (Gears == 1)) // 前进被逼停
		{
			/*if (y_seven < -0.3)
			{
				return 2; // 揉正内移大于30cm
			}*/
			double D_adv_ = x_seven - back_path; // 车辆后退最多走的距离
			double D_adv = rmin*(fabs(theta_ang_seven)- asin(sin(fabs(theta_ang_seven))- D_adv_/rmin));
			int point_adv;                       // 揉库最少的点数，每个点10cm
			if (D_adv < 0.2)
			{
				return 3; // 车辆停在可揉库范围外
			}
			else
			{
				point_adv = round(D_adv / 0.1);
			}

			double delta_idx_D = -atan(vehicle_parameters.WB / rmin);
			double D_idx_D = 0.1;
			double delta_idx_R = atan(vehicle_parameters.WB / rmin);
			double D_idx_R = -0.1;

			double rk_x = x_seven; // 记录揉库后的终点
			double rk_y = y_seven;
			double rk_theta = theta_ang_seven;



			int mod_point = point_rk; // 记录剩余的点数

			for (int index = 0; index < ceil(point_rk * 1.0 / point_adv); index++)
			{
				int gears_DR = index % 2; // 取余来判断挡位
				mod_point = mod_point - point_adv;
				int idx_max = point_adv;
				if (mod_point >= 0)
				{
					idx_max = idx_max;
				}
				else
				{
					idx_max = mod_point + point_adv;
				}

				if (gears_DR == 0) // 后退
				{
					VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_R*idx_max, delta_idx_R);

					rk_x = g_px; // 记录揉库后的终点
					rk_y = g_py;
					rk_theta = g_pth;
				}
				else  // 前进
				{
					VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_D*idx_max, delta_idx_D);

					rk_x = g_px; // 记录揉库后的终点
					rk_y = g_py;
					rk_theta = g_pth;

				}
			}

			// 给路
			if (fabs(rk_y) < 0.3)
			{
				path_point mid_LR;
				mid_LR.x = Start[0];
				mid_LR.y = Start[1];
				mid_LR.th = mod2pi(Start[2]);
				mid_LR.D = 0;
				mid_LR.delta = 0;
				pathpoint.push_back(mid_LR);

				double rk_x_D = Start[0]; // 记录揉库后的终点
				double rk_y_D = Start[1];
				double rk_theta_D = mod2pi(Start[2]);

				int mod_point = point_rk; // 记录剩余的点数

				for (int index = 0; index < ceil(point_rk * 1.0 / point_adv); index++)
				{
					int gears_DR = index % 2; // 取余来判断挡位
					mod_point = mod_point - point_adv;
					int idx_max = point_adv;
					if (mod_point >= 0)
					{
						idx_max = idx_max;
					}
					else
					{
						idx_max = mod_point + point_adv;
					}

					if (gears_DR == 0) // 后退
					{
						for (int index_1 = 0; index_1 < idx_max; index_1++)
						{
							VehicleDynamic(rk_x_D, rk_y_D, rk_theta_D, D_idx_R, delta_idx_R);
							mid_LR.x = g_px;
							mid_LR.y = g_py;
							mid_LR.th = g_pth;
							mid_LR.D = D_idx_R;
							mid_LR.delta = delta_idx_R;
							pathpoint.push_back(mid_LR);
							rk_x_D = g_px; // 记录揉库后的终点
							rk_y_D = g_py;
							rk_theta_D = g_pth;
						}
					}
					else  // 前进
					{
						for (int index_1 = 0; index_1 < idx_max; index_1++)
						{
							VehicleDynamic(rk_x_D, rk_y_D, rk_theta_D, D_idx_D, delta_idx_D);
							mid_LR.x = g_px;
							mid_LR.y = g_py;
							mid_LR.th = g_pth;
							mid_LR.D = D_idx_D;
							mid_LR.delta = delta_idx_D;
							pathpoint.push_back(mid_LR);
							rk_x_D = g_px; // 记录揉库后的终点
							rk_y_D = g_py;
							rk_theta_D = g_pth;
						}
					}
				}
				return 0;
			}
			else
			{
				return 2;
			}

		}
	}
	else
	{
		if (fabs(theta_ang_seven) < 1 * pi / 180) //判断车头方向是否超调，如果超调证明空间足够，请求A*
		{
			return 2;     // 角度小于1°，y向偏差大于30cm，失败交给A*
		}
		else if (theta_ang_seven < -60 * pi / 180)
		{
			return 1;        // 夹角大于60°，失败
		}

		int point_rk = round(fabs(theta_ang_seven) / 0.0182);  // 需要多少点才能将车辆角度调正

		if ((control.ObsUssInfo == 6u) || (control.ObsUssInfo == 7u) || (control.ObsUssInfo == 8u) || (control.ObsUssInfo == 9u) || (Gears == 2)) // 后退被逼停
		{
			/*if (y_seven < -0.3)
			{
				return 2; // 揉正外移大于30cm
			}*/

			double D_adv_ = front_path - x_seven; // 车辆前进最多走的距离
			double D_adv = rmin*(fabs(theta_ang_seven)- asin(sin(fabs(theta_ang_seven))- D_adv_/rmin));
			int point_adv;                       // 揉库最少的点数，每个点10cm
			if (D_adv < 0.2)
			{
				return 3; // 车辆停在可揉库范围外
			}
			else
			{
				point_adv = round(D_adv / 0.1);
			}
			int point_adv_min = point_rk / 6;
			if (point_adv_min <= 2) // 最少走20cm
			{
				point_adv_min = 2;
			}
			if (point_adv_min > point_adv)
			{
				return 3; // 揉库次数不够
			}

			double diff_y = 100; // 记录y向偏差
			int rk_idx = 0;        // 记录揉库次数
			int point_large = 100;

			double delta_idx_D = atan(vehicle_parameters.WB / rmin);
			double D_idx_D = 0.1;
			double delta_idx_R = -atan(vehicle_parameters.WB / rmin);
			double D_idx_R = -0.1;

			for (int idx = point_adv; idx > point_adv_min - 1; idx--) // 遍历所有可能的步长
			{
				double rk_x = x_seven; // 记录揉库后的终点
				double rk_y = y_seven;
				double rk_theta = theta_ang_seven;

				int mod_point = point_rk; // 记录剩余的点数

				for (int index = 0; index < ceil(point_rk * 1.0 / point_adv); index++)
				{
					int gears_DR = index % 2; // 取余来判断挡位
					mod_point = mod_point - idx;
					int idx_max = idx;
					if (mod_point >= 0)
					{
						idx_max = idx_max;
					}
					else
					{
						idx_max = mod_point + idx;
					}

					if (gears_DR == 0) // 前进
					{
						rk_idx = rk_idx + 1;

						VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_D*idx_max, delta_idx_D);

						rk_x = g_px; // 记录揉库后的终点
						rk_y = g_py;
						rk_theta = g_pth;
					}
					else  // 后退
					{
						rk_idx = rk_idx + 1;

						VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_R*idx_max, delta_idx_R);

						rk_x = g_px; // 记录揉库后的终点
						rk_y = g_py;
						rk_theta = g_pth;

					}
				}

				if ((fabs(rk_y) < 0.3) && (rk_idx <= (6 - small_level_park))) // 如果y向偏差在30cm输出
				{
					diff_y = rk_y;
					point_large = idx;
					break;
				}
				else
				{
					if ((fabs(diff_y) > fabs(rk_y)) && (rk_idx <= (6 - small_level_park)))
					{
						diff_y = rk_y;
						point_large = idx;
					}
				}
			}
			// 给路
			if (fabs(diff_y) < 0.3)
			{
				path_point mid_LR;
				mid_LR.x = Start[0];
				mid_LR.y = Start[1];
				mid_LR.th = mod2pi(Start[2]);
				mid_LR.D = 0;
				mid_LR.delta = 0;
				pathpoint.push_back(mid_LR);

				double rk_x = Start[0]; // 记录揉库后的终点
				double rk_y = Start[1];
				double rk_theta = mod2pi(Start[2]);

				int mod_point = point_rk; // 记录剩余的点数

				for (int index = 0; index < ceil(point_rk * 1.0 / point_large); index++)
				{
					int gears_DR = index % 2; // 取余来判断挡位
					mod_point = mod_point - point_large;
					int idx_max = point_large;
					if (mod_point >= 0)
					{
						idx_max = idx_max;
					}
					else
					{
						idx_max = mod_point + point_large;
					}

					if (gears_DR == 0) // 前进
					{
						for (int index_1 = 0; index_1 < idx_max; index_1++)
						{
							VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_D, delta_idx_D);
							mid_LR.x = g_px;
							mid_LR.y = g_py;
							mid_LR.th = g_pth;
							mid_LR.D = D_idx_D;
							mid_LR.delta = delta_idx_D;
							pathpoint.push_back(mid_LR);
							rk_x = g_px; // 记录揉库后的终点
							rk_y = g_py;
							rk_theta = g_pth;
						}
					}
					else  // 后退
					{
						for (int index_1 = 0; index_1 < idx_max; index_1++)
						{
							VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_R, delta_idx_R);
							mid_LR.x = g_px;
							mid_LR.y = g_py;
							mid_LR.th = g_pth;
							mid_LR.D = D_idx_R;
							mid_LR.delta = delta_idx_R;
							pathpoint.push_back(mid_LR);
							rk_x = g_px; // 记录揉库后的终点
							rk_y = g_py;
							rk_theta = g_pth;
						}
					}
				}
				return 0;
			}
			else
			{
				return 2;
			}

		}
		else if ((control.ObsUssInfo == 0u) || (control.ObsUssInfo == 1u) || (control.ObsUssInfo == 2u) || (control.ObsUssInfo == 3u) || (Gears == 1)) // 前进被逼停
		{
			/*if (y_seven > 0.3)
			{
				return 2; // 揉正内移大于30cm
			}*/
			double D_adv_ = x_seven - back_path; // 车辆后退最多走的距离
			double D_adv = rmin*(fabs(theta_ang_seven)- asin(sin(fabs(theta_ang_seven))- D_adv_/rmin));
			int point_adv;                       // 揉库最少的点数，每个点10cm
			if (D_adv < 0.2)
			{
				return 3; // 车辆停在可揉库范围外
			}
			else
			{
				point_adv = round(D_adv / 0.1);
			}

			double delta_idx_D = atan(vehicle_parameters.WB / rmin);
			double D_idx_D = 0.1;
			double delta_idx_R = -atan(vehicle_parameters.WB / rmin);
			double D_idx_R = -0.1;

			double rk_x = x_seven; // 记录揉库后的终点
			double rk_y = y_seven;
			double rk_theta = theta_ang_seven;

			int mod_point = point_rk; // 记录剩余的点数

			for (int index = 0; index < ceil(point_rk * 1.0 / point_adv); index++)
			{
				int gears_DR = index % 2; // 取余来判断挡位
				mod_point = mod_point - point_adv;
				int idx_max = point_adv;
				if (mod_point >= 0)
				{
					idx_max = idx_max;
				}
				else
				{
					idx_max = mod_point + point_adv;
				}

				if (gears_DR == 0) // 后退
				{
					VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_R*idx_max, delta_idx_R);

					rk_x = g_px; // 记录揉库后的终点
					rk_y = g_py;
					rk_theta = g_pth;
				}
				else  // 前进
				{
					VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_D*idx_max, delta_idx_D);

					rk_x = g_px; // 记录揉库后的终点
					rk_y = g_py;
					rk_theta = g_pth;

				}
			}

			// 给路
			if (fabs(rk_y) < 0.3)
			{
				path_point mid_LR;
				mid_LR.x = Start[0];
				mid_LR.y = Start[1];
				mid_LR.th = mod2pi(Start[2]);
				mid_LR.D = 0;
				mid_LR.delta = 0;
				pathpoint.push_back(mid_LR);

				double rk_x_D = Start[0]; // 记录揉库后的终点
				double rk_y_D = Start[1];
				double rk_theta_D = mod2pi(Start[2]);

				int mod_point = point_rk; // 记录剩余的点数

				for (int index = 0; index < ceil(point_rk * 1.0 / point_adv); index++)
				{
					int gears_DR = index % 2; // 取余来判断挡位
					mod_point = mod_point - point_adv;
					int idx_max = point_adv;
					if (mod_point >= 0)
					{
						idx_max = idx_max;
					}
					else
					{
						idx_max = mod_point + point_adv;
					}

					if (gears_DR == 0) // 后退
					{
						for (int index_1 = 0; index_1 < idx_max; index_1++)
						{
							VehicleDynamic(rk_x_D, rk_y_D, rk_theta_D, D_idx_R, delta_idx_R);
							mid_LR.x = g_px;
							mid_LR.y = g_py;
							mid_LR.th = g_pth;
							mid_LR.D = D_idx_R;
							mid_LR.delta = delta_idx_R;
							pathpoint.push_back(mid_LR);
							rk_x_D = g_px; // 记录揉库后的终点
							rk_y_D = g_py;
							rk_theta_D = g_pth;
						}	
					}
					else  // 前进
					{
						for (int index_1 = 0; index_1 < idx_max; index_1++)
						{
							VehicleDynamic(rk_x_D, rk_y_D, rk_theta_D, D_idx_D, delta_idx_D);
							mid_LR.x = g_px;
							mid_LR.y = g_py;
							mid_LR.th = g_pth;
							mid_LR.D = D_idx_D;
							mid_LR.delta = delta_idx_D;
							pathpoint.push_back(mid_LR);
							rk_x_D = g_px; // 记录揉库后的终点
							rk_y_D = g_py;
							rk_theta_D = g_pth;
						}
					}
				}
				return 0;
			}
			else
			{
				return 2;
			}

		}
	}
	return 2;
}

int  dynamic_Level(double Start1[3], double End1[3])
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
	double road_yaw2[6] = { 0,0,0,0,0,0 };
	//C-
	int roadfirst = 1;
	roadfirst = road_find_first_Level(Start[0], Start[1], Start[2], End[0], End[1], End[2], 0.05, R_L, 0.3, road_yaw);
	if (roadfirst == 0)
	{
		roadfirst = RoadCollision_Level(Start, End, road_yaw);
		if (roadfirst == 0)
		{
			LOG_WARNING("roadfirst=%d", roadfirst);
			//fflush(bydapa::common::Log::GetInstance()->fileptr());
			getroadpath_Level(Start, road_yaw);
			return 1;
		}
	}
	//C-S-
	int roadsecond = 1;
	roadsecond = road_find_three_cs_Level(Start[0], Start[1], Start[2] * 180 / pi, End[0], End[1], End[2] * 180 / pi, R_L, 0.0, road_yaw);
	if (roadsecond == 0)
	{
		roadsecond = RoadCollision_Level(Start, End, road_yaw);
		if (roadsecond == 0)
		{
			LOG_WARNING("roadsecond=%d", roadsecond);
			//fflush(bydapa::common::Log::GetInstance()->fileptr());
			getroadpath_Level(Start, road_yaw);
			return 1;
		}
	}
	//C-C-
	
	int roadthird = 1;
	roadthird = road_find_two_Level(Start[0], Start[1], Start[2], End[0], End[1], End[2], R_L, road_yaw);
	if (roadthird == 0)
	{
		roadthird = RoadCollision_Level(Start, End, road_yaw);
		if (roadthird == 0)
		{
			LOG_WARNING("roadthird=%d", roadthird);
			//fflush(bydapa::common::Log::GetInstance()->fileptr());
			getroadpath_Level(Start, road_yaw);
			return 1;
		}
	}
	
	//C+S- 最小半径
	int roadfourth = 1;
	roadfourth = road_find_four_Level(Start[0], Start[1], Start[2] * 180 / pi, End[0], End[1], End[2] * 180 / pi, R_L, 0.05, road_yaw);
	if (roadfourth == 0)
	{
		roadfourth = RoadCollision_Level(Start, End, road_yaw);
		if (roadfourth == 0)
		{
			LOG_WARNING("roadfourth=%d", roadfourth);
			//fflush(bydapa::common::Log::GetInstance()->fileptr());
			getroadpath_Level(Start, road_yaw);
			return 2;
		}
	}

	double level_seven_D = sqrt((Start[0] - End[0])*(Start[0] - End[0]) + (Start[1] - End[1])*(Start[1] - End[1]));
	if (level_seven_D < 1) // 1米内才揉库
	{
		int roadseven = 1;
		roadseven = road_find_seven1_Level(Start[0], Start[1], Start[2], End[0], End[1], End[2], R_L, road_yaw);
		if (roadseven == 0)
		{
			roadseven = RoadCollision_Level(Start, End, road_yaw);
			if (roadseven == 0)
			{
				LOG_WARNING("roadseven=%d", roadseven);
				//fflush(bydapa::common::Log::GetInstance()->fileptr());
				getroadpath_Level(Start, road_yaw);
				return 2;
			}
		}

		if (roadseven != 0)
		{
			roadseven = road_find_seven2_Level(Start[0], Start[1], Start[2], End[0], End[1], End[2], R_L);
			if (roadseven == 0)
			{
				small_level_park = small_level_park + 1;
				return 2;
			}
		}

	}

	return 0;
}
