#include "../common/global_variable.h"
#include "../common/global_function.h"

namespace byd_apa_plan
{
	bool VehicleCollisionGridLevel(double cpx, double cpy, double cph);
	int RoadCollision_Level(double Start[3], double End[3], double road_yaw[6]);
	// 几何
	int RoadCollision_Level_RD(double Start[3], double End[3], double R, double D, int &point_collision);
	int Cc_road_th(double Start[3], double End[3], double Start_0[3], double &D, double &R, int index_front);
	int Cc_road(double Start[3], double End[3], int right_left, double road_Cc[6]);
	int Roll_Cc(double Start1[3], double Start_collision_mid1[3], int right_left, double R_level_1, double D_level_1, double road_roll[28]);
	void getroadpath_Level_all(double Start[3], double road_roll[28], double D, double R);
	int back_level(double x, double y, double theta, double backpath, double frontpath, int point_rk, double &back_x, double &back_y, double &point_adv_);
	int getroadpath_Level_back(double point_rk, double point_adv, double back_x, double theta);
	int front_level(double x, double y, double theta, double backpath, double frontpath, int point_rk, double &front_x, double &front_y, double &point_large_);
	int getroadpath_Level_front(double point_rk, double point_large, double front_x, double theta);
	int Roll_park(double Start1[3], double End1[3], int right_left, double backpath, double frontpath, double road_roll[28]);
	int roll_level_park(double Start1[3], double End1[3], int right_left, double road_roll[28]);
	int  geometry_Level(double Start1[3], double End1[3]);
	

	bool VehicleCollisionGridLevel(double cpx, double cpy, double cph)
	{   
		bool isCollision = 0;
		double phi = mod2pi(cph); // 构造旋转矩阵
		double cosphi = cos(-phi);
		double sinphi = sin(-phi);
		double rect_x;
		double rect_y;

		for (unsigned int i = 0; i < Rect_x20.size(); i++)
		{
			rect_x = Rect_x20[i] * cosphi + Rect_y20[i] * sinphi + cpx;
			rect_y = Rect_y20[i] * cosphi - Rect_x20[i] * sinphi + cpy;
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
			int idx_xy = (xidx - 1) * pathfind_parameters.XIDX + (yidx - 1);
			if ((xidx <= 0 || xidx > pathfind_parameters.XIDX) || (yidx <= 0 || yidx > pathfind_parameters.XIDY))
			{
				/*isCollision = true;
				return isCollision;*/
			}
			else if ((obstmap[idx_xy].Status == 0) || (obstmap[idx_xy].Status == 3) || (obstmap[idx_xy].Status == 4))
			{
				isCollision = true;
				return isCollision;
			}
		}
		return isCollision;
	}

	void getroadpath_Level_all(double Start[3], double road_roll[28], double D, double R)
	{
		PathPoint mid_L;
		double D_L = 0;
		double delta_L = 0;
		mid_L.x = Start[0];
		mid_L.y = Start[1];
		mid_L.th = mod2pi(Start[2]);
		mid_L.D = 0;
		mid_L.delta = 0;
		pathpoint.push_back(mid_L);
		if (D > 0)
		{
			D_L = 0.1;
		}
		else if (D < 0)
		{
			D_L = -0.1;
		}
		if (fabs(R) > 100000 || fabs(R) < rmin_level - 0.1)
		{
			delta_L = 0;
		}
		else
		{
			delta_L = atan(vehicle_parameters.WB / R);
		}
		int i = fabs(D) / pathfind_parameters.MOTION_RESOLUTION;
		for (int idx_L = 0; idx_L < i; idx_L++) // round()四舍五入
		{
			VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D_L, delta_L); // 根据当前位姿和输入, 计算下一位置的位姿
			mid_L.x = g_px;
			mid_L.y = g_py;
			mid_L.th = g_pth;
			mid_L.D = D_L;
			mid_L.delta = delta_L;
			pathpoint.push_back(mid_L);
		}
		if (fabs(D - i * D_L) > 0.01)
		{
			VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D - i * D_L, delta_L); // 根据当前位姿和输入, 计算下一位置的位姿
			mid_L.x = g_px;
			mid_L.y = g_py;
			mid_L.th = g_pth;
			mid_L.D = D - i * D_L;
			mid_L.delta = delta_L;
			pathpoint.push_back(mid_L);
		}
	

		for (int index = 0; index < 14; index++)
		{
			if (road_roll[index * 2 + 1] > 0)
			{
				D_L = 0.1;
			}
			else if (road_roll[index * 2 + 1] < 0)
			{
				D_L = -0.1;
			}
			if (fabs(road_roll[index * 2]) > 100000 || fabs(road_roll[index * 2]) < rmin_level - 0.1)
			{
				delta_L = 0;
			}
			else
			{
				delta_L = atan(vehicle_parameters.WB / road_roll[index * 2]);
			}
			int ii = fabs(road_roll[index * 2 + 1]) / pathfind_parameters.MOTION_RESOLUTION;
			for (int idx_L = 0; idx_L < ii; idx_L++) // round()四舍五入
			{
				VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D_L, delta_L); // 根据当前位姿和输入, 计算下一位置的位姿
				mid_L.x = g_px;
				mid_L.y = g_py;
				mid_L.th = g_pth;
				mid_L.D = D_L;
				mid_L.delta = delta_L;
				if (fabs(road_roll[index * 2 + 1])>0.099)
				{
					pathpoint.push_back(mid_L);
				}
			
			}
			if (fabs(road_roll[index * 2 + 1] - ii * D_L) > 0.01)
			{
				VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, road_roll[index * 2 + 1] - ii * D_L, delta_L); // 根据当前位姿和输入, 计算下一位置的位姿
				mid_L.x = g_px;
				mid_L.y = g_py;
				mid_L.th = g_pth;
				mid_L.D = road_roll[index * 2 + 1] - ii * D_L;
				mid_L.delta = delta_L;
				if (fabs(road_roll[index * 2 + 1])>0.099)
				{
					pathpoint.push_back(mid_L);
				}
			
			}
		
		}
	}

	int RoadCollision_Level_RD(double Start[3], double End[3], double R, double D, int &point_collision) // 判断某一段单一曲率路径在哪个点碰撞
	{
		double x = Start[0];
		double y = Start[1];
		double th = mod2pi(Start[2]);
		double D_Level = 0;
		double delta_Level = 0;

		if (D > 0) // 前进还是后退
		{
			D_Level = 0.1;
		}
		else if (D < 0)
		{
			D_Level = -0.1;
		}

		if (fabs(R) < rmin_level - 0.1)
		{
			delta_Level = 0;
		}
		else
		{
			delta_Level = atan(vehicle_parameters.WB / R);
		}

		int roadisCollision = 0;
		int i = fabs(D) / pathfind_parameters.MOTION_RESOLUTION;
		for (int idx_L = 0; idx_L < i; idx_L++) // round()四舍五入
		{
			VehicleDynamic(x, y, th, D_Level, delta_Level); // 根据当前位姿和输入, 计算下一位置的位姿
			roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth);
			if (roadisCollision)
			{
				point_collision = idx_L - 1;
				End[0] = x;
				End[1] = y;
				End[2] = th;
				return 1;
			}
			x = g_px;
			y = g_py;
			th = g_pth;
		}
		if (fabs(D - i * D_Level) > 0.01)
		{
			VehicleDynamic(x, y, th, D - i * D_Level, delta_Level); // 根据当前位姿和输入, 计算下一位置的位姿
			roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth);
			if (roadisCollision)
			{
				point_collision = i;
				End[0] = x;
				End[1] = y;
				End[2] = th;
				return 1;
			}
			x = g_px;
			y = g_py;
			th = g_pth;
		}

		End[0] = x;
		End[1] = y;
		End[2] = th;

		return 0;
	}

	int Cc_road_th(double Start[3], double End[3], double Start_0[3], double &D, double &R, int right_left)
	{
		double r_Cc = rmin_level;
		// 转化为起点坐标系
		double X_Cc = (Start[0] - End[0]) * cos(End[2]) + (Start[1] - End[1]) * sin(End[2]);
		double Y_Cc = (Start[1] - End[1]) * cos(End[2]) - (Start[0] - End[0]) * sin(End[2]);
		double th_Cc = mod2pi(Start[2] - End[2]);
		double R_D = 0;
		double R_R = 0;

		if (fabs(Y_Cc) > 5) // 车辆距离车位过远
		{
			return 0;
		}

		// 起点终点角度偏差大于2度需调正  10*sin(2*pi/180) = 0.3490
		double Start_D_00[3] = { 0,0,0 };
		double Start_R_00[3] = { 0,0,0 };
		if (th_Cc > 0.035)
		{
			Start_D_00[0] = X_Cc + r_Cc * sin(th_Cc);
			Start_D_00[1] = Y_Cc + r_Cc - r_Cc * cos(th_Cc);
			Start_D_00[2] = 0;
			R_D = -r_Cc;
			Start_R_00[0] = X_Cc - r_Cc * sin(th_Cc);
			Start_R_00[1] = Y_Cc - r_Cc + r_Cc * cos(th_Cc);
			Start_R_00[2] = 0;
			R_R = r_Cc;
		}
		else if (th_Cc < -0.035)
		{
			Start_D_00[0] = X_Cc - r_Cc * sin(th_Cc);
			Start_D_00[1] = Y_Cc - r_Cc + r_Cc * cos(th_Cc);
			Start_D_00[2] = 0;
			R_D = r_Cc;
			Start_R_00[0] = X_Cc + r_Cc * sin(th_Cc);
			Start_R_00[1] = Y_Cc + r_Cc - r_Cc * cos(th_Cc);
			Start_R_00[2] = 0;
			R_R = -r_Cc;
		}
		else
		{
			Start_0[0] = Start[0];
			Start_0[1] = Start[1];
			Start_0[2] = Start[2];
			D = 0;
			R = 0;
			return 1;
		}

		double road_Cc[6] = { 0 };
		int a = Cc_road(Start, End, right_left, road_Cc);
		if (a == 0)
		{
			Start_0[0] = Start[0];
			Start_0[1] = Start[1];
			Start_0[2] = Start[2];
			D = 0;
			R = 0;
			return 1;
		}

		if (road_Cc[1] > 0)
		{
			Start_0[0] = Start_D_00[0] * cos(End[2]) - Start_D_00[1] * sin(End[2]) + End[0];
			Start_0[1] = Start_D_00[1] * cos(End[2]) + Start_D_00[0] * sin(End[2]) + End[1];
			Start_0[2] = End[2];
			R = R_D;
			D = fabs(th_Cc * r_Cc);

		}
		else
		{
			Start_0[0] = Start_R_00[0] * cos(End[2]) - Start_R_00[1] * sin(End[2]) + End[0];
			Start_0[1] = Start_R_00[1] * cos(End[2]) + Start_R_00[0] * sin(End[2]) + End[1];
			Start_0[2] = End[2];
			R = R_R;
			D = -fabs(th_Cc * r_Cc);
		}

		return 1;
	}

	int Cc_road(double Start[3], double End[3], int right_left, double road_Cc[6])
	{
		double R_Cc = 8;
		double r_Cc = rmin_level;
		// 转化为起点坐标系
		double X_Cc = (End[0] - Start[0]) * cos(Start[2]) + (End[1] - Start[1]) * sin(Start[2]);
		double Y_Cc = (End[1] - Start[1]) * cos(Start[2]) - (End[0] - Start[0]) * sin(Start[2]);
		double th_Cc = mod2pi(End[2] - Start[2]);

		if (fabs(Y_Cc) > 5) // 车辆距离车位过远
		{
			return 0;
		}

		if (right_left == 1)
		{
			double th_C = acos((R_Cc + r_Cc * cos(th_Cc) - fabs(Y_Cc)) / (R_Cc + r_Cc));
			if (th_Cc >= 0)
			{
				road_Cc[0] = 1e6;
				road_Cc[1] = X_Cc + (r_Cc + R_Cc) * sin(th_C) - r_Cc * sin(th_Cc);
				/*if ((fabs(road_Cc[1]) > 0) && (fabs(road_Cc[1]) < 0.1))
				{
					road_Cc[1] = 0;
				}*/
				road_Cc[2] = -R_Cc;
				road_Cc[3] = -R_Cc * th_C;
				road_Cc[4] = r_Cc;
				road_Cc[5] = -r_Cc * (th_C - th_Cc);
			}
			else
			{
				road_Cc[0] = 1e6;
				road_Cc[1] = X_Cc + (r_Cc + R_Cc) * sin(th_C) + r_Cc * sin(th_Cc);
				/*if ((fabs(road_Cc[1]) > 0) && (fabs(road_Cc[1]) < 0.1))
				{
					road_Cc[1] = 0;
				}*/
				road_Cc[2] = -R_Cc;
				road_Cc[3] = -R_Cc * th_C;
				road_Cc[4] = r_Cc;
				road_Cc[5] = -r_Cc * (th_C + th_Cc);
			}
		}
		else
		{
			double th_C = acos((R_Cc + r_Cc * cos(th_Cc) - fabs(Y_Cc)) / (R_Cc + r_Cc));
			if (th_Cc <= 0)
			{
				road_Cc[0] = 1e6;
				road_Cc[1] = X_Cc + (r_Cc + R_Cc) * sin(th_C) + r_Cc * sin(th_Cc);
				/*if ((fabs(road_Cc[1]) > 0) && (fabs(road_Cc[1]) < 0.1))
				{
					road_Cc[1] = 0;
				}*/
				road_Cc[2] = R_Cc;
				road_Cc[3] = -R_Cc * th_C;
				road_Cc[4] = -r_Cc;
				road_Cc[5] = -r_Cc * (th_C + th_Cc);
			}
			else
			{
				road_Cc[0] = 1e6;
				road_Cc[1] = X_Cc + (r_Cc + R_Cc) * sin(th_C) - r_Cc * sin(th_Cc);
				/*if ((fabs(road_Cc[1]) > 0) && (fabs(road_Cc[1]) < 0.1))
				{
					road_Cc[1] = 0;
				}*/
				road_Cc[2] = R_Cc;
				road_Cc[3] = -R_Cc * th_C;
				road_Cc[4] = -r_Cc;
				road_Cc[5] = -r_Cc * (th_C - th_Cc);
			}
		}
		if ((Gears == 1) && (road_Cc[1] > 0) && (backpath + road_Cc[1] <= 0))
		{
			return 0;
		}
		return 1;
	}

	int Roll_Cc(double Start1[3], double Start_collision_mid1[3], int right_left, double R_level_1, double D_level_1, double road_roll[28])
	{
		double r_Cc = rmin_level;
		double Start[3];
		Start[0] = Start1[0];
		Start[1] = Start1[1];
		Start[2] = Start1[2];
		double End_collision[3];
		double Start_collision_mid[3] = { 0,0,0 };
		Start_collision_mid[0] = Start_collision_mid1[0];
		Start_collision_mid[1] = Start_collision_mid1[1];
		Start_collision_mid[2] = Start_collision_mid1[2];
		int point_collision = 100;
		double Start_collision[3] = { 0,0,0 }; // 计算碰撞的起点坐标
		double road_Cc[6] = { 0,0,0,0,0,0 };

		double Croll_DR[20] = { 0 };
		for (int index_Roll5 = 0; index_Roll5 < 5; index_Roll5++)
		{
			for (int index_fb = 0; index_fb < 15; index_fb++)
			{ 
				double D_C = index_fb * 0.1 + 0.2;
				Start_collision[0] = Start_collision_mid[0];
				Start_collision[1] = Start_collision_mid[1];
				Start_collision[2] = Start_collision_mid[2];
				int index_level_1 = 0;
				for (int index = 0; index < index_Roll5+1; index++)
				{
					index_level_1 = RoadCollision_Level_RD(Start_collision, End_collision, r_Cc * right_left, D_C, point_collision); // 前进会不会碰撞
					if (index_level_1 == 1)
					{
						break;
					}
					Croll_DR[index * 4] = r_Cc * right_left;
					Croll_DR[index * 4 + 1] = -D_C;
					Start_collision[0] = End_collision[0];
					Start_collision[1] = End_collision[1];
					Start_collision[2] = End_collision[2];
					int index_level_2 = RoadCollision_Level_RD(Start_collision, End_collision, r_Cc * -right_left, -D_C, point_collision); // 后退会不会碰撞
					Croll_DR[index * 4 + 2] = r_Cc * -right_left;
					if (index_level_2 == 1)
					{
						Croll_DR[index * 4 + 3] = point_collision * 0.1;
					}
					else
					{
						Croll_DR[index * 4 + 3] = D_C;
					}
					Start_collision[0] = End_collision[0];
					Start_collision[1] = End_collision[1];
					Start_collision[2] = End_collision[2];
				}
				if (index_level_1 == 0)
				{
					int index_level_CC = 1;
					if (choice_cc == 0)
					{
						index_level_CC = Cc_road(Start, End_collision, right_left, road_Cc);
						Start_collision[0] = Start[0] + 0.15 * cos(Start[2]); // 预留15cm检测碰撞
						Start_collision[1] = Start[1] + 0.15 * sin(Start[2]);
						Start_collision[2] = Start[2];
					}
					else
					{
						index_level_CC = Calc_Cm(Start[0], Start[1], Start[2], End_collision[0], End_collision[1], End_collision[2], 0, rmin_level, 0, road_Cc);
						if (right_left == 1)
						{
							if (road_Cc[0] < 0)
							{
								index_level_CC = 1;
							}
						}
						else
						{
							if (road_Cc[0] > 0)
							{
								index_level_CC = 1;
							}
						}
						//printf("C111111111111111111111index_level_CC=%d\n",index_level_CC);
						if (index_level_CC != 0)
						{
							index_level_CC = Calc_CmCm(Start[0], Start[1], Start[2], End_collision[0], End_collision[1], End_collision[2], rmin_level, road_Cc);
							if (right_left == 1)
							{
								if ((road_Cc[0] > 0) && (road_Cc[2] < 0))
								{
									index_level_CC = 1;
								}
							}
							else
							{
								if ((road_Cc[0] < 0) && (road_Cc[2] > 0))
								{
									index_level_CC = 1;
								}
							}
							//printf("CC22222222222222222222index_level_CC=%d\n",index_level_CC);
						}
						Start_collision[0] = Start[0] + 0 * cos(Start[2]); // 不预留15cm检测碰撞
						Start_collision[1] = Start[1] + 0 * sin(Start[2]);
						Start_collision[2] = Start[2];
						//printf("@@@@@@@@@@@@@@@index_level_CC=%d\n",index_level_CC);
						if (index_level_CC == 0)
						{
							index_level_CC = 1;
						}
						else
						{
							index_level_CC = 0;
						}
						//printf("@@@@@@@@@@@@@@@index_level_CC=%d\n",index_level_CC);
					}
					if (index_level_CC == 1)
					{
						for (int index_Cc = 0; index_Cc < 3; index_Cc++)
						{
							index_level_CC = RoadCollision_Level_RD(Start_collision, End_collision, road_Cc[index_Cc * 2], road_Cc[index_Cc * 2 + 1], point_collision);
							if (index_level_CC == 0)
							{
								Start_collision[0] = End_collision[0];
								Start_collision[1] = End_collision[1];
								Start_collision[2] = End_collision[2];
							}
							else
							{
								break;
							}
						}
					}
					else
					{
						index_level_CC = 1;
					}
					if (index_level_CC == 0)
					{
						road_roll[0] = road_Cc[0];
						if (road_Cc[1] > 0)
						{
							road_roll[1] = road_Cc[1]+0.5;
						}
						else
						{
							road_roll[1] = road_Cc[1];
						}
						//road_roll[1] = road_Cc[1];
						road_roll[2] = road_Cc[2];
						road_roll[3] = road_Cc[3];
						road_roll[4] = road_Cc[4];
						road_roll[5] = road_Cc[5];
						road_roll[6] = Croll_DR[18];
						road_roll[7] = Croll_DR[19];
						road_roll[8] = Croll_DR[16];
						road_roll[9] = Croll_DR[17];
						road_roll[10] = Croll_DR[14];
						road_roll[11] = Croll_DR[15];
						road_roll[12] = Croll_DR[12];
						road_roll[13] = Croll_DR[13];
						road_roll[14] = Croll_DR[10];
						road_roll[15] = Croll_DR[11];
						road_roll[16] = Croll_DR[8];
						road_roll[17] = Croll_DR[9];
						road_roll[18] = Croll_DR[6];
						road_roll[19] = Croll_DR[7];
						road_roll[20] = Croll_DR[4];
						road_roll[21] = Croll_DR[5];
						road_roll[22] = Croll_DR[2];
						road_roll[23] = Croll_DR[3];
						road_roll[24] = Croll_DR[0];
						road_roll[25] = Croll_DR[1];
						road_roll[26] = R_level_1;
						road_roll[27] = D_level_1;

						return 1;
					}
				}
			}
		}

		return 0;
	}

	int roll_level_park(double Start1[3], double End1[3], int right_left, double road_roll[28])
	{
		//double r_Cc = 5.5;
		double End[3];
		double Start[3];
		Start[0] = Start1[0];
		Start[1] = Start1[1];
		Start[2] = Start1[2];
		End[0] = End1[0];
		End[1] = End1[1];
		End[2] = End1[2];
		int point_collision = 100;
		double Start_collision[3] = { 0,0,0 }; // 计算碰撞的起点坐标
		double End_collision[3] = { 0,0,0 };
		double R_level_1 = 1e6;
		double D_level_1 = 1.7;
		int safety_D = 0;
		int safety_D_max = 3;
		double road_Cc[6] = { 0,0,0,0,0,0 };
		///////////////////////////直线后退有解或无解最多退多少/////////////////////////////////////////////
		for (int index = -safety_D_max; index < 18; index++)
		{
			int index_level_1 = RoadCollision_Level_RD(End, End_collision, R_level_1, -0.1 * index, point_collision); // 直线往后退
			if (index == 17)
			{
				index_level_1 = 1;
			}
			int index_level_2 = 0;
			if (index_level_1 == 0) // 直线段不碰撞
			{
				if (choice_cc == 0)
				{
					index_level_2 = Cc_road(Start, End_collision, right_left, road_Cc);
				}
				else
				{
					safety_D_max = 0;
					//printf("@Start=%lf,%lf,%lf\n",Start[0], Start[1], Start[2]);
					//printf("@End_collision=%lf,%lf,%lf\n",End_collision[0], End_collision[1], End_collision[2]);
					index_level_2 = Calc_Cm(Start[0], Start[1], Start[2], End_collision[0], End_collision[1], End_collision[2], 0, rmin_level, 0, road_Cc);
					if (right_left == 1)
					{
						if (road_Cc[0] < 0)
						{
							index_level_2 = 1;
						}
					}
					else
					{
						if (road_Cc[0] > 0)
						{
							index_level_2 = 1;
						}
					}
					//printf("C111111111111111111111index_level_2=%d\n",index_level_2);
					if (index_level_2 != 0)
					{
						index_level_2 = Calc_CmCm(Start[0], Start[1], Start[2], End_collision[0], End_collision[1], End_collision[2], rmin_level, road_Cc);
						if (right_left == 1)
						{
							if ((road_Cc[0] > 0) && (road_Cc[2] < 0))
							{
								index_level_2 = 1;
							}
						}
						else
						{
							if ((road_Cc[0] < 0) && (road_Cc[2] > 0))
							{
								index_level_2 = 1;
							}
						}
						//printf("CC222222222222222222222index_level_2=%d\n",index_level_2);
					}
					//index_level_2 = road_find_two_Level(Start[0], Start[1], Start[2], End_collision[0], End_collision[1], End_collision[2], rmin_level, road_Cc);
					if (index_level_2 == 0)
					{
						index_level_2 = 1;
					}
					else
					{
						index_level_2 = 0;
					}
					//printf("road_Cc=%lf,%lf\n%lf,%lf\n%lf,%lf\n",road_Cc[0],road_Cc[1],road_Cc[2],road_Cc[3],road_Cc[4],road_Cc[5]);
				}
				if (index_level_2 == 1)
				{
					Start_collision[0] = Start[0];
					Start_collision[1] = Start[1];
					Start_collision[2] = Start[2];
					for (int index_Cc = 0; index_Cc < 3; index_Cc++)
					{
						index_level_2 = RoadCollision_Level_RD(Start_collision, End_collision, road_Cc[index_Cc * 2], road_Cc[index_Cc * 2 + 1], point_collision);
						if (index_level_2 == 0)
						{
							Start_collision[0] = End_collision[0];
							Start_collision[1] = End_collision[1];
							Start_collision[2] = End_collision[2];
						}
						else
						{
							break;
						}
					}
					if ((index_level_2 == 0) && (Gears == 1) && (road_Cc[1] > 0))
					{
						index_level_2 = 1;
					}
					if (index_level_2 == 0)
					{
						safety_D = safety_D + 1; // 满3就是前方预留30cm安全距离
						if (safety_D >= safety_D_max+1)
						{
							road_roll[0] = road_Cc[0];
							if (road_Cc[1] > 0)
							{
								road_roll[1] = road_Cc[1]+0.5;
							}
							else
							{
								road_roll[1] = road_Cc[1];
							}
							//road_roll[1] = road_Cc[1];
							road_roll[2] = road_Cc[2];
							road_roll[3] = road_Cc[3];
							road_roll[4] = road_Cc[4];
							road_roll[5] = road_Cc[5];
							road_roll[6] = R_level_1;
							road_roll[7] = 0.1 * index;
						
							return 1; // 前方预留30cm有解
						}
					}
				}
			}
			else
			{
				if (index > 1)
				{
					D_level_1 = 0.1 * (index - 1);
					break;
				}
			}
		}

		/*int index_level_L = RoadCollision_Level_RD(End, End_collision, R_level_1, D_level_1, point_collision);
		if (index_level_L == 1)
		{
			return 0;
		}*/
		double Start_collision_mid[3] = { End_collision[0],End_collision[1],End_collision[2] };
		///////////////////////////揉库求解///////////////////////////////////////////// 
		// 30cm如果碰撞就用20cm的C+C-揉库，20cm不做碰撞检测，防止规划失败，20cm相当于靠逼停带偏差泊入 20cm到100cm
		int level_return = Roll_Cc(Start, Start_collision_mid, right_left, R_level_1, D_level_1, road_roll);
		if (level_return == 1)
		{

			return 1;
		}

		return 0;
	}

	int back_level(double x, double y, double theta, double backpath, double frontpath, int point_rk, double &back_x, double &back_y, double &point_adv_)
	{
		double rmin_l = rmin_level;
		double D_adv_ = x - backpath; // 车辆后退最多走的距离
		if (D_adv_ < 0)
		{
			return 0;
		}
		double D_adv = rmin_l * (fabs(theta) - asin(sin(fabs(theta)) - D_adv_ / rmin_l));
		int point_adv;                       // 揉库最少的点数，每个点10cm
		if (D_adv < 0.2)
		{
			return 0; // 车辆停在可揉库范围外
		}
		else
		{
			point_adv = round(D_adv / 0.1);
		}

		double delta_idx_D = -atan(vehicle_parameters.WB / rmin_l);
		double D_idx_D = 0.1;
		double delta_idx_R = atan(vehicle_parameters.WB / rmin_l);
		double D_idx_R = -0.1;

		if (theta < 0)
		{
			delta_idx_D = -delta_idx_D;
			delta_idx_R = -delta_idx_R;
		}

		double rk_x = x; // 记录揉库后的终点
		double rk_y = y;
		double rk_theta = theta;



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
				VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_R * idx_max, delta_idx_R);

				rk_x = g_px; // 记录揉库后的终点
				rk_y = g_py;
				rk_theta = g_pth;
			}
			else  // 前进
			{
				VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_D * idx_max, delta_idx_D);

				rk_x = g_px; // 记录揉库后的终点
				rk_y = g_py;
				rk_theta = g_pth;

			}
		}
		back_x = -rk_x;
		back_y = rk_y;
		point_adv_ = point_adv;
		return 1;
	}

	int getroadpath_Level_back(double point_rk, double point_adv, double back_x, double theta)
	{
		double rmin_l = rmin_level;
		double delta_idx_D = -atan(vehicle_parameters.WB / rmin_l);
		double D_idx_D = 0.1;
		double delta_idx_R = atan(vehicle_parameters.WB / rmin_l);
		double D_idx_R = -0.1;

		if (theta < 0)
		{
			delta_idx_D = -delta_idx_D;
			delta_idx_R = -delta_idx_R;
		}

		PathPoint mid_LR;
		// mid_LR.x = start_to_fus[0]; // 原程序
		// mid_LR.y = start_to_fus[1];
		// mid_LR.th = mod2pi(start_to_fus[2]);
		// mid_LR.D = 0;
		// mid_LR.delta = 0;
		// pathpoint.push_back(mid_LR);

		// 1.19 //
		double rk_x_D;
		double rk_y_D;
		double rk_theta_D;
		if (plan_request == 0u)
		{
			rk_x_D = rk_sto[0];
			rk_y_D = rk_sto[1];
			rk_theta_D = mod2pi(rk_sto[2]);
		}
		else
		{
			mid_LR.x = start_to_fus[0];
			mid_LR.y = start_to_fus[1];
			mid_LR.th = mod2pi(start_to_fus[2]);
			mid_LR.D = 0;
			mid_LR.delta = 0;
			pathpoint.push_back(mid_LR);
			rk_x_D = start_to_fus[0]; // 记录揉库后的终点
			rk_y_D = start_to_fus[1];
			rk_theta_D = mod2pi(start_to_fus[2]);
		}

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
		if (fabs(back_x) > 0.01)
		{
			VehicleDynamic(rk_x_D, rk_y_D, rk_theta_D, back_x, 100000);
			mid_LR.x = g_px;
			mid_LR.y = g_py;
			mid_LR.th = g_pth;
			mid_LR.D = back_x;
			mid_LR.delta = 100000;
			pathpoint.push_back(mid_LR);
			rk_x_D = g_px; // 记录揉库后的终点
			rk_y_D = g_py;
			rk_theta_D = g_pth;
		}
	

		return 0;
	}

	int front_level(double x, double y, double theta, double backpath, double frontpath, int point_rk, double &front_x, double &front_y, double &point_large_)
	{
		double rmin_l = rmin_level;
		double D_adv_ = frontpath - x; // 车辆前进最多走的距离
		if (D_adv_ < 0)
		{
			return 0;
		}
		double D_adv = rmin_l * (fabs(theta) - asin(sin(fabs(theta)) - D_adv_ / rmin_l));
		int point_adv;                       // 揉库最少的点数，每个点10cm
		if (D_adv < 0.2)
		{
			return 0; // 车辆前近距离太小
		}
		else
		{
			point_adv = round(D_adv / 0.1);
		}
		int point_adv_min = round(point_rk / 10);
		if (point_adv_min <= 2) // 最少走20cm
		{
			point_adv_min = 2;
		}
		if (point_adv_min > point_adv)
		{
			return 0; // 揉库次数不够
		}

		double diff_x = 0;
		double diff_y = 100; // 记录y向偏差
		int rk_idx = 0;        // 记录揉库次数
		int point_large = 0;

		double delta_idx_D = -atan(vehicle_parameters.WB / rmin_l);
		double D_idx_D = 0.1;
		double delta_idx_R = atan(vehicle_parameters.WB / rmin_l);
		double D_idx_R = -0.1;
		if (theta < 0)
		{
			delta_idx_D = -delta_idx_D;
			delta_idx_R = -delta_idx_R;
		}

		for (int idx = point_adv; idx > point_adv_min - 1; idx--) // 遍历所有可能的步长
		{
			double rk_x = x; // 记录揉库后的终点
			double rk_y = y;
			double rk_theta = theta;

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
					VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_D * idx_max, delta_idx_D);

					rk_x = g_px; // 记录揉库后的终点
					rk_y = g_py;
					rk_theta = g_pth;
				}
				else  // 后退
				{
					rk_idx = rk_idx + 1;

					VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_R * idx_max, delta_idx_R);

					rk_x = g_px; // 记录揉库后的终点
					rk_y = g_py;
					rk_theta = g_pth;

				}
			}

			if ((fabs(rk_y) < 0.1) && (rk_idx <= (10))) // 如果y向偏差在10cm输出
			{
				diff_x = rk_x;
				diff_y = rk_y;
				point_large = idx;
				break;
			}
			else
			{
				if ((fabs(diff_y) > fabs(rk_y)) && (rk_idx <= (10)))
				{
					diff_x = rk_x;
					diff_y = rk_y;
					point_large = idx;
				}
			}
		}

		if (fabs(diff_y) < 0.5)
		{
			front_x = -diff_x;
			front_y = diff_y;
			point_large_ = point_large;
			return 1;
		}
		else
		{
			return 0;
		}
	}

	int getroadpath_Level_front(double point_rk, double point_large, double front_x, double theta)
	{
		double rmin_l = rmin_level;
		double delta_idx_D = -atan(vehicle_parameters.WB / rmin_l);
		double D_idx_D = 0.1;
		double delta_idx_R = atan(vehicle_parameters.WB / rmin_l);
		double D_idx_R = -0.1;
		if (theta < 0)
		{
			delta_idx_D = -delta_idx_D;
			delta_idx_R = -delta_idx_R;
		}

		PathPoint mid_LR;
		// 1.19 //
		double rk_x;
		double rk_y;
		double rk_theta;
		if (plan_request == 0u)
		{
			rk_x = rk_sto[0];
			rk_y = rk_sto[1];
			rk_theta = mod2pi(rk_sto[2]);
		}
		else
		{
			mid_LR.x = start_to_fus[0];
			mid_LR.y = start_to_fus[1];
			mid_LR.th = mod2pi(start_to_fus[2]);
			mid_LR.D = 0;
			mid_LR.delta = 0;
			pathpoint.push_back(mid_LR);
			rk_x = start_to_fus[0]; // 记录揉库后的终点
			rk_y = start_to_fus[1];
			rk_theta = mod2pi(start_to_fus[2]);
		}
		// 1.19 //
		
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
				if (ceil(point_rk * 1.0 / point_large) < 3)
				{
					for (int index_1 = 0; index_1 < 2; index_1++)
					{
						VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_D, 0);
						mid_LR.x = g_px;
						mid_LR.y = g_py;
						mid_LR.th = g_pth;
						mid_LR.D = D_idx_D;
						mid_LR.delta = 0;
						pathpoint.push_back(mid_LR);
						rk_x = g_px; // 记录揉库后的终点
						rk_y = g_py;
						rk_theta = g_pth;
					}
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
				if (ceil(point_rk * 1.0 / point_large) < 3)
				{
					for (int index_1 = 0; index_1 < 2; index_1++)
					{
						VehicleDynamic(rk_x, rk_y, rk_theta, D_idx_R, 0);
						mid_LR.x = g_px;
						mid_LR.y = g_py;
						mid_LR.th = g_pth;
						mid_LR.D = D_idx_R;
						mid_LR.delta = 0;
						pathpoint.push_back(mid_LR);
						rk_x = g_px; // 记录揉库后的终点
						rk_y = g_py;
						rk_theta = g_pth;
					}
				}
			}
		}
		if (ceil(point_rk * 1.0 / point_large) < 3)
		{
			double x_ctrl = (rk_x - end_to_fus[0]) * cos(end_to_fus[2]) + (rk_y - end_to_fus[1]) * sin(end_to_fus[2]);
			VehicleDynamic(rk_x, rk_y, rk_theta, -x_ctrl, 0);
			mid_LR.x = g_px;
			mid_LR.y = g_py;
			mid_LR.th = g_pth;
			mid_LR.D = -x_ctrl;
			mid_LR.delta = 0;
			pathpoint.push_back(mid_LR);
			rk_x = g_px; // 记录揉库后的终点
			rk_y = g_py;
			rk_theta = g_pth;
		}
		else
		{
			if (fabs(front_x) > 0.01)
			{
				VehicleDynamic(rk_x, rk_y, rk_theta, front_x, 0);
				mid_LR.x = g_px;
				mid_LR.y = g_py;
				mid_LR.th = g_pth;
				mid_LR.D = front_x;
				mid_LR.delta = 0;
				pathpoint.push_back(mid_LR);
				rk_x = g_px; // 记录揉库后的终点
				rk_y = g_py;
				rk_theta = g_pth;
			}
		}
	

		return 0;
	}

	int Roll_park(double Start1[3], double End1[3], int right_left, double backpath, double frontpath, double road_roll[28])
	{
		//double rmin_l = 5.5;
		double End[3];
		double Start[3];

		Start[0] = Start1[0];
		Start[1] = Start1[1];
		Start[2] = Start1[2];
		End[0] = End1[0];
		End[1] = End1[1];
		End[2] = End1[2];
		////////////////////////////////////揉库////////////////////////////////////////////////
		double y = (Start[1] - End[1]) * cos(End[2]) - (Start[0] - End[0]) * sin(End[2]); // 终点坐标系
		double x = (Start[0] - End[0]) * cos(End[2]) + (Start[1] - End[1]) * sin(End[2]);
		double theta = Start[2] - End[2];

		if (fabs(theta) < 1 * pi / 180)
		{
			if (fabs(y) < 0.2)  // 完成条件角度小于1°，y向偏差10cm
			{
				road_roll[0] = 1e6;
				road_roll[1] = -x;
				road_roll[2] = 0;
				road_roll[3] = 0;
				road_roll[4] = 0;
				road_roll[5] = 0;
				return 2;
			}
			else
			{
				return 0;     // 角度小于1°，y向偏差大于30cm，失败交给A*
			}
		}

		if (fabs(theta) > 60 * pi / 180)
		{
			return 0;        // 夹角大于60°，失败
		}

		int point_rk = round(fabs(theta) / (0.1/rmin_level));  // 需要多少点才能将车辆角度调正
		//double End_collision[3] = { 0,0,0 };

		if ((control.ObsUssInfo == 5u) || (control.ObsUssInfo == 6u) || (control.ObsUssInfo == 7u) || (control.ObsUssInfo == 8u) || (control.ObsUssInfo == 9u) || (control.ObsUssInfo == 10u))
		{
			double front_x;
			double front_y;
			double point_large_;
			//double D_adv_obs = x - backpath;
			//printf("D_adv_obs=%lf\n",D_adv_obs);
			double End_collision_40[3] = {0};
			int point_collision_40 = 0;
			int index_level_back_40 = RoadCollision_Level_RD(Start, End_collision_40, rmin_level * right_left, -0.5, point_collision_40);
			if (index_level_back_40 == 0)
			{
				/*double back_x;
				double back_y;
				double point_adv_;
				int b = back_level(x, y, theta, backpath, frontpath, point_rk, back_x, back_y, point_adv_);
				if (b == 1)
				{
					b = getroadpath_Level_back(point_rk, point_adv_, back_x, theta);
					return 1;
				}*/
				double front_x;
				double front_y;
				double point_large_;
				double back_x;
				double back_y;
				double point_adv_;
				int front = front_level(x, y, theta, backpath, frontpath, point_rk, front_x, front_y, point_large_);
				int back = back_level(x, y, theta, backpath, frontpath, point_rk, back_x, back_y, point_adv_);
				//printf("front=%d\n",front);
				//printf("back=%d\n",back);
				if ((front == 1) && (back == 1))
				{
					if (fabs(front_y) < fabs(back_y))
					{
						front = getroadpath_Level_front(point_rk, point_large_, front_x, theta);
						return 1;
					}
					else
					{
						back = getroadpath_Level_back(point_rk, point_adv_, back_x, theta);
						return 1;
					}
				}
				else if (front == 1)
				{
					front = getroadpath_Level_front(point_rk, point_large_, front_x, theta);
					return 1;
				}
				else if (back == 1)
				{
					back = getroadpath_Level_back(point_rk, point_adv_, back_x, theta);
					return 1;
				}
				else
				{
					return 0;
				}
			}
			int a = front_level(x, y, theta, backpath, frontpath, point_rk, front_x, front_y, point_large_);
			if (a == 1)
			{
				a = getroadpath_Level_front(point_rk, point_large_, front_x, theta);
				return 1;
			}
			else
			{
				double back_x;
				double back_y;
				double point_adv_;
				int b = back_level(x, y, theta, backpath, frontpath, point_rk, back_x, back_y, point_adv_);
				if (b == 1)
				{
					b = getroadpath_Level_back(point_rk, point_adv_, back_x, theta);
					return 1;
				}
				else
				{
					return 0;
				}
			}
		}
		else if ((control.ObsUssInfo == 11u) || (control.ObsUssInfo == 0u) || (control.ObsUssInfo == 1u) || (control.ObsUssInfo == 2u) || (control.ObsUssInfo == 3u) || (control.ObsUssInfo == 4u))
		{
			double back_x;
			double back_y;
			double point_adv_;
			//double D_adv_obs = frontpath - x;
			double End_collision_40[3] = {0};
			int point_collision_40 = 0;
			int index_level_back_40 = RoadCollision_Level_RD(Start, End_collision_40, rmin_level * -right_left, 0.5, point_collision_40);
			if (index_level_back_40 == 0)
			{
				/*double front_x;
				double front_y;
				double point_large_;
				int a = front_level(x, y, theta, backpath, frontpath, point_rk, front_x, front_y, point_large_);
				if (a == 1)
				{
					a = getroadpath_Level_front(point_rk, point_large_, front_x, theta);
					return 1;
				}*/
				double front_x;
				double front_y;
				double point_large_;
				double back_x;
				double back_y;
				double point_adv_;
				int front = front_level(x, y, theta, backpath, frontpath, point_rk, front_x, front_y, point_large_);
				int back = back_level(x, y, theta, backpath, frontpath, point_rk, back_x, back_y, point_adv_);
				if ((front == 1) && (back == 1))
				{
					if (fabs(front_y) < fabs(back_y))
					{
						front = getroadpath_Level_front(point_rk, point_large_, front_x, theta);
						return 1;
					}
					else
					{
						back = getroadpath_Level_back(point_rk, point_adv_, back_x, theta);
						return 1;
					}
				}
				else if (front == 1)
				{
					front = getroadpath_Level_front(point_rk, point_large_, front_x, theta);
					return 1;
				}
				else if (back == 1)
				{
					back = getroadpath_Level_back(point_rk, point_adv_, back_x, theta);
					return 1;
				}
				else
				{
					return 0;
				}
			}
			int b = back_level(x, y, theta, backpath, frontpath, point_rk, back_x, back_y, point_adv_);
			if (b == 1)
			{
				b = getroadpath_Level_back(point_rk, point_adv_, back_x, theta);
				return 1;
			}
			else
			{
				double front_x;
				double front_y;
				double point_large_;
				int a = front_level(x, y, theta, backpath, frontpath, point_rk, front_x, front_y, point_large_);
				if (a == 1)
				{
					a = getroadpath_Level_front(point_rk, point_large_, front_x, theta);
					return 1;
				}
				else
				{
					return 0;
				}
			}
		}
		else
		{
			double front_x;
			double front_y;
			double point_large_;
			double back_x;
			double back_y;
			double point_adv_;
			int front = front_level(x, y, theta, backpath, frontpath, point_rk, front_x, front_y, point_large_);
			int back = back_level(x, y, theta, backpath, frontpath, point_rk, back_x, back_y, point_adv_);
			if ((front == 1) && (back == 1))
			{
				if (fabs(front_y) < fabs(back_y))
				{
					front = getroadpath_Level_front(point_rk, point_large_, front_x, theta);
					return 1;
				}
				else
				{
					back = getroadpath_Level_back(point_rk, point_adv_, back_x, theta);
					return 1;
				}
			}
			else if (front == 1)
			{
				front = getroadpath_Level_front(point_rk, point_large_, front_x, theta);
				return 1;
			}
			else if (back == 1)
			{
				back = getroadpath_Level_back(point_rk, point_adv_, back_x, theta);
				return 1;
			}
			else
			{
				return 0;
			}
		}


		return 0;


	}

	int  geometry_Level(double Start1[3], double End1[3])
	{
		pathpoint.clear();
		double road_roll[28] = { 0 };
		double End[3];
		double Start[3];
		double Start_0[3] = { 0,0,0 }; // 选择的调正后起点

		Start[0] = Start1[0];
		Start[1] = Start1[1];
		Start[2] = Start1[2];
		End[0] = End1[0];
		End[1] = End1[1];
		End[2] = End1[2];
		//double X_C = (Start[0] - End[0]) * cos(End[2]) + (Start[1] - End[1]) * sin(End[2]);
		double Y_C = (Start[1] - End[1]) * cos(End[2]) - (Start[0] - End[0]) * sin(End[2]);
		//double th_C = mod2pi(Start[2] - End[2]);
		if (AstarOrGeo == 0)
		{
			if (fabs(Y_C) <= 1.2) 
			{
				int a = Roll_park(Start, End, right_left, backpath, frontpath, road_roll);
				if (a == 1)
				{
					return 1;
				}
				else if (a == 2)
				{
					getroadpath_Level_all(Start, road_roll, 0, 0);
					return 1;
				}
				else
				{
					return 0;
				}
			}
			else // 库外求解
			{
				double D_C = 0;
				double R_C = 0;
				int a = 0;
				if (choice_cc == 0)
				{
					int Y_diff = Cc_road_th(Start, End, Start_0, D_C, R_C, right_left);
					if (Y_diff == 0)
					{
						return 0;
					}
					a = roll_level_park(Start_0, End, right_left, road_roll);
					if ((R_C <= 0) && (road_roll[1] <= 0) && (road_roll[3] <= 0) && (road_roll[5] <= 0))
					{
						choice_cc = 1;
						///  /// 
						LevelDynamic = 1; 
					}
				}
				else
				{
					a = roll_level_park(Start, End, right_left, road_roll);
				}
			
				if (a == 1)
				{
					getroadpath_Level_all(Start, road_roll, D_C, R_C);
					return 1;
				}
			}
		}
		else
		{
			if (fabs(Y_C) <= 1.2) // 揉库
			{
			
				int a = Roll_park(Start, End, right_left, backpath, frontpath, road_roll);
				if (a == 1)
				{
					return 1;
				}
				else if (a == 2)
				{
					getroadpath_Level_all(Start, road_roll, 0, 0);
					return 1;
				}
				else
				{
					return 0;
				}
			}
		}

		return 0;
	}

	//////////////////////////////

	int RoadCollision_Level(double Start[3], double End[3], double road_yaw[6])
	{
		PathPoint mid_L;
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
		if (fabs(road_yaw[0]) < rmin_level - 0.1)
		{
			delta_L = 0;
		}
		else
		{
			delta_L = atan(vehicle_parameters.WB / road_yaw[0]);
		}
		if (fabs(road_yaw[2]) < rmin_level - 0.1)
		{
			delta_r = 0;
		}
		else
		{
			delta_r = atan(vehicle_parameters.WB / road_yaw[2]);
		}
		if (fabs(road_yaw[4]) < rmin_level - 0.1)
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
			roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth);
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
			roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth);
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
			roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth);
			if (roadisCollision)
			{
				return 1;
			}
		}

		return 0;
	}



	int  ParkingInLevel(double Start1[3], double End1[3])
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
		//double road_yaw2[6] = { 0,0,0,0,0,0 };
		//C-
		int roadfirst = 1;
		roadfirst = Calc_Cm(Start[0], Start[1], Start[2], End[0], End[1], End[2], 0.05, rmin_level, 0.05, road_yaw);
		if (roadfirst == 0)
		{
			roadfirst = RoadCollision_Level(Start, End, road_yaw);
			if (roadfirst == 0)
			{
				if (fabs(Start[1] - End[1]) >= 1 && plan_request == 1)  
				{
					LevelDynamic = 1; 
				    choice_cc = 1; 
				}
				GetOrCheckPathPoints(Start,road_yaw,3,0);
				return 1;
			}
		}
		//C-S-
		int roadsecond = 1;
		roadsecond = Calc_CmSm(Start[0], Start[1], Start[2] * 180 / pi, End[0], End[1], End[2] * 180 / pi, rmin_level, 0.1, road_yaw);
		if (roadsecond == 0 )
		{
			roadsecond = RoadCollision_Level(Start, End, road_yaw);
			if (roadsecond == 0)
			{
				if (fabs(Start[1] - End[1]) >= 1 && plan_request == 1) 
				{
					LevelDynamic = 1; 
				    choice_cc = 1; 
				}
				GetOrCheckPathPoints(Start,road_yaw,3,0);
				return 1;
			}
		}
		//C-C-
		int roadthird = 1;
		roadthird = Calc_CmCm(Start[0], Start[1], Start[2], End[0], End[1], End[2], rmin_level, road_yaw);
		if (roadthird == 0) 
		{
			roadthird = RoadCollision_Level(Start, End, road_yaw);
			if (roadthird == 0)
			{
				if (fabs(Start[1] - End[1]) >= 1 && plan_request == 1) 
				{
					LevelDynamic = 1; // 
				    choice_cc = 1; 
				}
				GetOrCheckPathPoints(Start, road_yaw, 3, 0);
				return 1;
			}
		}
		// 几何法
		int road_geometry = 0;
		road_geometry = geometry_Level(Start, End);
		if (road_geometry == 1)
		{
			return 2;
		}

		return 0;
	}
}
