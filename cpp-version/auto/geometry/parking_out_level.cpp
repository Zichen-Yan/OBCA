#include "../common/global_variable.h"
#include "../common/global_function.h"

namespace byd_apa_plan
{
	//
	int CSC_road(double Start1[3], double End1[3], double road_Cc[6]);
	int CCS_road(double Start1[3], double End1[3], double road_Cc[6]);
	//
	void getroadpath_Level_all_out(double Start[3], double road_roll[28], double D, double R);
	bool VehicleCollisionGridLevel_out(double cpx, double cpy, double cph, int Rect_);
	int RoadCollision_Level_RD_out(double Start[3], double End[3], double R, double D, int& point_collision, int Rect_);
	int getroadpath_Level_front_out(double Start_collision_mid1[3], double End1[3], int right_left, double R_level_1, double D_level_1, double road_roll[28]);
	int getroadpath_Level_back_out(double Start_collision_mid1[3], double End1[3], int right_left, double R_level_1, double D_level_1, double road_roll[28]);
	int Roll_out(double Start_collision_mid1[3], double End1[3], int right_left, double R_level_1, double D_level_1, double road_roll[28]);
	int roll_level_park_out(double Start1[3], double End1[3], int right_left, double road_roll[28]);
	/////////////////////////////////////////////////////////////////////
	void getroadpath_Level_all_out(double Start[3], double road_roll[28], double D, double R)
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
				pathpoint.push_back(mid_L);
			}
			if (fabs(road_roll[index * 2 + 1] - ii * D_L) > 0.01)
			{
				VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, road_roll[index * 2 + 1] - ii * D_L, delta_L); // 根据当前位姿和输入, 计算下一位置的位姿
				mid_L.x = g_px;
				mid_L.y = g_py;
				mid_L.th = g_pth;
				mid_L.D = road_roll[index * 2 + 1] - ii * D_L;
				mid_L.delta = delta_L;
				pathpoint.push_back(mid_L);
			}

		}

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
	}

	bool VehicleCollisionGridLevel_out(double cpx, double cpy, double cph, int Rect_)
	{
		bool isCollision = 0;
		double phi = mod2pi(cph); // 构造旋转矩阵
		double cosphi = cos(-phi);
		double sinphi = sin(-phi);
		double rect_x;
		double rect_y;
		
		int RS;
		if (Rect_ == 10 || Rect_ == 0)
		{
			RS = Rect_x0.size();
		}
		else
		{
			RS = Rect_x20.size();
		}

		for (int i = 0; i < RS; i++)
		{
			if (Rect_ == 10 || Rect_ == 0)
			{
				rect_x = Rect_x0[i] * cosphi + Rect_y0[i] * sinphi + cpx;
				rect_y = Rect_y0[i] * cosphi - Rect_x0[i] * sinphi + cpy;
			}
			else
			{
				rect_x = Rect_x20[i] * cosphi + Rect_y20[i] * sinphi + cpx;
				rect_y = Rect_y20[i] * cosphi - Rect_x20[i] * sinphi + cpy;
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
			int idx_xy = (xidx - 1) * pathfind_parameters.XIDX + (yidx - 1);
			if ((xidx <= 0 || xidx > pathfind_parameters.XIDX) || (yidx <= 0 || yidx > pathfind_parameters.XIDY))
			{
				isCollision = true;
				return isCollision;
			}

			if ((Rect_ == 2) && ((obstmap[idx_xy].Status == 0) || (obstmap[idx_xy].Status == 6) || (obstmap[idx_xy].Status == 4)))
			{
				isCollision = true;
				return isCollision;
			}
			else if ((Rect_ == 10) && ((obstmap[idx_xy].Status == 0) || (obstmap[idx_xy].Status == 4)))
			{
				isCollision = true;
				return isCollision;
			}
			else if ((obstmap[idx_xy].Status == 0) || (obstmap[idx_xy].Status == 3) || (obstmap[idx_xy].Status == 4))
			{
				isCollision = true;
				return isCollision;
			}
		}

		return isCollision;
	}

	int RoadCollision_Level_RD_out(double Start[3], double End[3], double R, double D, int& point_collision, int Rect_) // 判断某一段单一曲率路径在哪个点碰撞
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
			roadisCollision = VehicleCollisionGridLevel_out(g_px, g_py, g_pth, Rect_);
			if (roadisCollision)
			{
				point_collision = idx_L;
				End[0] = x;
				End[1] = y;
				End[2] = th;
				if (point_collision < 0)
				{
					point_collision = 0;
				}
				return 1;
			}
			x = g_px;
			y = g_py;
			th = g_pth;
		}
		if (fabs(D - i * D_Level) > 0.01)
		{
			VehicleDynamic(x, y, th, D - i * D_Level, delta_Level); // 根据当前位姿和输入, 计算下一位置的位姿
			roadisCollision = VehicleCollisionGridLevel_out(g_px, g_py, g_pth, Rect_);
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

	int CSC_road(double Start1[3], double End1[3], double road_Cc[6])
	{
		double rmin_ = rmin_level;
		double End[3];
		double Start[3];
		Start[0] = Start1[0];
		Start[1] = Start1[1];
		Start[2] = Start1[2];
		End[0] = End1[0];
		End[1] = End1[1];
		End[2] = End1[2];

		double ph = mod2pi(End[2] - Start[2]);
		double phi = mod2pi(Start[2]);
		// 起点start坐标系在基坐标系下的方向余弦矩阵(Z轴旋转，因为点在Z = 0平面，因此绕Z旋转仍然在Z = 0平面)
		// dcm*x 表示将基坐标中的x表示到旋转后的坐标系中，即计算坐标旋转后各向量在新坐标中的表示
		double x1 = (End[0] - Start[0]) * cos(phi) + (End[1] - Start[1]) * sin(phi);
		double y1 = (End[1] - Start[1]) * cos(phi) - (End[0] - Start[0]) * sin(phi);

		RsPath path_out;
		double x1_out = x1 / rmin_;
		double y1_out = y1 / rmin_;
		int index_level = CpSpCp(x1_out, y1_out, ph, path_out);
		int CSC_CCS = 1;
		if (index_level == 0)
		{
			index_level = CpCpS(x1_out, y1_out, ph, path_out);
			CSC_CCS = 2;
		}
		if (index_level == 1)
		{
			for (int index_CSC = 0; index_CSC < 3; index_CSC++)
			{
				if (path_out.rspath_type[index_CSC] == St)
				{
					road_Cc[index_CSC * 2] = 1e6;
				}

				else if (path_out.rspath_type[index_CSC] == Le)
				{
					road_Cc[index_CSC * 2] = rmin_;
				}

				else if (path_out.rspath_type[index_CSC] == Ri)
				{
					road_Cc[index_CSC * 2] = -rmin_;
				}
			}
			road_Cc[1] = path_out.t * rmin_;
			road_Cc[3] = path_out.u * rmin_;
			road_Cc[5] = path_out.v * rmin_;
			if (CSC_CCS == 1)
			{
				return 1;
			}
			else
			{
				return 2;
			}
		}
		else
		{
			return 0;
		}

		return 0;
	}

	int CCS_road(double Start1[3], double End1[3], double road_Cc[6])
	{
		double rmin_ = rmin_level;
		double End[3];
		double Start[3];
		Start[0] = Start1[0];
		Start[1] = Start1[1];
		Start[2] = Start1[2];
		End[0] = End1[0];
		End[1] = End1[1];
		End[2] = End1[2];

		double ph = mod2pi(End[2] - Start[2]);
		double phi = mod2pi(Start[2]);
		// 起点start坐标系在基坐标系下的方向余弦矩阵(Z轴旋转，因为点在Z = 0平面，因此绕Z旋转仍然在Z = 0平面)
		// dcm*x 表示将基坐标中的x表示到旋转后的坐标系中，即计算坐标旋转后各向量在新坐标中的表示
		double x1 = (End[0] - Start[0]) * cos(phi) + (End[1] - Start[1]) * sin(phi);
		double y1 = (End[1] - Start[1]) * cos(phi) - (End[0] - Start[0]) * sin(phi);

		RsPath path_out;
		double x1_out = x1 / rmin_;
		double y1_out = y1 / rmin_;
		int	index_level = CpCpS(x1_out, y1_out, ph, path_out);
		if (index_level == 1)
		{
			for (int index_CSC = 0; index_CSC < 3; index_CSC++)
			{
				if (path_out.rspath_type[index_CSC] == St)
				{
					road_Cc[index_CSC * 2] = 1e6;
				}

				else if (path_out.rspath_type[index_CSC] == Le)
				{
					road_Cc[index_CSC * 2] = rmin_;
				}

				else if (path_out.rspath_type[index_CSC] == Ri)
				{
					road_Cc[index_CSC * 2] = -rmin_;
				}
			}
			road_Cc[1] = path_out.t * rmin_;
			road_Cc[3] = path_out.u * rmin_;
			road_Cc[5] = path_out.v * rmin_;
			return 1;
		}
		else
		{
			return 0;
		}

		return 0;
	}

	int getroadpath_Level_front_out(double Start_collision_mid1[3], double End1[3], int right_left, double R_level_1, double D_level_1, double road_roll[28])
	{
		double r_Cc = rmin_level;
		double End[3];
		End[0] = End1[0];
		End[1] = End1[1];
		End[2] = End1[2];
		double End_collision[3];
		double Start_collision_mid[3] = { 0,0,0 };
		Start_collision_mid[0] = Start_collision_mid1[0];
		Start_collision_mid[1] = Start_collision_mid1[1];
		Start_collision_mid[2] = Start_collision_mid1[2];
		double Start_collision[3] = { 0,0,0 }; // 计算碰撞的起点坐标
		int point_collision = 100;
		double D_C = 3.5; // 2.5
		double Croll_DR[20] = { 0 };
		double road_Cc_[6] = { 0, 0, 0, 0, 0, 0 };
		// int safety_D = 0;
		// int safety_D_max = 2;
		double Start_collision_mid_[3] = { 0,0,0 };
		Start_collision_mid_[0] = Start_collision_mid1[0];
		Start_collision_mid_[1] = Start_collision_mid1[1];
		Start_collision_mid_[2] = Start_collision_mid1[2];
		VehicleDynamic(Start_collision_mid_[0], Start_collision_mid_[1], Start_collision_mid_[2], 0.3, atan(vehicle_parameters.WB / (r_Cc * right_left))); // 根据当前位姿和输入, 计算下一位置的位姿
		Start_collision_mid_[0] = g_px;
		Start_collision_mid_[1] = g_py;
		Start_collision_mid_[2] = g_pth;
		int index_level_1_ = RoadCollision_Level_RD_out(Start_collision_mid_, End_collision, r_Cc * right_left, D_C, point_collision, 1);
		if (index_level_1_ == 1)
		{
			D_C = (point_collision + 5) * 0.1;
		}
		else
		{
			D_C = D_C + 0.5;
		}
		//printf("D_C=%lf\n",D_C);
		///////////////////////////////////////////////////////////////////
		double road_Cc[6] = { 0 };
		int index_level_0 = 0;
		int indexCSC_50 = 0;
		//int max_CSCx = (r_Cc*sin(Start_collision_mid1[2])+fabs((fabs(End1[1]-Start_collision_mid1[1])-(r_Cc-r_Cc*cos(Start_collision_mid1[2])))/tan(Start_collision_mid1[2])))/0.1;
		for (int index_CSC50 = 15; index_CSC50 >= 0; index_CSC50--)
		{
			End_collision[0] = Start_collision_mid1[0];
			End_collision[1] = Start_collision_mid1[1];
			End_collision[2] = Start_collision_mid1[2];
			double End_[3];
			End_[0] = End[0] + index_CSC50 * 0.1;
			End_[1] = End[1];
			End_[2] = End[2];
			index_level_0 = CSC_road(End_collision, End_, road_Cc);
			if (index_level_0 == 1)
			{
				Start_collision[0] = End_collision[0];
				Start_collision[1] = End_collision[1];
				Start_collision[2] = End_collision[2];
				for (int index_Cc = 0; index_Cc < 3; index_Cc++)
				{
					index_level_0 = RoadCollision_Level_RD_out(Start_collision, End_collision, road_Cc[index_Cc * 2], road_Cc[index_Cc * 2 + 1], point_collision, 1);
					if (index_level_0 == 0)
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
				if (index_level_0 == 0)
				{
					indexCSC_50 = indexCSC_50 + 1;
					if (indexCSC_50 >= 2)
					{
						road_roll[0] = road_Cc[0];
						road_roll[1] = road_Cc[1];
						road_roll[2] = road_Cc[2];
						road_roll[3] = road_Cc[3];
						road_roll[4] = road_Cc[4];
						road_roll[5] = road_Cc[5];
						//printf("index_CSC50=%d\n",index_CSC50);
						return 1; // 前方预留30cm有解
					}
				}
			}
		}
		End_collision[0] = Start_collision_mid1[0];
		End_collision[1] = Start_collision_mid1[1];
		End_collision[2] = Start_collision_mid1[2];
		index_level_0 = CCS_road(End_collision, End, road_Cc);
		//printf("index_level_0=%d\n",index_level_0);
		if (index_level_0 == 1)
		{
			Start_collision[0] = End_collision[0];
			Start_collision[1] = End_collision[1];
			Start_collision[2] = End_collision[2];
			for (int index_Cc = 0; index_Cc < 3; index_Cc++)
			{
				index_level_0 = RoadCollision_Level_RD_out(Start_collision, End_collision, road_Cc[index_Cc * 2], road_Cc[index_Cc * 2 + 1], point_collision, 1);
				if (index_level_0 == 0)
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
			//printf("index_level_0=%d\n", index_level_0);
			if (index_level_0 == 0)
			{
				road_roll[0] = road_Cc[0];
				road_roll[1] = road_Cc[1];
				road_roll[2] = road_Cc[2];
				road_roll[3] = road_Cc[3];
				road_roll[4] = road_Cc[4];
				road_roll[5] = road_Cc[5];

				return 1; // 前方预留30cm有解
			}
		}
		///////////////////////////////////////////////////////////////////
		Start_collision[0] = Start_collision_mid[0];
		Start_collision[1] = Start_collision_mid[1];
		Start_collision[2] = Start_collision_mid[2];
		for (int index_Roll5 = 0; index_Roll5 < 5; index_Roll5++)
		{
			int index_level_1 = RoadCollision_Level_RD_out(Start_collision, End_collision, r_Cc * right_left, D_C, point_collision, 0); // 前进会不会碰撞
			Start_collision[0] = End_collision[0];
			Start_collision[1] = End_collision[1];
			Start_collision[2] = End_collision[2];
			Croll_DR[index_Roll5 * 4 + 0] = r_Cc * right_left;
			if (index_level_1 == 1)
			{
				Croll_DR[index_Roll5 * 4 + 1] = point_collision * 0.1;
			}
			else
			{
				Croll_DR[index_Roll5 * 4 + 1] = D_C;
			}
			int i = Croll_DR[index_Roll5 * 4 + 1] * 10;
			for (int index = 1; index < i + 1; index++)// 25
			{
				int index_level_2 = RoadCollision_Level_RD_out(Start_collision, End_collision, r_Cc * -right_left, -0.1 * index, point_collision, 0); // 后退会不会碰撞
				Croll_DR[index_Roll5 * 4 + 2] = r_Cc * -right_left;
				if (index_level_2 == 1)
				{
					Croll_DR[index_Roll5 * 4 + 3] = -point_collision * 0.1;
				}
				else
				{
					Croll_DR[index_Roll5 * 4 + 3] = -0.1 * index;
				}
				if (index_level_2 == 0)
				{
					int index_level_3 = CSC_road(End_collision, End, road_Cc_);
					if (index_level_3 != 0)
					{
						double Start_collision_copy[3] = { 0 };
						Start_collision_copy[0] = End_collision[0];
						Start_collision_copy[1] = End_collision[1];
						Start_collision_copy[2] = End_collision[2];
						double End_collision_copy[3] = { 0 };
						for (int index_Cc = 0; index_Cc < 3; index_Cc++)
						{
							index_level_3 = RoadCollision_Level_RD_out(Start_collision_copy, End_collision_copy, road_Cc_[index_Cc * 2], road_Cc_[index_Cc * 2 + 1], point_collision, 1);
							//printf("index_level_3=%d\n",index_level_3);
							//printf("Start_collision_copy=%lf,%lf,%lf\n",Start_collision_copy[0],Start_collision_copy[1],Start_collision_copy[2]);
							//printf("End_collision_copy=%lf,%lf,%lf\n",End_collision_copy[0],End_collision_copy[1],End_collision_copy[2]);
							if (index_level_3 == 0)
							{
								Start_collision_copy[0] = End_collision_copy[0];
								Start_collision_copy[1] = End_collision_copy[1];
								Start_collision_copy[2] = End_collision_copy[2];
							}
							else
							{
								break;
							}
						}
						if (index_level_3 == 0)
						{
							//safety_D = safety_D + 1; // 满2就是前方预留20cm安全距离
							//if (safety_D >= safety_D_max)
							//{
							road_roll[0] = R_level_1;
							road_roll[1] = D_level_1;
							road_roll[2] = Croll_DR[0];
							road_roll[3] = Croll_DR[1];
							road_roll[4] = Croll_DR[2];
							road_roll[5] = Croll_DR[3];
							road_roll[6] = Croll_DR[4];
							road_roll[7] = Croll_DR[5];
							road_roll[8] = Croll_DR[6];
							road_roll[9] = Croll_DR[7];
							road_roll[10] = Croll_DR[8];
							road_roll[11] = Croll_DR[9];
							road_roll[12] = Croll_DR[10];
							road_roll[13] = Croll_DR[11];
							road_roll[14] = Croll_DR[12];
							road_roll[15] = Croll_DR[13];
							road_roll[16] = Croll_DR[14];
							road_roll[17] = Croll_DR[15];
							road_roll[18] = Croll_DR[16];
							road_roll[19] = Croll_DR[17];
							road_roll[20] = Croll_DR[18];
							road_roll[21] = Croll_DR[19];
							road_roll[22] = road_Cc_[0];
							road_roll[23] = road_Cc_[1];
							road_roll[24] = road_Cc_[2];
							road_roll[25] = road_Cc_[3];
							road_roll[26] = road_Cc_[4];
							road_roll[27] = road_Cc_[5];

							return 1; // 前方预留30cm有解
						//}
						}
					}
				}
				else
				{
					Start_collision[0] = End_collision[0];
					Start_collision[1] = End_collision[1];
					Start_collision[2] = End_collision[2];

					break;
				}
			}
			Start_collision[0] = End_collision[0];
			Start_collision[1] = End_collision[1];
			Start_collision[2] = End_collision[2];
		}

		return 0;
	}

	int getroadpath_Level_back_out(double Start_collision_mid1[3], double End1[3], int right_left, double R_level_1, double D_level_1, double road_roll[28])
	{
		double r_Cc = rmin_level;
		double End[3];
		End[0] = End1[0];
		End[1] = End1[1];
		End[2] = End1[2];
		double End_collision[3];
		double Start_collision_mid[3] = { 0,0,0 };
		Start_collision_mid[0] = Start_collision_mid1[0];
		Start_collision_mid[1] = Start_collision_mid1[1];
		Start_collision_mid[2] = Start_collision_mid1[2];
		double Start_collision[3] = { 0,0,0 }; // 计算碰撞的起点坐标
		int point_collision = 100;
		double D_C = 1.0;
		double Croll_DR[20] = { 0 };
		double road_Cc_[6] = { 0,0,0,0,0,0 };
		// int safety_D = 0;
		// int safety_D_max = 2;
		///////////////////////////////////////////////////////////////////
		double road_Cc[6] = { 0 };
		End_collision[0] = Start_collision_mid1[0];
		End_collision[1] = Start_collision_mid1[1];
		End_collision[2] = Start_collision_mid1[2];
		int index_level_0 = CSC_road(End_collision, End, road_Cc);
		//printf("index_level_0=%d\n",index_level_0);
		if (index_level_0 != 0)
		{
			Start_collision[0] = End_collision[0];
			Start_collision[1] = End_collision[1];
			Start_collision[2] = End_collision[2];
			for (int index_Cc = 0; index_Cc < 3; index_Cc++)
			{
				index_level_0 = RoadCollision_Level_RD_out(Start_collision, End_collision, road_Cc[index_Cc * 2], road_Cc[index_Cc * 2 + 1], point_collision, 1);
				if (index_level_0 == 0)
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
			//printf("index_level_0=%d\n",index_level_0);
			if (index_level_0 == 0)
			{
				index_30 = index_30 + 1;
			}
			if (index_30 >= 2)
			{
				road_roll[0] = road_Cc[0];
				road_roll[1] = road_Cc[1];
				road_roll[2] = road_Cc[2];
				road_roll[3] = road_Cc[3];
				road_roll[4] = road_Cc[4];
				road_roll[5] = road_Cc[5];

				return 1; // 前方预留30cm有解
			}
		}
		///////////////////////////////////////////////////////////////////
		Start_collision[0] = Start_collision_mid[0];
		Start_collision[1] = Start_collision_mid[1];
		Start_collision[2] = Start_collision_mid[2];
		for (int index_Roll5 = 0; index_Roll5 < 5; index_Roll5++)
		{
			for (int index = 3; index < 11; index++)
			{
				int index_level_2 = RoadCollision_Level_RD_out(Start_collision, End_collision, r_Cc * -right_left, -0.1 * index, point_collision, 10); // 后退会不会碰撞 // 修改参数0为10
				Croll_DR[index_Roll5 * 4 + 0] = r_Cc * -right_left;
				if (index_level_2 == 1)
				{
					Croll_DR[index_Roll5 * 4 + 1] = -point_collision * 0.1;
				}
				else
				{
					Croll_DR[index_Roll5 * 4 + 1] = -0.1 * index;
				}
				//D_C = fabs(Croll_DR[index_Roll5 * 4 + 3])+0.1;
				//printf("-Croll_DR=%lf\n",Croll_DR[index_Roll5 * 4 + 1]);
				if (index_level_2 == 0)
				{
					//////////////////////////////////////////////////////////////////////////////////////
					double Start_collision_mid_[3] = { 0,0,0 };
					Start_collision_mid_[0] = End_collision[0];
					Start_collision_mid_[1] = End_collision[1];
					Start_collision_mid_[2] = End_collision[2];
					VehicleDynamic(Start_collision_mid_[0], Start_collision_mid_[1], Start_collision_mid_[2], 0.3, atan(vehicle_parameters.WB / (r_Cc * right_left))); // 根据当前位姿和输入, 计算下一位置的位姿
					End_collision[0] = g_px;
					End_collision[1] = g_py;
					End_collision[2] = g_pth;
					///////////////////////////////////////////////////////////////////////////////////////
					int index_level_3 = CCS_road(End_collision, End, road_Cc_);
					//printf("index_level_3 = %d\n",index_level_3);
					if (index_level_3 == 1)
					{
						double Start_collision_copy[3] = { 0 };
						Start_collision_copy[0] = End_collision[0];
						Start_collision_copy[1] = End_collision[1];
						Start_collision_copy[2] = End_collision[2];
						double End_collision_copy[3] = { 0 };
						for (int index_Cc = 0; index_Cc < 3; index_Cc++)
						{
							index_level_3 = RoadCollision_Level_RD_out(Start_collision_copy, End_collision_copy, road_Cc_[index_Cc * 2], road_Cc_[index_Cc * 2 + 1], point_collision, 1);
							if (index_level_3 == 0)
							{
								Start_collision_copy[0] = End_collision_copy[0];
								Start_collision_copy[1] = End_collision_copy[1];
								Start_collision_copy[2] = End_collision_copy[2];
							}
							else
							{
								break;
							}
						}
						if (index_level_3 == 0)
						{
							//safety_D = safety_D + 1; // 满3就是前方预留30cm安全距离
							//if (safety_D >= safety_D_max + 1)
							//{
							road_roll[0] = R_level_1;
							road_roll[1] = D_level_1;
							road_roll[2] = Croll_DR[0];
							road_roll[3] = Croll_DR[1];
							road_roll[4] = Croll_DR[2];
							road_roll[5] = Croll_DR[3];
							road_roll[6] = Croll_DR[4];
							road_roll[7] = Croll_DR[5];
							road_roll[8] = Croll_DR[6];
							road_roll[9] = Croll_DR[7];
							road_roll[10] = Croll_DR[8];
							road_roll[11] = Croll_DR[9];
							road_roll[12] = Croll_DR[10];
							road_roll[13] = Croll_DR[11];
							road_roll[14] = Croll_DR[12];
							road_roll[15] = Croll_DR[13];
							road_roll[16] = Croll_DR[14];
							road_roll[17] = Croll_DR[15];
							road_roll[18] = Croll_DR[16];
							road_roll[19] = Croll_DR[17];
							road_roll[20] = Croll_DR[18];
							road_roll[21] = Croll_DR[19];
							road_roll[22] = road_Cc_[0];
							road_roll[23] = road_Cc_[1] + 0.3;
							road_roll[24] = road_Cc_[2];
							road_roll[25] = road_Cc_[3];
							road_roll[26] = road_Cc_[4];
							road_roll[27] = road_Cc_[5];

							return 1; // 前方预留30cm有解
						//}
						}
					}
				}
				else
				{
					//Start_collision[0] = End_collision[0];
					//Start_collision[1] = End_collision[1];
					//Start_collision[2] = End_collision[2];

					break;
				}
			}
			Start_collision[0] = End_collision[0];
			Start_collision[1] = End_collision[1];
			Start_collision[2] = End_collision[2];
			////////////////////////////////////////////////////////////////
			double Start_collision_mid_[3] = { 0,0,0 };
			Start_collision_mid_[0] = Start_collision[0];
			Start_collision_mid_[1] = Start_collision[1];
			Start_collision_mid_[2] = Start_collision[2];
			VehicleDynamic(Start_collision_mid_[0], Start_collision_mid_[1], Start_collision_mid_[2], 0.3, atan(vehicle_parameters.WB / (r_Cc * right_left))); // 根据当前位姿和输入, 计算下一位置的位姿
			Start_collision_mid_[0] = g_px;
			Start_collision_mid_[1] = g_py;
			Start_collision_mid_[2] = g_pth;
			int index_level_1_ = RoadCollision_Level_RD_out(Start_collision_mid_, End_collision, r_Cc * right_left, D_C, point_collision, 1);
			if (index_level_1_ == 1)
			{
				D_C = (point_collision + 5) * 0.1;
			}
			else
			{
				D_C = D_C + 0.5;
			}
			////////////////////////////////////////////////////////////////
			int index_level_1 = RoadCollision_Level_RD_out(Start_collision, End_collision, r_Cc * right_left, D_C, point_collision, 0); // 前进会不会碰撞
			//printf("+Start_collision=%lf,%lf,%lf\n",Start_collision[0],Start_collision[1],Start_collision[2]);
			//printf("+End_collision=%lf,%lf,%lf\n",End_collision[0],End_collision[1],End_collision[2]);
			Start_collision[0] = End_collision[0];
			Start_collision[1] = End_collision[1];
			Start_collision[2] = End_collision[2];
			Croll_DR[index_Roll5 * 4 + 2] = r_Cc * right_left;
			if (index_level_1 == 1)
			{
				Croll_DR[index_Roll5 * 4 + 3] = point_collision * 0.1;
			}
			else
			{
				Croll_DR[index_Roll5 * 4 + 3] = D_C;
			}
		}
		road_roll[0] = R_level_1;
		road_roll[1] = D_level_1;
		road_roll[2] = Croll_DR[0];
		road_roll[3] = Croll_DR[1];
		road_roll[4] = Croll_DR[2];
		road_roll[5] = Croll_DR[3];
		road_roll[6] = Croll_DR[4];
		road_roll[7] = Croll_DR[5];
		road_roll[8] = Croll_DR[6];
		road_roll[9] = Croll_DR[7];
		road_roll[10] = Croll_DR[8];
		road_roll[11] = Croll_DR[9];
		road_roll[12] = Croll_DR[10];
		road_roll[13] = Croll_DR[11];
		road_roll[14] = Croll_DR[12];
		road_roll[15] = Croll_DR[13];
		road_roll[16] = Croll_DR[14];
		road_roll[17] = Croll_DR[15];
		road_roll[18] = Croll_DR[16];
		road_roll[19] = Croll_DR[17];
		road_roll[20] = Croll_DR[18];
		road_roll[21] = Croll_DR[19];
		road_roll[22] = r_Cc * right_left;
		road_roll[23] = 0.3;
		road_roll[24] = 0;
		road_roll[25] = 0;
		road_roll[26] = 0;
		road_roll[27] = 0;
		return 0;
	}

	int Roll_out(double Start_collision_mid1[3], double End1[3], int right_left, double R_level_1, double D_level_1, double road_roll[28])
	{
		if ((Gears != 1) || (control.ObsUssInfo == 6u) || (control.ObsUssInfo == 7u) || (control.ObsUssInfo == 8u) || (control.ObsUssInfo == 9u))
		{
			int a = getroadpath_Level_front_out(Start_collision_mid1, End1, right_left, R_level_1, D_level_1, road_roll);
			if (a == 1)
			{
				return 1;
			}
		}
		else if ((Gears == 1) || (control.ObsUssInfo == 0u) || (control.ObsUssInfo == 1u) || (control.ObsUssInfo == 2u) || (control.ObsUssInfo == 3u))
		{
			int a = getroadpath_Level_back_out(Start_collision_mid1, End1, right_left, R_level_1, D_level_1, road_roll);
			if (a == 1)
			{
				return 1;
			}
		}

		return 0;
	}

	int roll_level_park_out(double Start1[3], double End1[3], int right_left, double road_roll[28])
	{
		double End[3];
		double Start[3];
		Start[0] = Start1[0];
		Start[1] = Start1[1];
		Start[2] = Start1[2];
		End[0] = End1[0];
		End[1] = End1[1];
		End[2] = End1[2];

		double R_level_1 = 1000000;			   // 24.4.3 修改000000为 1000000
		double Start_collision[3] = {0, 0, 0}; // 计算碰撞的起点坐标
		double End_collision[3] = {0, 0, 0};
		int point_collision = 100;
		double road_Cc[6] = { 0,0,0,0,0,0 };
		int safety_D = 0;
		int safety_D_max = 2;
		double D_level_1 = -2.5;

		///////////////////////////直线后退有解或无解最多退多少/////////////////////////////////////////////
		for (int index = 0; index < 25; index++)
		{
			int index_level_1 = RoadCollision_Level_RD_out(Start, End_collision, R_level_1, -0.1 * index, point_collision, 0); // 直线往后退
			int index_level_2 = 0;
			if (index_level_1 == 0) // 直线段不碰撞
			{
				index_level_2 = CSC_road(End_collision, End, road_Cc);
				if (index_level_2 != 0)
				{
					Start_collision[0] = End_collision[0];
					Start_collision[1] = End_collision[1];
					Start_collision[2] = End_collision[2];
					for (int index_Cc = 0; index_Cc < 3; index_Cc++)
					{
						double End_collision_copy[3] = { 0 };
						index_level_2 = RoadCollision_Level_RD_out(Start_collision, End_collision_copy, road_Cc[index_Cc * 2], road_Cc[index_Cc * 2 + 1], point_collision, 1);
						if (index_level_2 == 0)
						{
							Start_collision[0] = End_collision_copy[0];
							Start_collision[1] = End_collision_copy[1];
							Start_collision[2] = End_collision_copy[2];
						}
						else
						{
							break;
						}
					}
					if (index_level_2 == 0)
					{
						safety_D = safety_D + 1; // 满3就是前方预留30cm安全距离
						if (safety_D >= safety_D_max)
						{
							road_roll[0] = R_level_1;
							road_roll[1] = -0.1 * index - 0.7;
							road_roll[2] = road_Cc[0];
							road_roll[3] = road_Cc[1];
							road_roll[4] = road_Cc[2];
							road_roll[5] = road_Cc[3];
							road_roll[6] = road_Cc[4];
							road_roll[7] = road_Cc[5];


							return 1; // 前方预留30cm有解
						}
					}
				}
			}
			else
			{
				if (index > 1)
				{
					D_level_1 = -0.1 * (index - 1);
					break;
				}
			}
		}

		double Start_collision_mid[3] = { End_collision[0],End_collision[1],End_collision[2] };
		///////////////////////////揉库求解///////////////////////////////////////////// 
		int level_return = Roll_out(Start_collision_mid, End, right_left, R_level_1, D_level_1, road_roll);
		if (level_return == 1)
		{
			return 1;
		}

		return 0;
	}

	int  ParkingOutLevel(double Start1[3], double End1[3])
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

		double road_roll[28] = { 0 };

		if ((index_level_out == 0) && (fabs(Start[1] - End[1]) < 1.2) && (fabs(Start[2]) < 35 * pi / 180) && ((control.ObsUssInfo == 0u) || (control.ObsUssInfo == 1u) || (control.ObsUssInfo == 2u) || (control.ObsUssInfo == 3u) || ((control.ObsUssInfo == 4u) && (right_left == -1)) || ((control.ObsUssInfo == 11u) && (right_left == 1))))
		{
			index_level_out = 1;
		}
		if (index_level_out == 1)
		{
			if (fabs(Start[2]) * rmin_level > 0.3)
			{
				int index_level_0_ = 0;
				double Start_collision_[3] = { 0 };
				double End_collision_[3] = { 0 };
				int point_collision_ = 0;
				for (int D_out = 0; D_out < 3; D_out++)
				{
					Start_collision_[0] = Start[0];
					Start_collision_[1] = Start[1];
					Start_collision_[2] = Start[2];
					double D_out_ = D_out * 0.1 + 0.3;
					road_roll[0] = rmin_level * right_left;
					road_roll[1] = -D_out_;
					road_roll[2] = rmin_level * -right_left;
					road_roll[3] = fabs(Start[2]) * rmin_level - D_out_;
					for (int index_Cc = 0; index_Cc < 2; index_Cc++)
					{

						index_level_0_ = RoadCollision_Level_RD_out(Start_collision_, End_collision_, road_roll[index_Cc * 2], road_roll[index_Cc * 2 + 1], point_collision_, 2);
						if (index_level_0_ == 0)
						{
							Start_collision_[0] = End_collision_[0];
							Start_collision_[1] = End_collision_[1];
							Start_collision_[2] = End_collision_[2];
						}
						else
						{
							break;
						}
					}
					if (index_level_0_ == 0)
					{
						index_level_out = 2;
						getroadpath_Level_all_out(Start, road_roll, 0.5, 0);
						return 1;
					}
					else
					{
						road_roll[0] = 0;
						road_roll[1] = 0;
						road_roll[2] = 0;
						road_roll[3] = 0;
					}
				}
			}
			index_level_out = 0;
		}
		else if (index_level_out == 2)
		{
			int index_level_0_ = 0;
			double Start_collision_[3] = { 0 };
			double End_collision_[3] = { 0 };
			int point_collision_ = 0;
			Start_collision_[0] = Start[0];
			Start_collision_[1] = Start[1];
			Start_collision_[2] = Start[2];
			road_roll[0] = rmin_level * -right_left;
			road_roll[1] = fabs(Start[2]) * rmin_level;
			index_level_0_ = RoadCollision_Level_RD_out(Start_collision_, End_collision_, road_roll[0], road_roll[1], point_collision_, 2);
			if (index_level_0_ == 0)
			{
				getroadpath_Level_all_out(Start, road_roll, 0.5, 0);
				return 1;
			}
			else
			{
				road_roll[0] = 0;
				road_roll[1] = 0;
			}
		}

		if ((fabs(Start1[2]) > 40 * pi / 180) || (fabs(Start[1] - End[1]) < 2))
		{
			double road_Cc[6] = { 0 };
			int index_level_0 = CSC_road(Start, End, road_Cc);
			if (index_level_0 != 0)
			{
				road_roll[0] = road_Cc[0];
				road_roll[1] = road_Cc[1];
				road_roll[2] = road_Cc[2];
				road_roll[3] = road_Cc[3];
				road_roll[4] = road_Cc[4];
				road_roll[5] = road_Cc[5];
				getroadpath_Level_all_out(Start, road_roll, 0.5, 0);
				return 1;
			}
			return 0;
		}
		if ((index_request == 1) && (control.ObsUssInfo != 6u) && (control.ObsUssInfo != 7u) && (control.ObsUssInfo != 8u) && (control.ObsUssInfo != 9u))
		{
			int a = roll_level_park_out(Start, End, right_left, road_roll);
			if (a == 1)
			{
				getroadpath_Level_all_out(Start, road_roll, 0.5, 0);
				return 1;
			}
		}
		else
		{
			int a = Roll_out(Start, End, right_left, 0, 0, road_roll);
			if (a == 1)
			{
				getroadpath_Level_all_out(Start, road_roll, 0.5, 0);
				return 1;
			}
		}

		return 0;
	}
}

