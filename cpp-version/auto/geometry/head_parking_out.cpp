#include "../common/global_variable.h"
#include "../common/global_function.h"

namespace byd_apa_plan
{
	bool VehicleCollisionGridHeadingOut(double cpx, double cpy, double cph)
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

	void getroadpath_heading_out(double Start[3], double road_roll[28])
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
			if (fabs(road_roll[index * 2]) > 100000 || fabs(road_roll[index * 2]) < 5.4)
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
	}

	int imagmap_heading_out_h(double Start[3], double End[3], int right_left, double &dist_head_out_, double road_roll[28])
	{
		if (right_left == 1)
		{
			int index_out = 0;
			for (int index_x = 0; index_x < 30; index_x++)
			{
				for (int index_y = 0; index_y < 40; index_y++)
				{
					double vir_x = park_out_head_left[index_x * 40 + index_y].first * cos(Start_out[2]) - park_out_head_left[index_x * 40 + index_y].second * sin(Start_out[2]) + Start_out[0];
					double vir_y = park_out_head_left[index_x * 40 + index_y].second * cos(Start_out[2]) + park_out_head_left[index_x * 40 + index_y].first * sin(Start_out[2]) + Start_out[1];
					int idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
					int idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
					if (idx_x == 0)
					{
						idx_x = 1;
					}
					if (idx_y == 0)
					{
						idx_y = 1;
					}
					int index_xy = (idx_x - 1) * 250 + (idx_y - 1);
					if ((index_xy >= 0) && (index_xy < 62500) && (idx_x >= 0) && (idx_x <= 250) && (idx_y >= 0) && (idx_y <= 250))
					{
						if (obstmap[index_xy].Status == 0)
						{
							index_out = 1;
							break;
						}
					}
				}
				if ((index_out == 1) || (index_x == 29))
				{
					if (dist_out_min > 2.2 + 0.01)
					{
						dist_head_out_ = dist_out_min + 2.9 - (index_x * 0.1);
						road_roll[0] = 0;
						road_roll[1] = dist_head_out_ - (Start[0] - Start_out[0]);
						road_roll[2] = rmin_vertical;
						road_roll[3] = rmin_vertical * fabs(End[2]);
						road_roll[4] = 0;
						road_roll[5] = 0.5;
					}
					else
					{
						dist_head_out_ = dist_out_min + 2.9 - (index_x * 0.1);
						road_roll[0] = 0;
						road_roll[1] = dist_head_out_ - (Start[0] - Start_out[0]);
						double D_out_diff = 2.8 * fabs(cos(End[2]))/2;
						double theta_o = acos(1-(1.5 + vehicle_parameters.W/2 + D_out_diff)/rmin_vertical); // 54
						if (fabs(End[2]) <= theta_o)
						{
							road_roll[2] = rmin_vertical;
							road_roll[3] = rmin_vertical * fabs(End[2]);
							road_roll[4] = 0;
							road_roll[5] = 0.5;
						}
						else
						{
							if (fabs(End[2]) <= pi * 32 / 180)
							{
								road_roll[2] = rmin_vertical;
								road_roll[3] = rmin_vertical * fabs(End[2]);
								road_roll[4] = 0;
								road_roll[5] = 0.5;
							}
							else if(fabs(End[2]) <= pi * 48 / 180)
							{
								road_roll[2] = rmin_vertical;
								road_roll[3] = rmin_vertical * pi * 32 / 180;
								road_roll[4] = -rmin_vertical;
								road_roll[5] = -rmin_vertical * (fabs(End[2]) - pi * 32 / 180);
								road_roll[6] = 0;
								road_roll[7] = 0.5;
							}
							else
							{
								road_roll[2] = rmin_vertical;
								road_roll[3] = rmin_vertical * pi * 32 / 180;
								road_roll[4] = -rmin_vertical;
								road_roll[5] = -rmin_vertical * pi * 16 / 180;
								road_roll[6] = rmin_vertical;
								road_roll[7] = rmin_vertical * (fabs(End[2]) - pi * (32 + 16) / 180);
								road_roll[8] = 0;
								road_roll[9] = 0.5;
							}
							
						}
					}
					
					if (road_roll[1] > 0)
					{
						return 1;
					}
					else
					{
						road_roll[1] = 0;
						return 1;
					}
				}
			}
		}
		else if(right_left == -1)
		{
			int index_out = 0;
			for (int index_x = 0; index_x < 30; index_x++)
			{
				for (int index_y = 0; index_y < 40; index_y++)
				{
					double vir_x = park_out_head_right[index_x * 40 + index_y].first * cos(Start_out[2]) - park_out_head_right[index_x * 40 + index_y].second * sin(Start_out[2]) + Start_out[0];
					double vir_y = park_out_head_right[index_x * 40 + index_y].second * cos(Start_out[2]) + park_out_head_right[index_x * 40 + index_y].first * sin(Start_out[2]) + Start_out[1];
					int idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
					int idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
					if (idx_x == 0)
					{
						idx_x = 1;
					}
					if (idx_y == 0)
					{
						idx_y = 1;
					}
					int index_xy = (idx_x - 1) * 250 + (idx_y - 1);
					if ((index_xy >= 0) && (index_xy < 62500) && (idx_x >= 0) && (idx_x <= 250) && (idx_y >= 0) && (idx_y <= 250))
					{
						if (obstmap[index_xy].Status == 0)
						{
							index_out = 1;
							break;
						}
					}

				}
				if ((index_out == 1) || (index_x == 29))
				{
					if (dist_out_min > 2.2 + 0.01)
					{
						dist_head_out_ = dist_out_min + 2.9 - (index_x * 0.1);
						road_roll[0] = 0;
						road_roll[1] = dist_head_out_ - (Start[0] - Start_out[0]);
						road_roll[2] = -rmin_vertical;
						road_roll[3] = rmin_vertical * fabs(End[2]);
						road_roll[4] = 0;
						road_roll[5] = 0.5;
					}
					else
					{
						dist_head_out_ = dist_out_min + 2.9 - (index_x * 0.1);
						road_roll[0] = 0;
						road_roll[1] = dist_head_out_ - (Start[0] - Start_out[0]);
						double D_out_diff = 2.8 * fabs(cos(End[2]))/2;
						double theta_o = acos(1-(1.5 + vehicle_parameters.W/2 + D_out_diff)/rmin_vertical); // 54
						if (fabs(End[2]) <= theta_o)
						{
							road_roll[2] = -rmin_vertical;
							road_roll[3] = rmin_vertical * fabs(End[2]);
							road_roll[4] = 0;
							road_roll[5] = 0.5;
						}
						else
						{
							if (fabs(End[2]) <= pi * 32 / 180)
							{
								road_roll[2] = -rmin_vertical;
								road_roll[3] = rmin_vertical * fabs(End[2]);
								road_roll[4] = 0;
								road_roll[5] = 0.5;
							}
							else if(fabs(End[2]) <= pi * 48 / 180)
							{
								road_roll[2] = -rmin_vertical;
								road_roll[3] = rmin_vertical * pi * 32 / 180;
								road_roll[4] = +rmin_vertical;
								road_roll[5] = -rmin_vertical * (fabs(End[2]) - pi * 32 / 180);
								road_roll[6] = 0;
								road_roll[7] = 0.5;
							}
							else
							{
								road_roll[2] = -rmin_vertical;
								road_roll[3] = rmin_vertical * pi * 32 / 180;
								road_roll[4] = rmin_vertical;
								road_roll[5] = -rmin_vertical * pi * 16 / 180;
								road_roll[6] = -rmin_vertical;
								road_roll[7] = rmin_vertical * (fabs(End[2]) - pi * (32 + 16) / 180);
								road_roll[8] = 0;
								road_roll[9] = 0.5;
							}
							
						}
					}
					if (road_roll[1] > 0)
					{
						return 1;
					}
					else
					{
						road_roll[1] = 0;
						return 1;
					}
				}
			}
		}
		else
		{
			int index_out = 0;
			for (int index_x = 0; index_x < 30; index_x++)
			{
				for (int index_y = 0; index_y < 40; index_y++)
				{
					double vir_x = park_out_head_left[index_x * 40 + index_y].first * cos(Start_out[2]) - park_out_head_left[index_x * 40 + index_y].second * sin(Start_out[2]) + Start_out[0];
					double vir_y = park_out_head_left[index_x * 40 + index_y].second * cos(Start_out[2]) + park_out_head_left[index_x * 40 + index_y].first * sin(Start_out[2]) + Start_out[1];
					double vir_x2 = park_out_head_right[index_x * 40 + index_y].first * cos(Start_out[2]) - park_out_head_right[index_x * 40 + index_y].second * sin(Start_out[2]) + Start_out[0];
					double vir_y2 = park_out_head_right[index_x * 40 + index_y].second * cos(Start_out[2]) + park_out_head_right[index_x * 40 + index_y].first * sin(Start_out[2]) + Start_out[1];
					int idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
					int idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
					int idx_x2 = ceil((pathfind_parameters.MAXX - vir_x2) / pathfind_parameters.MOTION_RESOLUTION);
					int idx_y2 = ceil((pathfind_parameters.MAXY - vir_y2) / pathfind_parameters.MOTION_RESOLUTION);
					if (idx_x == 0)
					{
						idx_x = 1;
					}
					if (idx_y == 0)
					{
						idx_y = 1;
					}
					if (idx_x2 == 0)
					{
						idx_x2 = 1;
					}
					if (idx_y2 == 0)
					{
						idx_y2 = 1;
					}
					int index_xy = (idx_x - 1) * 250 + (idx_y - 1);
					int index_xy2 = (idx_x2 - 1) * 250 + (idx_y2 - 1);
					if ((index_xy >= 0) && (index_xy < 62500) && (idx_x >= 0) && (idx_x <= 250) && (idx_y >= 0) && (idx_y <= 250))
					{
						if (obstmap[index_xy].Status == 0)
						{
							index_out = 1;
							break;
						}
					}
					if ((index_xy2 >= 0) && (index_xy2 < 62500) && (idx_x2 >= 0) && (idx_x2 <= 250) && (idx_y2 >= 0) && (idx_y2 <= 250))
					{
						if (obstmap[index_xy2].Status == 0)
						{
							index_out = 1;
							break;
						}
					}
				}
				if ((index_out == 1) || (index_x == 29))
				{
					if (index_x != 0)
					{
						dist_head_out_ = (dist_out_min>(2.2 + 2.9 - (index_x * 0.1)))?dist_out_min:(2.2 + 2.9 - (index_x * 0.1));
						dist_head_out_ -= 1;
						road_roll[0] = 0;
						road_roll[1] = dist_head_out_ - (Start[0] - Start_out[0]);
						if (road_roll[1] > 0)
						{
							return 1;
						}
						else
						{
							return 0;
						}
					}
					else
					{
						return 0;
					}
				}
			}
		}

		return 0;
	}

	int getroadpath_heading_front_out(double Start[3], double End[3], int right_left, double road_roll[28])
	{
		if (right_left == 1)
		{
			double D_all = rmin_vertical * fabs(End[2] - Start[2]);
			road_roll[0] = rmin_vertical;
			road_roll[1] = D_all;
			road_roll[2] = 0;
			road_roll[3] = 0.5;
			return 1;
		}
		else if (right_left == -1)
		{
			double D_all = rmin_vertical * fabs(End[2] - Start[2]);
			road_roll[0] = -rmin_vertical;
			road_roll[1] = D_all;
			road_roll[2] = 0;
			road_roll[3] = 0.5;
			return 1;
		}

		return 0;
	}

	int getroadpath_heading_back_out(double Start[3], double End[3], int right_left, double road_roll[28])
	{
		double point_left[2] = { 0,0.97 }; // 后轴中心左右扩大20cm的坐标
		double point_right[2] = { 0,-0.97 };
		double x = Start[0];
		double y = Start[1];
		double th = mod2pi(Start[2]);
		double D_ = 0;
		double delta_ = 0;
		int roadisCollision = 0;
		double D_oh = 0;
		D_oh = Start_out[0] + dist_head_out;
		if (right_left == 1)
		{
			double D_all = rmin_vertical * fabs(End[2] - Start[2]);
			int i = fabs(D_all) / pathfind_parameters.MOTION_RESOLUTION;
			D_ = -0.1;
			delta_ = -atan(vehicle_parameters.WB / rmin_vertical);

			if ((control.ObsUssInfo == 0u) || (control.ObsUssInfo == 1u) || (control.ObsUssInfo == 2u) || (control.ObsUssInfo == 3u))
			{
				int control_ob = control.ObsUssInfo;
				double Uss_x = upa_x[control_ob];
				double Uss_y = upa_y[control_ob];
				double vir_x;
				//double vir_y;
				vir_x = Uss_x * cos(Start[2]) - Uss_y * sin(Start[2]) + Start[0];
				//vir_y = Uss_y * cos(Start[2]) + Uss_x * sin(Start[2]) + Start[1];
				if ((vir_x - D_oh) < 4.6)
				{
					road_roll[0] = rmin_vertical;
					road_roll[1] = D_all;
					road_roll[2] = 0;
					road_roll[3] = 0.5;
					Gears = 2;
					return 1;
				}
			}
			
			for (int index = 0; index<i; index++)
			{
				VehicleDynamic(x, y, th, D_, delta_); // 根据当前位姿和输入, 计算下一位置的位姿
				roadisCollision = VehicleCollisionGridHeadingOut(g_px, g_py, g_pth);
				double point_left_x = point_left[0] * cos(g_pth) - point_left[1] * sin(g_pth) + g_px;
				//double point_left_y = point_left[1] * cos(g_pth) + point_left[0] * sin(g_pth) + g_py;
				if ((roadisCollision) || (point_left_x < D_oh - 0.2))
				{
					road_roll[0] = -rmin_vertical;
					road_roll[1] = -index * 0.1;
					road_roll[2] = rmin_vertical;
					road_roll[3] = D_all - index * 0.1;
					road_roll[4] = 0;
					road_roll[5] = 0.5;
					if (fabs(road_roll[1]) < 0.29)
					{
						return 0;
					}
					return 1;
				}
				else
				{
					x = g_px;
					y = g_py;
					th = g_pth;
				}
			}
			road_roll[0] = -rmin_vertical;
			road_roll[1] = -D_all * 0.1;
			road_roll[2] = 0;
			road_roll[3] = -0.5;
			return 1;
		}
		else if (right_left == -1)
		{
			double D_all = rmin_vertical * fabs(End[2] - Start[2]);
			int i = fabs(D_all) / pathfind_parameters.MOTION_RESOLUTION;
			D_ = -0.1;
			delta_ = atan(vehicle_parameters.WB / rmin_vertical);

			if ((control.ObsUssInfo == 0u) || (control.ObsUssInfo == 1u) || (control.ObsUssInfo == 2u) || (control.ObsUssInfo == 3u))
			{
				int control_ob = control.ObsUssInfo;
				double Uss_x = upa_x[control_ob];
				double Uss_y = upa_y[control_ob];
				double vir_x;
				//double vir_y;
				vir_x = Uss_x * cos(Start[2]) - Uss_y * sin(Start[2]) + Start[0];
				//vir_y = Uss_y * cos(Start[2]) + Uss_x * sin(Start[2]) + Start[1];
				if ((vir_x - D_oh) < 4.6)
				{
					road_roll[0] = -rmin_vertical;
					road_roll[1] = D_all;
					road_roll[2] = 0;
					road_roll[3] = 0.5;
					Gears = 2;
					return 1;
				}
			}

			for (int index = 0; index < i; index++)
			{
				VehicleDynamic(x, y, th, D_, delta_); // 根据当前位姿和输入, 计算下一位置的位姿
				roadisCollision = VehicleCollisionGridHeadingOut(g_px, g_py, g_pth);
				double point_right_x = point_right[0] * cos(g_pth) - point_right[1] * sin(g_pth) + g_px;
				//double point_right_y = point_right[1] * cos(g_pth) + point_right[0] * sin(g_pth) + g_py;
				if ((roadisCollision) || (point_right_x < D_oh - 0.2))
				{
					road_roll[0] = rmin_vertical;
					road_roll[1] = -index * 0.1;
					road_roll[2] = -rmin_vertical;
					road_roll[3] = D_all - index * 0.1;
					road_roll[4] = 0;
					road_roll[5] = 0.5;
					if (fabs(road_roll[1]) < 0.29)
					{
						return 0;
					}
					return 1;
				}
				else
				{
					x = g_px;
					y = g_py;
					th = g_pth;
				}
			}
			road_roll[0] = rmin_vertical;
			road_roll[1] = -D_all * 0.1;
			road_roll[2] = 0;
			road_roll[3] = -0.5;
			return 1;
		}

		return 0;
	}

	int Roll_heading_out(double Start_collision_mid1[3], double End[3], int right_left, double road_roll[28]) // 后退没有暂停,前进有暂停
	{
		if (Gears == 2)
		{
			int a = getroadpath_heading_front_out(Start_collision_mid1, End, right_left, road_roll);
			if (a == 1)
			{
				return 1;
			}
		}
		else if (Gears == 1)
		{
			int a = getroadpath_heading_back_out(Start_collision_mid1, End, right_left, road_roll);
			if (a == 1)
			{
				return 1;
			}
		}

		return 0;
	}

	bool  HeadParkingOut(double* start, double* end)
	{
		pathpoint.clear();
		double Start[3];
		double End[3];

		Start[0] = start[0];
		Start[1] = start[1];
		Start[2] = start[2];
		End[0] = end[0];
		End[1] = end[1];
		End[2] = end[2];

		double road_roll[28] = { 0 };

		if (((fabs(Start[2]) < 15 * pi / 180) || ((fabs(Start[2]) < pi / 6) && (control.ObsUssInfo == 255u))) && (flag_out_park == 1)) // 第一步被其他因素逼停但还能走或对象空间太小
		{
			flag_out_park = 0;
			if ((Start[0] - Start_out[0]) < dist_head_out - 0.1)
			{

			}
			else
			{
				if (right_left == 1)
				{
					if (dist_out_min > 2.2 + 0.01)
					{
						road_roll[0] = rmin_vertical;
						road_roll[1] = rmin_vertical * fabs(End[2] - Start[2]);
						road_roll[2] = 0;
						road_roll[3] = 0.5;
					}
					else
					{
						double D_out_diff = 2.8 * fabs(cos(End[2]))/2;
						double theta_o = acos(1-(1.5 + vehicle_parameters.W/2 + D_out_diff)/rmin_vertical); // 54
						if (fabs(End[2]) <= theta_o)
						{
							road_roll[0] = rmin_vertical;
							road_roll[1] = rmin_vertical * fabs(End[2] - Start[2]);
							road_roll[2] = 0;
							road_roll[3] = 0.5;
						}
						else
						{
							if (fabs(End[2]) <= pi * 32 / 180)
							{
								road_roll[0] = rmin_vertical;
								road_roll[1] = rmin_vertical * fabs(End[2] - Start[2]);
								road_roll[2] = 0;
								road_roll[3] = 0.5;
							}
							else if(fabs(End[2]) <= pi * 48 / 180)
							{
								road_roll[0] = rmin_vertical;
								road_roll[1] = rmin_vertical * (pi * 32 / 180 - fabs(Start[2]));
								road_roll[2] = -rmin_vertical;
								road_roll[3] = -rmin_vertical * (fabs(End[2]) - pi * 32 / 180);
								road_roll[4] = 0;
								road_roll[5] = 0.5;
							}
							else
							{
								road_roll[0] = rmin_vertical;
								road_roll[1] = rmin_vertical * (pi * 32 / 180 - fabs(Start[2]));
								road_roll[2] = -rmin_vertical;
								road_roll[3] = -rmin_vertical * pi * 16 / 180;
								road_roll[4] = rmin_vertical;
								road_roll[5] = rmin_vertical * (fabs(End[2]) - pi * (32 + 16) / 180);
								road_roll[6] = 0;
								road_roll[7] = 0.5;
							}
							
						}
					}
					getroadpath_heading_out(Start, road_roll);
					flag_out_park = 1;
					return 1;
				}
				else if (right_left == -1)
				{
					if (dist_out_min > 2.2 + 0.01)
					{
						road_roll[0] = -rmin_vertical;
						road_roll[1] = rmin_vertical * fabs(End[2] - Start[2]);
						road_roll[2] = 0;
						road_roll[3] = 0.5;
					}
					else
					{
						double D_out_diff = 2.8 * fabs(cos(End[2]))/2;
						double theta_o = acos(1-(1.5 + vehicle_parameters.W/2 + D_out_diff)/rmin_vertical); // 54
						if (fabs(End[2]) <= theta_o)
						{
							road_roll[0] = -rmin_vertical;
							road_roll[1] = rmin_vertical * fabs(End[2] - Start[2]);
							road_roll[2] = 0;
							road_roll[3] = 0.5;
						}
						else
						{
							if (fabs(End[2]) <= pi * 32 / 180)
							{
								road_roll[0] = -rmin_vertical;
								road_roll[1] = rmin_vertical * fabs(End[2] - Start[2]);
								road_roll[2] = 0;
								road_roll[3] = 0.5;
							}
							else if(fabs(End[2]) <= pi * 48 / 180)
							{
								road_roll[0] = -rmin_vertical;
								road_roll[1] = rmin_vertical * (pi * 32 / 180 - fabs(Start[2]));
								road_roll[2] = rmin_vertical;
								road_roll[3] = -rmin_vertical * (fabs(End[2]) - pi * 32 / 180);
								road_roll[4] = 0;
								road_roll[5] = 0.5;
							}
							else
							{
								road_roll[0] = -rmin_vertical;
								road_roll[1] = rmin_vertical * (pi * 32 / 180 - fabs(Start[2]));
								road_roll[2] = rmin_vertical;
								road_roll[3] = -rmin_vertical * pi * 16 / 180;
								road_roll[4] = -rmin_vertical;
								road_roll[5] = rmin_vertical * (fabs(End[2]) - pi * (32 + 16) / 180);
								road_roll[6] = 0;
								road_roll[7] = 0.5;
							}
							
						}
					}
					getroadpath_heading_out(Start, road_roll);
					flag_out_park = 1;
					return 1;
				}
			}
		}

		if (flag_out_park == 0)
		{
			int a = imagmap_heading_out_h(Start, End, right_left, dist_head_out, road_roll);
			if (a == 1)
			{
				flag_out_park = 1;
				getroadpath_heading_out(Start, road_roll);
				return 1;
			}
		}
		else
		{
			dist_head_out = dist_head_out + 1.2;
			int a = Roll_heading_out(Start, End, right_left, road_roll);
			if (a == 1)
			{
				dist_head_out = dist_head_out - 1.2;
				flag_out_park = flag_out_park + 1;
				getroadpath_heading_out(Start, road_roll);
				return 1;
			}
		}

		return 0;
	}
}