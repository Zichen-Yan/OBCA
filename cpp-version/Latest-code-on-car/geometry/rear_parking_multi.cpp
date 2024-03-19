#include "../common/global_variable.h"
#include "../common/global_function.h"

namespace byd_apa_plan
{
	int GetOrCheckPathPointsMulti(const double start[3], const double road_yaw[6], const int &collision_check_flag)
	{
		path_point mid_L;
		mid_L.x = start[0];
		mid_L.y = start[1];
		mid_L.th = mod2pi(start[2]);
		mid_L.D = 0;
		mid_L.delta = 0;
		//pathpoint.push_back(mid_L);
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
		if (fabs(road_yaw[0]) > 100000 || fabs(road_yaw[0]) < rmin_level - 0.1)
		{
			delta_L = 0;
		}
		else
		{
			delta_L = atan(vehicle_parameters.WB / road_yaw[0]);
		}
		if (fabs(road_yaw[2]) > 100000 || fabs(road_yaw[2]) < rmin_level - 0.1)
		{
			delta_r = 0;
		}
		else
		{
			delta_r = atan(vehicle_parameters.WB / road_yaw[2]);
		}
		if (fabs(road_yaw[4]) > 100000 || fabs(road_yaw[4]) < rmin_level - 0.1)
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
			//碰撞检测
			if (collision_check_flag == 1)
			{
				if (VehicleCollisionGrid(g_px, g_py, g_pth))
				{
					if ((fusion.ParkInMode == 0) && (ParkingSpaceFlag * pathpoint.back().y < -0.2 && delta_L != 0 && D_L == -0.1))
					{
						return 0;
					}
					return 1; // 
				}
				pathpoint.push_back(mid_L);
			}
			else {
				pathpoint.push_back(mid_L);
			}
			//
		}
		for (int idx_r = 0; idx_r < round(fabs(road_yaw[3]) / pathfind_parameters.MOTION_RESOLUTION); idx_r++) // round()四舍五入
		{
			VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D_r, delta_r); // 根据当前位姿和输入, 计算下一位置的位姿
			mid_L.x = g_px;
			mid_L.y = g_py;
			mid_L.th = g_pth;
			mid_L.D = D_r;
			mid_L.delta = delta_r;
			//碰撞检测
			if (collision_check_flag == 1)
			{
				if (VehicleCollisionGrid(g_px, g_py, g_pth))
				{
					return 2;
				}
				pathpoint.push_back(mid_L);
			}
			else {
				pathpoint.push_back(mid_L);
			}
			//
		}
		for (int idx_s = 0; idx_s < round(fabs(road_yaw[5]) / pathfind_parameters.MOTION_RESOLUTION); idx_s++) // round()四舍五入
		{
			VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D_s, delta_s); // 根据当前位姿和输入, 计算下一位置的位姿
			mid_L.x = g_px;
			mid_L.y = g_py;
			mid_L.th = g_pth;
			mid_L.D = D_s;
			mid_L.delta = delta_s;
			//碰撞检测
			if (collision_check_flag == 1)
			{
				if (VehicleCollisionGrid(g_px, g_py, g_pth))
				{
					return 3;
				}
				pathpoint.push_back(mid_L);
			}
			else {
				pathpoint.push_back(mid_L);
			}
			//
		}
		return 4;
	}

	//2023.02.01
	int FindPathOne(double start[3], double end[3], int once_flag = 1)
	{
		double x = .0, y = .0, phi = .0;
		RsPath ccs_path;
		double road_yaw[6] = { 0,0,0,0,0,0 };
		CoordinateTrans(x, y, phi, end[0], end[1], end[2], start[0], start[1], start[2]);
		x = x / rmin_vertical;
		y = y / rmin_vertical;
		if (CpCmSm(x, y, phi, ccs_path, 1)) // theta_car == 1
		{
			for (int i = 0; i < 3; i++)
			{
				if (ccs_path.rspath_type[i] == St)
				{
					road_yaw[i * 2] = 1000000;
				}
				else if (ccs_path.rspath_type[i] == Le)
				{
					road_yaw[i * 2] = rmin_vertical;
				}
				else if (ccs_path.rspath_type[i] == Ri)
				{
					road_yaw[i * 2] = -rmin_vertical;
				}
			}
			road_yaw[1] = rmin_vertical * ccs_path.t;
			road_yaw[3] = rmin_vertical * ccs_path.u;
			// 
			road_yaw[5] = rmin_vertical * ccs_path.v;
			ccs_path.lenth = ccs_path.lenth * rmin_vertical;
			if (once_flag == 0)
			{
				road_yaw[2] = 0;
				road_yaw[3] = 0;
				road_yaw[4] = 0;
				road_yaw[5] = 0;
			}
			return GetOrCheckPathPointsMulti(start, road_yaw, 1);
		}
		return 0;
	}

	int FindPathTwo(double start[3], double end[3])
	{
		double x = .0, y = .0, phi = .0;
		RsPath ccs_path;
		double road_yaw[6] = { 0,0,0,0,0,0 };
		CoordinateTrans(x, y, phi, end[0], end[1], end[2], start[0], start[1], start[2]);
		x = x / rmin_vertical;
		y = y / rmin_vertical;
		int res = 0;
		int pp_size_before = pathpoint.size();
		int pp_size_now = 0;
		if (CmCpSm(x, y, phi, ccs_path, 1)) // || 
		{
			for (int i = 0; i < 3; i++)
			{
				if (ccs_path.rspath_type[i] == St)
				{
					road_yaw[i * 2] = 1000000;
				}
				else if (ccs_path.rspath_type[i] == Le)
				{
					road_yaw[i * 2] = rmin_vertical;
				}
				else if (ccs_path.rspath_type[i] == Ri)
				{
					road_yaw[i * 2] = -rmin_vertical;
				}
			}
			road_yaw[1] = rmin_vertical * ccs_path.t;
			road_yaw[3] = rmin_vertical * ccs_path.u;
			// 
			road_yaw[5] = rmin_vertical * ccs_path.v;
			ccs_path.lenth = ccs_path.lenth * rmin_vertical;

			res = GetOrCheckPathPointsMulti(start, road_yaw, 1);
			if ((PlanForwardFirst >= 3 || PlanBackFirst >= 3) && res == 0)
			{
				pp_size_now = pathpoint.size();
				for (int i = pp_size_now; i > pp_size_before; i--)
				{
					pathpoint.pop_back();
				}
			}
		}
		if ((PlanForwardFirst >= 3 || PlanBackFirst >= 3) && (res == 0) && (SmLpSm(x, y, phi, ccs_path, 1)))
		{
			for (int i = 0; i < 3; i++)
			{
				if (ccs_path.rspath_type[i] == St)
				{
					road_yaw[i * 2] = 1000000;
				}
				else if (ccs_path.rspath_type[i] == Le)
				{
					road_yaw[i * 2] = rmin_vertical;
				}
				else if (ccs_path.rspath_type[i] == Ri)
				{
					road_yaw[i * 2] = -rmin_vertical;
				}
			}
			road_yaw[1] = rmin_vertical * ccs_path.t;
			road_yaw[3] = rmin_vertical * ccs_path.u;
			// 
			road_yaw[5] = rmin_vertical * ccs_path.v;
			ccs_path.lenth = ccs_path.lenth * rmin_vertical;
			res = GetOrCheckPathPointsMulti(start, road_yaw, 1);
		}
		return res;
	}

	int PlanMultiOnce(double start[3], double end[3], int& f_flag)
	{

		int pp_size_current = pathpoint.size();
		int flag = FindPathOne(start, end, 0);
		if (flag == 4)
		{
			int count = 15;
			for (int i = pathpoint.size() - 1; i > pp_size_current; i--)
			{
				int pp_size_before = pathpoint.size();
				start[0] = pathpoint[i].x;
				start[1] = pathpoint[i].y;
				start[2] = pathpoint[i].th;
				flag = FindPathTwo(start, end); // C-C+S-
				if (flag == 4)
				{
					return  1;
				}
				else if (flag == 1)
				{
					return 2;
				}
				else {
					int pp_size_now = pathpoint.size();
					for (int i = pp_size_now; i > pp_size_before; i--)
					{
						pathpoint.pop_back();
					}
				}
				count--;
				pathpoint.pop_back();
				if (count < 0)
				{
					break;
				}
			}
		}
		f_flag = 0;
		return 0;
	}
	//
	bool FindPathMulti(double start[3], double end[3], int& f_flag)
	{
		int flag = 100;
		int pp_size_now = 0;
		int pp_size_before_one = pathpoint.size();
		int index = 0;
		bool forward_flag = 1;
		bool once_flag = 1;
		if (Gears == 1 && (PlanBackFirst >= 3 || PlanForwardFirst >= 3))
		{
			forward_flag = 0;
		}
		while (index < 5)
		{
			if (forward_flag)
			{
				flag = FindPathOne(start, end, 1);  // C+C-S-
			}
			//std::cout << "flag_one = " << flag << std::endl;
			if (flag == 0)  // && (PlanMulti <= 1)
			{
				return false;
			}
			else if (flag == 4)
			{
				return true;
			}
			else {
				if (once_flag && flag == 2) // && (PlanForwardFirst == 3 || PlanBackFirst == 3)
				{
					pp_size_now = pathpoint.size();
					for (int i = pp_size_now; i > pp_size_before_one; i--)
					{
						pathpoint.pop_back();
					}
					flag = PlanMultiOnce(start, end, f_flag);
					if (flag == 1)
					{
						return true;
					}
					else if (flag == 0)
					{
						return false;
					}
					pp_size_before_one = pathpoint.size();
					start[0] = pathpoint[pp_size_before_one - 1].x;
					start[1] = pathpoint[pp_size_before_one - 1].y;
					start[2] = pathpoint[pp_size_before_one - 1].th;
				}
				else
				{
					pp_size_now = pathpoint.size();
					start[0] = pathpoint[pp_size_now - 1].x;
					start[1] = pathpoint[pp_size_now - 1].y;
					start[2] = pathpoint[pp_size_now - 1].th;
					//std::cout << -ParkingSpaceFlag * start[1] << std::endl;
					if (-ParkingSpaceFlag * start[1] < 0.0)
					{
						return FindPathDirectly(start, end, 1);
					}
					//
					flag = FindPathTwo(start, end); // C-C+S-
					//std::cout << "flag2 = " << flag << std::endl;
					if (flag == 0)
					{
						return false;
					}
					else if (flag == 4)
					{
						//std::cout << "start_1"<< start[0] << std::endl;
						return true;
					}
					else {
						forward_flag = 1;
						//once_flag = 0; //2023.12.18
						pp_size_before_one = pathpoint.size();
						start[0] = pathpoint[pp_size_before_one - 1].x;
						start[1] = pathpoint[pp_size_before_one - 1].y;
						start[2] = pathpoint[pp_size_before_one - 1].th;
						if (flag == 2)
						{
							return FindPathDirectly(start, end, 1);
						}
					}
				}
			}
			index++;
		}
		return false;
	}
}

//2023.02.01




