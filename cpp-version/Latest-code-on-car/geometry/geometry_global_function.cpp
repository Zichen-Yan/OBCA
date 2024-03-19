#include "../common/global_variable.h"
#include "../common/global_function.h"

namespace byd_apa_plan
{
	bool VehicleCollisionGrid(double cpx, double cpy, double cph)
	{

		//2023.01.11
		// change coordinate End to Fusion Point
		double cpx_ = .0, cpy_ = .0, cph_ = .0;
		cpx_ = cpx * cos(end_to_fus[2]) - cpy * sin(end_to_fus[2]) + end_to_fus[0];
		cpy_ = cpx * sin(end_to_fus[2]) + cpy * cos(end_to_fus[2]) + end_to_fus[1];
		cph_ = cph + end_to_fus[2];
		//2023.01.11

		//bool isCollision = 0;
		double phi = mod2pi(cph_); // 构造旋转矩阵
		double cosphi = cos(-phi);
		double sinphi = sin(-phi);
		double rect_x;
		double rect_y;


		for (unsigned int i = 0; i < Rect_x.size(); i++)
		{
			rect_x = Rect_x[i] * cosphi + Rect_y[i] * sinphi + cpx_;
			rect_y = Rect_y[i] * cosphi - Rect_x[i] * sinphi + cpy_;
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
			int idx_xy = (xidx - 1) * pathfind_parameters.XIDX + (yidx - 1);
			//uint8 grid_status = obstmap[idx_xy].Status;
			if ((xidx <= 0 || xidx > pathfind_parameters.XIDX) || (yidx <= 0 || yidx > pathfind_parameters.XIDY))
			{
				//isCollision = true;
				//return true;
			}
			else if ((obstmap[idx_xy].Status == 0u) || (obstmap[idx_xy].Status == 7u))
			{
				//isCollision = true;
				return true;
			}
		}
		return false;
	}

	bool GetOnePathPoints(double start[3], double p_, double r_, const int collision_check_flag = 1)
	{
		double D = 0, delta = 0;
		path_point mid_point;

		mid_point.x = start[0];
		mid_point.y = start[1];
		mid_point.th = mod2pi(start[2]);
		mid_point.D = 0;
		mid_point.delta = 0;
		//pathpoint.push_back(mid_point);
		if (p_ < 0)
		{
			D = -0.1;
		}
		else {
			D = 0.1;
		}
		delta = atan(vehicle_parameters.WB / r_);
		if (r_ == 1000000)
		{
			delta = 0.0;
			r_ = 1.0;
		}
		int i_szie = (fabs(r_ * p_) / pathfind_parameters.MOTION_RESOLUTION);

		//int roadisCollision = 0;
		for (int i = 0; i < i_szie; i++)
		{
			VehicleDynamic(mid_point.x, mid_point.y, mid_point.th, D, delta); // 根据当前位姿和输入, 计算下一位置的位姿
			// 碰撞检测
			if (collision_check_flag == 1)
			{
				if (VehicleCollisionGrid(g_px, g_py, g_pth))
				{
					//std::cout << i << std::endl;
					return i > 3;
				}
			}
			mid_point.x = g_px;
			mid_point.y = g_py;
			mid_point.th = g_pth;
			mid_point.D = D;
			mid_point.delta = delta;
			pathpoint.push_back(mid_point);
		}
		if (fabs((fabs(r_) * p_) - i_szie * D) > 0.01)
		{
			VehicleDynamic(mid_point.x, mid_point.y, mid_point.th, ((fabs(r_) * p_) - i_szie * D), delta); // 根据当前位姿和输入, 计算下一位置的位姿
			// if(collision_check_flag == 1)
			// {
			// 	if(VehicleCollisionGrid(g_px, g_py, g_pth))
			// 	{
			// 		//std::cout << i << std::endl;
			// 		return false;
			// 	}
			// }
			mid_point.x = g_px;
			mid_point.y = g_py;
			mid_point.th = g_pth;
			mid_point.D = ((fabs(r_) * p_) - i_szie * D);
			mid_point.delta = delta;
			pathpoint.push_back(mid_point);
		}

		return true;
	}

	bool GetOrCheckPathPoints(const double* start, const double* road_yaw, const int& i_coll_max, const int &collision_check_flag)
	{
		path_point mid_point;
		mid_point.x = start[0];
		mid_point.y = start[1];
		mid_point.th = start[2];
		mid_point.D = 0;
		mid_point.delta = 0;
		double dir_resolution = .0, delta = .0;
		int i_path_size = 0;
		
		for(int i_coll = 0; i_coll < i_coll_max; i_coll++)
		{
			dir_resolution = (road_yaw[2*i_coll+1] < 0) ? -0.1 : 0.1;
			
			delta = (fabs(road_yaw[2*i_coll]) > 100000 || fabs(road_yaw[2*i_coll]) < rmin_vertical*0.5) ? .0 : atan(vehicle_parameters.WB / road_yaw[2*i_coll]);
			i_path_size = floor(fabs(road_yaw[2*i_coll+1]) / pathfind_parameters.MOTION_RESOLUTION);
			for(int i_path = 0; i_path < i_path_size; i_path++)
			{
				VehicleDynamic(mid_point.x, mid_point.y, mid_point.th, dir_resolution, delta);
				mid_point.x = g_px;
				mid_point.y = g_py;
				mid_point.th = g_pth;
				mid_point.D = dir_resolution;
				mid_point.delta = delta;
				if(collision_check_flag == 1)
				{
					if(VehicleCollisionGrid(g_px, g_py, g_pth))
					{
						return false;
					}
				}
				else
				{
					pathpoint.push_back(mid_point);
				}
			}
			double less_dist = road_yaw[2*i_coll+1] - i_path_size * dir_resolution;
			if (fabs(less_dist) > 0.01)
			{
				VehicleDynamic(mid_point.x, mid_point.y, mid_point.th, less_dist, delta);
				mid_point.x = g_px;
				mid_point.y = g_py;
				mid_point.th = g_pth;
				mid_point.D = less_dist;
				mid_point.delta = delta;
				if(collision_check_flag == 1)
				{
					if(VehicleCollisionGrid(g_px, g_py, g_pth))
					{
						return false;
					}
				}
				else
				{
					pathpoint.push_back(mid_point);
				}
			}
		}
		return true;
	}
    

	void CoordinateTransToPrk()
	{
		//std::cout << "changetoprk" << std::endl;
		double x = .0, y = .0, phi = .0;
		//2022.12.30
		//变换到终点坐标系
		CoordinateTrans(x, y, phi, start_to_fus[0], start_to_fus[1], start_to_fus[2], end_to_fus[0], end_to_fus[1], end_to_fus[2]);
		Start[0] = x;
		Start[1] = y;
		Start[2] = phi;
		CoordinateTrans(x, y, phi, end_to_fus[0], end_to_fus[1], end_to_fus[2], end_to_fus[0], end_to_fus[1], end_to_fus[2]);
		End[0] = x;
		End[1] = y;
		End[2] = phi;
		//2022.12.30
		//2023.01.04
		if (fusion.parkingSpaceInfo.ParkingSpaceType != 2)
		{
			if (end_to_fus[2] > 0)
			{
				ParkingSpaceFlag = (fusion.ParkInMode == 0) ? 1 : -1;
				CoordinateTrans(x, y, phi, P3x, P3y, .0, end_to_fus[0], end_to_fus[1], end_to_fus[2]);
				P_r[0] = x;
				//P_r[1] = (fusion.TraceParkingID_USS != 0u) ? (y - 0.2) : y;
				P_r[1] = y - 0.2; // 20231205
			}
			else
			{
				ParkingSpaceFlag = (fusion.ParkInMode == 0) ? -1 : 1;
				CoordinateTrans(x, y, phi, P0x, P0y, .0, end_to_fus[0], end_to_fus[1], end_to_fus[2]);
				P_r[0] = x;
				//P_r[1] = (fusion.TraceParkingID_USS != 0u) ? (y + 0.2) : y;
				P_r[1] = y + 0.2; // 20231205
			}
		}
		else
		{
			if (app.APA_Park_Function == 2)
			{
				if (index_request == 1)
				{
					Start_out[0] = start_to_fus[0];
					Start_out[1] = start_to_fus[1];
					Start_out[2] = start_to_fus[2];
					if (end_to_fus[1] > 0)
					{
						right_left = 1;
					}
					else
					{
						right_left = -1;
					}
				}
			}
			else
			{
				if (index_request == 1)
				{
					if (Start[1] > 1) // 右泊
					{
						right_left = 1;
					}
					else if (Start[1] < -1) // 左泊
					{
						right_left = -1;
					}
				}
			}
		}
	}

	void CoordinateTrans(double& x, double& y, double& phi, double x_a, double y_a, double phi_a, double x_s, double y_s, double phi_s)
	{
		//
		double dx = x_a - x_s;
		double dy = y_a - y_s;
		x = dx * cos(phi_s) + dy * sin(phi_s);
		y = dy * cos(phi_s) - dx * sin(phi_s);
		phi = mod2pi(phi_a - phi_s);
	}

	std::vector<path_point> PathPointsCoordinateTrans(const std::vector<path_point>& pathpoints_, double end_to_fus_point[3])
	{
		int pp_size = pathpoints_.size();
		double x, y, phi;
		path_point pathpoint_;
		std::vector<path_point> pathpoints;
		for (int i = 0; i < pp_size; i++)
		{
			// 坐标系变换
			x = pathpoints_[i].x * cos(end_to_fus_point[2]) - pathpoints_[i].y * sin(end_to_fus_point[2]) + end_to_fus_point[0];
			y = pathpoints_[i].x * sin(end_to_fus_point[2]) + pathpoints_[i].y * cos(end_to_fus_point[2]) + end_to_fus_point[1];
			phi = pathpoints_[i].th + end_to_fus_point[2];
			pathpoint_.x = x;
			pathpoint_.y = y;
			pathpoint_.th = phi;
			pathpoint_.D = pathpoints_[i].D;
			pathpoint_.delta = pathpoints_[i].delta;
			pathpoints.push_back(pathpoint_);
		}
		return pathpoints;
	}

	std::vector<std::pair<double, double>> CalcCornerPoints(const double start[3], double d_l, double d_w)
	{
		double x = start[0], y = start[1], phi = start[2];
		double d0 = sqrt((vehicle_parameters.W / 2 + d_w / 2) * (vehicle_parameters.W / 2 + d_w / 2) + (vehicle_parameters.LB + d_l / 2) * (vehicle_parameters.LB + d_l / 2));// 未考虑安全边框
		double phi0 = acos((vehicle_parameters.LB + d_l / 2) / d0);
		double d1 = sqrt((vehicle_parameters.W / 2 + d_w / 2) * (vehicle_parameters.W / 2 + d_w / 2) + (vehicle_parameters.LF + d_l / 2) * (vehicle_parameters.LF + d_l / 2));
		double phi1 = acos((vehicle_parameters.LF + d_l / 2) / d1);


		std::pair<double, double> point;
		std::vector<std::pair<double, double>> cornerpoints;

		point.first = x - d0 * cos(phi - phi0);
		point.second = y - d0 * sin(phi - phi0);
		cornerpoints.push_back(point);

		point.first = x - d0 * cos(phi + phi0);
		point.second = y - d0 * sin(phi + phi0);
		cornerpoints.push_back(point);

		point.first = x + d1 * cos(phi - phi1);
		point.second = y + d1 * sin(phi - phi1);
		cornerpoints.push_back(point);

		point.first = x + d1 * cos(phi + phi1);
		point.second = y + d1 * sin(phi + phi1);
		cornerpoints.push_back(point);

		return cornerpoints;
	}
	
	std::vector<std::pair<double, double>> CalcCentreOfCircle(double x_, double y_, double phi_, double r_)  // 输入后轴中心点
	{
		std::vector<std::pair<double, double>> centrepoints;
		std::pair<double, double> point;

		// left
		point.first = x_ - r_ * sin(phi_);
		point.second = y_ + r_ * cos(phi_);
		centrepoints.push_back(point);
		// right
		point.first = x_ + r_ * sin(phi_);
		point.second = y_ - r_ * cos(phi_);
		centrepoints.push_back(point);

		return centrepoints;
	}
}
