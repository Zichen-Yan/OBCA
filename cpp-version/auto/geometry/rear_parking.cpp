#include "../common/global_variable.h"
#include "../common/global_function.h"

namespace byd_apa_plan
{
	bool FindPathDirectly(double start[3], double end[3], int directly_flag)
	{

		double road_yaw[6] = { 0, 0, 0, 0, 0, 0 };
		double road_yaw2[6] = { 0, 0, 0, 0, 0, 0 };

		// 2022.12.28
		int roadfirst = 1;
		roadfirst = Calc_Cm(start[0], start[1], start[2], end[0], end[1], end[2], 0.03, rmin_vertical, 0.3, road_yaw);
		if (roadfirst == 0)
		{
			if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) // 碰撞检测
			{
				// std::cout << "C-" <<std::endl;
				return GetOrCheckPathPoints(start, road_yaw, 3, 0);
			}
		}

		int roadsecond = 1;
		roadsecond = Calc_CmSm(start[0], start[1], start[2] * 180 / pi, end[0], end[1], end[2] * 180 / pi, rmin_vertical, 0.1, road_yaw);
		if (roadsecond == 0)
		{
			if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) // 碰撞检测
			{
				return GetOrCheckPathPoints(start, road_yaw, 3, 0);
			}
		}
		else if (roadsecond == 3)
		{
			if (GetOrCheckPathPoints(start, road_yaw, 3, 1))
			{
				road_yaw[2] = .0;
				road_yaw[3] = .0;
				int pp_size_before = pathpoint.size();
				if (GetOrCheckPathPoints(start, road_yaw, 3, 0))
				{
					int pp_size_now = pathpoint.size();
					int roadthird_cc = 1;
					for (int i = pp_size_now; i > pp_size_before; i--)
					{
						double start_p[3] = { pathpoint[i - 1].x, pathpoint[i - 1].y, pathpoint[i - 1].th };
						roadthird_cc = Calc_CmCm(start_p[0], start_p[1], start_p[2], end[0], end[1], end[2], rmin_vertical, road_yaw);

						if (roadthird_cc == 0)
						{
							if (GetOrCheckPathPoints(start_p, road_yaw, 3, 1)) // 碰撞检测
							{
								//std::cout << "C-C--C--" << std::endl;
								//return true;
								return GetOrCheckPathPoints(start_p, road_yaw, 3, 0);
							}
						}
						pathpoint.pop_back();
					}
					//return false;
				}
			}
		}

		// S-C-S-
		if (Calc_SmCmSm(start[0], start[1], start[2], end[0], end[1], end[2], rmin_vertical, road_yaw))
		{
			if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) // 碰撞检测
			{
				//std::cout << "S-C-S-\n";
				return GetOrCheckPathPoints(start, road_yaw, 3, 0);
			}
		}
		// C-C-
		double theta_limited = (directly_flag == 1) ? pi / 6 : pi / 18;
		if ((fabs(mod2pi(end[2] - start[2])) < theta_limited)) // 2.0*0.0873
		{
			int roadthird = 1;
			roadthird = Calc_CmCm(start[0], start[1], start[2], end[0], end[1], end[2], rmin_vertical, road_yaw);
			if (roadthird == 0)
			{
				if (directly_flag == 1)
				{
					return GetOrCheckPathPoints(start, road_yaw, 3, 0); //
				}
				// final plan
				if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) // 碰撞检测
				{
					return GetOrCheckPathPoints(start, road_yaw, 3, 0);
				}
			}
		}
		/*
		if (dynamic_final_end_flag) // 垂直车位逼停出去调整直接报完成
		{
			if (fabs(Start[0]) < 0.5 && fabs(Start[1]) < 0.1 && fabs(Start[2]) < 1 * pi / 180)
			{
				PathPoint end_point;
				end_point.x = end[0];
				end_point.y = end[1];
				end_point.th = end[2];
				end_point.D = -0.1;
				end_point.delta = 0;
				pathpoint.push_back(end_point);
				plan.IsPlanningCompleted = 1;
				return true;
			}
		}
		*/
		if (directly_flag == 0)
		{
			int roadfourth = 1;
			roadfourth = Calc_CpSm(start[0], start[1], start[2] * 180 / pi, end[0], end[1], end[2] * 180 / pi, rmin_vertical, 0.0, road_yaw);
			if (roadfourth == 0)
			{
				int res_cs = GetOrCheckPathPointsMulti(start, road_yaw, 1); // res == 1 or res == 4;
				if (res_cs == 1)
				{
					double current_start[3] = { .0, .0, .0 };
					int res_cc = 1;
					for (int i = 0; i < 2; i++)
					{
						pathpoint.pop_back();
					}
					int pp_size = pathpoint.size();
					current_start[0] = pathpoint[pp_size - 1].x;
					current_start[1] = pathpoint[pp_size - 1].y;
					current_start[2] = pathpoint[pp_size - 1].th;
					res_cc = Calc_CmCm(current_start[0], current_start[1], current_start[2], end_to_prk[0], end_to_prk[1], end_to_prk[2], rmin_vertical, road_yaw);
					if (res_cc == 0)
					{
						// 碰撞检测
						if (GetOrCheckPathPoints(current_start, road_yaw, 3, 1)) // 碰撞检测
						{
							//printf("C++C-C-\n");
							return GetOrCheckPathPoints(current_start, road_yaw, 3, 0);
						}
						//
					}
				}
				else {
					return true;
				}
			}
		}

		if (directly_flag == 0)
		{
			if (Gears == 2) // 2023.03.10
			{
				int roadeigth = 1;
				roadeigth = Calc_CpCmSm(start[0], start[1], start[2], end[0], end[1], end[2], rmin_vertical, road_yaw, road_yaw2);
				if (roadeigth == 0)
				{
					//
					if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) // 碰撞检测
					{
						//std::cout << "C+C-S-0" <<std::endl;
						return GetOrCheckPathPoints(start, road_yaw, 3, 0);//取路
					}
					//
				}
				else if (roadeigth == 1)
				{
					//
					return GetOrCheckPathPoints(start, road_yaw2, 3, 0);
				}
			}
			// 2023.03.10

			int roadsixth = 1;
			roadsixth = Calc_CpCpSm(start[0], start[1], start[2], end[0], end[1], end[2], rmin_vertical, road_yaw);
			//std::cout << "roadsixth = " << roadsixth << std::endl;
			if (roadsixth == 0)
			{
				//
				if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) // 碰撞检测
				{
					//std::cout << "C+C+S-" <<std::endl;
					//return true;
					return GetOrCheckPathPoints(start, road_yaw, 3, 0);
				}
				//
			}


			if (Gears == 1)
			{
				int roadseventh = 1;
				roadseventh = Calc_CmCpSm(start[0], start[1], start[2], end[0], end[1], end[2], rmin_vertical, road_yaw);
				if (roadseventh == 0)
				{
					//
					if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) // 碰撞检测
					{
						// std::cout << "C-C+S-" <<std::endl;
						flag_dynamic_plan_advanced = true;
						return GetOrCheckPathPoints(start, road_yaw, 3, 0);
					}
					//
				}
			}
		}
		return false;
	}
	// 2022.12.16

	//
	bool CalcForwardFirstParam(double start[3], std::pair<double, double> cenpoint, double& rmin_, double t_, double& x_lim_)
	{
		double end_mid[3] = { 0, 0, 0 };
		// 2023.01.03
		double dx = start[0] - cenpoint.first;
		double dy = start[1] - cenpoint.second;
		double theta = ParkingSpaceFlag * atan((dy / dx));
		end_mid[0] = cenpoint.first - rmin_ * cos(t_ + theta);
		end_mid[1] = cenpoint.second - ParkingSpaceFlag * rmin_ * sin(t_ + theta);
		end_mid[2] = start[2] + ParkingSpaceFlag * t_;
		// 2023.01.03
		std::vector<std::pair<double, double>> centrepoints = CalcCentreOfCircle(end_mid[0], end_mid[1], end_mid[2], rmin_vertical);
		// 2023.01.03
		double dist_to_P_r = .0;
		if (ParkingSpaceFlag == 1)
		{
			dist_to_P_r = sqrt(pow((rmin_vertical - vehicle_parameters.W / 2), 2) - pow((P_r_x_first - centrepoints[1].first), 2)) - fabs(P_r_y_first - centrepoints[1].second);
		}
		else
		{
			dist_to_P_r = sqrt(pow((rmin_vertical - vehicle_parameters.W / 2), 2) - pow((P_r_x_first - centrepoints[0].first), 2)) - fabs(P_r_y_first - centrepoints[0].second);
		}
		// 2023.01.03
		if (dist_to_P_r > 0 && (fabs(dist_to_P_r - Aim_dist) < 0.05))
		{
			return true;
		}
		if (dist_to_P_r > Aim_dist + 0.05)
		{
			// Delta_Y = Delta_Y - ParkingSpaceFlag * 0.1;
			rmin_ = rmin_ + 0.1;
		}
		if (dist_to_P_r < Aim_dist - 0.05)
		{
			x_lim_ = x_lim_ - 0.1;
		}
		return false;
	}

	// 2023.01.12
	bool IsBackwardFirst(double start[3])
	{
		double dist_to_P_r = .0;
		std::vector<std::pair<double, double>> centrepoints = CalcCentreOfCircle(start[0], start[1], start[2], rmin_vertical);
		if (ParkingSpaceFlag == 1 && (start[1] < P_r_y_first || fabs(start[2]) < pi/3)) //start[1] < P_r_y_first
		{
			dist_to_P_r = sqrt(pow((rmin_vertical - vehicle_parameters.W / 2), 2) - pow((P_r_x_first - centrepoints[1].first), 2)) - fabs(P_r_y_first - centrepoints[1].second);
			return (dist_to_P_r - Aim_dist <= 0.02);
		}
		else if (ParkingSpaceFlag == -1 && (start[1] > P_r_y_first || fabs(start[2]) < pi/3)) //&& start[1] > P_r_y_first
		{
			dist_to_P_r = sqrt(pow((rmin_vertical - vehicle_parameters.W / 2), 2) - pow((P_r_x_first - centrepoints[0].first), 2)) - fabs(P_r_y_first - centrepoints[0].second);
			return (dist_to_P_r - Aim_dist <= 0.02);
		}
		return false;
	}
	// 2023.01.12

	// 2023.01.30
	bool CalcForwardParamLimitedVertical(double& t_, double& rmin_, double start[3], double x_lim_, double w_exp = 0.0, double l_exp = 0.0)
	{
		bydapa::common::TicToc tictoc_ti;
		tictoc_ti.tic();
		while (1)
		{
			double nseconds_ti = tictoc_ti.toc();
			nseconds_ti = nseconds_ti * 1.0e-9;
			if (nseconds_ti > 0.2)
			{
				return false;
			}
			// 左转前进
			std::vector<std::pair<double, double>> cornerpoints = CalcCornerPoints(start, l_exp, w_exp);
			std::vector<std::pair<double, double>> centrepoints = CalcCentreOfCircle(start[0], start[1], start[2], rmin_);
			double r_lf = sqrt((rmin_ - vehicle_parameters.W / 2 - w_exp / 2) * (rmin_ - vehicle_parameters.W / 2 - w_exp / 2) + (vehicle_parameters.LF + l_exp) * (vehicle_parameters.LF + l_exp));
			
			// 2023.01.03
			double x_1 = .0, x_2 = .0;
			std::pair<double, double> centrepoint;
			if (ParkingSpaceFlag == 1)
			{
				x_1 = -((cornerpoints[3].first - centrepoints[0].first) / r_lf);
				x_2 = -(x_lim_ - centrepoints[0].first) / r_lf;
				centrepoint = centrepoints[0];
			}
			else
			{
				x_1 = -((cornerpoints[2].first - centrepoints[1].first) / r_lf);
				x_2 = -(x_lim_ - centrepoints[1].first) / r_lf;
				centrepoint = centrepoints[1];
			}
			// 2023.01.03

			if ((fabs(x_1) > 1.0) || (fabs(x_2) > 1.0))
			{
				return false;
			}

			t_ = mod2pi(asin(x_1) - asin(x_2));

			if (t_ > 0)
			{
				if (CalcForwardFirstParam(start, centrepoint, rmin_, t_, x_lim_))
				{
					rmin_ = ParkingSpaceFlag * rmin_;
					return t_ > 0;
				}
			}
			else
			{
				// rmin_ = rmin_+0.1;
				return false;
			}
		}
	}

	// 第二段倒退
	bool CalcBackwardParamLimitedVertical(double& u_, double start[3], double& r_, double aim_dy = 0.0)
	{
		// 左右转圆心
		std::vector<std::pair<double, double>> centrepoints = CalcCentreOfCircle(start[0], start[1], start[2], r_);
		// 2023.01.03
		double x_1 = .0, x_2 = .0, dist_to_P_r = .0;

		if (ParkingSpaceFlag == 1)
		{
			//dist_to_P_r = (r_ - vehicle_parameters.W / 2) - sqrt((P_r[0] - centrepoints[1].first) * (P_r[0] - centrepoints[1].first) + (P_r[1] - centrepoints[1].second) * (P_r[1] - centrepoints[1].second));
			dist_to_P_r = sqrt(pow((r_ - vehicle_parameters.W / 2), 2) - pow((P_r_x_first - centrepoints[1].first), 2)) - fabs(P_r_y_first - centrepoints[1].second);
			x_1 = ((start[1] - centrepoints[1].second) / r_);
			x_2 = ((aim_dy - centrepoints[1].second) / r_);
		}
		else
		{
			aim_dy = -aim_dy;
			//dist_to_P_r = (r_ - vehicle_parameters.W / 2) - sqrt((P_r[0] - centrepoints[0].first) * (P_r[0] - centrepoints[0].first) + (P_r[1] - centrepoints[0].second) * (P_r[1] - centrepoints[0].second));
			dist_to_P_r = sqrt(pow((r_ - vehicle_parameters.W / 2), 2) - pow((P_r_x_first - centrepoints[0].first), 2)) - fabs(P_r_y_first - centrepoints[0].second);
			x_1 = ((start[1] - centrepoints[0].second) / r_);
			x_2 = ((aim_dy - centrepoints[0].second) / r_);
		}
		// 2023.01.03
		double first_end_offset = 1.0;
		if(PlanBackwardFirst == 0 && PlanForwardFirst == 2)
		{
			first_end_offset = sqrt(pow(start_to_fus[0] - first_path_end.first, 2) + pow(start_to_fus[1] - first_path_end.second, 2));
		}
		bool flag  = ((0.01 * floor(dist_to_P_r * 100) - aim_dist[4] - 0.1 < 1e-6)
					&& (r_ == rmin_vertical || fabs(dist_to_P_r - Aim_dist) - 0.05  < 1e-6))
					|| (first_end_offset - 0.2 < 1e-6 || control.ObsUssInfo < 4u);
		if (!flag) //
		{
			return false;
		}
		// 2022.12.16

		while (fabs(x_2) > 1.0)
		{
			// 2023.02.24
			if (ParkingSpaceFlag == 1)
			{
				aim_dy = aim_dy - 0.01;
				x_2 = ((aim_dy - centrepoints[1].second) / r_);
			}
			else
			{
				aim_dy = aim_dy + 0.01;
				x_2 = ((aim_dy - centrepoints[0].second) / r_);
			}
			// 2023.02.24
		}
		if (fabs(x_1) <= 1.0 && fabs(x_2) <= 1.0)
		{
			u_ = mod2pi(-fabs(asin(x_1) - asin(x_2)));
			// 2023.01.04
			r_ = -ParkingSpaceFlag * r_;
			// 2023.01.04
			double end_theta = start[2] - ParkingSpaceFlag * u_;
			bool flag = 1;
			if (PlanBackwardFirst == 2 && PlanForwardFirst == 0)
			{
				flag = (fabs(end_theta) < pi/6) ? 1 : 0;
			}
			return (u_ < 0 && flag) || (fabs(r_) == rmin_vertical);
		}
		return false;
	}

	// 2023.01.12
	bool CalcBackwardFirstParam(double& r, double& u, double start[3], double aim_dy)
	{
		double a = (vehicle_parameters.W / 2 + Aim_dist);
		double b = (P_r_y_first - start[1]);
		double c = (P_r_x_first - start[0]);
		double d = 2 * ParkingSpaceFlag * (b * cos(start[2]) - c * sin(start[2])) + 2 * a;
		double e = a * a - b * b - c * c;

		r = e / d;
		return (ParkingSpaceFlag * r < 0) && CalcBackwardParamLimitedVertical(u, start, r, aim_dy);
	}
	// 2023.01.12

	// 2023.03.14
	bool CalcStraightParam(double start[3], double& dist, double& rmin_, int direction_flag)
	{

		double x = start[0], y = start[1], theta = start[2];
		double y_a = P_r_y_first + ParkingSpaceFlag * Aim_dist;
		double a = 2 * direction_flag * ((x - P_r_x_first) * cos(theta) + (y - y_a) * sin(theta));
		double b = x * x + y * y + rmin_vertical * rmin_vertical + P_r_x_first * P_r_x_first + y_a * y_a + 2 * rmin_vertical * ParkingSpaceFlag * ((x - P_r_x_first) * sin(theta) + (y_a - y) * cos(theta)) - 2 * (x * P_r_x_first + y_a * y);
		double c = rmin_vertical - vehicle_parameters.W / 2;
		double delta = a * a - 4 * (b - c * c);
		if (!(delta < 0))
		{
			double d1 = (-a + sqrt(delta)) / 2;
			double d2 = (-a - sqrt(delta)) / 2;
			if (direction_flag == 1)
			{
				dist = d1 > d2 ? d1 : d2;
			}
			else {
				dist = d1 < d2 ? d1 : d2;
			}
			if ((dist > 0) || (direction_flag * dist > 0))
			{
				rmin_ = 1e6;
				return true;
			}
		}
		return false;
	}
	// 2023.03.14

	//2023.03.16
	bool CalcStraightBackwardParam(double& r, double& u, double start[3], double aim_dy)
	{
		double r_ = 0, v_ = 0, w_ = 0;
		int pp_size_now = 0;
		double start_p[3] = { start[0], start[1], start[2] };
		v_ = (start[2] + ParkingSpaceFlag * pi / 2);
		r = rmin_vertical;
		pp_size_now = pathpoint.size() - 1;
		if (v_ > 0)
		{
			r_ = rmin_vertical;
			v_ = -v_;
		}
		else {
			r_ = -rmin_vertical;
		}

		if (!((v_ < 0 && v_ > -10 * pi / 180 && ParkingSpaceFlag == 1) || (v_ > 0 && v_ < 10 * pi / 180 && ParkingSpaceFlag == -1)))
		{
			if (GetOnePathPoints(start_p, v_, r_, 1)) //倒车调成水平
			{
				pp_size_now = pathpoint.size() - 1;
				start_p[0] = pathpoint[pp_size_now].x;
				start_p[1] = pathpoint[pp_size_now].y;
				start_p[2] = pathpoint[pp_size_now].th;
			}
		}

		if (CalcStraightParam(start_p, w_, r_, -1))
		{
			if (w_ > 0)
			{
				if (GetOnePathPoints(start_p, -w_, r_, 1))
				{
					pp_size_now = pathpoint.size() - 1;
					start[0] = pathpoint[pp_size_now].x;
					start[1] = pathpoint[pp_size_now].y;
					start[2] = pathpoint[pp_size_now].th;
					//std::cout << start[0] << " " <<start[1] <<  " " << start[2] << std::endl;	
				}
			}
			else {
				for (int i = pp_size_now; i > 0; i--)
				{
					start[0] = pathpoint[i].x;
					start[1] = pathpoint[i].y;
					start[2] = pathpoint[i].th;
					if (CalcBackwardParamLimitedVertical(u, start, r, aim_dy))
					{
						return true;
					}
					pathpoint.pop_back();
				}
				return false;
			}

		}
		else {
			return false;
		}
		return CalcBackwardParamLimitedVertical(u, start, r, aim_dy);
	}
	//2023.03.16

	// 2022.12.15
	bool FindPathForwardFirst(double start[3], double end[3], double x_lim_)
	{
		double t = .0, u = .0;
		double rmin_ = rmin_vertical, r_back = rmin_vertical;
		int pp_size_now = 0;
		double start_p[3] = { start[0], start[1], start[2] };
		// 2023.01.03
		if (PlanForwardFirst > 0 && PlanForwardFirst < 3)
		{
			int pp_size_first = pathpoint.size();
			// 2022.12.19
			if (CalcBackwardParamLimitedVertical(u, start_p, r_back, Aim_dy)) // 加入距离判断
			{
				if (GetOnePathPoints(start_p, u, r_back, 1))
				{
					int count = 15;
					int f_flag_front = 1;
					for (int i = pathpoint.size() - 1; i > pp_size_first; i--)
					{
						int pp_size_before = pathpoint.size();
						start_p[0] = pathpoint[i].x;
						start_p[1] = pathpoint[i].y;
						start_p[2] = pathpoint[i].th;
						// int flag = FindPathMulti(start, end);
						if (FindPathMulti(start_p, end, f_flag_front))
						{
							//
							PlanForwardFirst++;
							return true;
						}
						else
						{
							pp_size_now = pathpoint.size();
							for (int i = 0; i < fabs(pp_size_now - pp_size_before); i++)
							{
								pathpoint.pop_back();
							}
						}
						pathpoint.pop_back();
						count--;
						if (count < 0 || (f_flag_front == 0))
						{
							break;
						}
					}
				}
			}
			int index = pathpoint.size() - 1;
			for (int i = 0; i < index; i++)
			{
				pathpoint.pop_back();
			}
			PlanForwardFail = 0; // 2022.03.06
			if (PlanForwardFirst == 1) // problem 2023.03.08
			{
				PlanForwardFirst = 0;
			}
			return false;
		}

		if (CalcForwardParamLimitedVertical(t, rmin_, start_p, x_lim_, W_exp, L_exp))
		{
			if (GetOnePathPoints(start_p, t, rmin_, 1))
			{
				pp_size_now = pathpoint.size();
				if (pp_size_now < 10)
				{
					int index = pathpoint.size() - 1;
					for (int i = 0; i < index; i++)
					{
						pathpoint.pop_back();
					}
					return false;
				}
				PlanForwardFirst++;
				start_p[0] = pathpoint[pp_size_now - 1].x;
				start_p[1] = pathpoint[pp_size_now - 1].y;
				start_p[2] = pathpoint[pp_size_now - 1].th;
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
		return FindPathForwardFirst(start_p, end, x_lim_);
	}
	// 2022.12.15
	bool FindPathStraightForwardFirst(double start[3], double end[3], double x_lim_)
	{
		double t = .0, u = .0;
		double rmin_ = rmin_vertical, r_back = rmin_vertical;
		int pp_size_now = 0;

		double start_p[3] = { start[0], start[1], start[2] };
		//std::cout << "index_ = " << pathpoint.size() << std::endl;
		// 2023.01.03
		if (PlanForwardFirst > 0 && PlanForwardFirst < 3)
		{
			int pp_size_first = pathpoint.size();
			// 2022.12.19
			if (CalcBackwardParamLimitedVertical(u, start_p, r_back, Aim_dy)) // 加入距离判断
			{
				//std:: cout << "1111" << std::endl;
				if (GetOnePathPoints(start_p, u, r_back, 1))
				{
					//return true;
					int count = 15;
					int f_flag_front = 1;
					for (int i = pathpoint.size() - 1; i > pp_size_first; i--)
					{
						int pp_size_before = pathpoint.size();
						start_p[0] = pathpoint[i].x;
						start_p[1] = pathpoint[i].y;
						start_p[2] = pathpoint[i].th;
						// int flag = FindPathMulti(start, end);
						if (FindPathMulti(start_p, end, f_flag_front))
						{
							// CCS碰撞检测
							PlanForwardFirst++;
							if (FLAG_straight_forward)
							{
								PlanForwardFirst = 0;
							}
							return true;
						}
						else
						{
							pp_size_now = pathpoint.size();
							for (int i = 0; i < fabs(pp_size_now - pp_size_before); i++)
							{
								pathpoint.pop_back();
							}
						}
						//std::cout << "pppp" << std::endl;
						pathpoint.pop_back();
						count--;
						if (count < 0 || (f_flag_front == 0))
						{
							break;
						}
					}
				}
			}
			int index = pathpoint.size() - 1;
			for (int i = 0; i < index; i++)
			{
				pathpoint.pop_back();
			}
			//std::cout << "index_ = " << pathpoint.size() << std::endl;
			//return true;
			PlanForwardFail = 0; // 2022.03.06
			if (PlanForwardFirst == 1 || PlanForwardFirst == 2) // 2024.04.16: only straight forward PlanForwardFirst == 2 clear
			{
				PlanForwardFirst = 0;
			}
			return false;
			// 2022.12.16
		}
		if (CalcStraightParam(start_p, t, rmin_, 1))
		{
			long unsigned int s_size = 300;
			//std::cout << "&&&&&&& t = " << t << std::endl;
			if (t < 1.0 && t > 0)
			{
				s_size = fabs(t) / pathfind_parameters.MOTION_RESOLUTION;
				FLAG_straight_forward = true;
				t = t > 0 ? 1.2 : -1.2;
			}
			if (GetOnePathPoints(start_p, t, rmin_, 1))
			{
				pp_size_now = (pathpoint.size() - 1) < s_size ? (pathpoint.size() - 1) : s_size;
				PlanForwardFirst++;
				start_p[0] = pathpoint[pp_size_now].x;
				start_p[1] = pathpoint[pp_size_now].y;
				start_p[2] = pathpoint[pp_size_now].th;
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
		return FindPathStraightForwardFirst(start_p, end, x_lim_);
	}

	// 2023.01.12
	bool FindPathBackwardFirst(double start[3], double end[3])
	{
		double u = .0;
		double rmin_ = rmin_vertical;
		int pp_size_now = 0;
		double start_p[3] = { start[0], start[1], start[2] };
		if (PlanBackwardFirst < 3)
		{
			int pp_size_first = pathpoint.size();
			if (CalcBackwardFirstParam(rmin_, u, start_p, Aim_dy) || CalcStraightBackwardParam(rmin_, u, start_p, Aim_dy))
			{
				if (GetOnePathPoints(start_p, u, rmin_, 1))
				{
					int count = 15;
					int f_flag_back = 1;
					for (int i = pathpoint.size() - 1; i > pp_size_first; i--)
					{
						int pp_size_before = pathpoint.size();
						start_p[0] = pathpoint[i].x;
						start_p[1] = pathpoint[i].y;
						start_p[2] = pathpoint[i].th;
						// int flag = FindPathMulti(start, end);
						if (FindPathMulti(start_p, end, f_flag_back))
						{
							//
							PlanBackwardFirst++;
							return true;
						}
						else
						{
							pp_size_now = pathpoint.size();
							for (int i = 0; i < fabs(pp_size_now - pp_size_before); i++)
							{
								pathpoint.pop_back();
							}
						}
						pathpoint.pop_back();
						count--;
						if (count < 0 || (f_flag_back == 0))
						{
							break;
						}
					}
				}
			}
		}
		PlanBackwardFirst = 0; // 2023.02.09 paln failure
		return false;
	}
	// 2023.01.12

	//
	bool FindPathBackwardDirectly(double start[3], double end[3])
	{
		double r_ = 0, v_ = 0;
		double road_yaw[6] = { 0 };
		int pp_size_now = 0;
		double start_p[3] = { start[0], start[1], start[2] };
		if (fusion.parkingSpaceInfo.ParkingSpaceType == 1)
		{
			v_ = (start[2] + ParkingSpaceFlag * pi / 2);
			pp_size_now = pathpoint.size() - 1;
			if (v_ > 0)
			{
				r_ = rmin_vertical;
				v_ = -v_;
			}
			else {
				r_ = -rmin_vertical;
			}
			//std::cout << "rv = " << r_*v_ << std::endl;
			if (GetOnePathPoints(start_p, v_, r_, 1)) //倒车调成水平
			{
				pp_size_now = pathpoint.size() - 1;
				start_p[0] = pathpoint[pp_size_now].x;
				start_p[1] = pathpoint[pp_size_now].y;
				start_p[2] = pathpoint[pp_size_now].th;
			}
		}

		bool flag = (ParkingSpaceFlag == 1 && (dist_P23 == -1 || dist_P23 > 120)) 
					|| (ParkingSpaceFlag == -1 && (dist_P01 == -1 || dist_P01 > 120));
		if (flag && Calc_SmCmSm(start_p[0], start_p[1], start_p[2], end[0], end[1], end[2], rmin_vertical + 0.3, road_yaw)
			 && (road_yaw[5] < -0.3))
		{
			if (GetOrCheckPathPoints(start_p, road_yaw, 3, 1))
			{
				PlanBackwardFirst += 4;
				return GetOrCheckPathPoints(start_p, road_yaw, 3, 0);
			}
		}
		int index = pathpoint.size() - 1;
		for (int i = 0; i < index; i++)
		{
			pathpoint.pop_back();
		}
		return false;
	}
	//
	bool CalcOncePathVertical(double* start,double* end,const double &d)
	{
		//std::cout << "ooooooooooooo\n";
		double road_yaw[10] = {.0};
		double x_0 = start[0], y_0 = start[1], theta = start[2];
		//double delta_r = 0.0;
		double x_1 = end[0] + d * cos(end[2]) + ParkingSpaceFlag * rmin_vertical * sin(end[2]);
		double y_1 = end[1] + d * sin(end[2]) - ParkingSpaceFlag * rmin_vertical * cos(end[2]);
		double a = x_0 * x_0 + y_0 * y_0 + x_1 * x_1 + y_1 * y_1 - 2 * x_0 * x_1 - 2 * y_0 * y_1 + 2 * ParkingSpaceFlag * y_0 * delta_r - 2 * ParkingSpaceFlag * y_1 * delta_r - rmin_vertical * rmin_vertical - 2 * rmin_vertical * delta_r;
		double b = 2 * (rmin_vertical + delta_r - ParkingSpaceFlag * x_1 * sin(theta) + ParkingSpaceFlag * x_0 * sin(theta) - ParkingSpaceFlag * y_0 * cos(theta) + ParkingSpaceFlag * y_1 * cos(theta) - delta_r * cos(theta));
		double r_ = 0;
		if (fabs(b) - 1e-6 > 0) {
			r_ = a / b;
		}
		//std::cout << "r_ = " << r_ << std::endl;
		if (fabs(r_) >= rmin_vertical)
		{
			double d_x = x_0 - ParkingSpaceFlag * r_ * sin(theta) - x_1;
			double d_y = y_0 + ParkingSpaceFlag * r_ * cos(theta) - y_1 + ParkingSpaceFlag * delta_r;
			double t = .0, u = .0;
			u = -atan(fabs(d_x / d_y));
			t = u - ParkingSpaceFlag * theta;
			//std::cout << "t = " << t << " u = " << u << std::endl;
			if (t >= 0 && u <= 0 && r_ * t >= 0)
			{
				road_yaw[0] = ParkingSpaceFlag * r_;
				road_yaw[1] = r_ * t;
				road_yaw[2] = -ParkingSpaceFlag * (rmin_vertical + delta_r);
				road_yaw[3] = (rmin_vertical + delta_r) * u;
				road_yaw[4] = 1.0e6;
				road_yaw[5] = -d;
				if (GetOrCheckPathPoints(start, road_yaw, 3, 1))
				{
					PlanForwardFirst += 4;
					return GetOrCheckPathPoints(start, road_yaw, 3, 0);
				}
			}
		}
		else{
			// SCS-->CCS
			for (int i = 0; i < 5; i++)
			{
				double r_1 = rmin_vertical + 0.2 + i * 0.5;
				if (Calc_SpCmSm(x_0, y_0, theta, end[0], end[1], end[2], r_1, road_yaw) && (road_yaw[5] < -0.8) )
				{
					//std::cout << "slssls" << std::endl;
					if (GetOrCheckPathPoints(start, road_yaw, 3, 1))
					{
						//std::cout << "r_1 = " << r_1 << std::endl;
						PlanForwardFirst += 4;
						//return true;
						return GetOrCheckPathPoints(start, road_yaw, 3, 0);
					}
				}
			}

			// backward firstly then parking once;
			// modify vehicle heading
			double angle_modified = pi / 2 - fabs(start[2]);
			//std::cout << "angle_modified = " << angle_modified * 180 / pi << std::endl;
			double start_p[3] = {start[0], start[1], start[2]};
			if(angle_modified - 1e-6 < 0 && fabs(angle_modified) > 0.3 / rmin_vertical)
			{
				if(GetOnePathPoints(start_p, angle_modified, -ParkingSpaceFlag * rmin_vertical, 1))
				{
					start_p[0] = pathpoint.back().x;
					start_p[1] = pathpoint.back().y;
					start_p[2] = pathpoint.back().th;
				}
			}
			double angle_modified_length = (pathpoint.size() - 1) * 0.1;
			//std::cout << "angle_modified_length = " << angle_modified_length << std::endl;
			double backward_once_dist = (1.2 > angle_modified_length) ? (1.2 - angle_modified_length) : .0;
			//std::cout << "backward_once_dist = " << backward_once_dist << std::endl;
			if(true && GetOnePathPoints(start_p, -backward_once_dist, 1.0e6, 1))
			{
				double start_curr[3] = {pathpoint.back().x, pathpoint.back().y, pathpoint.back().th};
				x_0 = start_curr[0], y_0 = start_curr[1], theta = start_curr[2];
				a = x_0 * x_0 + y_0 * y_0 + x_1 * x_1 + y_1 * y_1 - 2 * x_0 * x_1 - 2 * y_0 * y_1 + 2 * ParkingSpaceFlag * y_0 * delta_r - 2 * ParkingSpaceFlag * y_1 * delta_r - rmin_vertical * rmin_vertical - 2 * rmin_vertical * delta_r;
				b = 2 * (rmin_vertical + delta_r - ParkingSpaceFlag * x_1 * sin(theta) + ParkingSpaceFlag * x_0 * sin(theta) - ParkingSpaceFlag * y_0 * cos(theta) + ParkingSpaceFlag * y_1 * cos(theta) - delta_r * cos(theta));
				if (fabs(b) - 1e-6 > 0) {
					r_ = a / b;
				}
				//std::cout << "r_ = " << r_ << std::endl;
				//return true;
				if (fabs(r_) >= rmin_vertical)
				{
					double d_x = x_0 - ParkingSpaceFlag * r_ * sin(theta) - x_1;
					double d_y = y_0 + ParkingSpaceFlag * r_ * cos(theta) - y_1 + ParkingSpaceFlag * delta_r;
					double t = .0, u = .0;
					u = -atan(fabs(d_x / d_y));
					t = u - ParkingSpaceFlag * theta;
					//std::cout << "t = " << t << " u = " << u << std::endl;
					if (t >= 0 && u <= 0 && r_ * t >= 0)
					{
						road_yaw[0] = ParkingSpaceFlag * r_;
						road_yaw[1] = r_ * t;
						road_yaw[2] = -ParkingSpaceFlag * (rmin_vertical + delta_r);
						road_yaw[3] = (rmin_vertical + delta_r) * u;
						road_yaw[4] = 1.0e6;
						road_yaw[5] = -d;
						if (GetOrCheckPathPoints(start_curr, road_yaw, 3, 1))
						{
							//PlanForwardFirst += 4;
							return GetOrCheckPathPoints(start_curr, road_yaw, 3, 0);
						}
					}
				}
			}
			
			int pp_size_now = pathpoint.size();
			for(int i = 0; i < pp_size_now - 1; i++)
			{
				pathpoint.pop_back();
			}
			

			// C+C+S-C-S-
			double dist = (d > length_of_straight) ? length_of_straight : d;
			double end_p[3] = {dist + rmin_vertical + delta_r, -ParkingSpaceFlag * (rmin_vertical + delta_r), -ParkingSpaceFlag * pi / 2};
			if(false && Calc_CpCpSm(start[0], start[1], start[2], end_p[0], end_p[1], end_p[2], rmin_vertical, road_yaw) == 0)
			{
				road_yaw[6] = -ParkingSpaceFlag * (rmin_vertical + delta_r);
				road_yaw[7] = -fabs(pi / 2 * road_yaw[6]);
				road_yaw[8] = 1e6;
				road_yaw[9] = -dist;
				if (GetOrCheckPathPoints(start, road_yaw, 5, 1))
				{
					//PlanForwardFirst += 4;
					return GetOrCheckPathPoints(start, road_yaw, 5, 0);
				}

			}
		}
		return false;
	}
	//
	//2023.03.23
	bool FindPathForwardDirecty(double start[3], double end[3])
	{
		double road_yaw[6] = { 0 };
		double start_p[3] = { start[0], start[1], start[2] };
		bool flag = (PlanForwardFirst == 0) 
					&& ((ParkingSpaceFlag == 1 && (dist_P23 == -1 || dist_P23 > 120)) 
					|| (ParkingSpaceFlag == -1 && (dist_P01 == -1 || dist_P01 > 120)));
		//
		if ((flag || fusion.parkingSpaceInfo.ParkingSpaceType == 3))  // && (fabs(start[2]) <= pi / 2)
		{
			if(Calc_SpCmSm(start_p[0], start_p[1], start_p[2], end[0], end[1], end[2], rmin_vertical, road_yaw))
			{
				double dist = std::max(length_of_straight, -road_yaw[5]);
				//std::cout << "dist = " << dist << std::endl;
				if(CalcOncePathVertical(start_p,end,dist))
				{
					return true;
				}
				// while road_yaw[1] < 1.2m
			}
		}
		return false;
	}
	//2023.03.23

	bool RearParking(double Start_[3], double End_[3])
	{
		double x_lim = P_r[0] + D_x;
		if(fusion.parkingSpaceInfo.ParkingSpaceType == 1 && dist_P03 > 0 && (dist_toward_P03 > 600 || dist_toward_P03 == -1))
		{
			P_r[0] = P_r[0] + dist_P03*0.01;
		}
		bool Pr_flag = ((ParkingSpaceFlag == 1 && (dist_P23 == -1 || dist_P23 > 100) && (dist_P01 == -1 || dist_P01 > 10))
			|| (ParkingSpaceFlag == -1 && (dist_P01 == -1 || dist_P01 > 100) && (dist_P23 == -1 || dist_P23 > 10)));
		if (fusion.parkingSpaceInfo.ParkingSpaceType == 1 && Pr_flag)
		{
			P_r[0] = P_r[0] - depth_no_obstacle;
		}

		if(index_request > 0 && index_request < 3)
		{
			if(P_r_x_first < P_r[0])
			{
				P_r_x_first = P_r[0];
			}
			P_r_y_first = P_r[1];
		}

		PathPoint current_point;
		current_point.x = Start_[0];
		current_point.y = Start_[1];
		current_point.th = Start_[2];
		current_point.D = 0;
		current_point.delta = 0;
		PlanForwardFail = PlanForwardFirst;
		// 2023.02.09
		if (PlanBackwardFirst < 3 && PlanForwardFirst < 3)
		{
			for (int i = 0; i < 5; i++)
			{
				pathpoint.clear();
				pathpoint.push_back(current_point);
				if (fusion.parkingSpaceInfo.ParkingSpaceType == 3 || Pr_flag)
				{
					Aim_dist = floor(100 * (aim_dist[4-i]- 0.05) * fabs(sin(park_theta))) / 100;
				}
				else {
					if(dist_toward_P03 > 550 || dist_toward_P03 == -1)
					{
						Aim_dist = floor(100 * aim_dist[4 - i] * fabs(sin(park_theta))) / 100;
					}
					else{
						Aim_dist = floor(100 * aim_dist[i] * fabs(sin(park_theta))) / 100;
					}
				}
				if (IsBackwardFirst(Start_) && ((PlanBackwardFirst >= 0) && (PlanForwardFail == 0)))
				{
					PlanBackwardFirst += 2;
					if (FindPathBackwardDirectly(Start_, End_) || FindPathBackwardFirst(Start_, End_))
					{
						return true;
					}
				}
				else
				{
					if ((FindPathForwardDirecty(Start_, End_)) || FindPathForwardFirst(Start_, End_, x_lim) || FindPathStraightForwardFirst(Start_, End_, x_lim))
					{
						return true;
					}
				}
			}
		}
		else
		{
			pathpoint.clear();
			pathpoint.push_back(current_point);
			int f_flag_multi = 1;
			/*
			if (FindPathDirectly(Start_, End_, 0))
			{
				return true;
			}
			else if (PlanForwardFirst == 4)
			{
				PlanBackwardFirst += 2;
				PlanForwardFirst = 0;
				std::cout << "bbbbbbbbbb\n";
				if (FindPathBackwardFirst(Start_, End_))
				{
					return true;
				}
			}
			*/
			if (FindPathMulti(Start_, End_, f_flag_multi)) //
			{
				PlanMulti++;
				return true;
			}
		}
		// 2023.02.09
		return false;
	}

}
