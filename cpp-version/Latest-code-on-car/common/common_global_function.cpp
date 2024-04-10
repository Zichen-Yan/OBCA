#include "global_variable.h"
#include "global_function.h"

namespace byd_apa_plan
{
	double mod2pi(double x)
	{
		double v = fmod(x, twopi);
		if (v < -pi)
			v += twopi;
		else if (v > pi)
			v -= twopi;
		return v;
	}

	void GetPrkVetical(double start[3], double b_depth, double park_theta)
	{
		double Px = 0, Py = 0;
		if (b_depth - 0.1 > 0.0)
		{
			double dist = (fusion.ParkInMode == 0) ? (b_depth - dist_to_block) : -(b_depth - vehicle_parameters.WB - dist_to_block);
			Px = (P0x + P3x) / 2.0 - dist * cos(park_theta);
			Py = (P0y + P3y) / 2.0 - dist * sin(park_theta);
			end_to_fus_before[0] = Px;
			end_to_fus_before[1] = Py;
			end_to_fus_before[2] = park_theta;
		}
		else
		{
			double dist = (fusion.ParkInMode == 0) ? vehicle_parameters.LB : -vehicle_parameters.LF;
			Px = (P1x + P2x) / 2 + dist * cos(park_theta);
			Py = (P1y + P2y) / 2 + dist * sin(park_theta);
			end_to_fus_before[0] = Px;
			end_to_fus_before[1] = Py;
			end_to_fus_before[2] = park_theta;
		}
		double delta_theta = mod2pi(end_to_fus_before[2] - start[2]);
		double bias_dist = (fusion.TraceParkingID_USS != 0) ? 0.03 : 0.05; //0.03 : 0.05
		double diffSE = ((start[0] - end_to_fus_before[0])*(start[0] - end_to_fus_before[0]) + (start[1] - end_to_fus_before[1])*(start[1] - end_to_fus_before[1])); // 距终点距离
				
		if ((fabs(delta_theta) > 10 * pi / 180) && (diffSE > 1.44))
		{
			end_to_fus[0] = end_to_fus_before[0] + (park_theta / fabs(park_theta)) * bias_dist * sin(park_theta);
			end_to_fus[1] = end_to_fus_before[1] - (park_theta / fabs(park_theta)) * bias_dist * cos(park_theta);
			end_to_fus[2] = end_to_fus_before[2];
			prk_bias = -(park_theta / fabs(park_theta)) * 0.1;
		}
		else
		{
			end_to_fus[0] = end_to_fus_before[0];
			end_to_fus[1] = end_to_fus_before[1];
			end_to_fus[2] = end_to_fus_before[2];
			prk_bias = .0;
		}
		start_to_fus[0] = start[0];
		start_to_fus[1] = start[1];
		start_to_fus[2] = start[2];
	}
	void GetPrkLevel(double start[3], double b_depth, double park_theta)
	{
		// 2022.12.07:水平车位终点朝P03边外移0.15m
		double CarL = (vehicle_parameters.LB + vehicle_parameters.LF) / 2 - vehicle_parameters.LB;
		double end_bias = 0.0; //0.15
		double P_mean_03x = (P0x + P3x) / 2;
		double P_mean_03y = (P0y + P3y) / 2;
		double P_mean_x = (P0x + P1x + P2x + P3x) / 4;
		double P_mean_y = (P0y + P1y + P2y + P3y) / 4;
		double dist_P03 = sqrt((P_mean_03x - P_mean_x) * (P_mean_03x - P_mean_x) + (P_mean_03y - P_mean_y) * (P_mean_03y - P_mean_y));
		double cos_P03 = (P_mean_x - P_mean_03x) / dist_P03;
		double sin_P03 = (P_mean_y - P_mean_03y) / dist_P03;
		P_mean_x = P_mean_x - end_bias * cos_P03;
		P_mean_y = P_mean_y - end_bias * sin_P03;
		double Px = P_mean_x - CarL * cos(park_theta);
		double Py = P_mean_y - CarL * sin(park_theta);
		end_to_fus[0] = Px;
		end_to_fus[1] = Py;
		end_to_fus[2] = park_theta;
		start_to_fus[0] = start[0];
		start_to_fus[1] = start[1];
		start_to_fus[2] = start[2];
		if (app.APA_Park_Function == 2)
		{
			if (end_to_fus[1] > 0)
			{
				end_to_fus[0] = 2.55;
				end_to_fus[1] = 1.1;
				end_to_fus[2] = park_theta;
			}
			else
			{
				end_to_fus[0] = 2.55;
				end_to_fus[1] = -1.1;
				end_to_fus[2] = park_theta;
			}
		}
		// 2022.12.07
	}
	void GetPrkInclined(double start[3], const double& b_depth, const double& park_theta)
	{
		if (b_depth - 0.1 > 0.0)
		{
			double dist = (fusion.ParkInMode == 0) ? (b_depth - dist_to_block) : -(b_depth - vehicle_parameters.WB - dist_to_block);
			double Px = 0;
			double Py = 0;
			double Px_b_depth = (P0x + P3x) / 2.0 - dist * cos(park_theta);
			double Py_b_depth = (P0y + P3y) / 2.0 - dist * sin(park_theta);
			Px = Px_b_depth;
			Py = Py_b_depth;
			end_to_fus_before[0] = Px;
			end_to_fus_before[1] = Py;
			end_to_fus_before[2] = park_theta;
		}
		else
		{
			double x, y, phi;
			double dist_02 = sqrt((P0x - P2x) * (P0x - P2x) + (P0y - P2y) * (P0y - P2y));
			double dist_13 = sqrt((P1x - P3x) * (P1x - P3x) + (P1y - P3y) * (P1y - P3y));
			if (dist_02 < dist_13)
			{
				CoordinateTrans(x, y, phi, P0x, P0y, 0.0, P2x, P2y, park_theta);
				double P1x_img_ = 0;
				double P1y_img_ = y;
				double P3x_img_ = x;
				double P3y_img_ = 0;

				double P1x_img = P1x_img_ * cos(park_theta) - P1y_img_ * sin(park_theta) + P2x;
				double P1y_img = P1y_img_ * cos(park_theta) + P1x_img_ * sin(park_theta) + P2y;
				double P3x_img = P3x_img_ * cos(park_theta) - P3y_img_ * sin(park_theta) + P2x;
				double P3y_img = P3y_img_ * cos(park_theta) + P3x_img_ * sin(park_theta) + P2y;

				double D_imgP2P3 = sqrt((P3x_img - P2x) * (P3x_img - P2x) + (P3y_img - P2y) * (P3y_img - P2y));
				if (D_imgP2P3 >= 5)
				{
					double dist = (fusion.ParkInMode == 0) ? vehicle_parameters.LF : -vehicle_parameters.LB;
					double Px = (P3x_img + P0x) / 2 - dist * cos(park_theta);
					double Py = (P3y_img + P0y) / 2 - dist * sin(park_theta);
					end_to_fus_before[0] = Px;
					end_to_fus_before[1] = Py;
					end_to_fus_before[2] = park_theta;
				}
				else
				{
					double dist = (fusion.ParkInMode == 0) ? vehicle_parameters.LB : -vehicle_parameters.LF;
					double Px = (P1x_img + P2x) / 2 + dist * cos(park_theta);
					double Py = (P1y_img + P2y) / 2 + dist * sin(park_theta);
					end_to_fus_before[0] = Px;
					end_to_fus_before[1] = Py;
					end_to_fus_before[2] = park_theta;
				}
			}
			else
			{
				CoordinateTrans(x, y, phi, P3x, P3y, 0.0, P1x, P1y, park_theta);
				double P2x_img_ = 0;
				double P2y_img_ = y;
				double P0x_img_ = x;
				double P0y_img_ = 0;

				double P2x_img = P2x_img_ * cos(park_theta) - P2y_img_ * sin(park_theta) + P1x;
				double P2y_img = P2y_img_ * cos(park_theta) + P2x_img_ * sin(park_theta) + P1y;
				double P0x_img = P0x_img_ * cos(park_theta) - P0y_img_ * sin(park_theta) + P1x;
				double P0y_img = P0y_img_ * cos(park_theta) + P0x_img_ * sin(park_theta) + P1y;

				double D_imgP2P3 = sqrt((P0x_img - P1x) * (P0x_img - P1x) + (P0y_img - P1y) * (P0y_img - P1y));
				if (D_imgP2P3 >= 5)
				{
					double dist = (fusion.ParkInMode == 0) ? vehicle_parameters.LF : -vehicle_parameters.LB;
					double Px = (P0x_img + P3x) / 2 - dist * cos(park_theta);
					double Py = (P0y_img + P3y) / 2 - dist * sin(park_theta);
					end_to_fus_before[0] = Px;
					end_to_fus_before[1] = Py;
					end_to_fus_before[2] = park_theta;
				}
				else
				{
					double dist = (fusion.ParkInMode == 0) ? vehicle_parameters.LB : -vehicle_parameters.LF;
					double Px = (P2x_img + P1x) / 2 + dist * cos(park_theta);
					double Py = (P2y_img + P1y) / 2 + dist * sin(park_theta);
					end_to_fus_before[0] = Px;
					end_to_fus_before[1] = Py;
					end_to_fus_before[2] = park_theta;
				}
			}
		}
		double delta_theta = mod2pi(end_to_fus_before[2] - start[2]);
		double bias_dist = (fusion.TraceParkingID_USS != 0) ? 0.03 : 0.05; // 0.03 : 0.05
		double diffSE = ((start[0] - end_to_fus_before[0])*(start[0] - end_to_fus_before[0]) + (start[1] - end_to_fus_before[1])*(start[1] - end_to_fus_before[1])); // 距终点距离
		if ((fabs(delta_theta) > 10 * pi / 180) && (diffSE > 1.44))
		{
			bias_dist = (fusion.ParkInMode == 0) ? bias_dist : -bias_dist;
			end_to_fus[0] = end_to_fus_before[0] + (park_theta / fabs(park_theta)) * bias_dist * sin(park_theta);
			end_to_fus[1] = end_to_fus_before[1] - (park_theta / fabs(park_theta)) * bias_dist * cos(park_theta);
			end_to_fus[2] = end_to_fus_before[2];
			prk_bias = -(park_theta / fabs(park_theta)) * 0.1;
		}
		else
		{
			end_to_fus[0] = end_to_fus_before[0];
			end_to_fus[1] = end_to_fus_before[1];
			end_to_fus[2] = end_to_fus_before[2];
			prk_bias = .0;
		}
		start_to_fus[0] = start[0];
		start_to_fus[1] = start[1];
		start_to_fus[2] = start[2];
	}
	int GetStartAndEndPoint()
	{
		if (control.ObsUssInfo == 20u)
		{
			ccc = ccc + 1;
			if (fusion.ParkInMode == 0)
			{
				ctrl_nav_x = calculation.nav_pos_X * 0.01;
				ctrl_nav_y = calculation.nav_pos_Y * 0.01;
			}
			else
			{
				double ctrl_nav_th = mod2pi(calculation.nav_heading * pi / 180);
				ctrl_nav_x = vehicle_parameters.WB * cos(ctrl_nav_th) - 0 * sin(ctrl_nav_th) + calculation.nav_pos_X * 0.01;
				ctrl_nav_y = 0 * cos(ctrl_nav_th) + vehicle_parameters.WB * sin(ctrl_nav_th) + calculation.nav_pos_Y * 0.01;
			}
			control.ObsUssInfo = 21u;
		}
		if (((fusion.position.Heading >= 0) && (fusion.position.Heading <= 360)) && ((calculation.nav_heading >= 0) && (calculation.nav_heading <= 360)))
		{
			//
			double x, y, phi;
			double start[3] = { .0,.0,.0 };
			//double end[3] = {.0,.0,.0};

			double fus_theta = mod2pi(fusion.position.Heading * pi / 180);  //  坐标系转换弧度
			double cal_theta = mod2pi(calculation.nav_heading * pi / 180);  //  坐标系转换弧度
			CoordinateTrans(x, y, phi, calculation.nav_pos_X, calculation.nav_pos_Y, cal_theta, fusion.position.X, fusion.position.Y, fus_theta);
			start[0] = x / 100.0;
			start[1] = y / 100.0;
			start[2] = phi;
			//
			CoordinateTrans(x, y, phi, fusion.parkingSpaceInfo.P0_X, fusion.parkingSpaceInfo.P0_Y, 0.0, fusion.position.X, fusion.position.Y, fus_theta);
			P0x = x / 100.0;
			P0y = y / 100.0;
			CoordinateTrans(x, y, phi, fusion.parkingSpaceInfo.P1_X, fusion.parkingSpaceInfo.P1_Y, 0.0, fusion.position.X, fusion.position.Y, fus_theta);
			P1x = x / 100.0;
			P1y = y / 100.0;
			CoordinateTrans(x, y, phi, fusion.parkingSpaceInfo.P2_X, fusion.parkingSpaceInfo.P2_Y, 0.0, fusion.position.X, fusion.position.Y, fus_theta);
			P2x = x / 100.0;
			P2y = y / 100.0;
			CoordinateTrans(x, y, phi, fusion.parkingSpaceInfo.P3_X, fusion.parkingSpaceInfo.P3_Y, 0.0, fusion.position.X, fusion.position.Y, fus_theta);
			P3x = x / 100.0;
			P3y = y / 100.0;

			//
			//double park_theta = 0;
			if (fusion.ParkInMode == 1)
			{
				park_theta = fusion.Theta + 180;
			}
			else
			{
				park_theta = fusion.Theta;
			}
			double park_theta_block = mod2pi(park_theta * pi / 180);
			park_theta = mod2pi(park_theta * pi / 180) - fus_theta;
			park_theta = mod2pi(park_theta);
			if (ccc > 0)
			{
				if (fusion.parkingSpaceInfo.ParkingSpaceType == 1)
				{
					double ctrl_x0 = fusion.parkingSpaceInfo.P0_X * 0.01;
					double ctrl_y0 = fusion.parkingSpaceInfo.P0_Y * 0.01;
					double ctrl_x3 = fusion.parkingSpaceInfo.P3_X * 0.01;
					double ctrl_y3 = fusion.parkingSpaceInfo.P3_Y * 0.01;
					if (fabs(ctrl_y3 - ctrl_y0) > 0.0001 || fabs(ctrl_x0 - ctrl_x3) > 0.0001)
					{
						b_depth = dist_to_block + fabs(((ctrl_y3 - ctrl_y0) * ctrl_nav_x + (ctrl_x0 - ctrl_x3) * ctrl_nav_y - ctrl_x0 * ctrl_y3 + ctrl_y0 * ctrl_x3) / sqrt((ctrl_y3 - ctrl_y0) * (ctrl_y3 - ctrl_y0) + (ctrl_x0 - ctrl_x3) * (ctrl_x0 - ctrl_x3)));
					}
					else
					{
						b_depth = fusion.depth_block / 100.0;
					}
				}
				else if (fusion.parkingSpaceInfo.ParkingSpaceType == 3)
				{
					double ctrl_x0 = (fusion.parkingSpaceInfo.P0_X + fusion.parkingSpaceInfo.P3_X) / 2 * 0.01;
					double ctrl_y0 = (fusion.parkingSpaceInfo.P0_Y + fusion.parkingSpaceInfo.P3_Y) / 2 * 0.01;
					double ctrl_x3 = ctrl_x0 + cos(park_theta_block + pi / 2);
					double ctrl_y3 = ctrl_y0 + sin(park_theta_block + pi / 2);
					if (fabs(ctrl_y3 - ctrl_y0) > 0.0001 || fabs(ctrl_x0 - ctrl_x3) > 0.0001)
					{
						b_depth = dist_to_block + fabs(((ctrl_y3 - ctrl_y0) * ctrl_nav_x + (ctrl_x0 - ctrl_x3) * ctrl_nav_y - ctrl_x0 * ctrl_y3 + ctrl_y0 * ctrl_x3) / sqrt((ctrl_y3 - ctrl_y0) * (ctrl_y3 - ctrl_y0) + (ctrl_x0 - ctrl_x3) * (ctrl_x0 - ctrl_x3)));
					}
					else
					{
						b_depth = fusion.depth_block / 100.0;
					}
				}
			}
			else
			{
				b_depth = fusion.depth_block / 100.0; // fusion.depth_block 单位cm
			}
			//垂直车位
			if (fusion.parkingSpaceInfo.ParkingSpaceType == 1)
			{
				GetPrkVetical(start, b_depth, park_theta);
			}
			else if (fusion.parkingSpaceInfo.ParkingSpaceType == 2)
			{
				GetPrkLevel(start, b_depth, park_theta);
			}
			else
			{
				GetPrkInclined(start, b_depth, park_theta);
			}

			if ((fabs(start_to_fus[0]) < pathfind_parameters.MAXX) && (fabs(start_to_fus[1]) < pathfind_parameters.MAXY) 
				&& (fabs(end_to_fus[0]) < pathfind_parameters.MAXX) && (fabs(end_to_fus[1]) < pathfind_parameters.MAXY))
			{
				return 0;
			}
			else
			{
				PlanOnlyOne = 0;
				return 1;
			}
		}
		else
		{
			PlanOnlyOne = 0;
			return 1;
		}
	}

	int IsPathOnlyOne()
	{
		int pp_size = pathpoint.size();
		for (int idx = 0; idx < pp_size-1; idx++)
		{
			if(fusion.ParkInMode == 1)
			{
				if(plan.coordinate[idx].rajectoryDirection == 2)
				{
					return 0;
				}
			}
			else
			{
				if(plan.coordinate[idx].rajectoryDirection == 1)
				{
					return 0;
				}
			}
		}
		return 1;
	}

	void ParameterInit()
	{
		/*
		// Motion resolution define 运动分辨率定义
		pathfind_parameters.MOTION_RESOLUTION = 0.1; //[m] path interporate resolution 路径插值分辨率
		pathfind_parameters.N_STEER = 1.0; //20.0; % number of steer command 转向指令数
		pathfind_parameters.EXTEND_AREA = 0;//[m] map extend length 地图延伸长度
		pathfind_parameters.XY_GRID_RESOLUTION = 0.1;
		pathfind_parameters.YAW_GRID_RESOLUTION = 3;

		//Grid bound 网格边界
		pathfind_parameters.MINX = -12.5;
		pathfind_parameters.MAXX = 12.5;
		pathfind_parameters.MINY = -12.5;
		pathfind_parameters.MAXY = 12.5;
		pathfind_parameters.MINYAW = -3.141592653589793;
		pathfind_parameters.MAXYAW = 3.141592653589793;

		//Cost related define 成本相关定义
		pathfind_parameters.SB_COST = 2.0; // % 0.9 switch back penalty cost 切换回惩罚成本
		pathfind_parameters.BACK_COST = 1; // 1; %1.5; % backward penalty cost 反向惩罚成本
		pathfind_parameters.STEER_CHANGE_COST = 0.9; // 0; %1.5; % steer angle change penalty cost 转向角改变惩罚成本
		pathfind_parameters.STEER_COST = 0; // steer angle change penalty cost 转向角惩罚成本
		pathfind_parameters.H_COST = 2; // Heuristic cost 启发式成本

		//车辆参数
		vehicle_parameters.WB = 2.92;  //[m] wheel base : rear to front steer 轴距
		vehicle_parameters.W = 1.92; //[m] width of vehicle 车宽
		vehicle_parameters.LF = 3.84; //[m] distance from rear to vehicle front end of vehicle 后轴中心到车辆最前端的距离
		vehicle_parameters.LB = 0.99; //[m] distance from rear to vehicle back end of vehicle 后轴中心到车辆最后端的距离
		vehicle_parameters.MAX_STEER = 0.4806; // 0.4881; //[rad] maximum steering angle 车辆轮胎最大转角
		*/
		vehicle_parameters.MIN_CIRCLE = vehicle_parameters.WB / tan(vehicle_parameters.MAX_STEER); //[m] mininum steering circle radius 车辆最小转弯半径 汉_5.5

		//角度设置
		find_steer_degree.clear();
		find_steer_degree.push_back(-vehicle_parameters.MAX_STEER);
		find_steer_degree.push_back(0);
		find_steer_degree.push_back(vehicle_parameters.MAX_STEER);

		pathfind_parameters.XIDX = ceil((pathfind_parameters.MAXX - pathfind_parameters.MINX) / pathfind_parameters.XY_GRID_RESOLUTION);
		pathfind_parameters.XIDY = ceil((pathfind_parameters.MAXY - pathfind_parameters.MINY) / pathfind_parameters.XY_GRID_RESOLUTION);
		pathfind_parameters.MAX_IDX = (int)pathfind_parameters.XIDX * pathfind_parameters.XIDY;
	}

	void IsParkingCompleted()
	{
		double Complete_x = 0.3;
		double Complete_y = 0.15;
		double Complete_theta = pi / 180 * 2;
		if (plan_request == 1u)
		{
			if (app.APA_Park_Function == 2)
			{
				Complete_x = 5;
				Complete_y = 0.8;
				Complete_theta = pi / 180 * 5;
			}
			else if (fusion.parkingSpaceInfo.ParkingSpaceType == 3)
			{
				Complete_x = 0.5;
			}
			else if (fusion.parkingSpaceInfo.ParkingSpaceType == 2)
			{
				Complete_x = 0.2;
				Complete_y = 0.25;
				if (fusion.TraceParkingID_USS != 0u)
				{
					Complete_x = 0.15;
					Complete_theta = pi / 180 * 1.01;
				}
			}
			if ((fabs(Start[0]) < Complete_x) && (fabs(Start[1]) < Complete_y) && (fabs(Start[2]) < Complete_theta))
			{
				plan.IsPlanningCompleted = 1;
				plan.TrajectoryLength = 0;
				for (int i = 0; i < 300; i++)
				{
					plan.coordinate[i].X = .0;
					plan.coordinate[i].Y = .0;
					plan.coordinate[i].Yaw = .0;
					plan.coordinate[i].Curvature = .0;
					plan.coordinate[i].rajectoryDirection = 0;
				}
			}
			else
			{
				plan.IsPlanningCompleted = 0;
			}
		}
		else
		{
			plan.IsPlanningCompleted = 0;
		}
	}

	void StateClear()
	{
		pathpoint.clear();
		if ((app.APAStatus == 0u) || (app.APAStatus == 1u) || (app.APAStatus == 4u) || (app.APAStatus == 5u) || (app.APAStatus == 6u) || (app.APAStatus == 7u))
		{
			Gears = 0;
			index_request = 0;
			ParkingSpaceFlag = 0; // 2023.03.06
			PlanBackFirst = 0;
			PlanForwardFirst = 0;
			PlanMulti = 0;

			PlanOnlyOne = 0;
			PathOnlyOneNow = 0;
			StartRecordFlag = 0;
			prk_inclined_dynamic_plan = false;
			// 2022.12.06:增加全局变量清空
			Start_last[0] = .0;
			Start_last[1] = .0;
			Start_last[2] = .0;

			d_s2s = .0;
			d_s2e = .0;
			d_phi = .0;
			dynamic_break = 0;
			// 2022.12.06
			//2023.03.06
			dist_P01 = -1;
			dist_P23 = -1;
			dist_P03 = -1;
			dist_toward_P03 = -1;
			//2023.03.06

			plan_request = 0u;
			//fusion.TraceParkingID_Cam = 0u;
			//fusion.TraceParkingID_USS = 0u;
			fus_end_last[0] = 0;
			fus_end_last[1] = 0;
			fus_end_last[2] = 0;
			//fusion.ParkInMode = 0;

			ccc = 0;
			control.ObsUssInfo = 255u;
			ctrl_nav_x = 0;
			ctrl_nav_y = 0;
			right_left = 0;

			Start_out[0] = .0;
			Start_out[1] = .0;
			Start_out[2] = .0;

			index_30 = 0;
			index_level_out = 0;
			choice_cc = 0;
			dynamic_plan_flag = false;
			straight_front_flag = false;
			dynamic_final_end_flag = false;

			AstarOrGeo = 0;
			small_level_park = 0;
			/// 2024.1.11 ///
			Path_2 = 0; //
			// bias_th = .0;
			// end_init[0] = .0;
			// end_init[1] = .0;
			// end_init[2] = .0;
			rk_sto[0] = 0; 
			rk_sto[1] = 0;
			rk_sto[2] = 0;
			/// 2024.1.11 ///
		}
	}

	void PathPointNormalization()
	{
		//
		fus_end_last[0] = end_to_fus[0];
		fus_end_last[1] = end_to_fus[1];
		fus_end_last[2] = end_to_fus[2];
		//
		for (int i = 0; i < 300; i++)
		{
			plan.coordinate[i].X = 0;
			plan.coordinate[i].Y = 0;
			plan.coordinate[i].Yaw = 0;
			plan.coordinate[i].rajectoryDirection = 0;
			plan.coordinate[i].Curvature = 0;
		}
		//
		int pp_size_now = pathpoint.size();
		pp_size_now = (pp_size_now < 300) ? pp_size_now : 300;
		if(pp_size_now < 2)
		{
			plan.PlanningStatus = 2;
			return;
		}
		plan.TrajectoryLength = (pp_size_now - 1) * 10;
		double theta = .0;
		double curve = .0;
		for(int idx = 0; idx < pp_size_now; idx++)
		{
			plan.coordinate[idx].X = pathpoint[idx].x * 100;
			plan.coordinate[idx].Y = pathpoint[idx].y * 100;
			theta = (pathpoint[idx].th < 0) ? (pathpoint[idx].th + twopi) : (pathpoint[idx].th);
			plan.coordinate[idx].Yaw = theta * 180 / pi;
			curve = (pathpoint[idx].delta == 0) ? 1/2500 : tan(pathpoint[idx].delta) / (vehicle_parameters.WB * 100);
			if(SEorES == 1)
			{
				plan.coordinate[idx].rajectoryDirection = (pathpoint[idx].D > 0) ? 2 : 1;
				if(idx < pp_size_now - 1)
				{
					plan.coordinate[idx+1].Curvature = curve;
				}
			}
			else
			{
				if(idx > 0)
				{
					plan.coordinate[idx - 1].rajectoryDirection = (pathpoint[idx].D > 0) ? 1 : 2;
				}
				plan.coordinate[idx].Curvature = curve;
			}
		}
		plan.coordinate[0].Curvature = plan.coordinate[1].Curvature;
		plan.coordinate[pp_size_now - 1].rajectoryDirection = (plan.coordinate[pp_size_now - 2].rajectoryDirection == 1) ? 2 : 1;

		Gears = plan.coordinate[0].rajectoryDirection;
		plan.PlanningStatus = 1;
	}

	void VehicleDynamic(double x, double y, double theta, double D, double delta) //根据当前位姿和输入, 计算下一位置的位姿
	{
		double mid_R;

		if (delta == 0)
		{
			g_px = x + D * cos(theta); // 运动学公式： x_dot = v_x * cos(theta); x_dot * t = v_x * t * cos(theta), 在采样时间t内, 则有x = x + v_x * t * cos(theta)，其中v_x * t = D
			g_py = y + D * sin(theta); // 运动学公式
			g_pth = theta + D / vehicle_parameters.WB * tan(delta); // L是轴距, 航向变化, theta_dot = v / R, R = L / tan(delta)
			g_pth = mod2pi(g_pth);
		}
		else
		{
			mid_R = vehicle_parameters.WB / tan(delta);
			g_px = x - mid_R * sin(theta) + mid_R * sin(theta + D / mid_R); // 运动学公式： x_dot = v_x * cos(theta); x_dot * t = v_x * t * cos(theta), 在采样时间t内, 则有x = x + v_x * t * cos(theta)，其中v_x * t = D
			g_py = y + mid_R * cos(theta) - mid_R * cos(theta + D / mid_R); // 运动学公式
			g_pth = theta + D / mid_R; // L是轴距, 航向变化, theta_dot = v / R, R = L / tan(delta)
			g_pth = mod2pi(g_pth);
		}
	}
}
