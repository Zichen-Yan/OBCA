#include "../common/global_variable.h"
#include "../common/global_function.h"

namespace byd_apa_plan
{
	char road_find_five(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_yaw[8], double road_yaw2[8])
	{
		//////////////////////////////车辆坐标转换处理/////////////////////////////////////////////////////////////////////
		double ph = mod2pi(theta_aim - theta_now);
		double phi = mod2pi(theta_now);
		// 起点start坐标系在基坐标系下的方向余弦矩阵(Z轴旋转，因为点在Z = 0平面，因此绕Z旋转仍然在Z = 0平面)
		// dcm*x 表示将基坐标中的x表示到旋转后的坐标系中，即计算坐标旋转后各向量在新坐标中的表示
		double x1 = (x_aim - x_now)*cos(phi) + (y_aim - y_now)*sin(phi);
		double y1 = (y_aim - y_now)*cos(phi) - (x_aim - x_now)*sin(phi);
		if (fabs(ph) < 45 * pi / 180)
		{
			RsPath path_five;
			double x1_five = x1 / car_r_limit;
			double y1_five = y1 / car_r_limit;
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
			//LOG_WARNING("Five_theta_car=%d", theta_car);
			//fflush(bydapa::common::Log::GetInstance()->fileptr());
			ErrorCode = CpCpSm(x1_five, y1_five, ph, path_five, theta_car);
			if (ErrorCode == 1)
			{
				for (int index_five_CCS = 0;index_five_CCS < 3;index_five_CCS++)
				{
					if (path_five.rspath_type[index_five_CCS] == St)
					{
						road_yaw[index_five_CCS * 2] = 1000000;
					}

					else if (path_five.rspath_type[index_five_CCS] == Le)
					{
						road_yaw[index_five_CCS * 2] = car_r_limit;
					}

					else if (path_five.rspath_type[index_five_CCS] == Ri)
					{
						road_yaw[index_five_CCS * 2] = -car_r_limit;
					}
				}

				road_yaw[1] = path_five.t * car_r_limit;
				road_yaw[3] = path_five.u * car_r_limit;
				road_yaw[5] = path_five.v * car_r_limit;
				path_five.lenth = path_five.lenth * car_r_limit;
				if (fabs(road_yaw[5]) > 6)
				{
					return 2;
				}

			}
			else
			{
				return 2;
			}

			double x_midpoint = 3.7;
			double y_midpoint = 0;
			double theta_midpoint = 0;

			double x = x_midpoint * cos(ph) - y_midpoint * sin(ph) + x1;
			double y = y_midpoint * cos(ph) + x_midpoint * sin(ph) + y1;
			double theta = mod2pi(ph - theta_midpoint);

			double R_s = 1000000;
			double L_s = -x_midpoint * theta_car;

			double road_yaw_R[8] = { 0,0,0,0,0,0,0,0 };
			int roadthird_five = 1;
			roadthird_five = Calc_CmCm(0, 0, 0, x, y, theta, rmin_vertical, road_yaw_R);
			if ((roadthird_five == 0) && (fabs(road_yaw_R[1] + road_yaw_R[3]) > 0.3) && (fabs(road_yaw[1] + road_yaw[3]) > 0.3))
			{
				road_yaw2[0] = road_yaw_R[0];
				road_yaw2[1] = road_yaw_R[1];
				road_yaw2[2] = road_yaw_R[2];
				road_yaw2[3] = road_yaw_R[3];
				road_yaw2[4] = R_s;
				road_yaw2[5] = L_s;
				return 1;
			}
			else if (fabs(road_yaw[1] + road_yaw[3]) > 0.3)
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
	char road_find_seven(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, double car_r_limit, double road_yaw[8], double road_yaw2[8])
	{
		//////////////////////////////车辆坐标转换处理/////////////////////////////////////////////////////////////////////
		double ph = mod2pi(theta_aim - theta_now);
		double phi = mod2pi(theta_now);
		// 起点start坐标系在基坐标系下的方向余弦矩阵(Z轴旋转，因为点在Z = 0平面，因此绕Z旋转仍然在Z = 0平面)
		// dcm*x 表示将基坐标中的x表示到旋转后的坐标系中，即计算坐标旋转后各向量在新坐标中的表示
		double x1 = (x_aim - x_now)*cos(phi) + (y_aim - y_now)*sin(phi);
		double y1 = (y_aim - y_now)*cos(phi) - (x_aim - x_now)*sin(phi);

		RsPath path_seven;
		double x1_seven = x1 / car_r_limit;
		double y1_seven = y1 / car_r_limit;
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
		ErrorCode = CmCpSm(x1_seven, y1_seven, ph, path_seven, theta_car);
		if (ErrorCode == 1)
		{
			for (int index_seven_CCS = 0;index_seven_CCS < 3;index_seven_CCS++)
			{
				if (path_seven.rspath_type[index_seven_CCS] == St)
				{
					road_yaw[index_seven_CCS * 2] = 1000000;
				}

				else if (path_seven.rspath_type[index_seven_CCS] == Le)
				{
					road_yaw[index_seven_CCS * 2] = car_r_limit;
				}

				else if (path_seven.rspath_type[index_seven_CCS] == Ri)
				{
					road_yaw[index_seven_CCS * 2] = -car_r_limit;
				}
			}

			road_yaw[1] = path_seven.t * car_r_limit;
			road_yaw[3] = path_seven.u * car_r_limit;
			road_yaw[5] = path_seven.v * car_r_limit;
			path_seven.lenth = path_seven.lenth * car_r_limit;
			if (fabs(road_yaw[5]) > 8)
			{
				return 2;
			}
		}
		else
		{
			return 2;
		}
		/////////////////////////////////////////////////////////////////////////////////////
		double delta_seven1 = 0;
		if (fabs(road_yaw[0]) > 100000 || fabs(road_yaw[0]) < rmin_vertical*0.5)
		{
			delta_seven1 = 0;
		}
		else
		{
			delta_seven1 = atan(vehicle_parameters.WB / road_yaw[0]);
		}

		int roadsecond_seven = 1;
		double start_imag[3] = { 0,0,0 };
		start_imag[0] = x_now;
		start_imag[1] = y_now;
		start_imag[2] = theta_now;
		roadsecond_seven = GetOrCheckPathPoints(start_imag, road_yaw, 4, 1);
		if ((roadsecond_seven == 1) && (fabs(road_yaw[1]) > 0.3))
		{
			return 0;
		}
		else
		{

			double D_seven = round(fabs(road_yaw[1]) * 10);
			if (D_seven > 10)
			{
				D_seven = 10;
			}

			// 遍历第一段CCS
			for (int index_D_seven = D_seven; index_D_seven > 0; index_D_seven--)
			{
				double g_x = 0;
				double g_y = 0;
				double g_th = 0;
				VehicleDynamic(g_x, g_y, g_th, road_yaw[1] + theta_car * index_D_seven * 0.1, delta_seven1);

				double ph_seven = ph - g_pth;
				double phi_seven = mod2pi(g_pth);
				// 起点start坐标系在基坐标系下的方向余弦矩阵(Z轴旋转，因为点在Z = 0平面，因此绕Z旋转仍然在Z = 0平面)
				// dcm*x 表示将基坐标中的x表示到旋转后的坐标系中，即计算坐标旋转后各向量在新坐标中的表示
				double x1_seven = (x1 - g_px)*cos(phi_seven) + (y1 - g_py)*sin(phi_seven);
				double y1_seven = (y1 - g_py)*cos(phi_seven) - (x1 - g_px)*sin(phi_seven);
				double x1_seven_six = x1_seven / car_r_limit;
				double y1_seven_six = y1_seven / car_r_limit;

				double road_yaw_sevenimag[8] = { 0,0,0,0,0,0,0,0 };
				ErrorCode = CpCmSm(x1_seven_six, y1_seven_six, ph_seven, path_seven, theta_car);

				if (ErrorCode == 1)
				{
					road_yaw_sevenimag[0] = road_yaw[0];
					road_yaw_sevenimag[1] = road_yaw[1] + theta_car * index_D_seven * 0.1;
					for (int index_seven_CCS = 0;index_seven_CCS < 3;index_seven_CCS++)
					{
						if (path_seven.rspath_type[index_seven_CCS] == St)
						{
							road_yaw_sevenimag[(index_seven_CCS + 1) * 2] = 1000000;
						}

						else if (path_seven.rspath_type[index_seven_CCS] == Le)
						{
							road_yaw_sevenimag[(index_seven_CCS + 1) * 2] = car_r_limit;
						}

						else if (path_seven.rspath_type[index_seven_CCS] == Ri)
						{
							road_yaw_sevenimag[(index_seven_CCS + 1) * 2] = -car_r_limit;
						}
					}

					road_yaw_sevenimag[3] = path_seven.t * car_r_limit;
					road_yaw_sevenimag[5] = path_seven.u * car_r_limit;
					road_yaw_sevenimag[7] = path_seven.v * car_r_limit;
					path_seven.lenth = path_seven.lenth * car_r_limit;
				}
				roadsecond_seven = GetOrCheckPathPoints(start_imag, road_yaw_sevenimag, 4, 1);
				if (roadsecond_seven == 1)
				{
					road_yaw2[0] = road_yaw_sevenimag[0];
					road_yaw2[1] = road_yaw_sevenimag[1];
					road_yaw2[2] = road_yaw_sevenimag[2];
					road_yaw2[3] = road_yaw_sevenimag[3];
					road_yaw2[4] = road_yaw_sevenimag[4];
					road_yaw2[5] = road_yaw_sevenimag[5];
					road_yaw2[6] = road_yaw_sevenimag[6];
					road_yaw2[7] = road_yaw_sevenimag[7];
					if (fabs(road_yaw2[1]) > 0.3)
					{
						return 1;
					}
				}
			}
		}
		return 2;
	}

	bool  RearParkingDirectly(double start[3], double end[3])
	{
		pathpoint.clear();
		path_point current_point;
		current_point.x = start[0];
		current_point.y = start[1];
		current_point.th = start[2];
		current_point.D = 0;
		current_point.delta = 0;
		pathpoint.push_back(current_point);
		
		double road_yaw[8] = {.0} ;
		double road_yaw2[8] = {.0};

		int theta_car_dynamic = 0;
		if (fusion.ParkInMode == 1)
		{
			theta_car_dynamic = -1;
		}
		else
		{
			theta_car_dynamic = 1;
		}
		//C-
		int roadfirst = 1;
		roadfirst = Calc_Cm(start[0], start[1], start[2], end[0], end[1], end[2], 0.05, rmin_vertical, 0.3, road_yaw);
		if (roadfirst == 0)
		{
			if (GetOrCheckPathPoints(start, road_yaw, 4, 1))
			{
				return GetOrCheckPathPoints(start, road_yaw, 4, 0);
			}
		}
		//C-S-
		int roadsecond = 1;
		roadsecond = Calc_CmSm(start[0], start[1], start[2] * 180 / pi, end[0], end[1], end[2] * 180 / pi, rmin_vertical, 0.03, road_yaw);
		if (roadsecond == 0)
		{
			if (GetOrCheckPathPoints(start, road_yaw, 4, 1))
			{
				return GetOrCheckPathPoints(start, road_yaw, 4, 0);
			}
		}
		//C-C-
		if ((fabs(end[2] - start[2]) < 5*pi/180) && (3 - Gears * 2 == 1 * theta_car_dynamic))
		{
			int roadthird = 1;
			roadthird = Calc_CmCm(start[0], start[1], start[2], end[0], end[1], end[2], rmin_vertical, road_yaw);
			if (roadthird == 0)
			{
				if (GetOrCheckPathPoints(start, road_yaw, 4, 1))
				{
					return GetOrCheckPathPoints(start, road_yaw, 4, 0);
				}
			}
		}
		// 提前完成
		if (PathOnlyOneNow == 1)
		{			
			if(fabs(start[0]) < 0.5 && fabs(start[1]) < 0.1 && fabs(start[2]) < 1 * pi / 180)
			{
				//pathpoint.clear();
				road_yaw[0] = 0;
				road_yaw[1] = -0.5;
				plan.IsPlanningCompleted = 1;
				return GetOrCheckPathPoints(start, road_yaw, 4, 0);
			}
		}
		//C+S-
		int roadfourth = 1;
		roadfourth = Calc_CpSm(start[0], start[1], start[2] * 180 / pi, end[0], end[1], end[2] * 180 / pi, rmin_vertical, 0, road_yaw);
		if (roadfourth == 0)
		{
			if (GetOrCheckPathPoints(start, road_yaw, 4, 1))
			{
				return GetOrCheckPathPoints(start, road_yaw, 4, 0);
			}
		}
		// C+C+S-
		int roadfifth = 1;
		roadfifth = road_find_five(start[0], start[1], start[2], end[0], end[1], end[2], rmin_vertical, road_yaw, road_yaw2);
		if (roadfifth == 0)
		{
			if (GetOrCheckPathPoints(start, road_yaw, 4, 1))
			{
				return GetOrCheckPathPoints(start, road_yaw, 4, 0);
			}
		}
		else if (roadfifth == 1)
		{
			if (GetOrCheckPathPoints(start, road_yaw2, 4, 1))
			{
				return GetOrCheckPathPoints(start, road_yaw2, 4, 0);
			}
			else
			{
				if (GetOrCheckPathPoints(start, road_yaw, 4, 1))
				{
					return GetOrCheckPathPoints(start, road_yaw, 4, 0);
				}
			}
		}
		// C+C-S-
		if (3 - Gears * 2 == -1 * theta_car_dynamic)
		{
			int roadsixth = 1;
			roadsixth = Calc_CpCmSm(start[0], start[1], start[2], end[0], end[1], end[2], rmin_vertical, road_yaw, road_yaw2);
			if (roadsixth == 0)
			{
				if (GetOrCheckPathPoints(start, road_yaw, 4, 1))
				{
					return GetOrCheckPathPoints(start, road_yaw, 4, 0);
				}
			}
			else if (roadsixth == 1)
			{
				return GetOrCheckPathPoints(start, road_yaw2, 4, 0);
			}
		}

		// C-C+(C-)S-
		if (3 - Gears * 2 == 1 * theta_car_dynamic)
		{
			int roadseventh = 1;
			roadseventh = road_find_seven(start[0], start[1], start[2], end[0], end[1], end[2], rmin_vertical, road_yaw, road_yaw2);
			if (roadseventh == 0)
			{
				//2024.03.05 新增提前动态规划
				dynamic_plan_flag = true;
				//
				return GetOrCheckPathPoints(start, road_yaw, 4, 0);
			}
			else if (roadseventh == 1)
			{
				return GetOrCheckPathPoints(start, road_yaw2, 4, 0);
			}
		}
		return false;
	}

	bool ParkingDirectly()
	{
		if ((fusion.parkingSpaceInfo.ParkingSpaceType != 2) && (app.APA_Park_Function != 2))
		{
			if (fusion.ParkInMode == 0)
			{
				//rear
				if((PlanBackFirst > 2 || PlanForwardFirst > 2) || AstarOrGeo == 1)
				{
					if (RearParkingDirectly(Start, End) != 0)
					{
						pathpoint = PathPointsCoordinateTrans(pathpoint, end_to_fus);
						PathPointNormalization();
						PathOnlyOneNow = IsPathOnlyOne();
						prk_inclined_dynamic_plan = ((PathOnlyOneNow == 0) && (dynamic_plan_flag));
						if(PathOnlyOneNow == 1 || prk_inclined_dynamic_plan)
						{
							StartRecordFlag = 1;
						}
						return true;
					}
					else
					{
						PathOnlyOneNow = 0;
						prk_inclined_dynamic_plan = false;
					}
				}
			}
			else
			{
				// head
				if(PlanBackFirst > 1 || AstarOrGeo == 1)
				{
					if(CalcHeadPathDirectly(Start,End))
					{
						pathpoint = PathPointsCoordinateTrans(pathpoint, end_to_fus);
						PathPointNormalization();
						PathOnlyOneNow = IsPathOnlyOne();
						prk_inclined_dynamic_plan = ((PathOnlyOneNow == 0) && (dynamic_plan_flag));
						if(PathOnlyOneNow == 1 || prk_inclined_dynamic_plan)
						{
							StartRecordFlag = 1;
						}
						return true;
					}
					else
					{
						PathOnlyOneNow = 0;
						prk_inclined_dynamic_plan = false;
					}
				}
			}
		}
		return false;
	}
}

