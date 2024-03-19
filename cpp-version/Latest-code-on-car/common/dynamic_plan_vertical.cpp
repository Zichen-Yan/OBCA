#include "global_variable.h"
#include "global_function.h"

namespace byd_apa_plan
{
	bool RearDynamicPlan(double start[3], double end[3], int dynamic_flag)
	{
		pathpoint.clear();
		double road_yaw[6] = { 0,0,0,0,0,0 };

		if (dynamic_flag == 0)
		{
			int roadfirst = 1;
			roadfirst = Calc_Cm(start[0], start[1], start[2], end[0], end[1], end[2], 0.03, rmin_vertical - 0.2, 0.3, road_yaw);
			if (roadfirst == 0)
			{
				//
				if (GetOrCheckPathPoints(start, road_yaw, 3, 1))
				{
					return GetOrCheckPathPoints(start, road_yaw, 3, 0);
				}
				//
			}
			int roadthird = 1;
			roadthird = Calc_CmSm(start[0], start[1], start[2] * 180 / pi, end[0], end[1], end[2] * 180 / pi, rmin_vertical - 0.2, 0.03, road_yaw);
			if (roadthird == 0)
			{
				if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) 
				{
					return GetOrCheckPathPoints(start, road_yaw, 3, 0);
				}
				//
			}

			if ((prk_inclined_dynamic_plan || (fusion.TraceParkingID_USS != 0)) && (d_phi < 15*pi/180)) //15
			{
				// C-C-
				int roadfifth = 1;
				roadfifth = Calc_CmCm(start[0], start[1], start[2], end[0], end[1], end[2], rmin_vertical, road_yaw);
				if (roadfifth == 0)
				{
					if (GetOrCheckPathPoints(start, road_yaw, 3, 1))
					{
						return GetOrCheckPathPoints(start, road_yaw, 3, 0);
					}
				}
			}

		}
		else
		{
			int roadsecond = 1;
			roadsecond = Calc_CmCm(start[0], start[1], start[2], end[0], end[1], end[2], rmin_vertical, road_yaw);
			if (roadsecond == 0)
			{
				if (GetOrCheckPathPoints(start, road_yaw, 3, 1))
				{
					return GetOrCheckPathPoints(start, road_yaw, 3, 0);
				}
				//
			}
		}
		return false;
	}

	// head
	bool HeadDynamicPlan(double start[3], double end[3], int dynamic_flag)
	{
		pathpoint.clear();
		double road_yaw[6] = { 0,0,0,0,0,0 };
		if (dynamic_flag == 0)
		{
			int roadfirst = 1;
			//Cp
			roadfirst = Calc_Cm(start[0], start[1], start[2], end[0], end[1], end[2], 0.03, rmin_vertical, 0.3, road_yaw);
			if (roadfirst == 0 && fabs(road_yaw[0]) > safe_dynamic_rmin)
			{
				if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) 
				{
					std::cout << "C\n";
					dynamic_times++;
					return GetOrCheckPathPoints(start, road_yaw, 3, 0);
				}
			}
			int roadthird = 1;
			//CpSp
			roadthird = Calc_CmSm(start[0], start[1], start[2] * 180 / pi, end[0], end[1], end[2] * 180 / pi, rmin_vertical, 0.03, road_yaw);
			if (roadthird == 0 && (fabs(road_yaw[0]) > safe_dynamic_rmin || fabs(road_yaw[3]) > 3.0)) // 4.5 -> 3.0
			{
				if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) 
				{
					dynamic_times++;
					std::cout << "CS\n";
					return GetOrCheckPathPoints(start, road_yaw, 3, 0);
				}
			}
			
			//CpSpCp
			if (fabs(start[2]) > pi/180 * 2) //角度限制更改
			{	
				int roadsixth = 1;
				double car_r_upper_cpspcp = 40;
				double car_r_lower_cpspcp = safe_dynamic_rmin;
					for(double car_r_now = car_r_upper_cpspcp; car_r_now >= car_r_lower_cpspcp; car_r_now -= 2){

						if (roadsixth == CalcCpSpCp(start,end,car_r_now,road_yaw))
						{			
							if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) // 碰撞检测
							{
								dynamic_times++;
								std::cout << "CSC\n" << std::endl;
								return GetOrCheckPathPoints(start, road_yaw, 3, 0);
							}
						}
					}
			}
			// CpCp
			if (fabs(start[2]) < pi/180 * 10)
			{
				int roadsecond = 1;
				double car_r_cpcp = safe_dynamic_rmin;
				if(AstarOrGeo == 1 && dynamic_times == 0){
					car_r_cpcp = safe_dynamic_rmin - 2;
				}
				roadsecond = Calc_CmCm(start[0], start[1], start[2], end[0], end[1], end[2], rmin_vertical, road_yaw);
				if (roadsecond == 0 && fabs(road_yaw[0]) > car_r_cpcp)
				{
					if (GetOrCheckPathPoints(start, road_yaw, 3, 1))
					{
						dynamic_times++;
						std::cout << "CC\n";
						return GetOrCheckPathPoints(start, road_yaw, 3, 0);
					}
				}
				else if(CalcSafeRadiusCpCp(start, end, car_r_cpcp))
				{
					std::cout << "SR_CC\n";
					return true;
				}
			}
			
			//CpCpSp固定半径
			int roadseventh = 1;
			double car_r_upper_cpcpsp = 40;
			double car_r_lower_cpcpsp = safe_dynamic_rmin;
			for(double car_r_now = car_r_upper_cpcpsp; car_r_now >= car_r_lower_cpcpsp; car_r_now -= 4){
				roadseventh = Calc_CmCmSm(start[0], start[1], start[2], end[0], end[1], end[2], car_r_now, road_yaw);//固定半径
				if (roadseventh == 0){
					if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) // 碰撞检测
					{
						dynamic_times++;
						std::cout << "CCS\n" << std::endl;
						return GetOrCheckPathPoints(start, road_yaw, 3, 0);
					}
				}
			}
			//NCC
			int roadninth = 1;
			double car_r_upper_ncc = 30;
			double car_r_lower_ncc = safe_dynamic_rmin - 2;
			for(double car_r_now = car_r_upper_ncc; car_r_now >= car_r_lower_ncc; car_r_now -= 1){
				roadninth = Calc_NCC(start[0], start[1], start[2], end[0], end[1], end[2], car_r_now);
				if (roadninth == 1){
				dynamic_times++;
				std::cout << "NCC\n";
				return true;
				}
			}
		}
		else
		{
			int roadsecond = 1;
			// CpCp
			roadsecond = Calc_CmCm(start[0], start[1], start[2], end[0], end[1], end[2], rmin_vertical, road_yaw);
			if (roadsecond == 0 && fabs(road_yaw[0]) > safe_dynamic_rmin)
			{
				if (GetOrCheckPathPoints(start, road_yaw, 3, 1))
				{
					std::cout << "CC\n";
					return GetOrCheckPathPoints(start, road_yaw, 3, 0);
				}
			}
		}
		return false;
	}
}
