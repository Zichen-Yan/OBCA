#include "global_variable.h"
#include "global_function.h"

namespace byd_apa_plan
{
	
	void InitLog(const bool &flag_log)
	{
		if(flag_log)
		{
			if(APAStatus_before == 7 && app.APAStatus == 2)
			{
				#ifdef DEBUG_PC
					std::string file_name = "../plan.txt";
				#else
					std::string file_name = "/home/byd/Downloads/j3_v18.1.0_230322/hbre/planAlgorithm_new_test/plan_Algorithm.txt";
				#endif
				pF = fopen(file_name.c_str(),"w");
				PrintToLog(flag_log, "version: PlanAlgorithm_J3_xxx; ");
				//fflush(pF);
				//fclose(pF);
			}
			if((APAStatus_before == 2) && ((app.APAStatus == 4) || (app.APAStatus == 5) || (app.APAStatus == 6)))
			{
				fflush(pF);
				fclose(pF);
			}
		}
	}

	void PrintToLog(const bool &flag_log, std::string str)
	{
		if(flag_log)
		{
			fprintf(pF, "%s",str.c_str());
			fflush(pF);
		}
	}

	void PlanInputInfoLog(const bool &flag_log)
	{
		if(flag_log)
		{
			//fprintf(pF, "Plan Information: ");
			fprintf(pF, "plan_request=%d, index_request=%d\n", plan_request,index_request);
			fprintf(pF, "fusion.position=%lf,%lf,%lf\n", fusion.position.X, fusion.position.Y, fusion.position.Heading);
			fprintf(pF, "nav_pos=%lf,%lf,%lf\n", calculation.nav_pos_X, calculation.nav_pos_Y, calculation.nav_heading);
			fprintf(pF, "park=%d,%d, %d,%d, %d,%d, %d,%d\n", fusion.parkingSpaceInfo.P0_X, fusion.parkingSpaceInfo.P0_Y,
											fusion.parkingSpaceInfo.P1_X, fusion.parkingSpaceInfo.P1_Y,
											fusion.parkingSpaceInfo.P2_X, fusion.parkingSpaceInfo.P2_Y,
											fusion.parkingSpaceInfo.P3_X, fusion.parkingSpaceInfo.P3_Y);
			fprintf(pF, "fusion.Theta=%lf,\n", fusion.Theta);
			fprintf(pF, "fusion.depth_block=%lf, b_depth=%lf,\n",fusion.depth_block,b_depth);
			fprintf(pF, "fusion.parkingSpaceInfo.ParkingSpaceType=%d, fusion.ParkInMode=%d\n", fusion.parkingSpaceInfo.ParkingSpaceType, fusion.ParkInMode);
			fprintf(pF, "fusion.cam_or_uss=%d, %d,\n", fusion.TraceParkingID_Cam,fusion.TraceParkingID_USS);
			fprintf(pF, "APA_Park_Function=%d,\n", app.APA_Park_Function);
			fprintf(pF, "ObsUssInfo=%d,\n", control.ObsUssInfo);
			fprintf(pF, "gears=%d,\n", Gears);
			fprintf(pF, "plan_park=%lf,%lf, %lf,%lf, %lf,%lf, %lf,%lf\n", P0x,P0y,P1x,P1y,P2x,P2y,P3x,P3y);
			fprintf(pF, "ParkingSpaceFlag=%d, right_left=%d,\n", ParkingSpaceFlag,right_left);
			fprintf(pF, "start_to_fus=%lf, %lf, %lf,\n", start_to_fus[0], start_to_fus[1], start_to_fus[2]);
			//fprintf(pF, "end_to_fus_before=%lf, %lf, %lf,\n", end_to_fus_before[0], end_to_fus_before[1], end_to_fus_before[2]);
			fprintf(pF, "end_to_fus=%lf, %lf, %lf,\n", end_to_fus[0], end_to_fus[1], end_to_fus[2]);
			fprintf(pF, "End_level_small=%lf, %lf, %lf,\n", End_level_small[0], End_level_small[1], End_level_small[2]);
			fprintf(pF, "start=%lf, %lf, %lf,\n", Start[0], Start[1], Start[2]);
			fprintf(pF, "end=%lf, %lf, %lf,\n", End[0], End[1], End[2]);
			fprintf(pF, "mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm\n");
			for (int map_i = 0; map_i < 62500; map_i++)
			{
				fprintf(pF, "%d,", (int)obstmap[map_i].Status);
			}
			fprintf(pF, "\n");
			fprintf(pF, "mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm\n");
			fprintf(pF, "IsPlanningCompleted=%d,\n", plan.IsPlanningCompleted);
			fprintf(pF, "dist_P01=%lf, dist_P23=%lf, dist_P03=%lf, dist_toward_P03=%lf, no_imag_map=%d,\n", dist_P01, dist_P23,dist_P03,dist_toward_P03,no_imag_map);
			fprintf(pF, "plan_backward=%d, plan_forward=%d, plan_multi=%d\n", PlanBackFirst, PlanForwardFirst, PlanMulti);
			fprintf(pF, "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n");
			fflush(pF);
		}
	}

	void PlanOutputInfoLog(const bool &flag_log)
	{
		if(flag_log)
		{
			fprintf(pF, "0 is Geo, 1 is Astar. AstarOrGeo=%d,\n", AstarOrGeo);
			fprintf(pF, "small_level_park=%d,SEorES=%d, CloseSizeMax=%d,no_imag_map=%d\n", small_level_park,SEorES,CloseSizeMax,no_imag_map);
			fprintf(pF, "geo_plan_time=%lf, astar_plan_time=%lf, plan_time=%lf, \n", nesconds_geo_plan, nseconds_plan - nesconds_geo_plan, nseconds_plan); // 记录算法计算时间
			fprintf(pF, "PlanOnlyOne=%d, PathOnlyOneNow=%d, \n", PlanOnlyOne, PathOnlyOneNow);
			fprintf(pF, "IsPlanningCompleted=%d,\n", plan.IsPlanningCompleted);
			fprintf(pF, "index_level_out=%d, Aim_dist=%lf,\n", index_level_out, Aim_dist);
			fprintf(pF, "PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP\n");
			for (int txt_i = 0; txt_i < 300; txt_i++)
			{
				fprintf(pF, "%lf,%lf,%lf,%lf,%d\n", plan.coordinate[txt_i].X, plan.coordinate[txt_i].Y, plan.coordinate[txt_i].Yaw, plan.coordinate[txt_i].Curvature, plan.coordinate[txt_i].rajectoryDirection);
			}
			fprintf(pF, "PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP\n");
			fflush(pF);
		}	
	}
}
