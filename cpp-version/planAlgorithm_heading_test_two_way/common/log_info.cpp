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
				spdlog::drop_all();
				try{
					#ifdef DEBUG_PC
						plan_logger = spdlog::basic_logger_mt("plan_logger", "../plan.txt", true);
					#else
						plan_logger = spdlog::basic_logger_mt("plan_logger", "/userdata/cyclonedds_test/log/plan.txt", true);
					#endif
				}
				catch(const spdlog::spdlog_ex &ex)
				{
					std::cout << "logger init failed: " << ex.what() << std::endl;
					return;
				}
				if(plan_logger != nullptr)
				{
					plan_logger->set_pattern("%v");
				}
			}
			if ((APAStatus_before == 2) && ((app.APAStatus == 4) || (app.APAStatus == 5) || (app.APAStatus == 6)))
			{
				if (plan_logger != nullptr)
				{
					plan_logger->flush_on(spdlog::level::info);
					spdlog::drop_all();
				}
			}
		}
	}

	void PrintToLog(const bool &flag_log, std::string str)
	{
		if (flag_log && plan_logger != nullptr)
		{
			plan_logger->info("{:s}", str);
			plan_logger->flush_on(spdlog::level::info);
		}
	}

	void PlanInputInfoLog(const bool &flag_log)
	{
		if(flag_log && plan_logger != nullptr)
		{
			plan_logger->info("version: PlanAlgorithm_J3_0428; plan_request={:d}, index_request={:d}", plan_request, index_request);
			plan_logger->info("fusion.position={:.4f},{:.4f},{:.4f}", fusion.position.X, fusion.position.Y, fusion.position.Heading);
			plan_logger->info("nav_pos={:.4f},{:.4f},{:.4f}", calculation.nav_pos_X, calculation.nav_pos_Y, calculation.nav_heading);
			plan_logger->info("park={:d},{:d}, {:d},{:d}, {:d},{:d}, {:d},{:d};  current_park={:d},{:d}, {:d},{:d}, {:d},{:d}, {:d},{:d}",
								fusion.parkingSpaceInfo.P0_X, fusion.parkingSpaceInfo.P0_Y,
							  	fusion.parkingSpaceInfo.P1_X, fusion.parkingSpaceInfo.P1_Y,
							  	fusion.parkingSpaceInfo.P2_X, fusion.parkingSpaceInfo.P2_Y,
							  	fusion.parkingSpaceInfo.P3_X, fusion.parkingSpaceInfo.P3_Y,
								
								currentParkInfo.CurrentPark.P0_X, currentParkInfo.CurrentPark.P0_Y,
							  	currentParkInfo.CurrentPark.P1_X, currentParkInfo.CurrentPark.P1_Y,
							  	currentParkInfo.CurrentPark.P2_X, currentParkInfo.CurrentPark.P2_Y,
							  	currentParkInfo.CurrentPark.P3_X, currentParkInfo.CurrentPark.P3_Y);
			plan_logger->info("fusion.Theta={:.4f},  currentParkInfo.Angle={:.4f}", fusion.Theta, currentParkInfo.Angle);
			plan_logger->info("fusion.depth_block={:.4f}, b_depth={:.4f},", fusion.depth_block, b_depth);
			plan_logger->info("fusion.parkingSpaceInfo.ParkingSpaceType={:d}, fusion.ParkInMode={:d}", fusion.parkingSpaceInfo.ParkingSpaceType, fusion.ParkInMode);
			plan_logger->info("fusion.cam_or_uss={:d},{:d},", fusion.TraceParkingID_Cam, fusion.TraceParkingID_USS);
			plan_logger->info("APA_Park_Function={:d},", app.APA_Park_Function);
			plan_logger->info("ObsUssInfo={:d},", control.ObsUssInfo);
			plan_logger->info("gears={:d},", Gears);
			plan_logger->info("plan_park={:.4f},{:.4f}, {:.4f},{:.4f}, {:.4f},{:.4f}, {:.4f},{:.4f}", P0x, P0y, P1x, P1y, P2x, P2y, P3x, P3y);
			plan_logger->info("ParkingSpaceFlag={:d}, right_left={:d},", ParkingSpaceFlag, right_left);
			plan_logger->info("start_to_fus={:.4f}, {:.4f}, {:.4f},", start_to_fus[0], start_to_fus[1], start_to_fus[2]);
			plan_logger->info("end_to_fus={:.4f}, {:.4f}, {:.4f},", end_to_fus[0], end_to_fus[1], end_to_fus[2]);
			plan_logger->info("End_level_small={:.4f}, {:.4f}, {:.4f},", End_level_small[0], End_level_small[1], End_level_small[2]);
			plan_logger->info("start_to_prk={:.4f}, {:.4f}, {:.4f},", start_to_prk[0], start_to_prk[1], start_to_prk[2]);
			plan_logger->info("end_to_prk={:.4f}, {:.4f}, {:.4f},", end_to_prk[0], end_to_prk[1], end_to_prk[2]);
			plan_logger->info("mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm");
			std::string map_string;
			for (int map_i = 0; map_i < 62500; map_i++)
			{
				map_string += std::to_string((int)obstmap[map_i].Status);
				map_string += ",";
			}
			plan_logger->info("{:s}", map_string);
			plan_logger->info("mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm");
			plan_logger->info("IsPlanningCompleted={:d},", plan.IsPlanningCompleted);
			plan_logger->info("dist_P01={:.4f}, dist_P23={:.4f}, dist_P03={:.4f}, dist_toward_P03={:.4f}, no_imag_map={:d},", dist_P01, dist_P23, dist_P03, dist_toward_P03, no_imag_map);
			plan_logger->info("PlanBackwardFirst={:d}, PlanForwardFirst={:d}, PlanMulti={:d}", PlanBackwardFirst, PlanForwardFirst, PlanMulti);
			plan_logger->info("cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc");
			plan_logger->flush_on(spdlog::level::info);
		}
	}

	void PlanOutputInfoLog(const bool &flag_log)
	{
		if(flag_log && plan_logger != nullptr)
		{
			plan_logger->info("0 is Geo, 1 is Astar. AstarOrGeo={:d},", AstarOrGeo);
			plan_logger->info("small_level_park={:d},SEorES={:d}, CloseSizeMax={:d},no_imag_map={:d}", small_level_park, SEorES, CloseSizeMax, no_imag_map);
			plan_logger->info("geo_plan_time={:4f}, astar_plan_time={:4f}, plan_time={:4f}, avg_dis={:4f}", nesconds_geo_plan, nseconds_plan - nesconds_geo_plan, nseconds_plan, avg_dis);
			plan_logger->info("PlanOnlyOne={:d}, PathOnlyOneNow={:d}, FLAG_dynamic_plan_advanced={:d}", PlanOnlyOne, PathOnlyOneNow, FLAG_dynamic_plan_advanced);
			plan_logger->info("IsPlanningCompleted={:d},", plan.IsPlanningCompleted);
			plan_logger->info("Aim_dist={:4f}, P_r_x_first={:4f}, first_path_end.x={:4f}, first_path_end.y={:4f}, index_level_out={:d}", Aim_dist, P_r_x_first, first_path_end.first, first_path_end.second, index_level_out);
			plan_logger->info("dist_head_out={:4f}, dist_rear_out={:4f}, dist_out_min={:4f}, dist_right_out={:4f}, dist_left_out={:4f}, flag_out_park={:d}",
								dist_head_out, dist_rear_out, dist_out_min, dist_right_out, dist_left_out, flag_out_park);
			plan_logger->info("PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP");
			for (int txt_i = 0; txt_i < 300; txt_i++)
			{
				plan_logger->info("{:4f},{:4f},{:4f},{:4f},{:d}", plan.coordinate[txt_i].X, plan.coordinate[txt_i].Y, plan.coordinate[txt_i].Yaw, plan.coordinate[txt_i].Curvature, plan.coordinate[txt_i].rajectoryDirection);
			}
			plan_logger->info("PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP");
			plan_logger->flush_on(spdlog::level::info);
		}
	}
}
