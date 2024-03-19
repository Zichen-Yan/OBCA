#include "global_function.h"
#include "global_variable.h"

namespace byd_apa_plan
{
    void ReadMap()
    {
        Read_obstmap = 1;
        bydapa::common::TicToc tictoc_map;
        tictoc_map.tic();
        while (Read_obstmap)
        {
            usleep(10);
            double nseconds_map = tictoc_map.toc();
            nseconds_map = nseconds_map * 1e-9;
            if (nseconds_map > 1)
            {
                break;
            }
        }
    }

    void Planning()
    {
        ReadMap();
        bydapa::common::TicToc tictoc_plan;
        tictoc_plan.tic();
        SEorES = 0;
        index_request = index_request + 1; 
        if (ErrorCode == 1)
        {
            plan.PlanningStatus = 2;
            #ifndef DEBUG_PC
            status_publisher("");
            #endif
            return;
        }
        CoordinateTransToPrk();
        ChangeFusionMap();
        IsParkingCompleted();
        PlanInputInfoLog(FLAG_log);
        if (plan.IsPlanningCompleted == 1)
        {
            #ifndef DEBUG_PC
            complete_publisher("");
            #endif
            PlanOutputInfoLog(FLAG_log);
            return;
        }
        else
        {
            if(ParkingDirectly())
            {
                std::cout << "DirDirDir\n";
                nseconds_plan = tictoc_plan.toc();
                nesconds_geo_plan = nseconds_plan * 1.0e-9;
                if(plan.IsPlanningCompleted == 1)
                {
                    #ifndef DEBUG_PC
                    complete_publisher("");
                    #endif
                    PlanOutputInfoLog(FLAG_log);
                    return;
                }
            }
            else
            {
                if (AstarOrGeo == 0)
                {
                    std::cout << "GeoGeoGeo\n";
                    if (fusion.parkingSpaceInfo.ParkingSpaceType == 1)
                    {
                        ParkingVertical();
                    }
                    else if (fusion.parkingSpaceInfo.ParkingSpaceType == 2)
                    {
                        ParkingLevel();
                    }
                    else if (fusion.parkingSpaceInfo.ParkingSpaceType == 3)
                    {
                        ParkingInclined();
                    }
                    nseconds_plan = tictoc_plan.toc();
                    nesconds_geo_plan = nseconds_plan * 1.0e-9;
                }
                if ((AstarOrGeo == 1) && (app.APA_Park_Function != 2))
                {
                    std::cout << "AstrAstr\n";
                    if (fusion.parkingSpaceInfo.ParkingSpaceType == 1)
                    {
                        ParkingVertical_Astar(); // 垂直车位
                    }
                    else if (fusion.parkingSpaceInfo.ParkingSpaceType == 2)
                    {
                        ParkingLevel_Astar(); // 水平车位
                    }
                    else if (fusion.parkingSpaceInfo.ParkingSpaceType == 3)
                    {
                        ParkingOblique_Astar(); // 斜车位
                    }
                    SEorES = 0;
                }
            }
            nseconds_plan = tictoc_plan.toc();
            nseconds_plan = nseconds_plan * 1.0e-9;
            std::cout << "plan_time = " << nseconds_plan << std::endl;
            #ifndef DEBUG_PC
            status_publisher("");
            #endif
            if (plan.PlanningStatus == 1)
            {
                #ifndef DEBUG_PC
                path_publisher("");
                #endif
                PlanOutputInfoLog(FLAG_log);
            }
        }
    }
    
    void DynamicPlanRear()
    {
        double dynamic_dist = (fusion.TraceParkingID_USS != 0) ? 0.01 : 0.09;
        if (d_s2s > dynamic_dist && d_s2e > 1.44)
        {
            ReadMap();
            CoordinateTransToPrk();
            bydapa::common::TicToc tictoc_plan;
            tictoc_plan.tic();
            ChangeFusionMap();
            if (d_phi > pi/18)
            {
                if (RearDynamicPlan(Start, End, 0) == 1) 
                {
                    nseconds_plan = tictoc_plan.toc();
                    nseconds_plan = nseconds_plan * 1.0e-9;
                    PlanInputInfoLog(FLAG_log);
                    StartRecordFlag = 1;
                    pathpoint = PathPointsCoordinateTrans(pathpoint, end_to_fus);
                    PathPointNormalization();
                    plan.PlanningStatus = 3;
                    #ifndef DEBUG_PC
                    status_publisher("");
                    path_publisher("");
                    #endif
                    PrintToLog(FLAG_log, "Dynamic plan C or CS success,");
                    PlanOutputInfoLog(FLAG_log);
                    return;
                }
                else{
                    plan.PlanningStatus = 5;
                }
            }
            else
            {
                if (RearDynamicPlan(Start, End, 1) == 1)
                {
                    nseconds_plan = tictoc_plan.toc();
                    nseconds_plan = nseconds_plan * 1.0e-9;
                    PlanInputInfoLog(FLAG_log); 
                    StartRecordFlag = 1;
                    pathpoint = PathPointsCoordinateTrans(pathpoint, end_to_fus);
                    PathPointNormalization(); 
                    plan.PlanningStatus = 3;
                    #ifndef DEBUG_PC
                    status_publisher("");
                    path_publisher("");
                    #endif
                    PrintToLog(FLAG_log, "Dynamic plan CC Success,");
                    PlanOutputInfoLog(FLAG_log);
                    return; 
                }
                else{
                    plan.PlanningStatus = 5;
                }
            }
        }
        else if((d_s2e >= 1.0) && (d_s2e < 1.44))
        {
            double final_end_to_fus[3] = {.0, .0, .0};
            PlanInputInfoLog(FLAG_log);
            //StartRecordFlag = 1;
            PathOnlyOneNow = 0;
            for(int i = 0; i < 300; i++)
            {
                plan.coordinate[i].X = 0;
                plan.coordinate[i].Y = 0;
                plan.coordinate[i].Yaw = 0;
                plan.coordinate[i].rajectoryDirection = 0;
                plan.coordinate[i].Curvature = 0;
            }
            final_end_to_fus[0] = end_to_fus[0];
            final_end_to_fus[1] = end_to_fus[1];
            final_end_to_fus[2] = end_to_fus[2];
            if(end_to_fus[2] < 0)
            {
                final_end_to_fus[2] = end_to_fus[2] + twopi;
            }
            plan.coordinate[0].X = final_end_to_fus[0] * 100;
            plan.coordinate[0].Y = final_end_to_fus[1] * 100;
            plan.coordinate[0].Yaw = final_end_to_fus[2] * 180/pi;
            plan.coordinate[0].rajectoryDirection = 2;
            plan.coordinate[0].Curvature = 0;
            plan.PlanningStatus = 3;
            dynamic_final_end_flag = true;
            #ifndef DEBUG_PC
            status_publisher("");
            path_publisher("");
            #endif	
            PrintToLog(FLAG_log,"dynamic final end, ");
            PlanOutputInfoLog(FLAG_log);
            return;
        }
        if((dynamic_break == 0))
        {
            double end_bias[3] = {.0, .0, .0};
            end_bias[0] = (fus_end_last[0] - end_to_fus[0]) * cos(end_to_fus[2]) + (fus_end_last[1] - end_to_fus[1]) * sin(end_to_fus[2]);
            end_bias[1] = (fus_end_last[1] - end_to_fus[1]) * cos(end_to_fus[2]) - (fus_end_last[0] - end_to_fus[0]) * sin(end_to_fus[2]);
            end_bias[2] = mod2pi(fus_end_last[2] - end_to_fus[2]);
            if((plan.PlanningStatus == 5)&& (index_request > 0) && ((d_s2e < 9) || (d_phi < 5*180/pi)))
            {
                // 横向偏差 0.1m -> 0.15m
                if((fabs(end_bias[1]) > 0.1) || (fabs(end_bias[2]) > 3*pi/180)) 
                {
                    PlanInputInfoLog(FLAG_log);
                    dynamic_break = 1;
                    plan.PlanningStatus = 4;
                    //新增提前停车，规划请求置2，防止在停车过程中继续动态规划
                    plan_request = 2u; 
                    //
                    #ifndef DEBUG_PC
                    status_publisher("");
                    #endif
                    PrintToLog(FLAG_log, "end bias too big, ");
                    PlanOutputInfoLog(FLAG_log);
                    return;
                }
            }
        }
    }

    void DynamicPlanHead()
    {
        //ToDo:车头动态规划
        double dynamic_dist = (fusion.TraceParkingID_USS != 0) ? 0.01 : 0.09;
        if (d_s2s > dynamic_dist && d_s2e > 1.44)
        {
            ReadMap();
            CoordinateTransToPrk();
            bydapa::common::TicToc tictoc_plan;
            tictoc_plan.tic();
            ChangeFusionMap();
            if (d_phi > 0*pi/18)
            {
                if (HeadDynamicPlan(Start, End, 0) == 1) 
                {
                    nseconds_plan = tictoc_plan.toc();
                    nseconds_plan = nseconds_plan * 1.0e-9;
                    PlanInputInfoLog(FLAG_log);
                    StartRecordFlag = 1;
                    pathpoint = PathPointsCoordinateTrans(pathpoint, end_to_fus);
                    PathPointNormalization();
                    plan.PlanningStatus = 3;
                    #ifndef DEBUG_PC
                    status_publisher("");
                    path_publisher("");
                    #endif
                    PrintToLog(FLAG_log, "Head Dynamic Plan C or CS.. success,");
                    PlanOutputInfoLog(FLAG_log);
                    return;
                }
                else{
                    plan.PlanningStatus = 5;
                }
            }
            /*
            else
            {
                if (HeadDynamicPlan(Start, End, 1) == 1)
                {
                    nseconds_plan = tictoc_plan.toc();
                    nseconds_plan = nseconds_plan * 1.0e-9;
                    PlanInputInfoLog(FLAG_log); 
                    StartRecordFlag = 1;
                    pathpoint = PathPointsCoordinateTrans(pathpoint, end_to_fus);
                    PathPointNormalization(); 
                    plan.PlanningStatus = 3;
                    #ifndef DEBUG_PC
                    status_publisher("");
                    path_publisher("");
                    #endif
                    PrintToLog(FLAG_log, "Head Dynamic Plan CC Success,");
                    PlanOutputInfoLog(FLAG_log);
                    return; 
                }
                else{
                    plan.PlanningStatus = 5;
                }
            }
            */
        }
        if((dynamic_break == 0))
        {
            double end_bias[3] = {.0, .0, .0};
            end_bias[0] = (fus_end_last[0] - end_to_fus[0]) * cos(end_to_fus[2]) + (fus_end_last[1] - end_to_fus[1]) * sin(end_to_fus[2]);
            end_bias[1] = (fus_end_last[1] - end_to_fus[1]) * cos(end_to_fus[2]) - (fus_end_last[0] - end_to_fus[0]) * sin(end_to_fus[2]);
            end_bias[2] = mod2pi(fus_end_last[2] - end_to_fus[2]);
            if((plan.PlanningStatus == 5)&& (index_request > 0) && ((d_s2e < 9) || (d_phi < 5*180/pi)))
            {
                // 横向偏差 0.1m -> 0.15m
                if((fabs(end_bias[1]) > 0.1) || (fabs(end_bias[2]) > 3*pi/180))
                {
                    PlanInputInfoLog(FLAG_log);
                    dynamic_break = 1;
                    plan.PlanningStatus = 4;
                    //新增提前停车，规划请求置2，防止在停车过程中继续动态规划
                    plan_request = 2u; 
                    //
                    #ifndef DEBUG_PC
                    status_publisher("");
                    #endif
                    PrintToLog(FLAG_log, "end bias too big, ");
                    PlanOutputInfoLog(FLAG_log);
                    return;
                }
            }
        }
    }

    void DynamicPlanLevel()
    {
        //ToDo:水平车位动态规划
        double end_bias[3] = { .0, .0, .0 };
        end_bias[0] = (fus_end_last[0] - end_to_fus[0]) * cos(end_to_fus[2]) + (fus_end_last[1] - end_to_fus[1]) * sin(end_to_fus[2]);
        end_bias[1] = (fus_end_last[1] - end_to_fus[1]) * cos(end_to_fus[2]) - (fus_end_last[0] - end_to_fus[0]) * sin(end_to_fus[2]);
        end_bias[2] = mod2pi(fus_end_last[2] - end_to_fus[2]);
        double dynamic_dist = (fusion.TraceParkingID_USS != 0) ? 0.01 : 0.09;
        if ((d_s2s > dynamic_dist) && (start_to_fus[0] > end_to_fus[0]) && ((end_bias[0] != 0) || (end_bias[1] != 0) || (end_bias[2] != 0)))  // && (d_s2e > 0.5)
		{
            ReadMap();
            CoordinateTransToPrk(); 
            bydapa::common::TicToc tictoc_plan;
            tictoc_plan.tic();
            ChangeFusionMap();
            bias_th = end_to_fus[2];
			if (Dynamic_plan_level(start_to_fus, end_to_fus) == 1) 
			{
                nseconds_plan = tictoc_plan.toc();
                nseconds_plan = nseconds_plan * 1.0e-9;
                PlanInputInfoLog(FLAG_log);
                StartRecordFlag = 1;
                PathPointNormalization();
                plan.PlanningStatus = 3;
                #ifndef DEBUG_PC
                status_publisher("");
                path_publisher("");
                #endif
                PrintToLog(FLAG_log, "Dynamic plan of level success,");
                PlanOutputInfoLog(FLAG_log);
                /// 24.2.27 ///
                if (start_to_fus[0] < end_to_fus[0] + 0.3)
                {
                    Path_2 = 0;
                    PathOnlyOneNow = 0;
					StartRecordFlag = 0;
					PlanOnlyOne = 0;
                }
                return;
			}
            /* else
            {
                // nseconds_plan = tictoc_plan.toc();
                // nseconds_plan = nseconds_plan * 1.0e-9;
                // // printf("nseconds_plan = %lf\n", nseconds_plan);
                // // printf("original path\n");
                // PlanInputInfoLog(FLAG_log);
                // StartRecordFlag = 1;
                // // PathPointNormalization();
                // plan.PlanningStatus = 3;
                // #ifndef DEBUG_PC
                // status_publisher("");
                // path_publisher("");
                // #endif
                // PrintToLog(FLAG_log, "original path,");
                // PlanOutputInfoLog(FLAG_log);
                return;
            } */
		}
    }

    void Process()
    {
        no_imag_map = 0;
		InitLog(FLAG_log);
		ParameterInit();
		StateClear();
		if (((fusion.Theta >= 0.0) && (fusion.Theta <= 360.0)) && ((fusion.TraceParkingID_Cam != 0u) || (fusion.TraceParkingID_USS != 0u)))
		{
			if(((PathOnlyOneNow == 1) || prk_inclined_dynamic_plan ) && (StartRecordFlag == 1))
			{
				Start_last[0] = start_to_fus[0];
				Start_last[1] = start_to_fus[1];
				Start_last[2] = start_to_fus[2];
				StartRecordFlag = 0;
				PlanOnlyOne++;
			}
			
			ErrorCode = GetStartAndEndPoint();
            
			if(PathOnlyOneNow == 1 || prk_inclined_dynamic_plan)
			{
				d_s2s = ((start_to_fus[0] - Start_last[0])*(start_to_fus[0] - Start_last[0]) + (start_to_fus[1] - Start_last[1])*(start_to_fus[1] - Start_last[1])); // 距上次起点直线距离 近似航迹?
				d_s2e = ((start_to_fus[0] - end_to_fus[0])*(start_to_fus[0] - end_to_fus[0]) + (start_to_fus[1] - end_to_fus[1])*(start_to_fus[1] - end_to_fus[1])); // 距终点距离
				d_phi = fabs(mod2pi(start_to_fus[2] - end_to_fus[2]));
			}
			if (plan_request == 1u)
			{
                dynamic_plan_flag = false;
				Planning();
                plan_request = 0u;
            }
            else if((plan_request == 0u) && (app.APA_Park_Function != 2)&& ((PlanOnlyOne > 0 && PathOnlyOneNow == 1) || (prk_inclined_dynamic_plan)))
            {
                if(fusion.parkingSpaceInfo.ParkingSpaceType != 2)
                {
                    if(fusion.ParkInMode == 1)
                    {
                        DynamicPlanHead();
                    }
                    else
                    {
                        DynamicPlanRear();
                    }
                }
                else if(fusion.parkingSpaceInfo.ParkingSpaceType == 2)
                {
                    DynamicPlanLevel();
                }
            }
		}
		APAStatus_before = app.APAStatus;	
    }
}