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
            PlanInputInfoLog(FLAG_log);
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
                nseconds_plan = tictoc_plan.toc();
                nesconds_geo_plan = nseconds_plan * 1.0e-9;
                if(plan.IsPlanningCompleted == 1)
                {
                    #ifndef DEBUG_PC
                    complete_publisher("");
                    #endif
                    PrintToLog(FLAG_log, "Parking Directly success,");
                    PlanOutputInfoLog(FLAG_log);
                    return;
                }
            }
            else
            {
                AstarOrGeo=1;
                if (AstarOrGeo == 0)
                {
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
                if ((AstarOrGeo == 1) && (app.APA_Park_Function == 1))
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
                PrintToLog(FLAG_log, "Geometry or AStar success,");
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
                if (RearDynamicPlan(start_to_prk, end_to_prk, 0) == 1) 
                {
                    plan.PlanningStatus = 3;
                    nseconds_plan = tictoc_plan.toc();
                    nseconds_plan = nseconds_plan * 1.0e-9;
                    PlanInputInfoLog(FLAG_log);
                    pathpoint = PathPointsCoordinateTrans(pathpoint, end_to_fus);
                    PathPointNormalization();
                    #ifndef DEBUG_PC
                    status_publisher("");
                    path_publisher("");
                    #endif
                    PathOnlyOneNow = IsPathOnlyOne();
                    StartRecordFlag = 1;
                    FLAG_dynamic_plan_advanced = false;
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
                if (RearDynamicPlan(start_to_prk, end_to_prk, 1) == 1)
                {
                    plan.PlanningStatus = 3;
                    nseconds_plan = tictoc_plan.toc();
                    nseconds_plan = nseconds_plan * 1.0e-9;
                    PlanInputInfoLog(FLAG_log); 
                    pathpoint = PathPointsCoordinateTrans(pathpoint, end_to_fus);
                    PathPointNormalization(); 
                    #ifndef DEBUG_PC
                    status_publisher("");
                    path_publisher("");
                    #endif
                    PathOnlyOneNow = IsPathOnlyOne();
                    StartRecordFlag = 1;
                    FLAG_dynamic_plan_advanced = false;
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
            FLAG_published_end = true;
            #ifndef DEBUG_PC
            status_publisher("");
            path_publisher("");
            #endif	
            PrintToLog(FLAG_log,"dynamic final end, ");
            PlanOutputInfoLog(FLAG_log);
            return;
        }
        if((FLAG_stop_advanced == 0) && !FLAG_dynamic_plan_advanced) // 
        {
            double end_bias[3] = {.0, .0, .0};
            end_bias[0] = (fus_end_last[0] - end_to_fus[0]) * cos(end_to_fus[2]) + (fus_end_last[1] - end_to_fus[1]) * sin(end_to_fus[2]);
            end_bias[1] = (fus_end_last[1] - end_to_fus[1]) * cos(end_to_fus[2]) - (fus_end_last[0] - end_to_fus[0]) * sin(end_to_fus[2]);
            end_bias[2] = mod2pi(fus_end_last[2] - end_to_fus[2]);
            if((plan.PlanningStatus == 5)&& (index_request > 0) && ((d_s2e < 9) || (d_phi < 5*pi/180)))
            {
                // 横向偏差 0.1m -> 0.15m
                if((fabs(end_bias[1]) > 0.1) || (fabs(end_bias[2]) > 3*pi/180)) 
                {
                    PlanInputInfoLog(FLAG_log);
                    FLAG_stop_advanced = 1;
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
                if (HeadDynamicPlan(start_to_prk, end_to_prk, 0) == 1) 
                {
                    plan.PlanningStatus = 3;
                    nseconds_plan = tictoc_plan.toc();
                    nseconds_plan = nseconds_plan * 1.0e-9;
                    PlanInputInfoLog(FLAG_log);
                    pathpoint = PathPointsCoordinateTrans(pathpoint, end_to_fus);
                    PathPointNormalization();
                    #ifndef DEBUG_PC
                    status_publisher("");
                    path_publisher("");
                    #endif
                    PathOnlyOneNow = IsPathOnlyOne();
                    StartRecordFlag = 1;
                    FLAG_dynamic_plan_advanced = false;
                    PrintToLog(FLAG_log, "Head Dynamic Plan C or CS.. success,");
                    PlanOutputInfoLog(FLAG_log);
                    return;
                }
                else{
                    plan.PlanningStatus = 5;
                }
            }
        }
        if((FLAG_stop_advanced == 0) && !FLAG_dynamic_plan_advanced)  //
        {
            double end_bias[3] = {.0, .0, .0};
            end_bias[0] = (fus_end_last[0] - end_to_fus[0]) * cos(end_to_fus[2]) + (fus_end_last[1] - end_to_fus[1]) * sin(end_to_fus[2]);
            end_bias[1] = (fus_end_last[1] - end_to_fus[1]) * cos(end_to_fus[2]) - (fus_end_last[0] - end_to_fus[0]) * sin(end_to_fus[2]);
            end_bias[2] = mod2pi(fus_end_last[2] - end_to_fus[2]);
            if((plan.PlanningStatus == 5)&& (index_request > 0) && ((d_s2e < 9) || (d_phi < 5*pi/180)))
            {
                // 横向偏差 0.1m -> 0.15m
                if((fabs(end_bias[1]) > 0.1) || (fabs(end_bias[2]) > 3*pi/180))
                {
                    PlanInputInfoLog(FLAG_log);
                    FLAG_stop_advanced = 1;
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
			if (Dynamic_plan_level(start_to_fus, end_to_fus) == 1) 
			{
                plan.PlanningStatus = 3;
                nseconds_plan = tictoc_plan.toc();
                nseconds_plan = nseconds_plan * 1.0e-9;
                if (nseconds_plan < 0.45)
                {
                    double x, y, phi;
		            double start_now[3] = {.0,.0,.0};
		            double fus_theta = mod2pi(fusion.position.Heading * 0.017453292519943);  //  坐标系转换弧度
		            double cal_theta = mod2pi(calculation.nav_heading * 0.017453292519943);  //  坐标系转换弧度
		            CoordinateTrans(x,y,phi,calculation.nav_pos_X,calculation.nav_pos_Y,cal_theta,fusion.position.X,fusion.position.Y,fus_theta);
		            start_now[0] = x / 100.0;
		            start_now[1] = y / 100.0;
		            start_now[2] = phi;
                    double d_s_n = (start_to_fus[0] - start_now[0]) * (start_to_fus[0] - start_now[0]) + (start_to_fus[1] - start_now[1]) * (start_to_fus[1] - start_now[1]);
                    if (d_s_n < 0.25)
                    {
                        PlanInputInfoLog(FLAG_log);
                        StartRecordFlag = 1;
                        PathPointNormalization();
                        #ifndef DEBUG_PC
                        status_publisher("");
                        path_publisher("");
                        #endif
                        PrintToLog(FLAG_log, "Dynamic plan of level success,");
                        PlanOutputInfoLog(FLAG_log);
                        return;
                    } 
                }
			}
            if (start_to_fus[0] < end_to_fus[0] + 0.6 || fabs(start_to_fus[1] - end_to_fus[1]) < 0.6)
            {
                LevelDynamic = 0;
                PathOnlyOneNow = 0;
				StartRecordFlag = 0;
				PlanOnlyOne = 0;
            }
		}
    }

    void DynamicPlanRearOut()
    {
        double dynamic_dist = 0.01;
        double dx_s2s_first = start_to_fus[0] - Start_out[0];
        if(d_s2s > dynamic_dist && (dx_s2s_first > dist_rear_out + 0.1))
        {
            Planning();
        }
    }

    void DynamicPlanHeadOut()
    {
        double dynamic_dist = 0.01;
        double dx_s2s_first = start_to_fus[0] - Start_out[0];
        if(d_s2s > dynamic_dist && (dx_s2s_first < dist_head_out - 0.1))
        {
            Planning();
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
			if(((PathOnlyOneNow == 1) || FLAG_dynamic_plan_advanced
                || (app.APA_Park_Function == 4) || (app.APA_Park_Function == 5)) 
                && (StartRecordFlag == 1))
			{
				fus_start_last[0] = start_to_fus[0];
				fus_start_last[1] = start_to_fus[1];
				fus_start_last[2] = start_to_fus[2];
				StartRecordFlag = 0;
				PlanOnlyOne++;
			}
			
			ErrorCode = GetStartAndEndPoint();
            
			if(PathOnlyOneNow == 1 || FLAG_dynamic_plan_advanced || (app.APA_Park_Function == 4) || (app.APA_Park_Function == 5))
			{
				d_s2s = ((start_to_fus[0] - fus_start_last[0])*(start_to_fus[0] - fus_start_last[0]) + (start_to_fus[1] - fus_start_last[1])*(start_to_fus[1] - fus_start_last[1])); // 距上次起点直线距离 近似航迹?
				d_s2e = ((start_to_fus[0] - end_to_fus[0])*(start_to_fus[0] - end_to_fus[0]) + (start_to_fus[1] - end_to_fus[1])*(start_to_fus[1] - end_to_fus[1])); // 距终点距离
				d_phi = fabs(mod2pi(start_to_fus[2] - end_to_fus[2]));
			}
			if (plan_request == 1u)
			{
                flag_dynamic_plan_advanced = false;
				Planning();
                plan_request = 0u;
            }
            else if((plan_request == 0u) && (app.APA_Park_Function == 1))
            {
                if((PlanOnlyOne > 0 && PathOnlyOneNow == 1) || (FLAG_dynamic_plan_advanced))
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
            else if(plan_request == 0u && app.APA_Park_Function == 4)
            {
                DynamicPlanHeadOut();
            }
            else if(plan_request == 0u && app.APA_Park_Function == 5)
            {
                DynamicPlanRearOut();
            }
		}
		APAStatus_before = app.APAStatus;	
    }
}