#include "global_variable.h"
#include "global_function.h"

namespace byd_apa_plan
{
    /////// 函数声明 /////
    int Dynamic_plan_level(double start1[3], double end1[3]);
    void loop_r(double start1[3], double end1[3], double &r1);
    int Calc_Path2(double start[3], double end[3], double end_collision[3], double r);                        // 24.2.27
    int roll_level_park_1(double Start1[3], double End1[3], int right_left, double road_roll[28]);            // 2.27
    void loop_R_th(double start1[3], double end1[3], double &R1, double &theta1);                             // 2.29
    void getroadpath_12(double start1[3], double end1[3], double R, double theta1, double end_collision1[3]); // 2.29

    bool VehicleCollisionGridLevel(double cpx, double cpy, double cph);
    int Roll_park(double Start1[3], double End1[3], int right_left, double backpath, double frontpath, double road_roll[28]);
    void getroadpath_Level_all(double Start[3], double road_roll[28], double D, double R);
    int roll_level_park(double Start1[3], double End1[3], int right_left, double road_roll[28]);
    int RoadCollision_Level(double Start[3], double End[3], double road_yaw[6]);
    int RoadCollision_Level_RD(double Start[3], double End[3], double R, double D, int &point_collision);
    int Cc_road(double Start[3], double End[3], int right_left, double road_Cc[6]);
    int Roll_Cc(double Start1[3], double Start_collision_mid1[3], int right_left, double R_level_1, double D_level_1, double road_roll[28]); // 2.27
    int Roll_Cc_1(double Start1[3], double Start_collision_mid1[3], int right_left, double R_level_1, double D_level_1, double road_roll[28]);

    /////////        开始动态规划       /////////////
    int Dynamic_plan_level(double start1[3], double end1[3]) ///// 主函数//// 传入的为融合坐标系下的坐标
    {
        double end[3];   ///// 目标点
        double start[3]; ///// 起点
        double R_c = 0;  //// 突变之后的半径
        double rk_start[3];
        start[0] = start1[0];
        start[1] = start1[1];
        start[2] = start1[2];
        end[0] = end1[0];
        end[1] = end1[1];
        end[2] = end1[2];
        double theta = 0;
        double Y_C_P = (start[1] - end[1]) * cos(end[2]) - (start[0] - end[0]) * sin(end[2]); 
        if (fabs(Y_C_P) > 0.8)                                                                
        {
            // 直接求解 // 修改求解的形式
            for (int d_y = 1; d_y <= 4; d_y++) // 1.29 直接求解，可能存在无解，无解之后采用遍历求解//
            {
                double road_roll[28] = {0};
                double D_C = 0;
                double R_C = 0;
                double end_1[3];
                end_1[0] = end[0];
                if (right_left == 1) // 右泊时向上偏差，左泊时向下偏差
                {
                    end_1[1] = end[1] + (d_y - 1) * 0.01;
                }
                else if (right_left == -1)
                {
                    end_1[1] = end[1] - (d_y - 1) * 0.01;
                }
                end_1[2] = end[2];
                int a = roll_level_park(start, end_1, right_left, road_roll);
                // printf("a=%d\n", a);
                if (a == 1)
                {
                    getroadpath_Level_all(start, road_roll, D_C, R_C);
                    printf("求解1\n");
                    return 1;
                }
                else if (a == 0 && d_y != 4)
                {
                    continue; // 带偏差求解
                }
                else
                {
                    break;
                }
            }
        }
        else // 24.2.27
        {
            double road_roll[28] = {0};
            double D_C = 0;
            double R_C = 0;
            double end_1[3];
            for (int d_ey = 1; d_ey <= 4; d_ey++)
            {
                if (right_left == 1) // 右泊时向上偏差，左泊时向下偏差
                {
                    end_1[1] = end[1] + (d_ey - 1) * 0.01;
                }
                else if (right_left == -1)
                {
                    end_1[1] = end[1] - (d_ey - 1) * 0.01;
                }
                end_1[0] = end[0];
                end_1[2] = end[2];

                int a1 = roll_level_park_1(start, end_1, right_left, road_roll);
                printf("roadfirst1 = %d\n", a1);
                if (a1 == 1)
                {
                    getroadpath_Level_all(start, road_roll, D_C, R_C);
                    printf("求解2\n");
                    return 1;
                }
                else if (a1 == 0 && d_ey != 4)
                {
                    continue; // 带偏差求解
                }
                else
                {
                    break;
                }
            }
        }
        // 无解遍历 //
        double X_C_P = (start[0] - end[0]) * cos(end[2]) + (start[1] - end[1]) * sin(end[2]);
        double d_x = floor(abs(X_C_P - backpath) / 0.1);
        if (d_x > 6 && fabs(Y_C_P) > 0.8)
        {
            loop_R_th(start, end, R_c, theta);
            if (R_c == 0 && theta == 0) // 1.25 //
            {
                printf("path_o0\n");
                return 0;
            }
            double end_collision[3];
            getroadpath_12(start, end, R_c, theta, end_collision); // 获取路径点
            rk_start[0] = end_collision[0];
            rk_start[1] = end_collision[1];
            rk_start[2] = end_collision[2];

            rk_sto[0] = rk_start[0]; // 揉库坐标存入pathpoint的时为融合坐标系下的开始点，
            rk_sto[1] = rk_start[1];
            rk_sto[2] = rk_start[2];

            /// 进入揉库
            double road_roll[28] = {0};
            int a1 = Roll_park(rk_start, end, right_left, backpath, frontpath, road_roll);
            if (a1 == 1)
            {
                printf("遍历1, a1 = 1\n");
                return 1;
            }
            else if (a1 == 2)
            {
                getroadpath_Level_all(rk_start, road_roll, 0, 0);
                printf("遍历1, a1 = 2\n");
                return 1;
            }
            else
            {
                printf("path_o1\n");
                return 0;
            }
        }
        else
        {
            double r;
            double end_collision1[3];
            // loop_r(start, end, r); // 
            r = rmin_level;
            if (right_left == 1)
            {
                if (Calc_Path2(start, end, end_collision1, r) != 1)
                {
                    printf("path_o2\n");
                    return 0;
                }
            }
            else
            {
                if (Calc_Path2(start, end, end_collision1, -r) != 1)
                {
                    printf("path_o2\n");
                    return 0;
                }
            }
            rk_start[0] = end_collision1[0];
            rk_start[1] = end_collision1[1];
            rk_start[2] = end_collision1[2];

            rk_sto[0] = rk_start[0]; // 揉库坐标存入pathpoint的时为融合坐标系下的开始点，这里要更新start_to_fus点的值
            rk_sto[1] = rk_start[1];
            rk_sto[2] = rk_start[2];

            /// 进入揉库
            double road_roll[28] = {0};
            int a1 = Roll_park(rk_start, end, right_left, backpath, frontpath, road_roll);
            if (a1 == 1)
            {
                printf("遍历2, a1 = 1\n");
                return 1;
            }
            else if (a1 == 2)
            {
                getroadpath_Level_all(rk_start, road_roll, 0, 0);
                printf("遍历2, a1 = 2\n");
                return 1;
            }
            else
            {
                printf("path_o2\n");
                return 0;
            }
        }
    }
    //////////////////////////////////////////

    ///////////////////////
    int roll_level_park_1(double Start1[3], double End1[3], int right_left, double road_roll[28])
    {
        double End[3];
        double Start[3];
        Start[0] = Start1[0];
        Start[1] = Start1[1];
        Start[2] = Start1[2];
        End[0] = End1[0];
        End[1] = End1[1];
        End[2] = End1[2];
        int point_collision = 100;
        double Start_collision[3] = {0, 0, 0}; // 计算碰撞的起点坐标
        double End_collision[3] = {0, 0, 0};
        double R_level_1 = 1e6;
        double D_level_1 = 1.7;
        int safety_D = 0;
        int safety_D_max = 3;
        double road_Cc[6] = {0, 0, 0, 0, 0, 0};
        ///////////////////////////直线后退有解或无解最多退多少/////////////////////////////////////////////
        for (int index = -safety_D_max; index < 18; index++)
        {
            int index_level_1 = RoadCollision_Level_RD(End, End_collision, R_level_1, -0.1 * index, point_collision); // 直线往后退
            if (index == 17)
            {
                index_level_1 = 1;
            }
            int index_level_2 = 0;
            if (index_level_1 == 0) // 直线段不碰撞
            {
                if (choice_cc == 0)
                {
                    index_level_2 = Cc_road(Start, End_collision, right_left, road_Cc);
                }
                else
                {
                    safety_D_max = 0;
                    index_level_2 = Calc_Cm(Start[0], Start[1], Start[2], End_collision[0], End_collision[1], End_collision[2], 0, rmin_level, 0, road_Cc);
                    if (right_left == 1)
                    {
                        if (road_Cc[0] < 0)
                        {
                            index_level_2 = 1;
                        }
                    }
                    else
                    {
                        if (road_Cc[0] > 0)
                        {
                            index_level_2 = 1;
                        }
                    }
                    if (index_level_2 != 0)
                    {
                        continue;
                    }
                    else
                    {
                        index_level_2 = 1;
                    }
                }
                if (index_level_2 == 1)
                {
                    Start_collision[0] = Start[0];
                    Start_collision[1] = Start[1];
                    Start_collision[2] = Start[2];
                    for (int index_Cc = 0; index_Cc < 3; index_Cc++)
                    {
                        index_level_2 = RoadCollision_Level_RD(Start_collision, End_collision, road_Cc[index_Cc * 2], road_Cc[index_Cc * 2 + 1], point_collision);
                        if (index_level_2 == 0)
                        {
                            Start_collision[0] = End_collision[0];
                            Start_collision[1] = End_collision[1];
                            Start_collision[2] = End_collision[2];
                        }
                        else
                        {
                            break;
                        }
                    }
                    if ((index_level_2 == 0) && (Gears == 1) && (road_Cc[1] > 0))
                    {
                        index_level_2 = 1;
                    }
                    if (index_level_2 == 0)
                    {
                        safety_D = safety_D + 1; // 满3就是前方预留30cm安全距离
                        if (safety_D >= safety_D_max + 1)
                        {
                            road_roll[0] = road_Cc[0];
                            if (road_Cc[1] > 0)
                            {
                                road_roll[1] = road_Cc[1] + 0.5;
                            }
                            else
                            {
                                road_roll[1] = road_Cc[1];
                            }
                            road_roll[2] = road_Cc[2];
                            road_roll[3] = road_Cc[3];
                            road_roll[4] = road_Cc[4];
                            road_roll[5] = road_Cc[5];
                            road_roll[6] = R_level_1;
                            road_roll[7] = 0.1 * index;

                            return 1; // 前方预留30cm有解
                        }
                    }
                }
            }
            else
            {
                if (index > 1)
                {
                    D_level_1 = 0.1 * (index - 1); // 和backpath的区别在哪
                    break;
                }
            }
        }
        double Start_collision_mid[3] = {End_collision[0], End_collision[1], End_collision[2]};
        int level_return = Roll_Cc_1(Start, Start_collision_mid, right_left, R_level_1, D_level_1, road_roll);
        if (level_return == 1)
        {
            return 1;
        }
        return 0;
    }
    /////////////////////////////////////

    /////////////////////////////////////
    void loop_R_th(double start1[3], double end1[3], double &R1, double &theta1) //
    {
        double start[3] = {start1[0], start1[1], start1[2]};
        double end[3] = {end1[0], end1[1], end1[2]};
        double x = start[0];
        double y = start[1];
        double th = start[2];
        double x1_o, x_e1;
        int flag_1 = 0;
        int flag_2 = 0;
        double D_L = -0.1;
        double delta_L = 0;
        double delta_L1 = 0;
        int f_th2 = 0;
        std::vector<double> R_c1;  
        std::vector<double> th_c;  
        std::vector<double> Th_c1; 
        std::vector<double> Y_E;    
        std::vector<double> Y_d;    
        std::vector<double> Y_e2;  
        std::vector<double> YY_C;  
        std::vector<double> Y_rk;  
        for (double R = rmin_level; R <= 8; R += 0.2)
        {
            R_c1.push_back(R);
            th_c.clear();
            if (right_left == 1)
            {
                for (double th1 = 0.1 / R; th1 + start[2] < 50 * pi / 180; th1 += 0.1 / R) //
                {
                    flag_1 = 0;
                    flag_2 = 0;
                    f_th2 = 0;
                    /// 1 ///
                    x = start[0];
                    y = start[1];
                    th = start[2];
                    x1_o = start[0] + R * sin(start[2]);
                    x_e1 = x1_o - R * sin(start[2] + th1);
                    double D1 = x_e1 - x;                      
                    delta_L = -atan(vehicle_parameters.WB / R); 
                    int i = floor(fabs(D1) / pathfind_parameters.MOTION_RESOLUTION);
                    int roadisCollision = 0;
                    if (D1 != 0 && i == 0) 
                    {
                        if (fabs(D1 - i * D_L) > 0.01)
                        {
                            VehicleDynamic(x, y, th, D1 - i * D_L, delta_L); 
                            roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth);
                            if (roadisCollision)
                            {
                                flag_1 = 1;
                                break;
                            }
                            x = g_px;
                            y = g_py;
                            th = g_pth;
                        }
                    }
                    if (flag_1 == 1)
                    {
                        break;
                    }
                    for (int idx_L = 0; idx_L < i; idx_L++)
                    {
                        VehicleDynamic(x, y, th, D_L, delta_L); 
                        roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth);
                        if (roadisCollision)
                        {
                            flag_1 = 1;
                            break;
                        }
                        x = g_px;
                        y = g_py;
                        th = g_pth;
                    }
                    if (flag_1 == 1)
                    {
                        break;
                    }
                    if (fabs(D1 - i * D_L) > 0.01)
                    {
                        VehicleDynamic(x, y, th, D1 - i * D_L, delta_L); 
                        roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth);
                        if (roadisCollision)
                        {
                            flag_1 = 1;
                            break;
                        }
                        x = g_px;
                        y = g_py;
                        th = g_pth;
                    }
                    if (flag_1 == 1)
                    {
                        break;
                    }
                    /// 2
                    double D2 = end[0] - x + backpath;
                    delta_L1 = atan(vehicle_parameters.WB / rmin_level);
                    int i2 = floor(fabs(D2) / pathfind_parameters.MOTION_RESOLUTION);
                    int roadisCollision1 = 0;
                    for (int idx_L2 = 0; idx_L2 < i2; idx_L2++) 
                    {
                        VehicleDynamic(x, y, th, D_L, delta_L1); 
                        roadisCollision1 = VehicleCollisionGridLevel(g_px, g_py, g_pth);
                        if (roadisCollision1)
                        {
                            flag_2 = 1;
                            break;
                        }
                        if ((g_pth + 0.01745 > end[2]) && (g_pth - 0.01745 < end[2])) 
                        {
                            x = g_px;
                            y = g_py;
                            th = g_pth;
                            f_th2 = 1; 
                            break;
                        }
                        x = g_px;
                        y = g_py;
                        th = g_pth;
                    }
                    if (flag_2 == 1)
                    {
                        continue;
                    }
                    if (fabs(D2 - i2 * D_L) > 0.01 && f_th2 == 0) 
                    {
                        VehicleDynamic(x, y, th, D2 - i2 * D_L, delta_L1); 
                        roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth);
                        if (roadisCollision)
                        {
                            flag_2 = 1;
                            break;
                        }
                        x = g_px;
                        y = g_py;
                        th = g_pth;
                    }
                    if (flag_2 == 1)
                    {
                        continue;
                    }
                    double sta[3] = {x, y, th};
                    rk_sto[0] = sta[0];
                    rk_sto[1] = sta[1];
                    rk_sto[2] = sta[2];
                    double road_roll[28] = {0};
                    int a1 = Roll_park(sta, end, right_left, backpath, frontpath, road_roll);
                    if (a1 == 2)
                    {
                        getroadpath_Level_all(sta, road_roll, 0, 0);
                    }
                    else if (a1 == 0)
                    {
                        continue;
                    }
                    PathPoint &last_element = pathpoint.back();
                    double last_y = last_element.y;
                    Y_E.push_back(last_y); 
                    th_c.push_back(th1);   
                    pathpoint.clear();     
                }
            }
            else
            {
                for (double th1 = -0.1 / R; th1 + start[2] > -50 * pi / 180; th1 -= 0.1 / R)
                {
                    flag_1 = 0;
                    flag_2 = 0;
                    f_th2 = 0;
                    /// 1 ///
                    x = start[0];
                    y = start[1];
                    th = start[2];
                    x1_o = start[0] - R * sin(start[2]);
                    x_e1 = x1_o + R * sin(start[2] + th1);
                    double D1 = x_e1 - x; 
                    delta_L = atan(vehicle_parameters.WB / R);
                    int i = floor(fabs(D1) / pathfind_parameters.MOTION_RESOLUTION);
                    int roadisCollision = 0;
                    if (D1 != 0 && i == 0) 
                    {
                        if (fabs(D1 - i * D_L) > 0.01)
                        {
                            VehicleDynamic(x, y, th, D1 - i * D_L, delta_L); 
                            roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth);
                            if (roadisCollision)
                            {
                                flag_1 = 1;
                                break;
                            }
                            x = g_px;
                            y = g_py;
                            th = g_pth;
                        }
                    }
                    if (flag_1 == 1)
                    {
                        break;
                    }
                    for (int idx_L = 0; idx_L < i; idx_L++)
                    {
                        VehicleDynamic(x, y, th, D_L, delta_L); 
                        roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth);
                        if (roadisCollision)
                        {
                            flag_1 = 1;
                            break;
                        }
                        x = g_px;
                        y = g_py;
                        th = g_pth;
                    }
                    if (flag_1 == 1)
                    {
                        break;
                    }
                    if (fabs(D1 - i * D_L) > 0.01)
                    {
                        VehicleDynamic(x, y, th, D1 - i * D_L, delta_L); 
                        roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth);
                        if (roadisCollision)
                        {
                            flag_1 = 1;
                            break;
                        }
                        x = g_px;
                        y = g_py;
                        th = g_pth;
                    }
                    if (flag_1 == 1)
                    {
                        break;
                    }
                    /// 2
                    double D2 = end[0] - x + backpath;
                    delta_L1 = -atan(vehicle_parameters.WB / rmin_level);
                    int i2 = floor(fabs(D2) / pathfind_parameters.MOTION_RESOLUTION);
                    int roadisCollision1 = 0;
                    for (int idx_L2 = 0; idx_L2 < i2; idx_L2++) 
                    {
                        VehicleDynamic(x, y, th, D_L, delta_L1); 
                        roadisCollision1 = VehicleCollisionGridLevel(g_px, g_py, g_pth);
                        if (roadisCollision1)
                        {
                            flag_2 = 1;
                            break;
                        }
                        if ((g_pth + 0.01745 > end[2]) && (g_pth - 0.01745 < end[2])) 
                        {
                            x = g_px;
                            y = g_py;
                            th = g_pth;
                            f_th2 = 1; 
                            break;
                        }
                        x = g_px;
                        y = g_py;
                        th = g_pth;
                    }
                    if (flag_2 == 1)
                    {
                        continue;
                    }
                    if (fabs(D2 - i2 * D_L) > 0.01 && f_th2 == 0) 
                    {
                        VehicleDynamic(x, y, th, D2 - i2 * D_L, delta_L1); 
                        roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth);
                        if (roadisCollision)
                        {
                            flag_2 = 1;
                            break;
                        }
                        x = g_px;
                        y = g_py;
                        th = g_pth;
                    }
                    if (flag_2 == 1)
                    {
                        continue;
                    }
                    double sta[3] = {x, y, th};
                    rk_sto[0] = sta[0];
                    rk_sto[1] = sta[1];
                    rk_sto[2] = sta[2];
                    double road_roll[28] = {0};
                    int a1 = Roll_park(sta, end, right_left, backpath, frontpath, road_roll);
                    if (a1 == 2)
                    {
                        getroadpath_Level_all(sta, road_roll, 0, 0);
                    }
                    else if (a1 == 0)
                    {
                        continue;
                    }
                    PathPoint &last_element = pathpoint.back();
                    double last_y = last_element.y;
                    Y_E.push_back(last_y); 
                    th_c.push_back(th1);   
                    pathpoint.clear();     
                }
            }
            if (flag_1 == 1 || flag_2 == 1 || Y_E.size() == 0) 
            {
                printf("Y_E.size = %zu\n", Y_E.size()); 
                th_c.clear();
                Y_E.clear();
                continue;
            }
            for (double num : Y_E)
            {
                if (right_left == 1)
                {
                    if (num >= end[1] - 0.01) 
                    {
                        Y_d.push_back(num); 
                        num -= end[1];
                        Y_e2.push_back(std::abs(num)); 
                    }
                }
                else
                {
                    if (num <= end[1] + 0.01) 
                    {
                        Y_d.push_back(num);
                        num -= end[1];
                        Y_e2.push_back(std::abs(num)); 
                    }
                }
            }
            if (Y_e2.size() == 0) 
            {
                printf("Y_e2.size = %zu\n", Y_e2.size()); 
                Y_e2.clear();
                Y_d.clear();
                th_c.clear();
                Y_E.clear();
                continue;
            }
            int col_rk = std::distance(Y_e2.begin(), std::min_element(Y_e2.begin(), Y_e2.end())); 
            double min_dy = Y_d[col_rk];                                                          
            auto c_y = std::find(Y_E.begin(), Y_E.end(), min_dy);                                 
            auto c_my = std::distance(Y_E.begin(), c_y);                                          
            Th_c1.push_back(th_c[c_my]);                                                          
            YY_C.push_back(Y_e2[col_rk]);                                                         
            Y_e2.clear();
            Y_d.clear();
            th_c.clear();
            Y_E.clear();
        }

        if (YY_C.size() == 0)
        {
            printf("YY_C.size = %zu\n", YY_C.size()); // bug1
            R1 = 0;
            theta1 = 0;
            return;
        }
        int b_c = std::distance(YY_C.begin(), std::min_element(YY_C.begin(), YY_C.end()));
        R1 = R_c1[b_c];
        theta1 = Th_c1[b_c];
    }
    /////////////////////////////////////////////

    ///////////////////////////////////////////
    void getroadpath_12(double start1[3], double end1[3], double R, double theta1, double end_collision1[3])
    {
        double start[3] = {start1[0], start1[1], start1[2]};
        double end[3] = {end1[0], end1[1], end1[2]};
        double x2;
        double x1_o, x_e1;
        int flag = 0;
        double D_L = -0.1;
        double delta_L = 0;
        double delta_L1 = 0;

        PathPoint mid_L;
        mid_L.x = start[0];
        mid_L.y = start[1];
        mid_L.th = mod2pi(start[2]);
        mid_L.D = 0;
        mid_L.delta = 0;
        pathpoint.push_back(mid_L);
        /// 1 ///
        if (right_left == 1)
        {
            delta_L = -atan(vehicle_parameters.WB / R); 
            x1_o = start[0] + R * sin(start[2]);
            x_e1 = x1_o - R * sin(start[2] + theta1); 
        }
        else
        {
            x1_o = start[0] - R * sin(start[2]);
            x_e1 = x1_o + R * sin(start[2] + theta1); 
            delta_L = atan(vehicle_parameters.WB / R);
        }
        double D = x_e1 - start[0];
        int i = floor(fabs(D) / pathfind_parameters.MOTION_RESOLUTION);
        if (D != 0 && i == 0) 
        {
            if (fabs(D - i * D_L) > 0.01)
            {
                VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D - i * D_L, delta_L); 
                mid_L.x = g_px;
                mid_L.y = g_py;
                mid_L.th = g_pth;
                mid_L.D = D_L;
                mid_L.delta = delta_L;
                pathpoint.push_back(mid_L);
            }
        }
        for (int idx_L = 0; idx_L < i; idx_L++) 
        {
            VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D_L, delta_L); 
            mid_L.x = g_px;
            mid_L.y = g_py;
            mid_L.th = g_pth;
            mid_L.D = D_L;
            mid_L.delta = delta_L;
            pathpoint.push_back(mid_L);
        }
        if (fabs(D - i * D_L) > 0.01)
        {
            VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D - i * D_L, delta_L); 
            mid_L.x = g_px;
            mid_L.y = g_py;
            mid_L.th = g_pth;
            mid_L.D = D_L;
            mid_L.delta = delta_L;
            pathpoint.push_back(mid_L);
        }

        x2 = g_px;
        //// ////
        double D1 = end[0] - x2 + backpath;
        if (right_left == 1) 
        {
            D_L = -0.1;
            delta_L1 = atan(vehicle_parameters.WB / rmin_level);
        }
        else
        {
            D_L = -0.1;
            delta_L1 = -atan(vehicle_parameters.WB / rmin_level);
        }
        int i1 = floor(fabs(D1) / pathfind_parameters.MOTION_RESOLUTION);
        for (int idx_L2 = 0; idx_L2 < i1; idx_L2++) 
        {
            VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D_L, delta_L1); 
            if (g_pth + 0.01745 > end[2] && g_pth - 0.01745 < end[2])  
            {
                mid_L.x = g_px;
                mid_L.y = g_py;
                mid_L.th = g_pth;
                mid_L.D = D_L;
                mid_L.delta = delta_L1;
                pathpoint.push_back(mid_L);
                flag = 1;
                break;
            }
            mid_L.x = g_px;
            mid_L.y = g_py;
            mid_L.th = g_pth;
            mid_L.D = D_L;
            mid_L.delta = delta_L1;
            pathpoint.push_back(mid_L);
        }
        if (flag == 0)
        {
            if (fabs(D1 - i1 * D_L) > 0.01)
            {
                VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D1 - i1 * D_L, delta_L1); 
                mid_L.x = g_px;
                mid_L.y = g_py;
                mid_L.th = g_pth;
                mid_L.D = D_L;
                mid_L.delta = delta_L1;
                pathpoint.push_back(mid_L);
            }
        }
        end_collision1[0] = g_px;
        end_collision1[1] = g_py;
        end_collision1[2] = g_pth;
    }
    /////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////
    void loop_r(double start1[3], double end1[3], double &r1)
    {
        double start[3] = {start1[0], start1[1], start1[2]};
        double end[3] = {end1[0], end1[1], end1[2]};
        double delta_L2 = 0;
        double D_L = -0.1;
        int flag_2 = 0;
        int f_th2 = 0;
        std::vector<double> r_all;
        std::vector<double> Y_d;
        std::vector<double> Y_e2;
        std::vector<double> Y_E;
        for (double r = rmin_level; r < 7; r += 0.15)
        {
            r_all.push_back(r);
            double x = start[0];
            double y = start[1];
            double th = start[2];
            double D2 = end[0] - start[0] + backpath;
            if (right_left == 1)
            {
                delta_L2 = atan(vehicle_parameters.WB / r);
            }
            else
            {
                delta_L2 = -atan(vehicle_parameters.WB / r);
            }
            int i2 = floor(fabs(D2) / pathfind_parameters.MOTION_RESOLUTION);
            int roadisCollision = 0;
            for (int idx_L2 = 0; idx_L2 < i2; idx_L2++) 
            {
                VehicleDynamic(x, y, th, D_L, delta_L2); 
                roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth);
                if (roadisCollision)
                {
                    flag_2 = 1;
                    break;
                }
                if ((g_pth + 0.01745 > end[2]) && (g_pth - 0.01745 < end[2])) 
                {
                    x = g_px;
                    y = g_py;
                    th = g_pth;
                    f_th2 = 1; 
                    break;
                }
                x = g_px;
                y = g_py;
                th = g_pth;
            }
            if (flag_2 == 1)
            {
                continue;
            }
            if (fabs(D2 - i2 * D_L) > 0.01 && f_th2 == 0) 
            {
                VehicleDynamic(x, y, th, D2 - i2 * D_L, delta_L2); 
                roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth);
                if (roadisCollision)
                {
                    flag_2 = 1;
                    break;
                }
                x = g_px;
                y = g_py;
                th = g_pth;
            }
            if (flag_2 == 1)
            {
                continue;
            }
            double sta[3] = {x, y, th};
            rk_sto[0] = sta[0];
            rk_sto[1] = sta[1];
            rk_sto[2] = sta[2];
            double road_roll[28] = {0};
            int a1 = Roll_park(sta, end, right_left, backpath, frontpath, road_roll);
            if (a1 == 2)
            {
                getroadpath_Level_all(sta, road_roll, 0, 0);
            }
            else if (a1 == 0)
            {
                continue;
            }
            PathPoint &last_element = pathpoint.back();
            double last_y = last_element.y;
            Y_E.push_back(last_y); 
            pathpoint.clear();    
        }
        if (Y_E.size() == 0) 
        {
            printf("Y_E.size = %zu\n", Y_E.size()); // bug4
            r1 = rmin_level;
            return;
        }
        for (double num : Y_E)
        {
            if (right_left == 1)
            {
                if (num >= end[1] - 0.01) 
                {
                    Y_d.push_back(num); 
                    num -= end[1];
                    Y_e2.push_back(std::abs(num)); 
                }
            }
            else
            {
                if (num <= end[1] + 0.01) 
                {
                    Y_d.push_back(num);
                    num -= end[1];
                    Y_e2.push_back(std::abs(num)); 
                }
            }
        }
        ///
        if (Y_e2.size() == 0) 
        {
            printf("Y_e2.size = %zu\n", Y_e2.size()); // bug5
            r1 = rmin_level;
            return;
        }
        int col_rk = std::distance(Y_e2.begin(), std::min_element(Y_e2.begin(), Y_e2.end())); 
        double min_dy = Y_d[col_rk];                                                          
        auto c_y = std::find(Y_E.begin(), Y_E.end(), min_dy);                                 
        auto c_my = std::distance(Y_E.begin(), c_y);                                          
        r1 = r_all[c_my];
    }
    /////////////////////////////////////////////////////////////////////////////////////////////

    ////////  /////////
    int Calc_Path2(double start[3], double end[3], double end_collision[3], double r) // 24.2.27
    {
        pathpoint3.clear();
        std::vector<double> point_r2x;
        std::vector<double> point_r2y;
        std::vector<double> point_r2th;
        ///
        point_r2x.push_back(start[0]);
        point_r2y.push_back(start[1]);
        point_r2th.push_back(start[2]);

        PathPoint mid_L;
        mid_L.x = start[0];
        mid_L.y = start[1];
        mid_L.th = mod2pi(start[2]);
        mid_L.D = 0;
        mid_L.delta = 0;
        pathpoint3.push_back(mid_L);
        double r_1 = r;
        double D = -start[0] + end[0] + backpath;
        double D_L = -0.1;
        int i = floor(fabs(D) / pathfind_parameters.MOTION_RESOLUTION);
        double delta_L = atan(vehicle_parameters.WB / r_1);
        int flag = 0;
        for (int idx_L = 0; idx_L < i; idx_L++) //
        {
            VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D_L, delta_L); 
            if (g_pth + 0.01745 > end[2] && g_pth - 0.01745 < end[2])
            {
                flag = 1;
                break;
            }
            mid_L.x = g_px;
            mid_L.y = g_py;
            mid_L.th = g_pth;
            mid_L.D = D_L;
            mid_L.delta = delta_L;
            pathpoint3.push_back(mid_L);

            point_r2x.push_back(g_px);
            point_r2y.push_back(g_py);
            point_r2th.push_back(g_pth);
        }
        if (flag == 0)
        {
            if (fabs(D - i * D_L) > 0.01)
            {
                VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D - i * D_L, delta_L); 
                mid_L.x = g_px;
                mid_L.y = g_py;
                mid_L.th = g_pth;
                mid_L.D = D - i * D_L;
                mid_L.delta = delta_L;
                pathpoint3.push_back(mid_L);
                point_r2x.push_back(g_px);
                point_r2y.push_back(g_py);
                point_r2th.push_back(g_pth);
            }
        }
        ///////

        ////////////  ////////
        std::vector<double> Y_c2;
        std::vector<double> X_c;
        std::vector<double> Y_c;
        std::vector<double> th_c;
        std::vector<double> Y_cc2;
        std::vector<double> Y_d;
        int p_r2x = point_r2x.size();
        double sta[3];
        for (int ii = 0; ii < p_r2x; ii++) 
        {
            sta[0] = point_r2x[ii];
            sta[1] = point_r2y[ii];
            sta[2] = point_r2th[ii];
            /////  /////
            if (sta[0] + 0.3 > end[0] + frontpath) 
            {
                continue; 
            }
            X_c.push_back(sta[0]);
            Y_c.push_back(sta[1]);
            th_c.push_back(sta[2]);

            rk_sto[0] = sta[0];
            rk_sto[1] = sta[1];
            rk_sto[2] = sta[2];
            ///
            double road_roll[28] = {0};
            int a1 = Roll_park(sta, end, right_left, backpath, frontpath, road_roll);
            if (a1 == 2)
            {
                getroadpath_Level_all(sta, road_roll, 0, 0);
            }
            PathPoint &last_element = pathpoint.back();
            double last_y = last_element.y;
            Y_c2.push_back(last_y);
            pathpoint.clear();
        }
        //
        printf("Y_c2.size = %zu\n", Y_c2.size()); // bug
        for (double num : Y_c2) ////
        {
            if (right_left == 1)
            {
                if (num >= end[1] - 0.01) 
                {
                    Y_d.push_back(num);
                    num -= end[1];
                    Y_cc2.push_back(std::abs(num)); 
                }
            }
            else
            {
                if (num <= end[1] + 0.01) 
                {
                    Y_d.push_back(num);
                    num -= end[1];
                    Y_cc2.push_back(std::abs(num)); 
                }
            }
        }
        if (Y_cc2.size() == 0)
        {
            printf("Y_cc2.size = %zu\n", Y_cc2.size());
            return 0;
        }
        int col_rk = std::distance(Y_cc2.begin(), std::min_element(Y_cc2.begin(), Y_cc2.end())); 
        double min_dy = Y_d[col_rk];
        auto c_y = std::find(Y_c2.begin(), Y_c2.end(), min_dy);
        auto c_my = std::distance(Y_c2.begin(), c_y);
        end_collision[0] = X_c[c_my];
        end_collision[1] = Y_c[c_my];
        end_collision[2] = th_c[c_my];
        ////////////////////////
        auto rkx = std::find(point_r2x.begin(), point_r2x.end(), end_collision[0]); 
        int c_rkx = std::distance(point_r2x.begin(), rkx);                          
        pathpoint3.erase(pathpoint3.begin() + c_rkx + 1, pathpoint3.end());
        pathpoint = pathpoint3;

        return 1;
    }
    //////////////////////////////////////////////////////////

    int Roll_Cc_1(double Start1[3], double Start_collision_mid1[3], int right_left, double R_level_1, double D_level_1, double road_roll[28])
	{
		double r_Cc = rmin_level;
		double Start[3];
		Start[0] = Start1[0];
		Start[1] = Start1[1];
		Start[2] = Start1[2];
		double End_collision[3];
		double Start_collision_mid[3] = {0, 0, 0};
		Start_collision_mid[0] = Start_collision_mid1[0];
		Start_collision_mid[1] = Start_collision_mid1[1];
		Start_collision_mid[2] = Start_collision_mid1[2];
		int point_collision = 100;
		double Start_collision[3] = {0, 0, 0}; 
		double road_Cc[6] = {0, 0, 0, 0, 0, 0};

		double Croll_DR[20] = {0};
		for (int index_Roll5 = 0; index_Roll5 < 5; index_Roll5++) 
		{
			for (int index_fb = 0; index_fb < 15; index_fb++) 
			{
				double D_C = index_fb * 0.1 + 0.2; 
				Start_collision[0] = Start_collision_mid[0];
				Start_collision[1] = Start_collision_mid[1];
				Start_collision[2] = Start_collision_mid[2];
				int index_level_1 = 0;
				for (int index = 0; index < index_Roll5 + 1; index++)
				{
					index_level_1 = RoadCollision_Level_RD(Start_collision, End_collision, r_Cc * right_left, D_C, point_collision); // 前进会不会碰撞
					if (index_level_1 == 1)
					{
						break;
					}
					Croll_DR[index * 4] = r_Cc * right_left;
					Croll_DR[index * 4 + 1] = -D_C;
					Start_collision[0] = End_collision[0];
					Start_collision[1] = End_collision[1];
					Start_collision[2] = End_collision[2];
					int index_level_2 = RoadCollision_Level_RD(Start_collision, End_collision, r_Cc * -right_left, -D_C, point_collision); // 后退会不会碰撞
					Croll_DR[index * 4 + 2] = r_Cc * -right_left;
					if (index_level_2 == 1)
					{
						Croll_DR[index * 4 + 3] = point_collision * 0.1;
					}
					else
					{
						Croll_DR[index * 4 + 3] = D_C;
					}
					Start_collision[0] = End_collision[0];
					Start_collision[1] = End_collision[1];
					Start_collision[2] = End_collision[2];
				}
				if (index_level_1 == 0)
				{
					int index_level_CC = 1;
					if (choice_cc == 0)
					{
						index_level_CC = Cc_road(Start, End_collision, right_left, road_Cc);
						Start_collision[0] = Start[0] + 0.15 * cos(Start[2]); 
						Start_collision[1] = Start[1] + 0.15 * sin(Start[2]);
						Start_collision[2] = Start[2];
					}
					else
					{
						index_level_CC = Calc_Cm(Start[0], Start[1], Start[2], End_collision[0], End_collision[1], End_collision[2], 0, rmin_level, 0, road_Cc);
						if (right_left == 1) 
						{
							if (road_Cc[0] < 0) 
							{
								index_level_CC = 1;
							}
						}
						else
						{
							if (road_Cc[0] > 0)
							{
								index_level_CC = 1;
							}
						}
						Start_collision[0] = Start[0] + 0 * cos(Start[2]); 
						Start_collision[1] = Start[1] + 0 * sin(Start[2]);
						Start_collision[2] = Start[2];
						if (index_level_CC == 0)
						{
							index_level_CC = 1;
						}
						else
						{
							index_level_CC = 0;
						}
					}
					if (index_level_CC == 1)
					{
						for (int index_Cc = 0; index_Cc < 3; index_Cc++)
						{
							index_level_CC = RoadCollision_Level_RD(Start_collision, End_collision, road_Cc[index_Cc * 2], road_Cc[index_Cc * 2 + 1], point_collision);
							if (index_level_CC == 0)
							{
								Start_collision[0] = End_collision[0];
								Start_collision[1] = End_collision[1];
								Start_collision[2] = End_collision[2];
							}
							else
							{
								break;
							}
						}
					}
					else
					{
						index_level_CC = 1;
					}
					if (index_level_CC == 0)
					{
						road_roll[0] = road_Cc[0];
						if (road_Cc[1] > 0)
						{
							road_roll[1] = road_Cc[1] + 0.5;
						}
						else
						{
							road_roll[1] = road_Cc[1];
						}
						road_roll[2] = road_Cc[2];
						road_roll[3] = road_Cc[3];
						road_roll[4] = road_Cc[4];
						road_roll[5] = road_Cc[5];
						road_roll[6] = Croll_DR[18];
						road_roll[7] = Croll_DR[19];
						road_roll[8] = Croll_DR[16];
						road_roll[9] = Croll_DR[17];
						road_roll[10] = Croll_DR[14];
						road_roll[11] = Croll_DR[15];
						road_roll[12] = Croll_DR[12];
						road_roll[13] = Croll_DR[13];
						road_roll[14] = Croll_DR[10];
						road_roll[15] = Croll_DR[11];
						road_roll[16] = Croll_DR[8];
						road_roll[17] = Croll_DR[9];
						road_roll[18] = Croll_DR[6];
						road_roll[19] = Croll_DR[7];
						road_roll[20] = Croll_DR[4];
						road_roll[21] = Croll_DR[5];
						road_roll[22] = Croll_DR[2];
						road_roll[23] = Croll_DR[3];
						road_roll[24] = Croll_DR[0];
						road_roll[25] = Croll_DR[1];
						road_roll[26] = R_level_1;
						road_roll[27] = D_level_1;
						return 1;
					}
				}
			}
		}
		return 0;
	}
}
