#include "global_variable.h"
#include "global_function.h"

namespace byd_apa_plan
{
    /////// 函数声明 /////
    int Dynamic_plan_level(double start1[3], double end1[3]);
    void loop_r(double start1[3], double end1[3], double &r1);
    void loop_R_th(double start1[3], double end1[3], double &R1, double d_y, double &theta1);
    void getroadpath_12(double start1[3], double end1[3], double R, double theta, double end_collision[3]);
    void calculate_coordinate_f(double x_o, double y_o, double theta1, double r, double &g_px, double &g_py, double &g_pth);
    void calculate_coordinate_b(double x_o, double y_o, double theta1, double r, double &g_px, double &g_py, double &g_pth);
    int Angle_judgment(double g_pth, double g_py, double end[3]);
    void Calc_Path2(double start[3], double end[3], double end_collision[3], double r);            // 24.2.27
    int roll_level_park_1(double Start1[3], double End1[3], int right_left, double road_roll[28]); // 2.27

    bool VehicleCollisionGridLevel(double cpx, double cpy, double cph);
    void ChangeFusionMap();
    int Roll_park(double Start1[3], double End1[3], int right_left, double backpath, double frontpath, double road_roll[28]);
    void getroadpath_Level_all(double Start[3], double road_roll[28], double D, double R);
    int geometry_Level(double Start1[3], double End1[3]);
    int roll_level_park(double Start1[3], double End1[3], int right_left, double road_roll[28]);
    int RoadCollision_Level(double Start[3], double End[3], double road_yaw[6]);

    int RoadCollision_Level_RD(double Start[3], double End[3], double R, double D, int &point_collision);
    int Cc_road(double Start[3], double End[3], int right_left, double road_Cc[6]);
    int Roll_Cc(double Start1[3], double Start_collision_mid1[3], int right_left, double R_level_1, double D_level_1, double road_roll[28]); // 2.27
    //////////////////////////////////////////

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
        double Y_C_P = (start[1] - end[1]) * cos(end[2]) - (start[0] - end[0]) * sin(end[2]); /// 验证当前点距离目标点的y向距离

        if (fabs(Y_C_P) > 0.4)
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
                    end_1[1] = end[1] - (d_ey - 1) * 0.01;
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
        if (d_x > 6 && fabs(Y_C_P) > 0.4)
        {
            loop_R_th(start, end, R_c, Y_C_P, theta); // 可能求不出解，需要改进
            if (R_c == 0 && theta == 0)               // 1.25 //
            {
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
            printf("遍历\n");
            if (a1 == 1)
            {
                return 1;
            }
            else if (a1 == 2)
            {
                getroadpath_Level_all(rk_start, road_roll, 0, 0);
                return 1;
            }
            else
            {
                return 0;
            }
        }
        else
        {
            double r;
            double end_collision1[3];
            if (end[1] - end_init[1] > 0) // 车位向上变化半径为最小转弯半径；
            {
                r = rmin_level;
            }
            else
            {
                loop_r(start, end, r); // 车位向下变化，通过遍历获得转弯半径
            }
            if (right_left == 1)
            {
                Calc_Path2(start, end, end_collision1, r);
            }
            else
            {
                Calc_Path2(start, end, end_collision1, -r);
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
            printf("a1=%d\n", a1);
            printf("遍历\n");
            if (a1 == 1)
            {
                return 1;
            }
            else if (a1 == 2)
            {
                getroadpath_Level_all(rk_start, road_roll, 0, 0);
                return 1;
            }
            else
            {
                return 0;
            }
        }
    }
    //////////////////////////////////////////

    /////////        遍历r       /////////////
    void loop_r(double start1[3], double end1[3], double &r1)
    {
        double start[3];
        start[0] = start1[0];
        start[1] = start1[1];
        start[2] = start1[2];
        double end[3];
        end[0] = end1[0];
        end[1] = end1[1];
        end[2] = end1[2];
        double start_0[3];
        double g_px = 0;
        double g_py = 0;
        double g_pth = 0;
        double x_o;
        double y_o;
        /// 创建动态容器
        std::vector<double> Y_r;
        std::vector<double> r_all;
        std::vector<double> Y_r1;
        if (right_left == 1)
        {
            for (double r = rmin_level; r <= 7; r += 0.3)
            {
                r_all.push_back(r);
                for (int j = 2; j <= 9; j++)
                {
                    int gears_DR = j % 2;
                    if (gears_DR == 0) // 后退
                    {
                        if (j == 2)
                        {
                            start_0[0] = start[0];
                            start_0[1] = start[1];
                            start_0[2] = start[2];
                        }
                        else
                        {
                            start_0[0] = g_px;
                            start_0[1] = g_py;
                            start_0[2] = g_pth;
                        }
                        ////// 计算圆弧的运动圆心 /////
                        x_o = start_0[0] - r * sin(start_0[2]);
                        y_o = start_0[1] + r * cos(start_0[2]);
                        for (double i_r1 = start_0[2] - pi / 2; i_r1 >= -pi / 2; i_r1 -= asin(0.1 / r)) // i_r1 -= 0.01724
                        {
                            calculate_coordinate_b(x_o, y_o, i_r1, r, g_px, g_py, g_pth);
                            ///// 碰撞检测 /////
                            int roadisCollision = 0;
                            roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth); /// 碰撞检测
                            if (roadisCollision || (g_px < end[0] + backpath))              // 如果过碰撞则为上一点坐标
                            {
                                start_0[0] = x_o + r * sin(i_r1 + pi / 2 + asin(0.1 / r)); /// i_r1 + pi / 2 + 0.01724
                                start_0[1] = y_o - r * cos(i_r1 + pi / 2 + asin(0.1 / r));
                                start_0[2] = i_r1 + pi / 2 + asin(0.1 / r);
                                break;
                            }
                            ////
                            if (Angle_judgment(g_pth, g_py, end) == 1)
                            {
                                break;
                            }
                        }
                    }
                    else if (gears_DR == 1) // 前进
                    {
                        ////// 计算圆弧的运动圆心 /////
                        x_o = start_0[0] + r * sin(start_0[2]);
                        y_o = start_0[1] - r * cos(start_0[2]);
                        for (double i_r2 = start_0[2] + pi / 2; i_r2 >= pi / 2; i_r2 -= asin(0.1 / r)) // i_r2 -= 0.01724
                        {
                            calculate_coordinate_f(x_o, y_o, i_r2, r, g_px, g_py, g_pth);
                            ///// 碰撞检测 /////
                            if ((g_px > end[0] + frontpath)) // (g_px < end[0] + backpath) ||
                            {
                                start_0[0] = x_o - r * sin(i_r2 - pi / 2 + asin(0.1 / r));
                                start_0[1] = y_o + r * cos(i_r2 - pi / 2 + asin(0.1 / r));
                                start_0[2] = i_r2 - pi / 2 + asin(0.1 / r);
                                break;
                            }
                            ////
                            if (Angle_judgment(g_pth, g_py, end) == 1)
                            {
                                break;
                            }
                        }
                    }
                }
                if ((g_px - end[0]) > 0.01) // 为了区分行走的方向
                {
                    double x_aim = end[0];
                    double y_aim = tan(g_pth) * (x_aim - g_px) + g_py; // 直线求解y向的终点
                    Y_r.push_back(y_aim);
                }
                else if ((g_px - end[0]) < -0.01)
                {
                    double x_aim = end[0];
                    double y_aim = tan(g_pth) * (x_aim - g_px) + g_py; // 直线求解y向的终点
                    Y_r.push_back(y_aim);
                }
            }
            for (double num : Y_r) ////
            {
                num -= end[1];
                Y_r1.push_back(std::abs(num)); // 将每个元素减去一个值并赋予另一个集合中
            }
        }
        else if (right_left == -1)
        {
            for (double r = rmin_level; r <= 7; r += 0.3)
            {
                r_all.push_back(r);
                for (int j = 2; j <= 9; j++)
                {
                    int gears_DR = j % 2;
                    if (gears_DR == 0) // 后退
                    {
                        if (j == 2)
                        {
                            start_0[0] = start[0];
                            start_0[1] = start[1];
                            start_0[2] = start[2];
                        }
                        else
                        {
                            start_0[0] = g_px;
                            start_0[1] = g_py;
                            start_0[2] = g_pth;
                        }
                        ////// 计算圆弧的运动圆心 /////
                        x_o = start_0[0] - r * sin(abs(start_0[2]));
                        y_o = start_0[1] - r * cos(abs(start_0[2]));
                        for (double i_r1 = start_0[2] + pi / 2; i_r1 <= pi / 2; i_r1 += asin(0.1 / r)) // i_r1 -= 0.01724
                        {
                            calculate_coordinate_b(x_o, y_o, i_r1, r, g_px, g_py, g_pth);
                            ///// 碰撞检测 /////
                            int roadisCollision = 0;
                            roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth); /// 碰撞检测
                            if (roadisCollision || (g_px < end[0] + backpath))              // 如果过碰撞则为上一点坐标
                            {
                                start_0[0] = x_o + r * sin(-i_r1 + pi / 2 + asin(0.1 / r)); /// i_r1 + pi / 2 + 0.01724
                                start_0[1] = y_o + r * cos(-i_r1 + pi / 2 + asin(0.1 / r));
                                start_0[2] = i_r1 - pi / 2 - asin(0.1 / r);
                                break;
                            }
                            ////
                            if (Angle_judgment(g_pth, g_py, end) == 1)
                            {
                                break;
                            }
                        }
                    }
                    else if (gears_DR == 1)
                    {
                        ////// 计算圆弧的运动圆心 /////
                        x_o = start_0[0] + r * sin(abs(start_0[2]));
                        y_o = start_0[1] + r * cos(abs(start_0[2]));
                        for (double i_r2 = start_0[2] - pi / 2; i_r2 <= -pi / 2; i_r2 += asin(0.1 / r)) // i_r2 -= 0.01724
                        {
                            calculate_coordinate_f(x_o, y_o, i_r2, r, g_px, g_py, g_pth);
                            ///// 碰撞检测 /////
                            if ((g_px > end[0] + frontpath)) // (g_px < end[0] + backpath) ||
                            {
                                start_0[0] = x_o - r * sin(abs(i_r2 + pi / 2 - asin(0.1 / r)));
                                start_0[1] = y_o - r * cos(abs(i_r2 + pi / 2 - asin(0.1 / r)));
                                start_0[2] = i_r2 + pi / 2 - asin(0.1 / r);
                                break;
                            }
                            ////
                            if (Angle_judgment(g_pth, g_py, end) == 1)
                            {
                                break;
                            }
                        }
                    }
                }
                if ((g_px - end[0]) > 0.01) // 为了区分行走的方向
                {
                    double x_aim = end[0];
                    double y_aim = tan(g_pth) * (x_aim - g_px) + g_py; // 直线求解y向的终点
                    Y_r.push_back(y_aim);
                }
                else if ((g_px - end[0]) < -0.01)
                {
                    double x_aim = end[0];
                    double y_aim = tan(g_pth) * (x_aim - g_px) + g_py; // 直线求解y向的终点
                    Y_r.push_back(y_aim);
                }
            }
            for (double num : Y_r) ////
            {
                num -= end[1];
                Y_r1.push_back(std::abs(num)); // 将每个元素减去一个值并赋予另一个集合中
            }
        }
        int col_R_th = std::distance(Y_r1.begin(), std::min_element(Y_r1.begin(), Y_r1.end())); // 最小y偏差值所在的位置
        r1 = r_all[col_R_th];                                                                   // 对应顺序的r1
    }
    //////////////////////////////////////////

    ///////// 遍历获得R和theta   /////////////
    void loop_R_th(double start1[3], double end1[3], double &R1, double d_y, double &theta1) //// & 符号通常与参数的类型一起写在一起，用于指示该参数是一个引用；
    {
        double start[3];   // 初始化参数
        double start_0[3]; // 储存除了第一次圆弧的运动起点，之后每次运动的起点
        double end[3];
        start[0] = start1[0];
        start[1] = start1[1];
        start[2] = start1[2];
        end[0] = end1[0];
        end[1] = end1[1];
        end[2] = end1[2];
        double r = rmin_level;
        double x1_o;
        double y1_o;
        double x2_o;
        double y2_o;
        double rk_xo;
        double rk_yo;
        double g_px;
        double g_py;
        double g_pth;
        int flag = 0;
        int ffla = 0;
        /// 创建动态容器
        std::vector<double> th_cc;
        std::vector<double> th_y2; /// 存储对应的角度关系
        std::vector<double> Y_y;
        std::vector<double> R_cc;
        std::vector<double> Y_y1; // 存储每个R对应的实际y值和目标y值之间相对值；
        std::vector<double> Y_y2; // 存储相对值最小的实际y值

        if (right_left == 1) // 右泊
        {
            for (double R = rmin_level; R <= 8; R += 0.15) //// 减少计算，设置R每次变化0.5m // 1.23
            {
                R_cc.push_back(R); // 插入值
                Y_y.clear();
                th_cc.clear();
                Y_y1.clear();
                /// 每个R执行时需要清除th_cc
                for (double theta = (pi / 2 - start[2]); theta >= 30 * pi / 180; theta -= asin(0.1 / R)) //// 角度搜索，每次0.0125弧度，对应半径10cm
                {
                    th_cc.push_back(theta);
                    flag = 0;
                    ffla = 0;
                    for (int i_xh = 1; i_xh <= 10; i_xh++)
                    {
                        int gears_DR = i_xh % 2;
                        if (i_xh == 1) // 一段圆弧运动
                        {
                            ////// 计算一段圆弧的运动圆心 /////
                            x1_o = start[0] + R * sin(start[2]);
                            y1_o = start[1] - R * cos(start[2]);
                            for (double i1 = pi / 2 + start[2]; i1 <= pi - theta + 0.0001; i1 += asin(0.1 / R))
                            {
                                g_px = x1_o - R * sin(i1 - pi / 2);
                                g_py = y1_o + R * cos(i1 - pi / 2);
                                g_pth = i1 - pi / 2;
                                ///// 碰撞检测 /////
                                int roadisCollision = 0;
                                roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth); /// 碰撞检测
                                if (roadisCollision)                                            /// 为下一次起点赋值
                                {
                                    start_0[0] = x1_o - R * sin(i1 - pi / 2 - asin(0.1 / R)); // 如果过碰撞则为上一点坐标
                                    start_0[1] = y1_o + R * cos(i1 - pi / 2 - asin(0.1 / R));
                                    start_0[2] = i1 - pi / 2 - asin(0.1 / R);
                                    break;
                                }
                                if (i1 + asin(0.1 / R) > pi - theta) /// 为下一次起点赋值
                                {
                                    start_0[0] = g_px;
                                    start_0[1] = g_py;
                                    start_0[2] = g_pth;
                                }
                            }
                            continue; /////
                        }
                        ////////  二段圆弧运动 ///////
                        else if (gears_DR == 0) //// 后退
                        {
                            ////// 计算二段圆弧的运动圆心 /////
                            x2_o = start_0[0] - r * sin(start_0[2]);
                            y2_o = start_0[1] + r * cos(start_0[2]);
                            for (double i2 = start_0[2] - pi / 2; i2 >= -pi / 2; i2 -= asin(0.1 / r))
                            {
                                calculate_coordinate_b(x2_o, y2_o, i2, r, g_px, g_py, g_pth);
                                ///// 碰撞检测 ///// 有问题
                                int roadisCollision = 0;
                                roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth); /// 碰撞检测
                                if (roadisCollision || (g_px < end[0] + backpath))              // 如果过碰撞则为上一点坐标
                                {
                                    start_0[0] = x2_o + r * sin(i2 + pi / 2 + asin(0.1 / r));
                                    start_0[1] = y2_o - r * cos(i2 + pi / 2 + asin(0.1 / r));
                                    start_0[2] = i2 + pi / 2 + asin(0.1 / r);
                                    // 24.1.11
                                    if ((start_0[0] > end[0] + 0.1)) // 限制x坐标，避免车头右前方发生碰撞
                                    {
                                        flag = 1;
                                        ffla = 1;
                                    }
                                    // 24.1.11
                                    break;
                                }
                                ////
                                if (Angle_judgment(g_pth, g_py, end) == 1)
                                {
                                    flag = 1;
                                    break;
                                }
                            }
                        }
                        else if (gears_DR == 1) /// 前进
                        {
                            ////// 计算圆弧的运动圆心 /////
                            rk_xo = start_0[0] + r * sin(start_0[2]);
                            rk_yo = start_0[1] - r * cos(start_0[2]);
                            for (double i3 = pi / 2 + start_0[2]; i3 >= pi / 2; i3 -= asin(0.1 / r))
                            {
                                calculate_coordinate_f(rk_xo, rk_yo, i3, r, g_px, g_py, g_pth);
                                ///// 碰撞检测 /////
                                if ((g_px > end[0] + frontpath)) // (g_px < end[0] + backpath) ||
                                {
                                    start_0[0] = rk_xo - r * sin(i3 - pi / 2 + asin(0.1 / r));
                                    start_0[1] = rk_yo + r * cos(i3 - pi / 2 + asin(0.1 / r));
                                    start_0[2] = i3 - pi / 2 + asin(0.1 / r);
                                    break;
                                }
                                ////
                                if (Angle_judgment(g_pth, g_py, end) == 1)
                                {
                                    flag = 1;
                                    break;
                                }
                            }
                        }
                        if (flag == 1)
                        {
                            break;
                        }
                    }
                    //// 24.1.12 ////
                    if (ffla == 1)
                    {
                        th_cc.pop_back();
                        continue;
                    }
                    //// 24.1.12 ////
                    if ((g_px - end[0]) > 0.01) // 为了区分行走的方向
                    {
                        double x_aim = end[0];
                        double y_aim = tan(g_pth) * (x_aim - g_px) + g_py; // 直线求解y向的终点
                        Y_y.push_back(y_aim);
                    }
                    else if ((g_px - end[0]) < -0.01)
                    {
                        double x_aim = end[0];
                        double y_aim = tan(g_pth) * (x_aim - g_px) + g_py; // 直线求解y向的终点
                        Y_y.push_back(y_aim);                              /// 存入每次的y值
                    }
                }
                for (double num : Y_y)
                {
                    num -= end[1];
                    Y_y1.push_back(std::abs(num)); // 将每个元素减去一个值并赋予另一个集合中
                }
                if (Y_y1.empty())
                {
                    continue;
                }
                else
                {
                    double y_m = *std::min_element(Y_y1.begin(), Y_y1.end());                                // 找出Y_y1最小的元素
                    int col_y2_th = std::distance(Y_y1.begin(), std::min_element(Y_y1.begin(), Y_y1.end())); // 最小y偏差值所在的位置
                    th_y2.push_back(th_cc[col_y2_th]);                                                       //// 最小y值偏差对应的角度；
                    Y_y2.push_back(y_m);
                }

                /// 24.1.12 ///
                if (ffla == 1 && flag == 1)
                {
                    continue;
                }
                /// 24.1.12 ///
            }
        }
        else if (right_left == -1)
        {
            for (double R = rmin_level; R <= 8.5; R += 0.3) //// 减少计算，设置R每次变化0.5m
            {
                R_cc.push_back(R); // 插入值
                Y_y.clear();
                th_cc.clear();
                Y_y1.clear();
                /// 每个R执行时需要清除th_cc
                for (double theta = (pi / 2 - abs(start[2])); theta >= 30 * pi / 180; theta -= asin(0.1 / R)) //// 角度搜索，每次0.0125弧度，对应半径10cm
                {
                    th_cc.push_back(theta);
                    flag = 0;
                    ffla = 0;
                    for (int i_xh = 1; i_xh <= 10; i_xh++)
                    {
                        int gears_DR = i_xh % 2;
                        if (i_xh == 1) // 一段圆弧运动
                        {
                            ////// 计算一段圆弧的运动圆心 /////
                            x1_o = start[0] + R * sin(abs(start[2]));
                            y1_o = start[1] + R * cos(abs(start[2]));
                            for (double i1 = -pi / 2 + start[2]; i1 >= -pi + theta - 0.0001; i1 -= asin(0.1 / R))
                            {
                                g_px = x1_o - R * sin(abs(i1 + pi / 2));
                                g_py = y1_o - R * cos(abs(i1 + pi / 2));
                                g_pth = i1 + pi / 2;
                                ///// 碰撞检测 /////
                                int roadisCollision = 0;
                                roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth); /// 碰撞检测
                                if (roadisCollision)                                            /// 为下一次起点赋值
                                {
                                    start_0[0] = x1_o - R * sin(abs(i1 + pi / 2 + asin(0.1 / R))); // 如果过碰撞则为上一点坐标
                                    start_0[1] = y1_o - R * cos(abs(i1 + pi / 2 + asin(0.1 / R)));
                                    start_0[2] = i1 + pi / 2 + asin(0.1 / R);
                                    break;
                                }
                                if (i1 - asin(0.1 / R) < -pi + theta) /// 为下一次起点赋值
                                {
                                    start_0[0] = g_px;
                                    start_0[1] = g_py;
                                    start_0[2] = g_pth;
                                }
                            }
                            continue; /////
                        }
                        ////////  二段圆弧运动 ///////
                        else if (gears_DR == 0) //// 后退
                        {
                            ////// 计算二段圆弧的运动圆心 /////
                            x2_o = start_0[0] - r * sin(abs(start_0[2]));
                            y2_o = start_0[1] - r * cos(abs(start_0[2]));
                            for (double i2 = start_0[2] + pi / 2; i2 <= pi / 2; i2 += asin(0.1 / r))
                            {
                                calculate_coordinate_b(x2_o, y2_o, i2, r, g_px, g_py, g_pth);
                                ///// 碰撞检测 /////
                                int roadisCollision = 0;
                                roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth); /// 碰撞检测
                                if (roadisCollision || (g_px < end[0] + backpath))              // 如果过碰撞则为上一点坐标
                                {
                                    start_0[0] = x2_o + r * sin(-i2 + pi / 2 + asin(0.1 / r));
                                    start_0[1] = y2_o + r * cos(-i2 + pi / 2 + asin(0.1 / r));
                                    start_0[2] = i2 - pi / 2 - asin(0.1 / r);
                                    // 24.1.11
                                    if ((start_0[0] > end[0] + 0.1)) //&& (i_xh == 2)
                                    {
                                        flag = 1;
                                        ffla = 1;
                                    }
                                    // 24.1.11
                                    break;
                                }
                                ////
                                if (Angle_judgment(g_pth, g_py, end) == 1)
                                {
                                    flag = 1;
                                    break;
                                }
                            }
                        }
                        else if (gears_DR == 1) /// 前进
                        {
                            ////// 计算圆弧的运动圆心 /////
                            rk_xo = start_0[0] + r * sin(abs(start_0[2]));
                            rk_yo = start_0[1] + r * cos(abs(start_0[2]));
                            for (double i3 = -pi / 2 + start_0[2]; i3 <= -pi / 2; i3 += asin(0.1 / r))
                            {
                                calculate_coordinate_f(rk_xo, rk_yo, i3, r, g_px, g_py, g_pth);
                                ///// 碰撞检测 /////
                                if ((g_px > end[0] + frontpath)) // (g_px < end[0] + backpath) ||
                                {
                                    start_0[0] = rk_xo - r * sin(abs(i3 + pi / 2 - asin(0.1 / r)));
                                    start_0[1] = rk_yo - r * cos(abs(i3 + pi / 2 - asin(0.1 / r)));
                                    start_0[2] = i3 + pi / 2 - asin(0.1 / r);
                                    break;
                                }
                                ////
                                if (Angle_judgment(g_pth, g_py, end) == 1)
                                {
                                    flag = 1;
                                    break;
                                }
                            }
                        }
                        if (flag == 1)
                        {
                            break;
                        }
                    }
                    //// 24.1.12 ////
                    if (ffla == 1)
                    {
                        th_cc.pop_back();
                        continue;
                    }
                    //// 24.1.12 ////
                    if ((g_px - end[0]) > 0.01) // 为了区分行走的方向
                    {
                        double x_aim = end[0];
                        double y_aim = tan(g_pth) * (x_aim - g_px) + g_py; // 直线求解y向的终点
                        Y_y.push_back(y_aim);
                    }
                    else if ((g_px - end[0]) < -0.01)
                    {
                        double x_aim = end[0];
                        double y_aim = tan(g_pth) * (x_aim - g_px) + g_py; // 直线求解y向的终点
                        Y_y.push_back(y_aim);                              /// 存入每次的y值
                    }
                }
                for (double num : Y_y) ////
                {
                    num -= end[1];
                    Y_y1.push_back(std::abs(num)); // 将每个元素减去一个值并赋予另一个集合中
                }
                if (Y_y1.empty())
                {
                    continue;
                }
                else
                {
                    double y_m = *std::min_element(Y_y1.begin(), Y_y1.end());                                // 找出Y_y1最小的元素
                    int col_y2_th = std::distance(Y_y1.begin(), std::min_element(Y_y1.begin(), Y_y1.end())); // 最小y偏差值所在的位置
                    th_y2.push_back(th_cc[col_y2_th]);                                                       //// 最小y值偏差对应的角度；
                    Y_y2.push_back(y_m);
                }
                /// 24.1.12 ///
                if (ffla == 1 && flag == 1)
                {
                    continue;
                }
                /// 24.1.12 ///
            }
        }
        if (Y_y2.empty())
        {
            R1 = 0;
            theta1 = 0;
        }
        else
        {
            int col_R_th = std::distance(Y_y2.begin(), std::min_element(Y_y2.begin(), Y_y2.end())); // 最小y偏差值所在的位置
            R1 = R_cc[col_R_th];                                                                    // 对应顺序的R
            theta1 = th_y2[col_R_th];
        } // 对应顺序的theta
    }
    //////////////////////////////////////////

    /////////  获取12段圆弧的路径点,求出揉库开始点  /////////////
    void getroadpath_12(double start1[3], double end1[3], double R, double theta1, double end_collision1[3])
    {
        path_point mid_L;
        double start[3];
        start[0] = start1[0];
        start[1] = start1[1];
        start[2] = start1[2];
        double end[3];
        end[0] = end1[0];
        end[1] = end1[1];
        end[2] = end1[2];
        double start_0[3];
        double theta = theta1;
        double r = rmin_level;

        double x1_o;
        double y1_o;
        double x2_o;
        double y2_o;
        double g_px;
        double g_py;
        double g_pth;
        double flag = 0;
        if (right_left == 1)
        {
            while (1)
            {
                ////// 计算一段圆弧的运动圆心 /////
                x1_o = start[0] + R * sin(start[2]);
                y1_o = start[1] - R * cos(start[2]);
                for (double i1 = pi / 2 + start[2]; i1 <= pi - theta + 0.0001; i1 += asin(0.1 / R)) ///  + 0.0001是为了确保误差
                {
                    g_px = x1_o - R * sin(i1 - pi / 2);
                    g_py = y1_o + R * cos(i1 - pi / 2);
                    g_pth = i1 - pi / 2;
                    /// 碰撞检测 ///
                    int roadisCollision = 0;
                    roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth); /// 碰撞检测
                    if (roadisCollision)                                            // 如果碰撞则为上一点坐标
                    {
                        end_collision1[0] = x1_o - R * sin(i1 - pi / 2 + asin(0.1 / R));
                        end_collision1[1] = y1_o + R * cos(i1 - pi / 2 + asin(0.1 / R));
                        end_collision1[2] = i1 - pi / 2 + asin(0.1 / R);
                        flag = 1;
                        break;
                    }
                    mid_L.x = g_px;
                    mid_L.y = g_py;
                    mid_L.th = g_pth;
                    double D_L = -0.1;
                    double delta_L = atan(vehicle_parameters.WB / R);
                    mid_L.D = D_L;
                    mid_L.delta = delta_L;
                    pathpoint.push_back(mid_L);
                }
                if (flag == 1) // 一段圆弧碰撞点后直接跳出
                {
                    break;
                }
                start_0[0] = g_px;
                start_0[1] = g_py;
                start_0[2] = g_pth;
                ////// 计算二段圆弧的运动圆心 /////
                x2_o = start_0[0] - r * sin(start_0[2]);
                y2_o = start_0[1] + r * cos(start_0[2]);
                for (double i2 = start_0[2] - pi / 2; i2 >= -pi / 2; i2 -= asin(0.1 / r))
                {
                    calculate_coordinate_b(x2_o, y2_o, i2, r, g_px, g_py, g_pth);
                    ///// 碰撞检测 /////
                    int roadisCollision = 0;
                    roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth); /// 碰撞检测
                    if (roadisCollision || (g_px < end[0] + backpath))              // 如果碰撞则为上一点坐标
                    {
                        end_collision1[0] = x2_o + r * sin(i2 + pi / 2 + asin(0.1 / r));
                        end_collision1[1] = y2_o - r * cos(i2 + pi / 2 + asin(0.1 / r));
                        end_collision1[2] = i2 + pi / 2 + asin(0.1 / r);
                        flag = 1;
                        break;
                    }
                    //// 记录不碰撞的点 ////
                    if (i2 != start_0[2] - pi / 2) // 除去第一个点
                    {
                        mid_L.x = g_px;
                        mid_L.y = g_py;
                        mid_L.th = g_pth;
                        double D_L = -0.1;
                        double delta_L = atan(vehicle_parameters.WB / r);
                        mid_L.D = D_L;
                        mid_L.delta = delta_L;
                        pathpoint.push_back(mid_L);
                    }
                    //// 角度循环完成且期间不碰撞的情况 ////
                    int xx_s2e = floor(start_0[2] / (asin(0.1 / r)));
                    double xx_th = start_0[2] - pi / 2 - (asin(0.1 / r)) * xx_s2e;
                    if (i2 >= xx_th - 0.000001 && i2 <= xx_th + 0.000001) /// 0.000001计算误差
                    {
                        end_collision1[0] = g_px;
                        end_collision1[1] = g_py;
                        end_collision1[2] = g_pth;
                        flag = 1;

                        break;
                    }
                }
                if (flag == 1)
                {
                    break;
                }
            }
        }
        else if (right_left == -1)
        {
            while (1)
            {
                ////// 计算一段圆弧的运动圆心 /////
                x1_o = start[0] + R * sin(abs(start[2]));
                y1_o = start[1] + R * cos(abs(start[2]));
                for (double i1 = -pi / 2 + start[2]; i1 >= -pi + theta - 0.0001; i1 -= asin(0.1 / R)) ///  + 0.0001是为了确保误差
                {
                    g_px = x1_o - R * sin(abs(i1 + pi / 2));
                    g_py = y1_o - R * cos(abs(i1 + pi / 2));
                    g_pth = i1 + pi / 2;
                    /// 碰撞检测 ///
                    int roadisCollision = 0;
                    roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth); /// 碰撞检测
                    if (roadisCollision)                                            // 如果碰撞则为上一点坐标
                    {
                        end_collision1[0] = x1_o - R * sin(abs(i1 + pi / 2 + asin(0.1 / R)));
                        end_collision1[1] = y1_o - R * cos(abs(i1 + pi / 2 + asin(0.1 / R)));
                        end_collision1[2] = i1 + pi / 2 + asin(0.1 / R);
                        flag = 1;
                        break;
                    }
                    mid_L.x = g_px;
                    mid_L.y = g_py;
                    mid_L.th = g_pth;
                    double D_L = -0.1;
                    double delta_L = atan(vehicle_parameters.WB / R);
                    mid_L.D = D_L;
                    mid_L.delta = delta_L;
                    pathpoint.push_back(mid_L);
                }
                if (flag == 1) // 一段圆弧碰撞点后直接跳出
                {
                    break;
                }
                start_0[0] = g_px;
                start_0[1] = g_py;
                start_0[2] = g_pth;
                ////// 计算二段圆弧的运动圆心 /////
                x2_o = start_0[0] - r * sin(abs(start_0[2]));
                y2_o = start_0[1] - r * cos(abs(start_0[2]));
                for (double i2 = start_0[2] + pi / 2; i2 <= pi / 2; i2 += asin(0.1 / r))
                {
                    calculate_coordinate_b(x2_o, y2_o, i2, r, g_px, g_py, g_pth);
                    ///// 碰撞检测 /////
                    int roadisCollision = 0;
                    roadisCollision = VehicleCollisionGridLevel(g_px, g_py, g_pth); /// 碰撞检测
                    if (roadisCollision || (g_px < end[0] + backpath))              // 如果碰撞则为上一点坐标
                    {
                        end_collision1[0] = x2_o + r * sin(-i2 + pi / 2 + asin(0.1 / r));
                        end_collision1[1] = y2_o + r * cos(-i2 + pi / 2 + asin(0.1 / r));
                        end_collision1[2] = i2 - pi / 2 - asin(0.1 / r);
                        flag = 1;
                        break;
                    }
                    //// 记录不碰撞的点 ////
                    mid_L.x = g_px;
                    mid_L.y = g_py;
                    mid_L.th = g_pth;
                    double D_L = -0.1;
                    double delta_L = atan(vehicle_parameters.WB / r);
                    mid_L.D = D_L;
                    mid_L.delta = delta_L;
                    pathpoint.push_back(mid_L);
                    //// 角度循环完成且期间不碰撞的情况 ////
                    int xx_s2e = floor(abs(start_0[2] / (asin(0.1 / r))));
                    double xx_th = start_0[2] + pi / 2 + (asin(0.1 / r)) * xx_s2e;
                    if (i2 >= xx_th - 0.000001 && i2 <= xx_th + 0.000001) /// 0.000001计算误差
                    {
                        end_collision1[0] = g_px;
                        end_collision1[1] = g_py;
                        end_collision1[2] = g_pth;
                        flag = 1;

                        break;
                    }
                }
                if (flag == 1)
                {
                    break;
                }
            }
        }
    }
    //////////////////////////////////////////

    /////////  坐标点计算  /////////////
    void calculate_coordinate_f(double x_o, double y_o, double theta1, double r, double &g_px, double &g_py, double &g_pth)
    {
        if (right_left == 1)
        {
            g_px = x_o - r * sin(theta1 - pi / 2);
            g_py = y_o + r * cos(theta1 - pi / 2);
            g_pth = theta1 - pi / 2;
        }
        else
        {
            g_px = x_o - r * sin(abs(theta1 + pi / 2));
            g_py = y_o - r * cos(abs(theta1 + pi / 2));
            g_pth = theta1 + pi / 2;
        }
    }
    void calculate_coordinate_b(double x_o, double y_o, double theta1, double r, double &g_px, double &g_py, double &g_pth)
    {
        if (right_left == 1)
        {
            g_px = x_o + r * sin(theta1 + pi / 2);
            g_py = y_o - r * cos(theta1 + pi / 2);
            g_pth = theta1 + pi / 2;
        }
        else
        {
            g_px = x_o + r * sin(-theta1 + pi / 2);
            g_py = y_o + r * cos(-theta1 + pi / 2);
            g_pth = theta1 - pi / 2;
        }
    }
    //////////////////////////////////////////

    /////////  角度判断  /////////////
    int Angle_judgment(double g_pth1, double g_py1, double end1[3])
    {
        double g_pth = g_pth1;
        double g_py = g_py1;
        double end[1] = {end1[1]};

        if ((bias_th - 0.01745 < g_pth) && (g_pth < bias_th + 0.01745) && (end[1] - 0.03 <= g_py) && (end[1] + 0.03 > g_py))
        {
            return 1; // 角度满足条件
        }
        else if ((bias_th - 0.01745 < g_pth) && (g_pth < bias_th + 0.01745)) /// 当该循环下角度遍历完成，即是车辆角度接近于0时，结束小循环，进入大循环
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    //////////////////////////////////////////

    //////// 计算行驶距离 /////////
    void Calc_Path2(double start[3], double end[3], double end_collision[3], double r) // 24.2.27
    {
        pathpoint3.clear();
        std::vector<double> point_r2x;
        std::vector<double> point_r2y;
        std::vector<double> point_r2th;
        ///
        point_r2x.push_back(start[0]);
        point_r2y.push_back(start[1]);
        point_r2th.push_back(start[2]);

        path_point mid_L;
        mid_L.x = start[0];
        mid_L.y = start[1];
        mid_L.th = mod2pi(start[2]);
        mid_L.D = 0;
        mid_L.delta = 0;
        pathpoint3.push_back(mid_L);
        double r_1 = r;
        double D = -start[0] + end_to_fus[0] + backpath;
        double D_L = -0.1;
        int i = floor(fabs(D) / pathfind_parameters.MOTION_RESOLUTION);
        double delta_L = atan(vehicle_parameters.WB / r_1);
        int flag = 0;
        for (int idx_L = 0; idx_L < i; idx_L++) // round()四舍五入
        {
            VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D_L, delta_L); // 根据当前位姿和输入, 计算下一位置的位姿
            if (g_pth + 0.01745 > end[2] && g_pth - 0.01745 < end[2]) // 3.11增加角度判断
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
                VehicleDynamic(mid_L.x, mid_L.y, mid_L.th, D - i * D_L, delta_L); // 根据当前位姿和输入, 计算下一位置的位姿
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

        //////////// 测试 ////////
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
            ///// 碰撞检测 /////
            if (sta[0] + 0.3 > end[0] + frontpath) // 水平大于30cm，则倾斜大于30cm
            {
                continue; // 后续找点时，需要加上跳过的点
            }
            X_c.push_back(sta[0]);
            Y_c.push_back(sta[1]);
            th_c.push_back(sta[2]);

            rk_sto[0] = sta[0];
            rk_sto[1] = sta[1];
            rk_sto[2] = sta[2];
            /// 进入揉库
            double road_roll[28] = {0};
            int a1 = Roll_park(sta, end, right_left, backpath, frontpath, road_roll);
            if (a1 == 2)
            {
                getroadpath_Level_all(sta, road_roll, 0, 0);
            }
            path_point &last_element = pathpoint.back();
            double last_y = last_element.y;
            Y_c2.push_back(last_y);
            pathpoint.clear();
        }
        for (double num : Y_c2) ////
        {
            if (right_left == 1)
            {
                if (num >= end[1] - 0.01) // 只允许向远离障碍物偏差
                {
                    Y_d.push_back(num);
                    num -= end[1];
                    Y_cc2.push_back(std::abs(num)); // 将每个元素减去一个值并赋予另一个集合中
                }
            }
            else
            {
                if (num <= end[1] + 0.01) // 只允许向远离障碍物偏差
                {
                    Y_d.push_back(num);
                    num -= end[1];
                    Y_cc2.push_back(std::abs(num)); // 将每个元素减去一个值并赋予另一个集合中
                }
            }
        }

        int col_rk = std::distance(Y_cc2.begin(), std::min_element(Y_cc2.begin(), Y_cc2.end())); // 最小y偏差值所在的位置
        double min_dy = Y_d[col_rk];
        auto c_y = std::find(Y_c2.begin(), Y_c2.end(), min_dy);
        auto c_my = std::distance(Y_c2.begin(), c_y);
        end_collision[0] = X_c[c_my];
        end_collision[1] = Y_c[c_my];
        end_collision[2] = th_c[c_my];
        ////////////////////////
        /* auto rkx = std::find(point_r2x.begin(), point_r2x.end(), end_collision[0]); // zhaochu 对应的元素
        int c_rkx = std::distance(point_r2x.begin(), rkx);                          // 找出元素对应的序号
        pathpoint3.erase(pathpoint3.begin() + c_rkx + 1, pathpoint3.end());
        pathpoint = pathpoint3; */
    }
    ////////////////////////

    /////// 2.27 /////////////
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
        double R_level_1 = 1000000;
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
                        return 0;
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
        ///////////////////////////揉库求解/////////////////////////////////////////////
        // 30cm如果碰撞就用20cm的C+C-揉库，20cm不做碰撞检测，防止规划失败，20cm相当于靠逼停带偏差泊入 20cm到100cm
        int level_return = Roll_Cc(Start, Start_collision_mid, right_left, R_level_1, D_level_1, road_roll);
        if (level_return == 1)
        {
            return 1;
        }
        return 0;
    }
}
