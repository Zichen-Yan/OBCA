#ifndef GLOBAL_VARIABLE_H
#define GLOBAL_VARIABLE_H


#include <iostream>
#include <fstream>
#include <vector>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unordered_map>
#include <queue>
#include <algorithm>

#include "./include/log.h"
#include "./include/def_types.h"
#include "./include/util_time.h"
#include "./include/json.hpp"
#include "base_data_struct.h"
#include <climits>

#ifndef DEBUG_PC
	#include "../msg/apa_api.h"
	#include <bclcpp/bclcpp.hpp>
#endif
namespace byd_apa_plan
{
	#define pi 3.141592653589793
	#define twopi 6.283185307179586

	extern double rmin_vertical;
	extern double rmin_level;

	extern double prk_width_vertical;
	extern double prk_back_rear;
	extern double prk_length_vertical;
	extern double prk_back_head;
	extern double prk_width_level;
	extern double prk_back_level;
	extern double prk_length_level;

	extern double dist_to_block;
	#ifndef DEBUG_PC
	extern boost::shared_ptr<bclcpp::Publisher> publisher_path;
	extern boost::shared_ptr<bclcpp::Publisher> publisher_status;
	extern boost::shared_ptr<bclcpp::Publisher> publisher_complete;
	extern int data_subscriber(std::string msg);
	extern void path_publisher(std::string msg);
	extern void status_publisher(std::string msg);
	extern void complete_publisher(std::string msg);
	#endif

	extern FILE* pF;
	extern App app;
	extern Fusion fusion;
	extern Calculation calculation;
	extern Control control;
	extern Plan plan;
	extern ParkingSpaceInfo parkingSpaceInfo;
	extern Path_config pathfind_parameters;
	extern Vehicle_config vehicle_parameters;
	extern uint8 Top_ProceState;

	#define plan_request control.PlanningRequest
	#define obstmap fusion.freeSpaceCell

	extern std::vector<path_point> pathpoint;
	
	extern std::vector<std::pair<double,double>> park_vertical_rear;
	extern std::vector<std::pair<double,double>> park_vertical_head;
	extern std::vector<std::pair<double,double>> park_level;

	extern std::vector<double> Car_x;
	extern std::vector<double> Car_y;

	extern std::vector<double> upa_x;
	extern std::vector<double> upa_y;

	extern std::vector<double> map_image_x;
	extern std::vector<double> map_image_y;

	extern std::vector<double> map_image_vertical_x;
	extern std::vector<double> map_image_vertical_y;

	extern std::vector<double> level_back2_imagmap_x;
	extern std::vector<double> level_back2_imagmap_y;

	extern std::vector<double> level_back_imagmap_x;
	extern std::vector<double> level_back_imagmap_y;

	extern std::vector<double> level_down2_imagmap_x;
	extern std::vector<double> level_down2_imagmap_y;

	extern std::vector<double> level_down_imagmap_x;
	extern std::vector<double> level_down_imagmap_y;

	extern std::vector<double> level_front_imagmap_x;
	extern std::vector<double> level_front_imagmap_y;

	extern std::vector<double> level_front2_imagmap_x;
	extern std::vector<double> level_front2_imagmap_y;

	extern std::vector<double> level_front_imagmap_x_l;
	extern std::vector<double> level_front_imagmap_y_l;

	extern std::vector<double> level_front_imagmap_x_r;
	extern std::vector<double> level_front_imagmap_y_r;

	extern std::vector<double> level_up2_imagmap_x;
	extern std::vector<double> level_up2_imagmap_y;

	extern std::vector<double> level_up_imagmap_x;
	extern std::vector<double> level_up_imagmap_y;

	extern std::vector<double> Rect_x0;
	extern std::vector<double> Rect_y0;

	extern std::vector<double> Rect_x10;
	extern std::vector<double> Rect_y10;

	extern std::vector<double> Rect_x;
	extern std::vector<double> Rect_y;

	extern std::vector<double> aim_dist;


	extern double Start[3];
	extern double End[3];
	extern double start_to_fus[3];
	extern double end_to_fus[3];
	extern double end_to_fus_before[3];
	extern double Start_last[3];
	extern double fus_end_last[3];

	extern double park_theta;
	extern double P0x;
	extern double P0y;
	extern double P1x;
	extern double P1y;
	extern double P2x;
	extern double P2y;
	extern double P3x;
	extern double P3y;
	extern double b_depth;
	extern int ccc;
	extern double ctrl_nav_x;
	extern double ctrl_nav_y;

	extern int Gears;
	extern int index_request;
	extern double nseconds_plan;
	extern double nesconds_geo_plan;
	extern int Read_obstmap;
	extern int APAStatus_before;
	extern int ErrorCode;

	extern int PathOnlyOneNow;
	extern int PlanOnlyOne;
	extern int StartRecordFlag;
	
	extern double d_s2s;
	extern double d_s2e;
	extern double d_phi;
	extern int dynamic_break;
	extern bool prk_inclined_dynamic_plan;
	extern bool dynamic_plan_flag;
	extern bool straight_front_flag;
	extern bool dynamic_final_end_flag;
	extern int dynamic_times;
	extern bool FLAG_log;

	// extern double Rect_x[478];
	// extern double Rect_y[478];
	// const int Rect_index = 478;
	// extern double Rect_x10[68];
	// extern double Rect_y10[68];
	// const int Rect_index10 = 68;
	// extern double Rect_x0[432];
	// extern double Rect_y0[432];
	// const int Rect_index0 = 432;
	// extern double Car_x[3664];
	// extern double Car_y[3664];
	// const int Car_idx = 3664;
	// extern double upa_x[10];
	// extern double upa_y[10];


	// geometry
	extern int ParkingSpaceFlag;
	extern int PlanBackFirst;
	extern int PlanForwardFirst;
	extern int PlanForwardFail;
	extern int PlanMulti;
	extern double dist_P01;
	extern double dist_P23;
	extern double dist_P03;
	extern double dist_toward_P03;
	extern double prk_bias;
	
	extern double P_r[2];
	extern double D_x;
	extern double Aim_dy;
	extern double W_exp;
	extern double L_exp;
	extern double Aim_dist;
	//head
	extern double depth_of_park;
	extern double safe_dynamic_rmin;
	//parallel
	extern double Start_out[3];
	extern int right_left;
	extern int index_back;
	extern int index_front;
	extern double backpath;
	extern double frontpath;
	extern int index_30;
	extern int choice_cc; 
	extern int index_level_out;

	extern int index_up_down;
	extern int index_front_back;
	extern int index_front_rl;

	extern int index_front2_back;
	extern int size_front;
	extern int size_back;

	// A*
	extern double nseconds;
	extern double g_px, g_py, g_pth;
	extern int no_imag_map;
	extern double End_level_small[3];
	extern double Theta_se;
	extern double Y_se;
	extern int SEorES; 
	extern int CloseSizeMax;
	extern Node g_tnode;              
	extern int g_xidx, g_yidx, g_thidx;
	extern std::unordered_map<std::string, Node> g_openset, g_closeset;
	extern std::priority_queue<std::pair<std::string, double>, std::vector<std::pair<std::string, double>>, cmp> pq;
	extern std::string str_idx;
	extern std::vector<double> find_steer_degree;
	extern double front_path;
	extern double back_path;
	extern int back_D;
	extern int front_D;
	extern int small_level_park;
	extern int AstarOrGeo;
	extern int Collision_flag;
	///    ///
	extern double end_init[3];
	extern double bias_th;
	extern int Path_2;
	extern double rk_sto[3];
	extern std::vector<path_point> pathpoint3;
	///    ///
	extern std::vector<std::vector<int>> dis_map;
	extern int use_circle_flag;
}
#endif