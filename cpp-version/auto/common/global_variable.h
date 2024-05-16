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
#include <climits>

#include "./include/log.h"
#include "./include/def_types.h"
#include "./include/util_time.h"
#include "./include/json.hpp"
#include "base_data_struct.h"
#include "./include/spdlog/spdlog.h"
#include "./include/spdlog/sinks/basic_file_sink.h"
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

	extern std::shared_ptr<spdlog::logger> plan_logger;
	extern App app;
	extern Fusion fusion;
	extern Calculation calculation;
	extern Control control;
	extern Plan plan;
	extern ParkingSpaceInfo parkingSpaceInfo;
	extern CurrentParkInfo currentParkInfo;
	extern PathFindParam pathfind_parameters;
	extern VehicleParam vehicle_parameters;
	extern uint8 Top_ProceState;

	#define plan_request control.PlanningRequest
	#define obstmap fusion.freeSpaceCell

	extern std::vector<PathPoint> pathpoint;
	
	extern std::vector<std::pair<double,double>> park_vertical_rear;
	extern std::vector<std::pair<double,double>> park_vertical_head;
	extern std::vector<std::pair<double,double>> park_level;

	extern std::vector<std::pair<double,double>> park_out_rear_left;
	extern std::vector<std::pair<double,double>> park_out_rear_right;

	extern std::vector<std::pair<double,double>> park_out_head_left;
	extern std::vector<std::pair<double,double>> park_out_head_right;

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

	extern std::vector<double> Rect_x20;
	extern std::vector<double> Rect_y20;

	extern std::vector<double> Rect_x20_sparse;
	extern std::vector<double> Rect_y20_sparse;

	


	extern double start_to_prk[3];
	extern double end_to_prk[3];
	extern double start_to_fus[3];
	extern double end_to_fus[3];
	//extern double end_to_fus_before[3];
	extern double fus_start_last[3];
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
	extern int block_collison_times;
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
	extern int FLAG_stop_advanced;
	extern bool FLAG_dynamic_plan_advanced;
	extern bool flag_dynamic_plan_advanced;
	extern bool FLAG_straight_forward;
	extern bool FLAG_published_end;
	//extern int dynamic_times;
	extern bool FLAG_log;
	extern bool FLAG_forward_once_more;


	// geometry
	extern int ParkingSpaceFlag;
	extern int PlanBackwardFirst;
	extern int PlanForwardFirst;
	extern int PlanForwardFail;
	extern int PlanMulti;
	extern double dist_P01;
	extern double dist_P23;
	extern double dist_P03;
	extern double dist_toward_P03;
	//extern double prk_bias;
	
	extern double P_r[2];
	extern double D_x;
	extern double Aim_dy;
	extern double W_exp;
	extern double L_exp;
	extern std::vector<double> aim_dist;
	extern double Aim_dist;
	extern double depth_no_obstacle;
	extern double length_of_straight;
	extern double P_r_x_first;
	extern double P_r_y_first;
	extern std::pair<double,double> first_path_end;
	extern double delta_r;
	//head
	extern double depth_of_park;
	extern double safe_dynamic_rmin;
	extern double max_y_error_completed;

	//parking out
	extern int flag_out_park;
	extern double default_dist_head_out;
	extern double default_dist_rear_out;
	extern double default_dist_out_min;
	extern double dist_head_out;
	extern double dist_rear_out;
	extern double dist_out_min;
	extern double dist_left_out;
	extern double dist_right_out;
	//mapper
	extern int col_max_front;
	extern int row_max_front;
	extern int col_max_toward;
	extern int row_max_toward;
	extern double toward_space_start;
	extern int obs_count_max;
	extern double virtual_obs_length;

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
	extern std::vector<Node> expanded_points;              
	extern int g_xidx, g_yidx, g_thidx;
	extern std::unordered_map<std::string, Node> g_openset, g_closeset;
	extern std::priority_queue<std::pair<std::string, double>, std::vector<std::pair<std::string, double>>, cmp> pq;
	extern std::string str_idx;
	// two-way
	extern std::priority_queue<std::pair<std::string, double>, std::vector<std::pair<std::string, double>>, cmp> pq_two;
	extern std::unordered_map<std::string, Node> g_openset_two, g_closeset_two;
	extern std::string str_idx_two;
	extern std::vector<PathPoint> pathpoint_two;
	extern Node g_tnode_two;

	extern std::vector<double> find_steer_degree;
	extern double front_path;
	extern double back_path;
	extern int back_D;
	extern int front_D;
	extern int small_level_park;
	extern int AstarOrGeo;
	extern int FLAG_collison_check_model;
	///    ///
	extern int LevelDynamic;
	extern double rk_sto[3];
	extern std::vector<PathPoint> pathpoint3;
	///    ///
	extern std::vector<std::vector<int>> dis_map;
	extern int use_circle_flag;
	extern std::string save_path;
	extern double min_dis;
	extern double avg_dis;
}
#endif