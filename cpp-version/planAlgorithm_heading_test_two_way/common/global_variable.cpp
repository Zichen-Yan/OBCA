#include "global_variable.h"

namespace byd_apa_plan
{
	////////////////日志开关/////////////////////
	bool FLAG_log = true;
	std::shared_ptr<spdlog::logger> plan_logger = nullptr;
	
	App app;
	Fusion fusion;
	Calculation calculation;
	Control control;
	Plan plan;
	ParkingSpaceInfo parkingSpaceInfo;
	CurrentParkInfo currentParkInfo;
	PathFindParam pathfind_parameters;
	VehicleParam vehicle_parameters;
	uint8 Top_ProceState = 0;

	std::vector<PathPoint> pathpoint;
	
	std::vector<std::pair<double,double>> park_vertical_rear;
	std::vector<std::pair<double,double>> park_vertical_head;
	std::vector<std::pair<double,double>> park_level;

	std::vector<std::pair<double,double>> park_out_rear_left;
	std::vector<std::pair<double,double>> park_out_rear_right;

	std::vector<std::pair<double,double>> park_out_head_left;
	std::vector<std::pair<double,double>> park_out_head_right;

	std::vector<double> Car_x;
	std::vector<double> Car_y;

	std::vector<double> upa_x;
	std::vector<double> upa_y;

	std::vector<double> map_image_x;
	std::vector<double> map_image_y;

	std::vector<double> map_image_vertical_x;
	std::vector<double> map_image_vertical_y;

	std::vector<double> level_back2_imagmap_x;
	std::vector<double> level_back2_imagmap_y;

	std::vector<double> level_back_imagmap_x;
	std::vector<double> level_back_imagmap_y;

	std::vector<double> level_down2_imagmap_x;
	std::vector<double> level_down2_imagmap_y;

	std::vector<double> level_down_imagmap_x;
	std::vector<double> level_down_imagmap_y;

	std::vector<double> level_front_imagmap_x;
	std::vector<double> level_front_imagmap_y;

	std::vector<double> level_front2_imagmap_x;
	std::vector<double> level_front2_imagmap_y;

	std::vector<double> level_front_imagmap_x_l;
	std::vector<double> level_front_imagmap_y_l;

	std::vector<double> level_front_imagmap_x_r;
	std::vector<double> level_front_imagmap_y_r;

	std::vector<double> level_up2_imagmap_x;
	std::vector<double> level_up2_imagmap_y;

	std::vector<double> level_up_imagmap_x;
	std::vector<double> level_up_imagmap_y;

	std::vector<double> Rect_x0;
	std::vector<double> Rect_y0;

	std::vector<double> Rect_x20;
	std::vector<double> Rect_y20;

	std::vector<double> Rect_x20_sparse;
	std::vector<double> Rect_y20_sparse;

	
	
	double rmin_vertical = 5.6;
	double rmin_level = 5.5;

	double prk_width_vertical = 2.5;
	double prk_back_rear = -1.2;
	double prk_length_vertical = 6.9;
	double prk_back_head = 4.1;
	double prk_width_level = 2.8;
	double prk_back_level = -1.4;
	double prk_length_level = 5.6;

	double dist_to_block = 0.25;
	//
	double start_to_prk[3] = {0};
	double end_to_prk[3] = { 0 };
	double start_to_fus[3] = { 0 };
	double end_to_fus[3] = { 0 };
	//double end_to_fus_before[3] = { 0 };
	double fus_start_last[3] = { 0 };
	double fus_end_last[3] = { 0 };
	double park_theta = 0;
	double P0x = 0;
	double P0y = 0;
	double P1x = 0;
	double P1y = 0;
	double P2x = 0;
	double P2y = 0;
	double P3x = 0;
	double P3y = 0;
	double b_depth = 0;
	int block_collison_times = 0;
	double ctrl_nav_x = 0;
	double ctrl_nav_y = 0;

	int Gears = 0;
	int index_request = 0;
	double nseconds_plan = 0;
	double nesconds_geo_plan = 0;
	int Read_obstmap = 0;
	int APAStatus_before = 0;
	int ErrorCode = 0;

	int PathOnlyOneNow = 0;
	int PlanOnlyOne = 0;
	int StartRecordFlag = 0;
	
	double d_s2s = 0;
	double d_s2e = 0;
	double d_phi = 0;
	int FLAG_stop_advanced = 0;
	bool FLAG_dynamic_plan_advanced = false;
	bool flag_dynamic_plan_advanced = false;
	bool FLAG_straight_forward = false;
	bool FLAG_published_end = false;
	bool FLAG_forward_once_more = true;
	// geometry
	int ParkingSpaceFlag = 0;
	int PlanBackwardFirst = 0;
	int PlanForwardFirst = 0;
	int PlanForwardFail = 0;
	int PlanMulti = 0;
	double dist_P01 = -1;
	double dist_P23 = -1;
	double dist_P03 = -1;
	double dist_toward_P03 = -1;
	//double prk_bias = 0;
	double P_r[2] = {0};
	double D_x = 4.8; 
	
	double Aim_dy = 0.5;
	double W_exp = 0.3;
	double L_exp = 0.3;
	std::vector<double> aim_dist;
	double Aim_dist = 0;
	double depth_no_obstacle = 1.0;
	double length_of_straight = 0.8;
	double P_r_x_first = .0;
	double P_r_y_first = .0;
	std::pair<double,double> first_path_end(100.0,100.0);
	double delta_r = 0.3;
	//head
	double depth_of_park = 0.5;
	double safe_dynamic_rmin = 22.0;
	double max_y_error_completed = 0.15;

	// parking out
	int flag_out_park = 0;
	double default_dist_head_out = 2.2;
	double default_dist_rear_out = -4.0;
	double default_dist_out_min = 2.2;
	double dist_head_out = .0;
	double dist_rear_out = .0;
	double dist_out_min = .0;
	double dist_left_out = .0;
	double dist_right_out = .0;
	//mapper
	int col_max_front = 15;
	int row_max_front = 10;
	int col_max_toward = 50;
	int row_max_toward = 20;
	double toward_space_start = 4.5;
	int obs_count_max = 1;
	double virtual_obs_length = 5.1;
	//parallel
	double Start_out[3] = {0};
	int right_left = 0;
	int index_back = 14;
	int index_front = 14;
	double backpath = 0;
	double frontpath = 0;
	int index_30 = 0;
	int choice_cc = 0;
	int index_level_out = 0;
	
	int index_up_down = 177;
	int index_front_back = 41;
	int index_front_rl = 43;

	int index_front2_back = 29;
	int size_front = 16;
	int size_back = 11;
	// A*
	double nseconds = 0;
	double g_px = 0, g_py = 0, g_pth = 0;
	int aaa = 0;
	//int bbb = 0;
	int no_imag_map = 0;
	double End_level_small[3] = {0};
	double Theta_se = 0;
	double Y_se = 0;
	int SEorES = 0; 
	int CloseSizeMax = 0;
	Node g_tnode;             
	int g_xidx = 0, g_yidx = 0, g_thidx = 0;
	std::unordered_map<std::string, Node> g_openset, g_closeset;
	std::priority_queue<std::pair<std::string, double>, std::vector<std::pair<std::string, double>>, cmp> pq;
	std::string str_idx;
	// two-way
	std::priority_queue<std::pair<std::string, double>, std::vector<std::pair<std::string, double>>, cmp> pq_two;
	std::unordered_map<std::string, Node> g_openset_two, g_closeset_two;
	std::string str_idx_two;
	std::vector<PathPoint> pathpoint_two;
	Node g_tnode_two;  

	std::vector<double> find_steer_degree;
	double back_path = 100;
	double front_path = 100;
	int back_D = 100;
	int front_D = 100;
	int small_level_park = 0;
	int AstarOrGeo = 0;
	//int dynamic_times = 0;
	int FLAG_collison_check_model = 0;
    ///    ///
	int LevelDynamic = 0;
	double rk_sto[3] = {0,0,0}; 
	std::vector<PathPoint> pathpoint3;
	///     ///
	std::vector<std::vector<int>> dis_map(250, std::vector<int>(250, 0));
	int use_circle_flag=1;

	double avg_dis=0;
	double Update_time=0;
	double Collision_time=0;
	double AE_time=0;
	// OBCA
	std::vector<Eigen::MatrixXd> hPolys_;
	int finish_flag=0;
}