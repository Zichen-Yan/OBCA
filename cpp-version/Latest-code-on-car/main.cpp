#include "./common/global_function.h"
#include "./common/global_variable.h"
#include "matplotlibcpp.h"

using namespace byd_apa_plan;

#define MAP_FILE "../map.txt"
#define MAP_SIZE (62500)

void readMap(){
	char buf[MAP_SIZE * 2 + 1] = {0};
	int fp = open(MAP_FILE, O_RDONLY);

	int ret = read(fp, &buf, MAP_SIZE * 2);
	if(0 == ret){
		//LOG_DEBUG("read buf size = 0!");
        std::cout << "read buf size = 0!" << std::endl;
	}

	for(int i = 0; i < MAP_SIZE * 2; i += 2){
		fusion.freeSpaceCell[i / 2].Status = buf[i] - '0';
	}

	close(fp);
	return;
}

void loadData(){

	APAStatus_before = 7;
	app.APAStatus = 2;
	app.APA_Park_Function = 1;
	control.PlanningRequest = 1;
	control.ObsUssInfo = 255u;

	fusion.TraceParkingID_Cam = 1;
	fusion.TraceParkingID_USS = 0;

	double fus_pos[3] = {1001.858414,67.462813,276.320007};
	double navi_pos[3] = {698.780029,57.119999,276.320007};
	int prk_pos[8] = {1071,-29, 1549,5, 1539,184, 1060,151};
	fusion.Theta = 184.028601;

	fusion.depth_block= -1.0;
	fusion.parkingSpaceInfo.ParkingSpaceType = 1;
	fusion.ParkInMode = 0;

    Gears = 0;
	PlanBackFirst = 0;
	PlanForwardFirst = 0;
	PlanMulti = 0;
	AstarOrGeo = 1;

	fusion.position.X = fus_pos[0];
	fusion.position.Y = fus_pos[1];
	fusion.position.Heading = fus_pos[2];

    calculation.nav_pos_X = navi_pos[0];      
	calculation.nav_pos_Y = navi_pos[1];        
	calculation.nav_heading = navi_pos[2];

	fusion.parkingSpaceInfo.P0_X = prk_pos[0];  
	fusion.parkingSpaceInfo.P0_Y = prk_pos[1];  
	fusion.parkingSpaceInfo.P1_X = prk_pos[2];  
	fusion.parkingSpaceInfo.P1_Y = prk_pos[3];  
	fusion.parkingSpaceInfo.P2_X = prk_pos[4];  
	fusion.parkingSpaceInfo.P2_Y = prk_pos[5];  
	fusion.parkingSpaceInfo.P3_X = prk_pos[6];  
	fusion.parkingSpaceInfo.P3_Y = prk_pos[7];  


	readMap();
}



int main()
{
	//bydapa::common::Log::GetInstance()->SetFilename("GeoAlgrithm_MR_ys", false);
	/////////////////////////////////////////////////////////////////////////////////////
	#ifndef DEBUG_PC
	bool ret = bclcpp::init();
	if (ret)
	{
		//LOG_WARNING("bclcpp::init ok");
	}
	else
	{
		//LOG_WARNING("bclcpp::init error");
	}
	
	publisher_path = bclcpp::create_publisher("geometry/pathinfo/ctrl", &byd_interfaces_msg_geometry_pathinfo_ctrl_desc, 1);
	publisher_status = bclcpp::create_publisher("geometry/statusinfo/ctrl", &byd_interfaces_msg_geometry_statusinfo_ctrl_desc, 1);
	publisher_complete = bclcpp::create_publisher("geometry/completeinfo/ctrl", &byd_interfaces_msg_geometry_completeinfo_ctrl_desc, 1);
	
	std::thread worker(data_subscriber, ("data_subscriber"));
	#else
	loadData();
	#endif
	bydapa::common::TicToc tictoc_config;
	tictoc_config.tic();
	bool status = ReadPlanConfig("../planConfig.json");
	double config_time = tictoc_config.toc();
	config_time = config_time * 1.0e-9;
	
	if(status)
	{
		printf("************Read Config Success************\n");
		printf("config_time = %lf\n",config_time);
	}
	park_delete_map();
	while (status)
	{
		Process();
		usleep(10000);
		PlotResult();
		break;
	}
	return 0;
}
