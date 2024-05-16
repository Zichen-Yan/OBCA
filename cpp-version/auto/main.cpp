#include "./common/global_function.h"
#include "./common/global_variable.h"
#include "matplotlibcpp.h"

using namespace byd_apa_plan;

#define MAP_SIZE (62500)

void loadData(std::string path)
{
	std::ifstream inputFile(path); // 打开文件
    if (!inputFile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
    }

    std::string res;
	std::getline(inputFile, res);
	for(int i = 0; i < MAP_SIZE * 2; i += 2){
		fusion.freeSpaceCell[i / 2].Status = res[i] - '0';
	}

	std::getline(inputFile, res);
	std::stringstream ss1(res); 
    std::vector<double> fus_pos; 
    std::string token;
    while (std::getline(ss1, token, ',')) { 
        fus_pos.push_back(std::stod(token));
    }

	std::getline(inputFile, res);
	std::stringstream ss2(res); 
    std::vector<double> navi_pos;
    while (std::getline(ss2, token, ',')) { 
        navi_pos.push_back(std::stod(token));
    }

	std::getline(inputFile, res);
	std::stringstream ss3(res); 
    std::vector<double> prk_pos;
    while (std::getline(ss3, token, ',')) { // 按逗号分割字符串
        prk_pos.push_back(std::stod(token));
    }

	std::getline(inputFile, res);
	fusion.Theta = std::stod(res);

	std::getline(inputFile, res);
	fusion.parkingSpaceInfo.ParkingSpaceType = std::stod(res);

    inputFile.close(); // 关闭文件

	APAStatus_before = 7;
	app.APAStatus = 2;
	app.APA_Park_Function = 1;
	control.PlanningRequest = 1;
	control.ObsUssInfo = 255u;

	fusion.TraceParkingID_Cam = 1;
	fusion.TraceParkingID_USS = 0;

	fusion.depth_block= -1;
	fusion.ParkInMode = 0;

    Gears = 0;
	PlanBackwardFirst = 0;
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
}



int main(int argc,char **argv)
{
	std::string map_path = argv[1];
	save_path = argv[2];
	size_t lastSlashPos = map_path.find_last_of("/");
	std::string map_name = map_path.substr(lastSlashPos + 1);
	save_path = save_path+"plan_" + map_name;
	std::cout << save_path << std::endl;

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
	loadData(map_path);
	#endif
	bydapa::common::TicToc tictoc_config;
	tictoc_config.tic();
	bool status = ReadPlanConfig("/home/byd2004/Pictures/auto/UREplanConfig.json");
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
		// PlotResult();
		break;
	}
	return 0;
}
