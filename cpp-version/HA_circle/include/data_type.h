#pragma once

#include <math.h>
#include "def_types.h"

typedef long int64_byd; // 20220407修改解决编译不通过的问题
struct Vehicle_config // 车辆参数设置
{
	double WB = 2.92;						 //[m] wheel base : rear to front steer 轴距
	double W = 1.94;						 //[m] width of vehicle 车宽
	double LF = 3.892;						 //[m] distance from rear to vehicle front end of vehicle 后轴中心到车辆最前端的距离
	double LB = 1.093;						 //[m] distance from rear to vehicle back end of vehicle 后轴中心到车辆最后端的距离
	double MAX_STEER = 0.4881;				 //[rad] maximum steering angle 车辆轮胎最大转角
	double MIN_CIRCLE = WB / tan(MAX_STEER); //[m] mininum steering circle radius 车辆最小转弯半径 汉_5.5

	// double W_max = 1.95;
	// double LF_max = 3.9;
	// double LB_max = 1.1;
};
extern Vehicle_config vehicle_parameters;	  // 车辆参数设置

struct path_point // 搜出的路径点进行记录
{
	double x = 0;
	double y = 0;
	double th = 0;
	double D = 0;
	double delta = 0;
};

struct Path_config // 惩罚和搜索的参数设置
{

	// Motion resolution define 运动分辨率定义
	float MOTION_RESOLUTION = 0.1; //[m] path interporate resolution 路径插值分辨率
	float N_STEER = 1.0;		   // 20.0; % number of steer command 转向指令数
	float EXTEND_AREA = 0;		   //[m] map extend length 地图延伸长度
	float XY_GRID_RESOLUTION = 0.1;
	float YAW_GRID_RESOLUTION = 3;

	// Grid bound 网格边界
	double MINX = -12.5;
	double MAXX = 12.5;
	double MINY = -12.5;
	double MAXY = 12.5;
	double MINYAW = -3.141592653589793;
	double MAXYAW = 3.141592653589793;

	int XIDX;
	int XIDY;

	// Cost related define 成本相关定义
	float SB_COST = 0.9;		   // switch back penalty cost 切换回惩罚成本
	float BACK_COST = 1;		   // 1; %1.5; % backward penalty cost 反向惩罚成本
	float STEER_CHANGE_COST = 0.9; // 0; %1.5; % steer angle change penalty cost 转向角改变惩罚成本
	float STEER_COST = 0;		   // steer angle change penalty cost 转向角惩罚成本
	float H_COST = 1;			   // Heuristic cost 启发式成本
};
extern Path_config pathfind_parameters;		  // 惩罚参数设置
////////////////////////融合结构体/////////////////////////////////////
struct Position
{
	double X;
	double Y;
	double Heading;
};

struct ParkingSpaceInfo
{
	uint8 id;
	int P0_X;
	int P0_Y;
	int P1_X;
	int P1_Y;
	int P2_X;
	int P2_Y;
	int P3_X;
	int P3_Y;
	int Width;
	uint8 ParkingSpaceValid;
	uint8 ParkingSpaceType;
};

struct FreeSpaceCell
{
	uint8 Status;
	// uint16 Probability;
};

struct Fusion
{
	int64_byd TimeStampMs;
	Position position;
	ParkingSpaceInfo parkingSpaceInfo;
	uint8 TraceParkingID_Cam;
	uint8 TraceParkingID_USS;
	double Theta;
	uint8 ParkInMode;
	double distance_01; // 20221013新增，障碍物边缘到P0P1边的距离
	double distance_23; // 20221013新增，障碍物边缘到P2P3边的距离
	double distance_03;
	double depth_block;
	FreeSpaceCell freeSpaceCell[62500];
};

extern Fusion fusion;
#define obstmap fusion.freeSpaceCell
extern std::vector<std::vector<int>> dis_map(250, std::vector<int>(250, 0));
