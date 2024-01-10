#ifndef findpath_H
#define findpath_H
#include "log.h"
#include <vector>
#include <iostream>
#include <cstring>
#include <unistd.h>
// #include <stdio.h>
// #include <time.h>
// #include <sys/time.h>
#include <string.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <stdint.h>
// #include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <math.h>
#include "util_time.h"
#include <fcntl.h>
#include <queue>
#include <unordered_map>
#include <opencv.hpp>
//#include "log.h"
#include <string>

#define twopi 6.283185307179586
#define pi 3.141592653589793
#define R_V 6
#define R_L 6
#define R_ 5.8
#define R_C 5.5

typedef  unsigned char uint8;
typedef  unsigned short uint16;
typedef  long int64_byd;  //20220407修改解决编译不通过的问题
typedef  unsigned int uint32;

class Node                 //定义每个node节点的类
{
public:
	int xidx = 0;
	int yidx = 0;
	int yawidx = 0;
	double D = 0;
	double delta = 0;
	double x = 0;    //节点真实坐标
	double y = 0;
	double theta = 0;//节点车辆的真实角度
	int parent[3];
	double cost = 1000;
	double fcost = 1000;
public:
	Node();
	Node(int xidx, int yidx, int yawidx, double D, double delta, double x, double y, double theta, int parent[3], double  cost, double fcost);
	friend bool operator <(Node n1,Node n2)
    {
        return n1.fcost > n2.fcost;
    }

}; //必须有分号

struct Path_config  //惩罚和搜索的参数设置
{
	// Motion resolution define 运动分辨率定义
	float MOTION_RESOLUTION = 0.1; //[m] path interporate resolution 路径插值分辨率
	float N_STEER = 1.0; //20.0; % number of steer command 转向指令数
	float EXTEND_AREA = 0;//[m] map extend length 地图延伸长度
	float XY_GRID_RESOLUTION = 0.1;
	float YAW_GRID_RESOLUTION = 3;

	//Grid bound 网格边界
	double MINX = -12.5;
	double MAXX = 12.5;
	double MINY = -12.5;
	double MAXY = 12.5;
	double MINYAW = -3.141592653589793;
	double MAXYAW = 3.141592653589793;

	int XIDX;
	int XIDY;

	//Cost related define 成本相关定义
	float SB_COST = 0.9; // switch back penalty cost 切换回惩罚成本
	float BACK_COST = 1; // 1; %1.5; % backward penalty cost 反向惩罚成本
	float STEER_CHANGE_COST = 0.9; // 0; %1.5; % steer angle change penalty cost 转向角改变惩罚成本
	float STEER_COST = 0; // steer angle change penalty cost 转向角惩罚成本
	float H_COST = 1; // Heuristic cost 启发式成本

};

/*
struct map_set   //地图设置
{
	float X_space_l = -5;
	float X_space_r = 25;
	float Y_space_up = 10;
	float Y_space_down = 0;
	float X_park_l = -1.4;
	float X_park_r = 1.4;
	float Park_wigh = 5;

};
*/
struct Vehicle_config   //车辆参数设置
{
	double WB = 2.92;  //[m] wheel base : rear to front steer 轴距
	double W = 1.91; //[m] width of vehicle 车宽
	double LF = 3.892; //[m] distance from rear to vehicle front end of vehicle 后轴中心到车辆最前端的距离
	double LB = 1.093; //[m] distance from rear to vehicle back end of vehicle 后轴中心到车辆最后端的距离
	double MAX_STEER = 0.4881; //[rad] maximum steering angle 车辆轮胎最大转角
	double MIN_CIRCLE = WB / tan(MAX_STEER); //[m] mininum steering circle radius 车辆最小转弯半径 汉_5.5
	
	//double W_max = 1.95;
	//double LF_max = 3.9;
	//double LB_max = 1.1;
};


struct path_point   //搜出的路径点进行记录
{
	double x = 0;
	double y = 0;
	double th = 0;
	double D = 0;
	double delta = 0;

};


// enum vturn { St, Ri, Le};
struct RsPath               //定义path的类型
{
	// vturn rspath_type[3];
	// double t;
	// double u;
	// double v;
	double lenth;
	std::string pathtype;
	double pathlength[5]={0,0,0,0,0};
};
/////////////////////////路径规划结构体///////////////////////////////
struct Coordinate 
{
	double X;
	double Y;
	double Yaw;
	uint8 rajectoryDirection;
	double Curvature;

};

struct Plan 
{
	int64_byd TimeStampMs;
	int TrajectoryLength;
	Coordinate coordinate[300];
	uint8 PlanningStatus;
	uint8 IsPlanningCompleted;
};
////////////////////////app结构体//////////////////////////////////////
struct App
{
	uint8 APAStatus;
	uint8 APA_nav_cmd;
	uint8 APA_Park_Function;
};
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
	//uint16 Probability;
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
	double distance_01;  //20221013新增，障碍物边缘到P0P1边的距离
	double distance_23;  //20221013新增，障碍物边缘到P2P3边的距离
	double distance_03;
	double depth_block;
	FreeSpaceCell freeSpaceCell[62500];
};
////////////////////////车辆控制结构体///////////////////////////
struct Control 
{
	uint8 PlanningRequest;
	uint8 PlanningRequestCount;
	uint32 ObsUssInfo;
};
////////////////////////航迹结构体///////////////////////////////
struct Calculation {
	int64_byd TimeStampMs;
	double nav_pos_X;
	double nav_pos_Y;
	double nav_heading;
	uint8 nav_status;

};
///////////声明全局变量//////////////////////////////
extern Plan plan;
extern App app;
extern Fusion fusion;
extern Control control;
extern Calculation calculation;

#define plan_request control.PlanningRequest
#define obstmap fusion.freeSpaceCell
extern int aaa;
extern int bbb;
extern int ccc;
extern int Gears;
extern int index_request;
extern int no_imag_map;
extern int PathOnlyOneNow;
extern double Start[3];
extern double End[3];
extern double End_before[3];
extern double End_level_small[3];
extern double Theta_se;
extern double Y_se;
extern int SEorES; // 起点到终点为0，终点到起点为1
extern int ErrorCode;
extern int CloseSizeMax;
extern double g_px, g_py, g_pth;  //点坐标
extern Node g_tnode;                //点
extern int g_xidx, g_yidx, g_thidx;

extern std::vector<Node> g_open, g_close;//open与close集合
extern std::unordered_map<std::string, Node> g_openset, g_closeset;
struct cmp {
    bool operator()(const std::pair<std::string, double>& left,
                    const std::pair<std::string, double>& right) const {
      return left.second >= right.second;
    }
  };
extern std::priority_queue<std::pair<std::string, double>, std::vector<std::pair<std::string, double>>, cmp> pq;
extern std::string str_idx;
extern std::vector<Node> expanded_points;//close集合已拓展的点集合

extern std::vector<path_point>pathpoint;//搜出的路点的集合
extern Path_config pathfind_parameters;//惩罚参数设置
//extern map_set map_parameters;        //地图数据设置
extern Vehicle_config vehicle_parameters;//车辆参数设置
extern std::vector<double> find_steer_degree;//方向盘角度设置
extern double nseconds;
extern double Rect_x[];
extern double Rect_y[];
extern double Rect_x10[];
extern double Rect_y10[];
extern double back_path;
extern double front_path;
extern int back_D;
extern int front_D;
extern int small_level_park;
extern double b_depth;
extern double ctrl_nav_x;
extern double ctrl_nav_y;
////////////////////函数声明////////////////////////////////
extern bool  HybridAStar(double Start[3], double End[3]);
extern bool  HybridAStar_level(double Start[3], double End[3]);
extern int  dynamic_vertical(double Start[3], double End[3]);
extern int  dynamic_Level(double Start[3], double End[3]);
extern int  dynamic_Oblique(double Start[3], double End[3]);
extern int  dynamic_vertical_multi(double Start[3], double End[3]);
extern int  dynamic_CC_road(double Start[3], double End[3]);
extern double mod2pi(double x);
extern bool VehicleCollisionGrid(double cpx, double cpy, double cph);
extern void parameter_initi();
extern void VehicleDynamic(double x, double y, double theta, double D, double delta);
bool LpRpSp(double x, double y, double phi, double& t, double& u, double& v);
bool LpRmL(double x, double y, double phi, double& t, double& u, double& v);

extern int start_end_get();
extern int imagmap_get();
extern void delete_map();
extern void is_Completed();
extern void hypathchange();
extern int hypathchangeSE();
extern int OnlyOne();
extern void StateClear();
extern void ErrorLog();
extern void InformationLog();
extern void PlanLog();
extern void Uss_map();
extern void ParkingVertical();
extern void ParkingLevel(); // 水平车位
extern void ParkingOblique(); // 斜车位

/////////////////////RS函数声明////////////////////////////////
RsPath FindRSPath(double x, double y, double phi);
int FindRSPath(double x, double y, double phi, RsPath &path);
void polar(double x, double y, double& r, double& theta);
std::pair<double, double> calc_tau_omega(const double u,
										 const double v,
										 const double xi,
										 const double eta,
										 const double phi);

bool LSR(double x, double y, double phi, double& t, double& u, double& v);
bool LSL(double x, double y, double phi, double& t, double& u, double& v);
bool LRL(double x, double y, double phi, double &t, double &u, double &v);
bool SLS(double x, double y, double phi, double &t, double &u, double &v);
bool LRLRn(double x, double y, double phi, double &t, double &u, double &v);
bool LRLRp(double x, double y, double phi, double &t, double &u, double &v);
bool LRSR(double x, double y, double phi, double &t, double &u, double &v);
bool LRSL(double x, double y, double phi, double &t, double &u, double &v);
bool LRSLR(double x, double y, double phi, double &t, double &u, double &v);
bool LpRpSp(double x, double y, double phi, double &t, double &u, double &v);

void CCS(double x, double y, double phi, RsPath& path);
void CCC(double x, double y, double phi, RsPath& path);
void CSC(double x, double y, double phi, RsPath& path);
void SCS(double x, double y, double phi, RsPath& path);
void CCCC(double x, double y, double phi, RsPath& path);
void CCSC(double x, double y, double phi, RsPath& path);
void CCSCC(double x, double y, double phi, RsPath& path);

extern int flag_CCS;
extern int flag_CCC;
extern int flag_CSC;
extern int flag_SCS;
extern int flag_CCCC;
extern int flag_CCSC;
extern int flag_CCSCC;
#endif // RS_H




