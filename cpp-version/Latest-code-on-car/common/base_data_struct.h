#ifndef BASE_DATA_STRUCT_H
#define BASE_DATA_STRUCT_H
#include <math.h>
#include <string>
namespace byd_apa_plan
{
	#define DEBUG_PC
	typedef  unsigned char uint8;
	typedef  unsigned short uint16;
	typedef  long int64_byd; 
	typedef  unsigned int uint32;

	class Node // 定义每个node节点的类
	{
	public:
		int xidx = 0;
		int yidx = 0;
		int yawidx = 0;
		double D = 0;
		double delta = 0;
		double x = 0; // 节点真实坐标
		double y = 0;
		double theta = 0; // 节点车辆的真实角度
		int parent[3];
		double cost = 1000;
		double fcost = 1000;
		public:
		Node() = default;
		Node(int xidx_, int yidx_, int yawidx_, double D_, double delta_,
			double x_, double y_, double theta_, int parent_[3],
			double  cost_, double fcost_) : xidx(xidx_), yidx(yidx_), yawidx(yawidx_), D(D_), delta(delta_),
			x(x_), y(y_), theta(theta_), cost(cost_),fcost(fcost_)
			{
				parent[0] = parent_[0];
				parent[1] = parent_[1];
				parent[2] = parent_[2];
			};
		friend bool operator <(Node n1, Node n2)
		{
			return n1.fcost > n2.fcost;
		}
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
		double MINYAW = -M_PI;
		double MAXYAW = M_PI;
		int XIDX;
		int XIDY;
		int MAX_IDX;

		// Cost related define 成本相关定义
		float SB_COST = 0.9;		   // switch back penalty cost 切换回惩罚成本
		float BACK_COST = 1;		   // 1; %1.5; % backward penalty cost 反向惩罚成本
		float STEER_CHANGE_COST = 0.9; // 0; %1.5; % steer angle change penalty cost 转向角改变惩罚成本
		float STEER_COST = 0;		   // steer angle change penalty cost 转向角惩罚成本
		float H_COST = 2;			   // Heuristic cost 启发式成本
	};

	struct Vehicle_config // 车辆参数设置
	{
		double WB = 2.92;						 //[m] wheel base : rear to front steer 轴距
		double W = 1.92;						 //[m] width of vehicle 车宽
		double LF = 3.84;						 //[m] distance from rear to vehicle front end of vehicle 后轴中心到车辆最前端的距离
		double LB = 0.99;						 //[m] distance from rear to vehicle back end of vehicle 后轴中心到车辆最后端的距离
		double MAX_STEER = 0.4881;				 //[rad] maximum steering angle 车辆轮胎最大转角
		double MIN_CIRCLE = WB / tan(MAX_STEER); //[m] mininum steering circle radius 车辆最小转弯半径 汉_5.5
	};

	struct path_point // 搜出的路径点进行记录
	{
		double x = 0;
		double y = 0;
		double th = 0;
		double D = 0;
		double delta = 0;
	};


	enum vturn { St, Ri, Le };
	struct RsPath               //定义path的类型
	{
		vturn rspath_type[3];
		double t;
		double u;
		double v;
		double lenth;
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
	////////////////////////车辆控制结构体///////////////////////////
	struct Control
	{
		uint8 PlanningRequest;
		uint8 PlanningRequestCount;
		uint32 ObsUssInfo;
	};
	////////////////////////航迹结构体///////////////////////////////
	struct Calculation
	{
		int64_byd TimeStampMs;
		double nav_pos_X;
		double nav_pos_Y;
		double nav_heading;
		uint8 nav_status;
	};

	struct cmp {
		bool operator()(const std::pair<std::string, double>& left,
			const std::pair<std::string, double>& right) const {
			return left.second >= right.second;
		}
	};
}
#endif