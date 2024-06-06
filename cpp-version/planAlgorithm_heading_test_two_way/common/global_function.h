#ifndef GLOBAL_FUNCTION_H
#define GLOBAL_FUNCTION_H

#include "base_data_struct.h"
#include <vector>
namespace byd_apa_plan
{
	//common
	extern double mod2pi(double x);
	extern int GetStartAndEndPoint();
	extern int IsPathOnlyOne();
	extern void ParameterInit();
	extern void IsParkingCompleted();
	extern void StateClear();
	extern void PathPointNormalization();
	extern void VehicleDynamic(double x, double y, double theta, double D, double delta);
	extern bool VehicleCollisionGrid(double cpx, double cpy, double cph);
	extern bool GetOnePathPoints(double start[3], double p_, double r_, const int collision_check_flag);
	extern bool GetOrCheckPathPoints(const double* start, const double* road_yaw, const int &i_coll_max, const int &collison_check_flag);
	extern void CoordinateTransToPrk();
	extern void CoordinateTrans(double& x, double& y, double& phi, double x_a, double y_a, double phi_a, double x_s, double y_s, double phi_s);
	extern std::vector<PathPoint> PathPointsCoordinateTrans(const std::vector<PathPoint>& pathpoints_, double end_to_fus_point[3]);
	extern std::vector<std::pair<double, double>> CalcCornerPoints(const double start[3], double d_l = 0, double d_w = 0);
	extern std::vector<std::pair<double, double>> CalcCentreOfCircle(double x_, double y_, double phi_, double r_);
	// 24.1.2
	extern int Dynamic_plan_level(double start1[3], double end1[3]);
	// 24.1.2
	//map
	extern void park_delete_map();
	extern void ModifyMapVertical();
	extern void ModifyMapLevel();
	extern void ModifyMapHybirdAStar();
	extern void ChangeFusionMap();
	extern std::vector<std::vector<int>> generateDistanceMap(std::vector<std::vector<int>>& grid);
	//log
	extern void InitLog(const bool &flag_log);
	extern void PrintToLog(const bool &flag_log, std::string str);
	extern void PlanInputInfoLog(const bool &flag_log);
	extern void PlanOutputInfoLog(const bool &flag_log);
	//config
	extern bool ReadPlanConfig(const std::string &filename);
	extern bool ReadConfig(const std::string &vehicle_filename, const std::string &plan_filename);
	
	//reeds-shepp
	extern bool SmLpSm(const double x, const double y, const double phi, RsPath& path_, int theta_car);
	extern bool CmCpSm(double x, double y, double phi, RsPath& path_five, int theta_car);
	extern bool CpCmSm(double x, double y, double phi, RsPath& path_six, int theta_car);
	extern bool CpCpSm(double x, double y, double phi, RsPath& path_six, int theta_car);
	extern int CpSpCp(double x, double y, double phi, RsPath& path);
	extern int CpCpS(double x, double y, double phi, RsPath& path);
	extern void CSC(double x, double y, double phi, RsPath &path);
	extern void CCC(double x, double y, double phi, RsPath &path);
	extern int CCS(double x, double y, double phi, RsPath &path);
	extern int CC(double x, double y, double theta, double car_r_limit, double road_yaw[6]);
	extern char GetAntiTrigo(double sin_result, double cos_result, double &last_angle_result);

	extern char Calc_Cm(double x_now, double y_now, double thea_now,
		double x_aim, double y_aim, double thea_aim,
		double y_tolerance, double min_r, double yaw_tolerance, double road_yaw[6]);

	extern char Calc_CmCm(double x_now, double y_now, double thea_now,
		double x_aim, double y_aim, double thea_aim,
		double car_r_limit, double road_yaw[6]);

	extern char Calc_CpSm(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim,
		double car_r_limit, double bias, double* road_yaw);

	extern char Calc_CmSm(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim,
		double car_r_limit, double bias, double* road_yaw);

	extern char Calc_CmCmSm(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim,
		double car_r_limit, double road_yaw[6]);

	extern char Calc_CpCpSm(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim,
		double car_r_limit, double* road_yaw);

	extern char Calc_CmCpSm(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim,
		double car_r_limit, double road_yaw[6]);

	extern char Calc_CpCmSm(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim,
		double car_r_limit, double road_yaw[6], double road_yaw2[6]);

	extern bool Calc_SmCmSm(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim,
		double car_r_limit, double road_yaw[6]);

	extern bool Calc_SpCmSm(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim,
		double car_r_limit, double road_yaw[6]);
	extern bool CalcCpSpCp(const double *start, const double* end,const double r_max40,double *road_yaw);
	extern char Calc_NCC(double x_now, double y_now, double theta_now, double x_aim, double y_aim, double theta_aim, const double CC_r); //Ncc
	extern bool CalcSafeRadiusCpCp(const double* start, const double* end, const double &r_safe);
	//extern bool CpSpCp(const double &x, const double &y, const double &phi, const double &curr_min,double* road_yaw);
	//level
	extern int  ParkingInLevel(double Start[3], double End[3]);
	extern int  ParkingOutLevel(double Start1[3], double End1[3]);
	extern void ParkingLevel();
	//vertical
	extern void ParkingVertical();
	extern void ParkingInclined();
	extern bool ParkingDirectly();
	extern bool RearParking(double start[3], double end[3]);
	extern bool RearParkingOut(double* start, double* end);
	extern bool CalcHeadPathDirectly(const double* start, const double* end);
	extern bool FindPathDirectly(double start[3], double end[3], int directly_flag = 0);
	extern bool FindPathMulti(double start[3], double end[3], int& f_flag);
	extern int GetOrCheckPathPointsMulti(const double start[3], const double road_yaw[6], const int &collision_check_flag);
	//dynamic plan
	extern bool RearDynamicPlan(double start[3], double end[3], int dynamic_flag = 0);
	extern bool HeadDynamicPlan(double start[3], double end[3], int dynamic_flag = 0);
	//head
	extern bool HeadParking(double* start, double* end);
	extern bool HeadParkingOut(double* start, double* end);
	//A*
	extern bool HybridAStar(double start[3], double end[3], const double &time_max);
	extern bool VehicleCollisionGrid_Astar(double cpx, double cpy, double cph);
	extern bool CollisionGrid_InitPos_test(double cpx, double cpy, double cph);
	
	extern void ParkingVertical_Astar();
	extern void ParkingLevel_Astar();
	extern void ParkingOblique_Astar();
	//main
	extern void Process();

	//test
	extern void PlotResult();
	//OBCA and corridor
	using Vec2f = Eigen::Matrix<double, 2, 1>;
	extern std::vector<Eigen::MatrixXd> getRectangle(std::vector<Eigen::Vector3d> statelist);
	extern bool CheckIfCollisionUsingLine(const Eigen::Vector2d p1, const Eigen::Vector2d p2, bool *res, double checkl);
	extern bool CheckCollisionUsingGlobalPosition(const Vec2f &p_w, bool *res);
	
	extern void calculatePolytope(const Polygon &polygon, Eigen::MatrixXd &A, Eigen::VectorXd &b);
	extern std::vector<PathPoint> sample_path(const std::vector<PathPoint> path, int sample_interval);
}
#endif