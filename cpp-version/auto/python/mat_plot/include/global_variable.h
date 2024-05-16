#ifndef GLOBAL_VARIABLE_H
#define GLOBAL_VARIABLE_H

#include <iostream>
#include <vector>
#include "matplotlibcpp.h"
#include <fstream>
#include <sstream> // 确保包含这个头文件
#include <vector>
#include "math.h"
#include "string.h"


extern std::string plan_time_show;


struct Coordinate
{
	double X;
	double Y;
	double Yaw;
	double rajectoryDirection;
	double Curvature;
};

struct Plan
{
	Coordinate coordinate[300];
};


#endif