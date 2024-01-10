#pragma once

#include <stdlib.h>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include "data_type.h"

class VehicleParam_
{
public:
	inline double width() const { return width_; }
	inline double length() const { return length_; }
	inline double wheel_base() const { return wheel_base_; }
	inline double front_suspension() const { return front_suspension_; }
	inline double rear_suspension() const { return rear_suspension_; }
	inline double max_steering_angle() const { return max_steering_angle_; }
	inline double max_longitudinal_acc() const { return max_longitudinal_acc_; }
	inline double max_lateral_acc() const { return max_lateral_acc_; }
	inline double d_cr() const { return d_cr_; }

	inline void set_width(const double val) { width_ = val; }
	inline void set_length(const double val) { length_ = val; }
	inline void set_wheel_base(const double val) { wheel_base_ = val; }
	inline void set_front_suspension(const double val)
	{
		front_suspension_ = val;
	}
	inline void set_rear_suspension(const double val) { rear_suspension_ = val; }
	inline void set_max_steering_angle(const double val)
	{
		max_steering_angle_ = val;
	}
	inline void set_max_longitudinal_acc(const double val)
	{
		max_longitudinal_acc_ = val;
	}
	inline void set_max_lateral_acc(const double val) { max_lateral_acc_ = val; }
	inline void set_d_cr(const double val) { d_cr_ = val; }

	/**
	 * @brief Print info
	 */
	void print() const;

private:
	double width_ = vehicle_parameters.W+0.4;
	double length_ = vehicle_parameters.LF+vehicle_parameters.LB+0.4;
	double wheel_base_ = vehicle_parameters.WB;
	double front_suspension_ = vehicle_parameters.LF-vehicle_parameters.WB;
	double rear_suspension_ = vehicle_parameters.LB;
	double max_steering_angle_ = vehicle_parameters.MAX_STEER/M_PI*180;
	double max_longitudinal_acc_ = 2.0;
	double max_lateral_acc_ = 2.0;
	double d_cr_ = (vehicle_parameters.LF+vehicle_parameters.LB)/2-vehicle_parameters.LB; // 1.015;  // length between geometry center and rear axle
};

using Vec2f = Eigen::Matrix<double, 2, 1>;
extern std::vector<Eigen::MatrixXd> hPolys_; //

std::vector<Eigen::MatrixXd> getRectangle(std::vector<Eigen::Vector3d> statelist);
void CheckIfCollisionUsingLine(const Eigen::Vector2d p1, const Eigen::Vector2d p2, bool *res, double checkl);
void CheckCollisionUsingGlobalPosition(const Vec2f &p_w, bool *res);