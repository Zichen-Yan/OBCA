#pragma once

#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>
#include "data_type.h"

namespace ca = casadi;

// Define a type representing a 2D point
typedef Eigen::Vector2d Point;

// Define a type representing the vertices of a polygon
typedef std::vector<Point> Polygon;

void calculatePolytope(const Polygon &polygon, Eigen::MatrixXd &A, Eigen::VectorXd &b);

class Smoother
{
private:
    double safe_Length;
    double safe_width;
    double offset;
    double WB;
    double max_x = 12.5;
    double max_y = 12.5;
    double D_bound = 0.2;

    int n_controls;
    int n_states;
    int n_dual_variable = 4;
    int N;

    double beta = 1; // 目标函数权重
    
    Vehicle_config cfg;
    ca::Slice all;

public:
    ca::MX X;
    ca::MX U;
    ca::MX target;

    ca::Opti opti;
    std::vector<path_point> path;
    ca::MX obj;
    std::vector<double> x0;

    Smoother(const Vehicle_config Vehicle_config, std::vector<path_point> warm_start);
    ~Smoother(){};
    ca::Function GetKinematicEquation();
    ca::Function Kinematic;
    void define_parameter_and_variable();
    void generate_constrain(std::vector<Eigen::MatrixXd> corridor);
    void generate_variable();
    void generate_object();
    std::vector<path_point> solve();
};

std::vector<path_point> sample_path(const std::vector<path_point> path, int sample_interval);
