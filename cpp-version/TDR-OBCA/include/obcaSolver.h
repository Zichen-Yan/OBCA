#ifndef _OBCASOLVER_H_
#define _OBCASOLVER_H_

#include <casadi/casadi.hpp>
#include <Eigen/Dense>

#ifndef findpath_H
#include "findpath.h"
#endif

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
    double D_bound = 0.3;
    int sample_interval = 3;

    int n_controls;
    int n_states;
    int n_dual_variable = 4;
    int N;
    int M;

    double beta = 1; // 目标函数权重
    
    ca::MX G;
    ca::MX g;
    Vehicle_config cfg;
    ca::Slice all;

public:
    ca::MX X;
    ca::MX U;
    ca::MX MU;
    ca::MX LAMBDA;
    ca::MX d;
    ca::MX target;

    ca::MX ws_X;
    ca::MX ws_mu;
    ca::MX ws_lambda;
    ca::MX ws_d;

    ca::Opti ws_opti;
    ca::Opti opti;
    std::vector<std::vector<std::pair<double, double>>> obstacles;
    std::vector<path_point> warm_start;
    std::vector<ca::MX> A_obst;
    std::vector<ca::MX> b_obst;
    ca::MX obj;
    ca::MX ws_obj;
    std::vector<double> x0;

    Smoother(const Vehicle_config Vehicle_config, std::vector<path_point> warm_start,
             std::vector<std::vector<std::pair<double, double>>> obst, double endpoint[3]);
    ~Smoother(){};
    ca::Function GetKinematicEquation();
    ca::Function Kinematic;
    std::vector<path_point> sample_path(const std::vector<path_point> path);
    void define_parameter_and_variable();
    void get_warm_start_dual_variables();
    void generate_constrain();
    void generate_variable();
    void generate_object();
    std::vector<path_point> solve();
};

#endif