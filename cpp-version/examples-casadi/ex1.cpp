#include <iostream>
#include <vector>
#include <casadi/casadi.hpp>

using namespace std;
using namespace casadi;

int main(int argc, char *argv[])
{

    cout << "casadi_test" << endl;


    // This is another way to define a nonlinear solver. Opti is new
    /*
     *    min  x1^2 + x2^2 + x3^2
     *    s.t.    6*x1 + 3&x2 + 2*x3 - p0 = 0
     *              p2*x1 + x2 - x3 - 1 = 0
     *              x1, x2, x3 >= 0
     */

    // Optimization variables
    SX x = SX::sym("x", 3);
    std::cout << "x:" << x << std::endl;

    // Parameters
    SX p = SX::sym("p", 2);
    std::cout << "p:" << p << std::endl;

    // Objective
    SX f = x(0) * x(0) + x(1) * x(1) + x(2) * x(2);
    std::cout << "f:" << f << std::endl;

    // Constraints
    SX g = vertcat(6 * x(0) + 3 * x(1) + 2 * x(2) - p(0), p(1) * x(0) + x(1) - x(2) - 1);
    std::cout << "g:" << g << std::endl;

    // Initial guess and bounds for the optimization variables
    vector<double> x0 = {0.15, 0.15, 0.00};
    vector<double> lbx = {0.00, 0.00, 0.00};
    vector<double> ubx = {inf, inf, inf};

    // Nonlinear bounds
    vector<double> lbg = {0.00, 0.00};
    vector<double> ubg = {0.00, 0.00};

    // Original parameter values
    vector<double> p0 = {5.00, 1.00};

    // NLP
    SXDict nlp = {{"x", x}, {"p", p}, {"f", f}, {"g", g}};

    // Create NLP solver and buffers
    Function solver = nlpsol("solver", "ipopt", nlp);
    std::map<std::string, DM> arg, res;

    // Solve the NLP
    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;
    arg["x0"] = x0;
    arg["p"] = p0;
    res = solver(arg);

    // Print the solution
    cout << "--------------------------------" << endl;
    cout << "Optimal solution for p = " << arg.at("p") << ":" << endl;
    cout << setw(30) << "Objective: " << res.at("f") << endl;
    cout << setw(30) << "Primal solution: " << res.at("x") << endl;
    cout << setw(30) << "Dual solution (x): " << res.at("lam_x") << endl;
    cout << setw(30) << "Dual solution (g): " << res.at("lam_g") << endl;

    return 0;
}
