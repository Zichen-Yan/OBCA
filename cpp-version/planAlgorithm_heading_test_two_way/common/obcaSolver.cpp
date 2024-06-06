#include "global_variable.h"
#include "global_function.h"

namespace byd_apa_plan
{
    void calculatePolytope(const Polygon &polygon, Eigen::MatrixXd &A, Eigen::VectorXd &b)
    {
        int numEdges = polygon.size();

        A.resize(numEdges, 2);
        b.resize(numEdges);

        for (int i = 0; i < numEdges; ++i)
        {
            Eigen::Vector2d edgeVector = polygon[(i + 1) % numEdges] - polygon[i];
            Eigen::Vector2d normalVector(edgeVector.y(), -edgeVector.x()); // Corrected sign
            normalVector.normalize();

            // Set the row of matrix A
            A.row(i) = normalVector;

            // Set the element of vector b
            b(i) = normalVector.dot(polygon[i]);
        }
    }

    std::vector<PathPoint> sample_path(const std::vector<PathPoint> path, int sample_interval)
    {
        int cnt = 0;
        PathPoint end = path[path.size()-1];
        std::vector<PathPoint> ans;
        for (const PathPoint &p : path)
        {
            if (cnt % sample_interval == 0)
            {
                ans.push_back(p);
            }
            cnt++;
        }
        if ((path.size()-1) % sample_interval != 0)
        {
            ans.push_back(end);
        }
        return ans;
    }

    Smoother::Smoother(const VehicleParam Vehicle_config, std::vector<PathPoint> warm_start)
        : cfg(Vehicle_config), path(warm_start)
    {
        safe_Length = cfg.LF + 0.2 + cfg.LB + 0.2;
        offset = safe_Length / 2 - (cfg.LB + 0.2);
        WB = cfg.WB;
        safe_width = cfg.W + 0.4;

        Kinematic = GetKinematicEquation();

        N = path.size();
        // store coarse path as warm start
        for (int i = 0; i < N; ++i)
        {
            PathPoint s = path[i];
            x0.insert(x0.end(), {s.x, s.y, s.th});
        }
        // initial control input
        for (int i = 0; i < N - 1; ++i)
        {
            PathPoint s = path[i];
            x0.insert(x0.end(), {s.delta, s.D});
        }
    }

    casadi::Function Smoother::GetKinematicEquation()
    {
        casadi::MX x = casadi::MX::sym("x");
        casadi::MX y = casadi::MX::sym("y");
        casadi::MX theta = casadi::MX::sym("theta");
        casadi::MX state = casadi::MX::vertcat({x, y, theta});
        n_states = state.size().first;

        casadi::MX steering = casadi::MX::sym("steering");
        casadi::MX D = casadi::MX::sym("D");
        casadi::MX control = casadi::MX::vertcat({steering, D});
        n_controls = control.size().first;
        // 质点中心dynamics
        // casadi::MX beta = atan(lr * tan(steering) / (lr + lf));
        // casadi::MX rhs = casadi::MX::vertcat({v * cos(theta + beta), v * sin(theta + beta), a, v / lr * sin(beta), steering_rate});

        // 后轴中心dynamics
        // casadi::MX rhs = casadi::MX::vertcat({v * cos(theta), v * sin(theta), v / WB * tan(steering), steering_rate, a});
        casadi::MX R = WB / (tan(steering) + 0.000001);
        casadi::MX rhs = casadi::MX::vertcat({x - R * sin(theta) + R * sin(theta + D / R),
                                    y + R * cos(theta) - R * cos(theta + D / R),
                                    theta + D / R});

        return casadi::Function("kinematic_equation", {state, control}, {rhs});
    }

    void Smoother::define_parameter_and_variable()
    {
        target = opti.parameter(n_states, N);
        X = opti.variable(n_states, N);
        U = opti.variable(n_controls, N - 1);
    }

    void Smoother::generate_constrain(std::vector<Eigen::MatrixXd> corridor)
    {
        std::cout << "generate_constrain" << std::endl;

        opti.subject_to(X(all, 0) == target(all, 0));
        for (int i = 0; i < N - 1; ++i)
        {
            std::vector<casadi::MX> input = {X(all, i), U(all, i)};
            casadi::MX f_value = casadi::MX::vertcat(Kinematic(input));
            opti.subject_to(X(all, i + 1) == f_value);
        }
        opti.subject_to(X(all, -1) == target(all, -1));

        casadi::MX A1;
        casadi::MX b1;

        std::vector<double> p0(x0.begin(), x0.begin() + n_states * N);
        Eigen::MatrixXd p_matrix(n_states, N);
        for (int i=0;i<N;i++)
        {
            for (int j=0;j<n_states;j++)
            {
                p_matrix(j,i)=p0[i*n_states+j];
            }
        }

        // for (int i = 0; i < N; ++i)
        // {
        //     if (i==N-1)
        //     {
        //         int xxx=1;
        //     }
        //     Eigen::MatrixXd tmp = corridor[i];
        //     Polygon obs = {
        //         {tmp(2, 3), tmp(3, 3)},
        //         {tmp(2, 2), tmp(3, 2)},
        //         {tmp(2, 1), tmp(3, 1)},
        //         {tmp(2, 0), tmp(3, 0)}};

        //     // Calculate the polytope for the first obstacle
        //     Eigen::MatrixXd A;
        //     Eigen::VectorXd b;

        //     calculatePolytope(obs, A, b); // 逆时针输入obs角点

        //     double x = p_matrix(0, i);
        //     double y = p_matrix(1, i);
        //     double th = p_matrix(2, i);

        //     Eigen::VectorXd t(2);
        //     t << x + offset * cos(th), y + offset * sin(th);
        //     Eigen::MatrixXd r(2, 2);
        //     r << cos(th),-sin(th),
        //          sin(th), cos(th);

        //     Eigen::VectorXd point1(2);
        //     point1 << safe_Length/2, safe_width/2;
        //     Eigen::VectorXd p1 = r*point1+t;
        //     Eigen::VectorXd X1 = A*p1-b;

        //     Eigen::VectorXd point2(2);
        //     point2 << -safe_Length/2, safe_width/2;
        //     Eigen::VectorXd p2 = r*point2+t;
        //     Eigen::VectorXd X2 = A*p2-b;

        //     Eigen::VectorXd point3(2);
        //     point3 << -safe_Length/2, -safe_width/2;
        //     Eigen::VectorXd p3 = r*point3+t;
        //     Eigen::VectorXd X3 = A*p3-b;

        //     Eigen::VectorXd point4(2);
        //     point4 << safe_Length/2, -safe_width/2;
        //     Eigen::VectorXd p4 = r*point4+t;
        //     Eigen::VectorXd X4 = A*p4-b;
        // }

        for (int i = 0; i < N; ++i)
        {
            Eigen::MatrixXd tmp = corridor[i];
            // counterclock
            Polygon obs = {
                {tmp(2, 3), tmp(3, 3)},
                {tmp(2, 2), tmp(3, 2)},
                {tmp(2, 1), tmp(3, 1)},
                {tmp(2, 0), tmp(3, 0)}};

            // Calculate the polytope for the first obstacle
            Eigen::MatrixXd A;
            Eigen::VectorXd b;

            calculatePolytope(obs, A, b);

            A1 = casadi::MX::zeros(4, 2);
            A1(0, 0) = A(0, 0);
            A1(0, 1) = A(0, 1);
            A1(1, 0) = A(1, 0);
            A1(1, 1) = A(1, 1);
            A1(2, 0) = A(2, 0);
            A1(2, 1) = A(2, 1);
            A1(3, 0) = A(3, 0);
            A1(3, 1) = A(3, 1);
            b1 = casadi::MX::zeros(4, 1);
            b1(0, 0) = b(0, 0);
            b1(1, 0) = b(1, 0);
            b1(2, 0) = b(2, 0);
            b1(3, 0) = b(3, 0);

            casadi::MX x = X(0, i);
            casadi::MX y = X(1, i);
            casadi::MX th = X(2, i);

            casadi::MX t = vertcat(x + offset * cos(th), // x,y表示后轴中心位置，算上offset后表示车辆的正中点
                            y + offset * sin(th));
            casadi::MX r = casadi::MX::zeros(2, 2);
            r(0, 0) = cos(th);
            r(0, 1) = -sin(th);
            r(1, 0) = sin(th);
            r(1, 1) = cos(th);

            casadi::MX point1 = casadi::MX::zeros(2, 1);
            point1(0,0) = safe_Length/2;
            point1(1,0) = safe_width/2;
            casadi::MX X1 = casadi::MX::mtimes(r, point1)+t;

            casadi::MX point2 = casadi::MX::zeros(2, 1);
            point2(0,0) = -safe_Length/2;
            point2(1,0) = safe_width/2;
            casadi::MX X2 = casadi::MX::mtimes(r, point2)+t;

            casadi::MX point3 = casadi::MX::zeros(2, 1);
            point3(0,0) = -safe_Length/2;
            point3(1,0) = -safe_width/2;
            casadi::MX X3 = casadi::MX::mtimes(r, point3)+t;

            casadi::MX point4 = casadi::MX::zeros(2, 1);
            point4(0,0) = safe_Length/2;
            point4(1,0) = -safe_width/2;
            casadi::MX X4 = casadi::MX::mtimes(r, point4)+t;

            opti.subject_to(casadi::MX::mtimes(A1, X1) <= b1);
            opti.subject_to(casadi::MX::mtimes(A1, X2) <= b1);
            opti.subject_to(casadi::MX::mtimes(A1, X3) <= b1);
            opti.subject_to(casadi::MX::mtimes(A1, X4) <= b1);
        }
    }

    void Smoother::generate_variable()
    {
        std::cout << "generate_variable" << std::endl;
        std::vector<double> tmp;
        casadi::MX lb;
        casadi::MX ub;
        // state constraints
        for (int i = 0; i < N; ++i)
        {
            tmp = {-max_x, -max_y, -2 * M_PI};
            lb = casadi::MX(tmp);
            tmp = {max_x, max_y, 2 * M_PI};
            ub = casadi::MX(tmp);
            opti.subject_to(opti.bounded(lb, X(all, i), ub));
        }
        // input constraints
        for (int i = 0; i < N - 1; ++i)
        {
            opti.subject_to(opti.bounded(casadi::MX(-D_bound), U(1, i), casadi::MX(D_bound)));
            opti.subject_to(opti.bounded(casadi::MX(-cfg.MAX_STEER), U(0, i), casadi::MX(cfg.MAX_STEER)));
        }
    }

    void Smoother::generate_object()
    {
        std::cout << "generate_object" << std::endl;
        
        obj = 0;
        
        // casadi::MX X_error;
        // casadi::MX Q = casadi::MX::diag(casadi::MX::vertcat({1, 1, 1}));
        // for (int i = 0; i < N; ++i)
        // {
        //     X_error = X(all, i) - target(all, i);
        //     obj += casadi::MX::mtimes(casadi::MX::mtimes(X_error.T(), Q), X_error);
        // }

        casadi::MX R = casadi::MX::zeros(2, 2);
        R(1, 1) = 10; // D
        R(0, 0) = 10;  // steering
        for (int i = 0; i < N-1; ++i)
        {
            casadi::MX con = U(all, i); // [steering, D]
            obj += casadi::MX::mtimes(casadi::MX::mtimes(con.T(), R), con);
        }
        
        casadi::MX U_error;
        casadi::MX R_diff = casadi::MX::zeros(2, 2);
        R_diff(1, 1) = 100; // D
        R_diff(0, 0) = 10;  // steering
        for (int i = 1; i < N-1; ++i)
        {
            U_error = U(all, i) - U(all, i - 1);
            obj += casadi::MX::mtimes(casadi::MX::mtimes(U_error.T(), R_diff), U_error);
        }
        opti.minimize(obj);
    }

    std::vector<PathPoint> Smoother::solve()
    {
        std::cout << "solve" << std::endl;
        std::cout << "Path length: " << N << std::endl;

        casadi::Dict solver_opts;
        solver_opts["ipopt.print_level"] = 0;
        solver_opts["ipopt.max_iter"] = 500;
        solver_opts["ipopt.tol"] = 1e-6;
        solver_opts["ipopt.linear_solver"] = "ma27";
        // solver_opts["ipopt.hessian_approximation"] = "limited-memory";
        solver_opts["expand"] = true;
        opti.solver("ipopt", solver_opts);

        std::vector<double> p0(x0.begin(), x0.begin() + n_states * N);
        casadi::DM p_matrix = casadi::DM::reshape(casadi::DM(p0), n_states, N);

        opti.set_value(target, p_matrix);
        opti.set_initial(X, p_matrix);

        std::vector<double> p1(x0.begin() + n_states * N, x0.begin() + n_states * N + n_controls * (N - 1));
        p_matrix = casadi::DM::reshape(casadi::DM(p1), n_controls, N - 1);
        opti.set_initial(U, p_matrix);

        casadi::DM sol_x;
        casadi::DM sol_u;
        int flag=0;
        try
        {
            std::unique_ptr<casadi::OptiSol> sol = std::make_unique<casadi::OptiSol>(opti.solve());
            sol_x = casadi::DM(sol->value(X));
            sol_u = casadi::DM(sol->value(U));
            flag=1;
        }
        catch (const std::exception &e)
        {
            // 处理异常，输出变量值
            std::cerr << "Exception caught: " << e.what() << std::endl;
            std::cout << "Variable X: " << opti.debug().value(X) << std::endl;
            // std::cout << "Variable U: " << opti.debug().value(U) << std::endl;
            // std::cout << "Variable d: " << opti.debug().value(d) << std::endl;
            // std::cout << "Variable LAMBDA: " << opti.debug().value(LAMBDA) << std::endl;
            sol_x = casadi::DM(opti.debug().value(X));
            sol_u = casadi::DM(opti.debug().value(U));
        }
        std::vector<double> init_(x0.begin(), x0.begin() + n_states);
        std::cout << "Init:" << init_ << std::endl;
        std::vector<double> final_(x0.begin() + n_states * (N - 1), x0.begin() + n_states * N);
        std::cout << "Final:" << final_ << std::endl;

        if (flag==1)
        {   
            // get optimized path
            std::vector<double> data_x = sol_x.get_elements();
            std::vector<double> data_u = sol_u.get_elements();
            std::vector<PathPoint> pts;
            for (int i = 0; i < N; i++)
            {
                PathPoint tmp;
                tmp.x = data_x[i * n_states];
                tmp.y = data_x[i * n_states + 1];
                tmp.th = data_x[i * n_states + 2];

                tmp.delta = data_u[i * n_controls];
                tmp.D = data_u[i * n_controls + 1];
                pts.push_back(tmp);
            }
            return pts;
        }
        else
        {
            return path;
        }
    }
}
