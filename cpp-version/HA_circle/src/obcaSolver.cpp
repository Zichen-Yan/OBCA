#include "obcaSolver.h"

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

std::vector<path_point> sample_path(const std::vector<path_point> path, int sample_interval)
{
    int cnt = 0;
    path_point end = path[path.size()-1];
    std::vector<path_point> ans;
    for (const path_point &p : path)
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

Smoother::Smoother(const Vehicle_config Vehicle_config, std::vector<path_point> warm_start)
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
        path_point s = path[i];
        x0.insert(x0.end(), {s.x, s.y, s.th});
    }
    // initial control input
    for (int i = 0; i < N - 1; ++i)
    {
        path_point s = path[i];
        x0.insert(x0.end(), {s.delta, s.D});
    }
}

ca::Function Smoother::GetKinematicEquation()
{
    ca::MX x = ca::MX::sym("x");
    ca::MX y = ca::MX::sym("y");
    ca::MX theta = ca::MX::sym("theta");
    ca::MX state = ca::MX::vertcat({x, y, theta});
    n_states = state.size().first;

    ca::MX steering = ca::MX::sym("steering");
    ca::MX D = ca::MX::sym("D");
    ca::MX control = ca::MX::vertcat({steering, D});
    n_controls = control.size().first;
    // 质点中心dynamics
    // ca::MX beta = atan(lr * tan(steering) / (lr + lf));
    // ca::MX rhs = ca::MX::vertcat({v * cos(theta + beta), v * sin(theta + beta), a, v / lr * sin(beta), steering_rate});

    // 后轴中心dynamics
    // ca::MX rhs = ca::MX::vertcat({v * cos(theta), v * sin(theta), v / WB * tan(steering), steering_rate, a});
    ca::MX R = WB / (tan(steering) + 0.000001);
    ca::MX rhs = ca::MX::vertcat({x - R * sin(theta) + R * sin(theta + D / R),
                                  y + R * cos(theta) - R * cos(theta + D / R),
                                  theta + D / R});

    return ca::Function("kinematic_equation", {state, control}, {rhs});
}

void Smoother::define_parameter_and_variable()
{
    target = opti.parameter(n_states, N);
    X = opti.variable(n_states, N);
    U = opti.variable(n_controls, N - 1);
}

void Smoother::generate_constrain(std::vector<Eigen::MatrixXd> corridor)
{
    corridors = corridor;
    std::cout << "generate_constrain" << std::endl;

    opti.subject_to(X(all, 0) == target(all, 0));
    for (int i = 0; i < N - 1; ++i)
    {
        std::vector<ca::MX> input = {X(all, i), U(all, i)};
        ca::MX f_value = ca::MX::vertcat(Kinematic(input));
        opti.subject_to(X(all, i + 1) == f_value);
    }
    // std::vector<double> xx = {1e-3, 1e-3, 1e-3};
    // ca::MX tmp = ca::MX(xx);
    // opti.subject_to(-tmp <= X(all, -1) - target(all, -1) <= tmp);
    opti.subject_to(X(all, -1) == target(all, -1));

    // std::vector<double> p0(x0.begin(), x0.begin() + n_states * N);
    // Eigen::MatrixXd p_matrix(n_states, N);
    // for (int i=0;i<N;i++)
    // {
    //     for (int j=0;j<n_states;j++)
    //     {
    //         p_matrix(j,i)=p0[i*n_states+j];
    //     }
    // }

    // for (int i = 0; i < N; ++i)
    // {
    //     if (i==N-1)
    //     {
    //         int xxx=1;
    //     }
    //     Eigen::MatrixXd tmp = corridors[i];
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

    ca::MX A1;
    ca::MX b1;
    for (int i = 0; i < N; ++i)
    {
        Eigen::MatrixXd tmp = corridors[i];
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

        A1 = ca::MX::zeros(4, 2);
        A1(0, 0) = A(0, 0);
        A1(0, 1) = A(0, 1);
        A1(1, 0) = A(1, 0);
        A1(1, 1) = A(1, 1);
        A1(2, 0) = A(2, 0);
        A1(2, 1) = A(2, 1);
        A1(3, 0) = A(3, 0);
        A1(3, 1) = A(3, 1);
        b1 = ca::MX::zeros(4, 1);
        b1(0, 0) = b(0, 0);
        b1(1, 0) = b(1, 0);
        b1(2, 0) = b(2, 0);
        b1(3, 0) = b(3, 0);

        ca::MX x = X(0, i);
        ca::MX y = X(1, i);
        ca::MX th = X(2, i);

        ca::MX t = vertcat(x + offset * cos(th), // x,y表示后轴中心位置，算上offset后表示车辆的正中点
                           y + offset * sin(th));
        ca::MX r = ca::MX::zeros(2, 2);
        r(0, 0) = cos(th);
        r(0, 1) = -sin(th);
        r(1, 0) = sin(th);
        r(1, 1) = cos(th);

        ca::MX point1 = ca::MX::zeros(2, 1);
        point1(0,0) = safe_Length/2;
        point1(1,0) = safe_width/2;
        ca::MX X1 = ca::MX::mtimes(r, point1)+t;

        ca::MX point2 = ca::MX::zeros(2, 1);
        point2(0,0) = -safe_Length/2;
        point2(1,0) = safe_width/2;
        ca::MX X2 = ca::MX::mtimes(r, point2)+t;

        ca::MX point3 = ca::MX::zeros(2, 1);
        point3(0,0) = -safe_Length/2;
        point3(1,0) = -safe_width/2;
        ca::MX X3 = ca::MX::mtimes(r, point3)+t;

        ca::MX point4 = ca::MX::zeros(2, 1);
        point4(0,0) = safe_Length/2;
        point4(1,0) = -safe_width/2;
        ca::MX X4 = ca::MX::mtimes(r, point4)+t;

        opti.subject_to(ca::MX::mtimes(A1, X1) <= b1);
        opti.subject_to(ca::MX::mtimes(A1, X2) <= b1);
        opti.subject_to(ca::MX::mtimes(A1, X3) <= b1);
        opti.subject_to(ca::MX::mtimes(A1, X4) <= b1);
    }
}

void Smoother::generate_variable()
{
    std::cout << "generate_variable" << std::endl;
    std::vector<double> tmp;
    ca::MX lb;
    ca::MX ub;
    // state constraints
    for (int i = 0; i < N; ++i)
    {
        tmp = {-max_x, -max_y, -2 * M_PI};
        lb = ca::MX(tmp);
        tmp = {max_x, max_y, 2 * M_PI};
        ub = ca::MX(tmp);
        opti.subject_to(opti.bounded(lb, X(all, i), ub));
    }
    // input constraints
    for (int i = 0; i < N - 1; ++i)
    {
        opti.subject_to(opti.bounded(ca::MX(-D_bound), U(1, i), ca::MX(D_bound)));
        opti.subject_to(opti.bounded(ca::MX(-cfg.MAX_STEER), U(0, i), ca::MX(cfg.MAX_STEER)));
    }
}

void Smoother::generate_object()
{
    std::cout << "generate_object" << std::endl;
    
    obj = 0;
    
    // ca::MX X_error;
    // ca::MX Q = ca::MX::diag(ca::MX::vertcat({1, 1, 1}));
    // for (int i = 0; i < N; ++i)
    // {
    //     X_error = X(all, i) - target(all, i);
    //     obj += ca::MX::mtimes(ca::MX::mtimes(X_error.T(), Q), X_error);
    // }

    ca::MX R = ca::MX::zeros(2, 2);
    R(1, 1) = 1; // D
    R(0, 0) = 100;  // steering
    for (int i = 0; i < N-1; ++i)
    {
        ca::MX con = U(all, i); // [steering, D]
        obj += ca::MX::mtimes(ca::MX::mtimes(con.T(), R), con);
    }
    
    ca::MX U_error;
    ca::MX R_diff = ca::MX::zeros(2, 2);
    R_diff(1, 1) = 100; // D
    R_diff(0, 0) = 10;  // steering
    for (int i = 1; i < N-1; ++i)
    {
        U_error = U(all, i) - U(all, i - 1);
        obj += ca::MX::mtimes(ca::MX::mtimes(U_error.T(), R_diff), U_error);
    }
    opti.minimize(obj);
}

std::vector<path_point> Smoother::solve()
{
    std::cout << "solve" << std::endl;
    std::cout << "Path length: " << N << std::endl;

    ca::Dict solver_opts;
    solver_opts["ipopt.print_level"] = 5;
    solver_opts["ipopt.max_iter"] = 500;

    solver_opts["ipopt.tol"] = 1e-6;
    solver_opts["ipopt.linear_solver"] = "ma27";
    // solver_opts["ipopt.derivative_test"] = "first-order";
    // solver_opts["ipopt.check_derivatives_for_naninf"] = "yes";
    // solver_opts["ipopt.hessian_approximation"] = "limited-memory";
    solver_opts["expand"] = true;
    opti.solver("ipopt", solver_opts);

    std::vector<double> p0(x0.begin(), x0.begin() + n_states * N);
    ca::DM p_matrix = ca::DM::reshape(ca::DM(p0), n_states, N);

    opti.set_value(target, p_matrix);
    opti.set_initial(X, p_matrix);

    std::vector<double> p1(x0.begin() + n_states * N, x0.begin() + n_states * N + n_controls * (N - 1));
    p_matrix = ca::DM::reshape(ca::DM(p1), n_controls, N - 1);
    opti.set_initial(U, p_matrix);

    ca::DM sol_x;
    ca::DM sol_u;
    int flag=0;
    try
    {
        std::unique_ptr<ca::OptiSol> sol = std::make_unique<ca::OptiSol>(opti.solve());
        sol_x = casadi::DM(sol->value(X));
        sol_u = casadi::DM(sol->value(U));
        flag=1;
    }
    catch (const std::exception &e)
    {
        // 处理异常，输出变量值
        std::cerr << "Exception caught: " << e.what() << std::endl;
        // std::cout << "Variable X: " << opti.debug().value(X) << std::endl;
        // std::cout << "Variable U: " << opti.debug().value(U) << std::endl;
        sol_x = casadi::DM(opti.debug().value(X));
        sol_u = casadi::DM(opti.debug().value(U));
    }
    std::vector<double> init_(x0.begin(), x0.begin() + n_states);
    std::cout << "Init:" << init_ << std::endl;
    std::vector<double> final_(x0.begin() + n_states * (N - 1), x0.begin() + n_states * N);
    std::cout << "Final:" << final_ << std::endl;

    // get optimized path
    std::vector<double> data_x = sol_x.get_elements();
    std::vector<double> data_u = sol_u.get_elements();
    std::vector<path_point> pts;
    for (int i = 0; i < N; i++)
    {
        path_point tmp;
        tmp.x = data_x[i * n_states];
        tmp.y = data_x[i * n_states + 1];
        tmp.th = data_x[i * n_states + 2];

        tmp.delta = data_u[i * n_controls];
        tmp.D = data_u[i * n_controls + 1];
        pts.push_back(tmp);
    }

    if (flag==0)
    {
        Eigen::MatrixXd s_matrix(n_states, N);
        for (int i=0;i<N;i++)
        {
            for (int j=0;j<n_states;j++)
            {
                s_matrix(j,i)=p0[i*n_states+j];
            }
        }
        for (int i = 0; i < N; ++i)
        {
            Eigen::MatrixXd tmp = corridors[i];
            Polygon obs = {
                {tmp(2, 3), tmp(3, 3)},
                {tmp(2, 2), tmp(3, 2)},
                {tmp(2, 1), tmp(3, 1)},
                {tmp(2, 0), tmp(3, 0)}};

            // Calculate the polytope for the first obstacle
            Eigen::MatrixXd A;
            Eigen::VectorXd b;

            calculatePolytope(obs, A, b); // 逆时针输入obs角点

            double x = s_matrix(0, i);
            double y = s_matrix(1, i);
            double th = s_matrix(2, i);

            Eigen::VectorXd t(2);
            t << x + offset * cos(th), y + offset * sin(th);
            Eigen::MatrixXd r(2, 2);
            r << cos(th),-sin(th),
                sin(th), cos(th);

            Eigen::VectorXd point1(2);
            point1 << safe_Length/2, safe_width/2;
            Eigen::VectorXd p1 = r*point1+t;
            Eigen::VectorXd X1 = A*p1-b;

            Eigen::VectorXd point2(2);
            point2 << -safe_Length/2, safe_width/2;
            Eigen::VectorXd p2 = r*point2+t;
            Eigen::VectorXd X2 = A*p2-b;

            Eigen::VectorXd point3(2);
            point3 << -safe_Length/2, -safe_width/2;
            Eigen::VectorXd p3 = r*point3+t;
            Eigen::VectorXd X3 = A*p3-b;

            Eigen::VectorXd point4(2);
            point4 << safe_Length/2, -safe_width/2;
            Eigen::VectorXd p4 = r*point4+t;
            Eigen::VectorXd X4 = A*p4-b;

            if (X1(2)>1e-5 || X1(3)>1e-5 || X2(2)>1e-5 || X2(3)>1e-5 || 
            X3(2)>1e-5 || X3(3)>1e-5 || X4(2)>1e-5 || X4(3)>1e-5)
            {
                std::cout << "Choose original Path" << std::endl;
                return path;
            }
        }
    }
    std::cout << "Choose Optimized Path" << std::endl;
    return pts;
}
