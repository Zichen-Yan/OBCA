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

std::vector<path_point> Smoother::sample_path(const std::vector<path_point> path)
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
    if (path.size() % sample_interval != 0)
    {
        ans.push_back(end);
    }
    return ans;
}

Smoother::Smoother(const Vehicle_config Vehicle_config, std::vector<path_point> warm_start,
                   std::vector<std::vector<std::pair<double, double>>> obst, double endpoint[3])
    : cfg(Vehicle_config), obstacles(obst)
{
    warm_start = warm_start;
    safe_Length = cfg.LF + 0.2 + cfg.LB + 0.2;
    offset = safe_Length / 2 - (cfg.LB + 0.2);
    WB = cfg.WB;
    safe_width = cfg.W + 0.4;

    G = ca::MX::zeros(4, 2);
    G(0, 0) = 1;
    G(1, 0) = -1;
    G(2, 1) = 1;
    G(3, 1) = -1;
    g = ca::MX::vertcat({safe_Length / 2, safe_Length / 2, 0.5 * safe_width, 0.5 * safe_width});

    Kinematic = GetKinematicEquation();

    std::vector<path_point> path = sample_path(warm_start);

    N = path.size();
    M = obstacles.size();
    // store coarse path as warm start
    for (int i = 0; i < N - 1; ++i)
    {
        path_point s = path[i];
        x0.insert(x0.end(), {s.x, s.y, s.th});
    }
    x0.insert(x0.end(), {endpoint[0], endpoint[1], endpoint[2]});
    // initial control input
    for (int i = 0; i < N - 1; ++i)
    {
        path_point s = path[i];
        x0.insert(x0.end(), {s.delta, s.D});
    }
    // // initial dual variables
    // for(int i = 0;i<n_dual_variable * N * 2 * M;++i)
    //     x0.push_back(0.1);
    // // initial slack variables
    // for(int i = 0;i<N * M;++i)
    //     x0.push_back(0.1);
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

    // warm start
    ws_lambda = ws_opti.variable(n_dual_variable, N * M);
    ws_mu = ws_opti.variable(n_dual_variable, N * M);
    ws_d = ws_opti.variable(1, N * M);
    ws_X = ws_opti.parameter(n_states, N);

    // MPC
    X = opti.variable(n_states, N);
    U = opti.variable(n_controls, N - 1);
    MU = opti.variable(n_dual_variable, N * M);
    LAMBDA = opti.variable(n_dual_variable, N * M);
    d = opti.variable(1, N * M);
}

void Smoother::get_warm_start_dual_variables()
{
    std::cout << "get_warm_start_dual_variables" << std::endl;
    ws_obj = 0;

    // compute obstacles polytope
    ca::MX A1;
    ca::MX b1;
    for (int j = 0; j < M; ++j)
    {
        std::vector<std::pair<double, double>> tmp = obstacles[j];
        Polygon obs = {
            {tmp[0].first, tmp[0].second},
            {tmp[1].first, tmp[1].second},
            {tmp[2].first, tmp[2].second},
            {tmp[3].first, tmp[3].second}};

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

        A_obst.push_back(A1);
        b_obst.push_back(b1);
    }

    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < M; ++j)
        {
            A1 = A_obst[j];
            b1 = b_obst[j];

            ca::MX A1Tlambda = ca::MX::mtimes(A1.T(), ws_lambda(all, M * i + j));
            ws_obj += 1 / beta * ca::MX::mtimes(A1Tlambda.T(), A1Tlambda);

            ca::MX x = ws_X(0, i);
            ca::MX y = ws_X(1, i);
            ca::MX th = ws_X(2, i);

            ca::MX t = vertcat(x + offset * cos(th), // x,y表示后轴中心位置，算上offset后表示车辆的正中点
                               y + offset * sin(th));
            ca::MX r = ca::MX::zeros(2, 2);
            r(0, 0) = cos(th);
            r(0, 1) = -sin(th);
            r(1, 0) = sin(th);
            r(1, 1) = cos(th);

            ca::MX lamb = ws_lambda(all, M * i + j);
            ca::MX mu = ws_mu(all, M * i + j);
            ca::MX dm = ws_d(all, M * i + j);
            ws_obj += dm;

            std::vector<double> mid = {0, 0};
            ca::MX eq = ca::MX(mid);
            ws_opti.subject_to(ca::MX::mtimes(G.T(), mu) + ca::MX::mtimes(ca::MX::mtimes(r.T(), A1.T()), lamb) == eq);
            ws_opti.subject_to(-dot(g, mu) + dot(ca::MX::mtimes(A1, t) - b1, lamb) + dm == 0);
        }
    }

    std::vector<double> tmp;
    ca::MX lb;
    ca::MX ub;
    for (int i = 0; i < M * N; ++i)
    {
        tmp = {0.0, 0.0, 0.0, 0.0};
        lb = ca::MX(tmp);

        ws_opti.subject_to(ws_mu(all, i) >= lb);
        ws_opti.subject_to(ws_lambda(all, i) >= lb);
        ws_opti.subject_to(ws_d(all, i) < 0);
    }
    ws_opti.minimize(ws_obj);

    ca::Dict solver_opts;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["ipopt.max_iter"] = 500;
    // solver_opts["ipopt.hessian_approximation"] = "limited-memory";
    solver_opts["expand"] = true;
    ws_opti.solver("ipopt", solver_opts);

    std::vector<double> p0(x0.begin(), x0.begin() + n_states * N);
    ca::DM p_matrix = ca::DM::reshape(ca::DM(p0), n_states, N);
    
    ws_opti.set_value(ws_X, p_matrix);
    try
    {
        std::unique_ptr<ca::OptiSol> sol = std::make_unique<ca::OptiSol>(ws_opti.solve());
        ws_lambda = casadi::DM(sol->value(ws_lambda));
        ws_mu = casadi::DM(sol->value(ws_mu));
        ws_d = casadi::DM(sol->value(ws_d));
        std::cerr << "warm start successful " << std::endl;
    }
    catch (const std::exception &e)
    {
        // 处理异常，输出变量值
        // std::cerr << "Exception caught: " << e.what() << std::endl;
        std::cout << "Variable ws_lambda: " << ws_opti.debug().value(ws_lambda) << std::endl;
        std::cout << "Variable ws_mu: " << ws_opti.debug().value(ws_mu) << std::endl;
        std::cout << "Variable ws_d: " << ws_opti.debug().value(ws_d) << std::endl;

        ws_lambda = casadi::DM(ws_opti.debug().value(ws_lambda));
        ws_mu = casadi::DM(ws_opti.debug().value(ws_mu));
        ws_d = casadi::DM(ws_opti.debug().value(ws_d));
    }
    // std::cout << "Variable ws_lambda: " << ws_lambda << std::endl;
    // std::cout << "Variable ws_mu: " << ws_mu << std::endl;
    // std::cout << "Variable ws_d: " << ws_d << std::endl;
}

void Smoother::generate_constrain()
{
    std::cout << "generate_constrain" << std::endl;

    opti.subject_to(X(all, 0) == target(all, 0));
    for (int i = 0; i < N - 1; ++i)
    {
        std::vector<ca::MX> input = {X(all, i), U(all, i)};
        ca::MX f_value = ca::MX::vertcat(Kinematic(input));
        opti.subject_to(X(all, i + 1) == f_value);
    }
    opti.subject_to(X(all, -1) == target(all, -1));

    ca::MX A1;
    ca::MX b1;
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < M; ++j)
        {
            A1 = A_obst[j];
            b1 = b_obst[j];

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

            ca::MX lamb = LAMBDA(all, M * i + j);
            ca::MX mu = MU(all, M * i + j);
            ca::MX soft = d(all,M * i + j);

            opti.subject_to(dot(ca::MX::mtimes(A1.T(), lamb), ca::MX::mtimes(A1.T(), lamb)) <= 1);
            std::vector<double> mid = {0, 0};
            ca::MX eq = ca::MX(mid);
            opti.subject_to(ca::MX::mtimes(G.T(), mu) + ca::MX::mtimes(ca::MX::mtimes(r.T(), A1.T()), lamb) == eq);
            // opti.subject_to(-dot(g, mu) + dot(ca::MX::mtimes(A1, t) - b1, lamb) > 0);
            opti.subject_to(-dot(g, mu) + dot(ca::MX::mtimes(A1, t) - b1, lamb) + soft == 0);
        }
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
        opti.subject_to(U(1, i) * U(1, i) <= D_bound * D_bound);
        opti.subject_to(opti.bounded(ca::MX(-cfg.MAX_STEER), U(0, i), ca::MX(cfg.MAX_STEER)));
    }
    // multiplier constraint
    for (int i = 0; i < M * N; ++i)
    {
        tmp = {0.0, 0.0, 0.0, 0.0};
        lb = ca::MX(tmp);
        // tmp = {10000, 10000, 10000, 10000};
        // ub = ca::MX(tmp);

        opti.subject_to(MU(all, i) >= lb);     // opti.bounded(lb, MU(all,i), ub)
        opti.subject_to(LAMBDA(all, i) >= lb); // opti.bounded(lb, LAMBDA(all,i), ub)
        opti.subject_to(d(all,i)<0);
    }
}

void Smoother::generate_object()
{
    std::cout << "generate_object" << std::endl;
    ca::MX R = ca::MX::zeros(2, 2);
    R(1, 1) = 100; // D
    R(0, 0) = 10;  // steering
    ca::MX Q = ca::MX::diag(ca::MX::vertcat({1, 1, 1}));

    ca::MX X_error;
    ca::MX U_error;
    obj = 0;
    for (int i = 0; i < N - 1; ++i)
    {
        ca::MX con = U(all, i); // [steering, D]
        obj += ca::MX::mtimes(ca::MX::mtimes(con.T(), R), con);

        // X接近warm start，steering变化率小
        if (i != 0)
        {
            X_error = X(all, i) - target(all, i);
            obj += ca::MX::mtimes(ca::MX::mtimes(X_error.T(), Q), X_error);

            U_error = U(all, i) - U(all, i - 1);
            obj += ca::MX::mtimes(ca::MX::mtimes(U_error.T(), R), U_error);
        }

        for(int j = 0;j<M;++j)
        {
            obj += beta*d(all,M*i+j);
        }
    }
    // ca::MX Qe = ca::MX::diag(ca::MX::vertcat({1000, 1000, 10000}));
    // X_error = X(all,-1)-target(all,-1);
    // obj += ca::MX::mtimes(ca::MX::mtimes(X_error.T(), Qe), X_error);

    opti.minimize(obj);
}

std::vector<path_point> Smoother::solve()
{
    std::cout << "solve" << std::endl;
    std::cout << "Path length: " << N << std::endl;

    ca::Dict solver_opts;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["ipopt.max_iter"] = 500;
    solver_opts["ipopt.tol"] = 1e-6;
    solver_opts["ipopt.linear_solver"] = "ma27";
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
    opti.set_initial(d,ca::DM(ws_d));
    opti.set_initial(MU, ca::DM(ws_mu));
    opti.set_initial(LAMBDA, ca::DM(ws_lambda));

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
        std::vector<path_point> pts;
        for (int i = 0; i < N; i++)
        {
            path_point tmp;
            tmp.x = data_x[i * n_states];
            tmp.y = data_x[i * n_states + 1];
            tmp.th = data_x[i * n_states + 2];

            tmp.delta = data_u[i * n_states];
            tmp.D = data_u[i * n_states + 1];
            pts.push_back(tmp);
        }
        return pts;
    }
    else
    {
        return warm_start;
    }
}
