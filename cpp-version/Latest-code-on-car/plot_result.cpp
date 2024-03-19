#include "global_function.h"
#include "global_variable.h"
#include "matplotlibcpp.h"

namespace byd_apa_plan
{
    namespace plt = matplotlibcpp;

    void CalcGridCoordinate(const int &index_, double &m_x, double &m_y) // change grid to point coordinates
    {
        if (index_ % 250 == 0)
        {
            m_x = (int)(index_ / 250);
            m_y = 250;
            m_x = (125 - m_x) * 10 + 5;
            m_y = (125 - m_y) * 10 + 5;
            m_x = m_x * 0.01;
            m_y = m_y * 0.01;
        }
        else
        {
            m_x = (int)(index_ / 250) + 1;
            m_y = index_ % 250;
            m_x = (125 - m_x) * 10 + 5;
            m_y = (125 - m_y) * 10 + 5;
            m_x = m_x * 0.01;
            m_y = m_y * 0.01;
        }
    }

    void PlotResult()
    {
        plt::figure_size(1000, 1000);
        plt::xlim(-12.5, 12.5);
        plt::ylim(-12.5, 12.5);
        plt::axis("equal");
        // axis.set_aspect(1);
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> yaw;
        // park
        double p_x[4] = {P0x, P1x, P2x, P3x};
        double p_y[4] = {P0y, P1y, P2y, P3y};
        // path
        for (int i = 0; i < 300; i++)
        {
            if (!(plan.coordinate[i].X == 0 && plan.coordinate[i].Y == 0 && plan.coordinate[i].Yaw == 0))
            {
                x.push_back(plan.coordinate[i].X * 0.01);
                y.push_back(plan.coordinate[i].Y * 0.01);
                yaw.push_back(plan.coordinate[i].Yaw * pi / 180);
            }
        }
        // map
        std::vector<double> m_x;
        std::vector<double> m_y;
        std::vector<double> v_x;
        std::vector<double> v_y;
        std::vector<double> u_x;
        std::vector<double> u_y;
        std::vector<double> o_x;
        std::vector<double> o_y;
        for (int idx = 1; idx <= 62500; idx++)
        {
            double x_c = 0, y_c = 0;
            CalcGridCoordinate(idx, x_c, y_c);
            if (obstmap[idx - 1].Status == 0)
            {
                m_x.push_back(x_c);
                m_y.push_back(y_c);
            }
            else if (obstmap[idx - 1].Status == 3)
            {
                v_x.push_back(x_c);
                v_y.push_back(y_c);
            }
            else if (obstmap[idx - 1].Status == 2)
            {
                u_x.push_back(x_c);
                u_y.push_back(y_c);
            }
            else if (obstmap[idx - 1].Status == 7)
            {
                o_x.push_back(x_c);
                o_y.push_back(y_c);
            }
        }

        // car multi
        std::vector<double> car_x;
        std::vector<double> car_y;
        for (unsigned int i = 0; i < Rect_x.size(); i++)
        {
            double x = Rect_x[i] * cos(start_to_fus[2]) - Rect_y[i] * sin(start_to_fus[2]) + start_to_fus[0];
            double y = Rect_x[i] * sin(start_to_fus[2]) + Rect_y[i] * cos(start_to_fus[2]) + start_to_fus[1];
            car_x.push_back(x);
            car_y.push_back(y);
        }
        //
        int plot_size = 0;
        for (int i = 0; i < 300; i++)
        {
            if (plan.coordinate[i].rajectoryDirection == 0)
            {
                plot_size = i;
                break;
            }
        }
        for (int plot_num = 0; plot_num < plot_size - 1; plot_num += 2)
        {
            std::vector<double> car_x_mid;
            std::vector<double> car_y_mid;
            for (unsigned int i = 0; i < Rect_x.size(); i++)
            {
                double x = Rect_x[i] * cos(plan.coordinate[plot_num].Yaw * pi / 180) - Rect_y[i] * sin(plan.coordinate[plot_num].Yaw * pi / 180) + plan.coordinate[plot_num].X * 0.01;
                double y = Rect_x[i] * sin(plan.coordinate[plot_num].Yaw * pi / 180) + Rect_y[i] * cos(plan.coordinate[plot_num].Yaw * pi / 180) + plan.coordinate[plot_num].Y * 0.01;
                car_x_mid.push_back(x);
                car_y_mid.push_back(y);
            }
             plt::plot(car_x_mid,car_y_mid,"k-");
        }
        //
        std::vector<double> car_x_end;
        std::vector<double> car_y_end;
        for (unsigned int i = 0; i < Rect_x.size(); i++)
        {
            double x = Rect_x[i] * cos(end_to_fus[2]) - Rect_y[i] * sin(end_to_fus[2]) + end_to_fus[0];
            double y = Rect_x[i] * sin(end_to_fus[2]) + Rect_y[i] * cos(end_to_fus[2]) + end_to_fus[1];
            car_x_end.push_back(x);
            car_y_end.push_back(y);
        }
        // car
        plt::plot(car_x, car_y, "b-");
        plt::plot(car_x_end, car_y_end, "y-");
        // map
        plt::plot(m_x, m_y, "r.");
        plt::plot(v_x, v_y, "b.");
        plt::plot(u_x, u_y, "c.");
        plt::plot(o_x, o_y, "m.");
        // park
        plt::plot(p_x, p_y, "g-");
        // path
        plt::plot(x, y, "g-");
        // plt::pause(2); //
        plt::show();
    }
}