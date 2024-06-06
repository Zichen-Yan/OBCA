#include "global_variable.h"
#include "global_function.h"

namespace byd_apa_plan
{
	void ChangeMapPoint(const std::vector<double> &vir_x, const std::vector<double> &vir_y, int vir_index, int change_flag)
	{
		double x, y;
		for (int i = 0; i < vir_index; i++)
		{
			x = vir_x[i] * cos(end_to_fus[2]) - vir_y[i] * sin(end_to_fus[2]) + end_to_fus[0];
			y = vir_y[i] * cos(end_to_fus[2]) + vir_x[i] * sin(end_to_fus[2]) + end_to_fus[1];
			int idx_x = ceil((pathfind_parameters.MAXX - x) / pathfind_parameters.MOTION_RESOLUTION);
			int idx_y = ceil((pathfind_parameters.MAXY - y) / pathfind_parameters.MOTION_RESOLUTION);
			if (idx_x == 0)
			{
				idx_x = 1;
			}
			if (idx_y == 0)
			{
				idx_y = 1;
			}
			int index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
			if (change_flag == 0)
			{
				if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && (idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
				{
					if (obstmap[index_xy].Status == 2u ) // || obstmap[index_xy].Status == 1u
					{
						obstmap[index_xy].Status = 7u;
					}
				}
			}
			else
			{
				if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && (idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
				{
					if (obstmap[index_xy].Status == 0u)
					{
						obstmap[index_xy].Status = 8u;
					}
					else if (obstmap[index_xy].Status == 7u)
					{
						obstmap[index_xy].Status = 5u;
					}
				}
			}
		}
	}

	void GenerateVirtualPoint()
	{
		double dist_P01_ = dist_P01 * 0.01;
		double dist_P23_ = dist_P23 * 0.01;
		int vir_index = 100 * virtual_obs_length / 3;
		std::vector<double> vir_point_x;
		std::vector<double> vir_point_y;
		double x, y, phi, P_virx, P_viry;
		if ((ParkingSpaceFlag == 1) && (dist_P01_ > 0.15) && (dist_P01_ < 1.0))
		{
			CoordinateTrans(x, y, phi, P0x, P0y, .0, end_to_fus[0], end_to_fus[1], end_to_fus[2]);
			P_virx = x + 0.1;
			P_viry = y + dist_P01_;
			for (int i = 0; i < vir_index; i++)
			{
				vir_point_x.push_back(P_virx - i * 0.03);
				vir_point_y.push_back(P_viry);
			}
			ChangeMapPoint(vir_point_x, vir_point_y, vir_index, 0);
		}
		else if ((ParkingSpaceFlag == -1) && (dist_P23_ > 0.15) && (dist_P23_ < 1.0))
		{
			CoordinateTrans(x, y, phi, P3x, P3y, .0, end_to_fus[0], end_to_fus[1], end_to_fus[2]);
			P_virx = x + 0.1;
			P_viry = y - dist_P23_;
			for (int i = 0; i < vir_index; i++)
			{
				vir_point_x.push_back(P_virx - i * 0.03);
				vir_point_y.push_back(P_viry);
			}
			ChangeMapPoint(vir_point_x, vir_point_y, vir_index, 0);
		}
	}

	int GetMapPointIndex(const int& x, const int& y)
	{
		int temp;
		if (x < 0 && y < 0)
		{
			temp = 125 - (y - 10) / 10 + (125 - (x - 10) / 10 - 1) * pathfind_parameters.XIDX;
		}
		else if (x > 0 && y < 0)
		{
			temp = 125 - (y - 10) / 10 + (125 - x / 10 - 1) * pathfind_parameters.XIDX;
		}
		else if (x < 0 && y > 0)
		{
			temp = 125 - y / 10 + (125 - (x - 10) / 10 - 1) * pathfind_parameters.XIDX;
		}
		else
		{
			temp = 125 - y / 10 + (125 - x / 10) * pathfind_parameters.XIDX;
		}
		return temp;
	}

	void CalcMapPointCoordinate(const int& index_, int& m_x, int& m_y) // change grid to point coordinates
	{
		if (index_ % pathfind_parameters.XIDX == 0)
		{
			m_x = (int)(index_ / pathfind_parameters.XIDX);
			m_y = pathfind_parameters.XIDY;
			m_x = (pathfind_parameters.XIDX * 0.5 - m_x) * 10 + 5;
			m_y = (pathfind_parameters.XIDY * 0.5 - m_y) * 10 + 5;
		}
		else
		{
			m_x = (int)(index_ / pathfind_parameters.XIDX) + 1;
			m_y = index_ % pathfind_parameters.XIDY;
			m_x = (pathfind_parameters.XIDX * 0.5 - m_x) * 10 + 5;
			m_y = (pathfind_parameters.XIDY * 0.5 - m_y) * 10 + 5;
		}
	}

	int CheckPointInOrOutPrk(const int& x, const int& y)
	{
		double x0 = P0x * 100;
		double y0 = P0y * 100;
		double x1 = P1x * 100;
		double y1 = P1y * 100;
		double x2 = P2x * 100;
		double y2 = P2y * 100;
		double x3 = P3x * 100;
		double y3 = P3y * 100;
		double S_01 = (x1 - x0) * (y - y0) - (y1 - y0) * (x - x0);
		double S_12 = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1);
		double S_23 = (x3 - x2) * (y - y2) - (y3 - y2) * (x - x2);
		double S_30 = (x0 - x3) * (y - y3) - (y0 - y3) * (x - x3);
		if (((S_01 >= 0) && (S_12 >= 0) && (S_23 >= 0) && (S_30 >= 0)) || ((S_01 <= 0) && (S_12 <= 0) && (S_23 <= 0) && (S_30 <= 0)))
			return 1;
		else
			return 0;
	}

	void CalcParkFrontDist(double &dist_to_p03)
	{
		double x0 = P0x * 100, y0 = P0y * 100;
		// double x1 = P1x * 100, y1 = P1y * 100;
		// double x2 = P2x * 100, y2 = P2y * 100;
		double x3 = P3x * 100, y3 = P3y * 100;
		int index = 0, f_index = 0;  
		int m_x = 0, m_y = 0;
		// int row_max = 10;
		// int col_max = 15;
		double x_curr=0;
		double y_curr=0;
		double line_k = 0;
		if(x0 != x3)
		{
			line_k = (y0 - y3) / (x0 - x3);
		}
		if (ParkingSpaceFlag == 1)
		{
			x_curr=x3;
			y_curr=y3;
		}
		else if (ParkingSpaceFlag == -1)
		{
			x_curr=x0;
			y_curr=y0;
		}
		f_index = GetMapPointIndex(x_curr, y_curr);
			
		for (int i = 0 ; i < row_max_front; i++)
		{
			for (int j = 0; j < col_max_front; j++)
			{
				int row=(i-row_max_front)*ParkingSpaceFlag;
				int col = -j;
				index = f_index + col * pathfind_parameters.XIDY + row;
				if (index > 1 && index < pathfind_parameters.MAX_IDX && obstmap[index - 1].Status == 0)
				{
					CalcMapPointCoordinate(index, m_x, m_y);			
					double dist = -1;
					if (line_k == 0)
					{
						dist = abs((double)m_y - y_curr);
					}
					else
					{									
						double line_b = -line_k * x_curr + y_curr;
						dist = abs(line_k * (double)m_x - (double)m_y + line_b);
						dist = dist / (sqrt(1 + line_k * line_k));
					}
					dist_to_p03 = dist + 5; //2023.03.09 -10
					return;
				}
				//obstmap[index - 1].Status = 6;
			}
		}
	}

	void CalcParkSideDist(double& d01, double& d23, const int &status)
	{
		double x0 = P0x * 100, y0 = P0y * 100;
		double x1 = P1x * 100, y1 = P1y * 100;
		double x2 = P2x * 100, y2 = P2y * 100;
		double x3 = P3x * 100, y3 = P3y * 100;
		double xc1 = (x0 + x1) / 2, yc1 = (y0 + y1) / 2;
		double xc2 = (x2 + x3) / 2, yc2 = (y2 + y3) / 2;
		double xa = (x0 + x3) / 2, ya = (y0 + y3) / 2;
		double xb = (x1 + x2) / 2, yb = (y1 + y2) / 2;
		double xcn = (x0 + xc1 + xc2 + x3) / 4, ycn = (y0 + yc1 + yc2 + y3) / 4;
		xcn = (xa + xcn) / 2;
		ycn = (ya + ycn) / 2;

		int temp_data0 = 0, temp_data3 = 0;
		int index = 0, f_index = 0;
		//int S_ab3 = bool((xa - xb) * (y3 - yb) - (x3 - xb) * (ya - yb) > 0);
		//int S_ab0 = bool((xa - xb) * (y0 - yb) - (x0 - xb) * (ya - yb) > 0);
		double S_ab0 = (x0 * ya + xa * yb + xb * y0 - x0 * yb - xa * y0 - xb * ya);
		double S_ab3 = (x3 * ya + xa * yb + xb * y3 - x3 * yb - xa * y3 - xb * ya);
		bool flag_ab0 = bool(S_ab0 > 0);
		bool flag_ab3 = bool(S_ab3 > 0);
		
		double m_l = .0, min_l_P01 = 1e6, min_l_P23 = 1e6;
		int min_index_P01 = 625, min_index_P23 = 625;
		int m_x = 0, m_y = 0;

		int l_max_0 = round(sqrt((x0 - xc2) * (x0 - xc2) + (y0 - yc2) * (y0 - yc2)) / 2);
		int l_max_3 = round(sqrt((x3 - xc1) * (x3 - xc1) + (y3 - yc1) * (y3 - yc1)) / 2);
		int l_max = l_max_0 > l_max_3 ? l_max_0 : l_max_3;
		int l_sqr = round((l_max + 100) / 10);
		int w_sqr = round(l_sqr/2);
		f_index = GetMapPointIndex(xcn, ycn);
		for (int i = -l_sqr; i < l_sqr; i++)
		{
			for (int j = -w_sqr; j < w_sqr; j++)
			{
				index = f_index + i * pathfind_parameters.XIDY + j;
				if (index > 1 && index < pathfind_parameters.MAX_IDX && (obstmap[index - 1].Status == status))
				{
					CalcMapPointCoordinate(index, m_x, m_y);
					//m_l = ((xa - xb) * (m_y - yb) - (m_x - xb) * (ya - yb));
					m_l = (m_x * ya + xa * yb + xb * m_y - m_x * yb - xa * m_y - xb * ya);
					bool flag_ab = bool(m_l > 0);
					m_l = fabs(m_l);
					if ((flag_ab == flag_ab0) && (m_l > fabs(S_ab0)) && m_l < min_l_P01)
					{
						temp_data0++;
						min_l_P01 = m_l;
						min_index_P01 = index;
					}
					if ((flag_ab == flag_ab3) && (m_l > fabs(S_ab3)) && m_l < min_l_P23)
					{
						temp_data3++;
						min_l_P23 = m_l;
						min_index_P23 = index;
					}
				}
			}
		}

		double dmn = -1;
		if (temp_data0 > 0)
		{
			CalcMapPointCoordinate(min_index_P01, m_x, m_y);
			if (x1 == x0)
				dmn = abs((double)m_x - x0);
			else
			{
				double mn_k = (y1 - y0) / (x1 - x0);
				double mn_b = -mn_k * x0 + y0;
				dmn = abs(mn_k * (double)m_x - (double)m_y + mn_b); //////error
				dmn = dmn / (sqrt(1 + mn_k * mn_k));
				dmn = dmn - 0; //2023.03.09 -10
			}
			if (CheckPointInOrOutPrk(m_x, m_y) == 1)
				d01 = -dmn;
			else
				d01 = dmn;
		}
		if (temp_data3 > 0)
		{
			CalcMapPointCoordinate(min_index_P23, m_x, m_y);
			if (x3 == x2)
				dmn = abs((double)m_x - x3);
			else
			{
				double mn_k = (y3 - y2) / (x3 - x2);
				double mn_b = -mn_k * x3 + y3;
				dmn = abs(mn_k * (double)m_x - (double)m_y + mn_b);
				dmn = dmn / (sqrt(1 + mn_k * mn_k));
				dmn = dmn - 0; //2023.03.09 -10
			}
			if (CheckPointInOrOutPrk(m_x, m_y) == 1)
				d23 = -dmn;
			else
				d23 = dmn;
		}
	}
	//
	void CalcParkTowardDist(double &toward_dist)
	{
		double x0 = P0x * 100, y0 = P0y * 100;
		// double x1 = P1x * 100, y1 = P1y * 100;
		// double x2 = P2x * 100, y2 = P2y * 100;
		double x3 = P3x * 100, y3 = P3y * 100;
		int index = 0, f_index = 0;  
		int m_x = 0, m_y = 0;
		//
		// int row_max = 20;
		// int col_max = 50;
		// int space_start = 450;
		// int obs_count_max = 1;
		//
		double x_curr=0;
		double y_curr=0;
		double line_k = 0;
		if(x0 != x3)
		{
			line_k = (y0 - y3) / (x0 - x3);
		}
		if (ParkingSpaceFlag == 1)
		{
			x_curr=x0;
			y_curr=y0 + toward_space_start * 100;
		}
		else if (ParkingSpaceFlag == -1)
		{
			x_curr=x3;
			y_curr=y3 - toward_space_start * 100;
		}
		f_index = GetMapPointIndex(x_curr, y_curr);
		for (int i = 0 ; i < row_max_toward; i++)
		{
			int obs_count = 0;
			for (int j = 0; j < col_max_toward; j++)
			{
				int row = -i*ParkingSpaceFlag;
				int col = -j;
				index = f_index + col * pathfind_parameters.XIDY + row;
				//std::cout << "index = " << index << std::endl;
				if (index > 1 && index < pathfind_parameters.MAX_IDX && obstmap[index - 1].Status == 0)
				{
					obs_count++;
					if(obs_count == obs_count_max)
					{
						CalcMapPointCoordinate(index, m_x, m_y);			
						double dist = -1;
						if (line_k == 0)
						{
							dist = abs((double)m_y - (y_curr - ParkingSpaceFlag * toward_space_start * 100));
						}
						else
						{									
							double line_b = -line_k * x_curr + (y_curr - ParkingSpaceFlag * toward_space_start * 100);
							dist = abs(line_k * (double)m_x - (double)m_y + line_b);
							dist = dist / (sqrt(1 + line_k * line_k));
						}
						//std::cout << "dist = " << dist << std::endl;
						toward_dist = dist + 5; //2023.03.09 -10
						return;
					}
				}
				//obstmap[index - 1].Status = 6;
			}
		}
		
	}
	//
    void ModifyMapVertical()
    {
		double dist_p01 = -1, dist_p23 = -1, dist_p01u = -1, dist_p23u = -1;
		CalcParkSideDist(dist_p01, dist_p23,0);
		CalcParkSideDist(dist_p01u, dist_p23u,2);
		dist_P01 = (dist_p01 < dist_p01u) ? dist_p01 : dist_p01u;
		if(dist_P01 < 0)
		{
			dist_P01 = (dist_p01 > 0) ? dist_p01 : dist_p01u;
		}
		dist_P23 = (dist_p23 < dist_p23u) ? dist_p23 : dist_p23u;
		if(dist_P23 < 0)
		{
			dist_P23 = (dist_p23 > 0) ? dist_p23 : dist_p23u;
		}
		CalcParkFrontDist(dist_P03);
		CalcParkTowardDist(dist_toward_P03);
        if(fusion.ParkInMode != 1)
        {
            if (PlanBackwardFirst < 3 && PlanForwardFirst < 5) //2023.03.09
            {
				if(dist_P03 > 0 && dist_toward_P03 > 0)
				{
					dist_toward_P03 -= dist_P03;
				}
                GenerateVirtualPoint();
            }
            //ClearParkSidePoint();
            return;
        }

    }
}
