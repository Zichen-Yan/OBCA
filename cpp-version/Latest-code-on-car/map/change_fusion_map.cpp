#include "global_variable.h"
#include "global_function.h"
#include "matplotlibcpp.h"

namespace byd_apa_plan
{
	namespace plt = matplotlibcpp;

	std::vector<std::vector<int>> dis_map(250, std::vector<int>(250, 0));
	//double upa_x[10] = {3.9279, 4.1882, 4.1883, 3.9287, 0, 0, -1.1476, -1.3611, -1.3613, -1.1484};
	//double upa_y[10] = {0.93467, 0.38208, -0.38146, -0.93351, 0, 0, -0.97322, -0.29331, 0.29157, 0.9721};
	void Uss_map()
	{
		double Uss_x;
		double Uss_y;
		if ((control.ObsUssInfo == 0u) || (control.ObsUssInfo == 1u) || (control.ObsUssInfo == 2u) || (control.ObsUssInfo == 3u) || (control.ObsUssInfo == 6u) || (control.ObsUssInfo == 7u) || (control.ObsUssInfo == 8u) || (control.ObsUssInfo == 9u))
		{
			int control_ob = control.ObsUssInfo;
			Uss_x = upa_x[control_ob];
			Uss_y = upa_y[control_ob];
			double vir_x;
			double vir_y;
			vir_x = Uss_x * cos(start_to_fus[2]) - Uss_y * sin(start_to_fus[2]) + start_to_fus[0];
			vir_y = Uss_y * cos(start_to_fus[2]) + Uss_x * sin(start_to_fus[2]) + start_to_fus[1];
			/***************************************/
			double P_mean_x = (P0x + P1x + P2x + P3x) / 4;
			double P_mean_y = (P0y + P1y + P2y + P3y) / 4;
			double D_P0_mean = sqrt((P_mean_x - P0x) * (P_mean_x - P0x) + (P_mean_y - P0y) * (P_mean_y - P0y));
			double D_P1_mean = sqrt((P_mean_x - P1x) * (P_mean_x - P1x) + (P_mean_y - P1y) * (P_mean_y - P1y));
			double D_P2_mean = sqrt((P_mean_x - P2x) * (P_mean_x - P2x) + (P_mean_y - P2y) * (P_mean_y - P2y));
			double D_P3_mean = sqrt((P_mean_x - P3x) * (P_mean_x - P3x) + (P_mean_y - P3y) * (P_mean_y - P3y));
			double D_P1P0 = sqrt((P1x - P0x) * (P1x - P0x) + (P1y - P0y) * (P1y - P0y));
			double D_P2P3 = sqrt((P2x - P3x) * (P2x - P3x) + (P2y - P3y) * (P2y - P3y));
			if ((D_P0_mean < 0.1) && (D_P1_mean < 0.1) && (D_P2_mean < 0.1) && (D_P3_mean < 0.1) && (D_P1P0 < 0.1) && (D_P2P3 < 0.1))
			{
				return;
			}
			double cos_theta_P0 = (P_mean_x - P0x) / D_P0_mean;
			double sin_theta_P0 = (P_mean_y - P0y) / D_P0_mean;
			double cos_theta_P1 = (P_mean_x - P1x) / D_P1_mean;
			double sin_theta_P1 = (P_mean_y - P1y) / D_P1_mean;
			double cos_theta_P2 = (P_mean_x - P2x) / D_P2_mean;
			double sin_theta_P2 = (P_mean_y - P2y) / D_P2_mean;
			double cos_theta_P3 = (P_mean_x - P3x) / D_P3_mean;
			double sin_theta_P3 = (P_mean_y - P3y) / D_P3_mean;
			double P0x_imag = P_mean_x - (D_P0_mean * 1.2) * cos_theta_P0;
			double P0y_imag = P_mean_y - (D_P0_mean * 1.2) * sin_theta_P0;
			double P1x_imag = P_mean_x - (D_P1_mean * 1.2) * cos_theta_P1;
			double P1y_imag = P_mean_y - (D_P1_mean * 1.2) * sin_theta_P1;
			double P2x_imag = P_mean_x - (D_P2_mean * 1.2) * cos_theta_P2;
			double P2y_imag = P_mean_y - (D_P2_mean * 1.2) * sin_theta_P2;
			double P3x_imag = P_mean_x - (D_P3_mean * 1.2) * cos_theta_P3;
			double P3y_imag = P_mean_y - (D_P3_mean * 1.2) * sin_theta_P3;
			if (fusion.parkingSpaceInfo.ParkingSpaceType == 1)
			{
				P0x_imag = P0x_imag + 1 * cos(End[2]);
				P0y_imag = P0y_imag + 1 * sin(End[2]);
				P3x_imag = P3x_imag + 1 * cos(End[2]);
				P3y_imag = P3y_imag + 1 * sin(End[2]);
			}
			else if (fusion.parkingSpaceInfo.ParkingSpaceType == 2)
			{
				double cos_theta_P1P0 = (P0x - P1x) / D_P1P0;
				double sin_theta_P1P0 = (P0y - P1y) / D_P1P0;
				double cos_theta_P2P3 = (P3x - P2x) / D_P2P3;
				double sin_theta_P2P3 = (P3y - P2y) / D_P2P3;
				P0x_imag = P0x + 1 * cos_theta_P1P0;
				P0y_imag = P0y + 1 * sin_theta_P1P0;
				P3x_imag = P3x + 1 * cos_theta_P2P3;
				P3y_imag = P3y + 1 * sin_theta_P2P3;
				P1x_imag = P1x - 0.3 * cos_theta_P1P0;
				P1y_imag = P1y - 0.3 * sin_theta_P1P0;
				P2x_imag = P2x - 0.3 * cos_theta_P2P3;
				P2y_imag = P2y - 0.3 * sin_theta_P2P3;
			}
			double S_01 = (P1x_imag - P0x_imag) * (vir_y - P0y_imag) - (P1y_imag - P0y_imag) * (vir_x - P0x_imag);
			double S_12 = (P2x_imag - P1x_imag) * (vir_y - P1y_imag) - (P2y_imag - P1y_imag) * (vir_x - P1x_imag);
			double S_23 = (P3x_imag - P2x_imag) * (vir_y - P2y_imag) - (P3y_imag - P2y_imag) * (vir_x - P2x_imag);
			double S_30 = (P0x_imag - P3x_imag) * (vir_y - P3y_imag) - (P0y_imag - P3y_imag) * (vir_x - P3x_imag);
			if (((S_01 >= 0) && (S_12 >= 0) && (S_23 >= 0) && (S_30 >= 0)) || ((S_01 <= 0) && (S_12 <= 0) && (S_23 <= 0) && (S_30 <= 0)))
			{
				return;
			}
			else
			{
				int idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
				int idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
				if (idx_x == 0)
				{
					idx_x = 1;
				}
				if (idx_y == 0)
				{
					idx_y = 1;
				}
				int index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
				if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && (idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
				{
					obstmap[index_xy].Status = 4;
				}
			}
		}
	}

	void ChangeMapPoint(const std::vector<double>& vir_x, const std::vector<double>& vir_y, const int &vir_index, double *const point, const uint8 status)
	{
		double x = .0, y = .0;
		for (int i = 0; i < vir_index; i++)
		{
			x = vir_x[i] * cos(point[2]) - vir_y[i] * sin(point[2]) + point[0];
			y = vir_y[i] * cos(point[2]) + vir_x[i] * sin(point[2]) + point[1];
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
			if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && (idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
			{
				obstmap[index_xy].Status = status;
			}
		}
	}

	void ChangeMapPrkPoint(const std::vector<std::pair<double ,double>>& prk_points, double *const point, const uint8 status)
	{
		double x = .0, y = .0;
		for (unsigned int i = 0; i < prk_points.size(); i++)
		{
			x = prk_points[i].first * cos(point[2]) - prk_points[i].second * sin(point[2]) + point[0];
			y = prk_points[i].second * cos(point[2]) + prk_points[i].first * sin(point[2]) + point[1];
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
			if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && (idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
			{
				obstmap[index_xy].Status = status; 
				dis_map[(idx_x - 1)][(idx_y - 1)]=0; // 删去停车位处的障碍物
			}
		}
	}

	void delete_map()
	{
		if ((fusion.parkingSpaceInfo.ParkingSpaceType == 1) || (fusion.parkingSpaceInfo.ParkingSpaceType == 3))
		{
			if (fusion.ParkInMode == 0)
			{
				ChangeMapPrkPoint(park_vertical_rear, end_to_fus, 5u);
			}
			else
			{
				ChangeMapPrkPoint(park_vertical_head, end_to_fus, 5u);
			}
		}
		else if (fusion.parkingSpaceInfo.ParkingSpaceType == 2)
		{
			ChangeMapPrkPoint(park_level, end_to_fus, 5u);
		}
		ChangeMapPoint(Car_x, Car_y, Car_x.size(), start_to_fus, 5u);
		ChangeMapPoint(Car_x, Car_y, Car_x.size(), end_to_fus, 5u);
	}

	void park_delete_map()
	{
		std::pair<double,double>prk_point;
		// 扣车位 垂直车尾
		int map_yidx = prk_width_vertical/2*100 + 1;
		int map_xidx = prk_length_vertical/0.02 + 1;
		for (int map_y_idx = 0; map_y_idx < map_yidx; map_y_idx++)
		{
			double map_y = -prk_width_vertical/2 + map_y_idx * 0.02;
			for (int map_x_idx = 0; map_x_idx < map_xidx; map_x_idx++)
			{
				double map_x = prk_back_rear + map_x_idx * 0.02;
				prk_point.first = map_x;
				prk_point.second = map_y;
				park_vertical_rear.push_back(prk_point);
			}
		}
		// 扣车位 垂直车头
		for (int map_y_idx = 0; map_y_idx < map_yidx; map_y_idx++)
		{
			double map_y = -prk_width_vertical/2 + map_y_idx * 0.02;
			for (int map_x_idx = 0; map_x_idx < map_xidx; map_x_idx++)
			{
				double map_x = prk_back_head - map_x_idx * 0.02;
				prk_point.first = map_x;
				prk_point.second = map_y;
				park_vertical_head.push_back(prk_point);
			}
		}
		// 扣车位 水平
		map_yidx = prk_width_level/2*100 + 1;
		map_xidx = prk_length_level/0.02 + 1;
		for (int map_y_idx = 0; map_y_idx < map_yidx; map_y_idx++)
		{
			double map_y = -prk_width_level/2 + map_y_idx * 0.02;
			for (int map_x_idx = 0; map_x_idx < map_xidx; map_x_idx++)
			{
				double map_x = prk_back_level + map_x_idx * 0.02;
				prk_point.first = map_x;
				prk_point.second = map_y;
				park_level.push_back(prk_point);
			}
		}
	}

	void ChangeFusionMap()
	{
		if (fusion.parkingSpaceInfo.ParkingSpaceType == 2)
		{
			ModifyMapLevel();
		}
		ModifyMapHybirdAStar();
		delete_map();

		// plt::figure_size(1000, 1000);
        // plt::xlim(-12.5, 12.5);
        // plt::ylim(-12.5, 12.5);
        // plt::axis("equal");

		// std::vector<double> obs_x;
        // std::vector<double> obs_y;
		// for (int i = 0; i < 250; i++)
		// {
		// 	for (int j = 0; j < 250; j++)
		// 	{
		// 		if (dis_map[i][j] == 1)
		// 		{
		// 			obs_x.push_back(i);
		// 			obs_y.push_back(j);
		// 		}
		// 	}
		// }
		// plt::plot(obs_x,obs_y,"r.");
		// plt::show();

		dis_map = generateDistanceMap(dis_map);

		Uss_map();
		if (fusion.parkingSpaceInfo.ParkingSpaceType == 1 || fusion.parkingSpaceInfo.ParkingSpaceType == 3)
		{
			ModifyMapVertical();
		}
	}

	std::vector<std::vector<int>> generateDistanceMap(std::vector<std::vector<int>>& grid) 
	{
		// Define directions: up, down, left, right
		const int dx[] = {0, 0, -1, 1, 1, 1, -1, -1};
		const int dy[] = {-1, 1, 0, 0, -1, 1, 1, -1};
		int N = grid.size();
		std::vector<std::vector<int>> distanceMap(N, std::vector<int>(N, INT_MAX));
		std::queue<std::pair<int, int>> q;
		// Initialize the queue and distance map
		for (int i = 0; i < N; ++i) {
			for (int j = 0; j < N; ++j) {
				if (grid[i][j] == 1) {
					distanceMap[i][j] = 0;
					q.push({i, j});
				}
			}
		}
		// Perform BFS
		while (!q.empty()) {
			std::pair<int,int> pl = q.front();
			q.pop();

			for (int k = 0; k < 4; ++k) {
				int nx = pl.first + dx[k];
				int ny = pl.second + dy[k];
				if (nx >= 0 && nx < N && ny >= 0 && ny < N && distanceMap[nx][ny] == INT_MAX) {
					distanceMap[nx][ny] = distanceMap[pl.first][pl.second] + 1;
					q.push({nx, ny});
				}
			}
		}
		return distanceMap;
	}
}
