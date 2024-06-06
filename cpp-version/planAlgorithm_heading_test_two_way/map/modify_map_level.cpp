#include "global_variable.h"
#include "global_function.h"

namespace byd_apa_plan{
    
	void level_imagmap(double end[3], int& index_back, int& index_front, int right_left)
	{
		if (right_left == 1)
		{
			for (int map_paragraph_index = 0; map_paragraph_index < 15; map_paragraph_index++)
			{
				int index_2 = 0;
				for (int map_idx = 0; map_idx < index_front_rl; map_idx++)
				{
					double vir_x = level_front_imagmap_x_r[map_idx + index_front_rl * map_paragraph_index] * cos(end[2]) - level_front_imagmap_y_r[map_idx + index_front_rl * map_paragraph_index] * sin(end[2]) + end[0];
					double vir_y = level_front_imagmap_y_r[map_idx + index_front_rl * map_paragraph_index] * cos(end[2]) + level_front_imagmap_x_r[map_idx + index_front_rl * map_paragraph_index] * sin(end[2]) + end[1];
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
					if (obstmap[index_xy].Status == 0)
					{
						index_back = map_paragraph_index;
						break;
					}
					else if (obstmap[index_xy].Status == 2)
					{
						index_2 = index_2 + 1;
					}
					if (index_2 == index_front_rl)
					{
						index_back = map_paragraph_index;
						break;
					}
				}
				if (index_back < 14)
				{
					break;
				}
			}
		}
		else if (right_left == -1)
		{
			for (int map_paragraph_index = 0; map_paragraph_index < 15; map_paragraph_index++)
			{
				int index_2 = 0;
				for (int map_idx = 0; map_idx < index_front_rl; map_idx++)
				{
					double vir_x = level_front_imagmap_x_l[map_idx + index_front_rl * map_paragraph_index] * cos(end[2]) - level_front_imagmap_y_l[map_idx + index_front_rl * map_paragraph_index] * sin(end[2]) + end[0];
					double vir_y = level_front_imagmap_y_l[map_idx + index_front_rl * map_paragraph_index] * cos(end[2]) + level_front_imagmap_x_l[map_idx + index_front_rl * map_paragraph_index] * sin(end[2]) + end[1];
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
					if (obstmap[index_xy].Status == 0)
					{
						index_back = map_paragraph_index;
						break;
					}
					else if (obstmap[index_xy].Status == 2)
					{
						index_2 = index_2 + 1;
					}
					if (index_2 == index_front_rl)
					{
						index_back = map_paragraph_index;
						break;
					}
				}
				if (index_back < 14)
				{
					break;
				}
			}
		}
		for (int map_idx = 0; map_idx < index_front_back; map_idx++)
		{
			double vir_x = level_front_imagmap_x[map_idx + index_front_back * index_back] * cos(end[2]) - level_front_imagmap_y[map_idx + index_front_back * index_back] * sin(end[2]) + end[0];
			double vir_y = level_front_imagmap_y[map_idx + index_front_back * index_back] * cos(end[2]) + level_front_imagmap_x[map_idx + index_front_back * index_back] * sin(end[2]) + end[1];
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
				if (obstmap[index_xy].Status == 2)
				{
					obstmap[index_xy].Status = 7;
				}
			}
		}

		for (int map_paragraph_index = 0; map_paragraph_index < 15; map_paragraph_index++)
		{
			for (int map_idx = 11; map_idx < index_front_back - 11; map_idx++)
			{
				double vir_x = level_back_imagmap_x[map_idx + index_front_back * map_paragraph_index] * cos(end[2]) - level_back_imagmap_y[map_idx + index_front_back * map_paragraph_index] * sin(end[2]) + end[0];
				double vir_y = level_back_imagmap_y[map_idx + index_front_back * map_paragraph_index] * cos(end[2]) + level_back_imagmap_x[map_idx + index_front_back * map_paragraph_index] * sin(end[2]) + end[1];
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
				if (obstmap[index_xy].Status == 0)
				{
					index_front = map_paragraph_index;
					break;
				}
			}
			if (index_front < 14)
			{
				break;
			}
		}
		for (int map_idx = 11; map_idx < index_front_back - 11; map_idx++)
		{
			double vir_x = level_back_imagmap_x[map_idx + index_front_back * index_front] * cos(end[2]) - level_back_imagmap_y[map_idx + index_front_back * index_front] * sin(end[2]) + end[0];
			double vir_y = level_back_imagmap_y[map_idx + index_front_back * index_front] * cos(end[2]) + level_back_imagmap_x[map_idx + index_front_back * index_front] * sin(end[2]) + end[1];
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
				if (obstmap[index_xy].Status == 2)
				{
					obstmap[index_xy].Status = 7;
				}
			}
		}

		if (right_left == 1) // 右泊
		{

			int index_dn = 4;
			for (int map_paragraph_index = 0; map_paragraph_index < 5; map_paragraph_index++)
			{
				for (int map_idx = 0; map_idx < index_up_down; map_idx++)
				{
					double vir_x = level_down_imagmap_x[map_idx + index_up_down * map_paragraph_index] * cos(end[2]) - level_down_imagmap_y[map_idx + index_up_down * map_paragraph_index] * sin(end[2]) + end[0];
					double vir_y = level_down_imagmap_y[map_idx + index_up_down * map_paragraph_index] * cos(end[2]) + level_down_imagmap_x[map_idx + index_up_down * map_paragraph_index] * sin(end[2]) + end[1];
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
					if (obstmap[index_xy].Status == 0)
					{
						index_dn = map_paragraph_index;
						break;
					}
				}
				if (index_dn < 4)
				{
					break;
				}
			}
			for (int map_idx = 0; map_idx < index_up_down; map_idx++)
			{
				double vir_x = level_down_imagmap_x[map_idx + index_up_down * index_dn] * cos(end[2]) - level_down_imagmap_y[map_idx + index_up_down * index_dn] * sin(end[2]) + end[0];
				double vir_y = level_down_imagmap_y[map_idx + index_up_down * index_dn] * cos(end[2]) + level_down_imagmap_x[map_idx + index_up_down * index_dn] * sin(end[2]) + end[1];
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
					if (obstmap[index_xy].Status == 2)
					{
						//obstmap[index_xy].Status = 3;
					}
				}
			}
		}
		else if (right_left == -1) // 左泊
		{
			int index_u = 4;
			for (int map_paragraph_index = 0; map_paragraph_index < 5; map_paragraph_index++)
			{
				for (int map_idx = 0; map_idx < index_up_down; map_idx++)
				{
					double vir_x = level_up_imagmap_x[map_idx + index_up_down * map_paragraph_index] * cos(end[2]) - level_up_imagmap_y[map_idx + index_up_down * map_paragraph_index] * sin(end[2]) + end[0];
					double vir_y = level_up_imagmap_y[map_idx + index_up_down * map_paragraph_index] * cos(end[2]) + level_up_imagmap_x[map_idx + index_up_down * map_paragraph_index] * sin(end[2]) + end[1];
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
					if (obstmap[index_xy].Status == 0)
					{
						index_u = map_paragraph_index;
						break;
					}
				}
				if (index_u < 4)
				{
					break;
				}
			}
			for (int map_idx = 0; map_idx < index_up_down; map_idx++)
			{
				double vir_x = level_up_imagmap_x[map_idx + index_up_down * index_u] * cos(end[2]) - level_up_imagmap_y[map_idx + index_up_down * index_u] * sin(end[2]) + end[0];
				double vir_y = level_up_imagmap_y[map_idx + index_up_down * index_u] * cos(end[2]) + level_up_imagmap_x[map_idx + index_up_down * index_u] * sin(end[2]) + end[1];
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
					if (obstmap[index_xy].Status == 2)
					{
						//obstmap[index_xy].Status = 3;
					}
				}
			}
		}
	}
	//
	void imagmap_get()
	{
		index_back = 14;
		index_front = 14;
		level_imagmap(end_to_fus, index_back, index_front, right_left);
		backpath = -1 * (0.2 + (index_back) * 0.1);
		frontpath = 0.2 + (index_front) * 0.1 + 0.1 + 0.1;
	}
	//
    void level_imagmap_out(double start[3])
	{
		
		for (unsigned int map_idx = 0; map_idx < level_up2_imagmap_x.size(); map_idx++)
		{
			double vir_x = level_up2_imagmap_x[map_idx] * cos(start[2]) - level_up2_imagmap_y[map_idx] * sin(start[2]) + start[0];
			double vir_y = level_up2_imagmap_y[map_idx] * cos(start[2]) + level_up2_imagmap_x[map_idx] * sin(start[2]) + start[1];
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
				if (obstmap[index_xy].Status == 0)
				{
					obstmap[index_xy].Status = 6;
				}
			}
		}
		for (unsigned int map_idx = 0; map_idx < level_down2_imagmap_x.size(); map_idx++)
		{
			double vir_x = level_down2_imagmap_x[map_idx] * cos(start[2]) - level_down2_imagmap_y[map_idx] * sin(start[2]) + start[0];
			double vir_y = level_down2_imagmap_y[map_idx] * cos(start[2]) + level_down2_imagmap_x[map_idx] * sin(start[2]) + start[1];
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
				if (obstmap[index_xy].Status == 0)
				{
					obstmap[index_xy].Status = 6;
				}
			}
		}

		// double level_front2_imagmap_x[493] = { 4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.1,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.2,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.4,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.6,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.7,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.8,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,4.9,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.1,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.2,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.3,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.4,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.5,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.6,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7,5.7 };
		// double level_front2_imagmap_y[493] = { -1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4 };
		// int index_front2_back = 29;
		// int size_front = 16;
		// double level_back2_imagmap_x[348] = { -2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.4,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.3,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.2,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-2,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.9,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.8,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.7,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.6,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.5,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.4,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3,-1.3 };
		// double level_back2_imagmap_y[348] = { -1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,2.2204e-16,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,2.2204e-16,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,2.2204e-16,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,2.2204e-16,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,2.2204e-16,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,2.2204e-16,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,2.2204e-16,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,2.2204e-16,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,2.2204e-16,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,2.2204e-16,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,2.2204e-16,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.8,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,2.2204e-16,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4 };
		// int size_back = 11;
		for (int map_paragraph_index = 0; map_paragraph_index < 17; map_paragraph_index++)
		{
			for (int map_idx = 0 + 5; map_idx < index_front2_back - 5; map_idx++)
			{
				double vir_x = level_front2_imagmap_x[map_idx + index_front2_back * map_paragraph_index] * cos(start[2]) - level_front2_imagmap_y[map_idx + index_front2_back * map_paragraph_index] * sin(start[2]) + start[0];
				double vir_y = level_front2_imagmap_y[map_idx + index_front2_back * map_paragraph_index] * cos(start[2]) + level_front2_imagmap_x[map_idx + index_front2_back * map_paragraph_index] * sin(start[2]) + start[1];
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
				if (obstmap[index_xy].Status == 0)
				{
					size_front = map_paragraph_index;
					break;
				}
			}
			if (size_front < 16)
			{
				break;
			}
		}
		for (int map_paragraph_index = 11; map_paragraph_index >= 0; map_paragraph_index--)
		{
			for (int map_idx = 0 + 5; map_idx < index_front2_back - 5; map_idx++)
			{
				double vir_x = level_back2_imagmap_x[map_idx + index_front2_back * map_paragraph_index] * cos(start[2]) - level_back2_imagmap_y[map_idx + index_front2_back * map_paragraph_index] * sin(start[2]) + start[0];
				double vir_y = level_back2_imagmap_y[map_idx + index_front2_back * map_paragraph_index] * cos(start[2]) + level_back2_imagmap_x[map_idx + index_front2_back * map_paragraph_index] * sin(start[2]) + start[1];
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
				if (obstmap[index_xy].Status == 0)
				{
					size_back = map_paragraph_index;
					break;
				}
			}
			if (size_back < 11)
			{
				break;
			}
		}
		if (index_request == 1)
		{
			dist_P01 = level_front2_imagmap_x[0] + (size_front - 1) * 0.1;
			dist_P23 = level_back2_imagmap_x[0] + (size_back + 1) * 0.1;
		}
		for (int map_idx = 0 + 4; map_idx < index_front2_back - 4; map_idx++)
		{
			double vir_x = level_front2_imagmap_x[map_idx + index_front2_back * size_front] * cos(start[2]) - level_front2_imagmap_y[map_idx + index_front2_back * size_front] * sin(start[2]) + start[0];
			double vir_y = level_front2_imagmap_y[map_idx + index_front2_back * size_front] * cos(start[2]) + level_front2_imagmap_x[map_idx + index_front2_back * size_front] * sin(start[2]) + start[1];
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
				obstmap[index_xy].Status = 3; // 13->3
			}
			end_to_fus[0] = vir_x + 2.;
		}

		int max_out_index = (dist_P01 - dist_P23 + 0.11) / 0.1;
		double map_x = 0;
		double map_y = 0;
		for (int map_x_idx = 0; map_x_idx < max_out_index; map_x_idx++)
		{
			map_x = dist_P23 + map_x_idx * 0.1;
			for (int map_y_idx = 0; map_y_idx < index_front2_back; map_y_idx++)
			{
				map_y = level_front2_imagmap_y[0] + map_y_idx * 0.1;
				double vir_x = map_x * cos(start[2]) - map_y * sin(start[2]) + start[0];
				double vir_y = map_y * cos(start[2]) + map_x * sin(start[2]) + start[1];
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
					if (obstmap[index_xy].Status == 0)
					{
						obstmap[index_xy].Status = 6;
					}
				}
			}
		}
	}
	//

    void ModifyMapLevel()
    {
        if (app.APA_Park_Function == 1)
		{
			imagmap_get();
		}
		else if (app.APA_Park_Function == 2)
		{
			level_imagmap_out(Start_out);
		}
    }
}