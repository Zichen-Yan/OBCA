#include "global_variable.h"
#include "global_function.h"

namespace byd_apa_plan{
 
	int hybird_astar_imagmap_get()
	{
		if (((fusion.position.Heading >= 0) && (fusion.position.Heading <= 360)) && ((calculation.nav_heading >= 0) && (calculation.nav_heading <= 360)))
		{
			// 垂直车位
			if (fusion.parkingSpaceInfo.ParkingSpaceType == 1)
			{
				if (app.APA_Park_Function == 1)
				{
					// 车尾泊入
					if (fusion.ParkInMode != 1) // 停车线
					{
						double vir_x;
						double vir_y;
						for (unsigned int map_index = 0; map_index < map_image_x.size(); map_index++)
						{
							vir_x = map_image_x[map_index] * cos(end_to_fus[2]) - ParkingSpaceFlag * map_image_y[map_index] * sin(end_to_fus[2]) + end_to_fus[0];
							vir_y = ParkingSpaceFlag * map_image_y[map_index] * cos(end_to_fus[2]) + map_image_x[map_index] * sin(end_to_fus[2]) + end_to_fus[1];
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
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
								(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								if (obstmap[index_xy].Status == 2)
								{
									obstmap[index_xy].Status = 3;
									// dis_map[(idx_x - 1)][(idx_y - 1)]=1; // 虚拟停车线算入距离地图
								}
							}
						}
						////////////////起始位置如果有虚拟障碍物就不要/////////////////////
						for (unsigned int map_index = 0; map_index < Car_x.size(); map_index++)
						{
							vir_x = Car_x[map_index] * cos(start_to_fus[2]) - Car_y[map_index] * sin(start_to_fus[2]) + start_to_fus[0];
							vir_y = Car_y[map_index] * cos(start_to_fus[2]) + Car_x[map_index] * sin(start_to_fus[2]) + start_to_fus[1];
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
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								obstmap[index_xy].Status = 5;
							}
						}
						/////////////////////////////////////////////////////////////////////////
						for (unsigned int map_vertical_index = 0; map_vertical_index < map_image_vertical_x.size(); map_vertical_index++)
						{
							vir_x = map_image_vertical_x[map_vertical_index] * cos(end_to_fus[2]) - map_image_vertical_y[map_vertical_index] * sin(end_to_fus[2]) + end_to_fus[0];
							vir_y = map_image_vertical_y[map_vertical_index] * cos(end_to_fus[2]) + map_image_vertical_x[map_vertical_index] * sin(end_to_fus[2]) + end_to_fus[1];
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
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								if (obstmap[index_xy].Status == 5)
								{
									no_imag_map = 1;
									double SE_Usspark = (start_to_fus[0] - end_to_fus[0]) * (start_to_fus[0] - end_to_fus[0]) + (start_to_fus[1] - end_to_fus[1]) * (start_to_fus[1] - end_to_fus[1]);
									if (SE_Usspark < 2.25)
									{
										for (int Uss_start_index = 0; Uss_start_index < 2; Uss_start_index++)
										{
											int control_ob = 7 + Uss_start_index;
											double Uss_x = upa_x[control_ob];
											double Uss_y = upa_y[control_ob];
											double vir_x_;
											double vir_y_;
											vir_x_ = Uss_x * cos(start_to_fus[2]) - Uss_y * sin(start_to_fus[2]) + start_to_fus[0];
											vir_y_ = Uss_y * cos(start_to_fus[2]) + Uss_x * sin(start_to_fus[2]) + start_to_fus[1];
											int idx_x_ = ceil((pathfind_parameters.MAXX - vir_x_) / pathfind_parameters.MOTION_RESOLUTION);
											int idx_y_ = ceil((pathfind_parameters.MAXY - vir_y_) / pathfind_parameters.MOTION_RESOLUTION);
											if (idx_x_ == 0)
											{
												idx_x_ = 1;
											}
											if (idx_y_ == 0)
											{
												idx_y_ = 1;
											}
											int index_xy_ = (idx_x_ - 1) * pathfind_parameters.XIDX + (idx_y_ - 1);
											if ((index_xy_ >= 0) && (index_xy_ < pathfind_parameters.MAX_IDX) && 
											(idx_x_ >= 0) && (idx_x_ <= pathfind_parameters.XIDX) && (idx_y_ >= 0) && (idx_y_ <= pathfind_parameters.XIDY))
											{
												obstmap[index_xy_].Status = 4;
												int i = idx_x - 1;
												int j = idx_y - 1;
												int dir1 = i * pathfind_parameters.XIDX + j + 1;
												int dir2 = i * pathfind_parameters.XIDX + j - 1;
												int dir3 = (i + 1) * pathfind_parameters.XIDX + j;
												int dir4 = (i - 1) * pathfind_parameters.XIDX + j;
												int dir5 = (i + 1) * pathfind_parameters.XIDX + j + 1;
												int dir6 = (i + 1) * pathfind_parameters.XIDX + j - 1;
												int dir7 = (i - 1) * pathfind_parameters.XIDX + j + 1;
												int dir8 = (i - 1) * pathfind_parameters.XIDX + j - 1;

												if (dir1 >= 0 && dir1 < pathfind_parameters.MAX_IDX && 
												dir2 >= 0 && dir2 < pathfind_parameters.MAX_IDX && 
												dir3 >= 0 && dir3 < pathfind_parameters.MAX_IDX && 
												dir4 >= 0 && dir4 < pathfind_parameters.MAX_IDX && 
												dir5 >= 0 && dir5 < pathfind_parameters.MAX_IDX && 
												dir6 >= 0 && dir6 < pathfind_parameters.MAX_IDX && 
												dir7 >= 0 && dir7 < pathfind_parameters.MAX_IDX && 
												dir8 >= 0 && dir8 < pathfind_parameters.MAX_IDX)
												{
													if (obstmap[dir1].Status != 0 && obstmap[dir1].Status != 3 && obstmap[dir1].Status != 5)
													{
														obstmap[dir1].Status = 9u;
													}
													if (obstmap[dir2].Status != 0 && obstmap[dir2].Status != 3 && obstmap[dir2].Status != 5)
													{
														obstmap[dir2].Status = 9u;
													}
													if (obstmap[dir3].Status != 0 && obstmap[dir3].Status != 3 && obstmap[dir3].Status != 5)
													{
														obstmap[dir3].Status = 9u;
													}
													if (obstmap[dir4].Status != 0 && obstmap[dir4].Status != 3 && obstmap[dir4].Status != 5)
													{
														obstmap[dir4].Status = 9u;
													}
													if (obstmap[dir5].Status != 0 && obstmap[dir5].Status != 3 && obstmap[dir5].Status != 5)
													{
														obstmap[dir5].Status = 9u;
													}
													if (obstmap[dir6].Status != 0 && obstmap[dir6].Status != 3 && obstmap[dir6].Status != 5)
													{
														obstmap[dir6].Status = 9u;
													}
													if (obstmap[dir7].Status != 0 && obstmap[dir7].Status != 3 && obstmap[dir7].Status != 5)
													{
														obstmap[dir7].Status = 9u;
													}
													if (obstmap[dir8].Status != 0 && obstmap[dir8].Status != 3 && obstmap[dir8].Status != 5)
													{
														obstmap[dir8].Status = 9u;
													}
												}
											}
										}
									}
								}
								else if (obstmap[index_xy].Status != 0)
								{
									obstmap[index_xy].Status = 3;
								}
							}
						}
					}
					// 车头泊入
					else
					{
						double vir_x;
						double vir_y;
						for (unsigned int map_index = 0; map_index < map_image_x.size(); map_index++)
						{
							vir_x = (-1 * map_image_x[map_index] + vehicle_parameters.LF - vehicle_parameters.LB) * cos(end_to_fus[2]) - (-ParkingSpaceFlag) * map_image_y[map_index] * sin(end_to_fus[2]) + end_to_fus[0];
							vir_y = -ParkingSpaceFlag * map_image_y[map_index] * cos(end_to_fus[2]) + (-1 * map_image_x[map_index] + vehicle_parameters.LF - vehicle_parameters.LB) * sin(end_to_fus[2]) + end_to_fus[1];
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
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								if (obstmap[index_xy].Status == 2)
								{
									obstmap[index_xy].Status = 3;
								}
							}
						}
						////////////////起始位置如果有虚拟障碍物就不要/////////////////////
						for (unsigned int map_index = 0; map_index < Car_x.size(); map_index++)
						{
							vir_x = Car_x[map_index] * cos(start_to_fus[2]) - Car_y[map_index] * sin(start_to_fus[2]) + start_to_fus[0];
							vir_y = Car_y[map_index] * cos(start_to_fus[2]) + Car_x[map_index] * sin(start_to_fus[2]) + start_to_fus[1];
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
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								obstmap[index_xy].Status = 5;
							}
						}
						/////////////////////////////////////////////////////////////////////////
						for (unsigned int map_vertical_index = 0; map_vertical_index < map_image_vertical_x.size(); map_vertical_index++)
						{
							vir_x = (-1 * map_image_vertical_x[map_vertical_index] + vehicle_parameters.LF - vehicle_parameters.LB) * cos(end_to_fus[2]) - map_image_vertical_y[map_vertical_index] * sin(end_to_fus[2]) + end_to_fus[0];
							vir_y = map_image_vertical_y[map_vertical_index] * cos(end_to_fus[2]) + (-1 * map_image_vertical_x[map_vertical_index] + vehicle_parameters.LF - vehicle_parameters.LB) * sin(end_to_fus[2]) + end_to_fus[1];
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
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								if (obstmap[index_xy].Status == 5)
								{
									no_imag_map = 1;
									double SE_Usspark = (start_to_fus[0] - end_to_fus[0]) * (start_to_fus[0] - end_to_fus[0]) + (start_to_fus[1] - end_to_fus[1]) * (start_to_fus[1] - end_to_fus[1]);
									if (SE_Usspark < 2.25)
									{
										for (int Uss_start_index = 0; Uss_start_index < 2; Uss_start_index++)
										{
											int control_ob = 1 + Uss_start_index;
											double Uss_x = upa_x[control_ob];
											double Uss_y = upa_y[control_ob];
											double vir_x_;
											double vir_y_;
											vir_x_ = Uss_x * cos(start_to_fus[2]) - Uss_y * sin(start_to_fus[2]) + start_to_fus[0];
											vir_y_ = Uss_y * cos(start_to_fus[2]) + Uss_x * sin(start_to_fus[2]) + start_to_fus[1];
											int idx_x_ = ceil((pathfind_parameters.MAXX - vir_x_) / pathfind_parameters.MOTION_RESOLUTION);
											int idx_y_ = ceil((pathfind_parameters.MAXY - vir_y_) / pathfind_parameters.MOTION_RESOLUTION);
											if (idx_x_ == 0)
											{
												idx_x_ = 1;
											}
											if (idx_y_ == 0)
											{
												idx_y_ = 1;
											}
											int index_xy_ = (idx_x_ - 1) * pathfind_parameters.XIDX + (idx_y_ - 1);
											if ((index_xy_ >= 0) && (index_xy_ < pathfind_parameters.MAX_IDX) && 
											(idx_x_ >= 0) && (idx_x_ <= pathfind_parameters.XIDX) && (idx_y_ >= 0) && (idx_y_ <= pathfind_parameters.XIDY))
											{
												obstmap[index_xy_].Status = 4;
												int i = idx_x - 1;
												int j = idx_y - 1;

												int dir1 = i * pathfind_parameters.XIDX + j + 1;
												int dir2 = i * pathfind_parameters.XIDX + j - 1;
												int dir3 = (i + 1) * pathfind_parameters.XIDX + j;
												int dir4 = (i - 1) * pathfind_parameters.XIDX + j;
												int dir5 = (i + 1) * pathfind_parameters.XIDX + j + 1;
												int dir6 = (i + 1) * pathfind_parameters.XIDX + j - 1;
												int dir7 = (i - 1) * pathfind_parameters.XIDX + j + 1;
												int dir8 = (i - 1) * pathfind_parameters.XIDX + j - 1;

												if (dir1 >= 0 && dir1 < pathfind_parameters.MAX_IDX && 
												dir2 >= 0 && dir2 < pathfind_parameters.MAX_IDX && 
												dir3 >= 0 && dir3 < pathfind_parameters.MAX_IDX && 
												dir4 >= 0 && dir4 < pathfind_parameters.MAX_IDX && 
												dir5 >= 0 && dir5 < pathfind_parameters.MAX_IDX && 
												dir6 >= 0 && dir6 < pathfind_parameters.MAX_IDX && 
												dir7 >= 0 && dir7 < pathfind_parameters.MAX_IDX && 
												dir8 >= 0 && dir8 < pathfind_parameters.MAX_IDX)
												{
													if (obstmap[dir1].Status != 0 && obstmap[dir1].Status != 3 && obstmap[dir1].Status != 5)
													{
														obstmap[dir1].Status = 9u;
													}
													if (obstmap[dir2].Status != 0 && obstmap[dir2].Status != 3 && obstmap[dir2].Status != 5)
													{
														obstmap[dir2].Status = 9u;
													}
													if (obstmap[dir3].Status != 0 && obstmap[dir3].Status != 3 && obstmap[dir3].Status != 5)
													{
														obstmap[dir3].Status = 9u;
													}
													if (obstmap[dir4].Status != 0 && obstmap[dir4].Status != 3 && obstmap[dir4].Status != 5)
													{
														obstmap[dir4].Status = 9u;
													}
													if (obstmap[dir5].Status != 0 && obstmap[dir5].Status != 3 && obstmap[dir5].Status != 5)
													{
														obstmap[dir5].Status = 9u;
													}
													if (obstmap[dir6].Status != 0 && obstmap[dir6].Status != 3 && obstmap[dir6].Status != 5)
													{
														obstmap[dir6].Status = 9u;
													}
													if (obstmap[dir7].Status != 0 && obstmap[dir7].Status != 3 && obstmap[dir7].Status != 5)
													{
														obstmap[dir7].Status = 9u;
													}
													if (obstmap[dir8].Status != 0 && obstmap[dir8].Status != 3 && obstmap[dir8].Status != 5)
													{
														obstmap[dir8].Status = 9u;
													}
												}
											}
										}
									}
								}
								else if (obstmap[index_xy].Status != 0)
								{
									obstmap[index_xy].Status = 3;
								}
								//obstmap[index_xy].Status = 3;
							}
						}
					}
				}
			}
			// 水平车位
			else if (fusion.parkingSpaceInfo.ParkingSpaceType == 2)
			{
				if ((app.APA_Park_Function == 1) || (app.APA_Park_Function == 3))
				{
					double P_mean_x = (P0x + P1x + P2x + P3x) / 4;
					double P_mean_y = (P0y + P1y + P2y + P3y) / 4;

					double D_P1_mean = sqrt((P_mean_x - P1x) * (P_mean_x - P1x) + (P_mean_y - P1y) * (P_mean_y - P1y));
					double D_P2_mean = sqrt((P_mean_x - P2x) * (P_mean_x - P2x) + (P_mean_y - P2y) * (P_mean_y - P2y));
					if ((D_P1_mean < 0.1) && (D_P2_mean < 0.1))
					{
						return 1;
					}
					double cos_theta_P1 = (P_mean_x - P1x) / D_P1_mean;
					double sin_theta_P1 = (P_mean_y - P1y) / D_P1_mean;
					double cos_theta_P2 = (P_mean_x - P2x) / D_P2_mean;
					double sin_theta_P2 = (P_mean_y - P2y) / D_P2_mean;
					double P1x_imag = P_mean_x - (D_P1_mean + 1) * cos_theta_P1;
					double P1y_imag = P_mean_y - (D_P1_mean + 1) * sin_theta_P1;
					double P2x_imag = P_mean_x - (D_P2_mean + 1) * cos_theta_P2;
					double P2y_imag = P_mean_y - (D_P2_mean + 1) * sin_theta_P2;
					double D_P12_imag = sqrt((P1x_imag - P2x_imag) * (P1x_imag - P2x_imag) + (P1y_imag - P2y_imag) * (P1y_imag - P2y_imag));
					if (D_P12_imag < 0.1)
					{
						return 1;
					}
					// double cos_theta_P12imag = (P2x_imag - P1x_imag) / D_P12_imag;
					// double sin_theta_P12imag = (P2y_imag - P1y_imag) / D_P12_imag;
					int index_imag_P12 = ceil(D_P12_imag / 0.025);

					int idx_x;
					int idx_y;
					int index_xy;
					double vir_x;
					double vir_y;
					////////////////起始位置如果有虚拟障碍物就不要/////////////////////
					for (unsigned int map_index = 0; map_index < Car_x.size(); map_index++)
					{
						vir_x = Car_x[map_index] * cos(start_to_fus[2]) - Car_y[map_index] * sin(start_to_fus[2]) + start_to_fus[0];
						vir_y = Car_y[map_index] * cos(start_to_fus[2]) + Car_x[map_index] * sin(start_to_fus[2]) + start_to_fus[1];
						idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
						idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
						if (idx_x == 0)
						{
							idx_x = 1;
						}
						if (idx_y == 0)
						{
							idx_y = 1;
						}
						int index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
						if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
						(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
						{
							obstmap[index_xy].Status = 5;
						}
					}
					/////////////////////////////////////////////////////////////////////
					idx_x = ceil((pathfind_parameters.MAXX - P1x_imag) / pathfind_parameters.MOTION_RESOLUTION);
					idx_y = ceil((pathfind_parameters.MAXY - P1y_imag) / pathfind_parameters.MOTION_RESOLUTION);
					if (idx_x == 0)
					{
						idx_x = 1;
					}
					if (idx_y == 0)
					{
						idx_y = 1;
					}
					index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
					if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
					(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
					{
						if (obstmap[index_xy].Status == 5)
						{
							no_imag_map = 1;
						}
						else if (obstmap[index_xy].Status != 0)
						{
							//obstmap[index_xy].Status = 3;
						}
						//obstmap[index_xy].Status = 3;
					}

					idx_x = ceil((pathfind_parameters.MAXX - P2x_imag) / pathfind_parameters.MOTION_RESOLUTION);
					idx_y = ceil((pathfind_parameters.MAXY - P2y_imag) / pathfind_parameters.MOTION_RESOLUTION);
					if (idx_x == 0)
					{
						idx_x = 1;
					}
					if (idx_y == 0)
					{
						idx_y = 1;
					}
					index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
					if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
					(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
					{
						if (obstmap[index_xy].Status == 5)
						{
							no_imag_map = 1;
						}
						else if (obstmap[index_xy].Status != 0)
						{
							//obstmap[index_xy].Status = 3;
						}
						//obstmap[index_xy].Status = 3;
					}

					if (index_imag_P12 >= 1)
					{
						double P12x_imag[65] = { 0 };
						double P12y_imag[65] = { 0 };
						double IDX = 65;
						if (end_to_fus[1] > 0)
						{
							for (int idx_p12 = 0; idx_p12 < 65; idx_p12++)
							{
								P12x_imag[idx_p12] = -1.8 + idx_p12 * 0.1;
								P12y_imag[idx_p12] = 1.6;
							}
						}
						else if (end_to_fus[1] < 0)
						{
							for (int idx_p12 = 0; idx_p12 < 65; idx_p12++)
							{
								P12x_imag[idx_p12] = -1.8 + idx_p12 * 0.1;
								P12y_imag[idx_p12] = -1.6;
							}
						}
						else
						{
							IDX = 0;
						}
						for (int idx = 0; idx < IDX; idx++)
						{
							vir_x = P12x_imag[idx] * cos(end_to_fus[2]) - P12y_imag[idx] * sin(end_to_fus[2]) + end_to_fus[0];
							vir_y = P12y_imag[idx] * cos(end_to_fus[2]) + P12x_imag[idx] * sin(end_to_fus[2]) + end_to_fus[1];
							idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
							idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
							if (idx_x == 0)
							{
								idx_x = 1;
							}
							if (idx_y == 0)
							{
								idx_y = 1;
							}
							index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								if (obstmap[index_xy].Status == 5)
								{
									no_imag_map = 1;
								}
								else if (obstmap[index_xy].Status != 0)
								{
									obstmap[index_xy].Status = 3;
								}
							}
						}
					}

					double D_P01_imag = sqrt((P1x - P0x) * (P1x - P0x) + (P1y - P0y) * (P1y - P0y));
					if (D_P01_imag < 0.1)
					{
						return 1;
					}
					// double cos_theta_P01imag = (P1x - P0x) / D_P01_imag;
					// double sin_theta_P01imag = (P1y - P0y) / D_P01_imag;
					// int index_imag_P01 = ceil(D_P01_imag / 0.3);
					double level_x_imagend[9] = { -2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1,-2.1 };
					double level_y_imagend[9] = { -1,-0.75,-0.5,-0.25,0,0.25,0.5,0.75,1 };
					int imagend_idx = 9;

					for (int map_index = 0; map_index < imagend_idx; map_index++)
					{
						vir_x = level_x_imagend[map_index] * cos(end_to_fus[2]) - level_y_imagend[map_index] * sin(end_to_fus[2]) + end_to_fus[0];
						vir_y = level_y_imagend[map_index] * cos(end_to_fus[2]) + level_x_imagend[map_index] * sin(end_to_fus[2]) + end_to_fus[1];
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
						if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
						(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
						{
							if (obstmap[index_xy].Status == 2)
							{
								obstmap[index_xy].Status = 3;
							}
						}
					}

					double D_P23_imag = sqrt((P2x - P3x) * (P2x - P3x) + (P2y - P3y) * (P2y - P3y));
					if (D_P23_imag < 0.1)
					{
						return 1;
					}
					double cos_theta_P23imag = (P3x - P2x) / D_P23_imag;
					double sin_theta_P23imag = (P3y - P2y) / D_P23_imag;
					int index_imag_P23 = ceil(D_P23_imag / 0.3);
					if (index_imag_P23 >= 1)
					{
						for (int idx = 1; idx < index_imag_P23; idx++)
						{
							vir_x = P2x + idx * 0.3 * cos_theta_P23imag;
							vir_y = P2y + idx * 0.3 * sin_theta_P23imag;
							idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
							idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
							if (idx_x == 0)
							{
								idx_x = 1;
							}
							if (idx_y == 0)
							{
								idx_y = 1;
							}
							index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								if (obstmap[index_xy].Status == 2)
								{
									obstmap[index_xy].Status = 3;
								}
							}
						}
					}
				}
			}
			// 斜车位
			else if (fusion.parkingSpaceInfo.ParkingSpaceType == 3)
			{
				if (app.APA_Park_Function == 1)
				{
					if (fusion.ParkInMode != 1)
					{
						double P_mean_x = (P0x + P1x + P2x + P3x) / 4;
						double P_mean_y = (P0y + P1y + P2y + P3y) / 4;

						int idx_x;
						int idx_y;
						int index_xy;
						double vir_x;
						double vir_y;

						double D_P03 = sqrt((P0x - P3x) * (P0x - P3x) + (P0y - P3y) * (P0y - P3y));
						double D_P12 = sqrt((P1x - P2x) * (P1x - P2x) + (P1y - P2y) * (P1y - P2y));
						double Diff_xP03 = P3x - P0x;
						double Diff_yP03 = P3y - P0y;
						double Diff_xP12 = P2x - P1x;
						double Diff_yP12 = P2y - P1y;
						if ((D_P03 < 0.1) || (D_P12 < 0.1))
						{
							return 1;
						}
						double cos_P03 = Diff_xP03 / D_P03;
						double sin_P03 = Diff_yP03 / D_P03;
						double P0x_imag = P0x - 0.5 * cos_P03;
						double P0y_imag = P0y - 0.5 * sin_P03;
						double P3x_imag = P3x + 0.5 * cos_P03;
						double P3y_imag = P3y + 0.5 * sin_P03;

						double D_P1_mean = sqrt((P_mean_x - P1x) * (P_mean_x - P1x) + (P_mean_y - P1y) * (P_mean_y - P1y));
						double D_P2_mean = sqrt((P_mean_x - P2x) * (P_mean_x - P2x) + (P_mean_y - P2y) * (P_mean_y - P2y));
						if ((D_P1_mean < 0.1) && (D_P2_mean < 0.1))
						{
							return 1;
						}
						double cos_theta_P1 = (P_mean_x - P1x) / D_P1_mean;
						double sin_theta_P1 = (P_mean_y - P1y) / D_P1_mean;
						double cos_theta_P2 = (P_mean_x - P2x) / D_P2_mean;
						double sin_theta_P2 = (P_mean_y - P2y) / D_P2_mean;
						double P1x_imag = P_mean_x - (D_P1_mean + 2) * cos_theta_P1;
						double P1y_imag = P_mean_y - (D_P1_mean + 2) * sin_theta_P1;
						double P2x_imag = P_mean_x - (D_P2_mean + 2) * cos_theta_P2;
						double P2y_imag = P_mean_y - (D_P2_mean + 2) * sin_theta_P2;

						double cos_P12 = Diff_xP12 / D_P12;
						double sin_P12 = Diff_yP12 / D_P12;
						//double P1x_imag = P1x - 0.5 * cos_P12;
						//double P1x_imag = P1y - 0.5 * sin_P12;
						//double P2x_imag = P2x + 0.5 * cos_P12;
						//double P2x_imag = P2y + 0.5 * sin_P12;
						////////////////起始位置如果有虚拟障碍物就不要/////////////////////
						for (unsigned int map_index = 0; map_index < Car_x.size(); map_index++)
						{
							vir_x = Car_x[map_index] * cos(start_to_fus[2]) - Car_y[map_index] * sin(start_to_fus[2]) + start_to_fus[0];
							vir_y = Car_y[map_index] * cos(start_to_fus[2]) + Car_x[map_index] * sin(start_to_fus[2]) + start_to_fus[1];
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
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								obstmap[index_xy].Status = 5;
							}
						}
						/////////////////////////////////////////////////////////////////////////
						for (unsigned int map_vertical_index = 0; map_vertical_index < map_image_vertical_x.size(); map_vertical_index++)
						{
							vir_x = map_image_vertical_x[map_vertical_index] * cos(end_to_fus[2]) - map_image_vertical_y[map_vertical_index] * sin(end_to_fus[2]) + end_to_fus[0];
							vir_y = map_image_vertical_y[map_vertical_index] * cos(end_to_fus[2]) + map_image_vertical_x[map_vertical_index] * sin(end_to_fus[2]) + end_to_fus[1];
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
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								if (obstmap[index_xy].Status == 5)
								{
									no_imag_map = 1;
									double SE_Usspark = (start_to_fus[0] - end_to_fus[0]) * (start_to_fus[0] - end_to_fus[0]) + (start_to_fus[1] - end_to_fus[1]) * (start_to_fus[1] - end_to_fus[1]);
									if (SE_Usspark < 2.25)
									{
										for (int Uss_start_index = 0; Uss_start_index < 2; Uss_start_index++)
										{
											int control_ob = 7 + Uss_start_index;
											double Uss_x = upa_x[control_ob];
											double Uss_y = upa_y[control_ob];
											double vir_x_;
											double vir_y_;
											vir_x_ = Uss_x * cos(start_to_fus[2]) - Uss_y * sin(start_to_fus[2]) + start_to_fus[0];
											vir_y_ = Uss_y * cos(start_to_fus[2]) + Uss_x * sin(start_to_fus[2]) + start_to_fus[1];
											int idx_x_ = ceil((pathfind_parameters.MAXX - vir_x_) / pathfind_parameters.MOTION_RESOLUTION);
											int idx_y_ = ceil((pathfind_parameters.MAXY - vir_y_) / pathfind_parameters.MOTION_RESOLUTION);
											if (idx_x_ == 0)
											{
												idx_x_ = 1;
											}
											if (idx_y_ == 0)
											{
												idx_y_ = 1;
											}
											int index_xy_ = (idx_x_ - 1) * pathfind_parameters.XIDX + (idx_y_ - 1);
											if ((index_xy_ >= 0) && (index_xy_ < pathfind_parameters.MAX_IDX) && 
											(idx_x_ >= 0) && (idx_x_ <= pathfind_parameters.XIDX) && (idx_y_ >= 0) && (idx_y_ <= pathfind_parameters.XIDY))
											{
												obstmap[index_xy_].Status = 4;
												int i = idx_x - 1;
												int j = idx_y - 1;
												int dir1 = i * pathfind_parameters.XIDX + j + 1;
												int dir2 = i * pathfind_parameters.XIDX + j - 1;
												int dir3 = (i + 1) * pathfind_parameters.XIDX + j;
												int dir4 = (i - 1) * pathfind_parameters.XIDX + j;
												int dir5 = (i + 1) * pathfind_parameters.XIDX + j + 1;
												int dir6 = (i + 1) * pathfind_parameters.XIDX + j - 1;
												int dir7 = (i - 1) * pathfind_parameters.XIDX + j + 1;
												int dir8 = (i - 1) * pathfind_parameters.XIDX + j - 1;

												if (dir1 >= 0 && dir1 < pathfind_parameters.MAX_IDX && 
												dir2 >= 0 && dir2 < pathfind_parameters.MAX_IDX && 
												dir3 >= 0 && dir3 < pathfind_parameters.MAX_IDX && 
												dir4 >= 0 && dir4 < pathfind_parameters.MAX_IDX && 
												dir5 >= 0 && dir5 < pathfind_parameters.MAX_IDX && 
												dir6 >= 0 && dir6 < pathfind_parameters.MAX_IDX && 
												dir7 >= 0 && dir7 < pathfind_parameters.MAX_IDX && 
												dir8 >= 0 && dir8 < pathfind_parameters.MAX_IDX)
												{
													if (obstmap[dir1].Status != 0 && obstmap[dir1].Status != 3 && obstmap[dir1].Status != 5)
													{
														obstmap[dir1].Status = 9u;
													}
													if (obstmap[dir2].Status != 0 && obstmap[dir2].Status != 3 && obstmap[dir2].Status != 5)
													{
														obstmap[dir2].Status = 9u;
													}
													if (obstmap[dir3].Status != 0 && obstmap[dir3].Status != 3 && obstmap[dir3].Status != 5)
													{
														obstmap[dir3].Status = 9u;
													}
													if (obstmap[dir4].Status != 0 && obstmap[dir4].Status != 3 && obstmap[dir4].Status != 5)
													{
														obstmap[dir4].Status = 9u;
													}
													if (obstmap[dir5].Status != 0 && obstmap[dir5].Status != 3 && obstmap[dir5].Status != 5)
													{
														obstmap[dir5].Status = 9u;
													}
													if (obstmap[dir6].Status != 0 && obstmap[dir6].Status != 3 && obstmap[dir6].Status != 5)
													{
														obstmap[dir6].Status = 9u;
													}
													if (obstmap[dir7].Status != 0 && obstmap[dir7].Status != 3 && obstmap[dir7].Status != 5)
													{
														obstmap[dir7].Status = 9u;
													}
													if (obstmap[dir8].Status != 0 && obstmap[dir8].Status != 3 && obstmap[dir8].Status != 5)
													{
														obstmap[dir8].Status = 9u;
													}
												}
											}
										}
									}
								}
								else if (obstmap[index_xy].Status != 0)
								{
									obstmap[index_xy].Status = 3;
								}
								//obstmap[index_xy].Status = 3;
							}
						}

						for (int idx = 1; idx < 50; idx++)
						{
							vir_x = P3x_imag + idx * 0.3 * cos_P03;
							vir_y = P3y_imag + idx * 0.3 * sin_P03;
							idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
							idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
							if (idx_x == 0)
							{
								idx_x = 1;
							}
							if (idx_y == 0)
							{
								idx_y = 1;
							}
							index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								if (obstmap[index_xy].Status == 2)
								{
									obstmap[index_xy].Status = 3;
								}
							}
						}
						for (int idx = 1; idx < 50; idx++)
						{
							vir_x = P0x_imag - idx * 0.3 * cos_P03;
							vir_y = P0y_imag - idx * 0.3 * sin_P03;
							idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
							idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
							if (idx_x == 0)
							{
								idx_x = 1;
							}
							if (idx_y == 0)
							{
								idx_y = 1;
							}
							index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								if (obstmap[index_xy].Status == 2)
								{
									obstmap[index_xy].Status = 3;
								}
							}
						}

						double D_imag_12 = sqrt((P1x_imag - P2x_imag) * (P1x_imag - P2x_imag) + (P1y_imag - P2y_imag) * (P1y_imag - P2y_imag));
						int idx_imag_12 = ceil(D_imag_12 / 0.025);
						if (idx_imag_12 > 0)
						{
							for (int idx = 0; idx < idx_imag_12; idx++)
							{
								vir_x = P1x_imag + idx * 0.025 * cos_P12;
								vir_y = P1y_imag + idx * 0.025 * sin_P12;
								idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
								idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
								if (idx_x == 0)
								{
									idx_x = 1;
								}
								if (idx_y == 0)
								{
									idx_y = 1;
								}
								index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
								if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
								(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
								{
									if (obstmap[index_xy].Status == 5)
									{
										no_imag_map = 1;
										double SE_Usspark = (start_to_fus[0] - end_to_fus[0]) * (start_to_fus[0] - end_to_fus[0]) + (start_to_fus[1] - end_to_fus[1]) * (start_to_fus[1] - end_to_fus[1]);
										if (SE_Usspark < 2.25)
										{
											for (int Uss_start_index = 0; Uss_start_index < 2; Uss_start_index++)
											{
												int control_ob = 7 + Uss_start_index;
												double Uss_x = upa_x[control_ob];
												double Uss_y = upa_y[control_ob];
												double vir_x_;
												double vir_y_;
												vir_x_ = Uss_x * cos(start_to_fus[2]) - Uss_y * sin(start_to_fus[2]) + start_to_fus[0];
												vir_y_ = Uss_y * cos(start_to_fus[2]) + Uss_x * sin(start_to_fus[2]) + start_to_fus[1];
												int idx_x_ = ceil((pathfind_parameters.MAXX - vir_x_) / pathfind_parameters.MOTION_RESOLUTION);
												int idx_y_ = ceil((pathfind_parameters.MAXY - vir_y_) / pathfind_parameters.MOTION_RESOLUTION);
												if (idx_x_ == 0)
												{
													idx_x_ = 1;
												}
												if (idx_y_ == 0)
												{
													idx_y_ = 1;
												}
												int index_xy_ = (idx_x_ - 1) * pathfind_parameters.XIDX + (idx_y_ - 1);
												if ((index_xy_ >= 0) && (index_xy_ < pathfind_parameters.MAX_IDX) && 
												(idx_x_ >= 0) && (idx_x_ <= pathfind_parameters.XIDX) && (idx_y_ >= 0) && (idx_y_ <= pathfind_parameters.XIDY))
												{
													obstmap[index_xy_].Status = 4;
													int i = idx_x - 1;
													int j = idx_y - 1;
													int dir1 = i * pathfind_parameters.XIDX + j + 1;
													int dir2 = i * pathfind_parameters.XIDX + j - 1;
													int dir3 = (i + 1) * pathfind_parameters.XIDX + j;
													int dir4 = (i - 1) * pathfind_parameters.XIDX + j;
													int dir5 = (i + 1) * pathfind_parameters.XIDX + j + 1;
													int dir6 = (i + 1) * pathfind_parameters.XIDX + j - 1;
													int dir7 = (i - 1) * pathfind_parameters.XIDX + j + 1;
													int dir8 = (i - 1) * pathfind_parameters.XIDX + j - 1;

													if (dir1 >= 0 && dir1 < pathfind_parameters.MAX_IDX && 
													dir2 >= 0 && dir2 < pathfind_parameters.MAX_IDX && 
													dir3 >= 0 && dir3 < pathfind_parameters.MAX_IDX && 
													dir4 >= 0 && dir4 < pathfind_parameters.MAX_IDX && 
													dir5 >= 0 && dir5 < pathfind_parameters.MAX_IDX && 
													dir6 >= 0 && dir6 < pathfind_parameters.MAX_IDX && 
													dir7 >= 0 && dir7 < pathfind_parameters.MAX_IDX && 
													dir8 >= 0 && dir8 < pathfind_parameters.MAX_IDX)
													{
														if (obstmap[dir1].Status != 0 && obstmap[dir1].Status != 3 && obstmap[dir1].Status != 5)
														{
															obstmap[dir1].Status = 9u;
														}
														if (obstmap[dir2].Status != 0 && obstmap[dir2].Status != 3 && obstmap[dir2].Status != 5)
														{
															obstmap[dir2].Status = 9u;
														}
														if (obstmap[dir3].Status != 0 && obstmap[dir3].Status != 3 && obstmap[dir3].Status != 5)
														{
															obstmap[dir3].Status = 9u;
														}
														if (obstmap[dir4].Status != 0 && obstmap[dir4].Status != 3 && obstmap[dir4].Status != 5)
														{
															obstmap[dir4].Status = 9u;
														}
														if (obstmap[dir5].Status != 0 && obstmap[dir5].Status != 3 && obstmap[dir5].Status != 5)
														{
															obstmap[dir5].Status = 9u;
														}
														if (obstmap[dir6].Status != 0 && obstmap[dir6].Status != 3 && obstmap[dir6].Status != 5)
														{
															obstmap[dir6].Status = 9u;
														}
														if (obstmap[dir7].Status != 0 && obstmap[dir7].Status != 3 && obstmap[dir7].Status != 5)
														{
															obstmap[dir7].Status = 9u;
														}
														if (obstmap[dir8].Status != 0 && obstmap[dir8].Status != 3 && obstmap[dir8].Status != 5)
														{
															obstmap[dir8].Status = 9u;
														}
													}
												}
											}
										}
									}
									else if (obstmap[index_xy].Status != 0)
									{
										obstmap[index_xy].Status = 3;
									}
									//obstmap[index_xy].Status = 3;
								}

							}

						}

						double cos_theta_add = -sin_P03;
						double sin_theta_add = cos_P03;
						double cos_theta_diff = sin_P03;
						double sin_theta_diff = -cos_P03;
						double P0x_towards_1 = P0x + 5 * cos_theta_add;
						double P0y_towards_1 = P0y + 5 * sin_theta_add;
						double P0x_towards_2 = P0x + 5 * cos_theta_diff;
						double P0y_towards_2 = P0y + 5 * sin_theta_diff;
						double D_add = sqrt((P0x_towards_1 - end_to_fus[0]) * (P0x_towards_1 - end_to_fus[0]) + (P0y_towards_1 - end_to_fus[1]) * (P0y_towards_1 - end_to_fus[1]));
						double D_diff = sqrt((P0x_towards_2 - end_to_fus[0]) * (P0x_towards_2 - end_to_fus[0]) + (P0y_towards_2 - end_to_fus[1]) * (P0y_towards_2 - end_to_fus[1]));
						double P0x_towards = 0;
						double P0y_towards = 0;
						if (D_add > D_diff)
						{
							P0x_towards = P0x_towards_1;
							P0y_towards = P0y_towards_1;
						}
						else
						{
							P0x_towards = P0x_towards_2;
							P0y_towards = P0y_towards_2;
						}
						for (int idx = 0; idx < 80; idx++)
						{
							vir_x = P0x_towards + idx * 0.3 * cos_P03;
							vir_y = P0y_towards + idx * 0.3 * sin_P03;
							idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
							idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
							if (idx_x == 0)
							{
								idx_x = 1;
							}
							if (idx_y == 0)
							{
								idx_y = 1;
							}
							index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								if (obstmap[index_xy].Status == 2)
								{
									obstmap[index_xy].Status = 3;
								}
							}
						}
						for (int idx = 1; idx < 80; idx++)
						{
							vir_x = P0x_towards - idx * 0.3 * cos_P03;
							vir_y = P0y_towards - idx * 0.3 * sin_P03;
							idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
							idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
							if (idx_x == 0)
							{
								idx_x = 1;
							}
							if (idx_y == 0)
							{
								idx_y = 1;
							}
							index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								if (obstmap[index_xy].Status == 2)
								{
									obstmap[index_xy].Status = 3;
								}
							}
						}

						double D_P01_imag = sqrt((P1x_imag - P0x_imag) * (P1x_imag - P0x_imag) + (P1y_imag - P0y_imag) * (P1y_imag - P0y_imag));
						if (D_P01_imag < 0.1)
						{
							return 1;
						}
						double cos_theta_P01imag = (P1x_imag - P0x_imag) / D_P01_imag;
						double sin_theta_P01imag = (P1y_imag - P0y_imag) / D_P01_imag;
						int index_imag_P01 = ceil(D_P01_imag / 0.3);
						if (index_imag_P01 >= 1)
						{
							for (int idx = 1; idx < index_imag_P01; idx++)
							{
								vir_x = P0x_imag + idx * 0.3 * cos_theta_P01imag;
								vir_y = P0y_imag + idx * 0.3 * sin_theta_P01imag;
								idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
								idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
								if (idx_x == 0)
								{
									idx_x = 1;
								}
								if (idx_y == 0)
								{
									idx_y = 1;
								}
								index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
								if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
								(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
								{
									if (obstmap[index_xy].Status == 2)
									{
										obstmap[index_xy].Status = 3;
									}
								}
							}
						}

						double D_P23_imag = sqrt((P2x_imag - P3x_imag) * (P2x_imag - P3x_imag) + (P2y_imag - P3y_imag) * (P2y_imag - P3y_imag));
						if (D_P23_imag < 0.1)
						{
							return 1;
						}
						double cos_theta_P23imag = (P3x_imag - P2x_imag) / D_P23_imag;
						double sin_theta_P23imag = (P3y_imag - P2y_imag) / D_P23_imag;
						int index_imag_P23 = ceil(D_P23_imag / 0.3);
						if (index_imag_P23 >= 1)
						{
							for (int idx = 1; idx < index_imag_P23; idx++)
							{
								vir_x = P2x_imag + idx * 0.3 * cos_theta_P23imag;
								vir_y = P2y_imag + idx * 0.3 * sin_theta_P23imag;
								idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
								idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
								if (idx_x == 0)
								{
									idx_x = 1;
								}
								if (idx_y == 0)
								{
									idx_y = 1;
								}
								index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
								if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
								(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
								{
									if (obstmap[index_xy].Status == 2)
									{
										obstmap[index_xy].Status = 3;
									}
								}
							}
						}
					}
					else // 车头泊入
					{
						double P_mean_x = (P0x + P1x + P2x + P3x) / 4;
						double P_mean_y = (P0y + P1y + P2y + P3y) / 4;

						int idx_x;
						int idx_y;
						int index_xy;
						double vir_x;
						double vir_y;

						double D_P03 = sqrt((P0x - P3x) * (P0x - P3x) + (P0y - P3y) * (P0y - P3y));
						double D_P12 = sqrt((P1x - P2x) * (P1x - P2x) + (P1y - P2y) * (P1y - P2y));
						double Diff_xP03 = P3x - P0x;
						double Diff_yP03 = P3y - P0y;
						double Diff_xP12 = P2x - P1x;
						double Diff_yP12 = P2y - P1y;
						if ((D_P03 < 0.1) || (D_P12 < 0.1))
						{
							return 1;
						}
						double cos_P03 = Diff_xP03 / D_P03;
						double sin_P03 = Diff_yP03 / D_P03;
						double P0x_imag = P0x - 0.5 * cos_P03;
						double P0y_imag = P0y - 0.5 * sin_P03;
						double P3x_imag = P3x + 0.5 * cos_P03;
						double P3y_imag = P3y + 0.5 * sin_P03;

						double D_P1_mean = sqrt((P_mean_x - P1x) * (P_mean_x - P1x) + (P_mean_y - P1y) * (P_mean_y - P1y));
						double D_P2_mean = sqrt((P_mean_x - P2x) * (P_mean_x - P2x) + (P_mean_y - P2y) * (P_mean_y - P2y));
						if ((D_P1_mean < 0.1) && (D_P2_mean < 0.1))
						{
							return 1;
						}
						double cos_theta_P1 = (P_mean_x - P1x) / D_P1_mean;
						double sin_theta_P1 = (P_mean_y - P1y) / D_P1_mean;
						double cos_theta_P2 = (P_mean_x - P2x) / D_P2_mean;
						double sin_theta_P2 = (P_mean_y - P2y) / D_P2_mean;
						double P1x_imag = P_mean_x - (D_P1_mean + 2) * cos_theta_P1;
						double P1y_imag = P_mean_y - (D_P1_mean + 2) * sin_theta_P1;
						double P2x_imag = P_mean_x - (D_P2_mean + 2) * cos_theta_P2;
						double P2y_imag = P_mean_y - (D_P2_mean + 2) * sin_theta_P2;

						double cos_P12 = Diff_xP12 / D_P12;
						double sin_P12 = Diff_yP12 / D_P12;
						//double P1x_imag = P1x - 0.5 * cos_P12;
						//double P1x_imag = P1y - 0.5 * sin_P12;
						//double P2x_imag = P2x + 0.5 * cos_P12;
						//double P2x_imag = P2y + 0.5 * sin_P12;
						////////////////起始位置如果有虚拟障碍物就不要/////////////////////
						for (unsigned int map_index = 0; map_index < Car_x.size(); map_index++)
						{
							vir_x = Car_x[map_index] * cos(start_to_fus[2]) - Car_y[map_index] * sin(start_to_fus[2]) + start_to_fus[0];
							vir_y = Car_y[map_index] * cos(start_to_fus[2]) + Car_x[map_index] * sin(start_to_fus[2]) + start_to_fus[1];
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
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								obstmap[index_xy].Status = 5;
							}
						}
						/////////////////////////////////////////////////////////////////////////
						for (unsigned int map_vertical_index = 0; map_vertical_index <  map_image_vertical_x.size(); map_vertical_index++)
						{
							vir_x = (-1 * map_image_vertical_x[map_vertical_index] + vehicle_parameters.LF - vehicle_parameters.LB) * cos(end_to_fus[2]) - map_image_vertical_y[map_vertical_index] * sin(end_to_fus[2]) + end_to_fus[0];
							vir_y = map_image_vertical_y[map_vertical_index] * cos(end_to_fus[2]) + (-1 * map_image_vertical_x[map_vertical_index] + vehicle_parameters.LF - vehicle_parameters.LB) * sin(end_to_fus[2]) + end_to_fus[1];
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
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								if (obstmap[index_xy].Status == 5)
								{
									no_imag_map = 1;
									double SE_Usspark = (start_to_fus[0] - end_to_fus[0]) * (start_to_fus[0] - end_to_fus[0]) + (start_to_fus[1] - end_to_fus[1]) * (start_to_fus[1] - end_to_fus[1]);
									if (SE_Usspark < 2.25)
									{
										for (int Uss_start_index = 0; Uss_start_index < 2; Uss_start_index++)
										{
											int control_ob = 0 + Uss_start_index;
											double Uss_x = upa_x[control_ob];
											double Uss_y = upa_y[control_ob];
											double vir_x_;
											double vir_y_;
											vir_x_ = Uss_x * cos(start_to_fus[2]) - Uss_y * sin(start_to_fus[2]) + start_to_fus[0];
											vir_y_ = Uss_y * cos(start_to_fus[2]) + Uss_x * sin(start_to_fus[2]) + start_to_fus[1];
											int idx_x_ = ceil((pathfind_parameters.MAXX - vir_x_) / pathfind_parameters.MOTION_RESOLUTION);
											int idx_y_ = ceil((pathfind_parameters.MAXY - vir_y_) / pathfind_parameters.MOTION_RESOLUTION);
											if (idx_x_ == 0)
											{
												idx_x_ = 1;
											}
											if (idx_y_ == 0)
											{
												idx_y_ = 1;
											}
											int index_xy_ = (idx_x_ - 1) * pathfind_parameters.XIDX + (idx_y_ - 1);
											if ((index_xy_ >= 0) && (index_xy_ < pathfind_parameters.MAX_IDX) && 
											(idx_x_ >= 0) && (idx_x_ <= pathfind_parameters.XIDX) && (idx_y_ >= 0) && (idx_y_ <= pathfind_parameters.XIDY))
											{
												obstmap[index_xy_].Status = 4;
												int i = idx_x - 1;
												int j = idx_y - 1;
												int dir1 = i * pathfind_parameters.XIDX + j + 1;
												int dir2 = i * pathfind_parameters.XIDX + j - 1;
												int dir3 = (i + 1) * pathfind_parameters.XIDX + j;
												int dir4 = (i - 1) * pathfind_parameters.XIDX + j;
												int dir5 = (i + 1) * pathfind_parameters.XIDX + j + 1;
												int dir6 = (i + 1) * pathfind_parameters.XIDX + j - 1;
												int dir7 = (i - 1) * pathfind_parameters.XIDX + j + 1;
												int dir8 = (i - 1) * pathfind_parameters.XIDX + j - 1;

												if (dir1 >= 0 && dir1 < pathfind_parameters.MAX_IDX && 
												dir2 >= 0 && dir2 < pathfind_parameters.MAX_IDX && 
												dir3 >= 0 && dir3 < pathfind_parameters.MAX_IDX && 
												dir4 >= 0 && dir4 < pathfind_parameters.MAX_IDX && 
												dir5 >= 0 && dir5 < pathfind_parameters.MAX_IDX && 
												dir6 >= 0 && dir6 < pathfind_parameters.MAX_IDX && 
												dir7 >= 0 && dir7 < pathfind_parameters.MAX_IDX && 
												dir8 >= 0 && dir8 < pathfind_parameters.MAX_IDX)
												{
													if (obstmap[dir1].Status != 0 && obstmap[dir1].Status != 3 && obstmap[dir1].Status != 5)
													{
														obstmap[dir1].Status = 9u;
													}
													if (obstmap[dir2].Status != 0 && obstmap[dir2].Status != 3 && obstmap[dir2].Status != 5)
													{
														obstmap[dir2].Status = 9u;
													}
													if (obstmap[dir3].Status != 0 && obstmap[dir3].Status != 3 && obstmap[dir3].Status != 5)
													{
														obstmap[dir3].Status = 9u;
													}
													if (obstmap[dir4].Status != 0 && obstmap[dir4].Status != 3 && obstmap[dir4].Status != 5)
													{
														obstmap[dir4].Status = 9u;
													}
													if (obstmap[dir5].Status != 0 && obstmap[dir5].Status != 3 && obstmap[dir5].Status != 5)
													{
														obstmap[dir5].Status = 9u;
													}
													if (obstmap[dir6].Status != 0 && obstmap[dir6].Status != 3 && obstmap[dir6].Status != 5)
													{
														obstmap[dir6].Status = 9u;
													}
													if (obstmap[dir7].Status != 0 && obstmap[dir7].Status != 3 && obstmap[dir7].Status != 5)
													{
														obstmap[dir7].Status = 9u;
													}
													if (obstmap[dir8].Status != 0 && obstmap[dir8].Status != 3 && obstmap[dir8].Status != 5)
													{
														obstmap[dir8].Status = 9u;
													}
												}
											}
										}
									}
								}
								else if (obstmap[index_xy].Status != 0)
								{
									obstmap[index_xy].Status = 3;
								}
								//obstmap[index_xy].Status = 3;
							}
						}

						for (int idx = 1; idx < 50; idx++)
						{
							vir_x = P3x_imag + idx * 0.3 * cos_P03;
							vir_y = P3y_imag + idx * 0.3 * sin_P03;
							idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
							idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
							if (idx_x == 0)
							{
								idx_x = 1;
							}
							if (idx_y == 0)
							{
								idx_y = 1;
							}
							index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								if (obstmap[index_xy].Status == 2)
								{
									obstmap[index_xy].Status = 3;
								}
							}
						}
						for (int idx = 1; idx < 50; idx++)
						{
							vir_x = P0x_imag - idx * 0.3 * cos_P03;
							vir_y = P0y_imag - idx * 0.3 * sin_P03;
							idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
							idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
							if (idx_x == 0)
							{
								idx_x = 1;
							}
							if (idx_y == 0)
							{
								idx_y = 1;
							}
							index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								if (obstmap[index_xy].Status == 2)
								{
									obstmap[index_xy].Status = 3;
								}
							}
						}

						double D_imag_12 = sqrt((P1x_imag - P2x_imag) * (P1x_imag - P2x_imag) + (P1y_imag - P2y_imag) * (P1y_imag - P2y_imag));
						int idx_imag_12 = ceil(D_imag_12 / 0.025);
						if (idx_imag_12 > 0)
						{
							for (int idx = 0; idx < idx_imag_12; idx++)
							{
								vir_x = P1x_imag + idx * 0.025 * cos_P12;
								vir_y = P1y_imag + idx * 0.025 * sin_P12;
								idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
								idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
								if (idx_x == 0)
								{
									idx_x = 1;
								}
								if (idx_y == 0)
								{
									idx_y = 1;
								}
								index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
								if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
								(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
								{
									if (obstmap[index_xy].Status == 5)
									{
										no_imag_map = 1;
										double SE_Usspark = (start_to_fus[0] - end_to_fus[0]) * (start_to_fus[0] - end_to_fus[0]) + (start_to_fus[1] - end_to_fus[1]) * (start_to_fus[1] - end_to_fus[1]);
										if (SE_Usspark < 2.25)
										{
											for (int Uss_start_index = 0; Uss_start_index < 2; Uss_start_index++)
											{
												int control_ob = 1 + Uss_start_index;
												double Uss_x = upa_x[control_ob];
												double Uss_y = upa_y[control_ob];
												double vir_x_;
												double vir_y_;
												vir_x_ = Uss_x * cos(start_to_fus[2]) - Uss_y * sin(start_to_fus[2]) + start_to_fus[0];
												vir_y_ = Uss_y * cos(start_to_fus[2]) + Uss_x * sin(start_to_fus[2]) + start_to_fus[1];
												int idx_x_ = ceil((pathfind_parameters.MAXX - vir_x_) / pathfind_parameters.MOTION_RESOLUTION);
												int idx_y_ = ceil((pathfind_parameters.MAXY - vir_y_) / pathfind_parameters.MOTION_RESOLUTION);
												if (idx_x_ == 0)
												{
													idx_x_ = 1;
												}
												if (idx_y_ == 0)
												{
													idx_y_ = 1;
												}
												int index_xy_ = (idx_x_ - 1) * pathfind_parameters.XIDX + (idx_y_ - 1);
												if ((index_xy_ >= 0) && (index_xy_ < pathfind_parameters.MAX_IDX) && 
												(idx_x_ >= 0) && (idx_x_ <= pathfind_parameters.XIDX) && (idx_y_ >= 0) && (idx_y_ <= pathfind_parameters.XIDY))
												{
													obstmap[index_xy_].Status = 4;
													int i = idx_x - 1;
													int j = idx_y - 1;
													int dir1 = i * pathfind_parameters.XIDX + j + 1;
													int dir2 = i * pathfind_parameters.XIDX + j - 1;
													int dir3 = (i + 1) * pathfind_parameters.XIDX + j;
													int dir4 = (i - 1) * pathfind_parameters.XIDX + j;
													int dir5 = (i + 1) * pathfind_parameters.XIDX + j + 1;
													int dir6 = (i + 1) * pathfind_parameters.XIDX + j - 1;
													int dir7 = (i - 1) * pathfind_parameters.XIDX + j + 1;
													int dir8 = (i - 1) * pathfind_parameters.XIDX + j - 1;

													if (dir1 >= 0 && dir1 < pathfind_parameters.MAX_IDX && 
													dir2 >= 0 && dir2 < pathfind_parameters.MAX_IDX && 
													dir3 >= 0 && dir3 < pathfind_parameters.MAX_IDX && 
													dir4 >= 0 && dir4 < pathfind_parameters.MAX_IDX && 
													dir5 >= 0 && dir5 < pathfind_parameters.MAX_IDX && 
													dir6 >= 0 && dir6 < pathfind_parameters.MAX_IDX && 
													dir7 >= 0 && dir7 < pathfind_parameters.MAX_IDX && 
													dir8 >= 0 && dir8 < pathfind_parameters.MAX_IDX)
													{
														if (obstmap[dir1].Status != 0 && obstmap[dir1].Status != 3 && obstmap[dir1].Status != 5)
														{
															obstmap[dir1].Status = 9u;
														}
														if (obstmap[dir2].Status != 0 && obstmap[dir2].Status != 3 && obstmap[dir2].Status != 5)
														{
															obstmap[dir2].Status = 9u;
														}
														if (obstmap[dir3].Status != 0 && obstmap[dir3].Status != 3 && obstmap[dir3].Status != 5)
														{
															obstmap[dir3].Status = 9u;
														}
														if (obstmap[dir4].Status != 0 && obstmap[dir4].Status != 3 && obstmap[dir4].Status != 5)
														{
															obstmap[dir4].Status = 9u;
														}
														if (obstmap[dir5].Status != 0 && obstmap[dir5].Status != 3 && obstmap[dir5].Status != 5)
														{
															obstmap[dir5].Status = 9u;
														}
														if (obstmap[dir6].Status != 0 && obstmap[dir6].Status != 3 && obstmap[dir6].Status != 5)
														{
															obstmap[dir6].Status = 9u;
														}
														if (obstmap[dir7].Status != 0 && obstmap[dir7].Status != 3 && obstmap[dir7].Status != 5)
														{
															obstmap[dir7].Status = 9u;
														}
														if (obstmap[dir8].Status != 0 && obstmap[dir8].Status != 3 && obstmap[dir8].Status != 5)
														{
															obstmap[dir8].Status = 9u;
														}
													}
												}
											}
										}
									}
									else if (obstmap[index_xy].Status != 0)
									{
										obstmap[index_xy].Status = 3;
									}
									//obstmap[index_xy].Status = 3;
								}

							}

						}

						double cos_theta_add = -sin_P03;
						double sin_theta_add = cos_P03;
						double cos_theta_diff = sin_P03;
						double sin_theta_diff = -cos_P03;
						double P0x_towards_1 = P0x + 5 * cos_theta_add;
						double P0y_towards_1 = P0y + 5 * sin_theta_add;
						double P0x_towards_2 = P0x + 5 * cos_theta_diff;
						double P0y_towards_2 = P0y + 5 * sin_theta_diff;
						double D_add = sqrt((P0x_towards_1 - end_to_fus[0]) * (P0x_towards_1 - end_to_fus[0]) + (P0y_towards_1 - end_to_fus[1]) * (P0y_towards_1 - end_to_fus[1]));
						double D_diff = sqrt((P0x_towards_2 - end_to_fus[0]) * (P0x_towards_2 - end_to_fus[0]) + (P0y_towards_2 - end_to_fus[1]) * (P0y_towards_2 - end_to_fus[1]));
						double P0x_towards = 0;
						double P0y_towards = 0;
						if (D_add > D_diff)
						{
							P0x_towards = P0x_towards_1;
							P0y_towards = P0y_towards_1;
						}
						else
						{
							P0x_towards = P0x_towards_2;
							P0y_towards = P0y_towards_2;
						}
						for (int idx = 0; idx < 80; idx++)
						{
							vir_x = P0x_towards + idx * 0.3 * cos_P03;
							vir_y = P0y_towards + idx * 0.3 * sin_P03;
							idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
							idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
							if (idx_x == 0)
							{
								idx_x = 1;
							}
							if (idx_y == 0)
							{
								idx_y = 1;
							}
							index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								if (obstmap[index_xy].Status == 2)
								{
									obstmap[index_xy].Status = 3;
								}
							}
						}
						for (int idx = 1; idx < 80; idx++)
						{
							vir_x = P0x_towards - idx * 0.3 * cos_P03;
							vir_y = P0y_towards - idx * 0.3 * sin_P03;
							idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
							idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
							if (idx_x == 0)
							{
								idx_x = 1;
							}
							if (idx_y == 0)
							{
								idx_y = 1;
							}
							index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
							if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
							(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
							{
								if (obstmap[index_xy].Status == 2)
								{
									obstmap[index_xy].Status = 3;
								}
							}
						}

						double D_P01_imag = sqrt((P1x_imag - P0x_imag) * (P1x_imag - P0x_imag) + (P1y_imag - P0y_imag) * (P1y_imag - P0y_imag));
						if (D_P01_imag < 0.1)
						{
							return 1;
						}
						double cos_theta_P01imag = (P1x_imag - P0x_imag) / D_P01_imag;
						double sin_theta_P01imag = (P1y_imag - P0y_imag) / D_P01_imag;
						int index_imag_P01 = ceil(D_P01_imag / 0.3);
						if (index_imag_P01 >= 1)
						{
							for (int idx = 1; idx < index_imag_P01; idx++)
							{
								vir_x = P0x_imag + idx * 0.3 * cos_theta_P01imag;
								vir_y = P0y_imag + idx * 0.3 * sin_theta_P01imag;
								idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
								idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
								if (idx_x == 0)
								{
									idx_x = 1;
								}
								if (idx_y == 0)
								{
									idx_y = 1;
								}
								index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
								if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
								(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
								{
									if (obstmap[index_xy].Status == 2)
									{
										obstmap[index_xy].Status = 3;
									}
								}
							}
						}

						double D_P23_imag = sqrt((P2x_imag - P3x_imag) * (P2x_imag - P3x_imag) + (P2y_imag - P3y_imag) * (P2y_imag - P3y_imag));
						if (D_P23_imag < 0.1)
						{
							return 1;
						}
						double cos_theta_P23imag = (P3x_imag - P2x_imag) / D_P23_imag;
						double sin_theta_P23imag = (P3y_imag - P2y_imag) / D_P23_imag;
						int index_imag_P23 = ceil(D_P23_imag / 0.3);
						if (index_imag_P23 >= 1)
						{
							for (int idx = 1; idx < index_imag_P23; idx++)
							{
								vir_x = P2x_imag + idx * 0.3 * cos_theta_P23imag;
								vir_y = P2y_imag + idx * 0.3 * sin_theta_P23imag;
								idx_x = ceil((pathfind_parameters.MAXX - vir_x) / pathfind_parameters.MOTION_RESOLUTION);
								idx_y = ceil((pathfind_parameters.MAXY - vir_y) / pathfind_parameters.MOTION_RESOLUTION);
								if (idx_x == 0)
								{
									idx_x = 1;
								}
								if (idx_y == 0)
								{
									idx_y = 1;
								}
								index_xy = (idx_x - 1) * pathfind_parameters.XIDX + (idx_y - 1);
								if ((index_xy >= 0) && (index_xy < pathfind_parameters.MAX_IDX) && 
								(idx_x >= 0) && (idx_x <= pathfind_parameters.XIDX) && (idx_y >= 0) && (idx_y <= pathfind_parameters.XIDY))
								{
									if (obstmap[index_xy].Status == 2)
									{
										obstmap[index_xy].Status = 3;
									}
								}
							}
						}
					}
				}
			}

			return 0;
		}
		else
		{
			return 1;
		}
	}

    void ExpandObstaclePoint()
    {
        for (int i = 0; i < pathfind_parameters.XIDX; i++) 
        {
            for (int j = 0; j < pathfind_parameters.XIDY; j++) 
            {
                if ((obstmap[i * pathfind_parameters.XIDX + j].Status == 0) || (obstmap[i * pathfind_parameters.XIDX + j].Status == 3) 
				|| (obstmap[i * pathfind_parameters.XIDX + j].Status == 4))
                {
					dis_map[i][j]=1;
                    int dir1 = i * pathfind_parameters.XIDX + j + 1;
                    int dir2 = i * pathfind_parameters.XIDX + j - 1;
                    int dir3 = (i + 1) * pathfind_parameters.XIDX + j;
                    int dir4 = (i - 1) * pathfind_parameters.XIDX + j;
                    int dir5 = (i + 1) * pathfind_parameters.XIDX + j + 1;
                    int dir6 = (i + 1) * pathfind_parameters.XIDX+ j - 1;
                    int dir7 = (i - 1) * pathfind_parameters.XIDX + j + 1;
                    int dir8 = (i - 1) * pathfind_parameters.XIDX+ j - 1;

                    if (dir1 >= 0 && dir1 < pathfind_parameters.MAX_IDX && obstmap[dir1].Status != 0 && obstmap[dir1].Status != 3 && obstmap[dir1].Status != 4)
                    {
						// dis_map[i][j+1]=1;
                        obstmap[dir1].Status = 9u;
                    }
                    if (dir2 >= 0 && dir2 < pathfind_parameters.MAX_IDX && obstmap[dir2].Status != 0 && obstmap[dir2].Status != 3 && obstmap[dir1].Status != 4)
                    {
						// dis_map[i][j-1]=1;
                        obstmap[dir2].Status = 9u;
                    }
                    if (dir3 >= 0 && dir3 < pathfind_parameters.MAX_IDX && obstmap[dir3].Status != 0 && obstmap[dir3].Status != 3 && obstmap[dir1].Status != 4)
                    {
						// dis_map[i+1][j]=1;
                        obstmap[dir3].Status = 9u;
                    }
                    if (dir4 >= 0 && dir4 < pathfind_parameters.MAX_IDX && obstmap[dir4].Status != 0 && obstmap[dir4].Status != 3 && obstmap[dir1].Status != 4)
                    {
						// dis_map[i-1][j]=1;
                        obstmap[dir4].Status = 9u;
                    }
                    if (dir5 >= 0 && dir5 < pathfind_parameters.MAX_IDX && obstmap[dir5].Status != 0 && obstmap[dir5].Status != 3 && obstmap[dir1].Status != 4)
                    {
						// dis_map[i+1][j+1]=1;
                        obstmap[dir5].Status = 9u;
                    }
                    if (dir6 >= 0 && dir6 < pathfind_parameters.MAX_IDX && obstmap[dir6].Status != 0 && obstmap[dir6].Status != 3 && obstmap[dir1].Status != 4)
                    {
						// dis_map[i+1][j-1]=1;
                        obstmap[dir6].Status = 9u;
                    }
                    if (dir7 >= 0 && dir7 < pathfind_parameters.MAX_IDX && obstmap[dir7].Status != 0 && obstmap[dir7].Status != 3 && obstmap[dir1].Status != 4)
                    {
						// dis_map[i-1][j+1]=1;
                        obstmap[dir7].Status = 9u;
                    }
                    if (dir8 >= 0 && dir8 < pathfind_parameters.MAX_IDX && obstmap[dir8].Status != 0 && obstmap[dir8].Status != 3 && obstmap[dir1].Status != 4)
                    {
						// dis_map[i-1][j-1]=1;
                        obstmap[dir8].Status = 9u;
                    }
                }
            }
        }
    }
    void ModifyMapHybirdAStar()
    {
        hybird_astar_imagmap_get();
        ExpandObstaclePoint();
    }

}
