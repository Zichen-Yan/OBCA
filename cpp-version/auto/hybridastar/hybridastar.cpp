// #include "../common/global_variable.h"
// #include "../common/global_function.h"
// #define SAFE
// #define URE

// namespace byd_apa_plan
// {
// 	std::string get_str_idx(Node &node);
// 	std::string get_par_str_idx(Node &node);
// 	/////////////////////RS函数声明////////////////////////////////
// 	RsPath FindRSPathCCC(double x, double y, double phi);
// 	int FindRSPathCCS(double x, double y, double phi, RsPath& path);
// 	RsPath FindRSPathCSC(double x, double y, double phi);
// 	double TotalCost(Node wknode, double End[3]);
// 	int inNodes(Node node, std::vector<Node>* nodes);
// 	void Update(Node wknode, double End[3]);
// 	bool CalcNextNode(Node wknode, double D, double delta);
// 	RsPath AnalysticExpantion(Node wknode, double End[3]);
// 	void getFinalPath(RsPath path, std::string stridx);


// 	double TotalCost(Node wknode, double End[3])
// 	{
// 		#ifdef SAFE
// 		// compute dis cost
// 		double length = vehicle_parameters.LB + vehicle_parameters.LF + 0.4;
//         double width = vehicle_parameters.W+0.4;
//         double radius = sqrt(pow(width/2,2)+pow(length/6,2))-0.14;
// 		double offset=length/3;
// 		double rect_x = length / 2 - (vehicle_parameters.LB + 0.2); // 车辆中心坐标
		
// 		double center_x1 = (rect_x+offset-0.25) * cos(wknode.theta) + wknode.x;	// 圆心坐标
// 		double center_y1 = (rect_x+offset-0.25) * sin(wknode.theta) + wknode.y;
// 		int xidx1 = ceil((pathfind_parameters.MAXX - center_x1) / pathfind_parameters.XY_GRID_RESOLUTION);
// 		int yidx1 = ceil((pathfind_parameters.MAXY - center_y1) / pathfind_parameters.XY_GRID_RESOLUTION);
//         if (xidx1 == 0)
// 		{
// 			xidx1 = 1;
// 		}
// 		if (yidx1 == 0)
// 		{
// 			yidx1 = 1;
// 		}
// 		double distance1 = dis_map[(xidx1 - 1)][(yidx1 - 1)] * pathfind_parameters.XY_GRID_RESOLUTION;
		
//         #ifdef URE
// 		double center_x3 = (rect_x-offset+0.4) * cos(wknode.theta) + wknode.x;
// 		double center_y3 = (rect_x-offset+0.4) * sin(wknode.theta) + wknode.y;
// 		#else
// 		double center_x3 = (rect_x-offset+0.25) * cos(wknode.theta) + wknode.x;
// 		double center_y3 = (rect_x-offset+0.25) * sin(wknode.theta) + wknode.y;
// 		#endif
// 		int xidx3 = ceil((pathfind_parameters.MAXX - center_x3) / pathfind_parameters.XY_GRID_RESOLUTION);
// 		int yidx3 = ceil((pathfind_parameters.MAXY - center_y3) / pathfind_parameters.XY_GRID_RESOLUTION);
//         if (xidx3 == 0)
// 		{
// 			xidx3 = 1;
// 		}
// 		if (yidx3 == 0)
// 		{
// 			yidx3 = 1;
// 		}
// 		double distance3 = dis_map[(xidx3 - 1)][(yidx3 - 1)] * pathfind_parameters.XY_GRID_RESOLUTION;

// 		double dis = std::min(distance1,distance3);
// 		double distance_cost;
// 		if (dis < radius+0.15){
// 			distance_cost = 1 / (dis - 1) * 5;
// 		}
// 		else{
// 			distance_cost = 0;
// 		}
// 		#endif
// 		// 转换到起点为原点的坐标系
// 		double rmin = vehicle_parameters.MIN_CIRCLE;
// 		double x = End[0] - wknode.x;
// 		double y = End[1] - wknode.y;
// 		double ph = End[2] - wknode.theta;
// 		double phi = mod2pi(wknode.theta);
// 		// 起点start坐标系在基坐标系下的方向余弦矩阵(Z轴旋转，因为点在Z = 0平面，因此绕Z旋转仍然在Z = 0平面)
// 		// dcm*x 表示将基坐标中的x表示到旋转后的坐标系中，即计算坐标旋转后各向量在新坐标中的表示
// 		double x1 = x*cos(phi) + y*sin(phi);
// 		double y1 = y*cos(phi) - x*sin(phi);
// 		//看是否从当前点到目标点存在无碰撞的Reeds - Shepp轨迹，前面pvec = End - Start; 的意义就在这里，注意！这里的x, y, prev(3)是把起点转换成以start为原点坐标系下的坐标
// 		double cost = 0;
		
// 		if (fusion.parkingSpaceInfo.ParkingSpaceType == 2)
// 		{
// 			if ((Theta_se < 0.2793) && (Y_se > 1))
// 			{
// 				RsPath path;
// 				ErrorCode = FindRSPathCCS(x1, y1, ph, path);
// 				if (ErrorCode == 0)
// 				{
// 					#ifdef SAFE
// 					cost += pathfind_parameters.H_COST * rmin * path.lenth + wknode.cost + distance_cost; /// Z.Y.
// 					#else
// 					cost += pathfind_parameters.H_COST * rmin * path.lenth + wknode.cost;
// 					#endif
// 				}
// 				else
// 				{
// 					cost = 1000;
// 				}
// 			}
// 			else
// 			{
// 				RsPath path = FindRSPathCCC(x1, y1, ph);
// 				#ifdef SAFE
// 				cost += pathfind_parameters.H_COST * rmin * path.lenth + wknode.cost + distance_cost; /// Z.Y.
// 				#else
// 				cost += pathfind_parameters.H_COST * rmin * path.lenth + wknode.cost;
// 				#endif
// 			}
// 		}
// 		else
// 		{
// 			if (SEorES == 0)
// 			{
// 				RsPath path;
// 				ErrorCode = FindRSPathCCS(x1, y1, ph, path);
// 				if (ErrorCode == 0)
// 				{
// 					#ifdef SAFE
// 					cost += pathfind_parameters.H_COST * rmin * path.lenth + wknode.cost + distance_cost; /// Z.Y.
// 					#else
// 					cost += pathfind_parameters.H_COST * rmin * path.lenth + wknode.cost;
// 					#endif
// 				}
// 				else
// 				{
// 					cost = 1000;
// 				}
// 			}
// 			else
// 			{
// 				RsPath path = FindRSPathCCC(x1, y1, ph);
// 				#ifdef SAFE
// 				cost += pathfind_parameters.H_COST * rmin * path.lenth + wknode.cost + distance_cost; /// Z.Y.
// 				#else
// 				cost += pathfind_parameters.H_COST * rmin * path.lenth + wknode.cost;
// 				#endif
// 			}
// 		}
// 		return cost;
// 	}

// 	bool CalcIdx(double x, double y, double theta)
// 	{
// 		float gres = pathfind_parameters.XY_GRID_RESOLUTION; //[m]世界坐标生成栅格坐标的分辨率（2m一个栅格）
// 		float yawres = pathfind_parameters.YAW_GRID_RESOLUTION; // 栅格角度分辨率
// 		g_xidx = round((x - pathfind_parameters.MINX) / gres); // 计算栅格索引
// 		g_yidx = round((y - pathfind_parameters.MINY) / gres); // 计算栅格索引
// 		theta = mod2pi(theta); // 控制theta范围在[-pi, pi]区间
// 		g_thidx = round((theta - pathfind_parameters.MINYAW) / (yawres / 180 * pi)); // 计算栅格索引
// 		double middle_x = (pathfind_parameters.MAXX - pathfind_parameters.MINX) / gres; // 地图边界栅格索引
// 		double middle_y = (pathfind_parameters.MAXY - pathfind_parameters.MINY) / gres; // 地图边界栅格索引
// 		if (g_xidx <= 0 || g_xidx > ceil(middle_x)) // 判断是否在地图内
// 		{
// 			return false;
// 		}
// 		else if (g_yidx <= 0 || g_yidx > ceil(middle_y)) // 判断是否在地图内
// 		{
// 			return false;
// 		}

// 		return true;
// 	}

// 	int inNodes(Node node, std::vector<Node>  *nodes) // 判断节点node是否在nodes里面
// 	{
// 		unsigned int i; 
// 		int fan = -1;
// 		for (i = 0; i < nodes->size(); i++)
// 		{
// 			if (node.xidx == (*nodes)[i].xidx
// 				&& node.yidx == (*nodes)[i].yidx
// 				&& node.yawidx == (*nodes)[i].yawidx)
// 			{
// 				fan = i;
// 			}
// 		}
// 		return fan;
// 	}

// 	bool CalcNextNode(Node wknode, double D, double delta)//根据当前点及距离及方向盘角度计算下一点
// 	{
// 		double cost = 0;
// 		int fa[3] = { wknode.xidx, wknode.yidx, wknode.yawidx };
// 		g_px = wknode.x;
// 		g_py = wknode.y;
// 		g_pth = wknode.theta;
// 		int i, nlist = 3;
// 		for (i = 0; i < nlist; i++)
// 		{
// 			VehicleDynamic(g_px, g_py, g_pth, D, delta);//更新全局变量的g_px、g_py、g_pth
			
// 			if (VehicleCollisionGrid_Astar(g_px, g_py, g_pth) == true)
// 			{
// 				return false;
// 			}
// 		}

// 		if (CalcIdx(g_px, g_py, g_pth) == false) // 把路径末端点的实际坐标转换为栅格坐标，（全局变量的g_xidx, g_yidx, g_thidx）
// 		{
// 			return false;
// 		}
// 		else
// 		{
// 			cost = wknode.cost;
// 		}

// 		if (D > 0) //前进
// 		{
// 			cost = cost + 0.3; // 每条轨迹大概是2米的长度。这里乘以1.5是确保下一个末端状态肯定在另一个栅格中，不会还在一个栅格中！在地图栅格中子结点拓展。比如对角线长度是1.4，此时还是在同一个栅格中
// 		}
// 		else // 后退
// 		{
// 			cost = cost + pathfind_parameters.BACK_COST*0.3;
// 		}

// 		if (D != wknode.D)
// 		{
// 			cost = cost + pathfind_parameters.SB_COST;
// 		}
// 		else if (delta != wknode.delta)
// 		{
// 	//		cost = cost + pathfind_parameters.STEER_CHANGE_COST;
// 			cost = cost + pathfind_parameters.STEER_CHANGE_COST * fabs(delta - wknode.delta)/ vehicle_parameters.MAX_STEER;
// 		}
// 		// cost = cost + pathfind_parameters.STEER_COST*fabs(delta);
// 	//	cost = cost + pathfind_parameters.STEER_CHANGE_COST*fabs(delta - wknode.delta);
// 		g_tnode = Node(g_xidx, g_yidx, g_thidx, D, delta, g_px, g_py, g_pth, fa, cost, 0); //tnode是路径的末端点，cost为到当前状态到此路径末端状态的成本

// 		return true;
// 	}

// 	void Update(Node wknode, double End[3]) //扩展点 ，并记录点，扩展close、OPEen集合
// 	{
// 		//int minidx = 0;                              //最小值下标
// 		unsigned int ilr = 0;                                   //左右转
// 		//double sres = vehicle_parameters.MAX_STEER / pathfind_parameters.N_STEER;     //转向分辨率
// 		double delta = -vehicle_parameters.MAX_STEER;                 //右转最小转角
// 		double mres = pathfind_parameters.MOTION_RESOLUTION;           // motino resolution    运动分辨率

// 		// all possible control input，所有可能的控制输入

// 		for (ilr = 0; ilr < find_steer_degree.size(); ilr++)
// 		{
// 			delta = find_steer_degree[ilr];
// 			if (CalcNextNode(wknode, -mres, delta) == false)  //输出全局变量的g_tnode，fcost=0；
// 			{
// 				continue;
// 			}
// 			str_idx = get_str_idx(g_tnode);
// 			if (g_closeset.find(str_idx) != g_closeset.end())  // exist
// 				continue;
			
// 			g_tnode.fcost = TotalCost(g_tnode, End);
// 			/*if (Gears == 2)
// 			{
// 				g_tnode.fcost = g_tnode.fcost + 10;
// 			}*/
// 			if (g_openset.find(str_idx) != g_openset.end())
// 			{
// 				if (g_openset[str_idx].fcost > g_tnode.fcost)
// 				{
// 					g_openset[str_idx]=g_tnode;
// 					pq.push({str_idx, g_tnode.fcost});
// 				}
// 			}
// 			else
// 			{
// 				g_openset[str_idx]=g_tnode;
// 				pq.push({str_idx, g_tnode.fcost});
// 			}
// 		}

// 		for (ilr = 0; ilr < find_steer_degree.size(); ilr++)
// 		{
// 			delta = find_steer_degree[ilr];
// 			if (CalcNextNode(wknode, mres, delta) == false)  //输出全局变量的g_tnode，fcost=0；
// 			{
// 				continue;
// 			}
// 			str_idx = get_str_idx(g_tnode);
// 			if (g_closeset.find(str_idx) != g_closeset.end())  // exist
// 				continue;
			
// 			g_tnode.fcost = TotalCost(g_tnode, End);
// 			/*if (Gears == 1)
// 			{
// 				g_tnode.fcost = g_tnode.fcost + 10;
// 			}*/

// 			if (g_openset.find(str_idx) != g_openset.end())
// 			{
// 				if (g_openset[str_idx].fcost > g_tnode.fcost)
// 				{
// 					g_openset[str_idx]=g_tnode;
// 					pq.push({str_idx, g_tnode.fcost});
// 				}
// 			}
// 			else
// 			{
// 				g_openset[str_idx]=g_tnode;
// 				pq.push({str_idx, g_tnode.fcost});
// 			}
// 		}
// 	}

// 	RsPath AnalysticExpantion(Node wknode, double End[3]) //参数节点的RS曲线是否发生碰撞
// 	{
// 		bool isCollision = false;
// 		// 将起点转换到原点计算轨迹，变换坐标系了
// 		double rmin = vehicle_parameters.MIN_CIRCLE;
// 		double x = End[0] - wknode.x;
// 		double y = End[1] - wknode.y;
// 		double ph = End[2] - wknode.theta;
// 		double phi = mod2pi(wknode.theta);
// 		// 起点start坐标系在基坐标系下的方向余弦矩阵(Z轴旋转，因为点在Z = 0平面，因此绕Z旋转仍然在Z = 0平面)
// 		// dcm*x 表示将基坐标中的x表示到旋转后的坐标系中，即计算坐标旋转后各向量在新坐标中的表示
// 		double x1 = x*cos(phi) + y*sin(phi);
// 		double y1 = y*cos(phi) - x*sin(phi);
// 		//看是否从当前点到目标点存在无碰撞的Reeds - Shepp轨迹，前面pvec = End - Start; 的意义就在这里，注意！这里的x, y, prev(3)是把起点转换成以start为原点坐标系下的坐标
// 		RsPath path;

// 		if (fusion.parkingSpaceInfo.ParkingSpaceType == 2)
// 		{
// 			if ((Theta_se < 0.2793) && (Y_se > 1))
// 			{
// 				ErrorCode = FindRSPathCCS(x1, y1, ph, path);
// 			}
// 			else
// 			{
// 				path = FindRSPathCCC(x1, y1, ph);
// 			}
// 		}
// 		else
// 		{
// 			if (SEorES == 0)
// 			{
// 				ErrorCode = FindRSPathCCS(x1, y1, ph, path);
// 			}
// 			else
// 			{
// 				path = FindRSPathCCC(x1, y1, ph);
// 			}
// 		}

// 		if (((ErrorCode == 2) && (SEorES == 0))||(path.lenth > 999))
// 		{
// 			path.lenth = 0;
// 		}
// 		else
// 		{
// 			//给全局变量赋初值
// 			g_px = wknode.x;
// 			g_py = wknode.y;
// 			g_pth = wknode.theta;

// 			// 以下是根据路径点和车辆运动学模型计算位置，检测是否会产生碰撞，返回isok的值。对每段路径从起点到终点按顺序进行处理，这一个线段的终点pvec是下一个线段的起点px, py, pth，

// 			double segs[3] = { rmin * path.t, rmin * path.u, rmin * path.v };
// 			int i = 0;
// 			double D = 0;
// 			double delta = 0;
// 			int idx = 0;
// 			for (i = 0; i < 3; i++)
// 			{
// 				if (segs[i] == 0)
// 				{
// 					continue;
// 				}

// 				D = (segs[i] > 0) ? pathfind_parameters.MOTION_RESOLUTION : -pathfind_parameters.MOTION_RESOLUTION;

// 				if (path.rspath_type[i] == St)
// 				{
// 					delta = 0;
// 				}

// 				else if (path.rspath_type[i] == Le)
// 				{
// 					delta = vehicle_parameters.MAX_STEER;
// 				}

// 				else if (path.rspath_type[i] == Ri)
// 				{
// 					delta = -vehicle_parameters.MAX_STEER;
// 				}

// 				// 把此段的路径离散成为路点，即栅格索引, 然后为路点，然后检测是否存在障碍物碰撞问题
// 				for (idx = 0; idx < round(fabs(segs[i]) / pathfind_parameters.MOTION_RESOLUTION); idx++) // round()四舍五入
// 					// D和delta是固定，说明转弯的时候是按固定半径的圆转弯
// 				{
// 					VehicleDynamic(g_px, g_py, g_pth, D, delta); // 根据当前位姿和输入, 计算下一位置的位姿
// 					isCollision = VehicleCollisionGrid_Astar(g_px, g_py, g_pth);
// 					if (isCollision)
// 					{
// 						path.lenth = 0;
// 						break;
// 					}
// 				}
// 				if (isCollision)
// 				{
// 					path.lenth = 0;
// 					break;
// 				}
// 			}
// 			// 终点位姿小于期望阈值也舍弃
// 			if (fabs(mod2pi(g_pth) - End[2]) > pi / 180 * 3)
// 			{
// 				path.lenth = 0;
// 			}
// 		}
// 		return path;
// 	}

// 	void getFinalPath(RsPath path, std::string stridx)
// 	{
// 		int i = 1, idx = 0;//临时循环变量，i=1是确保第一个循环可以走下去

// 		double distance=0;
// 		double length = vehicle_parameters.LB + vehicle_parameters.LF + 0.4;
// 		double rect_x = length / 2 - (vehicle_parameters.LB + 0.2); // 车辆中心坐标

// 		PathPoint mid;
// 		Node wknode = g_closeset[stridx]; // RS曲线中最后一个元素是目标点

// 		std::vector<Node> nodes;
// 		nodes.push_back(wknode);
// 		double rmin = vehicle_parameters.MIN_CIRCLE;
// 		// 找目标点wknode的parent, 回溯，直到Close集合为空
// 		while (wknode.xidx != wknode.parent[0] || wknode.yidx != wknode.parent[1] || wknode.yawidx != wknode.parent[2]) // 如果父节点不是自己才继续查找
// 		{
// 			stridx = get_par_str_idx(wknode);
// 			wknode = g_closeset[stridx];
// 			nodes.push_back(wknode);
// 			g_closeset.erase(stridx);
// 		}

// 		mid.x = nodes[nodes.size() - 1].x;
// 		mid.y = nodes[nodes.size() - 1].y;
// 		mid.th = nodes[nodes.size() - 1].theta;
// 		mid.D = nodes[nodes.size() - 1].D;
// 		mid.delta = nodes[nodes.size() - 1].delta;
// 		pathpoint.push_back(mid);//先把起点的坐标放到路径中

// 		//char nlist = 3;

// 		// 路径要么是纯RS路径，要么是由RS路径和混合A*组合一起来的路径，先处理混合A*的结点，最后处理RS路径，肯定有RS路径
// 		if (nodes.size() >= 2)
// 		{
// 			for (i = nodes.size() - 1; i >= 1; i--)
// 			{
// 				for (idx = 0; idx < 3; idx++)
// 				{
// 					VehicleDynamic(mid.x, mid.y, mid.th, nodes[i - 1].D, nodes[i - 1].delta);
// 					mid.x = g_px;
// 					mid.y = g_py;
// 					mid.th = g_pth;
// 					mid.D = nodes[i - 1].D;
// 					mid.delta = nodes[i - 1].delta;
// 					pathpoint.push_back(mid);

// 					// ------------------ safe ----------------
// 					double center_x1 = rect_x * cos(mid.th) + mid.x;	// 圆心坐标
// 					double center_y1 = rect_x * sin(mid.th) + mid.y;
// 					int xidx1 = ceil((pathfind_parameters.MAXX - center_x1) / pathfind_parameters.XY_GRID_RESOLUTION);
// 					int yidx1 = ceil((pathfind_parameters.MAXY - center_y1) / pathfind_parameters.XY_GRID_RESOLUTION);
// 					double dis = dis_map[(xidx1 - 1)][(yidx1 - 1)] * pathfind_parameters.XY_GRID_RESOLUTION;
					
// 					distance += dis;
// 				}
// 			}
// 		}

// 		double segs[3] = { rmin*path.t, rmin*path.u, rmin*path.v };
// 		double D = 0;
// 		double delta = 0;
// 		for (i = 0; i < 3; i++)
// 		{
// 			if (segs[i] == 0)
// 			{
// 				continue;
// 			}

// 			D = (segs[i]>0) ? pathfind_parameters.MOTION_RESOLUTION : -pathfind_parameters.MOTION_RESOLUTION;

// 			if (path.rspath_type[i] == St)
// 			{
// 				delta = 0;
// 			}

// 			else if (path.rspath_type[i] == Le)
// 			{
// 				delta = vehicle_parameters.MAX_STEER;
// 			}

// 			else if (path.rspath_type[i] == Ri)
// 			{
// 				delta = -vehicle_parameters.MAX_STEER;
// 			}

// 			// 把此段的路径离散成为路点，即栅格索引, 然后为路点，然后检测是否存在障碍物碰撞问题
// 			for (idx = 0; idx < round(fabs(segs[i]) / pathfind_parameters.MOTION_RESOLUTION); idx++) // round()四舍五入
// 				// D和delta是固定，说明转弯的时候是按固定半径的圆转弯
// 			{
// 				VehicleDynamic(mid.x, mid.y, mid.th, D, delta); // 根据当前位姿和输入, 计算下一位置的位姿
// 				mid.x = g_px;
// 				mid.y = g_py;
// 				mid.th = g_pth;
// 				mid.D = D;
// 				mid.delta = delta;
// 				pathpoint.push_back(mid);

// 				// ------------------ safe ----------------
// 				double center_x1 = rect_x * cos(mid.th) + mid.x;	// 圆心坐标
// 				double center_y1 = rect_x * sin(mid.th) + mid.y;
// 				int xidx1 = ceil((pathfind_parameters.MAXX - center_x1) / pathfind_parameters.XY_GRID_RESOLUTION);
// 				int yidx1 = ceil((pathfind_parameters.MAXY - center_y1) / pathfind_parameters.XY_GRID_RESOLUTION);
// 				double dis = dis_map[(xidx1 - 1)][(yidx1 - 1)] * pathfind_parameters.XY_GRID_RESOLUTION;

// 				distance += dis;
// 			}
// 		}
// 		avg_dis=distance / pathpoint.size();

// 		// 重复的路径删掉
// 		int point_size = pathpoint.size();
// 		for (int point_index_before = 0; point_index_before < point_size - 1; point_index_before++)
// 		{
// 			for (int point_index_end = point_index_before + 1; point_index_end < point_size;point_index_end++)
// 			{
// 				if ((fabs(pathpoint[point_index_before].x - pathpoint[point_index_end].x) < 0.001) && (fabs(pathpoint[point_index_before].y - pathpoint[point_index_end].y) < 0.001) && (fabs(pathpoint[point_index_before].th - pathpoint[point_index_end].th)* 57.295779513082323 < 0.1))
// 				{
// 					pathpoint.erase(pathpoint.begin() + point_index_before + 1, pathpoint.begin() + point_index_end);
// 					point_index_before = point_size;
// 					point_index_end = point_size;
// 				}
// 			}
// 		}
// 		point_size = 0;
// 	}

// 	std::string get_str_idx(Node &node)
// 	{
// 		std::string str = "xy_yaw";
// 		str += std::to_string(node.xidx);
// 		str += "/";
// 		str += std::to_string(node.yidx);
// 		str += "/";
// 		str += std::to_string(node.yawidx);
// 		return str;
// 	}

// 	std::string get_par_str_idx(Node &node)
// 	{
// 		std::string str = "xy_yaw";
// 		str += std::to_string(node.parent[0]);
// 		str += "/";
// 		str += std::to_string(node.parent[1]);
// 		str += "/";
// 		str += std::to_string(node.parent[2]);
// 		return str;
// 	}

// 	bool HybridAStar(double start[3], double end[3],const double &time_max)
// 	{	
// 		FLAG_collison_check_model = 1;
// 		Node tnode;		// 函数临时变量
// 		RsPath path;    // 函数临时变量
// 		g_openset.clear();    // open 集合清空
// 		g_closeset.clear();   // close 集合清空
// 		pathpoint.clear(); // 路 集合清空
// 		while (!pq.empty())
// 			pq.pop();
// 		Theta_se = fabs(start[2] - end[2]);
// 		Y_se = fabs(start[1] - end[1]);

// 		if (CalcIdx(start[0], start[1], start[2]))
// 		{
// 			int mid[3] = { g_xidx, g_yidx, g_thidx };// 函数临时变量
// 			tnode = Node(g_xidx, g_yidx, g_thidx, 0, 0, start[0], start[1], start[2], mid, 0, 0);
// 			tnode.fcost = TotalCost(tnode, end); // 起始点f
			
// 			str_idx = get_str_idx(tnode); // hybrid A*的Open集合
// 			pq.push({str_idx, tnode.fcost});
// 			g_openset[str_idx]=tnode;

// 			bool isCollision = CollisionGrid_InitPos_test(tnode.x, tnode.y, tnode.theta);
// 			if (isCollision)
// 			{
// 				std::cout << "not use circle" << std::endl;
// 				use_circle_flag = 0;
// 			}
// 			else
// 			{
// 				std::cout << "use circle" << std::endl;
// 				use_circle_flag = 1;
// 			}

// 			CloseSizeMax = 0;
// 			bydapa::common::TicToc tictoc;
// 			tictoc.tic();
			
// 			while (!g_openset.empty())
// 			{
// 				nseconds = tictoc.toc();
// 				nseconds = nseconds * 1.0e-9;
// 				if (nseconds > time_max)
// 				{
// 					ErrorCode = 3;
// 					FLAG_collison_check_model = 0;
// 					return false;
// 				}
// 				while (1)
// 				{
// 					str_idx = pq.top().first; // 选出代价最小的OPEN节点
// 					if (g_openset.find(str_idx) == g_openset.end())
// 						pq.pop();
// 					else
// 						break;	
// 				}
				
// 				g_tnode = g_openset[str_idx];
// 				pq.pop();
// 				g_openset.erase(str_idx);
// 				g_closeset[str_idx]=g_tnode;
// 				// expanded_points.push_back(g_tnode);
				
// 				path = AnalysticExpantion(g_tnode, end); // 代价最小的OPEN节点的RS是否发生碰撞，如果碰撞则下一点，不碰撞则搜路结束
// 				if (path.lenth)
// 				{
// 					getFinalPath(path, str_idx);
// 					FLAG_collison_check_model = 0;
// 					return true; 
// 				}

// 				Update(g_tnode, end);

// 				if (CloseSizeMax < (int)g_closeset.size())
// 				{
// 					CloseSizeMax = (int)g_closeset.size();
// 				}
// 			}
// 		}
// 		else
// 		{
// 			ErrorCode = 1;
// 		}
// 		g_openset.clear();	 // open集合清空
// 		g_closeset.clear(); // close 集合清空
// 		FLAG_collison_check_model = 0;
// 		return false;	 // 搜不到路
// 	}

// 	//////////////////////////RS函数定义////////////////////////////////////////////////////////////////////////
// 	RsPath FindRSPathCCC(double x, double y, double phi)
// 	{
// 		RsPath path;
// 		double rmin = vehicle_parameters.MIN_CIRCLE;
// 		path.lenth = 1000;
// 		x = x / rmin;
// 		y = y / rmin;
// 		//CSC(x, y, phi, path);
// 		CCC(x, y, phi, path);
// 		//CCS(x, y, phi, path);

// 		return path;
// 	}

// 	int FindRSPathCCS(double x, double y, double phi, RsPath& path)
// 	{
// 	//	RsPath path;
// 		double rmin = vehicle_parameters.MIN_CIRCLE;
// 		path.lenth = 1000;
// 		x = x / rmin;
// 		y = y / rmin;
// 		ErrorCode = CCS(x, y, phi, path);
// 		if (ErrorCode == 0)
// 		{
// 			return 0;
// 		}
// 		else
// 		{
// 			return 2;
// 		}
// 	}

// 	RsPath FindRSPathCSC(double x, double y, double phi)
// 	{
// 		RsPath path;
// 		double rmin = vehicle_parameters.MIN_CIRCLE;
// 		path.lenth = 1000;
// 		x = x / rmin;
// 		y = y / rmin;
// 		CSC(x, y, phi, path);

// 		return path;
// 	}
// }
