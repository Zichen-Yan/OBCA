#include "../common/global_variable.h"
#include "../common/global_function.h"

namespace byd_apa_plan
{
	bool VehicleCollisionGrid_Astar(double cpx, double cpy, double cph)
	{   
		double phi = mod2pi(cph);
		double cosphi = cos(phi);
		double sinphi = sin(phi);

		std::pair<double,double> p0={fusion.parkingSpaceInfo.P0_X/100.0,fusion.parkingSpaceInfo.P0_Y/100.0}; 
		std::pair<double,double> p1={fusion.parkingSpaceInfo.P2_X/100.0,fusion.parkingSpaceInfo.P2_Y/100.0};
		int in_parking_lot;
		if (((p0.first<=cpx && cpx<=p1.first) || (p1.first<=cpx && cpx<=p0.first)) && ((p0.second<=cpy && cpy<=p1.second) || (p1.second<=cpy && cpy<=p0.second)))
			in_parking_lot=1;
		else
			in_parking_lot=0;

		if (in_parking_lot)
		{	
			double rect_x = .0;
			double rect_y = .0;
			double rect_index = Rect_x10.size();
			if ((fusion.parkingSpaceInfo.ParkingSpaceType == 2) || (no_imag_map == 1) || (Collision_flag == 0))
			{
				rect_index = Rect_x.size();
			}
			for (int i = 0; i < rect_index; i++)
			{
				if (rect_index != Rect_x.size())
				{
					rect_x = Rect_x10[i] * cosphi - Rect_y10[i] * sinphi + cpx;
					rect_y = Rect_y10[i] * cosphi + Rect_x10[i] * sinphi + cpy;
				}
				else
				{
					rect_x = Rect_x[i] * cosphi - Rect_y[i] * sinphi + cpx;
					rect_y = Rect_y[i] * cosphi + Rect_x[i] * sinphi + cpy;
				}
				int xidx = ceil((pathfind_parameters.MAXX - rect_x) / pathfind_parameters.XY_GRID_RESOLUTION);
				int yidx = ceil((pathfind_parameters.MAXY - rect_y) / pathfind_parameters.XY_GRID_RESOLUTION);
				if (xidx == 0)
				{
					xidx = 1;
				}
				if (yidx == 0)
				{
					yidx = 1;
				}
				int idx_xy = (xidx - 1) * pathfind_parameters.XIDX + (yidx - 1);
				if ((xidx <= 0 || xidx > pathfind_parameters.XIDX) || (yidx <= 0 || yidx > pathfind_parameters.XIDY))
				{
					return true;
				}
				else 
				{
					if (rect_index != Rect_x.size())
					{
						if ((obstmap[idx_xy].Status == 0) || (obstmap[idx_xy].Status == 3) || (obstmap[idx_xy].Status == 4) || (obstmap[idx_xy].Status == 8) || (obstmap[idx_xy].Status == 9))
						{
							return true;
						}
					}
					else
					{
						if ((obstmap[idx_xy].Status == 0) || (obstmap[idx_xy].Status == 4) || (obstmap[idx_xy].Status == 8))
						{
							return true;
						}
					}
				}
			}
		}
		else
		{
			double length=vehicle_parameters.LB+vehicle_parameters.LF+0.4;
			double wid = vehicle_parameters.W+0.4;
			int result = VehicleCollisionTreeSearch3(cpx, cpy, cph, cosphi, sinphi, length, wid);
			if (result)
			{
				if(VehicleCollisionTreeSearch8(cpx, cpy, cph, result, cosphi, sinphi, length, wid))
					return true;
			}
		}
		return false;
	}

	int VehicleCollisionTreeSearch3(double cpx, double cpy, double cph, double cosphi, double sinphi, double length, double wid)
	{
		int result = 0; // 0无碰撞，1前车碰撞，2后车碰撞，3前后都碰撞
		double offset; //圆心间距离
		double radius; //圆半径

		offset=length/3;
		radius = sqrt(pow(wid/2,2)+pow(length/6,2))-0.1; 
		
		double rect_x=length/2-(vehicle_parameters.LB+0.2); //车辆中心坐标

		double center_x1= (rect_x+offset-0.25) * cosphi + cpx; //圆心坐标
		double center_y1= (rect_x+offset-0.25) * sinphi + cpy;

		double center_x2 = (rect_x) * cosphi + cpx;
		double center_y2 = (rect_x) * sinphi + cpy;

		double center_x3 = (rect_x-offset+0.25) * cosphi + cpx;
		double center_y3 = (rect_x-offset+0.25) * sinphi + cpy;

		double center_x;
		double center_y; 

		center_x=center_x1;
		center_y=center_y1;
		// 栅格化
		int xidx = ceil((pathfind_parameters.MAXX - center_x) / pathfind_parameters.XY_GRID_RESOLUTION);
		int yidx = ceil((pathfind_parameters.MAXY - center_y) / pathfind_parameters.XY_GRID_RESOLUTION);
		if (xidx == 0){
			xidx = 1;
		}
		if (yidx == 0){
			yidx = 1;
		}
		if ((xidx <= 0 || xidx > pathfind_parameters.XIDX) || (yidx <= 0 || yidx > pathfind_parameters.XIDY))
		{
			result = 1;
		}
		else {
			double dis = dis_map[(yidx-1)][249-(xidx-1)]*pathfind_parameters.XY_GRID_RESOLUTION;
			if (dis<=radius)
			{
				result = 1;
			}
		}

		center_x=center_x2;
		center_y=center_y2;
		// 栅格化
		xidx = ceil((pathfind_parameters.MAXX - center_x) / pathfind_parameters.XY_GRID_RESOLUTION);
		yidx = ceil((pathfind_parameters.MAXY - center_y) / pathfind_parameters.XY_GRID_RESOLUTION);
		if (xidx == 0){
			xidx = 1;
		}
		if (yidx == 0){
			yidx = 1;
		}
		if ((xidx <= 0 || xidx > pathfind_parameters.XIDX) || (yidx <= 0 || yidx > pathfind_parameters.XIDY))
		{
			return 3;
		}
		else {
			double dis = dis_map[(yidx-1)][249-(xidx-1)]*pathfind_parameters.XY_GRID_RESOLUTION;
			if (dis<=radius)
			{
				return 3;
			}
		}

		center_x=center_x3;
		center_y=center_y3;
		// 栅格化
		xidx = ceil((pathfind_parameters.MAXX - center_x) / pathfind_parameters.XY_GRID_RESOLUTION);
		yidx = ceil((pathfind_parameters.MAXY - center_y) / pathfind_parameters.XY_GRID_RESOLUTION);
		if (xidx == 0){
			xidx = 1;
		}
		if (yidx == 0){
			yidx = 1;
		}
		if ((xidx <= 0 || xidx > pathfind_parameters.XIDX) || (yidx <= 0 || yidx > pathfind_parameters.XIDY))
		{
			if (result==1)
				return 3;
			else
				result=2;
		}
		else {
			double dis = dis_map[(yidx-1)][249-(xidx-1)]*pathfind_parameters.XY_GRID_RESOLUTION;
			if (dis<=radius)
			{
				if (result==1)
					return 3;
				else
					result=2;
			}
		}
		return result;
	}

	bool VehicleCollisionTreeSearch8(double cpx, double cpy, double cph, int result, double cosphi, double sinphi, double length, double wid)
	{
		double offset; //圆心间距离
		double radius; //圆半径

		offset=length/8;
		radius = sqrt(pow(wid/4,2)+pow(length/8,2)); 
		
		double center_x1; //圆心坐标
		double center_y1;
		double center_x2;
		double center_y2;
		double center_x3; 
		double center_y3;
		double center_x4; 
		double center_y4;

		double rect_x; //车辆中心坐标
		double rect_y;
		std::vector<std::pair<double,double>> circle_list1;
		std::vector<std::pair<double,double>> circle_list2;
		std::vector<double> radius_list1;
		std::vector<double> radius_list2;
		
		rect_x=length/2-(vehicle_parameters.LB+0.2);
		if (result==1 || result==3)
		{
			rect_y=wid/4;

			center_x1 = (rect_x+3*offset-0.2) * cosphi - (rect_y-0.28) * sinphi + cpx;
			center_y1 = (rect_x+3*offset-0.2) * sinphi + (rect_y-0.28) * cosphi + cpy;

			center_x2 = (rect_x+offset) * cosphi - (rect_y-0.15) * sinphi + cpx;
			center_y2 = (rect_x+offset) * sinphi + (rect_y-0.15) * cosphi + cpy;

			rect_y=-wid/4;

			center_x3 = (rect_x+3*offset-0.2) * cosphi - (rect_y+0.28) * sinphi + cpx;
			center_y3 = (rect_x+3*offset-0.2) * sinphi + (rect_y+0.28) * cosphi + cpy;

			center_x4 = (rect_x+offset) * cosphi - (rect_y+0.15) * sinphi + cpx;
			center_y4 = (rect_x+offset) * sinphi + (rect_y+0.15) * cosphi + cpy;

			circle_list1={{center_x1,center_y1},{center_x2,center_y2},{center_x3,center_y3},{center_x4,center_y4}};
			radius_list1 = {radius+0.15,radius,radius+0.15,radius};
		}
		if (result==2 || result==3)
		{
			rect_y=wid/4;

			center_x1 = (rect_x-offset) * cosphi - (rect_y-0.15) * sinphi + cpx;
			center_y1 = (rect_x-offset) * sinphi + (rect_y-0.15) * cosphi + cpy;

			center_x2 = (rect_x-3*offset+0.2) * cosphi - (rect_y-0.28) * sinphi + cpx;
			center_y2 = (rect_x-3*offset+0.2) * sinphi + (rect_y-0.28) * cosphi + cpy;

			rect_y=-wid/4;

			center_x3 = (rect_x-offset) * cosphi - (rect_y+0.15) * sinphi + cpx;
			center_y3 = (rect_x-offset) * sinphi + (rect_y+0.15) * cosphi + cpy;

			center_x4 = (rect_x-3*offset+0.2) * cosphi - (rect_y+0.28) * sinphi + cpx;
			center_y4 = (rect_x-3*offset+0.2) * sinphi + (rect_y+0.28) * cosphi + cpy;

			circle_list2={{center_x1,center_y1},{center_x2,center_y2},{center_x3,center_y3},{center_x4,center_y4}};
			radius_list2 = {radius,radius+0.15,radius,radius+0.15};
		}
		circle_list1.insert(circle_list1.end(), circle_list2.begin(), circle_list2.end());
		radius_list1.insert(radius_list1.end(), radius_list2.begin(), radius_list2.end());
		
		size_t idx = 0;
		double center_x;
		double center_y;
		for (const auto& circle : circle_list1) {
			center_x=circle.first;
			center_y=circle.second;
			// 栅格化
			int xidx = ceil((pathfind_parameters.MAXX - center_x) / pathfind_parameters.XY_GRID_RESOLUTION);
			int yidx = ceil((pathfind_parameters.MAXY - center_y) / pathfind_parameters.XY_GRID_RESOLUTION);
			if (xidx == 0){
				xidx = 1;
			}
			if (yidx == 0){
				yidx = 1;
			}
			if ((xidx <= 0 || xidx > pathfind_parameters.XIDX) || (yidx <= 0 || yidx > pathfind_parameters.XIDY))
			{
				return true;
			}
			else {
				double dis = dis_map[(yidx-1)][249-(xidx-1)]*pathfind_parameters.XY_GRID_RESOLUTION;
				double radius = radius_list1[idx++];
				if (dis<=radius)
				{
					return true;
				}
			}
		}
		return false;
	}
}