#include "../common/global_variable.h"
#include "../common/global_function.h"

namespace byd_apa_plan{

	bool CalcCriticalPosition(double* pos)
	{
		double r_lf = sqrt((rmin_vertical + vehicle_parameters.W/2 + W_exp/2)*(rmin_vertical + vehicle_parameters.W/2 + W_exp/2) 
							+ (vehicle_parameters.LF + L_exp/2)*(vehicle_parameters.LF + L_exp/2));       
		double x_c0 = .0, y_c0 = .0;
		double x_p = .0, y_p = .0;
		double x_pr = .0, y_pr = .0, phi_pr = .0;
		if(ParkingSpaceFlag == 1)
		{
			CoordinateTrans(x_pr,y_pr,phi_pr,P3x,P3y,0,end_to_fus[0],end_to_fus[1],end_to_fus[2]);
		}
		else{
			CoordinateTrans(x_pr,y_pr,phi_pr,P0x,P0y,0,end_to_fus[0],end_to_fus[1],end_to_fus[2]);
		}
		x_p = P_r[0];
		y_p = P_r[1];
		if(dist_P03 > 0 && park_theta != 0)
		{
			x_pr += -dist_P03 * 0.01 / sin(park_theta); // TODO: plus dist_to_P03
			x_p += -dist_P03 * 0.01 / sin(park_theta);
		}
		if(dist_P23 < 0 && false)  // TODO：park side limit > 2.1m 
		{
			//std::cout << "xxxxxxxxxxxxxxx\n";
			y_c0 = -ParkingSpaceFlag * rmin_vertical;
			x_c0 = -1.5;
		}
		else{
			std::cout << "depth_of_park = " << depth_of_park << std::endl;
			y_c0 = -ParkingSpaceFlag * rmin_vertical;
			x_c0 = -sqrt(r_lf * r_lf - (y_c0 - y_pr)*(y_c0 - y_pr)) + x_pr + depth_of_park;
		}
		if(fusion.parkingSpaceInfo.ParkingSpaceType == 1)
		{
			double x_c1 = x_pr - r_lf;
			double y_c1 = ParkingSpaceFlag * sqrt(4 * rmin_vertical *rmin_vertical - (x_c1 - x_c0) * (x_c1 - x_c0)) + y_c0;
			double theta = -ParkingSpaceFlag * 20*pi/180; // < 35度
			pos[0] = x_c1 + rmin_vertical * cos(theta);
			pos[1] = y_c1 + rmin_vertical * sin(theta);
			pos[2] = ParkingSpaceFlag * pi / 2 + theta;
			
			return !(std::isnan(pos[0]) || std::isnan(pos[1]) || std::isnan(pos[2]));
		}
		else if(fusion.parkingSpaceInfo.ParkingSpaceType == 3)
		{
			if ((x_pr - x_p) != 0)
			{
				double k = (y_pr - y_p) / (x_pr - x_p);
				double b_l = y_pr - k * x_pr;
				double a = 1 + k*k;
				double b = 2*(k*b_l + ParkingSpaceFlag * k*sqrt(1+k*k)*r_lf - k*y_c0 - x_c0);
				double c = x_c0*x_c0 + y_c0*y_c0 + b_l*b_l + (1 + k*k)*r_lf*r_lf + 2*(ParkingSpaceFlag * b_l*sqrt(1+k*k)*r_lf - ParkingSpaceFlag * y_c0*sqrt(1+k*k)*r_lf - b_l*y_c0) - 4*rmin_vertical*rmin_vertical;
				double delta = b*b - 4*a*c;
				if(delta >= 0)
				{
					double x_1 = (-b + sqrt(delta)) / (2*a);
					double x_2 = (-b - sqrt(delta)) / (2*a);
					double x_c1 = std::max(x_1,x_2);
					double y_c1 = k*x_c1 + b_l + ParkingSpaceFlag * sqrt(1 + k*k)*r_lf;
					double theta = atan(k);
					pos[0] = x_c1 + ParkingSpaceFlag * rmin_vertical * sin(theta);
					pos[1] = y_c1 - ParkingSpaceFlag * rmin_vertical * cos(theta);
					pos[2] = theta;
					return !(std::isnan(pos[0]) || std::isnan(pos[1]) || std::isnan(pos[2]));
				}
			}
		}
		return false;
	}

	int CalcCmCpSpMulti(const double *start, const double *end)
	{
		double x = .0, y = .0, phi = .0;
		CoordinateTrans(x,y,phi,end[0],end[1],end[2],start[0],start[1],start[2]);
		double curr_rmin = rmin_vertical * 1.0;
		x = x / curr_rmin;
		y = y / curr_rmin;
		RsPath rs_path;
		double road_yaw[6] = {.0};
		int parking_mode = (fusion.ParkInMode == 0) ? 1 : -1;
		
		if(CpCmSm(x,y,phi,rs_path,parking_mode))
		{
			for (int idx = 0;idx < 3;idx++)
			{
				if (rs_path.rspath_type[idx] == St)
				{
					road_yaw[idx * 2] = 1000000;
				}

				else if (rs_path.rspath_type[idx] == Le)
				{
					road_yaw[idx * 2] = curr_rmin;
				}

				else if (rs_path.rspath_type[idx] == Ri)
				{
					road_yaw[idx * 2] = -curr_rmin;
				}
			}

			road_yaw[1] = (rs_path.t) * curr_rmin;
			road_yaw[3] = (rs_path.u) * curr_rmin;
			road_yaw[5] = (rs_path.v) * curr_rmin;
			rs_path.lenth = rs_path.lenth * curr_rmin;
			return GetOrCheckPathPointsMulti(start, road_yaw, 1);
		}
		return 0;
	}

	int CalcCpCmSpMulti(const double *start,const double* end)
	{
		double x = .0, y = .0, phi = .0;
		CoordinateTrans(x,y,phi,end[0],end[1],end[2],start[0],start[1],start[2]);
		double curr_rmin = rmin_vertical * 1.0;
		x = x / curr_rmin;
		y = y / curr_rmin;
		RsPath rs_path;
		double road_yaw[6] = {.0};
		int parking_mode = (fusion.ParkInMode == 0) ? 1 : -1;
		
		if(CmCpSm(x,y,phi,rs_path,parking_mode))
		{
			for (int idx = 0;idx < 3;idx++)
			{
				if (rs_path.rspath_type[idx] == St)
				{
					road_yaw[idx * 2] = 1000000;
				}

				else if (rs_path.rspath_type[idx] == Le)
				{
					road_yaw[idx * 2] = curr_rmin;
				}

				else if (rs_path.rspath_type[idx] == Ri)
				{
					road_yaw[idx * 2] = -curr_rmin;
				}
			}

			road_yaw[1] = (rs_path.t) * curr_rmin;
			road_yaw[3] = (rs_path.u) * curr_rmin;
			road_yaw[5] = (rs_path.v) * curr_rmin;
			rs_path.lenth = rs_path.lenth * curr_rmin;
			return GetOrCheckPathPointsMulti(start, road_yaw, 1);
		}
		return 0;
	}

	bool SpCmSp(const double x, const double y, const double phi, RsPath &path_)
	{
		double phi_mod = mod2pi(phi);
		double xd = 0.0;
		double u = 0.0;
		double t = 0.0;
		double v = 0.0;
		double epsilon = 1e-1;
		if (y > 0.0 && (phi_mod) > epsilon && (phi_mod) < pi)
		{
			xd = -y / tan(phi_mod) + x;
			t = xd + tan(phi_mod / 2.0);
			u = -phi_mod;
			v = sqrt((x - xd) * (x - xd) + y * y) + tan(phi_mod / 2.0);
			path_.u = u;
			path_.t = t;
			path_.v = v;
			path_.lenth = fabs(t) + fabs(u) + fabs(v);
			path_.rspath_type[0] = St;
			path_.rspath_type[1] = Ri;
			path_.rspath_type[2] = St;
			return (t >= 0 && u <= 0 && v >= 0); // 0.0862
		}
		else if (y < 0.0 && fabs(phi_mod) > epsilon && fabs(phi_mod) < pi)
		{
			xd = -y / tan(phi_mod) + x;
			t = xd - tan(phi_mod / 2.0);
			u = phi_mod;
			v = sqrt((x - xd) * (x - xd) + y * y) - tan(phi_mod / 2.0);
			path_.u = u;
			path_.t = t;
			path_.v = v;
			path_.lenth = fabs(t) + fabs(u) + fabs(v);
			path_.rspath_type[0] = St;
			path_.rspath_type[1] = Le;
			path_.rspath_type[2] = St;
			return (t >= 0 && u <= 0 && v >= 0); // 0.0862
		}
		return false;
	}

	int CalcSpCmSp(double *start,double *end)
	{
		double x = .0, y = .0, phi = .0;
		CoordinateTrans(x,y,phi,end[0],end[1],end[2],start[0],start[1],start[2]);
		double curr_rmin = rmin_vertical * 1.0;
		x = x / curr_rmin;
		y = y / curr_rmin;
		RsPath rs_path;
		double road_yaw[6] = {.0};
		//int parking_mode = (fusion.ParkInMode == 0) ? 1 : -1;
		if(SpCmSp(x,y,phi,rs_path))
		{
			for (int idx = 0;idx < 3;idx++)
			{
				if (rs_path.rspath_type[idx] == St)
				{
					road_yaw[idx * 2] = 1000000;
				}

				else if (rs_path.rspath_type[idx] == Le)
				{
					road_yaw[idx * 2] = curr_rmin;
				}

				else if (rs_path.rspath_type[idx] == Ri)
				{
					road_yaw[idx * 2] = -curr_rmin;
				}
			}

			road_yaw[1] = (rs_path.t) * curr_rmin;
			road_yaw[3] = (rs_path.u) * curr_rmin;
			road_yaw[5] = (rs_path.v) * curr_rmin;
			rs_path.lenth = rs_path.lenth * curr_rmin;
			return GetOrCheckPathPointsMulti(start, road_yaw, 1);
		}
		return 0;
	}

	bool CalcCmSimplify(double* start, double* critical_point, const double &critical_r, const double &curve_theta)
	{
		std::vector<std::pair<double, double>> circle_critical = CalcCentreOfCircle(critical_point[0], critical_point[1], critical_point[2], critical_r);
		double road_yaw[6] = {.0};
		double x_0 = start[0], y_0 = start[1], theta = start[2];
		double delta_r = 0.0;
		double x_1 = critical_point[0]  + ParkingSpaceFlag * critical_r * sin(critical_point[2]);
		double y_1 = critical_point[1]  - ParkingSpaceFlag * critical_r * cos(critical_point[2]);
		double a = x_0 * x_0 + y_0 * y_0 + x_1 * x_1 + y_1 * y_1 - 2 * x_0 * x_1 - 2 * y_0 * y_1 + 2 * ParkingSpaceFlag * y_0 * delta_r - 2 * ParkingSpaceFlag * y_1 * delta_r - critical_r * critical_r - 2 * critical_r * delta_r;
		double b = 2 * (critical_r + delta_r - ParkingSpaceFlag * x_1 * sin(theta) + ParkingSpaceFlag * x_0 * sin(theta) - ParkingSpaceFlag * y_0 * cos(theta) + ParkingSpaceFlag * y_1 * cos(theta) - delta_r * cos(theta));
		double r_ = 0;
		if (b != 0)
		{
			r_ = a / b;
		}
		std::cout << "***************" << std::endl;
		std::cout << "r_ = " << r_ << std::endl;
		if (fabs(r_) >= rmin_vertical)
		{
			double d_x = x_0 - ParkingSpaceFlag * r_ * sin(theta) - x_1;
			double d_y = y_0 + ParkingSpaceFlag * r_ * cos(theta) - y_1 + ParkingSpaceFlag * delta_r;
			double t = .0, u = .0;
			u = atan(fabs(d_x / d_y));
			t = u - ParkingSpaceFlag * theta;
			u = u - ParkingSpaceFlag * curve_theta;
			std::cout << "curve_theta = " << curve_theta << std::endl;
			std::cout << "t = " << t << " u = " << u << std::endl;
			if (t <= 0 && u >= 0 && r_ * t <= 0)
			{
				road_yaw[0] = ParkingSpaceFlag * r_;
				road_yaw[1] = r_ * t;
				road_yaw[2] = -ParkingSpaceFlag * (critical_r + delta_r);
				road_yaw[3] = (critical_r + delta_r) * u;
				road_yaw[4] = 0;
				road_yaw[5] = 0;
				
				if (GetOrCheckPathPoints(start, road_yaw, 3, 1))
				{
					pathpoint.clear();
					return GetOrCheckPathPoints(start, road_yaw, 3, 0);
				}
			}
		}
		return false;

	}

	bool SmCm(const double &x, const double &y, const double &phi, const double &curr_min,double* road_yaw)
	{
		if(phi != 0 && fabs(phi) <= pi/2 && y * phi < 0)
		{

			double r_cs = (fabs(y) / (1 - cos(phi)) > 1000000) ? 1000000 : (y/ (1 - cos(phi)));
			std::cout << "r_cs = " << r_cs << std::endl;
			if(fabs(r_cs) >= rmin_vertical)
			{
				double x_cs = (x - r_cs * sin(phi));
				std::cout << "x_cs = " << x_cs << std::endl;
				if(fabs(r_cs < 1000000) && x_cs <= 0)
				{
					road_yaw[2] = 1000000;
					road_yaw[3] = x_cs;
					road_yaw[4] = r_cs;
					road_yaw[5] = r_cs * phi;
					return true; 
				}
			}
		}
		return false;
	}

	bool CalcSmCmSimplify(double *start, double* start_p, double* end, const double &r_first, const double &theta_first)
	{
		double x = .0, y = .0, phi = .0;
		CoordinateTrans(x,y,phi,end[0],end[1],end[2],start_p[0],start_p[1],start_p[2]);
		double curr_rmin = rmin_vertical * 1.0;
		double road_yaw[6] = {.0};
		road_yaw[0] = r_first;
		road_yaw[1] = fabs(r_first)*theta_first;
		if(SmCm(x,y,phi,curr_rmin,road_yaw))
		{
			if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) // 碰撞检测
			{
				//return true;
				pathpoint.clear();
				return GetOrCheckPathPoints(start, road_yaw, 3, 0);
			}
		}
		return false;
	}

	bool SpCm(const double &x, const double &y, const double &phi, const double &curr_min,double* road_yaw)
	{
		if(phi != 0 && fabs(phi) <= pi/2 && y * phi < 0)
		{
			double r_cs = (fabs(y) / (1 - cos(phi)) > 1000000) ? 1000000 : (y/ (1 - cos(phi)));
			if(fabs(r_cs) >= rmin_vertical)
			{
				double x_cs = (x - r_cs * sin(phi));
				if(fabs(r_cs < 1000000) && x_cs >= 0)
				{
					road_yaw[2] = 1000000;
					road_yaw[3] = x_cs;
					road_yaw[4] = r_cs;
					road_yaw[5] = r_cs * phi;
					return true; 
				}
			}
		}
		return false;
	}

	bool CalcSpCmSimplify(double *start, double* end, const double &r_first, const double &theta_first)
	{
		double x = .0, y = .0, phi = .0;
		double road_yaw[8] = {.0};
		road_yaw[0] = -r_first;
		road_yaw[1] = fabs(r_first*theta_first);
		double start_p[3] = {start[0],start[1],start[2]};
		if(road_yaw[1] > 0)
		{
			int pp_size_before = pathpoint.size();
			if(GetOnePathPoints(start, -theta_first, -r_first, 1)) 
			{
				start_p[0] = pathpoint.back().x;
				start_p[1] = pathpoint.back().y;
				start_p[2] = pathpoint.back().th;
			}
			int pp_size_now = pathpoint.size();
			for(int i = 0; i < pp_size_now - pp_size_before; i++)
			{
				pathpoint.pop_back();
			}
		}
		CoordinateTrans(x,y,phi,end[0],end[1],end[2],start_p[0],start_p[1],start_p[2]);
		double curr_rmin = rmin_vertical * 1.0;
		if(SpCm(x,y,phi,curr_rmin,road_yaw))
		{
			double dist = .0;
			if(road_yaw[3] < 1.0)
			{
				dist = 1.2 - road_yaw[3];
				road_yaw[3] = 1.2;
				road_yaw[6] = road_yaw[4];
				road_yaw[7] = road_yaw[5];
				road_yaw[4] = 1000000;
				road_yaw[5] = -dist;
				
			}
			else
			{
				//ToDo:Increase the length of the line
				road_yaw[3] += 0.2;
				road_yaw[6] = road_yaw[4];
				road_yaw[7] = road_yaw[5];
				road_yaw[4] = 1000000;
				road_yaw[5] = -0.2;
			}
			if (GetOrCheckPathPoints(start, road_yaw, 4, 1)) // 碰撞检测
			{
				pathpoint.clear();
				PlanBackFirst -= 2;
				return GetOrCheckPathPoints(start, road_yaw, 4, 0);
			}
		}
		return false;
	}

	bool CalcCpCpRadius(const double &x, const double &y, const double &phi, double &min_r)
	{
		double a = pow(cos(phi) + 1, 2) + pow(sin(phi), 2) - 4;
		double b = 2 * x * sin(phi) - 2 * y * (cos(phi) + 1);
		double c = x * x + y * y;
		double delta = b * b - 4 * a * c;
		double r_1 = .0, r_2 = .0;
		if(!(delta < 0 || (a == 0 && b == 0)))
		{
			if(a == 0)
			{
				min_r = (fabs(c * 0.000001) >= fabs(b)) ? 1000000 : (-c/b);
				return true;
			}
			else{
				r_1 = (fabs((-b + pow(delta, 0.5)) * 0.000002) >= fabs(a)) ? 1000000 : ((-b + pow(delta, 0.5)) / 2 / a);
				r_2 = (fabs((-b - pow(delta, 0.5)) * 0.000002) >= fabs(a)) ? 1000000 : ((-b - pow(delta, 0.5)) / 2 / a);
				min_r = (fabs(r_1) < fabs(r_2)) ? r_1 : r_2;
				return true;
			}
		}
		return false;
	}

	bool CpCp(const double &x, const double &y, const double &phi, const double& curr_min,double* road_yaw)
	{
		double a = pow(cos(phi) + 1, 2) + pow(sin(phi), 2) - 4;
		double b = 2 * x * sin(phi) - 2 * y * (cos(phi) + 1);
		double c = x * x + y * y;
		double delta = b * b - 4 * a * c;
		double r_1 = .0, r_2 = .0, sin_res = .0, cos_res = .0;
		if(!(delta < 0 || (a == 0 && b == 0)))
		{
			if(a == 0)
			{
				r_1 = (fabs(c * 0.000001) >= fabs(b)) ? 1000000 : (-c/b);
				if(fabs(r_1) >= curr_min)
				{
					double angle_rad = .0;
					sin_res = x / 2 / r_1 + sin(phi) / 2;
					cos_res = -y / 2 / r_1 + cos(phi) / 2 + 0.5;
					if(!GetAntiTrigo(sin_res, cos_res, angle_rad) && (angle_rad * (phi - angle_rad)) <= 0)
					{
						road_yaw[0] = r_1;
						road_yaw[1] = r_1 * angle_rad;
						road_yaw[2] = -r_1;
						road_yaw[3] = -r_1 * (phi - angle_rad);
						return true;
					}
				}
			}
			else{
				r_1 = (fabs((-b + pow(delta, 0.5)) * 0.000002) >= fabs(a)) ? 1000000 : ((-b + pow(delta, 0.5)) / 2 / a);
				r_2 = (fabs((-b - pow(delta, 0.5)) * 0.000002) >= fabs(a)) ? 1000000 : ((-b - pow(delta, 0.5)) / 2 / a);
				double min_r = (fabs(r_1) < fabs(r_2)) ? r_1 : r_2;
				std::cout << "min_r = " << min_r << std::endl;
				if(fabs(min_r) >= curr_min)
				{
					double angle_rad = .0;
					sin_res = x / 2 / min_r + sin(phi) / 2;
					cos_res = -y / 2 / min_r + cos(phi) / 2 + 0.5;
					if(!GetAntiTrigo(sin_res, cos_res, angle_rad) && (angle_rad * (phi - angle_rad)) <= 0)
					{
						road_yaw[0] = min_r;
						road_yaw[1] = min_r * angle_rad;
						road_yaw[2] = -min_r;
						road_yaw[3] = -min_r * (phi - angle_rad);
						return true;
					}
				}
			}
		}
		return false;
	}

	bool CalcCpCp(const double* start, const double* end, const int &collision_flag)
	{
		double x = .0, y = .0, phi = .0;
		CoordinateTrans(x,y,phi,end[0],end[1],end[2],start[0],start[1],start[2]);
		double curr_rmin = rmin_vertical * 1.0;
		double road_yaw[6] = {.0};
		if(CpCp(x,y,phi,curr_rmin,road_yaw))
		{
			if(collision_flag == 0)
			{
				return GetOrCheckPathPointsMulti(start, road_yaw, 1);
			}
			else if(GetOrCheckPathPoints(start, road_yaw, 3, 1)) // 碰撞检测
			{
				//return true;
				return GetOrCheckPathPoints(start, road_yaw, 3, 0);
			}

			//
			
			std::cout << "fffffffff\n";
		}
		return false;
	}
	
	bool CalcSafeRadiusCpCp(const double* start, const double* end, const double &r_safe)
	{
		int flag = 0;
		double x = .0, y = .0, phi = .0, min_r = .0;
		CoordinateTrans(x,y,phi,end[0],end[1],end[2],start[0],start[1],start[2]);
		if(CalcCpCpRadius(x, y, phi, min_r) && fabs(min_r - .0) > 1e-5)
		{
			double t = .0, u = .0, sin_res = .0;
			double road_yaw[6] = {.0};
			std::cout << "min_r = " << min_r << std::endl;
			flag = min_r / fabs(min_r);
			std::cout << "flag = " << flag << std::endl;
			std::vector<std::pair<double, double>> center_points = CalcCentreOfCircle(start[0],start[1],start[2],r_safe);
			if(flag == -1)
			{
				sin_res = 0.5 * fabs(center_points[1].first) / r_safe;
				std::cout << "sin_res = " << sin_res << std::endl;
			}
			else if(flag == 1){
				sin_res = 0.5 * fabs(center_points[0].first) / r_safe;
				std::cout << "sin_res = " << sin_res << std::endl;
			}
			if(fabs(sin_res) - 1.0 < .0)
			{
				std::cout << "start[2] = " << start[2] << std::endl;
				u = asin(sin_res);
				std::cout << "u = " << u << std::endl;
				t = (u - flag*start[2]);
				std::cout << "t = " << t << std::endl;
				double x = r_safe * sin(flag*start[2] + t) + r_safe * sin(u) + center_points[0].first;
				std::cout << "x = " << x << std::endl;
				double d_y = r_safe * (cos(start[2]) - cos(t + flag*start[2]) - cos(u) + 1);
				std::cout << "start[1] = " << start[1] << std::endl;
				std::cout << "d_y = " << d_y << std::endl;
				double y_error = start[1] + flag * d_y;
				std::cout << "y = " << y << std::endl;
				if((fabs(y_error) - 0.15 < 1e-5) && (t >= 0))
				{
					road_yaw[0] = flag * r_safe;
					road_yaw[1] = r_safe * t;
					road_yaw[2] = -flag * r_safe;
					road_yaw[3] = r_safe * u;
					if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) // 碰撞检测
					{
						//return true;
						return GetOrCheckPathPoints(start, road_yaw, 3, 0);
					}
					std::cout << "xxxxxxxxxxx\n";
				}
			}
		}
		return false;
	}

	bool CmSp(const double &x, const double &y, const double &phi, const double &curr_min,double* road_yaw)
	{
		//std::cout << "y * phi = " << y * phi << std::endl;
		if(phi != 0 && fabs(phi) <= pi/2 && y * phi > 0)
		{
			double r_cs = (fabs(y) / (1 - cos(phi)) > 1000000) ? 1000000 : (y/ (1 - cos(phi)));
			//std::cout << "r_cs = " << r_cs << std::endl;
			if(fabs(r_cs) >= 5.5)
			{
				double x_cs = -(x - r_cs * sin(phi));
				//std::cout << "x_cs = " << x_cs << std::endl; 
				if(fabs(r_cs < 1000000) && x_cs >= 0)
				{
					road_yaw[0] = r_cs;
					road_yaw[1] = -r_cs * phi;
					road_yaw[2] = 1000000;
					road_yaw[3] = x_cs;
					return true; 
				}
			}
		}
		return false;
	}

	bool CalcCmSp(const double *start, const double* end)
	{
		double x = .0, y = .0, phi = .0;
		CoordinateTrans(x,y,phi,start[0],start[1],start[2],end[0],end[1],end[2]);
		double curr_rmin = rmin_vertical * 1.0;
		double road_yaw[6] = {.0};
		if(CmSp(x,y,phi,curr_rmin,road_yaw)) 
		{
			/*
			if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) // 碰撞检测
			{
				//return true;
				return GetOrCheckPathPoints(start, road_yaw, 3, 0);
			}
			*/
			std::cout << "road_yaw[3] = " << road_yaw[3] << std::endl;
			int flag = GetOrCheckPathPointsMulti(start,road_yaw,1);
			if(flag == 4 && (road_yaw[3] < 5.5)) // 2024.03.08 增加直线长度限制
			{
				return true;
			}
			else if(flag == 1)
			{
				double start_p[3] = {.0};
				start_p[0] = pathpoint.back().x;
				start_p[1] = pathpoint.back().y;
				start_p[2] = pathpoint.back().th;
				if(CalcCpCp(start_p,end, 1))
				{
					return true;
				}
			}
		}
		pathpoint.clear();
		return false;
	}

	bool CpSp(const double &x, const double &y, const double &phi, const double &curr_min,double* road_yaw)
	{
		if(phi != 0 && fabs(phi) <= pi/2 && y * phi < 0)
		{
			double r_cs = (fabs(y) / (1 - cos(phi)) > 1000000) ? 1000000 : (y/ (1 - cos(phi)));
			if(fabs(r_cs) >= curr_min)
			{
				double x_cs = (x - r_cs * sin(phi));
				if(fabs(r_cs < 1000000) && x_cs <= 0)
				{
					road_yaw[0] = r_cs;
					road_yaw[1] = -r_cs * phi;
					road_yaw[2] = 1000000;
					road_yaw[3] = -x_cs;
					return true; 
				}
			}
		}
		return false;
	}

	bool CalcCpSp(const double *start, const double* end)
	{
		double x = .0, y = .0, phi = .0;
		CoordinateTrans(x,y,phi,start[0],start[1],start[2],end[0],end[1],end[2]);
		double curr_rmin = rmin_vertical * 1.0;
		double road_yaw[6] = {.0};
		if(CpSp(x,y,phi,curr_rmin,road_yaw))
		{
			//return false;
			if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) // 碰撞检测
			{
				//return true;
				return GetOrCheckPathPoints(start, road_yaw, 3, 0);
			}
		}
		return false;
	}

	bool CalcSmCpCp(const double *start_p,double *end)
	{
		double x = .0, y = .0, phi = .0;
		double start[3] = {start_p[0],start_p[1],start_p[2]};
		CoordinateTrans(x,y,phi,start[0],start[1],start[2],end[0],end[1],end[2]);
		double curr_rmin = 6.0; // not vehicle min R
		x = x / curr_rmin;
		y = y / curr_rmin;
		RsPath rs_path;
		double road_yaw[6] = {.0};
		int parking_mode = (fusion.ParkInMode == 0) ? 1 : -1;
		if(CpCpSm(x,y,phi,rs_path,parking_mode))
		{
			for (int idx = 0;idx < 3;idx++)
			{
				if (rs_path.rspath_type[idx] == St)
				{
					road_yaw[4 - idx * 2] = 1000000;
				}

				else if (rs_path.rspath_type[idx] == Le)
				{
					road_yaw[4 - idx * 2] = curr_rmin;
				}

				else if (rs_path.rspath_type[idx] == Ri)
				{
					road_yaw[4 - idx * 2] = -curr_rmin;
				}
			}
			std::cout << "aaaaaaaaa\n";
			road_yaw[1] = (-rs_path.v) * curr_rmin;
			road_yaw[3] = (-rs_path.u) * curr_rmin;
			road_yaw[5] = (-rs_path.t) * curr_rmin;
			rs_path.lenth = rs_path.lenth * curr_rmin;
			if (true) // 碰撞检测 GetOrCheckPathPoints(start, road_yaw, 3, 1)
			{
				road_yaw[1] -= 0.3;  // 0.3
				road_yaw[3] = 0;
				road_yaw[5] = 0;
				double dist  = road_yaw[1];
				double r_l = 1e6;
				std::cout << "dist = " << dist << std::endl;
				//return true;
				if(GetOnePathPoints(start,dist ,r_l,1)) //GetOrCheckPathPoints(start, road_yaw, 3, 1)
				{
					start[0] = pathpoint.back().x;
					start[1] = pathpoint.back().y;
					start[2] = pathpoint.back().th;
					return CalcCpCp(start,end, 0);
				}
			}
		}
		else{
			return CalcCpCp(start,end, 0);
		}
		return false;
	}

	bool CalcCmCm_CpCp(const double *start, const double *end, const double r_max20)
	{
		bool flag = false;
		if (fabs(start[0]) < 3 && fabs(start[1]) < 0.3 && fabs(start[2]) < pi/180 * 10)
		{
			flag = true;
		}
		if (fabs(start[0]) < 4 && fabs(start[1]) < 0.2 && fabs(start[2]) < pi/180 * 10)
		{
			flag = true;
		}
		if (flag == false){
			return false;
		}
		double road_yaw1[6] = {.0};
		double road_yaw2[6] = {.0};

		double length_back_init = 3.7-fabs(start[0]);
		for (; length_back_init < 5.6-fabs(start[0]); length_back_init += 0.2)
		{
			double length_back = length_back_init - fabs(start[0]);
			double startm[3] = {.0};
			int phi_LR = 0;
			if (start[2] < 0)
			{
				phi_LR = -1;
			}
			else
			{
				phi_LR = 1;
			}
			road_yaw1[0] = r_max20 * phi_LR;
			road_yaw1[1] = -r_max20 * fabs(start[2]);
			road_yaw1[2] = 1000000;
			road_yaw1[3] = -(length_back - fabs(r_max20 * sin(start[2])));

			if (GetOrCheckPathPoints(start, road_yaw1, 2, 1)) // 碰撞检测
			{
				if (GetOrCheckPathPoints(start, road_yaw1, 2, 0))
				{
					startm[0] = pathpoint.back().x;
					startm[1] = pathpoint.back().y;
					startm[2] = pathpoint.back().th;
				}
			}
			int roadCC = 1;
			roadCC = Calc_CmCm(startm[0], startm[1], startm[2], end[0], end[1], end[2], 20, road_yaw2);
			if (roadCC == 0)
			{
				if (GetOrCheckPathPoints(startm, road_yaw2, 2, 1))
				{
					return GetOrCheckPathPoints(startm, road_yaw2, 2, 0);
				}
			}
			pathpoint.clear();
		}
		pathpoint.clear();
		return false;
	}

	bool CalcCm_SpCpSp(const double *start, const double *end, const double r_max25, double *road_yaw2)
	{
		int road_LR = 0;
		if (start[1] > 0)
		{
			road_LR = 1;
		}
		else if (start[1] < 0)
		{
			road_LR = -1;
		}
		double x = .0, y = .0, phi3 = .0;
		double startm[3] = {.0};
		double phi = -5 * pi / 180 * road_LR;
		startm[0] = start[0] - rmin_vertical * sin(fabs(phi));
		startm[1] = start[1] + road_LR * rmin_vertical * (1 - cos(phi));
		startm[2] = phi;
		CoordinateTrans(x, y, phi3, end[0], end[1], end[2], startm[0], startm[1], startm[2]);
		double y_cs = fabs(y) - r_max25 * (1 - cos(phi3));
		if (y_cs > 0)
		{
			double x_cs = y_cs / fabs(tan(phi3));
			double s1 = fabs(x) - x_cs - fabs(r_max25 * sin(phi3));
			double s2 = sqrt(y_cs * y_cs + x_cs * x_cs);
			road_yaw2[0] = rmin_vertical * road_LR;
			road_yaw2[1] = -rmin_vertical * (5 * pi / 180); // 10度
			road_yaw2[2] = 1000000;
			road_yaw2[3] = s1;
			road_yaw2[4] = road_LR * r_max25;
			road_yaw2[5] = r_max25 * fabs(phi3);
			road_yaw2[6] = 1000000;
			road_yaw2[7] = s2;
			return true;
		}
		return false;
	}

	bool CalcCmSm_CpSpCp(const double *start_p, const double *end, const double r_max25, const double length_back)
	{

		double start[3] = {start_p[0], start_p[1], start_p[2]};
		double road_yaw[6] = {.0};
		double road_yaw2[8] = {.0};
		int phi_LR = 0;
		if (start[2] < 0)
		{
			phi_LR = -1;
		}
		else
		{
			phi_LR = 1;
		}
		int road_LR=0;
		if(start[1] > 0)
		{
			road_LR = 1;
		}
		else if(start[1] < 0)
		{
			road_LR = -1;
		}
		road_yaw[0] = r_max25*phi_LR;
		road_yaw[1] = -r_max25 * fabs(start[2]);
		road_yaw[2] = 1000000;
		road_yaw[3] = -(length_back-r_max25 * sin(fabs(start[2])));
		road_yaw[4] = rmin_vertical*road_LR;
		road_yaw[5] = -rmin_vertical * (15*pi/180);//多转15度
		if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) // 碰撞检测
		{			
			if(GetOrCheckPathPoints(start, road_yaw, 3, 0))
			{
				start[0] = pathpoint.back().x;
				start[1] = pathpoint.back().y;
				start[2] = pathpoint.back().th;	
			}
		}	
		double phi = -5*pi/180 *road_LR;
		double x_cs3 = -r_max25 * sin(fabs(phi));
		double y_cs3 = road_LR*r_max25*(1-cos(phi));
		double endm[3] = {x_cs3,y_cs3,phi};	
		int roadsixth = 1;
		if (roadsixth == CalcCpSpCp(start,end,25,road_yaw2))
		{								
			if (GetOrCheckPathPoints(start, road_yaw2, 3, 1)) // 碰撞检测
			{
				return GetOrCheckPathPoints(start, road_yaw2, 3, 0);
			}
		}
		else
		{
			int roadsecond = 1;
			roadsecond = Calc_CmCm(start[0], start[1], start[2], endm[0], endm[1], endm[2], rmin_vertical, road_yaw2);
			if (roadsecond == 0 )
			{
				road_yaw2[4] = road_LR*r_max25 ;
				road_yaw2[5] = r_max25 * fabs(phi);
				if (GetOrCheckPathPoints(start, road_yaw2, 3, 1))
				{
					return GetOrCheckPathPoints(start, road_yaw2, 3, 0);
				}
			}
		}																							
		pathpoint.clear();
		return false;
	}
	
	bool CalcCmSm_CpCp(const double *start_p,const double *end,const double r_max25)
	{		
		bool flag = false;
		if (fabs(start_p[0]) < 3 && fabs(start_p[1]) < 0.3 && fabs(start_p[2]) < pi/180 * 10)
		{
			flag = true;
		}
		if (fabs(start_p[0]) < 4 && fabs(start_p[1]) < 0.2 && fabs(start_p[2]) < pi/180 * 10)
		{
			flag = true;
		}
		if (flag == false){
			return false;
		}
		
		double start[3] = {start_p[0],start_p[1],start_p[2]};
		double road_yaw[6] = {.0};
		double road_yaw2[8]={.0};	
		double x_p3 = .0, y_p3 = .0, phi_p3 = .0; 
		double x_p0 = .0, y_p0 = .0, phi_p0 = .0; 
		double x_p03 = .0;
		double length_back = .0;
		CoordinateTrans(x_p3,y_p3,phi_p3,P3x,P3y,0,end_to_fus[0],end_to_fus[1],end_to_fus[2]);
		CoordinateTrans(x_p0,y_p0,phi_p0,P0x,P0y,0,end_to_fus[0],end_to_fus[1],end_to_fus[2]);
		if(fabs(x_p3) > fabs(x_p0))
		{
			x_p03 = x_p0;
		}
		else
		{
			x_p03 = x_p3;
		}
		length_back=0.5*fabs(x_p03)+vehicle_parameters.LF - fabs(start[0]);//-r_max25 * sin(fabs(start[2]));
		if(length_back < 0) //4.34-6.44
		{
			return false;
		} 
		double car_length_upper = length_back+0.5*fabs(x_p03)+1.61;
		double car_length_lower = length_back;
		if(length_back-r_max25 * sin(fabs(start[2])) < 0){
			return false;
		}
		for(double car_length_now = car_length_lower; car_length_now <= car_length_upper; car_length_now += 0.2)
		{

			int phi_LR = 0;
			if(start[2] < 0)
			{
				phi_LR = -1;
			}
			else
			{
				phi_LR = 1;
			}
			road_yaw[0] = r_max25*phi_LR;
			road_yaw[1] = -r_max25 * fabs(start[2]);
			road_yaw[2] = 1000000;
			road_yaw[3] = -(car_length_now-r_max25 * sin(fabs(start[2])));
			double startm[3] ={.0};
			if (GetOrCheckPathPoints(start, road_yaw, 2, 1)) // 碰撞检测
			{
				if(GetOrCheckPathPoints(start, road_yaw, 2, 0))
				{
					startm[0] = pathpoint.back().x;
					startm[1] = pathpoint.back().y;
					startm[2] = pathpoint.back().th;	
				}
			}
			int road_LR=0;
			if(startm[1] > 0)
			{
				road_LR = 1;
			}
			else if(startm[1] < 0)
			{
				road_LR = -1;
			}
			double r_cs = r_max25;
			double phi = -5*pi/180 *road_LR;
			double x_cs3 = -r_cs * sin(fabs(phi));
			double y_cs3 = road_LR*r_cs*(1-cos(phi));
			double endm[3] = {x_cs3,y_cs3,phi};		
			int roadCC = 1;
			roadCC = Calc_CmCm(startm[0], startm[1], startm[2], end[0], end[1], end[2], rmin_vertical, road_yaw2);
			if (roadCC == 0 && fabs(road_yaw2[0]) > 20 )
			{
				if (GetOrCheckPathPoints(startm, road_yaw2, 2, 1))
				{
					return GetOrCheckPathPoints(startm, road_yaw2, 2, 0);
				}
			}
			int roadsixth = 1;
			if (roadsixth == CalcCpSpCp(startm,end,25,road_yaw2))
			{																	
				if (GetOrCheckPathPoints(startm, road_yaw2, 3, 1)) // 碰撞检测
				{
					return GetOrCheckPathPoints(startm, road_yaw2, 3, 0);
				}																					
			}
			if(CalcCm_SpCpSp(startm,end,25,road_yaw2) && car_length_now+fabs(rmin_vertical*sin(phi))<car_length_upper)
			{			
				if (GetOrCheckPathPoints(startm, road_yaw2, 4, 1)) // 碰撞检测
				{
					return GetOrCheckPathPoints(startm, road_yaw2, 4, 0);
				}																	
			}
			double x_cs2 = .0, y_cs2 = .0, phi_cs2 = .0;
			CoordinateTrans(x_cs2,y_cs2,phi_cs2,startm[0],startm[1],startm[2],endm[0],endm[1],endm[2]);
			if(CmSp(x_cs2,y_cs2,phi_cs2,rmin_vertical,road_yaw2) && car_length_now+fabs(road_yaw2[0]*sin(phi_cs2)) < car_length_upper)
			{
				road_yaw2[4] = road_LR * r_max25 ;
				road_yaw2[5] = r_max25 * (5*pi/180);
				if (GetOrCheckPathPoints(startm, road_yaw2, 3, 1)) // 碰撞检测
				{
					return GetOrCheckPathPoints(startm, road_yaw2, 3, 0);
				}	
			}
		    pathpoint.clear();		
		}
		pathpoint.clear();
		return CalcCmSm_CpSpCp(start,end,25,length_back+0.5*fabs(x_p03));	
	}	
			
	void ModifyVehicleHeading(double* start_p, double* const pos, double &r_curr, double &theta)
	{
		double aim_theta = (fusion.parkingSpaceInfo.ParkingSpaceType == 1) ? pi/2 : fabs(pos[2]);
		theta = (start_p[2] - ParkingSpaceFlag * aim_theta);
		r_curr = -rmin_vertical;
		if(fabs(theta) > 5*pi/180)
		{
			//ToDo:小角度不调车角度
			if(theta > 0)
			{
				r_curr = rmin_vertical;
				theta = -theta;
			}
			int pp_size_before = pathpoint.size();
			if(GetOnePathPoints(start_p, theta, r_curr, 1))
			{
				start_p[0] = pathpoint.back().x;
				start_p[1] = pathpoint.back().y;
				start_p[2] = pathpoint.back().th;	
			}
			else{
				r_curr = .0;
				theta = .0;
				int pp_size_now = pathpoint.size();
				for(int i = 0; i < pp_size_now - pp_size_before; i++)
				{
					pathpoint.pop_back();
				}
			}
		}
		else{
			theta = .0;
		}
	}

	bool CalcPathToCriticalPostion(double *start_p,double *pos, double &r_curr,double &theta)
	{
		if(PlanBackFirst == 0)
		{
			ModifyVehicleHeading(start_p,pos,r_curr,theta);
			return CalcSmCpCp(start_p, pos);		
		}
		else
		{
			return CalcCpCp(start_p, pos, 0);
		}
		return false;
	}

	bool CalcHeadPathMulti(double* start,double* end)
	{
		int flag = 100;
		//int pp_size_now = 0;
		//int pp_size_before_one = pathpoint.size();
		int index = 0;
		bool flag_backward = true;
		if(Gears == 2 && (PlanBackFirst >= 2))
		{
			flag_backward = false; 
		}
		while (index < 5)
		{
			if(index == 0)
			{
				std::cout << "###########\n";
			}
			if(flag_backward)
			{
				flag = CalcCmCpSpMulti(start,end);
				std::cout << "flag_one = " << flag << std::endl;
			}
			if(flag == 0)
			{
				return false;
			}
			else if(flag == 4)
			{
				return true;
			}
			else{
				start[0] = pathpoint.back().x;
				start[1] = pathpoint.back().y;
				start[2] = pathpoint.back().th;
				//flag = CalcSpCmSp(start,end);
				flag = CalcCpCmSpMulti(start,end);
				std::cout << "flag_two = " << flag << std::endl;
				if(flag == 0)
				{
					return false;
				}
				else if(flag == 4)
				{
					return true;
				}
				else{
					flag_backward = true;
					start[0] = pathpoint.back().x;
					start[1] = pathpoint.back().y;
					start[2] = pathpoint.back().th;
					if(flag == 2)
					{
						std::cout << "start[2] = " << start[2] * 180/pi << std::endl;
						if(fabs(start[2]) < 10*pi/180)
						{
							return CalcCpCp(start, end, 1);
						}
						//ToDo: plan multi once more
					}
				}
				
			}
			index++;
		}
		return false;
	}

	bool SimplifyPath(double* start, double *start_p,int &critical_size, const double &r_curr, const double &theta)
	{
		for(unsigned int idx = critical_size;idx < pathpoint.size();idx++)
		{
			if(pathpoint[idx].D == -0.1)
			{
				critical_size++;
			}
			else{
				break;
			}
		}
		int curve_size = critical_size;
		std::cout << "curve_size = " << curve_size << std::endl;
		for (unsigned int idx = critical_size; idx < pathpoint.size(); idx++){
			if(pathpoint[idx].D > 0 && fabs(pathpoint[idx].delta) > 0.001)
			{
				curve_size++;
			}
			else{
				break;
			}
		}
		std::cout << "curve_size = " << curve_size << std::endl;
		double end[3] = {pathpoint[critical_size-1].x,pathpoint[critical_size-1].y,pathpoint[critical_size-1].th};
		std::vector<path_point> pathpoint_cp(pathpoint.begin(),pathpoint.end());
		//test Cm simplify
		if(CalcCmSimplify(start,end,rmin_vertical, pathpoint[curve_size-1].th))
		{
			std::cout << "cmcmcmcmcm\n";
			pathpoint.insert(pathpoint.end(),pathpoint_cp.begin()+curve_size,pathpoint_cp.end());
			return true;
		}
		//
		if(CalcSmCmSimplify(start, start_p,end, r_curr,theta) || CalcSpCmSimplify(start,end, r_curr,theta))  // || CalcSpCmSimplify(start,end, r_curr,theta)
		{
			std::cout << "spspspsp\n";
			pathpoint.insert(pathpoint.end(),pathpoint_cp.begin()+critical_size,pathpoint_cp.end());
			return true;
		}
		return false;
	}

	bool CalcHeadPathDirectly(const double* start, const double* end)
	{
		//CpSp
		std::cout << "pp_size = " << pathpoint.size() << std::endl;
		if(CalcCpSp(start,end))
		{
			std::cout << "CpSp\n";
			return true;
		}
		//CpCp
		if(fabs(start[2]) < pi/18)
		{
			std::cout << "start[2] = " << start[2] << std::endl;
			if(CalcCpCp(start,end, 1))
			{
				std::cout << "CpCp\n";
				return true;
			}
			else if(CalcSafeRadiusCpCp(start, end, safe_dynamic_rmin))
			{
				return true;
			}
		}
		//倒退前进
		if(CalcCmCm_CpCp(start,end,25))
		{
			return true;
		}
		if(CalcCmSm_CpCp(start,end,25))
		{
			return true;
		}
		//CmSp CmCpSp
		if(Gears == 1)
		{
			
			if(CalcCmSp(start,end))
			{
				return true;
			}
			if(CalcCmCpSpMulti(start,end) == 4)
			{
				std::cout << "CmCpSp\n";
				return true;
			}
		}
		//CpSp CpCmSp
		if(Gears == 2)
		{
			//ToDo:SpCpSp
			if(CalcCpCmSpMulti(start,end) == 4)
			{
				dynamic_plan_flag = true;
				return true;
			}
		}
		pathpoint.clear();
		return false;
	}

	bool HeadParking(double* start, double* end)
	{
		path_point start_point;
		start_point.x = start[0];
		start_point.y = start[1];
		start_point.th = start[2];
		start_point.D = 0;
		start_point.delta = 0;
		pathpoint.clear();
		pathpoint.push_back(start_point);
		if(PlanBackFirst < 2)
		{
			double critical_pos[3] = {.0};
			double start_p[3] = {start[0],start[1],start[2]};
			double r_curr = .0, theta = .0;
			if (CalcCriticalPosition(critical_pos) && CalcPathToCriticalPostion(start_p,critical_pos,r_curr,theta))
			{
				//return true;
				bydapa::common::TicToc tictoc_multi;
				tictoc_multi.tic();
				while(1)
				{
					double multi_time = tictoc_multi.toc();
					multi_time = multi_time * 1e-9;
					int pp_size_before = pathpoint.size();
					if(multi_time > 1.0 || pp_size_before == 0)
					{
						return false;
					}
					critical_pos[0] = pathpoint.back().x;
					critical_pos[1] = pathpoint.back().y;
					critical_pos[2] = pathpoint.back().th;
					int critical_size = pathpoint.size();
					if(CalcHeadPathMulti(critical_pos,end))
					{
						//ToDo:Simplify the path when there are no obstacles on both sides
						if(true && (PlanBackFirst == 0) && SimplifyPath(start,start_p,critical_size, r_curr, theta))
						{
							PlanBackFirst ++;
						}
						PlanBackFirst++;
						std::cout << "PlanBackFirst = " << PlanBackFirst << std::endl;
						return true;
					}
					else{
						int pp_size_now = pathpoint.size();
						for(int i = pp_size_before; i < pp_size_now;i++)
						{
							pathpoint.pop_back();
						}
						
					}
					pathpoint.pop_back();
				}
			}
		}
		else{
			if(CalcHeadPathDirectly(start,end))
			{
				return true;
			}
			pathpoint.push_back(start_point);
			
			if(CalcHeadPathMulti(start,end))
			{
				PlanMulti++;
				return true;
			}
		}
		return false;
	}
}