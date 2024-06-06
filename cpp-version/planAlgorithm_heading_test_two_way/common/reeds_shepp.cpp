#include "global_variable.h"
#include "global_function.h"

namespace byd_apa_plan
{
	void polar(double x, double y, double& r, double& theta)
	{
		r = sqrt(x * x + y * y);
		theta = atan2(y, x);
	}
	bool LpRpSp(double x, double y, double phi, double& t, double& u, double& v)
	{
		double xi = (1 + x * sin(phi) - y * cos(phi) + cos(phi));

		if (fabs(xi) <= 2)
		{
			t = mod2pi(acos(xi / 2) + phi);
			u = mod2pi(acos(xi / 2));
			if ((fabs(phi) < pi / 4) || (fabs(phi) > 3 * pi / 4))
			{
				v = (x + sin(phi) - 2 * sin(t)) / cos(phi);
			}
			else
			{
				v = (y + 2 * cos(t) - 1 - cos(phi)) / sin(phi);
			}
			return true;
		}
		return false;
	}
	bool LpSpRp(double x, double y, double phi, double& t, double& u, double& v)
	{
		double t1, u1;
		polar(x + sin(phi), y - 1. - cos(phi), u1, t1);
		u1 = u1 * u1;
		if (u1 >= 4)
		{
			double theta;
			u = sqrt(u1 - 4);
			theta = atan2(2, u);
			t = mod2pi(t1 + theta);
			v = mod2pi(t - phi);
			return t >= 0 && v >= 0;
		}
		return false;
	}
	bool LpSpLp(double x, double y, double phi, double &t, double &u, double &v)
	{
		polar(x - sin(phi), y - 1. + cos(phi), u, t);
		if (t >= 0)
		{
			v = mod2pi(phi - t);
			if (v >= 0)
			{
				return true;
			}
		}
		return false;
	}
	bool LpRmL(double x, double y, double phi, double &t, double &u, double &v)
	{
		double xi = x - sin(phi), eta = y - 1. + cos(phi), u1, theta;
		polar(xi, eta, u1, theta);
		if (u1 <= 4.)
		{
			u = -2.*asin(.25 * u1);
			t = mod2pi(theta + .5 * u + pi);
			v = mod2pi(phi - t + u);

			return t >= 0 && u <= 0;
		}
		return false;
	}

	void CSC(double x, double y, double phi, RsPath &path)
	{
		double t, u, v, Lmin = 10000, L;
		if (LpSpLp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
		{
			path.t = t;
			path.u = u;
			path.v = v;
			path.lenth = L;
			path.rspath_type[0] = Le;
			path.rspath_type[1] = St;
			path.rspath_type[2] = Le;
			Lmin = L;
		}
		if (LpSpLp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
		{
			path.t = -t;
			path.u = -u;
			path.v = -v;
			path.lenth = L;
			path.rspath_type[0] = Le;
			path.rspath_type[1] = St;
			path.rspath_type[2] = Le;
			Lmin = L;
		}
		if (LpSpLp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
		{
			path.t = t;
			path.u = u;
			path.v = v;
			path.lenth = L;
			path.rspath_type[0] = Ri;
			path.rspath_type[1] = St;
			path.rspath_type[2] = Ri;
			Lmin = L;
		}
		if (LpSpLp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
		{
			path.t = -t;
			path.u = -u;
			path.v = -v;
			path.lenth = L;
			path.rspath_type[0] = Ri;
			path.rspath_type[1] = St;
			path.rspath_type[2] = Ri;
			Lmin = L;
		}
		if (LpSpRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
		{
			path.t = t;
			path.u = u;
			path.v = v;
			path.lenth = L;
			path.rspath_type[0] = Le;
			path.rspath_type[1] = St;
			path.rspath_type[2] = Ri;
			Lmin = L;
		}
		if (LpSpRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
		{
			path.t = -t;
			path.u = -u;
			path.v = -v;
			path.lenth = L;
			path.rspath_type[0] = Le;
			path.rspath_type[1] = St;
			path.rspath_type[2] = Ri;
			Lmin = L;
		}
		if (LpSpRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
		{
			path.t = t;
			path.u = u;
			path.v = v;
			path.lenth = L;
			path.rspath_type[0] = Ri;
			path.rspath_type[1] = St;
			path.rspath_type[2] = Le;
			Lmin = L;
		}
		if (LpSpRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
		{
			path.t = -t;
			path.u = -u;
			path.v = -v;
			path.lenth = L;
			path.rspath_type[0] = Ri;
			path.rspath_type[1] = St;
			path.rspath_type[2] = Le;
			Lmin = L;
		}
	}
	void CCC(double x, double y, double phi, RsPath &path)
	{
		double t, u, v, L;
		if (LpRmL(x, y, phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v)))
		{
			path.t = t;
			path.u = u;
			path.v = v;
			path.lenth = L;
			path.rspath_type[0] = Le;
			path.rspath_type[1] = Ri;
			path.rspath_type[2] = Le;
		}
		if (LpRmL(-x, y, -phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
		{
			path.t = -t;
			path.u = -u;
			path.v = -v;
			path.lenth = L;
			path.rspath_type[0] = Le;
			path.rspath_type[1] = Ri;
			path.rspath_type[2] = Le;
		}
		if (LpRmL(x, -y, -phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
		{
			path.t = t;
			path.u = u;
			path.v = v;
			path.lenth = L;
			path.rspath_type[0] = Ri;
			path.rspath_type[1] = Le;
			path.rspath_type[2] = Ri;
		}
		if (LpRmL(-x, -y, phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
		{
			path.t = -t;
			path.u = -u;
			path.v = -v;
			path.lenth = L;
			path.rspath_type[0] = Ri;
			path.rspath_type[1] = Le;
			path.rspath_type[2] = Ri;
		}

		double xb = x*cos(phi) + y*sin(phi), yb = x*sin(phi) - y*cos(phi);
		if (LpRmL(xb, yb, phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v)))
		{
			path.t = v;
			path.u = u;
			path.v = t;
			path.lenth = L;
			path.rspath_type[0] = Le;
			path.rspath_type[1] = Ri;
			path.rspath_type[2] = Le;
		}
		if (LpRmL(-xb, yb, -phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
		{
			path.t = -v;
			path.u = -u;
			path.v = -t;
			path.lenth = L;
			path.rspath_type[0] = Le;
			path.rspath_type[1] = Ri;
			path.rspath_type[2] = Le;
		}
		if (LpRmL(xb, -yb, -phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
		{
			path.t = v;
			path.u = u;
			path.v = t;
			path.lenth = L;
			path.rspath_type[0] = Ri;
			path.rspath_type[1] = Le;
			path.rspath_type[2] = Ri;
		}
		if (LpRmL(-xb, -yb, phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
		{
			path.t = -v;
			path.u = -u;
			path.v = -t;
			path.lenth = L;
			path.rspath_type[0] = Ri;
			path.rspath_type[1] = Le;
			path.rspath_type[2] = Ri;
		}
	}
	int CCS(double x, double y, double phi, RsPath &path)
	{
		double t, u, v, L;
		if (LpRpSp(x, y, phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v)))
		{
			path.t = t;
			path.u = u;
			path.v = v;
			path.lenth = L;
			path.rspath_type[0] = Le;
			path.rspath_type[1] = Ri;
			path.rspath_type[2] = St;
			
		}
		if (LpRpSp(-x, y, -phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
		{
			path.t = -t;
			path.u = -u;
			path.v = -v;
			path.lenth = L;
			path.rspath_type[0] = Le;
			path.rspath_type[1] = Ri;
			path.rspath_type[2] = St;
			
		}
		if (LpRpSp(x, -y, -phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
		{
			path.t = t;
			path.u = u;
			path.v = v;
			path.lenth = L;
			path.rspath_type[0] = Ri;
			path.rspath_type[1] = Le;
			path.rspath_type[2] = St;
			
		}
		if (LpRpSp(-x, -y, phi, t, u, v) && path.lenth > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
		{
			path.t = -t;
			path.u = -u;
			path.v = -v;
			path.lenth = L;
			path.rspath_type[0] = Ri;
			path.rspath_type[1] = Le;
			path.rspath_type[2] = St;
			
		}

		if (path.lenth < 1000)
		{
			return 0;
		}
		else
		{
			return 2;
		}
	}

	int CC(double x, double y, double theta, double car_r_limit, double road_yaw[6])
	{
		//////////////////////////////距离太短，不做输出求解,保护后面数据不越界//////////////////////////////////////////////////////////
		if (fabs(x) < 0.05 || fabs(y) > 1000 || fabs(x) > 1000)
		{
			return 2;//距离目标小于5cm
		}
		double a;
		double b;
		double c;
		double sqr;
		//double car_r_limit = 5.5;
		double R_1 = 0;
		double sin_theta_first_1 = 0;//车辆第一个旋转角度的sin值
		double cos_theta_first_1 = 0;//车辆第一个旋转角度的cos值
		double angle_1 = .0;
		//double distance1 = 0;
		double R_2 = 0;
		double sin_theta_first_2 = 0;//车辆第一个旋转角度的sin值
		double cos_theta_first_2 = 0;//车辆第一个旋转角度的cos值
		double angle_2 = .0;
		//double distance2 = 0;
		///////////////////计算公式///////////
		a = pow(cos(theta) + 1, 2) + pow(sin(theta), 2) - 4;
		b = 2 * x * sin(theta) - 2 * y * (cos(theta) + 1);
		c = x * x + y * y;
		sqr = pow(b, 2) - 4 * a * c;
		if (fabs(a) < 0.0001)
		{
			a = 0;
		}
		///////////////////////////////////////均方根为负值则无解//////////////////////////////////////////////
		if (sqr < 0)
		{
			return 1;
		}
		///////////////////////////////////////a为0，b为0则无解//////////////////////////////////////////////
		if (a == 0 && b == 0)
		{
			return 1;
		}
		/////////////////////////////////////a=0，单独计算求解//////////////////////////////////////
		if (a == 0)
		{
			if (fabs(c * 0.000001) >= fabs(b))   //保护除数
			{
				R_1 = 1e6;
			}
			else
			{
				R_1 = -c / b;
			}
			if (fabs(R_1) < car_r_limit)
			{
				return 1;//无解
			}
			sin_theta_first_1 = x / 2 / (R_1)+sin(theta) / 2;
			cos_theta_first_1 = -y / 2 / (R_1)+cos(theta) / 2 + 0.5;
			if (GetAntiTrigo(sin_theta_first_1, cos_theta_first_1, angle_1) != 0)//无解
			{
				return 1;
			}
			////////////////////可删除下面两行验证用/////////////////////////////////////////
			//double aaa_x1 = 2 * R_1 * sin(angle_1[0]) - R_1 * sin(0.0);
			//double aaa_y1 = R_1 - 2 * R_1 * cos(angle_1[0]) + R_1 * cos(0.0);
			if (angle_1 * (theta - angle_1) > 0)//两端路必须为同一方向
			{
				return 1;//无解
			}
			//double distance1 = fabs(R_1 * angle_1[0]) + fabs(R_1 * (theta - angle_1[0]));//可以删除，验证用
			road_yaw[0] = R_1;
			road_yaw[1] = R_1 * angle_1;
			road_yaw[2] = -R_1;
			road_yaw[3] = -R_1 * (theta - angle_1);
			return 0;
		}

		////一元二次方程两个解，求解1
		if (fabs((-b + pow(sqr, 0.5)) * 0.000002) >= fabs(a))   //保护除数
		{
			R_1 = 1e6;
		}
		else
		{
			R_1 = (-b + pow(sqr, 0.5)) / 2 / a;
		}
		if (fabs(R_1) >= car_r_limit)
		{
			sin_theta_first_1 = x / 2 / (R_1)+sin(theta) / 2;
			cos_theta_first_1 = -y / 2 / (R_1)+cos(theta) / 2 + 0.5;
			if (GetAntiTrigo(sin_theta_first_1, cos_theta_first_1, angle_1) != 0)
			{
				return 1;
			}
			//double distance1 = fabs(R_1 * angle_1[0]) + fabs(R_1 * (theta - angle_1[0]));
		}
		////一元二次方程两个解，求解2
		if (fabs((-b - pow(sqr, 0.5)) * 0.000002) >= fabs(a))   //保护除数
		{
			R_2 = 1e6;
		}
		else
		{
			R_2 = (-b - pow(sqr, 0.5)) / 2 / a;
		}
		if (fabs(R_2) >= car_r_limit)
		{
			sin_theta_first_2 = x / 2 / (R_2)+sin(theta) / 2;
			cos_theta_first_2 = -y / 2 / (R_2)+cos(theta) / 2 + 0.5;
			if (GetAntiTrigo(sin_theta_first_2, cos_theta_first_2, angle_2) != 0)
			{
				return 1;
			}
			//double distance2 = fabs(R_2 * angle_2[0]) + fabs(R_2 * (theta - angle_2[0]));
		}
		if ((fabs(R_1) >= car_r_limit) && (fabs(R_2) >= car_r_limit))//两个解都有，取同一方向
		{
			if ((angle_1) * (theta - angle_1) <= 0)
			{
				road_yaw[0] = R_1;
				road_yaw[1] = R_1 * angle_1;
				road_yaw[2] = -R_1;
				road_yaw[3] = -R_1 * (theta - angle_1);
			}
			else if ((angle_2) * (theta - angle_2) <= 0)
			{
				road_yaw[0] = R_2;
				road_yaw[1] = R_2 * angle_2;
				road_yaw[2] = -R_2;
				road_yaw[3] = -R_2 * (theta - angle_2);
			}
			else
			{
				return 1;
			}
		}
		else if (fabs(R_1) < car_r_limit && fabs(R_2) < car_r_limit)
		{
			return 1;
		}

		else if (fabs(R_1) >= car_r_limit)
		{
			if ((angle_1) * (theta - angle_1) <= 0)
			{
				road_yaw[0] = R_1;
				road_yaw[1] = R_1 * angle_1;
				road_yaw[2] = -R_1;
				road_yaw[3] = -R_1 * (theta - angle_1);
			}
			else
			{
				return 1;
			}
		}
		else
		{
			if ((angle_2) * (theta - angle_2) <= 0)
			{
				road_yaw[0] = R_2;
				road_yaw[1] = R_2 * angle_2;
				road_yaw[2] = -R_2;
				road_yaw[3] = -R_2 * (theta - angle_2);
			}
			else
			{
				return 1;
			}
		}
		return 0;
	}

	int CpSpCp(double x, double y, double phi, RsPath& path)
	{
		double t, u, v, L = .0;
		if (LpSpRp(x, y, phi, t, u, v) && (t >= 0) && (u >= 0) && (v >= 0))
		{
			path.t = t;
			path.u = u;
			path.v = v;
			path.lenth = L;
			path.rspath_type[0] = Le;
			path.rspath_type[1] = St;
			path.rspath_type[2] = Ri;
			return 1;
		}
		if (LpSpRp(-x, y, -phi, t, u, v) && (-t >= 0) && (-u >= 0) && (-v >= 0)) // timeflip
		{
			path.t = -t;
			path.u = -u;
			path.v = -v;
			path.lenth = L;
			path.rspath_type[0] = Le;
			path.rspath_type[1] = St;
			path.rspath_type[2] = Ri;
			return 1;
		}
		if (LpSpRp(x, -y, -phi, t, u, v) && (t >= 0) && (u >= 0) && (v >= 0)) // reflect
		{
			path.t = t;
			path.u = u;
			path.v = v;
			path.lenth = L;
			path.rspath_type[0] = Ri;
			path.rspath_type[1] = St;
			path.rspath_type[2] = Le;
			return 1;
		}
		if (LpSpRp(-x, -y, phi, t, u, v) && (-t >= 0) && (-u >= 0) && (-v >= 0)) // timeflip + reflect
		{
			path.t = -t;
			path.u = -u;
			path.v = -v;
			path.lenth = L;
			path.rspath_type[0] = Ri;
			path.rspath_type[1] = St;
			path.rspath_type[2] = Le;
			return 1;
		}
		return 0;
	}

	int CpCpS(double x, double y, double phi, RsPath& path)
	{
		double t, u, v, L = .0;
		if (LpRpSp(x, y, phi, t, u, v) && (t >= 0) && (u >= 0))
		{
			path.t = t;
			path.u = u;
			path.v = v;
			path.lenth = L;
			path.rspath_type[0] = Le;
			path.rspath_type[1] = Ri;
			path.rspath_type[2] = St;
			return 1;

		}
		if (LpRpSp(-x, y, -phi, t, u, v) && (t <= 0) && (u <= 0)) // timeflip
		{
			path.t = -t;
			path.u = -u;
			path.v = -v;
			path.lenth = L;
			path.rspath_type[0] = Le;
			path.rspath_type[1] = Ri;
			path.rspath_type[2] = St;
			return 1;
		}
		if (LpRpSp(x, -y, -phi, t, u, v) && (t >= 0) && (u >= 0)) // reflect
		{
			path.t = t;
			path.u = u;
			path.v = v;
			path.lenth = L;
			path.rspath_type[0] = Ri;
			path.rspath_type[1] = Le;
			path.rspath_type[2] = St;
			return 1;
		}
		if (LpRpSp(-x, -y, phi, t, u, v) && (t <= 0) && (u <= 0)) // timeflip + reflect
		{
			path.t = -t;
			path.u = -u;
			path.v = -v;
			path.lenth = L;
			path.rspath_type[0] = Ri;
			path.rspath_type[1] = Le;
			path.rspath_type[2] = St;
			return 1;
		}
		return 0;
	}

	//
	bool CmCpSm(double x, double y, double phi, RsPath& path_five, int theta_car)
	{
		double t, u, v;
		if (LpRpSp(x, y, phi, t, u, v) && (((t <= 0) && (u >= 0) && (v <= 0) && (theta_car == 1)) || ((t >= 0) && (u <= 0) && (v >= 0) && (theta_car == -1))))
		{
			path_five.t = t;
			path_five.u = u;
			path_five.v = v;
			path_five.lenth = fabs(t) + fabs(u) + fabs(v);
			path_five.rspath_type[0] = Le;
			path_five.rspath_type[1] = Ri;
			path_five.rspath_type[2] = St;
			return true;
		}
		if (LpRpSp(-x, y, -phi, t, u, v) && (((t >= 0) && (u <= 0) && (v >= 0) && (theta_car == 1)) || ((t <= 0) && (u >= 0) && (v <= 0) && (theta_car == -1))))
			//	if (LpRpSp(-x, y, -phi, t, u, v) && (t <= 0) && (u <= 0) && (v >= 0)) // timeflip
		{
			path_five.t = -t;
			path_five.u = -u;
			path_five.v = -v;
			path_five.lenth = fabs(t) + fabs(u) + fabs(v);
			path_five.rspath_type[0] = Le;
			path_five.rspath_type[1] = Ri;
			path_five.rspath_type[2] = St;
			return true;
		}
		if (LpRpSp(x, -y, -phi, t, u, v) && (((t <= 0) && (u >= 0) && (v <= 0) && (theta_car == 1)) || ((t >= 0) && (u <= 0) && (v >= 0) && (theta_car == -1))))
			//	if (LpRpSp(x, -y, -phi, t, u, v) && (t >= 0) && (u >= 0) && (v <= 0)) // reflect
		{
			path_five.t = t;
			path_five.u = u;
			path_five.v = v;
			path_five.lenth = fabs(t) + fabs(u) + fabs(v);
			path_five.rspath_type[0] = Ri;
			path_five.rspath_type[1] = Le;
			path_five.rspath_type[2] = St;
			return true;
		}
		if (LpRpSp(-x, -y, phi, t, u, v) && (((t >= 0) && (u <= 0) && (v >= 0) && (theta_car == 1)) || ((t <= 0) && (u >= 0) && (v <= 0) && (theta_car == -1))))
			//	if (LpRpSp(-x, -y, phi, t, u, v) && (t <= 0) && (u <= 0) && (v >= 0)) // timeflip + reflect
		{
			path_five.t = -t;
			path_five.u = -u;
			path_five.v = -v;
			path_five.lenth = fabs(t) + fabs(u) + fabs(v);
			path_five.rspath_type[0] = Ri;
			path_five.rspath_type[1] = Le;
			path_five.rspath_type[2] = St;
			return true;
		}

		return false;
	}
	//
	bool CpCmSm(double x, double y, double phi, RsPath& path_six, int theta_car)
	{
		double t, u, v;
		if (LpRpSp(x, y, phi, t, u, v) && (((t >= 0) && (u <= 0) && (v <= 0) && (theta_car == 1)) || ((t <= 0) && (u >= 0) && (v >= 0) && (theta_car == -1))))
			//	if (LpRpSp(x, y, phi, t, u, v) && (t >= 0) && (u <= 0) && (v <= 0))
		{
			path_six.t = t;
			path_six.u = u;
			path_six.v = v;
			path_six.lenth = fabs(t) + fabs(u) + fabs(v);
			path_six.rspath_type[0] = Le;
			path_six.rspath_type[1] = Ri;
			path_six.rspath_type[2] = St;
			return true;
		}
		if (LpRpSp(-x, y, -phi, t, u, v) && (((t <= 0) && (u >= 0) && (v >= 0) && (theta_car == 1)) || ((t >= 0) && (u <= 0) && (v <= 0) && (theta_car == -1))))
			//	if (LpRpSp(-x, y, -phi, t, u, v) && (t <= 0) && (u >= 0) && (v >= 0)) // timeflip
		{
			path_six.t = -t;
			path_six.u = -u;
			path_six.v = -v;
			path_six.lenth = fabs(t) + fabs(u) + fabs(v);
			path_six.rspath_type[0] = Le;
			path_six.rspath_type[1] = Ri;
			path_six.rspath_type[2] = St;
			return true;
		}
		if (LpRpSp(x, -y, -phi, t, u, v) && (((t >= 0) && (u <= 0) && (v <= 0) && (theta_car == 1)) || ((t <= 0) && (u >= 0) && (v >= 0) && (theta_car == -1))))
			//	if (LpRpSp(x, -y, -phi, t, u, v) && (t >= 0) && (u <= 0) && (v <= 0)) // reflect
		{
			path_six.t = t;
			path_six.u = u;
			path_six.v = v;
			path_six.lenth = fabs(t) + fabs(u) + fabs(v);
			path_six.rspath_type[0] = Ri;
			path_six.rspath_type[1] = Le;
			path_six.rspath_type[2] = St;
			return true;
		}
		if (LpRpSp(-x, -y, phi, t, u, v) && (((t <= 0) && (u >= 0) && (v >= 0) && (theta_car == 1)) || ((t >= 0) && (u <= 0) && (v <= 0) && (theta_car == -1))))
			//	if (LpRpSp(-x, -y, phi, t, u, v) && (t <= 0) && (u >= 0) && (v >= 0)) // timeflip + reflect
		{
			path_six.t = -t;
			path_six.u = -u;
			path_six.v = -v;
			path_six.lenth = fabs(t) + fabs(u) + fabs(v);
			path_six.rspath_type[0] = Ri;
			path_six.rspath_type[1] = Le;
			path_six.rspath_type[2] = St;
			return true;
		}

		return false;

	}
	//
	bool CpCpSm(double x, double y, double phi, RsPath& path_six, int theta_car)
	{
		double t, u, v;
		if (LpRpSp(x, y, phi, t, u, v) && (((t >= 0) && (u >= 0) && (v <= 0) && (theta_car == 1)) || ((t <= 0) && (u <= 0) && (v >= 0) && (theta_car == -1))))
			//	if (LpRpSp(x, y, phi, t, u, v) && (t >= 0) && (u <= 0) && (v <= 0))
		{
			path_six.t = t;
			path_six.u = u;
			path_six.v = v;
			path_six.lenth = fabs(t) + fabs(u) + fabs(v);
			path_six.rspath_type[0] = Le;
			path_six.rspath_type[1] = Ri;
			path_six.rspath_type[2] = St;
			return true;
		}
		if (LpRpSp(-x, y, -phi, t, u, v) && (((t <= 0) && (u <= 0) && (v >= 0) && (theta_car == 1)) || ((t >= 0) && (u >= 0) && (v <= 0) && (theta_car == -1))))
			//	if (LpRpSp(-x, y, -phi, t, u, v) && (t <= 0) && (u >= 0) && (v >= 0)) // timeflip
		{
			path_six.t = -t;
			path_six.u = -u;
			path_six.v = -v;
			path_six.lenth = fabs(t) + fabs(u) + fabs(v);
			path_six.rspath_type[0] = Le;
			path_six.rspath_type[1] = Ri;
			path_six.rspath_type[2] = St;
			return true;
		}
		if (LpRpSp(x, -y, -phi, t, u, v) && (((t >= 0) && (u >= 0) && (v <= 0) && (theta_car == 1)) || ((t <= 0) && (u <= 0) && (v >= 0) && (theta_car == -1))))
			//	if (LpRpSp(x, -y, -phi, t, u, v) && (t >= 0) && (u <= 0) && (v <= 0)) // reflect
		{
			path_six.t = t;
			path_six.u = u;
			path_six.v = v;
			path_six.lenth = fabs(t) + fabs(u) + fabs(v);
			path_six.rspath_type[0] = Ri;
			path_six.rspath_type[1] = Le;
			path_six.rspath_type[2] = St;
			return true;
		}
		if (LpRpSp(-x, -y, phi, t, u, v) && (((t <= 0) && (u <= 0) && (v >= 0) && (theta_car == 1)) || ((t >= 0) && (u >= 0) && (v <= 0) && (theta_car == -1))))
			//	if (LpRpSp(-x, -y, phi, t, u, v) && (t <= 0) && (u >= 0) && (v >= 0)) // timeflip + reflect
		{
			path_six.t = -t;
			path_six.u = -u;
			path_six.v = -v;
			path_six.lenth = fabs(t) + fabs(u) + fabs(v);
			path_six.rspath_type[0] = Ri;
			path_six.rspath_type[1] = Le;
			path_six.rspath_type[2] = St;
			return true;
		}

		return false;

	}
	//
	bool CmCmSm(double x, double y, double phi, RsPath& path_six, int theta_car)
	{
		double t, u, v;
		if (LpRpSp(x, y, phi, t, u, v) && (((t <= 0) && (u <= 0) && (v <= 0) && (theta_car == 1)) || ((t >= 0) && (u >= 0) && (v >= 0) && (theta_car == -1))))
			//	if (LpRpSp(x, y, phi, t, u, v) && (t >= 0) && (u <= 0) && (v <= 0))
		{
			path_six.t = t;
			path_six.u = u;
			path_six.v = v;
			path_six.lenth = fabs(t) + fabs(u) + fabs(v);
			path_six.rspath_type[0] = Le;
			path_six.rspath_type[1] = Ri;
			path_six.rspath_type[2] = St;
			return true;
		}
		if (LpRpSp(-x, y, -phi, t, u, v) && (((t >= 0) && (u >= 0) && (v >= 0) && (theta_car == 1)) || ((t <= 0) && (u <= 0) && (v <= 0) && (theta_car == -1))))
			//	if (LpRpSp(-x, y, -phi, t, u, v) && (t <= 0) && (u >= 0) && (v >= 0)) // timeflip
		{
			path_six.t = -t;
			path_six.u = -u;
			path_six.v = -v;
			path_six.lenth = fabs(t) + fabs(u) + fabs(v);
			path_six.rspath_type[0] = Le;
			path_six.rspath_type[1] = Ri;
			path_six.rspath_type[2] = St;
			return true;
		}
		if (LpRpSp(x, -y, -phi, t, u, v) && (((t <= 0) && (u <= 0) && (v <= 0) && (theta_car == 1)) || ((t >= 0) && (u >= 0) && (v >= 0) && (theta_car == -1))))
			//	if (LpRpSp(x, -y, -phi, t, u, v) && (t >= 0) && (u <= 0) && (v <= 0)) // reflect
		{
			path_six.t = t;
			path_six.u = u;
			path_six.v = v;
			path_six.lenth = fabs(t) + fabs(u) + fabs(v);
			path_six.rspath_type[0] = Ri;
			path_six.rspath_type[1] = Le;
			path_six.rspath_type[2] = St;
			return true;
		}
		if (LpRpSp(-x, -y, phi, t, u, v) && (((t >= 0) && (u >= 0) && (v >= 0) && (theta_car == 1)) || ((t <= 0) && (u <= 0) && (v <= 0) && (theta_car == -1))))
			//	if (LpRpSp(-x, -y, phi, t, u, v) && (t <= 0) && (u >= 0) && (v >= 0)) // timeflip + reflect
		{
			path_six.t = -t;
			path_six.u = -u;
			path_six.v = -v;
			path_six.lenth = fabs(t) + fabs(u) + fabs(v);
			path_six.rspath_type[0] = Ri;
			path_six.rspath_type[1] = Le;
			path_six.rspath_type[2] = St;
			return true;
		}
		return false;
	}
	//2023.03.18
	//2023.02.21
	bool SmLmSm(const double x, const double y, const double phi, RsPath& path_, int theta_car)
	{
		double phi_mod = mod2pi(phi);
		double xd = 0.0;
		double u = 0.0;
		double t = 0.0;
		double v = 0.0;
		double epsilon = 1e-1;
		if (y > 0.0 && fabs(phi_mod) > epsilon && fabs(phi_mod) < pi) {
			xd = -y / tan(phi_mod) + x;
			t = xd - tan(phi_mod / 2.0);
			u = phi_mod;
			v = -sqrt((x - xd) * (x - xd) + y * y) - tan(phi_mod / 2.0);
			path_.u = u;
			path_.t = t;
			path_.v = v;
			path_.lenth = fabs(t) + fabs(u) + fabs(v);
			path_.rspath_type[0] = St;
			path_.rspath_type[1] = Le;
			path_.rspath_type[2] = St;
			return (t <= 0 && u <= 0 && v <= 0);
		}
		else if (y < 0.0 && phi_mod > epsilon && phi_mod < pi) {
			xd = -y / tan(phi_mod) + x;
			t = xd + tan(phi_mod / 2.0);
			u = -phi_mod;
			v = -sqrt((x - xd) * (x - xd) + y * y) + tan(phi_mod / 2.0);
			path_.u = u;
			path_.t = t;
			path_.v = v;
			path_.lenth = fabs(t) + fabs(u) + fabs(v);
			path_.rspath_type[0] = St;
			path_.rspath_type[1] = Ri;
			path_.rspath_type[2] = St;
			return (t <= 0 && u <= 0 && v <= 0);
		}
		return false;
	}
	bool SpLmSm(const double x, const double y, const double phi, RsPath& path_, int theta_car)
	{
		double phi_mod = mod2pi(phi);
		double xd = 0.0;
		double u = 0.0;
		double t = 0.0;
		double v = 0.0;
		double epsilon = 1e-1;
		
		if (y > 0.0 && fabs(phi_mod) > epsilon && fabs(phi_mod) < pi) {
			xd = -y / tan(phi_mod) + x;
			t = xd - tan(phi_mod / 2.0);
			u = phi_mod;
			v = -sqrt((x - xd) * (x - xd) + y * y) - tan(phi_mod / 2.0);
			path_.u = u;
			path_.t = t;
			path_.v = v;
			path_.lenth = fabs(t) + fabs(u) + fabs(v);
			path_.rspath_type[0] = St;
			path_.rspath_type[1] = Le;
			path_.rspath_type[2] = St;
			return (t >= 0 && u <= 0);  // && v <= -0.0517
		}
		else if (y < 0.0 && phi_mod > epsilon && phi_mod < pi) {
			xd = -y / tan(phi_mod) + x;
			t = xd + tan(phi_mod / 2.0);
			u = -phi_mod;
			v = -sqrt((x - xd) * (x - xd) + y * y) + tan(phi_mod / 2.0);
			path_.u = u;
			path_.t = t;
			path_.v = v;
			path_.lenth = fabs(t) + fabs(u) + fabs(v);
			path_.rspath_type[0] = St;
			path_.rspath_type[1] = Ri;
			path_.rspath_type[2] = St;
			//std::cout << "t = " << t << " u = " << u << "v = " << v << std::endl;
			return (t >= 0 && u <= 0); //  && v <= -0.0517
		}
		return false;
	}

	bool SmLpSm(const double x, const double y, const double phi, RsPath& path_, int theta_car)
	{
		double phi_mod = mod2pi(phi);
		double xd = 0.0;
		double u = 0.0;
		double t = 0.0;
		double v = 0.0;
		double epsilon = 1e-1;
		if (y > 0.0 && fabs(phi_mod) > epsilon && fabs(phi_mod) < pi)
		{
			xd = -y / tan(phi_mod) + x;
			t = xd + tan(phi_mod / 2.0);
			u = -phi_mod;
			v = -sqrt((x - xd) * (x - xd) + y * y) + tan(phi_mod / 2.0);
			path_.u = u;
			path_.t = t;
			path_.v = v;
			path_.lenth = fabs(t) + fabs(u) + fabs(v);
			path_.rspath_type[0] = St;
			path_.rspath_type[1] = Ri;
			path_.rspath_type[2] = St;
			return (t <= 0 && u >= 0 && v <= -0.0517); // 0.0862
		}
		else if (y < 0.0 && phi_mod > epsilon && phi_mod < pi)
		{
			xd = -y / tan(phi_mod) + x;
			t = xd - tan(phi_mod / 2.0);
			u = phi_mod;
			v = -sqrt((x - xd) * (x - xd) + y * y) - tan(phi_mod / 2.0);
			path_.u = u;
			path_.t = t;
			path_.v = v;
			path_.lenth = fabs(t) + fabs(u) + fabs(v);
			path_.rspath_type[0] = St;
			path_.rspath_type[1] = Le;
			path_.rspath_type[2] = St;
			//std::cout << "t = " << t << " u = " << u << "v = " << v << std::endl;
			return (t <= 0 && u >= 0 && v <= -0.0517); // 0.0862
		}
		return false;
	}

	bool Calc_SmCmSm(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim,
		double car_r_limit, double road_yaw[6])
	{
		double x, y, phi;
		RsPath ccs_path;
		CoordinateTrans(x, y, phi, x_aim, y_aim, theta_aim, x_now, y_now, theta_now);
		x = x / car_r_limit;
		y = y / car_r_limit;
		int theta_car = 0;
		if (fusion.ParkInMode == 1)
		{
			theta_car = -1;
		}
		else
		{
			theta_car = 1;
		}
		if (SmLmSm(x, y, phi, ccs_path, theta_car))
		{
			for (int i = 0; i < 3; i++)
			{
				if (ccs_path.rspath_type[i] == St)
				{
					road_yaw[i * 2] = 1e6;
				}
				else if (ccs_path.rspath_type[i] == Le)
				{
					road_yaw[i * 2] = car_r_limit;
				}
				else if (ccs_path.rspath_type[i] == Ri)
				{
					road_yaw[i * 2] = -car_r_limit;
				}
			}
			road_yaw[1] = car_r_limit * ccs_path.t;
			road_yaw[3] = car_r_limit * ccs_path.u;
			// 
			road_yaw[5] = car_r_limit * ccs_path.v;
			ccs_path.lenth = ccs_path.lenth * car_r_limit;
			return true;
		}
		return false;
	}

	bool Calc_SpCmSm(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim,
		double car_r_limit, double road_yaw[6])
	{
		double x, y, phi;
		RsPath ccs_path;
		CoordinateTrans(x, y, phi, x_aim, y_aim, theta_aim, x_now, y_now, theta_now);
		x = x / car_r_limit;
		y = y / car_r_limit;
		int theta_car = 0;
		if (fusion.ParkInMode == 1)
		{
			theta_car = -1;
		}
		else
		{
			theta_car = 1;
		}
		if (SpLmSm(x, y, phi, ccs_path, theta_car))
		{
			for (int i = 0; i < 3; i++)
			{
				if (ccs_path.rspath_type[i] == St)
				{
					road_yaw[i * 2] = 1e6;
				}
				else if (ccs_path.rspath_type[i] == Le)
				{
					road_yaw[i * 2] = car_r_limit;
				}
				else if (ccs_path.rspath_type[i] == Ri)
				{
					road_yaw[i * 2] = -car_r_limit;
				}
			}
			road_yaw[1] = car_r_limit * ccs_path.t;
			road_yaw[3] = car_r_limit * ccs_path.u;
			// 
			road_yaw[5] = car_r_limit * ccs_path.v;
			ccs_path.lenth = ccs_path.lenth * car_r_limit;
			return true;
		}
		return false;
	}

	bool Calc_SmCpSm(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim,
		double car_r_limit, double road_yaw[6])
	{
		double x, y, phi;
		RsPath ccs_path;
		CoordinateTrans(x, y, phi, x_aim, y_aim, theta_aim, x_now, y_now, theta_now);
		x = x / car_r_limit;
		y = y / car_r_limit;
		int theta_car = 0;
		if (fusion.ParkInMode == 1)
		{
			theta_car = -1;
		}
		else
		{
			theta_car = 1;
		}
		if (SmLpSm(x, y, phi, ccs_path, theta_car))
		{
			for (int i = 0; i < 3; i++)
			{
				if (ccs_path.rspath_type[i] == St)
				{
					road_yaw[i * 2] = 1e6;
				}
				else if (ccs_path.rspath_type[i] == Le)
				{
					road_yaw[i * 2] = car_r_limit;
				}
				else if (ccs_path.rspath_type[i] == Ri)
				{
					road_yaw[i * 2] = -car_r_limit;
				}
			}
			road_yaw[1] = car_r_limit * ccs_path.t;
			road_yaw[3] = car_r_limit * ccs_path.u;
			//
			road_yaw[5] = car_r_limit * ccs_path.v;
			ccs_path.lenth = ccs_path.lenth * car_r_limit;
			return true;
		}
		//std::cout << "******************" << std::endl;
		return false;
	}

	char GetAntiTrigo(double sin_result, double cos_result, double &last_angle_result)
	{
		if (sin_result > 1 || cos_result > 1)//如果正弦余弦值大于1，则返回1，超界
		{
			return 1;
		}
		if (cos_result >= 0)
		{
			last_angle_result = asin(sin_result);
		}
		else if (cos_result < 0 && sin_result <= 0)
		{
			last_angle_result = -1 * pi - asin(sin_result);
		}
		else
		{
			last_angle_result = acos(cos_result);
		}
		return 0;
	}

	//2022.12.27
	//
	char Calc_Cm(double x_now, double y_now, double thea_now,
		double x_aim, double y_aim, double thea_aim,
		double y_tolerance, double min_r, double yaw_tolerance, double road_yaw[6])  //根据车辆当前点及目标点计算转弯半径
	{
		double tolerance_y = 0;//纠正偏航角偏差后横向距离误差
		double tolerance_y_a = 0;
		double tolerance_y_b = 0;
		double r_yaw = 0;//纠正偏航角需要的转弯半径
		double r_yaw_a = 0;
		double r_yaw_b = 0;
		double mid_thea;
		yaw_tolerance = yaw_tolerance * pi / 180;
		//////////////////////////////////////////坐标转换，输出车辆当前坐标为原点的x坐标y坐标及角度////////////////////////////////////////////////////////////
		double y = (y_aim - y_now) * cos(thea_now) - (x_aim - x_now) * sin(thea_now);// 坐标转换后 Y向的实际偏差
		double x = (x_aim - x_now) * cos(thea_now) + (y_aim - y_now) * sin(thea_now);
		double aim_now_thea_tolerance = 0;
		aim_now_thea_tolerance = mod2pi(thea_aim - thea_now);
		////////如果距离目标太近，则无需调整//////////////////////////////////////////////////////////////////////////////////////////////////////
		if (fabs(x) < 0.05)
		{
			return 2;//距离目标小于5cm
		}
		//////////////////////////////////////////纠正偏航角计算方向盘角度////////////////////////////////////////////////////////////
		mid_thea = aim_now_thea_tolerance;
		if (fabs(x) * 0.000001 >= fabs(sin(mid_thea)))
		{
			r_yaw = 1e6;
		}
		else
		{
			r_yaw = x / sin(mid_thea);
		}
		tolerance_y = r_yaw - r_yaw * cos(mid_thea) - y;
		//////////////////////////////////////////纠正偏航角计算方向盘角度,-0.5///////////////////////////////////////////////////////
		mid_thea = aim_now_thea_tolerance - yaw_tolerance;
		if (fabs(x) * 0.000001 >= fabs(sin(mid_thea)))
		{
			r_yaw_a = 1e6;
		}
		else
		{
			r_yaw_a = x / sin(mid_thea);
		}
		tolerance_y_a = r_yaw_a - r_yaw_a * cos(mid_thea) - y;
		//////////////////////////////////////////纠正偏航角计算方向盘角度,+0.5///////////////////////////////////////////////////////
		mid_thea = aim_now_thea_tolerance + yaw_tolerance;
		if (fabs(x) * 0.000001 >= fabs(sin(mid_thea)))
		{
			r_yaw_b = 1e6;
		}
		else
		{
			r_yaw_b = x / sin(mid_thea);
		}
		tolerance_y_b = r_yaw_b - r_yaw_b * cos(mid_thea) - y;
		//////////////////////方便后面运算//////////////////////////////////////////////////////////////////////
		if (fabs(r_yaw) < min_r)
		{
			tolerance_y = 10;
		}
		if (fabs(r_yaw_a) < min_r)
		{
			tolerance_y_a = 10;
		}
		if (fabs(r_yaw_b) < min_r)
		{
			tolerance_y_b = 10;
		}
		/////////////////////////////////////如果横向误差都大于yaw_tolerance，则无解//////////////////////////////
		if (fabs(tolerance_y) > y_tolerance && fabs(tolerance_y_a) > y_tolerance && fabs(tolerance_y_b) > y_tolerance)
		{
			return 1;//y向误差太大或者转弯半径小于车辆要求
		}
		//////////////////////////////////////////选择最小的y向误差////////////////////////////////////////////////
		if (fabs(tolerance_y) <= fabs(tolerance_y_a) && fabs(tolerance_y) <= fabs(tolerance_y_b))
		{
			road_yaw[0] = r_yaw;
			if (r_yaw == 1e6)
			{
				road_yaw[1] = x;
			}
			else if (x > 0)
			{
				road_yaw[1] = fabs(r_yaw) * fabs(aim_now_thea_tolerance);//车需要后退
			}
			else
			{
				road_yaw[1] = -fabs(r_yaw) * fabs(aim_now_thea_tolerance);//车需要后退
			}
		}
		else if (fabs(tolerance_y_a) <= fabs(tolerance_y) && fabs(tolerance_y_a) <= fabs(tolerance_y_b))
		{
			road_yaw[0] = r_yaw_a;
			if (r_yaw_a == 1e6)
			{
				road_yaw[1] = x;
			}
			else if (x > 0)
			{
				road_yaw[1] = fabs(r_yaw_a) * fabs(aim_now_thea_tolerance - yaw_tolerance);//车需要后退
			}
			else
			{
				road_yaw[1] = -fabs(r_yaw_a) * fabs(aim_now_thea_tolerance - yaw_tolerance);//车需要后退
			}
		}
		else
		{
			road_yaw[0] = r_yaw_b;
			if (r_yaw_b == 1e6)
			{
				road_yaw[1] = x;
			}
			else if (x > 0)
			{
				road_yaw[1] = fabs(r_yaw_b) * fabs(aim_now_thea_tolerance + yaw_tolerance);//车需要后退
			}
			else
			{
				road_yaw[1] = -fabs(r_yaw_b) * fabs(aim_now_thea_tolerance + yaw_tolerance);//车需要后退
			}
		}
		return 0;
	}

	char Calc_CmCm(double x_now, double y_now, double thea_now,
		double x_aim, double y_aim, double thea_aim,
		double car_r_limit, double road_yaw[6])
	{
		//////////////////////////////车辆坐标转换处理/////////////////////////////////////////////////////////////////////
		double y = (y_aim - y_now) * cos(thea_now) - (x_aim - x_now) * sin(thea_now);// 坐标转换后 Y向的实际偏差
		double x = (x_aim - x_now) * cos(thea_now) + (y_aim - y_now) * sin(thea_now);
		double theta = 0;
		theta = mod2pi(thea_aim - thea_now);
		//////////////////////////////距离太短，不做输出求解,保护后面数据不越界//////////////////////////////////////////////////////////
		if (fabs(x) < 0.05 || fabs(y) > 1000 || fabs(x) > 1000)
		{
			return 2;//距离目标小于5cm
		}
		double a;
		double b;
		double c;
		double sqr;
		//double car_r_limit = 5.5;
		double R_1 = 0;
		double sin_theta_first_1 = 0;//车辆第一个旋转角度的sin值
		double cos_theta_first_1 = 0;//车辆第一个旋转角度的cos值
		double angle_1 = .0;
		//double distance1 = 0;
		double R_2 = 0;
		double sin_theta_first_2 = 0;//车辆第一个旋转角度的sin值
		double cos_theta_first_2 = 0;//车辆第一个旋转角度的cos值
		double angle_2 = .0;
		//double distance2 = 0;
		///////////////////计算公式///////////
		a = pow(cos(theta) + 1, 2) + pow(sin(theta), 2) - 4;
		b = 2 * x * sin(theta) - 2 * y * (cos(theta) + 1);
		c = x * x + y * y;
		sqr = pow(b, 2) - 4 * a * c;
		if (fabs(a) < 0.0001)
		{
			a = 0;
		}
		///////////////////////////////////////均方根为负值则无解//////////////////////////////////////////////
		if (sqr < 0)
		{
			return 1;
		}
		///////////////////////////////////////a为0，b为0则无解//////////////////////////////////////////////
		if (a == 0 && b == 0)
		{
			return 1;
		}
		/////////////////////////////////////a=0，单独计算求解//////////////////////////////////////
		if (a == 0)
		{
			if (fabs(c * 0.000001) >= fabs(b))   //保护除数
			{
				R_1 = 1e6;
			}
			else
			{
				R_1 = -c / b;
			}
			if (fabs(R_1) < car_r_limit)
			{
				return 1;//无解
			}
			sin_theta_first_1 = x / 2 / (R_1)+sin(theta) / 2;
			cos_theta_first_1 = -y / 2 / (R_1)+cos(theta) / 2 + 0.5;
			if (GetAntiTrigo(sin_theta_first_1, cos_theta_first_1, angle_1) != 0)//无解
			{
				return 1;
			}
			////////////////////可删除下面两行验证用/////////////////////////////////////////
			//double aaa_x1 = 2 * R_1 * sin(angle_1[0]) - R_1 * sin(0.0);
			//double aaa_y1 = R_1 - 2 * R_1 * cos(angle_1[0]) + R_1 * cos(0.0);
			if (angle_1 * (theta - angle_1) > 0)//两端路必须为同一方向
			{
				return 1;//无解
			}
			//double distance1 = fabs(R_1 * angle_1[0]) + fabs(R_1 * (theta - angle_1[0]));//可以删除，验证用
			road_yaw[0] = R_1;
			road_yaw[1] = R_1 * angle_1;
			road_yaw[2] = -R_1;
			road_yaw[3] = -R_1 * (theta - angle_1);
			return 0;
		}

		////一元二次方程两个解，求解1
		if (fabs((-b + pow(sqr, 0.5)) * 0.000002) >= fabs(a))   //保护除数
		{
			R_1 = 1e6;
		}
		else
		{
			R_1 = (-b + pow(sqr, 0.5)) / 2 / a;
		}
		if (fabs(R_1) >= car_r_limit)
		{
			sin_theta_first_1 = x / 2 / (R_1)+sin(theta) / 2;
			cos_theta_first_1 = -y / 2 / (R_1)+cos(theta) / 2 + 0.5;
			if (GetAntiTrigo(sin_theta_first_1, cos_theta_first_1, angle_1) != 0)
			{
				return 1;
			}
			//double distance1 = fabs(R_1 * angle_1[0]) + fabs(R_1 * (theta - angle_1[0]));
		}
		////一元二次方程两个解，求解2
		if (fabs((-b - pow(sqr, 0.5)) * 0.000002) >= fabs(a))   //保护除数
		{
			R_2 = 1e6;
		}
		else
		{
			R_2 = (-b - pow(sqr, 0.5)) / 2 / a;
		}
		if (fabs(R_2) >= car_r_limit)
		{
			sin_theta_first_2 = x / 2 / (R_2)+sin(theta) / 2;
			cos_theta_first_2 = -y / 2 / (R_2)+cos(theta) / 2 + 0.5;
			if (GetAntiTrigo(sin_theta_first_2, cos_theta_first_2, angle_2) != 0)
			{
				return 1;
			}
			//double distance2 = fabs(R_2 * angle_2[0]) + fabs(R_2 * (theta - angle_2[0]));
		}
		if ((fabs(R_1) >= car_r_limit) && (fabs(R_2) >= car_r_limit))//两个解都有，取同一方向
		{
			if ((angle_1) * (theta - angle_1) <= 0)
			{
				road_yaw[0] = R_1;
				road_yaw[1] = R_1 * angle_1;
				road_yaw[2] = -R_1;
				road_yaw[3] = -R_1 * (theta - angle_1);
			}
			else if ((angle_2) * (theta - angle_2) <= 0)
			{
				road_yaw[0] = R_2;
				road_yaw[1] = R_2 * angle_2;
				road_yaw[2] = -R_2;
				road_yaw[3] = -R_2 * (theta - angle_2);
			}
			else
			{
				return 1;
			}
		}
		else if (fabs(R_1) < car_r_limit && fabs(R_2) < car_r_limit)
		{
			return 1;
		}

		else if (fabs(R_1) >= car_r_limit)
		{
			if ((angle_1) * (theta - angle_1) <= 0)
			{
				road_yaw[0] = R_1;
				road_yaw[1] = R_1 * angle_1;
				road_yaw[2] = -R_1;
				road_yaw[3] = -R_1 * (theta - angle_1);
			}
			else
			{
				return 1;
			}
		}
		else
		{
			if ((angle_2) * (theta - angle_2) <= 0)
			{
				road_yaw[0] = R_2;
				road_yaw[1] = R_2 * angle_2;
				road_yaw[2] = -R_2;
				road_yaw[3] = -R_2 * (theta - angle_2);
			}
			else
			{
				return 1;
			}
		}
		return 0;
	}


	char Calc_CpSm(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim,
		double car_r_limit, double bias, double* road_R_S)
	{
		//double l_back = 0;  //直线后退的值
		double y_1 = 0;     //走完第一段弧的y值
		double x_1 = 0;     //走完第一段弧的y值
							//////////////////////////////车辆坐标转换处理/////////////////////////////////////////////////////////////////////
		double thea_aim1 = theta_aim / 180 * 3.1415926;
		//double theta_now1 = (theta_now - theta_aim) / 180 * 3.1415926;
		double y = (y_now - y_aim) * cos(thea_aim1) - (x_now - x_aim) * sin(thea_aim1);// 坐标转换后 Y向的实际偏差
		double x = (x_now - x_aim) * cos(thea_aim1) + (y_now - y_aim) * sin(thea_aim1);
		double theta_ang = theta_now - theta_aim;
		//////把角度调整为-180到180之间
		theta_ang = fmod(theta_ang, 360);
		if (theta_ang > 180)
			theta_ang = theta_ang - 360;
		else if (theta_ang < -180)
			theta_ang = theta_ang + 360;
		double theta_rad = theta_ang * pi / 180;
		if (fabs(theta_ang) > 60)  //当前角度与目标角度差值较大，cs不一定是最优解,60参数可以调整
		{
			return 10;
		}
		////////////////////////////////允许y存在误差，查看是否有解？？？？？？////////////////////////////////////////////////
		//偏差允许有解：方向盘先向左打，然后把车辆调成和目标一致，查看圆弧直线是否需要换挡
		double car_r_limit_m = car_r_limit;
		x_1 = x - car_r_limit_m * sin(theta_rad);
		y_1 = y + car_r_limit_m * cos(theta_rad) - car_r_limit_m;   //把方向盘角度调整为和目标一致！
		if (car_r_limit_m * (-theta_rad) * x_1 > 0)//圆弧的解需换挡
		{
			if (y * y_1 < 0)   //最小转弯半径也照样调整不到位,一个在y=0的左边，一个在右边
			{
				if (fabs(y_1) < bias && fabs(x_1) <= 6.0)//是否在误差允许范围内 并且未超出对向限制空间
				{
					road_R_S[0] = car_r_limit_m;
					road_R_S[1] = car_r_limit_m * (-theta_rad);
					road_R_S[2] = 1e6;
					road_R_S[3] = -x_1;
					return 0;
				}
				else
				{
					return 2;//无解
				}
			}
		}
		else
		{
			//无需处理
		}
		//偏差允许有解：方向盘向右打，然后把车辆调成和目标一致，查看圆弧直线是否需要换挡
		car_r_limit_m = -car_r_limit;
		x_1 = x - car_r_limit_m * sin(theta_rad);
		y_1 = y + car_r_limit_m * cos(theta_rad) - car_r_limit_m;   //把方向盘角度调整为和目标一致！
		if (car_r_limit_m * (-theta_rad) * x_1 > 0)//圆弧的解需换挡
		{
			if (y * y_1 < 0)   //最小转弯半径也照样调整不到位
			{
				if (fabs(y_1) < bias && fabs(x_1) <= 6.0)//是否在误差允许范围内 并且未超出对向限制空间
				{
					road_R_S[0] = car_r_limit_m;
					road_R_S[1] = car_r_limit_m * (-theta_rad);
					road_R_S[2] = 1e6;
					road_R_S[3] = -x_1;
					return 0;
				}
				else
				{
					return 2;//无解
				}
			}
		}
		else
		{
			//无需处理
		}
		if (x < 0.05 && fabs(y)>19)         //越界保护考虑
		{
			return 1;//越界不在求解
		}
		if (cos(theta_rad) > 0.999999) //如果车辆当前角度与目标角度一致，那么其他公式有解，保护不越界
		{
			return 2;//无解
		}

		road_R_S[0] = y / (1 - cos(theta_rad));//计算转弯半径

		if (fabs(road_R_S[0]) < car_r_limit)//最小转弯半径小于车辆实际转弯半径
		{
			return 2;//无解
		}
		x_1 = x - road_R_S[0] * sin(theta_rad);
		y_1 = y + road_R_S[0] * cos(theta_rad) - road_R_S[0];   //把方向盘角度调整为和目标一致！
		double dx = (fusion.parkingSpaceInfo.ParkingSpaceType == 3) ? 7.0 : 6.0;
		if (road_R_S[0] * (-theta_rad) * x_1 > 0)//圆弧的解需换挡，则有解
		{
			if (fabs(x_1) > dx || (x_1 < 0))
			{
				return 2;//无解,不允许太靠前了去调整
			}
		}
		else
		{
			return 9;//无解，按照正常不应该到这一步，因为先计算的无需换挡
		}

		road_R_S[1] = road_R_S[0] * (-theta_rad);
		road_R_S[2] = 1e6;
		road_R_S[3] = -x_1;
		return 0;
	}

	char Calc_CmSm(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim,
		double car_r_limit, double bias, double* road_R_S)
	{
		//double l_back = 0;  //直线后退的值
		double y_1 = 0;     //走完第一段弧的y值
		double x_1 = 0;     //走完第一段弧的y值
		//////////////////////////////车辆坐标转换处理/////////////////////////////////////////////////////////////////////
		double theta_ang = theta_now - theta_aim;
		theta_ang = fmod(theta_ang, 360);
		if (theta_ang > 180)
			theta_ang = theta_ang - 360;
		else if (theta_ang < -180)
			theta_ang = theta_ang + 360;
		double theta_rad = theta_ang / 180 * 3.1415926;
		// 车辆方向与车位夹角
		if (fabs(theta_ang) > 60)  //当前角度与目标角度差值较大，cs不一定是最优解,60参数可以调整
		{
			return 10;
		}

		double thea_aim1 = theta_aim / 180 * pi;
		double y = (y_now - y_aim) * cos(thea_aim1) - (x_now - x_aim) * sin(thea_aim1);
		double x = (x_now - x_aim) * cos(thea_aim1) + (y_now - y_aim) * sin(thea_aim1);

		//偏差允许有解：方向盘先向左打，然后把车辆调成和目标一致，查看圆弧直线是否需要换挡
		double car_r_limit_m = car_r_limit;
		x_1 = x - car_r_limit_m * sin(theta_rad);
		y_1 = y + car_r_limit_m * cos(theta_rad) - car_r_limit_m;   //把方向盘角度调整为和目标一致！
		if (car_r_limit_m * (-theta_rad) * x_1 < 0)//圆弧的解无需换挡
		{
			if (y * y_1 < 0)   //最小转弯半径也照样调整不到位,一个在y=0的左边，一个在右边
			{
				if (fabs(y_1) < bias)//是否在误差允许范围内
				{
					road_R_S[0] = car_r_limit_m;
					road_R_S[1] = car_r_limit_m * (-theta_rad);
					road_R_S[2] = 1e6;
					road_R_S[3] = -x_1;
					return 3;
				}
				else
				{
					return 2;//无解
				}
			}
		}
		else
		{
			//无需处理
		}
		//偏差允许有解：方向盘向右打，然后把车辆调成和目标一致，查看圆弧直线是否需要换挡
		car_r_limit_m = -car_r_limit;
		x_1 = x - car_r_limit_m * sin(theta_rad);
		y_1 = y + car_r_limit_m * cos(theta_rad) - car_r_limit_m;   //把方向盘角度调整为和目标一致！
		if (car_r_limit_m * (-theta_rad) * x_1 < 0)//圆弧的解无需换挡
		{
			if (y * y_1 < 0)   //最小转弯半径也照样调整不到位
			{
				if (fabs(y_1) < bias)//是否在误差允许范围内
				{
					road_R_S[0] = car_r_limit_m;
					road_R_S[1] = car_r_limit_m * (-theta_rad);
					road_R_S[2] = 1e6;
					road_R_S[3] = -x_1;
					return 3;
				}
				else
				{
					return 2;//无解
				}
			}
		}
		else
		{
			//无需处理
		}
		if (x < 0.05 && fabs(y)>19)         //越界保护考虑
		{
			return 1;//越界不在求解
		}
		if (cos(theta_rad) > 0.999999) //如果车辆当前角度与目标角度一致，那么其他公式有解，保护不越界
		{
			return 2;//无解
		}

		road_R_S[0] = y / (1 - cos(theta_rad));//计算转弯半径

		if (fabs(road_R_S[0]) < car_r_limit)//最小转弯半径小于车辆实际转弯半径
		{
			return 2;//无解
		}
		x_1 = x - road_R_S[0] * sin(theta_rad);
		y_1 = y + road_R_S[0] * cos(theta_rad) - road_R_S[0];   //把方向盘角度调整为和目标一致！
		if (road_R_S[0] * (-theta_rad) * x_1 > 0)//圆弧的解如果需要换挡，则无解
		{
			return 2;//无解
		}

		road_R_S[1] = road_R_S[0] * (-theta_rad);
		road_R_S[2] = 1e6;
		road_R_S[3] = -x_1;
		return 0;
	}


	char Calc_CmCmSm(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim,
		double car_r_limit, double road_yaw[6])
	{
		double x, y, phi;
		RsPath ccs_path;
		CoordinateTrans(x, y, phi, x_aim, y_aim, theta_aim, x_now, y_now, theta_now);
		x = x / car_r_limit;
		y = y / car_r_limit;
		int theta_car = 0;
		if (fusion.ParkInMode == 1)
		{
			theta_car = -1;
		}
		else
		{
			theta_car = 1;
		}
		if (CmCmSm(x, y, phi, ccs_path, theta_car))
		{
			for (int i = 0; i < 3; i++)
			{
				if (ccs_path.rspath_type[i] == St)
				{
					road_yaw[i * 2] = 1e6;
				}
				else if (ccs_path.rspath_type[i] == Le)
				{
					road_yaw[i * 2] = car_r_limit;
				}
				else if (ccs_path.rspath_type[i] == Ri)
				{
					road_yaw[i * 2] = -car_r_limit;
				}
			}
			road_yaw[1] = car_r_limit * ccs_path.t;
			road_yaw[3] = car_r_limit * ccs_path.u;
			// 
			road_yaw[5] = car_r_limit * ccs_path.v;
			ccs_path.lenth = ccs_path.lenth * car_r_limit;
			return 0;
		}
		return 2;
	}

	//2022.12.27
	char Calc_CpCpSm(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim,
		double car_r_limit, double* road_yaw)
	{
		double x, y, phi;
		RsPath ccs_path;
		CoordinateTrans(x, y, phi, x_aim, y_aim, theta_aim, x_now, y_now, theta_now);
		x = x / car_r_limit;
		y = y / car_r_limit;
		int theta_car = 0;
		if (fusion.ParkInMode == 1)
		{
			theta_car = -1;
		}
		else
		{
			theta_car = 1;
		}
		if (CpCpSm(x, y, phi, ccs_path, theta_car))
		{
			for (int i = 0; i < 3; i++)
			{
				if (ccs_path.rspath_type[i] == St)
				{
					road_yaw[i * 2] = 1e6;
				}
				else if (ccs_path.rspath_type[i] == Le)
				{
					road_yaw[i * 2] = car_r_limit;
				}
				else if (ccs_path.rspath_type[i] == Ri)
				{
					road_yaw[i * 2] = -car_r_limit;
				}
			}
			road_yaw[1] = car_r_limit * ccs_path.t;
			road_yaw[3] = car_r_limit * ccs_path.u;
			//
			road_yaw[5] = car_r_limit * ccs_path.v;
			ccs_path.lenth = ccs_path.lenth * car_r_limit;
			double dx = (fusion.parkingSpaceInfo.ParkingSpaceType == 1) ? 5.5 : 7.5;
			if (fabs(road_yaw[5]) > dx)
			{
				return 2;
			}
			else
			{
				return 0;
			}
		}
		return 2;
	}

	char Calc_CmCpSm(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim,
		double car_r_limit, double road_yaw[6])
	{
		double x, y, phi;
		RsPath ccs_path;
		CoordinateTrans(x, y, phi, x_aim, y_aim, theta_aim, x_now, y_now, theta_now);
		x = x / car_r_limit;
		y = y / car_r_limit;
		int theta_car = 0;
		if (fusion.ParkInMode == 1)
		{
			theta_car = -1;
		}
		else
		{
			theta_car = 1;
		}
		if (CmCpSm(x, y, phi, ccs_path, theta_car))
		{
			for (int i = 0; i < 3; i++)
			{
				if (ccs_path.rspath_type[i] == St)
				{
					road_yaw[i * 2] = 1e6;
				}
				else if (ccs_path.rspath_type[i] == Le)
				{
					road_yaw[i * 2] = car_r_limit;
				}
				else if (ccs_path.rspath_type[i] == Ri)
				{
					road_yaw[i * 2] = -car_r_limit;
				}
			}
			road_yaw[1] = car_r_limit * ccs_path.t;
			road_yaw[3] = car_r_limit * ccs_path.u;
			//
			road_yaw[5] = car_r_limit * ccs_path.v;
			ccs_path.lenth = ccs_path.lenth * car_r_limit;
			if (fabs(road_yaw[1]) > 0.3)
			{
				return 0;
			}
			return 0;
		}
		return 2;
	}

	char Calc_CpCmSm(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim,
		double car_r_limit, double road_yaw[6], double road_yaw2[6])
	{
		//////////////////////////////车辆坐标转换处理/////////////////////////////////////////////////////////////////////
		double ph = mod2pi(theta_aim - theta_now);
		double phi = mod2pi(theta_now);
		// 起点start坐标系在基坐标系下的方向余弦矩阵(Z轴旋转，因为点在Z = 0平面，因此绕Z旋转仍然在Z = 0平面)
		// dcm*x 表示将基坐标中的x表示到旋转后的坐标系中，即计算坐标旋转后各向量在新坐标中的表示
		double x1 = (x_aim - x_now) * cos(phi) + (y_aim - y_now) * sin(phi);
		double y1 = (y_aim - y_now) * cos(phi) - (x_aim - x_now) * sin(phi);

		RsPath path_six;
		double x1_six = x1 / car_r_limit;
		double y1_six = y1 / car_r_limit;
		//int ErrorCode = 100;
		int theta_car = 0;
		if (fusion.ParkInMode == 1)
		{
			theta_car = -1;
		}
		else
		{
			theta_car = 1;
		}
		//LOG_WARNING("Six_theta_car=%d", theta_car);
		//fflush(bydapa::common::Log::GetInstance()->fileptr());
		if (CpCmSm(x1_six, y1_six, ph, path_six, theta_car))
		{
			for (int index_six_CCS = 0; index_six_CCS < 3; index_six_CCS++)
			{
				if (path_six.rspath_type[index_six_CCS] == St)
				{
					road_yaw[index_six_CCS * 2] = 1e6;
				}

				else if (path_six.rspath_type[index_six_CCS] == Le)
				{
					road_yaw[index_six_CCS * 2] = car_r_limit;
				}

				else if (path_six.rspath_type[index_six_CCS] == Ri)
				{
					road_yaw[index_six_CCS * 2] = -car_r_limit;
				}
			}

			road_yaw[1] = path_six.t * car_r_limit;
			road_yaw[3] = path_six.u * car_r_limit;
			road_yaw[5] = path_six.v * car_r_limit;
			path_six.lenth = path_six.lenth * car_r_limit;
			if (fabs(road_yaw[3] + road_yaw[5]) > 8)
			{
				return 2;
			}

		}
		else
		{
			return 2;
		}
		/////////////////////////////////////////////////////////////////////////////////////
		double delta_six1 = 0;
		if (fabs(road_yaw[0]) > 100000 || fabs(road_yaw[0]) < rmin_vertical)
		{
			delta_six1 = 0;
		}
		else
		{
			delta_six1 = atan(vehicle_parameters.WB / road_yaw[0]);
		}
		road_yaw2[0] = road_yaw[0];
		road_yaw2[1] = road_yaw[1];
		road_yaw2[2] = road_yaw[2];
		road_yaw2[3] = road_yaw[3];
		road_yaw2[4] = road_yaw[4];
		road_yaw2[5] = road_yaw[5];
		double road_yaw_siximag[6] = { 0,0,0,0,0,0 };
		double road_yaw_siximagthree[6] = { 0,0,0,0,0,0 };
		double Start_imag[3] = { 0,0,0 };

		Start_imag[0] = x_now;
		Start_imag[1] = y_now;
		Start_imag[2] = theta_now;

		for (int index_D_six = 2; index_D_six > 0; index_D_six--)
		{
			double g_x = 0;
			double g_y = 0;
			double g_th = 0;
			VehicleDynamic(g_x, g_y, g_th, road_yaw[1] + theta_car * index_D_six * 0.1, delta_six1);

			int roadsecond_six = 1;
			roadsecond_six = Calc_CmSm(g_px, g_py, g_pth * 180 / pi, x1, y1, ph * 180 / pi, rmin_vertical, .0, road_yaw_siximagthree);
			if ((roadsecond_six == 0) && (fabs(road_yaw[1] + theta_car * index_D_six * 0.1) > 0.3))
			{
				road_yaw_siximag[0] = road_yaw[0];
				road_yaw_siximag[1] = road_yaw[1] + theta_car * index_D_six * 0.1;
				road_yaw_siximag[2] = road_yaw_siximagthree[0];
				road_yaw_siximag[3] = road_yaw_siximagthree[1];
				road_yaw_siximag[4] = road_yaw_siximagthree[2];
				road_yaw_siximag[5] = road_yaw_siximagthree[3];

				if ((GetOrCheckPathPoints(Start_imag, road_yaw_siximag, 3, 1)) && (fabs(road_yaw_siximag[5]) > 1))
				{
					road_yaw2[0] = road_yaw_siximag[0];
					road_yaw2[1] = road_yaw_siximag[1];
					road_yaw2[2] = road_yaw_siximag[2];
					road_yaw2[3] = road_yaw_siximag[3];
					road_yaw2[4] = road_yaw_siximag[4];
					road_yaw2[5] = road_yaw_siximag[5];
					return 1;
				}

			}
		}
		if (fabs(road_yaw[1]) > 0.3)
		{
			return 0;
		}
		return 2;
	}

	bool CpSpCp_Head(const double &x, const double &y, const double &phi, const double &curr_min,double* road_yaw)
	{
		int road_LR = 0;
		if(y > 0.05)
		{
			road_LR = 1;
		}
		else if(y < -0.05)
		{
			road_LR = -1;
		}
		
		if(phi != 0 && fabs(phi) <= pi/2 && y * phi < 0)
		{
			double r_cs = (fabs(y) / (1 - cos(phi)) > 1e6) ? 1e6 : (y/ (1 - cos(phi)));
			if(fabs(r_cs) >= 5.9)
			{
				double x_cs = (x - r_cs * sin(phi));
				if(fabs(r_cs < 1e6) && x_cs <= 0 )
				{
					road_yaw[0] = r_cs;
					road_yaw[1] = -r_cs * phi;
					road_yaw[2] = 1e6;
					road_yaw[3] = -x_cs;
					road_yaw[4] = road_LR*curr_min ;
					road_yaw[5] = curr_min * (5*pi/180);
					return true; 
				}
			}
		}
		return false;
	}

	bool CalcCpSpCp(const double *start_p, const double* end, const double r_max40,double *road_yaw) //dynamic
	{
		double start[3] = {start_p[0],start_p[1],start_p[2]};
		int road_LR = 0;
		if(start[1] > 0)
		{
			road_LR = 1;
		}
		else if(start[1] < 0)
		{
			road_LR = -1;
		}
		double r_cs = r_max40;
		double phi_r = -5*pi/180 *road_LR;
		double x_cs3 = -r_cs * sin(fabs(phi_r));
		double y_cs3 = road_LR*r_cs*(1-cos(phi_r));	
		double endm[3] = {x_cs3,y_cs3,phi_r};
		double cal_x=fabs(start[1]-fabs(y_cs3))/tan(fabs(phi_r));
		double real_x =fabs(start[0]-x_cs3);
		double x = .0, y = .0, phi = .0;
		CoordinateTrans(x,y,phi,endm[0],endm[1],endm[2],start[0],start[1],start[2]);
	
		int roadflag_CS = 1;
		int roadflag_C = 1;
		double rmin_vertical_change = rmin_vertical;
		if(fabs(start_p[0]) < 3.5){
			rmin_vertical_change = rmin_vertical + 1;
		}
		if (fabs(start[0])-fabs(x_cs3) > 0)
		{
			roadflag_CS=Calc_CmSm(start[0], start[1], start[2] * 180 / pi, endm[0], endm[1], endm[2] * 180 / pi, rmin_vertical_change, 0.03, road_yaw);
			if (roadflag_CS == 0 )
			{
				road_yaw[4] = road_LR*r_max40 ;
				road_yaw[5] = r_max40 * (5*pi/180);
				return true;
			}
			else
			{
				roadflag_C=Calc_Cm(start[0], start[1], start[2], endm[0], endm[1], endm[2], 0, rmin_vertical_change, 0.03, road_yaw);
				if (roadflag_C == 0 )
				{
					road_yaw[2] = road_LR*r_max40;
					road_yaw[3] = r_max40 * (5*pi/180);
					return true;
				}
			}		
			if (fabs(real_x - cal_x) < 0.05 && fabs(start[2]-phi)< 1*pi/180 && fabs(start[1]) > fabs(y_cs3))
			{
				//std::cout<<"sc"<<std::endl;
				road_yaw[0] = 1e6;
				road_yaw[1] = sqrt(x*x+y*y);
				road_yaw[2] = road_LR*r_max40 ;
				road_yaw[3] = r_max40*(5*pi/180);
				return true;
			}
		}
		else 
		{
			if(start[1] * start[0] < 0 && fabs(start[1]) < fabs(y_cs3) &&  fabs(-r_cs * sin(fabs(start[2]))-start[0]) < 0.03 && r_cs*(1-cos(start[2]))-fabs(start[1]) < 0.03)
			{			
				road_yaw[0] = road_LR*r_max40 ;
				road_yaw[1] = r_max40*fabs(start[2]);
			}
			
		}
		return false;
	}


	char Calc_NCC(double x_now, double y_now, double theta_now,
		double x_aim, double y_aim, double theta_aim , const double CC_r)  // 1 success
	{
		// std::cout << "x_now = " << x_now << y_now << theta_now << std::endl;
		if ( fabs(theta_now) > pi/180 * 10 || fabs(x_now) < 2 || fabs(x_now) > 6 || fabs(y_now) > 0.4 || fabs(y_now) < 0.05){ // 0 fail
			return 0;
		}
		if (fabs(x_now) > 3.5 && fabs(x_now) < 4.5 && fabs(y_now) < 0.15){ // 0 fail
			return 0;
		}
		if (fabs(x_now) > 1.5 && fabs(x_now) < 3.5 && fabs(y_now) < 0.05){ // 0 fail
			return 0;
		}
		double road_yaw[6] = {.0};
		double start[3] = {x_now, y_now, theta_now};
		double adjustment = .0;
		double y_now_change = .0;
		int flag_ncc = -1;
		int flag_ncc_c = -1;
		int flag_ncc_cs = -1;
		if(y_now < 0){
			adjustment = 0.01;
		}else{
			adjustment = -0.01;
		}
		int max_size = round(fabs(y_now)/0.01);
		for(int i = 0;i <= max_size ; i = i+1){
			y_now_change = y_now + adjustment *i;
			flag_ncc = Calc_CmCm(x_now, y_now_change , theta_now, 0, 0, 0, CC_r, road_yaw);
			if (flag_ncc == 0) // 0 success
			{				
				if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) 
				{				
					return GetOrCheckPathPoints(start, road_yaw, 3, 0);
				}
			}
		}

		if(fabs(x_now) < 3.5 && CC_r < 26){
			for(int i = 0;i <= max_size; i = i+1){
				y_now_change = y_now + adjustment *i;
				flag_ncc_c = Calc_Cm(x_now, y_now_change, theta_now, 0, 0, 0, 0.03, CC_r, 0.3, road_yaw);
				flag_ncc_cs = Calc_CmSm(x_now, y_now_change, theta_now * 180 / pi,0,0,0, CC_r, 0.03, road_yaw);
				if (flag_ncc_c == 0 || flag_ncc_cs == 0) // 0 success
				{				
					if (GetOrCheckPathPoints(start, road_yaw, 3, 1)) 
					{				
						return GetOrCheckPathPoints(start, road_yaw, 3, 0);
					}
				}
			}
		}

		return 0;
	}
	

}