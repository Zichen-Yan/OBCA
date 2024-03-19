#include "../common/global_variable.h"
#include "../common/global_function.h"

namespace byd_apa_plan
{
	void ParkingVertical_Astar()
	{
		double time_max = 2.0;
		if (HybridAStar(start_to_fus, end_to_fus,time_max)) //正向规划
		{
			PathPointNormalization();
			PathOnlyOneNow = IsPathOnlyOne();
			if(PathOnlyOneNow)
			{
				StartRecordFlag = 1;
			}
		}
		else
		{
			SEorES = 1;
			time_max = 8.0;
			if (HybridAStar(end_to_fus, start_to_fus,time_max)) //反向规划
			{
				std::reverse(pathpoint.begin(),pathpoint.end());
				PathPointNormalization();
				PathOnlyOneNow = IsPathOnlyOne();
				if(PathOnlyOneNow)
				{
					StartRecordFlag = 1;
				}
			}
			else
			{
				plan.PlanningStatus = 2;
			}
		}
		
	}

	void ParkingLevel_Astar()
	{
		if (app.APA_Park_Function == 1)
		{
			if (ParkingInLevel(start_to_fus, end_to_fus) != 0)
			{
				PathPointNormalization(); // 将得到的路规范化
			}
			else if (small_level_park == 0)
			{
				SEorES = 1;
				double time_max = 8.0;
				pathfind_parameters.H_COST = 1;
				pathfind_parameters.SB_COST = 1.0;

				if (HybridAStar(end_to_fus, start_to_fus,time_max)) //反向规划
				{
					std::reverse(pathpoint.begin(),pathpoint.end());
					PathPointNormalization();  // 将得到的路规范化
				}
				else
				{
					plan.PlanningStatus = 2;
				}
			}
			else
			{
				plan.PlanningStatus = 2;
			}
		}
		else
		{
			plan.PlanningStatus = 2;
		}
	}

	void ParkingOblique_Astar()
	{
		double time_max = 2.0;
		if (HybridAStar(start_to_fus, end_to_fus,time_max)) //正向规划
		{
			PathPointNormalization();
			PathOnlyOneNow = IsPathOnlyOne();
			if (PathOnlyOneNow)
			{
				StartRecordFlag = 1;
			}
		}
		else
		{
			SEorES = 1;
			time_max = 8.0;
			if (HybridAStar(end_to_fus, start_to_fus,time_max)) //反向规划
			{
				std::reverse(pathpoint.begin(),pathpoint.end());
				PathPointNormalization();
				PathOnlyOneNow = IsPathOnlyOne();
				if (PathOnlyOneNow)
				{
					StartRecordFlag = 1;
				}
			}
			else
			{
				plan.PlanningStatus = 2;
			}
		}
	}
}