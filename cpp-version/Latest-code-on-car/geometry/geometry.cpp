#include "../common/global_variable.h"
#include "../common/global_function.h"

namespace byd_apa_plan
{
	void ParkingVertical()
	{
		if (app.APA_Park_Function != 2)
		{
			if (fusion.ParkInMode == 0)
			{
				if (RearParking(Start, End))
				{
					
					pathpoint = PathPointsCoordinateTrans(pathpoint, end_to_fus);
					PathPointNormalization();
					PathOnlyOneNow = IsPathOnlyOne();
					prk_inclined_dynamic_plan = (PathOnlyOneNow == 0 && (dynamic_plan_flag || (PlanMulti > 0 && Gears == 2 && fabs(plan.coordinate[0].Curvature) > 0.001)));
					if (PathOnlyOneNow == 1 || prk_inclined_dynamic_plan)
					{
						StartRecordFlag = 1;
					}
				}
				else
				{
					PathOnlyOneNow = 0;
					prk_inclined_dynamic_plan = false;
					AstarOrGeo = 1;
				}
			}
			else
			{
				if(HeadParking(Start, End))
				{
					pathpoint = PathPointsCoordinateTrans(pathpoint, end_to_fus);
					PathPointNormalization();
					PathOnlyOneNow = IsPathOnlyOne();
					if (PathOnlyOneNow == 1)
					{
						StartRecordFlag = 1;
					}
				}
				else
				{
					PathOnlyOneNow = 0;
					AstarOrGeo = 1;
				}
			}

		}
		else
		{
			plan.PlanningStatus = 2;
		}
	}

	void ParkingLevel()
	{
		if (app.APA_Park_Function == 2)
		{

			if (ParkingOutLevel(start_to_fus, end_to_fus) != 0)
			{
				PathPointNormalization();
				//fprintf(pF, " parking out success ");
			}
			else
			{
				plan.PlanningStatus = 2;
			}
		}
		else
		{
			if ((ParkingInLevel(start_to_fus, end_to_fus) != 0))
			{

				PathPointNormalization(); 
				/// 2024.1.10 ///
				if (Path_2 == 1)
				{
					PathOnlyOneNow = 1;
					StartRecordFlag = 1;
				}
				else 
				{
					PathOnlyOneNow = 0;
					StartRecordFlag = 0;
				}
				/// 2024.1.10 ///
			}
			else
			{
				AstarOrGeo = 1;
			}
		}
	}

	void ParkingInclined()
	{
		if (app.APA_Park_Function != 2)
		{
			if (fusion.ParkInMode == 0)
			{
				if (RearParking(Start, End))
				{
					pathpoint = PathPointsCoordinateTrans(pathpoint, end_to_fus);
					PathPointNormalization();
					PathOnlyOneNow = IsPathOnlyOne();
					prk_inclined_dynamic_plan = ((PathOnlyOneNow == 0) && (PlanForwardFirst == 3 || PlanBackFirst == 3) && fusion.parkingSpaceInfo.ParkingSpaceType == 3);
					if (PathOnlyOneNow == 1 || (prk_inclined_dynamic_plan && Gears == 2))
					{
						StartRecordFlag = 1;
					}
				}
				else
				{
					PathOnlyOneNow = 0;
					prk_inclined_dynamic_plan = false;
					AstarOrGeo = 1;
				}
			}
			else
			{
				if(HeadParking(Start, End))
				{
					pathpoint = PathPointsCoordinateTrans(pathpoint, end_to_fus);
					PathPointNormalization();
					PathOnlyOneNow = IsPathOnlyOne();
					prk_inclined_dynamic_plan = (PathOnlyOneNow == 0 && dynamic_plan_flag);
					if(PathOnlyOneNow == 1 || prk_inclined_dynamic_plan)
					{
						StartRecordFlag = 1;
					}
				}
				else
				{
					PathOnlyOneNow = 0;
					prk_inclined_dynamic_plan = false;
					AstarOrGeo = 1;
				}
			}
		}
		else
		{
			plan.PlanningStatus = 2;
		}
	}
}