#include "../common/global_variable.h"
#include "../common/global_function.h"

namespace byd_apa_plan
{
	void ParkingVertical()
	{
		if(app.APA_Park_Function == 4)
		{
			if(HeadParkingOut(start_to_fus, end_to_fus))
			{
				plan.PlanningStatus = 1;
				PathPointNormalization();
				if(index_request == 1)
				{
					StartRecordFlag = 1;
				}
			}
			else{
				plan.PlanningStatus = 2;
			}
		}
		else if(app.APA_Park_Function == 5)
		{
			if(RearParkingOut(start_to_fus, end_to_fus))
			{
				plan.PlanningStatus = 1;
				PathPointNormalization();
				if(index_request == 1)
				{
					StartRecordFlag = 1;
				}
			}
			else{
				plan.PlanningStatus = 2;
			}
		}
		else if(app.APA_Park_Function == 1)
		{
			if (fusion.ParkInMode == 1)
			{
				if(HeadParking(start_to_prk, end_to_prk))
				{
					plan.PlanningStatus = 1;
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
			else
			{
				if (RearParking(start_to_prk, end_to_prk))
				{
					plan.PlanningStatus = 1;
					pathpoint = PathPointsCoordinateTrans(pathpoint, end_to_fus);
					PathPointNormalization();
					PathOnlyOneNow = IsPathOnlyOne();
					FLAG_dynamic_plan_advanced = (PathOnlyOneNow == 0 && ((PlanMulti > 0 && Gears == 2 && fabs(plan.coordinate[0].Curvature) > 0.001)));
					if (PathOnlyOneNow == 1 || FLAG_dynamic_plan_advanced)
					{
						StartRecordFlag = 1;
					}
				}
				else
				{
					PathOnlyOneNow = 0;
					FLAG_dynamic_plan_advanced = false;
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
				plan.PlanningStatus = 1;
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
				plan.PlanningStatus = 1;
				PathPointNormalization(); 
				/// 2024.1.10 ///
				if (LevelDynamic == 1)
				{
					PathOnlyOneNow = 1;
					StartRecordFlag = 1;
				}
				else 
				{
					PathOnlyOneNow = 0;
					StartRecordFlag = 0;
					PlanOnlyOne = 0;
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
		if(app.APA_Park_Function == 4)
		{
			if(HeadParkingOut(start_to_fus, end_to_fus))
			{
				plan.PlanningStatus = 1;
				PathPointNormalization();
				if(index_request == 1)
				{
					StartRecordFlag = 1;
				}
			}
			else{
				plan.PlanningStatus = 2;
			}
		}
		else if(app.APA_Park_Function == 5)
		{
			if(RearParkingOut(start_to_fus, end_to_fus))
			{
				plan.PlanningStatus = 1;
				PathPointNormalization();
				if(index_request == 1)
				{
					StartRecordFlag = 1;
				}
			}
			else{
				plan.PlanningStatus = 2;
			}
		}
		else if(app.APA_Park_Function == 1)
		{
			if (fusion.ParkInMode == 1)
			{
				if(HeadParking(start_to_prk, end_to_prk))
				{
					plan.PlanningStatus = 1;
					pathpoint = PathPointsCoordinateTrans(pathpoint, end_to_fus);
					PathPointNormalization();
					PathOnlyOneNow = IsPathOnlyOne();
					FLAG_dynamic_plan_advanced = (PathOnlyOneNow == 0 && PlanBackwardFirst == 2 && Gears == 1);
					if(PathOnlyOneNow == 1 || (FLAG_dynamic_plan_advanced))
					{
						StartRecordFlag = 1;
					}
				}
				else
				{
					PathOnlyOneNow = 0;
					FLAG_dynamic_plan_advanced = false;
					AstarOrGeo = 1;
				}
			}
			else
			{
				if (RearParking(start_to_prk, end_to_prk))
				{
					plan.PlanningStatus = 1;
					pathpoint = PathPointsCoordinateTrans(pathpoint, end_to_fus);
					PathPointNormalization();
					PathOnlyOneNow = IsPathOnlyOne();
					FLAG_dynamic_plan_advanced = ((PathOnlyOneNow == 0) && (PlanForwardFirst == 3 || PlanBackwardFirst == 3) && (Gears == 2));
					if (PathOnlyOneNow == 1 || FLAG_dynamic_plan_advanced)
					{
						StartRecordFlag = 1;
					}
				}
				else
				{
					PathOnlyOneNow = 0;
					FLAG_dynamic_plan_advanced = false;
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