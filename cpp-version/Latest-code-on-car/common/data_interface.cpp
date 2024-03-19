#include "global_variable.h"

namespace byd_apa_plan
{
#ifndef DEBUG_PC
	boost::shared_ptr<bclcpp::Publisher> publisher_path;
	boost::shared_ptr<bclcpp::Publisher> publisher_status;
	boost::shared_ptr<bclcpp::Publisher> publisher_complete;

	void handle_message_test_subscriber_appstatus(const boost::shared_ptr<bclcpp::Message> message)
	{
		byd_interfaces_msg_app_status* msg = (byd_interfaces_msg_app_status*)message->data();
		app.APAStatus = msg->APAStatus;
	}

	void handle_message_test_subscriber_appnav(const boost::shared_ptr<bclcpp::Message> message)
	{

		byd_interfaces_msg_app_nav_cmd* msg = (byd_interfaces_msg_app_nav_cmd*)message->data();
		app.APA_nav_cmd = msg->APA_nav_cmd;		
	}

	void handle_message_test_subscriber_appmode(const boost::shared_ptr<bclcpp::Message> message)
	{
		byd_interfaces_msg_app_mode* msg = (byd_interfaces_msg_app_mode*)message->data();
		//fusion.ParkInMode = msg->ParkInMode;
	}

	void handle_message_test_subscriber_apptype(const boost::shared_ptr<bclcpp::Message> message)
	{	
		byd_interfaces_msg_app_park_type* msg = (byd_interfaces_msg_app_park_type*)message->data();
		app.APA_Park_Function = msg->APA_Park_Function;
	}

	void handle_message_test_subscriber_ctrlrequest(const boost::shared_ptr<bclcpp::Message> message)
	{
		byd_interfaces_msg_vehiclecontrol_planj_reqinfo* msg = (byd_interfaces_msg_vehiclecontrol_planj_reqinfo*)message->data();
		control.ObsUssInfo = msg->ObsUssInfo;
		control.PlanningRequest = msg->PlanningRequest;
	}

	void handle_message_test_subscriber_ctrlcount(const boost::shared_ptr<bclcpp::Message> message)
	{
		byd_interfaces_msg_vehiclecontrol_planj_interfaceinfo* msg = (byd_interfaces_msg_vehiclecontrol_planj_interfaceinfo*)message->data();
		control.PlanningRequestCount = msg->PlanningRequestCount;
	}

	void handle_message_test_subscriber_fuspark(const boost::shared_ptr<bclcpp::Message> message)
	{	
		byd_interfaces_msg_fusion_parkingtarget* msg = (byd_interfaces_msg_fusion_parkingtarget*)message->data();
		fusion.depth_block = msg->depth_block;
		fusion.distance_01 = msg->distance_01;
		fusion.distance_03 = msg->distance_03;
		fusion.distance_23 = msg->distance_23;
		#ifndef HEAD_VERTICAL
		fusion.ParkInMode = msg->ParkInMode;
		#endif
		fusion.Theta = msg->Theta;
		fusion.TraceParkingID_Cam = msg->TraceParkingID_Cam;
		fusion.TraceParkingID_USS = msg->TraceParkingID_USS;
		fusion.position.X = msg->position.X;
		fusion.position.Y = msg->position.Y;
		fusion.position.Heading = msg->position.Heading;
		fusion.parkingSpaceInfo.P0_X = msg->ParkingSpace.P0_X;
		fusion.parkingSpaceInfo.P0_Y = msg->ParkingSpace.P0_Y;
		fusion.parkingSpaceInfo.P1_X = msg->ParkingSpace.P1_X;
		fusion.parkingSpaceInfo.P1_Y = msg->ParkingSpace.P1_Y;
		fusion.parkingSpaceInfo.P2_X = msg->ParkingSpace.P2_X;
		fusion.parkingSpaceInfo.P2_Y = msg->ParkingSpace.P2_Y;
		fusion.parkingSpaceInfo.P3_X = msg->ParkingSpace.P3_X;
		fusion.parkingSpaceInfo.P3_Y = msg->ParkingSpace.P3_Y;
		fusion.parkingSpaceInfo.ParkingSpaceValid = msg->ParkingSpace.ParkingSpaceValid;
		fusion.parkingSpaceInfo.Width = msg->ParkingSpace.Width;
		fusion.parkingSpaceInfo.ParkingSpaceType = msg->ParkingSpace.ParkingSpaceType;
		if (fusion.parkingSpaceInfo.ParkingSpaceType == 4)
		{
			fusion.parkingSpaceInfo.ParkingSpaceType = 1;
		}

		if ((fusion.depth_block > 410) && (fusion.parkingSpaceInfo.ParkingSpaceType == 1))
		{
			fusion.depth_block = 410;
		}
	}

	void handle_message_test_subscriber_fusmap(const boost::shared_ptr<bclcpp::Message> message)
	{
		if (Read_obstmap == 1)
		{
			byd_interfaces_msg_fusion_gridmap* msg = (byd_interfaces_msg_fusion_gridmap*)message->data();
			memcpy(fusion.freeSpaceCell, msg->freeSpacefusion, sizeof(FreeSpaceCell) * 62500);
			Read_obstmap = 0;
		}
		else
		{

		}

	}

	void handle_message_test_subscriber_nav(const boost::shared_ptr<bclcpp::Message> message)
	{
		byd_interfaces_msg_apa_nav* msg = (byd_interfaces_msg_apa_nav*)message->data();
		calculation.nav_pos_X = msg->nav_pos_x;
		calculation.nav_pos_Y = msg->nav_pos_y;
		calculation.nav_heading = msg->nav_heading;
	}

	void handle_message_test_subscriber_topSM(const boost::shared_ptr<bclcpp::Message> message)
	{
		byd_interfaces_msg_topM *msg = (byd_interfaces_msg_topM *)message->data();
		Top_ProceState = msg->top_proce_sts;
	}

	int data_subscriber(std::string msg)
	{
		auto subscriber_appstatus = bclcpp::create_subscription("app/status", &byd_interfaces_msg_app_status_desc, handle_message_test_subscriber_appstatus);
		auto subscriber_appnav = bclcpp::create_subscription("app/nav/cmd", &byd_interfaces_msg_app_nav_cmd_desc, handle_message_test_subscriber_appnav);
		auto subscriber_appmode = bclcpp::create_subscription("app/mode", &byd_interfaces_msg_app_mode_desc, handle_message_test_subscriber_appmode);
		auto subscriber_apptype = bclcpp::create_subscription("app/park/type", &byd_interfaces_msg_app_park_type_desc, handle_message_test_subscriber_apptype);
		auto subscriber_ctrlrequest = bclcpp::create_subscription("vehiclecontrol/planj/reqinfo", &byd_interfaces_msg_vehiclecontrol_planj_reqinfo_desc, handle_message_test_subscriber_ctrlrequest);
		auto subscriber_ctrlcount = bclcpp::create_subscription("vehiclecontrol/planj/interfaceinfo", &byd_interfaces_msg_vehiclecontrol_planj_interfaceinfo_desc, handle_message_test_subscriber_ctrlcount);
		auto subscriber_fuspark = bclcpp::create_subscription("fusion/parkingtarget", &byd_interfaces_msg_fusion_parkingtarget_desc, handle_message_test_subscriber_fuspark);
		auto subscriber_fusmap = bclcpp::create_subscription("fusion/gridmap", &byd_interfaces_msg_fusion_gridmap_desc, handle_message_test_subscriber_fusmap);
		auto subscriber_nav = bclcpp::create_subscription("apa/nav", &byd_interfaces_msg_apa_nav_desc, handle_message_test_subscriber_nav);
		auto subscriber_topSM = bclcpp::create_subscription("topM", &byd_interfaces_msg_topM_desc, handle_message_test_subscriber_topSM);
		if ((nullptr != subscriber_appstatus)
			&& (nullptr != subscriber_appnav)
			&& (nullptr != subscriber_appmode)
			&& (nullptr != subscriber_apptype)
			&& (nullptr != subscriber_ctrlrequest)
			&& (nullptr != subscriber_ctrlcount)
			&& (nullptr != subscriber_fuspark)
			&& (nullptr != subscriber_fusmap)
			&& (nullptr != subscriber_nav)
			&& (nullptr != subscriber_topSM))
		{
			printf("bclcpp::create_plan_subscription ok\n");
			bclcpp::spin();
		}
		else
		{
			printf("bclcpp::create_plan_subscription error\n");
		}

		return 0;
	}

	void path_publisher(std::string msg)
	{
		if (nullptr != publisher_path)
		{		
			auto message = publisher_path->create_message();
			if (nullptr == message)
			{
				return;
			}

			byd_interfaces_msg_geometry_pathinfo_ctrl* data = (byd_interfaces_msg_geometry_pathinfo_ctrl*)(message->data());
			memset(data, 0, sizeof(byd_interfaces_msg_geometry_pathinfo_ctrl));
			data->TrajectoryLength = plan.TrajectoryLength;
			for (int path_a = 0; path_a < 300; path_a++)
			{
				data->coordinate[path_a].X = plan.coordinate[path_a].X;
				data->coordinate[path_a].Y = plan.coordinate[path_a].Y;
				data->coordinate[path_a].Yaw = plan.coordinate[path_a].Yaw;
				data->coordinate[path_a].Curvature = plan.coordinate[path_a].Curvature;
				data->coordinate[path_a].rajectoryDirection = plan.coordinate[path_a].rajectoryDirection;
			}
			if (!publisher_path->publish(message))
			{
				printf("bclcpp::publish_path failed\n");
			}
		}
		else
		{
			printf("bclcpp::create_publisher_publish_path error\n");
		}
	}

	void status_publisher(std::string msg)
	{
		if (nullptr != publisher_status)
		{
			auto message = publisher_status->create_message();
			if (nullptr == message)
			{
				return;
			}

			byd_interfaces_msg_geometry_statusinfo_ctrl* data = (byd_interfaces_msg_geometry_statusinfo_ctrl*)(message->data());
			memset(data, 0, sizeof(byd_interfaces_msg_geometry_statusinfo_ctrl));
			data->PlanningStatus = plan.PlanningStatus;
			if (!publisher_status->publish(message))
			{
				printf("bclcpp::publish_status failed\n");
			}

		}
		else
		{
			printf("bclcpp::create_publisher_publish_status error\n");
		}
	}

	void complete_publisher(std::string msg)
	{
		if (nullptr != publisher_complete)
		{
			auto message = publisher_complete->create_message();
			if (nullptr == message)
			{
				return;
			}

			byd_interfaces_msg_geometry_completeinfo_ctrl* data = (byd_interfaces_msg_geometry_completeinfo_ctrl*)(message->data());
			memset(data, 0, sizeof(byd_interfaces_msg_geometry_completeinfo_ctrl));
			data->IsPlanningCompleted = plan.IsPlanningCompleted;
			if (!publisher_complete->publish(message))
			{
				printf("bclcpp::publish_complete failed\n");
			}
		}
		else
		{
			printf("bclcpp::create_publisher_publish_complete error\n");
		}
	}
#endif
}
