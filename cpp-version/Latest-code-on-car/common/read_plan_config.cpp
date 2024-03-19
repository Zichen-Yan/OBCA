#include "global_variable.h"
#include "global_function.h"

namespace byd_apa_plan
{
    bool ReadArrayFromConfig(std::vector<double> &value_x, std::vector<double> &value_y,
                             const nlohmann::json &j, const std::vector<std::string> &key_x, const std::vector<std::string> &key_y)
    {
        //std::cout << "**********array config************\n";
        if(key_x.size() == 2 && key_y.size() == 2)
        {
            const nlohmann::json& curr_x = j.at(key_x[0]).at(key_x[1]);
            const nlohmann::json& curr_y = j.at(key_y[0]).at(key_y[1]);
            if(curr_x.is_array() && curr_y.is_array())
            {
                //std::cout << key_x[0]<< "_" << key_x[1] << std::endl;
                for(const auto& element : curr_x)
                {
                    value_x.push_back(element.get<double>());
                }
                for(const auto& element : curr_y)
                {
                    value_y.push_back(element.get<double>());
                }
                //std::cout << "value_x.size() = " << value_x.size() << std::endl;
                //std::cout << "value_y.size() = " << value_y.size() << std::endl;
                return (value_x.size() != 0) && (value_y.size() != 0) && (value_x.size() == value_y.size());
            }
        }
        else if (key_x.size() == 3 && key_y.size() == 3)
        {
            const nlohmann::json& curr_x = j.at(key_x[0]).at(key_x[1]).at(key_x[2]);
            const nlohmann::json& curr_y = j.at(key_y[0]).at(key_y[1]).at(key_y[2]);
            if(curr_x.is_array() && curr_y.is_array())
            {
                //std::cout << key_x[0]<< "_" << key_x[1] << "_" << key_x[2] << std::endl;
                for(const auto& element : curr_x)
                {
                    value_x.push_back(element.get<double>());
                }
                for(const auto& element : curr_y)
                {
                    value_y.push_back(element.get<double>());
                }
                //std::cout << "value_x.size() = " << value_x.size() << std::endl;
                //std::cout << "value_y.size() = " << value_y.size() << std::endl;
                return (value_x.size() != 0) && (value_y.size() != 0) && (value_x.size() == value_y.size());
            }
        }
        return false;
    }
    bool ReadLogConfig(const nlohmann::json &j)
	{
		FLAG_log = j.at("log").at("log_enable").get<bool>();
		return true;
	}

    bool ReadVehicleConfig(const nlohmann::json &j)
    {
        vehicle_parameters.WB = j.at("vehicle").at("wheel_base").get<double>();
        vehicle_parameters.W = j.at("vehicle").at("width").get<double>();
        vehicle_parameters.LF = j.at("vehicle").at("length_front").get<double>();
        vehicle_parameters.LB = j.at("vehicle").at("length_back").get<double>();
        vehicle_parameters.MAX_STEER = j.at("vehicle").at("max_steer").get<double>();

        rmin_vertical= j.at("vehicle").at("rmin_vertical").get<double>();
        rmin_level = j.at("vehicle").at("rmin_parallel").get<double>();

        std::vector<std::string> key_x = {"vehicle","vehicle_model_x"};
        std::vector<std::string> key_y = {"vehicle","vehicle_model_y"};
        if(!ReadArrayFromConfig(Car_x, Car_y, j, key_x, key_y))
        {
            //std::cout << "false\n";
            return false;
        }
        return true;
    }

    bool ReadParkConfig(const nlohmann::json &j)
    {
        prk_width_vertical = j.at("park").at("vertical").at("park_width_vertical").get<double>();
        prk_length_vertical = j.at("park").at("vertical").at("park_length_vertical").get<double>();
        prk_back_rear = j.at("park").at("vertical").at("park_back_rear").get<double>();
        prk_back_head = j.at("park").at("vertical").at("park_back_head").get<double>();

        prk_width_level = j.at("park").at("parallel").at("park_width_level").get<double>();
        prk_length_level = j.at("park").at("parallel").at("park_length_level").get<double>();
        prk_back_level = j.at("park").at("parallel").at("park_back_level").get<double>();
        return true;
    }

    bool ReadMapConfig(const nlohmann::json &j)
    {
        pathfind_parameters.MINX = j.at("map").at("min_x").get<double>();
        pathfind_parameters.MINY = j.at("map").at("min_y").get<double>();
        pathfind_parameters.MAXX = j.at("map").at("max_x").get<double>();
        pathfind_parameters.MAXY = j.at("map").at("max_y").get<double>();
        pathfind_parameters.MINYAW = j.at("map").at("min_yaw").get<double>();
        pathfind_parameters.MAXYAW = j.at("map").at("max_yaw").get<double>();
        pathfind_parameters.MOTION_RESOLUTION = j.at("map").at("motion_resolution").get<double>();
        pathfind_parameters.XY_GRID_RESOLUTION = j.at("map").at("xy_grid_resolution").get<double>();
        return true;
    }

    bool ReadHybridAstarConfig(const nlohmann::json &j)
    {
        pathfind_parameters.BACK_COST = j.at("hybridAstar").at("cost").at("back_cost").get<double>();
        pathfind_parameters.H_COST = j.at("hybridAstar").at("cost").at("heuristic_cost").get<double>();
        pathfind_parameters.STEER_CHANGE_COST = j.at("hybridAstar").at("cost").at("steer_change_cost").get<double>();
        pathfind_parameters.STEER_COST = j.at("hybridAstar").at("cost").at("steer_cost").get<double>();
        pathfind_parameters.SB_COST = j.at("hybridAstar").at("cost").at("switch_back_cost").get<double>();

        pathfind_parameters.N_STEER = j.at("hybridAstar").at("search").at("number_of_steer").get<double>();
        pathfind_parameters.YAW_GRID_RESOLUTION = j.at("hybridAstar").at("search").at("yaw_grid_resolution").get<double>();

        std::vector<std::string> key_x = {"hybridAstar","upa_x"};
        std::vector<std::string> key_y = {"hybridAstar","upa_y"};
        if(!ReadArrayFromConfig(upa_x, upa_y, j, key_x, key_y))
        {
            //std::cout << "false\n";
            return false;
        }


        key_x = {"hybridAstar","vertical","map_image_x"};
        key_y = {"hybridAstar","vertical","map_image_y"};
        if(!ReadArrayFromConfig(map_image_x, map_image_y, j, key_x, key_y))
        {
            //std::cout << "false\n";
            return false;
        }


        key_x = {"hybridAstar","vertical","map_image_vertical_x"};
        key_y = {"hybridAstar","vertical","map_image_vertical_y"};
        if(!ReadArrayFromConfig(map_image_vertical_x, map_image_vertical_y, j, key_x, key_y))
        {
            //std::cout << "false\n";
            return false;
        }
        return true;
    }

    bool ReadGeometryConfig(const nlohmann::json &j)
    {
        // vertical
        D_x = j.at("geometry").at("vertical").at("rear_parking").at("toward_space_limited").get<double>();
        W_exp = j.at("geometry").at("vertical").at("rear_parking").at("width_exp").get<double>();
        L_exp = j.at("geometry").at("vertical").at("rear_parking").at("length_exp").get<double>();
        Aim_dy = j.at("geometry").at("vertical").at("rear_parking").at("aim_dy").get<double>();
        dist_to_block = j.at("geometry").at("vertical").at("rear_parking").at("dist_to_block").get<double>();
        //head
        depth_of_park = j.at("geometry").at("vertical").at("head_parking").at("depth_of_park").get<double>();
        safe_dynamic_rmin = j.at("geometry").at("vertical").at("head_parking").at("head_dynamic_plan").at("safe_dynamic_rmin").get<double>();
        
        const nlohmann::json& dist_array =j.at("geometry").at("vertical").at("rear_parking").at("aim_dist");
        if(dist_array.is_array())
        {
            for(const auto& element : dist_array)
            {
                aim_dist.push_back(element.get<double>());
            } 
        }

        // parallel
        index_front = j.at("geometry").at("parallel").at("index_front").get<int>();
        index_back = j.at("geometry").at("parallel").at("index_back").get<int>();

        index_up_down = j.at("geometry").at("parallel").at("index_up_down").get<int>();
        index_front_back = j.at("geometry").at("parallel").at("index_front_back").get<int>();
        index_front_rl = j.at("geometry").at("parallel").at("index_front_rl").get<int>();

        index_front2_back = j.at("geometry").at("parallel").at("index_front2_back").get<int>();
        size_front = j.at("geometry").at("parallel").at("size_front").get<int>();
        size_back = j.at("geometry").at("parallel").at("size_back").get<int>();


        std::vector<std::string>  key_x = {"geometry","parallel","level_back2_imagmap_x"};
        std::vector<std::string>  key_y = {"geometry","parallel","level_back2_imagmap_y"};
        if(!ReadArrayFromConfig(level_back2_imagmap_x, level_back2_imagmap_y, j, key_x, key_y))
        {
            //std::cout << "false\n";
            return false;
        } 


        key_x = {"geometry","parallel","level_back_imagmap_x"};
        key_y = {"geometry","parallel","level_back_imagmap_y"};
        if(!ReadArrayFromConfig(level_back_imagmap_x, level_back_imagmap_y, j, key_x, key_y))
        {
            //std::cout << "false\n";
            return false;
        }


        key_x = {"geometry","parallel","level_down2_imagmap_x"};
        key_y = {"geometry","parallel","level_down2_imagmap_y"};
        if(!ReadArrayFromConfig(level_down2_imagmap_x, level_down2_imagmap_y, j, key_x, key_y))
        {
            //std::cout << "false\n";
            return false;
        }


        key_x = {"geometry","parallel","level_down_imagmap_x"};
        key_y = {"geometry","parallel","level_down_imagmap_y"};
        if(!ReadArrayFromConfig(level_down_imagmap_x, level_down_imagmap_y, j, key_x, key_y))
        {
            //std::cout << "false\n";
            return false;
        }


        key_x = {"geometry","parallel","level_front2_imagmap_x"};
        key_y = {"geometry","parallel","level_front2_imagmap_y"};
        if(!ReadArrayFromConfig(level_front2_imagmap_x, level_front2_imagmap_y, j, key_x, key_y))
        {
            //std::cout << "false\n";
            return false;
        }


        key_x = {"geometry","parallel","level_front_imagmap_x"};
        key_y = {"geometry","parallel","level_front_imagmap_y"};
        if(!ReadArrayFromConfig(level_front_imagmap_x, level_front_imagmap_y, j, key_x, key_y))
        {
            //std::cout << "false\n";
            return false;
        }


        key_x = {"geometry","parallel","level_front_imagmap_x_l"};
        key_y = {"geometry","parallel","level_front_imagmap_y_l"};
        if(!ReadArrayFromConfig(level_front_imagmap_x_l, level_front_imagmap_y_l, j, key_x, key_y))
        {
            //std::cout << "false\n";
            return false;
        }


        key_x = {"geometry","parallel","level_front_imagmap_x_r"};
        key_y = {"geometry","parallel","level_front_imagmap_y_r"};
        if(!ReadArrayFromConfig(level_front_imagmap_x_r, level_front_imagmap_y_r, j, key_x, key_y))
        {
            //std::cout << "false\n";
            return false;
        }


        key_x = {"geometry","parallel","level_up2_imagmap_x"};
        key_y = {"geometry","parallel","level_up2_imagmap_y"};
        if(!ReadArrayFromConfig(level_up2_imagmap_x, level_up2_imagmap_y, j, key_x, key_y))
        {
            //std::cout << "false\n";
            return false;
        }


        key_x = {"geometry","parallel","level_up_imagmap_x"};
        key_y = {"geometry","parallel","level_up_imagmap_y"};
        if(!ReadArrayFromConfig(level_up_imagmap_x, level_up_imagmap_y, j, key_x, key_y))
        {
            //std::cout << "false\n";
            return false;
        }
        return true;
    }

    bool ReadCollisionModelConfig(const nlohmann::json &j)
    {

        std::vector<std::string>  key_x = {"collisionModel","model_0cm_x",};
        std::vector<std::string>  key_y = {"collisionModel","model_0cm_y",};
        if(!ReadArrayFromConfig(Rect_x0, Rect_y0, j, key_x, key_y))
        {
            //std::cout << "false\n";
            return false;
        }


        key_x = {"collisionModel","model_10cm_x",};
        key_y = {"collisionModel","model_10cm_y",};
        if(!ReadArrayFromConfig(Rect_x10, Rect_y10, j, key_x, key_y))
        {
            //std::cout << "false\n";
            return false;
        }


        key_x = {"collisionModel","model_20cm_x",};
        key_y = {"collisionModel","model_20cm_y",};
        if(!ReadArrayFromConfig(Rect_x, Rect_y, j, key_x, key_y))
        {
            //std::cout << "false\n";
            return false;
        }
        return true;
    }

    bool ReadPlanConfig(const std::string filename)
    {
        std::ifstream ifile(filename);
		nlohmann::json j;
		try{
			j = nlohmann::json::parse(ifile);
		}catch (nlohmann::json::parse_error& ex) {
			ifile.close();
			return -1;
		}
        
        if(ReadVehicleConfig(j) && ReadParkConfig(j) && ReadMapConfig(j) && ReadLogConfig(j)
            && ReadHybridAstarConfig(j) && ReadGeometryConfig(j) && ReadCollisionModelConfig(j))
        {
            ifile.close();
            return true;
        }
        return false;
    }
}