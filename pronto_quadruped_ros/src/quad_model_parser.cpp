#include "pronto_quadruped_ros/quad_model_parser.hpp"
#include "rclcpp/rclcpp.hpp"
namespace pronto
{

    Model_Parser::Model_Parser(
        std::string urdf_path,
        const std::vector<std::string>& joint_names
    )
    {
        // offset of each leg in joint_names
        const std::map<std::string,size_t> leg_offset = {{"LF",0},{"RF",3},{"LH",6},{"RH",9}};
        int parse_jnt_leg_count;

        std::map<std::string,std::array<double,3>> leg_map;
        if(joint_names.size() != leg_offset.size()*quadruped_jnt_id.size())
        {
            RCLCPP_ERROR(rclcpp::get_logger("URDF_parser"),"Expected %zu joint names, got %zu",leg_offset.size()*quadruped_jnt_id.size(),joint_names.size());
            urdf_usable_ = false;
        }
        else if(!model_.initString(urdf_path))
        {
            RCLCPP_INFO(rclcpp::get_logger("URDF_MODEL_PARSER"),"The Model parsing throw an error");
            urdf_usable_ = false;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("URDF_MODEL_PARSER"),"The Model parsing  is performed corctly");
            urdf_usable_ = true;
            for(size_t ind = 0; ind < quadruped_leg_id.size(); ind ++)
            {
                // look the leg's joints up by their exact configured names
                parse_jnt_leg_count = 0;
                for(size_t j = 0; j < quadruped_jnt_id.size(); j++)
                {
                    auto model_jnt = model_.joints_.find(joint_names[leg_offset.at(quadruped_leg_id[ind]) + j]);
                    if(model_jnt == model_.joints_.end())
                    {
                        continue;
                    }
                    auto axis = model_jnt->second->axis;
                    if(axis.x != 0.0 || axis.y != 0.0 || axis.z != 0.0)
                    {
                        parse_jnt_leg_count ++;
                        leg_map.insert({quadruped_jnt_id[j],{axis.x,axis.y,axis.z}});
                    }
                }

                if(parse_jnt_leg_count == 2 || parse_jnt_leg_count == 3)
                {
                    if(!jnts_find_)
                        jnts_find_ = parse_jnt_leg_count;
                    else if(jnts_find_ != parse_jnt_leg_count)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("URDF_parser"),"All leg must have the same joints number per leg");
                        urdf_usable_=false;
                    }
                    kin_tree_.insert({quadruped_leg_id[ind],leg_map});
                    leg_map = std::map<std::string,std::array<double,3>>();
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("URDF_parser"),"The leg %s has %d joints, its not correct",quadruped_leg_id[ind].c_str(),parse_jnt_leg_count);
                    urdf_usable_=false;
                }
            // jnt_names_.resize(0);
            // jnt_ptr_.resize(0);
            // get_jnt_list();
            }
        }
    };


    Axis_Direction Model_Parser::get_ker_dir()
    {
        if(jnts_find_ == 3)
        {
            return Axis_Direction::all;
        }
        else
        {
            Axis_Direction ax = Axis_Direction::not_init;
            std::array<double,3>hfe_dir,kfe_dir;
            std::array<double,3> x_dir = {1.0,0.0,0.0},y_dir={0.0,1.0,0.0},z_dir={0.0,0.0,1.0},x_dir_n = {-1.0,0.0,0.0},y_dir_n={0.0,-1.0,0.0},z_dir_n={0.0,0.0,-1.0};
            for(auto &leg_map:kin_tree_)
            {
                try
                {
                    hfe_dir = leg_map.second[quadruped_jnt_id[1]];
                    kfe_dir = leg_map.second[quadruped_jnt_id[2]];
                }
                catch(std::out_of_range& e)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("URDF_Parser"),"Raised error %s, check the urdf file", e.what());
                    return Axis_Direction::error;
                }
                if(hfe_dir != kfe_dir)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("URDF_Parser"),"The joints direction of leg %s are not correct",leg_map.first.c_str());
                    return Axis_Direction::error;
                }


                if(ax == Axis_Direction::not_init)
                {
                    if(hfe_dir == x_dir || hfe_dir == x_dir_n)
                        ax = Axis_Direction::x;
                    else if(hfe_dir == y_dir || hfe_dir == y_dir_n)
                        ax = Axis_Direction::y;
                    else if(hfe_dir == z_dir || hfe_dir == z_dir_n)
                        ax = Axis_Direction::z;
                    else
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("URDF_Parser"),"The joints direction of leg %s are not correct",leg_map.first.c_str());
                        return Axis_Direction::error;
                    }
                }
                else
                {
                    Axis_Direction act_dir;
                    if(hfe_dir == x_dir || hfe_dir == x_dir_n)
                        act_dir = Axis_Direction::x;
                    else if(hfe_dir == y_dir || hfe_dir == y_dir_n)
                        act_dir = Axis_Direction::y;
                    else if(hfe_dir == z_dir || hfe_dir == y_dir_n )
                        act_dir = Axis_Direction::z;
                    else
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("URDF_Parser"),"The joints direction of leg %s are not correct",leg_map.first.c_str());
                        return Axis_Direction::error;
                    }
                    if(act_dir != ax)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("URDF_Parser"),"The joints direction of leg %s are not correct",leg_map.first.c_str());
                        return Axis_Direction::error;
                    }
                }

            }
            return ax;
        }
        return Axis_Direction::error;
    }

    void Model_Parser::get_imu_base_tranform(std::string base_link, std::string imu_link, Eigen::Isometry3d& tranformation)
    {
        tranformation = Eigen::Isometry3d();
        auto imu_link_model =  model_.links_.find(imu_link);
        if( imu_link_model != model_.links_.end())
        {
           if(imu_link_model->second->parent_joint->parent_link_name == base_link)
           {
                auto position = imu_link_model->second->parent_joint->parent_to_joint_origin_transform.position;
                auto rotation = imu_link_model->second->parent_joint->parent_to_joint_origin_transform.rotation;

                tranformation.translation() << position.x,position.y,position.z;
                Eigen::Quaterniond q(rotation.w,rotation.x,rotation.y,rotation.z);
                tranformation.linear() = q.toRotationMatrix();
                
           }   

           else
            throw std::runtime_error("URDF ERROR: BASE Declaration");
        }
        else
            throw std::runtime_error("URDF ERROR: IMU Declaration");
    }

};
