#include "urdf/model.h"
#include <Eigen/Core>
#include <Eigen/Dense>

#include <string>
#include <vector>
#include <array>
#include <map>




namespace pronto
{

    const std::vector<std::string> quadruped_leg_id =
    {
        "LF",
        "LH",
        "RF",
        "RH"
    };

    const std::vector<std::string> quadruped_jnt_id =
    {
        "HAA",
        "HFE",
        "KFE"
    };

    enum Axis_Direction
    {
        x,y,z,all,error,not_init
    };

    // this class will be used by the legodom manager to adapt the feet_contact_force and the velocity correction function to the
    // desired model
    class Model_Parser
    {
        public:
            // build the model from urdf file; joint_names lists the URDF joints in leg order
            // LF, RF, LH, RH, each as HAA, HFE, KFE (a missing HAA allows 2-joint legs)
            Model_Parser(std::string urdf_path, const std::vector<std::string>& joint_names);

            ~Model_Parser(){};

            Axis_Direction get_ker_dir();
            
            void get_imu_base_tranform(std::string base_link, std::string imu_link, Eigen::Isometry3d& tranformation);

            int get_robot_DOF()
            {
                if(urdf_usable_)
                    return jnts_find_*quadruped_leg_id.size();
                else
                {
                    throw std::logic_error("NOT VALID MODEL");
                    return 0;
                }
            }
            inline void get_jnt_names(std::vector<std::string>& jnt_name)
            {
                get_jnt_list();
                jnt_name = jnt_names_;
                jnt_name.resize(jnt_names_.size());
                for(size_t i = 0; i< jnt_names_.size();i++)
                    jnt_name[i] = jnt_names_[i];
            };




        protected:

            void get_jnt_list()
            {
                auto all_jnt = model_.joints_;
                urdf::Vector3 axis;
                std::array<double,3> ax;
                for(auto &jnt:all_jnt)
                {
                    axis = jnt.second->axis;
                    if(axis.x != 0.0 || axis.y != 0.0 || axis.z != 0.0)
                    {
                        jnt_names_.push_back(jnt.first);
                        ax = {axis.x,axis.y,axis.z};
                        jnt_ptr_.push_back(ax);
                    }
                }
            }


        private:

            urdf::Model model_;

            bool urdf_usable_ = false;

            std::vector<std::string> jnt_names_;
            std::vector<std::array<double,3>> jnt_ptr_;

            int jnts_find_ = 0;
            std::map<std::string,std::map<std::string,std::array<double,3>>> kin_tree_;


    };
};
