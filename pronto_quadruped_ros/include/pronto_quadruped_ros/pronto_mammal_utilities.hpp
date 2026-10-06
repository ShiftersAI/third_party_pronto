#include <pronto_quadruped_commons/feet_contact_forces.h>
#include <pronto_quadruped_commons/leg_vector_map.h>
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <array>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include "pronto_quadruped_commons/feet_contact_forces.h"
#include "pronto_quadruped_commons/feet_jacobians.h"
#include "pronto_quadruped_commons/forward_kinematics.h"

//include pinocchio's stuff
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <iostream>


#define FB_DOF 7
#define FB_VEL 6
#define LEG_NUM 4
#define PRONTO_JNT_P_LEG 3
namespace pronto_pinocchio
{
    using JointState = pronto::quadruped::JointState;
    using LegID = pronto::quadruped::LegID;
    using Vector3d = Eigen::Vector3d;

    typedef Eigen::Matrix<double, 19, 1> JointStatePinocchio;
    typedef Eigen::Matrix<double, 18, 1> JointVelocityPinocchio;
    typedef Eigen::Matrix<double, 6, 18> CompleteJac;
    typedef Eigen::Matrix<double, 3, 3> LinVelJac;


    // create a class that directly manage the pinocchio structure and compite the estimated ground reaction force
    class Pinocchio_Feet_Force : public pronto::quadruped::FeetContactForces
    {
        public:

            Pinocchio_Feet_Force()
            {}
            Pinocchio_Feet_Force(pinocchio::Model mod,int ker,int DOF,std::vector<int> conv_pro2pin,const std::array<std::string,LEG_NUM>& foot_names):
            model_(mod),
            data_(pinocchio::Data(mod)),
            ker_(ker),
            DOF_(DOF)
            {
                int step = DOF/LEG_NUM;
                conv_pro2pin_.resize(conv_pro2pin.size());
                for(size_t i = 0; i<conv_pro2pin_.size();i++)
                {
                    conv_pro2pin_[i]=conv_pro2pin[i];
                }
                for(size_t i = 0; i < LEG_NUM; i++)
                {
                    conv_leg_pro2pin_[i] = conv_pro2pin_[i*step]/PRONTO_JNT_P_LEG;
                }
                std::cerr<<"jnt leg map is [ "<<conv_leg_pro2pin_[0]<<" "<<conv_leg_pro2pin_[1]<<" "<<conv_leg_pro2pin_[2]<<" "<<conv_leg_pro2pin_[3]<<" ]"<<std::endl;
                q_pin_ = Eigen::VectorXd::Zero(DOF_ + FB_DOF);
                dq_pin_ = Eigen::VectorXd::Zero(DOF_ + FB_VEL);
                ddq_pin_ = Eigen::VectorXd::Zero(DOF_ + FB_VEL);
                tau_rnea_ = Eigen::VectorXd::Zero(DOF_);
                tau_msr_ = Eigen::VectorXd::Zero(DOF_);
                Jac_.resize(6,DOF_+FB_VEL);
                // foot frames, indexed by LegID (LF, RF, LH, RH)
                for(size_t i = 0; i < LEG_NUM; i++)
                {
                    if(!model_.existFrame(foot_names[i]))
                    {
                        throw std::invalid_argument("foot frame '" + foot_names[i] + "' not found in the URDF");
                    }
                    foot_frame_id_[i] = model_.getFrameId(foot_names[i]);
                }
                for(auto &jnt_ptr:model_.names)
                {

                    if(jnt_ptr != "universe" && jnt_ptr != "root_joint")
                    {
                        std::cerr<<"jnt name is "<<jnt_ptr<<std::endl;
                        pin_jnt_name_.push_back(jnt_ptr);

                    }
                }
            };
            ~Pinocchio_Feet_Force(){};
            bool get_foot_vel(const LegID& leg, Eigen::Vector3d& foot_vel)
            {
                pinocchio::FrameIndex leg_id;
                pinocchio::Motion a;
                foot_vel.setZero();
                leg_id = foot_frame_id_[leg];
                pinocchio::updateFramePlacement(model_,data_,leg_id);


                a = pinocchio::getFrameVelocity(model_,data_,leg_id,pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
                foot_vel = a.linear();
                return true;
            }
            // function to set state dim
            void set_state_dim()
            {
                // set up the pinocchio data
                std::cerr<< "DOF IS "<< DOF_<<std::endl;
                q_pin_ = Eigen::VectorXd::Zero(DOF_ + FB_DOF);
                dq_pin_ = Eigen::VectorXd::Zero(DOF_ + FB_VEL);
                ddq_pin_ = Eigen::VectorXd::Zero(DOF_ + FB_VEL);
                tau_rnea_ = Eigen::VectorXd::Zero(DOF_);
                tau_msr_ = Eigen::VectorXd::Zero(DOF_);
                Jac_.resize(6,DOF_+FB_VEL);
            }

            // set up the state, it is needed to enable the update

            void set_State(
                    std::map<std::string,std::tuple<double,double,double>> jnt_stt,
                    Quaterniond orient,
                    Vector3d xd,
                    Vector3d xdd,
                    Vector3d omega,
                    Vector3d omegad
            );


            // update everything, kinematic, jacobian and RNEA
            bool update_All();

            void q_size()
            {
                std::cerr<<" the q dimension is "<< q_pin_.size() <<" and q_dot is "<< dq_pin_.size() << std::endl;
            }

            // compute the ground reaction force and save it in foot_grf
            bool getFootGRF(
                const JointState& q,
                const JointState& qd,
                const JointState& tau,
                const Quaterniond& orient,
                const LegID& leg,
                Vector3d& foot_grf,
                const JointState& qdd = JointState::Zero(),
                const Vector3d& xd = Vector3d::Zero(),
                const Vector3d& xdd = Vector3d::Zero(),
                const Vector3d& omega = Vector3d::Zero(),
                const Vector3d& omegad = Vector3d::Zero()
            ) override;



            bool getLegJacobian(
                    const LegID& leg,
                    LinVelJac& Jac,
                    bool world
                );

            void get_w2b_R(Eigen::Matrix3d &R)
            {
                R = R_w2b_;
            };


            // retrun the chosen foot forward kineatic
            Eigen::Vector3d get_FK(LegID leg)
            {

                pinocchio::SE3 T;
                Eigen::Vector3d f_pos;
                pinocchio::FrameIndex leg_id = foot_frame_id_[leg];
                T = data_.oMf[leg_id];
                f_pos = T.translation();
                return f_pos;

            };


        std::vector<int> conv_leg_pro2pin_ = {0,0,0,0};

        private:
            pinocchio::Model model_;
            pinocchio::Data data_;
            std::vector<int> conv_pro2pin_;
            int ker_,DOF_;
            int leg_count_ = 0;
            bool update_need_ = false;
            Eigen::VectorXd q_pin_,dq_pin_,ddq_pin_, tau_rnea_, tau_msr_ ;
            pinocchio::Data::Matrix6x Jac_;
            std::vector<std::string> pin_jnt_name_ = {};
            Eigen::Matrix3d R_w2b_;
            int updated_ = 0;
            std::array<pinocchio::FrameIndex,LEG_NUM> foot_frame_id_ = {};


    };

    class Pinocchio_Jacobian : public pronto::quadruped::FeetJacobians
    {
        public:
            Pinocchio_Jacobian()
            {};
            Pinocchio_Jacobian(
                Pinocchio_Feet_Force* pin_ff
                ):
            pin_ff_(pin_ff)
            {};

            ~Pinocchio_Jacobian(){};
            Eigen::Vector3d get_foot_vel(LegID leg)
            {
                Eigen::Vector3d f_v;

                pin_ff_->get_foot_vel(leg,f_v);
                return f_v;
            };
           pronto::quadruped::FootJac getFootJacobian(const pronto::quadruped::JointState& q, const pronto::quadruped::LegID& leg) override;
           pronto::quadruped::FootJac getFootJacobianAngular(const pronto::quadruped::JointState& q,const pronto::quadruped::LegID& leg) override;

        private:
            Pinocchio_Feet_Force* pin_ff_;
    };

    class Pinocchio_FK : public pronto::quadruped::ForwardKinematics
    {
        public:
            Pinocchio_FK(){};
            Pinocchio_FK(
                Pinocchio_Feet_Force* pin_ff
            ):
            pin_ff_(pin_ff)
            {};
            ~Pinocchio_FK(){};

            Eigen::Vector3d getFootPos(const JointState& q, const LegID& leg) override;

            Eigen::Matrix3d getFootOrientation(const JointState& q, const LegID& leg) override;
        private:
            Pinocchio_Feet_Force* pin_ff_;
    };
};
