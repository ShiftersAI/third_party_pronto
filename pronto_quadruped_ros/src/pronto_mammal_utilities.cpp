#include "pronto_quadruped_ros/pronto_mammal_utilities.hpp"
#include "pronto_quadruped_commons/feet_contact_forces.h"
#include "Eigen/Geometry"

#include <iostream>

namespace pronto_pinocchio
{


    /***** Start The Feet Forces fonction definition *****/

    bool Pinocchio_Feet_Force::getLegJacobian(
        const LegID& leg,
        LinVelJac& Jac,
        bool world
    )
    {
        Jac.setZero();
        // get the correct foot frame
        pinocchio::FrameIndex leg_id = foot_frame_id_[leg];

        //get the complete jacobian in world frame
        q_pin_.block<3,1>(3,0) = Eigen::Vector3d::Zero();
        q_pin_(6)  = 1.0;
        pinocchio::computeFrameJacobian(model_,data_,q_pin_,leg_id,pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,Jac_);

        // int leg_num ;
        // // if(leg == LegID::RF)
        // //     leg_num = LegID::LH;
        // // else if(leg == LegID::LH)
        // //     leg_num = LegID::RF;
        // // else
        //     leg_num = leg;
        if(DOF_ == 8)
            Jac.block<3,2>(0,1) = Jac_.block<3,2>(0,6 + conv_leg_pro2pin_[(int)leg]*2);
        else
            Jac = Jac_.block<3,3>(0,6 + conv_leg_pro2pin_[(int)leg]*3);
        // std::cerr<<"the block data LF are "<<std::endl<<q_pin_.block<3,1>(6, 0)<<std::endl;
        // std::cerr << " the jacobian block is "<< std::endl<<Jac<<std::endl;
        // if(!world)
        //     Jac = R_w2b_.inverse() * Jac;
        return true;

    };



    bool Pinocchio_Feet_Force::getFootGRF(
                const JointState& q,
                const JointState& qd,
                const JointState& tau,
                const Quaterniond& orient,
                const LegID& leg,
                Vector3d& foot_grf,
                const JointState& ,
                const Vector3d& xd,
                const Vector3d& xdd,
                const Vector3d& omega,
                const Vector3d& omegad
            )
    {

        if(updated_ == 0)
        {
            // update all pinocchio quantities
            q_pin_.setZero();
            dq_pin_.setZero();
            ddq_pin_.setZero();
            tau_msr_.setZero();


            R_w2b_ = orient.toRotationMatrix();
            q_pin_.block<4,1>(3,0) = orient.coeffs();
            dq_pin_.block<3,1>(0,0) = xd;
            ddq_pin_.block<3,1>(0,0) = xdd;
            dq_pin_.block<3,1>(3,0) = omega;
            ddq_pin_.block<3,1>(3,0) = omegad;

            if(DOF_ == 12)
            {
                for(size_t i = 0; i < pin_jnt_name_.size(); i++)
                {
                    // std::cerr << "the "<< i<<"-th joints name is "<< pin_jnt_name_[i]<<std::endl;
                    q_pin_(i + FB_DOF) = q(conv_pro2pin_[i]);
                    dq_pin_(i + FB_VEL) = qd(conv_pro2pin_[i]);
                    tau_msr_(i) = tau(conv_pro2pin_[i]);
                }
            }
            else if(DOF_ == 8)
            {
                for(size_t i = 0; i < pin_jnt_name_.size(); i++)
                {
                    // std::cerr << "the "<< i<<"-th joints name is "<< pin_jnt_name_[i]<<std::endl;
                    q_pin_(i + FB_DOF) = q(conv_pro2pin_[i]);
                    dq_pin_(i + FB_VEL) = qd(conv_pro2pin_[i]);
                    tau_msr_(i) = tau(conv_pro2pin_[i]);
                }
            }
            update_All();
            updated_++;
        }
        else if(updated_ == 3)
            updated_ = 0;
        else
            updated_++;
        LinVelJac J;
        Eigen::Vector2d tau_leg_2d;
        Eigen::Vector3d tau_leg;
        Eigen::Matrix2d J_2d;
        if(DOF_ == 8)
        {

            J_2d.setZero();
        }
        else
        {
            J.setZero();
        }

        tau_rnea_ = data_.tau;

        // get tau from rnea per joints
        int leg_num ;
        // if(leg == LegID::RF)
        //     leg_num = LegID::LH;
        // else if(leg == LegID::LH)
        //     leg_num = LegID::RF;
        // else
            leg_num = leg;
        if(DOF_ ==  8)
            tau_leg_2d = -tau_rnea_.block<2,1>(conv_leg_pro2pin_[(int)leg]*2 + FB_VEL,0) + tau_msr_.block<2,1>(conv_leg_pro2pin_[(int)leg]*2,0);

        else
            tau_leg = - tau_rnea_.block<3,1>(conv_leg_pro2pin_[(int)leg]*3 + FB_VEL,0) + tau_msr_.block<3,1>(conv_leg_pro2pin_[(int)leg]*3,0);

        // std::cerr << "the rnea trq of jnt "<<leg<<" is "<<tau_rnea_.block<3,1>(leg*3 + FB_VEL,0).transpose()<<std::endl;
        // std::cerr << "the msrd trq of jnt "<<leg<<" is "<<tau_msr_.block<3,1>(leg*3,0).transpose()<<std::endl<<std::endl;
        // std::cerr<<"the measured tau is " << std::endl << tau_msr_ << std::endl << "the rnea tau is " << tau_rnea_ << std::endl;
        // get the Jacobian am compute grf
        this->getLegJacobian(leg,J,true);
        // std::cerr <<"the complete jacobian "<<leg<<"-th is "<<std::endl << J <<std::endl;
        // std::cerr<<"the J is " << std::endl << J << std::endl;
        if(DOF_ == 12)
        {
            // std::cerr<<"the "<<leg<<"-th jacobian transpose is "<<std::endl<<J.transpose().inverse()<<std::endl;
            // std::cerr<<"the "<<leg<<"-th torque is "<<tau_leg.transpose()<<std::endl;
            foot_grf = - J.transpose().inverse()*tau_leg;
            // std::cerr<<"the "<<leg<<"-th torque is "<<foot_grf.transpose()<<std::endl;


        }
        else
        {

            foot_grf.setZero();
            if(ker_ == 0)
                J_2d = J.block<2,2>(1,1);
            else if(ker_ == 2)
                J_2d = J.block<2,2>(0,1);
            else if(ker_ == 1)
            {
                J_2d.block<1,2>(0,0) = J.block<1,2>(0,1);
                J_2d.block<1,2>(1,0) = J.block<1,2>(2,1);
            }
            // std::cerr << "the 2d jacobian is "<<std::endl<< J_2d.transpose() << " the nverse jac is " << std::endl<< J_2d.transpose().inverse()<<std::endl << " and tau diff is " << tau_leg_2d << std::endl;;

            Eigen::Vector2d grf_2d = -J_2d.transpose().inverse()*tau_leg_2d;
            // std::cerr<< "foot "<< leg << "has GRF" << grf_2d(0)<< " " << grf_2d(1)  << std::endl;
            if(ker_ == 0)
            {
                foot_grf(1) = grf_2d(0);
                foot_grf(2) = grf_2d(1);
            }
            else if(ker_ == 1)
            {
                foot_grf(0) = grf_2d(0);
                foot_grf(2) = grf_2d(1);
            }
            else if(ker_ == 2)
            {
                foot_grf(0) = grf_2d(0);
                foot_grf(1) = grf_2d(1);
            }
        }
        // std::cerr<< "foot "<< leg << " has GRF in world " << foot_grf(0)<< " " << foot_grf(1) << " " << foot_grf(2) << std::endl;
        // change the frame from world to base
        // foot_grf = R_w2b_.inverse()*foot_grf;

        // std::cerr<< "foot "<< leg << " has GRF " << foot_grf(0)<< " " << foot_grf(1) << " " << foot_grf(2) << std::endl;
        return true;
    };

    bool Pinocchio_Feet_Force::update_All()
    {

        // update kinematic
        pinocchio::forwardKinematics(model_,data_,q_pin_);

        // update Jacobian
        pinocchio::computeJointJacobians(model_,data_,q_pin_);

        //update frame
        pinocchio::updateFramePlacements(model_,data_);

        // compute rnea
        pinocchio::rnea(model_,data_, q_pin_, dq_pin_, ddq_pin_);
        return true;

    }

    void Pinocchio_Feet_Force::set_State(
                    std::map<std::string,std::tuple<double,double,double>> jnt_stt,
                    Quaterniond orient,
                    Vector3d xd,
                    Vector3d xdd,
                    Vector3d omega,
                    Vector3d omegad

    )
    {


        q_pin_.setZero();
        dq_pin_.setZero();
        ddq_pin_.setZero();
        tau_msr_.setZero();


        R_w2b_ = orient.toRotationMatrix();
        q_pin_.block<4,1>(3,0) = orient.coeffs();
        dq_pin_.block<3,1>(0,0) = xd;
        ddq_pin_.block<3,1>(0,0) = xdd;
        dq_pin_.block<3,1>(3,0) = omega;
        ddq_pin_.block<3,1>(3,0) = omegad;
        // for(auto & jnt:model_.names)
        try
        {
            for(size_t i = 0; i < pin_jnt_name_.size(); i++)
            {
                // std::cerr << "the "<< i<<"-th joints name is "<< pin_jnt_name_[i]<<std::endl;

                q_pin_(i + FB_DOF) = std::get<0>(jnt_stt[pin_jnt_name_[i]]);
                dq_pin_(i + FB_VEL) = std::get<1>(jnt_stt[pin_jnt_name_[i]]);
                tau_msr_(i) = std::get<2>(jnt_stt[pin_jnt_name_[i]]);


            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }


    }
    /***** Start The Feet Jacobian fonction definition *****/

    // get the foot jacobian
    pronto::quadruped::FootJac Pinocchio_Jacobian::getFootJacobian(
        const pronto::quadruped::JointState&,
        const pronto::quadruped::LegID& leg)
    {
        LinVelJac J;
        pin_ff_->getLegJacobian(leg,J,false);
        // J.block<3,1>(0,0) = this->get_foot_vel(leg);

        return J;
    }

    pronto::quadruped::FootJac Pinocchio_Jacobian::getFootJacobianAngular(const pronto::quadruped::JointState&,const pronto::quadruped::LegID&)
    {
        LinVelJac J;
        J.setZero();
        return J;
    }


   /***** Start The FK fonction definition *****/

    // get the foot pos
     Eigen::Vector3d Pinocchio_FK::getFootPos(const JointState& , const LegID& leg)
     {
            return pin_ff_->get_FK(leg);
     };

    Eigen::Matrix3d Pinocchio_FK::getFootOrientation(const JointState& q, const LegID& leg)
     {
        Eigen::Matrix3d a;
        a.setZero();
        return a;
     }

};
