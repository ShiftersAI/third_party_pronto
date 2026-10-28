/* Copyright (c) 2018-2020 University of Oxford
 * All rights reserved.
 *
 * Author: Marco Camurri (mcamurri@robots.ox.ac.uk)
 *
 * This file is part of pronto_quadruped,
 * a library for leg odometry on quadruped robots.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <pronto_core/sensing_module.hpp>

#include <pronto_quadruped_ros/stance_estimator_ros.hpp>
#include <pronto_quadruped_ros/leg_odometer_ros.hpp>
#include <pronto_quadruped/DataLogger.hpp>

#include <pronto_msgs/msg/joint_state_with_acceleration.hpp>
#include <pronto_msgs/msg/quadruped_stance.hpp>
#include <pronto_msgs/msg/quadruped_force_torque_sensors.hpp>
#include <pronto_msgs/msg/velocity_with_sigma_bounds.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>

#include <pi3hat_moteus_int_msgs/msg/joints_states.hpp>


namespace pronto {
namespace quadruped {

void normalizeSecNSec(unsigned long& sec, unsigned long& nsec)
{
     unsigned long nsec_part= nsec % 1000000000UL;
     unsigned long sec_part = nsec / 1000000000UL;
     sec += sec_part;
     nsec = nsec_part;
}

builtin_interfaces::msg::Time fromNSec(long t)
{
    builtin_interfaces::msg::Time tempo;
    unsigned long sec = t / 1000000000;
    unsigned long nsec = t % 1000000000;
    normalizeSecNSec(sec, nsec); // this normalization is probably unnecessary
    tempo.set__sec(sec);
    tempo.set__nanosec(nsec);
    return tempo;
 }

unsigned long toNsec(const builtin_interfaces::msg::Time & time) 
{ 
    uint64_t sec = time.sec;
    uint64_t nsec = time.nanosec;
    return sec*1000000000ull + nsec; 
}
unsigned long tomsec(const builtin_interfaces::msg::Time & time) 
{ 
    uint64_t sec = time.sec;
    uint64_t nsec = time.nanosec;
    return sec*1000000000ull + nsec; 
}
class LegodoHandlerBase {
public:
    using LegScalarMap = LegDataMap<double>;
    using Update = RBISUpdateInterface;

public:
    LegodoHandlerBase(rclcpp::Node::SharedPtr nh,
                      StanceEstimatorROS* fcf,
                      LegOdometerROS* fj,
                      std::vector<std::string> jnt_names=
                                     {"LF_HAA", "LF_HFE", "LF_KFE",
                                      "RF_HAA", "RF_HFE", "RF_KFE",
                                      "LH_HAA", "LH_HFE", "LH_KFE",
                                      "RH_HAA", "RH_HFE", "RH_KFE"});
    virtual ~LegodoHandlerBase() = default;
    // void order_msg()
    // {
    //     for(auto &jnt_n:jnt_names_)
    //     {
    //         find()
    //     }
    // }

protected:
    StanceEstimatorROS* stance_estimator_;
    LegOdometerROS* leg_odometer_;

    std::string base_link_name_;           ///< Name of the base_link
    std::vector<std::string> foot_names_;  ///< Name of the feet frames (in LF, RF, LH, RH order)

    Eigen::Vector3d r_legodo_;
    Eigen::Matrix3d cov_legodo_;

    JointState q_;
    JointState qd_;
    JointState qdd_;
    JointState tau_;
    // Joint zero corrections (rad, Pronto joint order LF RF LH RH x HAA HFE KFE), added to every
    // measured joint position before kinematics, Jacobians and contact forces.
    JointState joint_offsets_ = JointState::Zero();

    RBIS head_state_;
    RBIM head_cov_;

    Eigen::Vector3d xd_;               ///< Linear velocity of the base frame, expressed in inertial frame
    Eigen::Vector3d xdd_;              ///< Net linear acceleration of the base frame without gravity, expressed in inertial frame
    Eigen::Vector3d omega_;            ///< Angular velocity of the base, expressed in base frame
    Eigen::Vector3d omegad_;           ///< Angular acceleration of the base, expressed in base frame
    Eigen::Quaterniond orientation_;   ///< Orientation of the base with respect to the inertial frame, expressed in base frame

    LegBoolMap stance_;
    LegScalarMap stance_prob_;
    LegVectorMap grf_;

    uint64_t utime_;  // time in microseconds
    uint64_t nsec_;   // time in nanoseconds

    uint16_t downsample_factor_ = 1;
    uint64_t utime_offset_ = 0;

    // Debug Publishers
    std::vector<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> vel_debug_;  
    std::vector<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr> grf_debug_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr> grf_in_foot_frame_debug_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_raw_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr prior_joint_accel_debug_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr prior_velocity_debug_;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr prior_accel_debug_;
    rclcpp::Publisher<pronto_msgs::msg::VelocityWithSigmaBounds>::SharedPtr vel_sigma_bounds_pub_;
    std::vector<std::string> jnt_names_;
    bool debug_ = true;  // Debug output including CSV output
    bool output_log_to_file_ = true;
    geometry_msgs::msg::WrenchStamped wrench_msg_;
    pronto_msgs::msg::QuadrupedStance stance_msg_;
    rclcpp::Publisher<pronto_msgs::msg::QuadrupedStance>::SharedPtr stance_pub_;

    std::unique_ptr<pronto::DataLogger> dl_pose_;
    std::unique_ptr<pronto::DataLogger> dl_vel_;
    std::unique_ptr<pronto::DataLogger> dl_vel_sigma_;

    pronto_msgs::msg::VelocityWithSigmaBounds vel_sigma_bound_msg_;

    virtual Update* computeVelocity();
    virtual void getPreviousState (const StateEstimator *est);
    rclcpp::Node::SharedPtr nh_;

    
};


class LegodoHandlerPinRos : public pronto::SensingModule<pi3hat_moteus_int_msgs::msg::JointsStates>,
                            public LegodoHandlerBase
{
    public:
        LegodoHandlerPinRos(
            rclcpp::Node::SharedPtr nh,
            StanceEstimatorROS* stance_est,
            LegOdometerROS* legodo,
            std::vector<std::string> jnt_names=
                                     {"LF_HAA", "LF_HFE", "LF_KFE",
                                      "RF_HAA", "RF_HFE", "RF_KFE",
                                      "LH_HAA", "LH_HFE", "LH_KFE",
                                      "RH_HAA", "RH_HFE", "RH_KFE"}
        );
        virtual ~LegodoHandlerPinRos() = default;
        Update* processMessage(const pi3hat_moteus_int_msgs::msg::JointsStates *msg, 
                                StateEstimator *est) override;

        bool processMessageInit(const pi3hat_moteus_int_msgs::msg::JointsStates *msg,
                                const std::map<std::string, bool> &sensor_initialized,
                                const RBIS &default_state,
                                const RBIM &default_cov,
                                RBIS &init_state,
                                RBIM &init_cov) override;
    

};

class LegodoHandlerPinRos_Sim : public pronto::SensingModule<sensor_msgs::msg::JointState>,
                            public LegodoHandlerBase
{
    public:
        LegodoHandlerPinRos_Sim(
            rclcpp::Node::SharedPtr nh,
            StanceEstimatorROS* stance_est,
            LegOdometerROS* legodo,
            std::vector<std::string> jnt_names=
                                     {"LF_HAA", "LF_HFE", "LF_KFE",
                                      "RF_HAA", "RF_HFE", "RF_KFE",
                                      "LH_HAA", "LH_HFE", "LH_KFE",
                                      "RH_HAA", "RH_HFE", "RH_KFE"}
        );
        virtual ~LegodoHandlerPinRos_Sim() = default;
        Update* processMessage(const sensor_msgs::msg::JointState *msg, 
                                StateEstimator *est) override;

        bool processMessageInit(const sensor_msgs::msg::JointState *msg,
                                const std::map<std::string, bool> &sensor_initialized,
                                const RBIS &default_state,
                                const RBIM &default_cov,
                                RBIS &init_state,
                                RBIM &init_cov) override;
    

};
class LegodoHandlerROS : public pronto::SensingModule<sensor_msgs::msg::JointState>,
                         public LegodoHandlerBase
{
public:
    LegodoHandlerROS(rclcpp::Node::SharedPtr nh,
                     StanceEstimatorROS *stance_est,
                     LegOdometerROS *legodo);
    virtual ~LegodoHandlerROS() = default;

    Update* processMessage(const sensor_msgs::msg::JointState *msg, 
                            StateEstimator *est) override;

    bool processMessageInit(const sensor_msgs::msg::JointState *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov) override;

private:
    rclcpp::Node::SharedPtr nh_;
};

class LegodoHandlerWithAccelerationROS : public pronto::SensingModule<pronto_msgs::msg::JointStateWithAcceleration>,
                                           public LegodoHandlerBase
{
public:
    LegodoHandlerWithAccelerationROS(rclcpp::Node::SharedPtr nh,
                                       StanceEstimatorROS *stance_est,
                                       LegOdometerROS *legodo) : 
    nh_(nh),
    LegodoHandlerBase(nh, stance_est, legodo) 
    {
    }

    virtual ~LegodoHandlerWithAccelerationROS() = default;
    
    Update* processMessage(const pronto_msgs::msg::JointStateWithAcceleration *msg, 
                            StateEstimator *est) override;

    bool processMessageInit(const pronto_msgs::msg::JointStateWithAcceleration *msg,
                            const std::map<std::string, bool>& sensor_initialized,
                            const RBIS& default_state,
                            const RBIM& default_cov,
                            RBIS& init_state,
                            RBIM& init_cov) override { return true; }

private:
    rclcpp::Node::SharedPtr nh_;
};

class ForceSensorLegodoHandlerROS : public LegodoHandlerBase,
                                    public pronto::DualSensingModule<sensor_msgs::msg::JointState,
                                                                     pronto_msgs::msg::QuadrupedForceTorqueSensors>
{
public:
    ForceSensorLegodoHandlerROS(rclcpp::Node::SharedPtr nh,
                                StanceEstimatorROS* stance_est,
                                LegOdometerROS* legodo);
    virtual ~ForceSensorLegodoHandlerROS() = default;

        
    Update* processMessage(const sensor_msgs::msg::JointState *msg, 
                            StateEstimator *est) override;

    bool processMessageInit(const sensor_msgs::msg::JointState *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov) override;

    void processSecondaryMessage(const pronto_msgs::msg::QuadrupedForceTorqueSensors& msg) override;

private:
    rclcpp::Node::SharedPtr nh_;
};

class FootSensorLegodoHandlerROS : public LegodoHandlerBase,
                                   public pronto::DualSensingModule<sensor_msgs::msg::JointState,
                                                                    pronto_msgs::msg::QuadrupedStance>
{
public:
    FootSensorLegodoHandlerROS(rclcpp::Node::SharedPtr nh,
                                StanceEstimatorROS* stance_est,
                                LegOdometerROS* legodo);
    virtual ~FootSensorLegodoHandlerROS() = default;

    Update* processMessage(const sensor_msgs::msg::JointState *msg, 
                                StateEstimator *est) override;

    bool processMessageInit(const sensor_msgs::msg::JointState *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov) override;

    void processSecondaryMessage(const pronto_msgs::msg::QuadrupedStance& msg) override;

private:
    rclcpp::Node::SharedPtr nh_;
};

}  // namespace quadruped
}  // namespace pronto
