#include "pronto_ros/ros_frontend.hpp"
#include <stdexcept>

namespace pronto {
using SensorId = pronto::ROSFrontEnd::SensorId;

ROSFrontEnd::ROSFrontEnd(rclcpp::Node::SharedPtr nh, bool verbose) :
    nh_(nh), verbose_(verbose)
{
    bool publish_pose;
    std::string pose_frame_id = "odom";
    std::string pose_topic = "POSE_BODY";
    std::string twist_topic = "TWIST_BODY";
    std::string twist_frame_id = "base";
    std::string tf_child_frame_id = "base";
    int history_span = 1e6; // keep 1 second as default
    tf2_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*nh);


    //declare Parameters
    nh_->declare_parameter<std::string>("pose_frame_id",pose_frame_id);
    nh_->declare_parameter<std::string>("pose_topic",pose_topic);
    nh_->declare_parameter<std::string>("twist_frame_id",twist_frame_id);
    nh_->declare_parameter<std::string>("twist_topic",twist_topic);
    nh_->declare_parameter<bool>("publish_tf",false);
    nh_->declare_parameter<bool>("publish_pose",false);
    nh_->declare_parameter<std::string>("tf_child_frame_id",tf_child_frame_id);
    // Minimum time between TF messages, s; 0 sends one per filter update (the IMU rate).
    nh_->declare_parameter<double>("tf_min_period",0.0);
    nh_->declare_parameter<int>("utime_history_span",history_span);

    // looking for relative param names, the nodehandle namespace will be added
    // automatically, e.g. publish_pose -> /state_estimator_pronto/publish_pose
    RCLCPP_INFO_STREAM(nh_->get_logger(),"Create ROSFrontEnd");
    // declare all parameters
    if(nh_->get_parameter("publish_pose", publish_pose)){
        if(publish_pose){


            if(!nh_->get_parameter("pose_frame_id", pose_frame_id)){
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Couldn't get param \"pose_frame_id\". Setting default to: \"" + pose_frame_id +"\"");
            }

            if(nh_->get_parameter("pose_topic", pose_topic)){
                pose_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 200);
                pose_msg_.header.frame_id = pose_frame_id;
            }

            if(!nh_->get_parameter("twist_frame_id", twist_frame_id)){
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Couldn't get param \"twist_frame_id\". Setting default to: \"" + twist_frame_id +"\"");
            }
            if(nh_->get_parameter("twist_topic", twist_topic)){
                twist_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(twist_topic, 200);
                twist_msg_.header.frame_id = twist_frame_id;
            }
            // try to
            if(!nh_->get_parameter("publish_tf", publish_tf_)){
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Couldn't get param \"publish_tf\". Not publishing TF.");
            }
            if(publish_tf_){

                if(!nh_->get_parameter("tf_child_frame_id", tf_child_frame_id)){
                    RCLCPP_WARN_STREAM(nh_->get_logger(), "Couldn't get param \"tf_frame_id\". Setting default to \"base\".");
                }
                const double tf_min_period = nh_->get_parameter("tf_min_period").as_double();
                if (tf_min_period < 0.0) {
                    throw std::invalid_argument("tf_min_period must be >= 0, got " + std::to_string(tf_min_period));
                }
                tf_min_period_ = rclcpp::Duration::from_seconds(tf_min_period);
                RCLCPP_INFO_STREAM(nh_->get_logger(), "TF minimum period: " << tf_min_period << " s");
                transform_msg_.header.frame_id = pose_frame_id;
                // NOTE implicitly assuming the twist frame id is the base frame
                transform_msg_.child_frame_id = tf_child_frame_id;
                RCLCPP_INFO_STREAM(nh_->get_logger(), "Publishing TF with frame_id: \"" + transform_msg_.header.frame_id + "\" and child_frame_id: \"" + transform_msg_.child_frame_id + "\"");
            }
        }

    }



    if(!nh_->get_parameter<int>("utime_history_span", history_span)){
     RCLCPP_WARN_STREAM(nh_->get_logger(), "Couldn't get param \"utime_history_span\". Setting default to \"" + std::to_string(history_span) + "\"");
    }
    history_span_ = history_span;
    initializeState();
    initializeCovariance();
    if(verbose_){
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Frontend constructed.");
    }

    aicp_path_publisher = nh_->create_publisher<nav_msgs::msg::Path>("/aicp/relative_path",100);
}

void ROSFrontEnd::initializeState()
{
    // declare parameter

    // setting the initial values from the state
    std::vector<double> init_velocity = std::vector<double>(3,0.0);
    std::vector<double> init_position  = std::vector<double>(3,0.0);
    std::vector<double> init_omega = std::vector<double>(3,0.0);
    std::vector<double> init_orient = std::vector<double>(3,0.0);

    //declare parameter
    nh_->declare_parameter<std::vector<double>>("x0.velocity", init_velocity);
    nh_->declare_parameter<std::vector<double>>("x0.position", init_position);
    nh_->declare_parameter<std::vector<double>>("x0.angular_velocity", init_omega);
    nh_->declare_parameter<std::vector<double>>("x0.rpy", init_orient);
    if(!nh_->get_parameter("x0.velocity", init_velocity)){
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Couldn't get default velocity. Setting to zero.");
    }
    default_state.velocity() = Eigen::Map<Eigen::Vector3d>(init_velocity.data());

    // setting the initial values from the state

    if(!nh_->get_parameter("x0.position", init_position)){
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Couldn't get default position. Setting to zero.");
    }
    default_state.position() = Eigen::Map<Eigen::Vector3d>(init_position.data());


    if(!nh_->get_parameter("x0.angular_velocity", init_omega)){
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Couldn't get default ang velocity. Setting to zero.");
    }
    default_state.angularVelocity() = Eigen::Map<Eigen::Vector3d>(init_omega.data());


    if(!nh_->get_parameter("x0.rpy", init_orient)){
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Couldn't get default ang velocity. Setting to zero.");
        default_state.orientation() = Eigen::Quaterniond::Identity();
    }
    default_state.orientation() = rotation::setQuatEulerAngles(Eigen::Map<Eigen::Vector3d>(init_orient.data()));
    init_state = default_state;
    head_state = default_state;
    if(verbose_){
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Filter Initial State initialized.");
    }
}


void ROSFrontEnd::initializeCovariance(){
    default_cov = RBIM::Zero();

    // allocate init cov params
    double sigma_Delta_xy_init = 0;
    double sigma_Delta_z_init = 0;
    double sigma_chi_xy_init = 0;
    double sigma_chi_z_init = 0;
    double sigma_vb_init = 0;
    double sigma_gyro_bias_init = 0;
    double sigma_accelerometer_bias_init = 0;

    //declare init cov params
    nh_->declare_parameter<double>("sigma0.Delta_xy", sigma_Delta_xy_init);
    nh_->declare_parameter<double>("sigma0.Delta_z", sigma_Delta_z_init);
    nh_->declare_parameter<double>("sigma0.chi_xy", sigma_chi_xy_init);
    nh_->declare_parameter<double>("sigma0.chi_z", sigma_chi_z_init);
    nh_->declare_parameter<double>("sigma0.vb", sigma_vb_init);
    nh_->declare_parameter<double>("sigma0.gyro_bias", sigma_gyro_bias_init);
    nh_->declare_parameter<double>("sigma0.accel_bias", sigma_accelerometer_bias_init);

    // get params
    if(!nh_->get_parameter("sigma0.Delta_xy", sigma_Delta_xy_init)){
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Couldn't get param sigma0/Delta_xy. Setting to zero.");
    }


    if(!nh_->get_parameter("sigma0.Delta_z", sigma_Delta_z_init)){
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Couldn't get param sigma0/Delta_z. Setting to zero.");
    }

    if(!nh_->get_parameter("sigma0.chi_xy", sigma_chi_xy_init)){
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Couldn't get param sigma0/chi_xy. Setting to zero.");
    }
    sigma_chi_xy_init *= M_PI / 180.0; // convert to radians

    if(!nh_->get_parameter("sigma0.chi_z", sigma_chi_z_init)){
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Couldn't get param sigma0/chi_z. Setting to zero.");
    }
    sigma_chi_z_init *= M_PI / 180.0; // convert to radians

    if(!nh_->get_parameter("sigma0.vb", sigma_vb_init)){
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Couldn't get param sigma0/vb. Setting to zero.");
    }

    if(!nh_->get_parameter("sigma0.gyro_bias", sigma_gyro_bias_init)){
        RCLCPP_WARN_STREAM(nh_->get_logger(),"Couldn't get param sigma0/accel_bias. Setting to zero.");
    }
    sigma_gyro_bias_init *= M_PI / 180.0; // convert to radians

    if(!nh_->get_parameter("sigma0.accel_bias", sigma_accelerometer_bias_init)){
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Couldn't get param sigma0/accel_bias. Setting to zero.");
    }

    Eigen::Vector3d xyz_cov_diag =
        Eigen::Array3d(sigma_Delta_xy_init, sigma_Delta_xy_init, sigma_Delta_z_init).square();

    Eigen::Vector3d init_chi_cov_diag = Eigen::Array3d(sigma_chi_xy_init, sigma_chi_xy_init, sigma_chi_z_init).square();

    //set all the sub-blocks of the covariance matrix
    default_cov.block<3, 3>(RBIS::velocity_ind, RBIS::velocity_ind) = std::pow(sigma_vb_init,2) * Eigen::Matrix3d::Identity();
    default_cov.block<3, 3>(RBIS::chi_ind, RBIS::chi_ind) = init_chi_cov_diag.asDiagonal();
    default_cov.block<3, 3>(RBIS::position_ind, RBIS::position_ind) = xyz_cov_diag.asDiagonal();
    default_cov.block<3, 3>(RBIS::gyro_bias_ind, RBIS::gyro_bias_ind) = Eigen::Matrix3d::Identity()
        * std::pow(sigma_gyro_bias_init,2);
    default_cov.block<3, 3>(RBIS::accel_bias_ind, RBIS::accel_bias_ind) = Eigen::Matrix3d::Identity()
        * std::pow(sigma_accelerometer_bias_init,2);

    init_cov = default_cov;
    head_cov = default_cov;
    if(verbose_){
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Filter Covariance initialized.");
    }
}

ROSFrontEnd::~ROSFrontEnd()
{
}

bool ROSFrontEnd::initializeFilter()
{
    // if the modules are not ready we return false
    if(!areModulesInitialized())
    {
        return false;
    }
    // if the filter is already initialized we quietly return
    if(isFilterInitialized()){
        return true;
    }
    state_est_.reset(new StateEstimator(new RBISResetUpdate(init_state,
                                                               init_cov,
                                                               RBISUpdateInterface::reset,
                                                               init_state.utime),
                                           history_span_));

    filter_initialized_ = true;
    if(verbose_){
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Filter initialized.");
    }
    return true;
}

bool ROSFrontEnd::areModulesInitialized(){
    std::map<SensorId,bool>::iterator it = initialized_list_.begin();
    for(; it != initialized_list_.end(); ++it){
        if(!it->second){
            return false;
        }
    }
    return true;
}

bool ROSFrontEnd::isFilterInitialized(){
    return filter_initialized_;
}

}

