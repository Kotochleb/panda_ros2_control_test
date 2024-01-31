#include <gravity_compensation_controller/gravity_compensation_controller.hpp>

#include <memory>
#include <string>

#include <Eigen/Dense>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <ros/time.h>

#include <pluginlib/class_list_macros.h>
#include <urdf_parser/urdf_parser.h>

#include <hardware_interface/joint_command_interface.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <iostream>

namespace gravity_compensation_controller
{

    bool GravityCompensationController::init(
        hardware_interface::RobotHW *robot_hw,
        ros::NodeHandle &controller_nh)
    {
        std::string robot_model;
        if (!controller_nh.getParam("/robot_description", robot_model))
        {
            ROS_ERROR("GravityCompensationController: Robot description couldn't be retrieved from param server.");
            return false;
        }

        const auto urdf_tree = urdf::parseURDF(robot_model);
        pinocchio::urdf::buildModel(urdf_tree, model_);
        data_ = pinocchio::Data(model_);

        std::vector<std::string> joint_names;
        if (!controller_nh.getParam("joint_names", joint_names))
        {
            ROS_ERROR_NAMED(kControllerName,
                            "Parameter 'joint_names' not set");
            return false;
        }
        if (joint_names.size() != kNumJoints_)
        {
            ROS_ERROR_NAMED(kControllerName,
                            "Parameter 'joint_names' has different number of joints than URDF model");
            return false;
        }

        if (!controller_nh.getParam("alpha", alpha_))
        {
            ROS_ERROR("GravityCompensationController: Parameter 'alpha' not set");
            return false;
        }

        std::vector<double> kp_gains_vect;
        if (!controller_nh.getParam("kd_gains", kp_gains_vect))
        {
            ROS_ERROR("GravityCompensationController: Parameter 'kd_gains' not set");
            return false;
        }

        if (kp_gains_.size() != kNumJoints_)
        {
            ROS_ERROR("GravityCompensationController: Number of KD gains does not match number ot joints!");
            return false;
        }
        kp_gains_ = Eigen::Map<Vector7d>(kp_gains_vect.data());

        std::vector<double> kd_gains_vect;
        if (!controller_nh.getParam("kd_gains", kd_gains_vect))
        {
            ROS_ERROR("GravityCompensationController: Parameter 'kd_gains' not set");
            return false;
        }
        if (kd_gains_vect.size() != kNumJoints_)
        {
            ROS_ERROR("GravityCompensationController: Number of KD gains does not match number ot joints!");
            return false;
        }
        kd_gains_ = Eigen::Map<Vector7d>(kd_gains_vect.data());

        auto model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr)
        {
            ROS_ERROR("GravityCompensationController: Error getting model interface from hardware");
            return false;
        }
        try
        {
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
                model_interface->getHandle("panda_model"));
        }
        catch (hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR("GravityCompensationController: Exception getting model handle from interface: %s", ex.what());
            return false;
        }

        auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr)
        {
            ROS_ERROR("GravityCompensationController: Error getting state interface from hardware");
            return false;
        }
        try
        {
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
                state_interface->getHandle("panda_robot"));
        }
        catch (hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR("GravityCompensationController: Exception getting state handle from interface: %s", ex.what());
            return false;
        }

        auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr)
        {
            ROS_ERROR("GravityCompensationController: Error getting effort joint interface from hardware");
            return false;
        }

        joint_handles_.reserve(kNumJoints_);
        for (size_t i = 0; i < kNumJoints_; i++)
        {
            joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
        }

        trajectory_sub_ = controller_nh.subscribe(
            "/joint_trajectory", 1, &GravityCompensationController::joint_trajectory_cb, this);

        ROS_INFO_NAMED(kControllerName, "Controller initialized.");

        return true;
    }

    void GravityCompensationController::joint_trajectory_cb(const trajectory_msgs::JointTrajectory& jt)
    {
        Vector7d new_command;
        for (std::size_t i = 0; i < kNumJoints_; i++)
        {
            new_command[i] = jt.points[i].positions[0];
        }
        std::cout << new_command << std::endl;
        command_.writeFromNonRT(new_command);
    }

    void GravityCompensationController::starting(const ros::Time &time)
    {
        last_time_ = time;
        franka::RobotState state = state_handle_->getRobotState();
        command_.writeFromNonRT(Eigen::Map<Vector7d>(state.q.data()));
        ROS_INFO_NAMED(kControllerName, "Controller Started!");
    }

    void GravityCompensationController::update(const ros::Time &time, const ros::Duration & /*period*/)
    {
        const double dt = (time - last_time_).toSec();
        last_time_ = time;

        franka::RobotState state = state_handle_->getRobotState();
        const Eigen::Map<Vector7d> q(state.q.data());

        const Vector7d target_pose_ = *(command_.readFromRT());

        const Vector7d error_q = target_pose_ - q;

        const Eigen::Map<Vector7d> dq(state.dq.data());
        const Vector7d error_dq = -dq;

        Vector7d tau_d;
        tau_d << kp_gains_.cwiseProduct(error_q) / dt +
                     kd_gains_.cwiseProduct(error_dq) / dt;

        for (std::size_t i = 0; i < kNumJoints_; i++)
        {
            joint_handles_[i].setCommand(tau_d(i));
        }
    }
}

PLUGINLIB_EXPORT_CLASS(
    gravity_compensation_controller::GravityCompensationController,
    controller_interface::ControllerBase)