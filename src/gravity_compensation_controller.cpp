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
        std::copy_n(kp_gains_vect.begin(), kNumJoints_, kp_gains_.begin());

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
        std::copy_n(kd_gains_vect.begin(), kNumJoints_, kd_gains_.begin());


        std::vector<double> torque_limits_vect;
        if (!controller_nh.getParam("torque_limits", torque_limits_vect))
        {
            ROS_ERROR("GravityCompensationController: Parameter 'torque_limits' not set");
            return false;
        }
        if (kd_gains_vect.size() != kNumJoints_)
        {
            ROS_ERROR("GravityCompensationController: Number of torque limits does not match number ot joints!");
            return false;
        }
        std::copy_n(torque_limits_vect.begin(), kNumJoints_, torque_limits_.begin());

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
        std::array<double, 7> new_command;
        for (std::size_t i = 0; i < kNumJoints_; i++)
        {
            new_command[i] = jt.points[i].positions[0];
        }
        command_.writeFromNonRT(new_command);
    }

    void GravityCompensationController::starting(const ros::Time &time)
    {
        last_time_ = time;
        franka::RobotState state = state_handle_->getRobotState();
        command_.writeFromNonRT(state.q);
        ROS_INFO_NAMED(kControllerName, "Controller Started!");
    }

    void GravityCompensationController::update(const ros::Time &time, const ros::Duration & /*period*/)
    {
        const double dt = (time - last_time_).toSec();
        last_time_ = time;

        franka::RobotState state = state_handle_->getRobotState();
        const std::array<double, 7> &state_pos = state.q;
        const std::array<double, 7> &state_vel = state.dq;
        const std::array<double, 7> target_pose = *(command_.readFromRT());

        for (std::size_t i = 0; i < kNumJoints_; i++)
        {
            const double kp = kp_gains_[i] * (target_pose[i] - state_pos[i]);
            const double kd = -kd_gains_[i] * state_vel[i];
            const double lim = torque_limits_[i];
            double tau_d = (kp + kd) * 1000.0;
            if (std::isnan(tau_d))
            {
                tau_d = 0.0;
            }
            joint_handles_[i].setCommand(std::clamp(tau_d, -lim, lim));
        }
    }
}

PLUGINLIB_EXPORT_CLASS(
    gravity_compensation_controller::GravityCompensationController,
    controller_interface::ControllerBase)