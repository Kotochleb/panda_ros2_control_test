#ifndef GRAVITY_COMPENSATION_CONTROLLER_HPP
#define GRAVITY_COMPENSATION_CONTROLLER_HPP

#include <array>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ros/time.h>

#include <realtime_tools/realtime_buffer.h>

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <realtime_tools/realtime_box.h>

namespace gravity_compensation_controller
{

    // this controller gets access to the ForceTorqueSensorInterface
    class GravityCompensationController : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
                                                                                                franka_hw::FrankaStateInterface,
                                                                                                hardware_interface::EffortJointInterface>
    {
    public:
        GravityCompensationController() {}
        virtual bool init(hardware_interface::RobotHW *hw, ros::NodeHandle &controller_nh) override;
        virtual void starting(const ros::Time &time) override;
        virtual void update(const ros::Time &time, const ros::Duration &period) override;

    private:
        void joint_trajectory_cb(const trajectory_msgs::JointTrajectory& jt);

        std::string name_;
        pinocchio::Model model_;
        pinocchio::Data data_;

        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;

        std::array<double, 7> kp_gains_;
        std::array<double, 7> kd_gains_;
        std::array<double, 7> target_pose_;
        std::array<double, 7> torque_limits_;
        double alpha_;

        ros::Time last_time_;
        ros::Subscriber trajectory_sub_;
        realtime_tools::RealtimeBuffer<std::array<double, 7>> command_;

        const std::string kControllerName = "GravityCompensationController";
        static constexpr unsigned int kNumJoints_ = 7;
    };

}

#endif // GRAVITY_COMPENSATION_CONTROLLER_HPP