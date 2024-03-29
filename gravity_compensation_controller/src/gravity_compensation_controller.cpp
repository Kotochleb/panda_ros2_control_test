#include "gravity_compensation_controller/gravity_compensation_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "gravity_compensation_controller/gravity_compensation_controller.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"

#include <urdf_parser/urdf_parser.h>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <iostream>
namespace gravity_compensation_controller
{
    using controller_interface::interface_configuration_type;
    using controller_interface::InterfaceConfiguration;
    using hardware_interface::HW_IF_EFFORT;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;

    GravityCompensationController::GravityCompensationController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn GravityCompensationController::on_init()
    {
        try
        {
            // Create the parameter listener and get the parameters
            param_listener_ = std::make_shared<ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    InterfaceConfiguration GravityCompensationController::command_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for (const auto &joint_name : params_.joint_names)
        {
            conf_names.push_back(joint_name + "/" + HW_IF_EFFORT);
            // conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
        }
        return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    InterfaceConfiguration GravityCompensationController::state_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for (const auto &joint_name : params_.joint_names)
        {
            conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
        }
        for (const auto &joint_name : params_.joint_names)
        {
            conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
        }
        for (const auto &joint_name : params_.joint_names)
        {
            conf_names.push_back(joint_name + "/" + HW_IF_EFFORT);
        }
        return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::return_type GravityCompensationController::update(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        auto logger = get_node()->get_logger();
        const std::size_t n_joints = params_.joint_names.size();

        if (!pin_init_)
        {
            std::string rd;
            robot_description_.get(rd);
            if (rd != "")
            {
                pinocchio::Model model_full;
                const auto urdf_tree = urdf::parseURDF(rd);
                pinocchio::urdf::buildModel(urdf_tree, model_full);

                const auto q0 = pinocchio::neutral(model_full);
                std::vector<unsigned long> locked_joints_id{
                    model_full.getJointId("panda_finger_joint1"),
                    model_full.getJointId("panda_finger_joint2"),
                };
                model_ = pinocchio::buildReducedModel(model_full, locked_joints_id, q0);
                data_ = pinocchio::Data(model_);
                pin_init_ = true;
                RCLCPP_INFO(logger, "Pinocchio initialized!");
            }
        }

        if (!pose_initialized_)
        {
            std::vector<double> initial_state(n_joints);
            for (std::size_t i = 0; i < n_joints; i++)
            {
                initial_state[i] = state_interfaces_[i].get_value();
            }
            received_trajectory_.set(initial_state);
            pose_initialized_ = true;
            RCLCPP_INFO(logger, "First pose was initialized");
        }

        std::vector<double> last_command_msg(n_joints);
        received_trajectory_.get(last_command_msg);

        Vector7d tau_g;
        if (pin_init_)
        {
            Vector7d q;
            for (std::size_t i = 0; i < n_joints; i++)
            {
                q[i] = state_interfaces_[i].get_value();
            }
            tau_g = pinocchio::computeGeneralizedGravity(model_, data_, q);
        }
        else
        {
            tau_g = Vector7d::Zero();
        }

        for (std::size_t i = 0; i < n_joints; i++)
        {
            // std::cout << state_interfaces_[i + 2 * n_joints].get_value() << std::endl;
            const double kp = params_.kp_gains[i] * (last_command_msg[i] - state_interfaces_[i].get_value());
            const double kd = -params_.kd_gains[i] * state_interfaces_[i + n_joints].get_value();
            const double lim = params_.torque_limits[i];
            double tau_d = (kp + kd) * 1000.0;
            if (std::isnan(tau_d))
            {
                tau_d = 0.0;
            }
            command_interfaces_[i].set_value(std::clamp(tau_d, -lim, lim) + tau_g[i]);
        }
        // for (const auto &ci : command_interfaces_)
        // {
        //     std::cout << ci.get_value() << std::endl;
        // }
        // std::cout << std::endl;

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn GravityCompensationController::on_configure(
        const rclcpp_lifecycle::State &)
    {
        auto logger = get_node()->get_logger();

        // update parameters if they have changed
        if (param_listener_->is_old(params_))
        {
            params_ = param_listener_->get_params();
            RCLCPP_INFO(logger, "Parameters were updated");
        }

        const std::size_t n_joints = params_.joint_names.size();
        if (n_joints == 0)
        {
            RCLCPP_ERROR(logger, "No joint names provided!");
            return controller_interface::CallbackReturn::ERROR;
        }

        if (n_joints != params_.kp_gains.size())
        {
            RCLCPP_ERROR(
                logger, "The number of Kp gains [%zu] does not match number oj joints [%zu]!",
                params_.kp_gains.size(), n_joints);
            return controller_interface::CallbackReturn::ERROR;
        }

        if (n_joints != params_.kd_gains.size())
        {
            RCLCPP_ERROR(
                logger, "The number of Kd gains [%zu] does not match number oj joints [%zu]!",
                params_.kd_gains.size(), n_joints);
            return controller_interface::CallbackReturn::ERROR;
        }

        // initialize command subscriber
        joint_trajectory_sub_ = get_node()->create_subscription<JointTrajectory>(
            "/joint_trajectory", rclcpp::SystemDefaultsQoS(),
            [this, n_joints](const std::shared_ptr<JointTrajectory> msg) -> void
            {
                if (!this->subscriber_is_active_)
                {
                    RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
                    return;
                }
                std::vector<double> goal(n_joints);
                for (std::size_t i = 0; i < n_joints; i++)
                {
                    goal[i] = msg->points[i].positions[0];
                }
                this->received_trajectory_.set(goal);
            });

        robot_description_.set("");
        if (params_.remove_gravity)
        {
            robot_description_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
                "/robot_description", rclcpp::QoS(1).transient_local(),
                [this](const std_msgs::msg::String &msg) -> void
                {
                    this->robot_description_.set(msg.data);
                });
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GravityCompensationController::on_activate(
        const rclcpp_lifecycle::State &)
    {
        subscriber_is_active_ = true;
        pose_initialized_ = false;
        RCLCPP_INFO(get_node()->get_logger(), "Subscriber is now active.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GravityCompensationController::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        subscriber_is_active_ = false;
        RCLCPP_INFO(get_node()->get_logger(), "Subscriber was deactivated.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GravityCompensationController::on_cleanup(
        const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GravityCompensationController::on_error(const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::ERROR;
    }

    controller_interface::CallbackReturn GravityCompensationController::on_shutdown(
        const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

} // namespace gravity_compensation_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    gravity_compensation_controller::GravityCompensationController, controller_interface::ControllerInterface)
