#include "gravity_compensation_controller/gravity_compensation_controller.hpp"

#include <memory>
#include <string>
#include <vector>

#include "gravity_compensation_controller/gravity_compensation_controller.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/state.hpp"

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
        std::vector<std::string> conf_names(params_.joint_names.size());
        for (const auto &joint_name : params_.joint_names)
        {
            conf_names.push_back(joint_name + "/" + HW_IF_EFFORT);
        }
        return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    InterfaceConfiguration GravityCompensationController::state_interface_configuration() const
    {
        std::vector<std::string> conf_names(2 * params_.joint_names.size());
        for (const auto &joint_name : params_.joint_names)
        {
            conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
        }
        for (const auto &joint_name : params_.joint_names)
        {
            conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
        }
        return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::return_type GravityCompensationController::update(
        const rclcpp::Time &, const rclcpp::Duration &period)
    {
        auto logger = get_node()->get_logger();
        const std::size_t n_joints = params_.joint_names.size();

        if (pose_initialized_)
        {
            std::vector<double> initial_state;
            for (std::size_t i = 0; i < n_joints; i++)
            {
                initial_state[i] = state_interfaces_[0].get_value();
            }
            received_trajectory_.set(initial_state);
            pose_initialized_ = false;
        }

        std::vector<double> last_command_msg(n_joints);
        received_trajectory_.get(last_command_msg);

        for (std::size_t i = 0; i < n_joints; i++)
        {
            const double kp = params_.kp_gains[i] * state_interfaces_[i].get_value();
            const double kd = params_.kd_gains[i] * state_interfaces_[i + n_joints].get_value();
            const double tau_d = kp + kd * period.seconds();
            const double lim = params_.torque_limits[i];
            const double tau_d_clamped = std::max(-lim, std::min(tau_d, lim));
            command_interfaces_[i].set_value(tau_d_clamped);
        }

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
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GravityCompensationController::on_activate(
        const rclcpp_lifecycle::State &)
    {
        subscriber_is_active_ = true;
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
