// =============================================================================
// gripper_centering_controller.cpp   (ros2_control plugin, runs at 1 kHz)
// -----------------------------------------------------------------------------
// Target:
//   Make two independently effort-driven fingers behave like the real OM-X
//   single-motor gripper: a commanded squeeze F, plus a stiff virtual coupling
//   that keeps the finger PAIR centred so a swinging payload cannot slosh it
//   sideways (pilot #1 root cause, 22 Nov).
//
// Physics / Math Implemented:
//   Finger joints q_L, q_R (axes point OUTWARD, negative effort = close).
//   Common mode  c = (q_L + q_R)/2   -> set by the squeeze F (force control)
//   Lateral mode s = (q_L - q_R)/2   -> restored by a PD (position control)
//       u_L = -F - K*s - D*ds/dt
//       u_R = -F + K*s + D*ds/dt
//   Sum u_L + u_R = -2F: the squeeze is EXACTLY F per finger, so the G2
//   stiffness calibration (k*x/F = 0.995) is untouched. Generalised force on
//   s is -2K*s - 2D*ds/dt -> effective 300 N/m, 15 N*s/m.
//   Explicit stability at T = 1 ms, finger mass 0.02 kg:
//       K*T^2/m = 0.0075 << 1,  D*T/m = 0.38 < 2.
//
// Inputs & Outputs:
//   Sub   : /gripper/force_cmd (std_msgs/Float64, N, > 0 squeeze, < 0 open)
//   State : <joint>/position [m], <joint>/velocity [m/s] for both fingers
//   Cmd   : <joint>/effort [N] for both fingers, clamped to +-max_effort
// =============================================================================

#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace rt1_gripper_controller
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

class GripperCenteringController : public controller_interface::ControllerInterface
{
public:
    // --- 1. PARAMETERS ---
    CallbackReturn on_init() override
    {
        auto node = get_node();
        node->declare_parameter<std::string>("left_joint", "gripper_left_joint");
        node->declare_parameter<std::string>("right_joint", "gripper_right_joint");
        node->declare_parameter<double>("centering_stiffness", 150.0);   // K [N/m]
        node->declare_parameter<double>("centering_damping", 7.5);       // D [N*s/m]
        node->declare_parameter<double>("max_squeeze_n", 5.0);           // |F| cap
        node->declare_parameter<double>("max_effort_n", 10.0);           // per-finger cap
        node->declare_parameter<double>("initial_force_n", -1.0);        // start open
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        auto node      = get_node();
        left_joint_    = node->get_parameter("left_joint").as_string();
        right_joint_   = node->get_parameter("right_joint").as_string();
        stiffness_     = node->get_parameter("centering_stiffness").as_double();
        damping_       = node->get_parameter("centering_damping").as_double();
        max_squeeze_n_ = node->get_parameter("max_squeeze_n").as_double();
        max_effort_n_  = node->get_parameter("max_effort_n").as_double();
        squeeze_cmd_n_.store(node->get_parameter("initial_force_n").as_double());
        // Same topic the FSM and sim_helpers already use. An atomic double is
        // enough to pass one number from the subscriber thread to the RT loop.
        force_subscription_ = node->create_subscription<std_msgs::msg::Float64>(
            "/gripper/force_cmd", rclcpp::QoS(10),
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                squeeze_cmd_n_.store(std::clamp(msg->data, -max_squeeze_n_, max_squeeze_n_));}
        );
        return CallbackReturn::SUCCESS;
    }

    // --- 2. HARDWARE INTERFACES CLAIMED ---
    InterfaceConfiguration command_interface_configuration() const override
    {
        return 
        {
            interface_configuration_type::INDIVIDUAL,
            {
                left_joint_ + "/" + hardware_interface::HW_IF_EFFORT,
                right_joint_ + "/" + hardware_interface::HW_IF_EFFORT
            }
        };
    }

    InterfaceConfiguration state_interface_configuration() const override
    {
        return 
        {
            interface_configuration_type::INDIVIDUAL,
            {
                left_joint_  + "/" + hardware_interface::HW_IF_POSITION,
                left_joint_  + "/" + hardware_interface::HW_IF_VELOCITY,
                right_joint_ + "/" + hardware_interface::HW_IF_POSITION,
                right_joint_ + "/" + hardware_interface::HW_IF_VELOCITY
            }
        };
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        // Look interfaces up by name: do not rely on the order they were loaned in.
        left_effort_  = findIndex(command_interfaces_, left_joint_,  hardware_interface::HW_IF_EFFORT);
        right_effort_ = findIndex(command_interfaces_, right_joint_, hardware_interface::HW_IF_EFFORT);
        left_pos_     = findIndex(state_interfaces_,   left_joint_,  hardware_interface::HW_IF_POSITION);
        left_vel_     = findIndex(state_interfaces_,   left_joint_,  hardware_interface::HW_IF_VELOCITY);
        right_pos_    = findIndex(state_interfaces_,   right_joint_, hardware_interface::HW_IF_POSITION);
        right_vel_    = findIndex(state_interfaces_,   right_joint_, hardware_interface::HW_IF_VELOCITY);
        if (
            left_effort_  < 0 || 
            right_effort_ < 0 || 
            left_pos_ < 0 || 
            left_vel_ < 0 ||
            right_pos_ < 0 || 
            right_vel_ < 0)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(), 
                "Missing a gripper interface"
            );
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(
            get_node()->get_logger(), 
            "Gripper centring active: K=%.1f N/m D=%.2f N*s/m",
            stiffness_, 
            damping_);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        if (left_effort_ >= 0 && right_effort_ >= 0) 
        {
            // set_value() returns false if the interface is busy this cycle; on
            // deactivate a missed zero is harmless (the next controller overwrites it).
            [[maybe_unused]] const bool left_zeroed  = command_interfaces_[left_effort_].set_value(0.0);
            [[maybe_unused]] const bool right_zeroed = command_interfaces_[right_effort_].set_value(0.0);
        }
        return CallbackReturn::SUCCESS;
    }

    // --- 3. CONTROL LAW (every physics step) ---
    controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
    {
        // get_optional() is empty if the hardware is mid-write this cycle.
        // Skipping one 1 ms cycle keeps the previous effort: safe and invisible.
        const auto left_pos_m = state_interfaces_[left_pos_].get_optional();
        const auto right_pos_m = state_interfaces_[right_pos_].get_optional();
        const auto left_vel_m_s = state_interfaces_[left_vel_].get_optional();
        const auto right_vel_m_s = state_interfaces_[right_vel_].get_optional();
        if (!left_pos_m || !right_pos_m || !left_vel_m_s || !right_vel_m_s) 
        {
            return controller_interface::return_type::OK;
        }
        // Lateral offset of the finger pair (0 = centred) and its rate.
        const double lateral_offset_m = 0.5 * (*left_pos_m - *right_pos_m);
        const double lateral_rate_m_s = 0.5 * (*left_vel_m_s - *right_vel_m_s);
        const double centring_n = stiffness_ * lateral_offset_m + damping_ * lateral_rate_m_s;
        const double squeeze_n  = squeeze_cmd_n_.load();
        const double left_effort_n  = std::clamp(-squeeze_n - centring_n, -max_effort_n_, max_effort_n_);
        const double right_effort_n = std::clamp(-squeeze_n + centring_n, -max_effort_n_, max_effort_n_);
        // If either write is refused this cycle, the next cycle (1 ms later) retries.
        [[maybe_unused]] const bool left_written  = command_interfaces_[left_effort_].set_value(left_effort_n);
        [[maybe_unused]] const bool right_written = command_interfaces_[right_effort_].set_value(right_effort_n);
        return controller_interface::return_type::OK;
    }

private:
    template<typename InterfaceVector>
    static int findIndex(
        const InterfaceVector & interfaces, 
        const std::string & joint, 
        const std::string & type)
    {
        for (size_t i = 0; i < interfaces.size(); ++i) 
        {
            if (interfaces[i].get_prefix_name() == joint && 
                interfaces[i].get_interface_name() == type) 
            {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    std::string left_joint_, right_joint_;
    double stiffness_{150.0}, damping_{7.5}, max_squeeze_n_{5.0}, max_effort_n_{10.0};
    std::atomic<double> squeeze_cmd_n_{-1.0};
    int left_effort_{-1}, right_effort_{-1}, left_pos_{-1}, left_vel_{-1}, right_pos_{-1}, right_vel_{-1};
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr force_subscription_;
};

}  // namespace rt1_gripper_controller

PLUGINLIB_EXPORT_CLASS(
    rt1_gripper_controller::GripperCenteringController, 
    controller_interface::ControllerInterface
)
