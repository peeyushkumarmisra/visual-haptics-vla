// =============================================================================
// probe_controller_node.cpp  (expert demonstrator FSM)
// -----------------------------------------------------------------------------
// Target:
//   Scripted expert that produces the demonstrations RT-1 imitates. It probes
//   the mystery block (pinch -> stiffness, micro-lift -> mass), decides the bin
//   from VISUAL metrics only (same evidence the policy sees), then sorts it.
//
// Physics / Math Implemented:
//   - Stiffness: eps_vis = (W0 - d_px)/W0 after squeezing 0.3 N -> 2.0 N.
//       soft (k <= 650) -> eps >= ~0.09, stiff (k >= 2000) -> eps <= ~0.03;
//       threshold 0.06. Plate spring settles in < 0.1 s (omega >= 122 rad/s).
//   - Mass: Coulomb slip when m*g > 2*mu*F. At 2 N, mu 0.8: slip above 0.326 kg.
//       Slip shows as dy_rel > 6 px during a 15 mm micro-lift.
//       Recovery: one regrasp at 4.5 N.
//   - Carry: force raised AFTER the decision (light 3 N, heavy 6 N).
//   - Motion: fixed joint-space waypoints from planar IK (pitch 0.1 rad down).
//       All arm move durations are multiplied by `motion_time_scale` (live knob).
//       Place = swing at pad z 0.10, lower (best-effort) to z 0.06 (block bottom
//       ~20 mm above the floor: the block slides a few mm down in the grip
//       during the carry), release, rise vertically to z 0.17, retreat, home.
//
// Inputs & Outputs:
//   Sub : /joint_states, /probe/metrics, /episode/info (latched)
//   Pub : /arm_controller/joint_trajectory (joints 1-4, rad)
//         /gripper/force_cmd (Float64, N, > 0 squeeze, < 0 open)
//         /fsm/status (rt1_interfaces/FsmStatus, 50 Hz)
//   Cli : /probe/latch_reference, /probe/clear_reference (std_srvs/Trigger)
//   Srv : /fsm/run_episode (std_srvs/Trigger)
// =============================================================================
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "rt1_interfaces/msg/episode_info.hpp"
#include "rt1_interfaces/msg/fsm_status.hpp"
#include "rt1_interfaces/msg/probe_metrics.hpp"

namespace rt1_data_pipeline
{

using FsmStatusMsg = rt1_interfaces::msg::FsmStatus;
using TriggerSrv = std_srvs::srv::Trigger;
using JointTarget = std::array<double, 4>;   // joint1..joint4 [rad]

const std::array<std::string, 4> kArmJointNames = {"joint1", "joint2", "joint3", "joint4"};

// One unit of work inside a state. A state = an ordered queue of steps.
struct Step
{
  std::string label;
  std::function<void()> on_start;        // runs once when the step begins
  std::function<bool(double)> is_done;   // polled every tick, arg = elapsed sim seconds
  double timeout_s;                      // exceeded -> episode FAILED
};

// Prompt routing rule: blocks of `special_class` go to `special_bin`, others to the other bin.
struct PromptRule
{
  std::string axis;            // "stiffness" | "mass"
  std::string special_class;   // "soft" | "light"
  std::string special_bin;     // "blue_bin" | "red_bin"
};

class ProbeControllerNode : public rclcpp::Node
{
public:
  ProbeControllerNode()
  : rclcpp::Node("probe_controller_node")
  {
    // --- 1. PARAMETERS: waypoints (planar IK, FK-verified) ---
    home_ = declareTarget("pose_home", {0.0, -1.0, 0.3, 0.7});
    pregrasp_ = declareTarget("pose_pregrasp", {0.0, -0.388, 1.139, -0.652});  // pad z 0.09
    grasp_ = declareTarget("pose_grasp", {0.0, 0.352, 1.237, -1.489});         // pad z 0.035
    lift_ = declareTarget("pose_lift", {0.0, 0.119, 1.246, -1.264});           // pad z 0.05
    carry_ = declareTarget("pose_carry", {0.0, -0.533, 1.034, -0.401});        // pad z 0.11
    // Retreat high: r 0.20 m, z 0.17 at joint1 = 0 (swing back well above the bins).
    retreat_high_ = declareTarget("pose_retreat_high", {0.0, -0.659, 0.602, 0.156});
    const double bin_yaw_rad = declare_parameter<double>("bin_yaw_rad", 1.030);  // atan2(0.20, 0.12)

    // Place high: pad r 0.233 m, z 0.10 (clears the 3 cm bin walls).
    const JointTarget place_shape =
      declareTarget("pose_place_shape", {0.0, -0.182, 0.855, -0.573});
    place_blue_ = place_shape;
    place_blue_[0] = +bin_yaw_rad;   // blue bin at world +Y
    place_red_ = place_shape;
    place_red_[0] = -bin_yaw_rad;    // red bin at world -Y

    // Place low: pad z 0.06 -> block bottom ~20 mm above the bin floor.
    const JointTarget place_low_shape =
      declareTarget("pose_place_low_shape", {0.0, 0.138, 0.964, -1.002});
    place_low_blue_ = place_low_shape;
    place_low_blue_[0] = +bin_yaw_rad;
    place_low_red_ = place_low_shape;
    place_low_red_[0] = -bin_yaw_rad;

    // Vertical rise after release: IK at pad z 0.08 .. 0.17 m, reach 0.233 m.
    // Short segments keep the path within 1 mm of vertical.
    const std::vector<JointTarget> rise_shapes = {
      {0.0, -0.040, 0.925, -0.785},   // z 0.08
      {0.0, -0.182, 0.855, -0.573},   // z 0.10
      {0.0, -0.284, 0.759, -0.375},   // z 0.12
      {0.0, -0.350, 0.641, -0.191},   // z 0.14
      {0.0, -0.390, 0.427, 0.063}};   // z 0.17
    for (JointTarget shape : rise_shapes) {
      shape[0] = +bin_yaw_rad;
      rise_path_blue_.push_back(shape);
      shape[0] = -bin_yaw_rad;
      rise_path_red_.push_back(shape);
    }

    // --- 2. PARAMETERS: forces, thresholds, speed ---
    open_force_n_ = declare_parameter<double>("open_force_n", -1.0);
    contact_force_n_ = declare_parameter<double>("contact_force_n", 0.3);
    probe_force_n_ = declare_parameter<double>("probe_force_n", 2.0);
    regrasp_force_n_ = declare_parameter<double>("regrasp_force_n", 4.5);
    carry_force_light_n_ = declare_parameter<double>("carry_force_light_n", 3.0);
    carry_force_heavy_n_ = declare_parameter<double>("carry_force_heavy_n", 6.0);
    eps_soft_threshold_ = declare_parameter<double>("eps_soft_threshold", 0.06);
    slip_threshold_px_ = declare_parameter<double>("slip_threshold_px", 6.0);
    joint_tolerance_rad_ = declare_parameter<double>("joint_tolerance_rad", 0.03);
    max_regrasps_ = declare_parameter<int>("max_regrasps", 1);
    // Multiplies every ARM move duration (not the probe waits). < 1 = faster.
    // Read at each move start, so `ros2 param set` takes effect next move.
    declare_parameter<double>("motion_time_scale", 1.0);

    prompt_rules_ = {
      {"P_STIFF_0", {"stiffness", "soft", "blue_bin"}},
      {"P_STIFF_1", {"stiffness", "soft", "red_bin"}},
      {"P_MASS_0", {"mass", "light", "blue_bin"}},
      {"P_MASS_1", {"mass", "light", "red_bin"}},
    };

    // --- 3. ROS INTERFACES ---
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10),
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr & msg) {
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
          joint_positions_[msg->name[i]] = msg->position[i];
        }
      });
    metrics_sub_ = create_subscription<rt1_interfaces::msg::ProbeMetrics>(
      "/probe/metrics", rclcpp::QoS(10),
      [this](const rt1_interfaces::msg::ProbeMetrics::ConstSharedPtr & msg) {onMetrics(*msg);});
    episode_info_sub_ = create_subscription<rt1_interfaces::msg::EpisodeInfo>(
      "/episode/info", rclcpp::QoS(1).transient_local().reliable(),
      [this](const rt1_interfaces::msg::EpisodeInfo::ConstSharedPtr & msg) {
        episode_info_ = *msg;
        have_episode_info_ = true;
      });

    trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", rclcpp::QoS(10));
    grip_pub_ = create_publisher<std_msgs::msg::Float64>("/gripper/force_cmd", rclcpp::QoS(10));
    status_pub_ = create_publisher<FsmStatusMsg>("/fsm/status", rclcpp::QoS(10));

    latch_client_ = create_client<TriggerSrv>("/probe/latch_reference");
    clear_client_ = create_client<TriggerSrv>("/probe/clear_reference");

    run_service_ = create_service<TriggerSrv>(
      "/fsm/run_episode",
      [this](
        const std::shared_ptr<TriggerSrv::Request>,
        std::shared_ptr<TriggerSrv::Response> response) {onRunEpisode(response);});

    // 50 Hz on SIMULATION time: all waits scale with the real-time factor.
    tick_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.02), [this]() {onTick();});

    RCLCPP_INFO(get_logger(), "Expert FSM ready. Call /fsm/run_episode after /episode/next.");
  }

private:
  // ===========================================================================
  // HELPERS
  // ===========================================================================
  JointTarget declareTarget(const std::string & name, const std::vector<double> & defaults)
  {
    const auto values = declare_parameter<std::vector<double>>(name, defaults);
    if (values.size() != 4) {
      throw std::invalid_argument(name + " must have 4 joint values");
    }
    return {values[0], values[1], values[2], values[3]};
  }

  static std::string stateName(uint8_t state)
  {
    switch (state) {
      case FsmStatusMsg::IDLE: return "IDLE";
      case FsmStatusMsg::HOME: return "HOME";
      case FsmStatusMsg::APPROACH: return "APPROACH";
      case FsmStatusMsg::PINCH_TEST: return "PINCH_TEST";
      case FsmStatusMsg::MICRO_LIFT: return "MICRO_LIFT";
      case FsmStatusMsg::ROUTE_BIN: return "ROUTE_BIN";
      case FsmStatusMsg::RELEASE: return "RELEASE";
      case FsmStatusMsg::RETURN: return "RETURN";
      case FsmStatusMsg::DONE: return "DONE";
      case FsmStatusMsg::FAILED: return "FAILED";
      default: return "UNKNOWN";
    }
  }

  bool isRestingState() const
  {
    return state_ == FsmStatusMsg::IDLE || state_ == FsmStatusMsg::DONE ||
           state_ == FsmStatusMsg::FAILED;
  }

  double maxJointError(const JointTarget & target) const
  {
    double worst_error_rad = 0.0;
    for (size_t i = 0; i < kArmJointNames.size(); ++i) {
      const auto found = joint_positions_.find(kArmJointNames[i]);
      if (found == joint_positions_.end()) {
        return std::numeric_limits<double>::infinity();  // no joint state yet
      }
      worst_error_rad = std::max(worst_error_rad, std::abs(found->second - target[i]));
    }
    return worst_error_rad;
  }

  void publishTrajectory(const JointTarget & target, double duration_s)
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names.assign(kArmJointNames.begin(), kArmJointNames.end());
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(target.begin(), target.end());
    // One point: the controller interpolates (splines) from the CURRENT pose.
    point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
    trajectory.points.push_back(point);
    trajectory_pub_->publish(trajectory);
  }

  void publishGrip(double force_n)
  {
    grip_force_cmd_n_ = force_n;
    std_msgs::msg::Float64 msg;
    msg.data = force_n;
    grip_pub_->publish(msg);
  }

  // ===========================================================================
  // STEP FACTORIES
  // ===========================================================================
  // must_reach = false: the move may end in contact (placing into the bin), so it
  // completes after its duration + 0.3 s even if the arm stopped short.
  Step moveArm(const std::string & label, const JointTarget & target, double duration_s,
    bool stop_on_slip = false, bool must_reach = true)
  {
    // Shared so on_start can fix the scaled duration and is_done reads the same value.
    auto scaled_duration_s = std::make_shared<double>(duration_s);
    return Step{
      label,
      [this, target, duration_s, scaled_duration_s]() {
        const double scale = std::max(0.2, get_parameter("motion_time_scale").as_double());
        *scaled_duration_s = duration_s * scale;
        publishTrajectory(target, *scaled_duration_s);
      },
      [this, target, scaled_duration_s, stop_on_slip, must_reach](double elapsed_s) {
        if (stop_on_slip && slip_detected_) {
          return true;   // abort the lift early; MICRO_LIFT completion handles regrasp
        }
        if (!must_reach && elapsed_s >= *scaled_duration_s + 0.3) {
          return true;   // contact may stop the arm short of the target: that is fine
        }
        return elapsed_s >= *scaled_duration_s && maxJointError(target) < joint_tolerance_rad_;
      },
      duration_s * 2.0 + 4.0};   // generous: covers scale up to 2x
  }

  Step setGrip(const std::string & label, double force_n)
  {
    return Step{label, [this, force_n]() {publishGrip(force_n);}, [](double) {return true;}, 1.0};
  }

  Step waitFor(const std::string & label, double seconds, bool stop_on_slip = false)
  {
    return Step{
      label, nullptr,
      [this, seconds, stop_on_slip](double elapsed_s) {
        return (stop_on_slip && slip_detected_) || elapsed_s >= seconds;
      },
      seconds + 1.0};
  }

  Step instant(const std::string & label, std::function<void()> action)
  {
    return Step{label, std::move(action), [](double) {return true;}, 1.0};
  }

  Step callTrigger(const std::string & label, rclcpp::Client<TriggerSrv>::SharedPtr client)
  {
    // The future is shared between on_start and is_done; the response is
    // delivered by the executor between ticks (no blocking inside the timer).
    auto pending = std::make_shared<std::shared_future<TriggerSrv::Response::SharedPtr>>();
    auto send_if_ready = [client, pending]() {
        if (client->service_is_ready()) {
          *pending = client->async_send_request(
            std::make_shared<TriggerSrv::Request>()).future.share();
        }
      };
    return Step{
      label, send_if_ready,
      [this, pending, send_if_ready, label](double) {
        if (!pending->valid()) {   // service was not up at step start: keep trying
          send_if_ready();
          return false;
        }
        if (pending->wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
          return false;
        }
        const auto response = pending->get();
        if (!response->success) {
          fail("SERVICE_" + label + ": " + response->message);
          return false;
        }
        return true;
      },
      3.0};
  }

  Step measureStiffness()
  {
    return Step{
      "measure_eps",
      [this]() {
        eps_samples_.clear();
        collecting_eps_ = true;
      },
      [this](double elapsed_s) {
        // Average >= 5 valid samples over >= 0.4 s: rejects single-frame noise.
        if (elapsed_s < 0.4 || eps_samples_.size() < 5) {
          return false;
        }
        collecting_eps_ = false;
        eps_at_decision_ = std::accumulate(eps_samples_.begin(), eps_samples_.end(), 0.0) /
          static_cast<double>(eps_samples_.size());
        decided_stiffness_ = eps_at_decision_ >= eps_soft_threshold_ ? "soft" : "stiff";
        return true;
      },
      2.0};   // timeout -> no valid probe metrics (markers/block not visible)
  }

  std::vector<Step> liftSequence()
  {
    std::vector<Step> sequence;
    sequence.push_back(instant("slip_monitor_on", [this]() {
        slip_detected_ = false;
        monitoring_slip_ = true;
      }));
    sequence.push_back(moveArm("micro_lift", lift_, 1.0, true));
    sequence.push_back(waitFor("hold", 0.4, true));
    sequence.push_back(instant("slip_monitor_off", [this]() {monitoring_slip_ = false;}));
    return sequence;
  }

  // ===========================================================================
  // STATE MACHINE
  // ===========================================================================
  void enterState(uint8_t new_state)
  {
    state_ = new_state;
    steps_.clear();
    step_running_ = false;
    current_step_label_.clear();
    std::vector<Step> plan;

    if (new_state == FsmStatusMsg::HOME) {
      plan.push_back(callTrigger("clear_reference", clear_client_));
      plan.push_back(setGrip("open", open_force_n_));
      plan.push_back(moveArm("home", home_, 1.0));
    } else if (new_state == FsmStatusMsg::APPROACH) {
      plan.push_back(moveArm("pregrasp", pregrasp_, 1.2));
      plan.push_back(moveArm("grasp", grasp_, 1.0));
    } else if (new_state == FsmStatusMsg::PINCH_TEST) {
      plan.push_back(setGrip("contact", contact_force_n_));
      plan.push_back(waitFor("settle_contact", 0.5));
      plan.push_back(callTrigger("latch_w0", latch_client_));   // W0 = gap at first contact
      plan.push_back(setGrip("probe", probe_force_n_));
      plan.push_back(waitFor("settle_probe", 0.6));
      plan.push_back(measureStiffness());
    } else if (new_state == FsmStatusMsg::MICRO_LIFT) {
      if (regrasp_count_ > 0) {
        // Slip recovery: open, go back down, squeeze harder, lift again.
        plan.push_back(instant("slip_monitor_off", [this]() {monitoring_slip_ = false;}));
        plan.push_back(setGrip("open_for_regrasp", open_force_n_));
        plan.push_back(waitFor("opening", 0.3));
        plan.push_back(moveArm("regrasp_descend", grasp_, 0.8));
        plan.push_back(setGrip("regrasp_contact", contact_force_n_));
        plan.push_back(waitFor("regrasp_settle", 0.4));
        plan.push_back(setGrip("regrasp_squeeze", regrasp_force_n_));
        plan.push_back(waitFor("regrasp_hold", 0.5));
      }
      for (auto & step : liftSequence()) {
        plan.push_back(std::move(step));
      }
    } else if (new_state == FsmStatusMsg::ROUTE_BIN) {
      const bool to_blue = decided_bin_ == "blue_bin";
      const double carry_force_n =
        decided_mass_ == "heavy" ? carry_force_heavy_n_ : carry_force_light_n_;
      plan.push_back(setGrip("carry_grip", carry_force_n));
      plan.push_back(waitFor("carry_settle", 0.3));
      plan.push_back(moveArm("carry_raise", carry_, 1.0));
      plan.push_back(moveArm("to_bin", to_blue ? place_blue_ : place_red_, 3.0));
      plan.push_back(moveArm("lower_into_bin", to_blue ? place_low_blue_ : place_low_red_, 1.0,
        false, false));   // best-effort: may end in contact with the bin floor
    } else if (new_state == FsmStatusMsg::RELEASE) {
      plan.push_back(setGrip("release", open_force_n_));
      plan.push_back(waitFor("fingers_open", 0.6));   // fingers open in ~0.1 s
    } else if (new_state == FsmStatusMsg::RETURN) {
      // Lift straight up first (near-vertical segments to z 0.17), then swing
      // back high, then home: the open fingers never touch the block.
      const auto & rise_path = decided_bin_ == "blue_bin" ? rise_path_blue_ : rise_path_red_;
      for (size_t i = 0; i < rise_path.size(); ++i) {
        plan.push_back(moveArm("rise_" + std::to_string(i), rise_path[i], 0.25));
      }
      plan.push_back(moveArm("retreat_high", retreat_high_, 1.5));
      plan.push_back(moveArm("home", home_, 1.5));
    }
    // IDLE / DONE / FAILED have no steps.

    for (auto & step : plan) {
      steps_.push_back(std::move(step));
    }
    RCLCPP_INFO(get_logger(), "[ep %u] -> %s", active_episode_id_, stateName(state_).c_str());
  }

  void onStateComplete()
  {
    if (state_ == FsmStatusMsg::HOME) {
      enterState(FsmStatusMsg::APPROACH);
    } else if (state_ == FsmStatusMsg::APPROACH) {
      enterState(FsmStatusMsg::PINCH_TEST);
    } else if (state_ == FsmStatusMsg::PINCH_TEST) {
      enterState(FsmStatusMsg::MICRO_LIFT);
    } else if (state_ == FsmStatusMsg::MICRO_LIFT) {
      if (slip_detected_) {
        if (regrasp_count_ < max_regrasps_) {
          ++regrasp_count_;
          enterState(FsmStatusMsg::MICRO_LIFT);   // same state, regrasp plan
        } else {
          fail("SLIP_UNRECOVERED");
        }
        return;
      }
      // Slipped at the baseline grip at least once -> heavy.
      decided_mass_ = regrasp_count_ > 0 ? "heavy" : "light";
      if (!decideBin()) {
        return;
      }
      enterState(FsmStatusMsg::ROUTE_BIN);
    } else if (state_ == FsmStatusMsg::ROUTE_BIN) {
      enterState(FsmStatusMsg::RELEASE);
    } else if (state_ == FsmStatusMsg::RELEASE) {
      enterState(FsmStatusMsg::RETURN);
    } else if (state_ == FsmStatusMsg::RETURN) {
      state_ = FsmStatusMsg::DONE;
      RCLCPP_INFO(get_logger(), "[ep %u] DONE: %s/%s -> %s (truth %s)",
        active_episode_id_, decided_stiffness_.c_str(), decided_mass_.c_str(),
        decided_bin_.c_str(), episode_info_.target_bin.c_str());
    }
  }

  bool decideBin()
  {
    const auto rule = prompt_rules_.find(episode_info_.prompt_id);
    if (rule == prompt_rules_.end()) {
      fail("UNKNOWN_PROMPT_" + episode_info_.prompt_id);
      return false;
    }
    const std::string & block_class =
      rule->second.axis == "stiffness" ? decided_stiffness_ : decided_mass_;
    const std::string other_bin = rule->second.special_bin == "blue_bin" ? "red_bin" : "blue_bin";
    decided_bin_ = block_class == rule->second.special_class ? rule->second.special_bin : other_bin;
    decision_made_ = true;
    // Ground truth is used ONLY to flag the expert, never to steer it.
    decision_matches_truth_ = decided_bin_ == episode_info_.target_bin;
    return true;
  }

  void fail(const std::string & code)
  {
    failure_code_ = code;
    state_ = FsmStatusMsg::FAILED;
    steps_.clear();
    step_running_ = false;
    collecting_eps_ = false;
    monitoring_slip_ = false;
    // Safe state: open the gripper and park the arm at home, so the next block
    // (spawned at the grasp location) can never appear around the fingers.
    publishGrip(open_force_n_);
    publishTrajectory(home_, 3.0);
    RCLCPP_WARN(get_logger(), "[ep %u] FAILED: %s", active_episode_id_, code.c_str());
  }

  void onTick()
  {
    if (isRestingState()) {
      publishStatus();
      return;
    }
    if (steps_.empty()) {
      onStateComplete();
      publishStatus();
      return;
    }
    const rclcpp::Time now_sim = now();
    if (!step_running_) {
      step_running_ = true;
      step_start_time_ = now_sim;
      current_step_label_ = steps_.front().label;
      if (steps_.front().on_start) {
        steps_.front().on_start();
      }
    }
    const double elapsed_s = (now_sim - step_start_time_).seconds();
    const bool step_done = steps_.front().is_done(elapsed_s);
    if (state_ == FsmStatusMsg::FAILED) {   // is_done may have failed the episode
      publishStatus();
      return;
    }
    if (step_done) {
      steps_.pop_front();
      step_running_ = false;
    } else if (elapsed_s > steps_.front().timeout_s) {
      fail("TIMEOUT_" + stateName(state_) + "_" + current_step_label_);
    }
    publishStatus();
  }

  void onMetrics(const rt1_interfaces::msg::ProbeMetrics & metrics)
  {
    if (!metrics.valid || !metrics.reference_latched) {
      return;
    }
    if (collecting_eps_) {
      eps_samples_.push_back(metrics.eps_vis);
    }
    // Rows grow downward: block lagging below the rising fingers -> dy_rel > 0.
    if (monitoring_slip_ && metrics.dy_rel_px > slip_threshold_px_) {
      slip_detected_ = true;
    }
  }

  void onRunEpisode(std::shared_ptr<TriggerSrv::Response> response)
  {
    if (!isRestingState()) {
      response->success = false;
      response->message = "busy in " + stateName(state_);
      return;
    }
    if (!have_episode_info_) {
      response->success = false;
      response->message = "no /episode/info yet: call /episode/next first";
      return;
    }
    if (has_run_once_ && episode_info_.episode_id == last_run_episode_id_) {
      response->success = false;
      response->message = "episode already run: call /episode/next first";
      return;
    }
    // Reset per-episode memory.
    has_run_once_ = true;
    last_run_episode_id_ = episode_info_.episode_id;
    active_episode_id_ = episode_info_.episode_id;
    failure_code_.clear();
    decision_made_ = false;
    decision_matches_truth_ = false;
    decided_stiffness_.clear();
    decided_mass_.clear();
    decided_bin_.clear();
    eps_at_decision_ = 0.0;
    slip_detected_ = false;
    regrasp_count_ = 0;
    enterState(FsmStatusMsg::HOME);
    response->success = true;
    response->message = "started episode " + std::to_string(active_episode_id_);
  }

  void publishStatus()
  {
    FsmStatusMsg status;
    status.header.stamp = now();
    status.episode_id = active_episode_id_;   // the episode being RUN, not the newest spawned
    status.state = state_;
    status.state_name = stateName(state_);
    status.step_label = current_step_label_;
    status.failure_code = failure_code_;
    status.grip_force_cmd_n = grip_force_cmd_n_;
    status.decision_made = decision_made_;
    status.decided_stiffness = decided_stiffness_;
    status.decided_mass = decided_mass_;
    status.decided_bin = decided_bin_;
    status.eps_vis_at_decision = eps_at_decision_;
    status.slip_detected = slip_detected_;
    status.regrasp_count = static_cast<uint8_t>(regrasp_count_);
    status.decision_matches_truth = decision_matches_truth_;
    status_pub_->publish(status);
  }

  // --- waypoints & tuning ---
  JointTarget home_{}, pregrasp_{}, grasp_{}, lift_{}, carry_{}, retreat_high_{};
  JointTarget place_blue_{}, place_red_{};
  JointTarget place_low_blue_{}, place_low_red_{};
  std::vector<JointTarget> rise_path_blue_, rise_path_red_;
  double open_force_n_{-1.0}, contact_force_n_{0.3}, probe_force_n_{2.0}, regrasp_force_n_{4.5};
  double carry_force_light_n_{3.0}, carry_force_heavy_n_{6.0};
  double eps_soft_threshold_{0.06}, slip_threshold_px_{6.0}, joint_tolerance_rad_{0.03};
  int max_regrasps_{1};
  std::unordered_map<std::string, PromptRule> prompt_rules_;

  // --- live inputs ---
  std::unordered_map<std::string, double> joint_positions_;
  rt1_interfaces::msg::EpisodeInfo episode_info_;
  bool have_episode_info_{false};

  // --- FSM state ---
  uint8_t state_{FsmStatusMsg::IDLE};
  std::deque<Step> steps_;
  bool step_running_{false};
  rclcpp::Time step_start_time_{0, 0, RCL_ROS_TIME};
  std::string current_step_label_;
  std::string failure_code_;
  bool has_run_once_{false};
  uint32_t last_run_episode_id_{0};
  uint32_t active_episode_id_{0};

  // --- decisions & measurements ---
  double grip_force_cmd_n_{0.0};
  bool collecting_eps_{false};
  std::vector<double> eps_samples_;
  double eps_at_decision_{0.0};
  bool monitoring_slip_{false};
  bool slip_detected_{false};
  int regrasp_count_{0};
  bool decision_made_{false};
  bool decision_matches_truth_{false};
  std::string decided_stiffness_, decided_mass_, decided_bin_;

  // --- ROS handles ---
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<rt1_interfaces::msg::ProbeMetrics>::SharedPtr metrics_sub_;
  rclcpp::Subscription<rt1_interfaces::msg::EpisodeInfo>::SharedPtr episode_info_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr grip_pub_;
  rclcpp::Publisher<FsmStatusMsg>::SharedPtr status_pub_;
  rclcpp::Client<TriggerSrv>::SharedPtr latch_client_, clear_client_;
  rclcpp::Service<TriggerSrv>::SharedPtr run_service_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace rt1_data_pipeline

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rt1_data_pipeline::ProbeControllerNode>());
  rclcpp::shutdown();
  return 0;
}