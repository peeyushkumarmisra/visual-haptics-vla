// =============================================================================
// probe_metrics_node.cpp
// -----------------------------------------------------------------------------
// Target:
//   ROS wrapper around the probe_metrics library: read the stitched canvas,
//   measure the cam_side tile, publish probe signals with the SAME stamp as
//   the image (so the episode logger pairs them exactly).
//
// Physics / Math Implemented: see probe_metrics.hpp.
//
// Inputs & Outputs:
//   In : /stitched_canvas/image_raw     sensor_msgs/Image rgb8 224x224
//   Out: /probe/metrics                 rt1_interfaces/ProbeMetrics, ~30 Hz
//   Srv: /probe/latch_reference         std_srvs/Trigger, W0 := current d_px
//        /probe/clear_reference         std_srvs/Trigger, forget W0 (new episode)
// =============================================================================
#include <memory>
#include <string>
#include <vector>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rt1_data_pipeline/probe_metrics.hpp"
#include "rt1_interfaces/msg/probe_metrics.hpp"

namespace rt1_data_pipeline
{

class ProbeMetricsNode : public rclcpp::Node
{
public:
    ProbeMetricsNode(): rclcpp::Node("probe_metrics_node")
    {
        // --- 1. PARAMETERS ---
        const auto canvas_topic = declare_parameter<std::string>("canvas_topic", "/stitched_canvas/image_raw");
        const auto metrics_topic = declare_parameter<std::string>("metrics_topic", "/probe/metrics");
        // cam_side occupies the top tile of the canvas (stitcher layout).
        side_tile_height_px_ = declare_parameter<int>("side_tile_height_px", 112);
        config_.marker_hsv.lower_hsv = toScalar(declare_parameter<std::vector<int64_t>>("marker_hsv_lower", {45, 120, 40}));
        config_.marker_hsv.upper_hsv = toScalar(declare_parameter<std::vector<int64_t>>("marker_hsv_upper", {75, 255, 255}));
        config_.block_hsv.lower_hsv = toScalar(declare_parameter<std::vector<int64_t>>("block_hsv_lower", {125, 80, 40}));
        config_.block_hsv.upper_hsv = toScalar(declare_parameter<std::vector<int64_t>>("block_hsv_upper", {155, 255, 255}));
        config_.min_marker_area_px = declare_parameter<double>("min_marker_area_px", 20.0);
        config_.min_block_area_px = declare_parameter<double>("min_block_area_px", 200.0);
        config_.min_marker_area_ratio = declare_parameter<double>("min_marker_area_ratio", 0.6);

        // --- 2. ROS INTERFACES ---
        // Reliable depth 10 matches the stitcher's publisher.
        canvas_subscription_ = 
            create_subscription<sensor_msgs::msg::Image>(
                canvas_topic, rclcpp::QoS(10),
                [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) 
                {onCanvas(msg);}
            );
        metrics_publisher_ = 
            create_publisher<rt1_interfaces::msg::ProbeMetrics>(
                metrics_topic, rclcpp::QoS(10)
            );
        latch_service_ = 
            create_service<std_srvs::srv::Trigger>(
                "/probe/latch_reference",
                [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
                {onLatch(response);}
            );
        clear_service_ = 
            create_service<std_srvs::srv::Trigger>(
                "/probe/clear_reference",
                [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
                {
                    reference_.reset();
                    response->success = true;
                    response->message = "reference cleared";
                    RCLCPP_INFO(get_logger(), "Reference cleared");
                }
            );

        RCLCPP_INFO(get_logger(), "Probe metrics on [%s] top %d rows -> [%s]",
        canvas_topic.c_str(), side_tile_height_px_, metrics_topic.c_str());
    }

private:
    static cv::Scalar toScalar(const std::vector<int64_t> & hsv_triplet)
    {
        if (hsv_triplet.size() != 3) 
        {
            throw std::invalid_argument("HSV parameter must have exactly 3 values [H, S, V]");
        }
        return cv::Scalar(
            static_cast<double>(hsv_triplet[0]),
            static_cast<double>(hsv_triplet[1]),
            static_cast<double>(hsv_triplet[2])
        );
    }

    // --- 3. PER-FRAME PROCESSING ---
    void onCanvas(const sensor_msgs::msg::Image::ConstSharedPtr & canvas_msg)
    {
        cv_bridge::CvImageConstPtr canvas_cv;
        try 
        {
            canvas_cv = cv_bridge::toCvShare(canvas_msg, "rgb8");
        } 
        catch (const cv_bridge::Exception & conversion_error) 
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", conversion_error.what());
            return;
        }
        if (canvas_cv->image.rows < side_tile_height_px_) 
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Canvas smaller than side tile");
            return;
        }
        // ROI view of the top tile: no pixel copy, just a window into the canvas.
        const cv::Mat side_tile_rgb = canvas_cv->image(cv::Rect(0, 0, canvas_cv->image.cols, side_tile_height_px_));
        last_measurement_ = probe::measureFrame(side_tile_rgb, config_);
        const probe::ProbeMetrics metrics = probe::computeMetrics(last_measurement_, reference_, config_);
        rt1_interfaces::msg::ProbeMetrics metrics_msg;
        metrics_msg.header = canvas_msg->header;   // same stamp as the image it came from
        metrics_msg.valid = metrics.valid;
        metrics_msg.reference_latched = reference_.has_value();
        metrics_msg.d_px = last_measurement_.d_px;
        metrics_msg.w0_px = reference_ ? reference_->w0_px : 0.0;
        metrics_msg.eps_vis = metrics.eps_vis;
        metrics_msg.y_fingers_px = last_measurement_.y_fingers_px;
        metrics_msg.y_block_px = last_measurement_.y_block_px;
        metrics_msg.dy_rel_px = metrics.dy_rel_px;
        metrics_msg.left_marker_area_px = last_measurement_.left_marker.area_px;
        metrics_msg.right_marker_area_px = last_measurement_.right_marker.area_px;
        metrics_publisher_->publish(metrics_msg);
    }

    // --- 4. LATCH W0 FROM THE MOST RECENT FRAME ---
    // Single-threaded executor: this never runs at the same time as onCanvas,
    // so reading last_measurement_ here is safe without a mutex.
    void onLatch(std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!last_measurement_.markers_found || !last_measurement_.block_found) {
        response->success = false;
        response->message = "cannot latch: markers or block not visible in the last frame";
        return;
        }
        reference_ = probe::makeReference(last_measurement_);
        response->success = true;
        response->message = "W0 = " + std::to_string(reference_->w0_px) + " px";
        RCLCPP_INFO(get_logger(), "Latched %s", response->message.c_str());
    }

    int side_tile_height_px_{112};
    probe::ProbeConfig config_;
    probe::FrameMeasurement last_measurement_;
    std::optional<probe::ProbeReference> reference_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr canvas_subscription_;
    rclcpp::Publisher<rt1_interfaces::msg::ProbeMetrics>::SharedPtr metrics_publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr latch_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
};

}  // namespace rt1_data_pipeline

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rt1_data_pipeline::ProbeMetricsNode>());
    rclcpp::shutdown();
    return 0;
}
