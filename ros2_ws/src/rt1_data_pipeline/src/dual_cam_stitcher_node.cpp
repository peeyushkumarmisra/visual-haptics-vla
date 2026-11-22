// =================================================================================================
// dual_cam_stitcher_node.cpp
// -------------------------------------------------------------------------------------------------
// Target:            see dual_cam_stitcher_node.hpp (manifest header).
// Physics / Math:    approximate-time sync + INTER_AREA 2x downsample + vconcat.
// Inputs & Outputs:  2 x rgb8 448x224 -> 1 x rgb8 224x224 on /stitched_canvas/image_raw.
// ================================================================================================

#include "rt1_data_pipeline/dual_cam_stitcher_node.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <functional>

namespace rt1_data_pipeline
{

DualCamStitcherNode::DualCamStitcherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("dual_cam_stitcher_node", options)
{
    // --- 1. PARAMETERS ---
    side_topic_ = declare_parameter<std::string>("side_topic", "/cam_side/image");
    top_topic_ = declare_parameter<std::string>("top_topic", "/cam_top/image");
    output_topic_ = declare_parameter<std::string>("output_topic", "/stitched_canvas/image_raw");
    // 10 ms slop: both cameras render at 30 Hz (33 ms period) on the same sim
    // clock, so a true pair is ~0 ms apart; 10 ms rejects frames from the
    // neighbouring tick (33 ms away) while tolerating render jitter.
    sync_slop_seconds_ = declare_parameter<double>("sync_slop_s", 0.010);
    tile_width_px_ = declare_parameter<int>("tile_width_px", 224);
    tile_height_px_ = declare_parameter<int>("tile_height_px", 112);
    // --- 2. SUBSCRIBERS (sensor-data QoS) ---
    // Sensor-data QoS = best-effort, shallow queue: a late camera frame is
    // worthless, so we never want the middleware re-sending old images.
    // A best-effort subscriber still connects to the bridge's reliable publisher.
    side_image_subscriber_.subscribe(this, side_topic_, rmw_qos_profile_sensor_data);
    top_image_subscriber_.subscribe(this, top_topic_, rmw_qos_profile_sensor_data);
    // --- 3. APPROXIMATE-TIME SYNCHRONIZER ---
    // Queue size 10 = up to ~0.33 s of frames per camera kept while searching
    // for the best-matching pair.
    image_synchronizer_ = std::make_shared<ImageSynchronizer>(
        ApproxSyncPolicy(10), side_image_subscriber_, top_image_subscriber_);
    // Hard upper bound on the stamp gap inside one pair.
    image_synchronizer_->setMaxIntervalDuration(
        rclcpp::Duration::from_seconds(sync_slop_seconds_));
    image_synchronizer_->registerCallback(
        std::bind(
            &DualCamStitcherNode::onSynchronizedPair, this,
            std::placeholders::_1, std::placeholders::_2));
    // --- 4. PUBLISHER ---
    // Reliable, depth 10: downstream consumers (episode logger, rqt_image_view)
    // must not silently drop canvases, and reliable is what rqt expects.
    stitched_canvas_publisher_ = create_publisher<ImageMsg>(output_topic_, rclcpp::QoS(10));
    RCLCPP_INFO(
        get_logger(), "Stitching [%s] (top) + [%s] (bottom) -> [%s], tile %dx%d, slop %.3f s",
        side_topic_.c_str(), top_topic_.c_str(), output_topic_.c_str(), tile_width_px_, 
        tile_height_px_, sync_slop_seconds_);
}

cv::Mat DualCamStitcherNode::resizeToTile(const cv::Mat & full_frame_rgb) const
{
    cv::Mat tile_rgb;
    // INTER_AREA averages every source pixel that falls into a target pixel.
    // For downsampling this is anti-aliased: thin edges (plate seams, fingertip
    // markers) shrink smoothly instead of flickering in and out, which matters
    // because the policy reads millimetre-scale compression from these pixels.
    cv::resize(
        full_frame_rgb, tile_rgb, cv::Size(tile_width_px_, tile_height_px_),
        0.0, 0.0, cv::INTER_AREA);
    return tile_rgb;
}

void DualCamStitcherNode::onSynchronizedPair(
    const ImageMsg::ConstSharedPtr & side_image_msg,
    const ImageMsg::ConstSharedPtr & top_image_msg)
{
    // --- 1. ROS IMAGE -> OPENCV (zero-copy when already rgb8) ---
    cv_bridge::CvImageConstPtr side_cv_image;
    cv_bridge::CvImageConstPtr top_cv_image;
    try 
    {
        // toCvShare reuses the message buffer instead of copying it, because the
        // encoding already matches; we only read these pixels, never modify them.
        side_cv_image = cv_bridge::toCvShare(side_image_msg, "rgb8");
        top_cv_image = cv_bridge::toCvShare(top_image_msg, "rgb8");
    } 
    catch (const cv_bridge::Exception & conversion_error) 
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "cv_bridge conversion failed: %s",conversion_error.what());
    return;
    }
    // --- 2. RESIZE EACH CAMERA TO ONE TILE ---
    const cv::Mat side_tile_rgb = resizeToTile(side_cv_image->image);
    const cv::Mat top_tile_rgb = resizeToTile(top_cv_image->image);
    // --- 3. VERTICAL STACK: cam_side on top, cam_top below ---
    // Fixed layout = fixed meaning per pixel row; the policy learns
    // "upper half = strain view, lower half = gap view".
    cv::Mat stitched_canvas_rgb;
    cv::vconcat(side_tile_rgb, top_tile_rgb, stitched_canvas_rgb);
    // --- 4. TIMESTAMP = OLDER INPUT STAMP ---
    // The canvas can be no fresher than its oldest pixel. Claiming the newer
    // stamp would make the canvas look ahead of the robot state it is paired
    // with in the episode logger, a small but real causality error.
    const rclcpp::Time side_stamp(side_image_msg->header.stamp);
    const rclcpp::Time top_stamp(top_image_msg->header.stamp);
    std_msgs::msg::Header canvas_header;
    canvas_header.stamp = (side_stamp < top_stamp) ? side_image_msg->header.stamp : top_image_msg->header.stamp;
    canvas_header.frame_id = "stitched_canvas";
    // --- 5. OPENCV -> ROS AND PUBLISH ---
    stitched_canvas_publisher_->publish(
        *cv_bridge::CvImage(canvas_header, "rgb8", stitched_canvas_rgb).toImageMsg());
}

}  // namespace rt1_data_pipeline

// --- ENTRY POINT ---
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rt1_data_pipeline::DualCamStitcherNode>());
  rclcpp::shutdown();
  return 0;
}