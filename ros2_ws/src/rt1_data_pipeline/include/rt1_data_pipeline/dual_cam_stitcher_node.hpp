// =================================================================================================
// dual_cam_stitcher_node.hpp
// -------------------------------------------------------------------------------------------------
// Target:
//  Fuse the two fixed RGB cameras (cam_side, cam_top) into ONE 224x224 image, the single visual 
//  input the RT-1 policy sees each control tick.
//
// Physics / Math Implemented:
//   - Temporal alignment: approximate-time pairing, |t_side - t_top| <= slop.
//   - Spatial resampling: 448x224 -> 224x112 per camera (exact 2x area downsample, aspect ratio 
//                         preserved), then vertical concatenation:
//     canvas rows   0..111 = cam_side  (strain / plate compression view)
//     canvas rows 112..223 = cam_top   (fingertip gap / routing view)
//
// Inputs & Outputs:
//  In: /cam_side/image, /cam_top/image sensor_msgs/Image, rgb8, 448x224, 30 Hz in simulation time, 
//       optical frames (REP-103)
//  Out: /stitched_canvas/image_raw sensor_msgs/Image, rgb8, 224x224, stamp = OLDER of the two 
//       input stamps, frame_id "stitched_canvas"
// ================================================================================================
#ifndef RT1_DATA_PIPELINE__DUAL_CAM_STITCHER_NODE_HPP_
#define RT1_DATA_PIPELINE__DUAL_CAM_STITCHER_NODE_HPP_

#include <memory>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rt1_data_pipeline
{

class DualCamStitcherNode : public rclcpp::Node
{
public:
  explicit DualCamStitcherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --- Type aliases: keep the long message_filters names readable ---
  using ImageMsg = sensor_msgs::msg::Image;
  using ImageSubscriber = message_filters::Subscriber<ImageMsg>;
  // ApproximateTime: pairs messages whose stamps are close but not identical
  // (two sensors never fire in the exact same nanosecond).
  using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;
  using ImageSynchronizer = message_filters::Synchronizer<ApproxSyncPolicy>;

  // Called only when one cam_side frame and one cam_top frame are matched.
  void onSynchronizedPair(
    const ImageMsg::ConstSharedPtr & side_image_msg,
    const ImageMsg::ConstSharedPtr & top_image_msg);

  // Resize one camera frame to the tile size used inside the canvas.
  cv::Mat resizeToTile(const cv::Mat & full_frame_rgb) const;

  // --- Parameters (declared in the constructor, overridable at launch) ---
  std::string side_topic_;
  std::string top_topic_;
  std::string output_topic_;
  double sync_slop_seconds_;
  int tile_width_px_;
  int tile_height_px_;

  // --- ROS interfaces ---
  // ORDER MATTERS: the synchronizer holds references to both subscribers,
  // so the subscribers must be declared (constructed) before it.
  ImageSubscriber side_image_subscriber_;
  ImageSubscriber top_image_subscriber_;
  std::shared_ptr<ImageSynchronizer> image_synchronizer_;
  rclcpp::Publisher<ImageMsg>::SharedPtr stitched_canvas_publisher_;
};

}  // namespace rt1_data_pipeline

#endif  // RT1_DATA_PIPELINE__DUAL_CAM_STITCHER_NODE_HPP_