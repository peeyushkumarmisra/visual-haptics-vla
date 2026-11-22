// =============================================================================
// probe_metrics.hpp  (pure library: OpenCV only, no ROS)
// -----------------------------------------------------------------------------
// Target:
//   Turn ONE cam_side tile into the physical probe signals the RT-1 policy is
//   trained to reason about: visual strain (stiffness cue) and relative slip
//   (mass cue). Kept ROS-free so it can be unit-tested on synthetic images.
//
// Physics / Math Implemented:
//   - Colour segmentation in HSV: hue identifies the material (marker green,
//     block purple) independent of shading, which mostly changes V.
//   - Blob centroid = first image moments m10/m00, m01/m00 (sub-pixel).
//   - Fingertip gap      d_px  = x_right - x_left
//   - Visual strain      eps_vis = (W0 - d_px) / W0, W0 = d_px at first contact.
//       Rigid block: d_px stays at W0      -> eps_vis ~ 0
//       Soft block : plates compress 2x    -> eps_vis = 2x / W0_mm  (> 0)
//   - Relative slip      dy_rel = (y_block - y_fingers) - (y_block - y_fingers)|latch
//       Held block moves with the fingers -> dy_rel ~ 0
//       Slipping block lags (rows grow downward) -> dy_rel > 0
//
// Inputs & Outputs:
//   In : cam_side tile, cv::Mat CV_8UC3 RGB, 224x112 (any size works)
//   Out: FrameMeasurement (pixel geometry), ProbeMetrics (eps_vis, dy_rel, valid)
// =============================================================================
#ifndef RT1_DATA_PIPELINE__PROBE_METRICS_HPP_
#define RT1_DATA_PIPELINE__PROBE_METRICS_HPP_

#include <opencv2/core.hpp>
#include <optional>
#include <vector>

namespace rt1_data_pipeline::probe
{

// OpenCV HSV Scale: H 0..180 (degrees / 2), S 0..255, V 0..255.
struct HsvRange
{
    cv::Scalar lower_hsv;
    cv::Scalar upper_hsv;
};

struct Blob
{
    cv::Point2d centroid_px;  // (column, row), sub-pixel
    double area_px{0.0};
};

struct ProbeConfig
{
    // Green markers: hue ~60 (120 deg). Loose V lower bound because scene
    // shading darkens the marker (seen in the 2.2a screenshot).
    HsvRange marker_hsv{cv::Scalar(45, 120, 40), cv::Scalar(75, 255, 255)};
    // Purple block (rgb 0.55, 0.20, 0.75): hue ~140 (280 deg).
    HsvRange block_hsv{cv::Scalar(125, 80, 40), cv::Scalar(155, 255, 255)};
    double min_marker_area_px{20.0};   // rejects single-pixel noise
    double min_block_area_px{200.0};
    // Once latched, a marker smaller than this fraction of its latch area is
    // treated as partly hidden behind a plate: its centroid would be biased.
    double min_marker_area_ratio{0.6};
};

// Raw geometry of one frame (no reference needed).
struct FrameMeasurement
{
    bool markers_found{false};
    bool block_found{false};
    Blob left_marker;    // image-left  (cam_side looks along -X: image left = world -Y)
    Blob right_marker;   // image-right
    Blob block;
    double d_px{0.0};
    double y_fingers_px{0.0};
    double y_block_px{0.0};
};

// Snapshot taken at first contact (PINCH_TEST); the zero for strain and slip.
struct ProbeReference
{
    double w0_px{0.0};
    double dy0_px{0.0};                 // (y_block - y_fingers) at latch
    double left_marker_area_px{0.0};
    double right_marker_area_px{0.0};
};

struct ProbeMetrics
{
    bool valid{false};
    double eps_vis{0.0};
    double dy_rel_px{0.0};
};

// All blobs of one colour, largest first, each at least min_area_px.
std::vector<Blob> findBlobs(
    const cv::Mat & hsv_image, 
    const HsvRange & range, 
    double min_area_px);

FrameMeasurement measureFrame(
    const cv::Mat & side_tile_rgb, 
    const ProbeConfig & config);

ProbeReference makeReference(
    const FrameMeasurement & measurement);

ProbeMetrics computeMetrics(
    const FrameMeasurement & measurement,
    const std::optional<ProbeReference> & reference,
    const ProbeConfig & config);

}  // namespace rt1_data_pipeline::probe

#endif  // RT1_DATA_PIPELINE__PROBE_METRICS_HPP_
