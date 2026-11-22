// =============================================================================
// probe_metrics.cpp
// -----------------------------------------------------------------------------
// Target:            see probe_metrics.hpp (manifest).
// Physics / Math:    HSV segmentation -> connected components -> centroids ->
//                    d_px, eps_vis, dy_rel.
// Inputs & Outputs:  RGB tile -> FrameMeasurement -> ProbeMetrics.
// =============================================================================
#include "rt1_data_pipeline/probe_metrics.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <utility>

namespace rt1_data_pipeline::probe
{

// --- 1. COLOUR SEGMENTATION + BLOB EXTRACTION ---
std::vector<Blob> findBlobs(
    const cv::Mat & hsv_image, 
    const HsvRange & range, 
    double min_area_px)
{
    // Binary mask: 255 where the pixel's H, S and V all lie inside the range.
    cv::Mat colour_mask;
    cv::inRange(hsv_image, range.lower_hsv, range.upper_hsv, colour_mask);
    // Group touching mask pixels into separate blobs (8-connectivity so a
    // diagonal anti-aliased edge does not split one marker into two).
    cv::Mat component_labels;
    cv::Mat component_stats;
    cv::Mat component_centroids;
    const int component_count = cv::connectedComponentsWithStats(
        colour_mask, component_labels, component_stats, component_centroids, 8, CV_32S);
    std::vector<Blob> blobs;
    // Label 0 is always the background, so real blobs start at 1.
    for (int label = 1; label < component_count; ++label) 
    {
        const double area_px = component_stats.at<int>(label, cv::CC_STAT_AREA);
        if (area_px < min_area_px) 
        {   
            continue;   
        }
        // Centroid = mean pixel position = first moments / area (sub-pixel).
        blobs.push_back(
            {cv::Point2d(
                component_centroids.at<double>(label, 0),
                component_centroids.at<double>(label, 1)),
            area_px});
    }
    // Largest first: the true markers/block dominate any stray speckles.
    std::sort(
        blobs.begin(), blobs.end(),
        [](const Blob & a, const Blob & b) {return a.area_px > b.area_px;});
    return blobs;
}


// --- 2. PER-FRAME GEOMETRY ---
FrameMeasurement measureFrame(const cv::Mat & side_tile_rgb, const ProbeConfig & config)
{
    FrameMeasurement measurement;
    // Convert once; both colour searches reuse the same HSV image.
    cv::Mat side_tile_hsv;
    cv::cvtColor(side_tile_rgb, side_tile_hsv, cv::COLOR_RGB2HSV);
    // Fingertip markers: exactly two expected; take the two largest.
    const std::vector<Blob> marker_blobs =
        findBlobs(side_tile_hsv, config.marker_hsv, config.min_marker_area_px);
    if (marker_blobs.size() >= 2) 
    {
        Blob marker_a = marker_blobs[0];
        Blob marker_b = marker_blobs[1];
        // Order by column so d_px is always positive.
        if (marker_a.centroid_px.x > marker_b.centroid_px.x) 
        {
            std::swap(marker_a, marker_b);
        }
        measurement.left_marker = marker_a;
        measurement.right_marker = marker_b;
        measurement.d_px = marker_b.centroid_px.x - marker_a.centroid_px.x;
        measurement.y_fingers_px = 0.5 * (marker_a.centroid_px.y + marker_b.centroid_px.y);
        measurement.markers_found = true;
    }
    // Block: the single largest purple region.
    const std::vector<Blob> block_blobs =
        findBlobs(side_tile_hsv, config.block_hsv, config.min_block_area_px);
    if (!block_blobs.empty()) 
    {
        measurement.block = block_blobs.front();
        measurement.y_block_px = measurement.block.centroid_px.y;
        measurement.block_found = true;
    }
  return measurement;
}


// --- 3. REFERENCE AT FIRST CONTACT ---
ProbeReference makeReference(const FrameMeasurement & measurement)
{
    ProbeReference reference;
    reference.w0_px  = measurement.d_px;
    reference.dy0_px = measurement.y_block_px - measurement.y_fingers_px;
    reference.left_marker_area_px  = measurement.left_marker.area_px;
    reference.right_marker_area_px = measurement.right_marker.area_px;
    return reference;
}

// --- 4. STRAIN + SLIP ---
ProbeMetrics computeMetrics(
    const FrameMeasurement & measurement,
    const std::optional<ProbeReference> & reference,
    const ProbeConfig & config)
{
    ProbeMetrics metrics;  // valid = false by default

    if (!measurement.markers_found || !measurement.block_found) 
    {
        return metrics;
    }
    if (!reference.has_value()) 
    {
        // Geometry is good but there is no zero yet: strain/slip stay 0.
        metrics.valid = true;
        return metrics;
    }
    if (reference->w0_px <= 1.0) 
    {
        return metrics;  // degenerate latch, avoid dividing by ~0
    }
    // Occlusion guard: a marker that shrank a lot is partly behind a plate,
    // so its centroid has moved toward the visible part, which fakes strain.
    const bool left_marker_whole = measurement.left_marker.area_px >= config.min_marker_area_ratio * reference->left_marker_area_px;
    const bool right_marker_whole = measurement.right_marker.area_px >= config.min_marker_area_ratio * reference->right_marker_area_px;
    if (!left_marker_whole || !right_marker_whole) 
    {
        return metrics;
    }
    metrics.valid = true;
    // Positive when the gap closes, i.e. the block compresses.
     metrics.eps_vis = (reference->w0_px - measurement.d_px) / reference->w0_px;
    // Positive when the block sinks relative to the fingers (slip under gravity).
    metrics.dy_rel_px = (measurement.y_block_px - measurement.y_fingers_px) - reference->dy0_px;
    return metrics;
}

}  // namespace rt1_data_pipeline::probe