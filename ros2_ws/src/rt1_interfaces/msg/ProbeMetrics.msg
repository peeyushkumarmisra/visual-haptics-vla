# Visual probe metrics measured in the cam_side tile (rows 0..111 of the stitched canvas).

# Pixel units are tile pixels (224x112 tile). Image rows grow DOWNWARD.

std_msgs/Header header          # copied from the canvas -> same stamp as the image

bool valid                      # markers + block found (and markers not occluded, once latched)
bool reference_latched          # W0 has been latched for this grasp

float64 d_px                    # horizontal distance between fingertip-marker centroids
float64 w0_px                   # d_px at first contact (0 if not latched)
float64 eps_vis                 # visual strain (W0 - d_px) / W0  (0 if not latched)

float64 y_fingers_px            # mean row of the two marker centroids
float64 y_block_px              # row of the block centroid
float64 dy_rel_px               # (y_block - y_fingers) - same at latch; > 0 = block slipping down

float64 left_marker_area_px     # blob areas, for the occlusion check
float64 right_marker_area_px