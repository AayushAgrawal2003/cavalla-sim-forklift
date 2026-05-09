# Output of line_detector. Describes the state of the yellow guide line and
# (optional) perpendicular stop line as observed in a single camera frame.
#
# Frame convention: image-pixel coordinates, origin top-left, +x right, +y down.
# `guide_yaw_rad` is the signed angle of the guide line w.r.t. image-up
# (i.e. the +Y_image-up axis); 0 means a perfectly vertical line, positive
# means the line tilts clockwise in the image.
#
# `guide_offset_px` is the signed horizontal pixel distance between the
# detected guide-line column at the bottom of the ROI and the configured
# `desired_guide_pixel_col` (the column the line should be in when the
# forklift is properly aligned -- this absorbs the camera's x-offset
# from the forklift centerline). Positive = guide is to the right of
# the desired column.
#
# `guide_offset_m` is the same offset projected to metres if the detector
# has a calibrated ground homography; NaN otherwise.
#
# `stop_distance_px` is the y-pixel distance from the bottom of the image
# to the stop line's centroid (positive, larger means farther away). Only
# meaningful when `stop_detected == true`.

std_msgs/Header header

bool guide_detected
float32 guide_offset_px
float32 guide_yaw_rad
float32 guide_offset_m

bool stop_detected
float32 stop_distance_px

# Secondary perpendicular stop tape in the alternate (yellow) colour.
# Used by `back_out` so the back-away phase can have its own stop
# trigger that doesn't fire on the red drop-off stop tape.
bool yellow_stop_detected
float32 yellow_stop_distance_px

int32 image_width
int32 image_height
