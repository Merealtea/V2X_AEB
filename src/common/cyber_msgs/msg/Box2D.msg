# 2D box in image
std_msgs/Header header

int16 center_x    # dim 0, 0 < center_x < img_height
int16 center_y    # dim 1, 0 < center_y < img_width
int16 height
int16 width
float32 score