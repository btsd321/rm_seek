std_msgs/Header header
Point2d box_center  # center of the box, not the point where the diagonals intersect 
float64 box_width
float64 box_height
float64 confidence
# Point2d diagonal_intersection  # the point where the diagonals intersect 
Point2d[4] target_points
