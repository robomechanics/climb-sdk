# A set of contact forces between a robot and the world

std_msgs/Header header          # Header with timestamp (frame_id is not used)
string[] frame                  # Contact frame ID

# Contact wrench of world on robot expressed in contact frame
geometry_msgs/Wrench[] wrench