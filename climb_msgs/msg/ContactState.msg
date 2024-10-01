# Describes a set of contact points between a robot and the world

# Contact modes
uint8 FREE = 0                  # Not in contact (default)
uint8 DISENGAGED = 1            # In contact but not fully grasped
uint8 ENGAGED = 2               # Fully grasped

std_msgs/Header header          # Header with timestamp (frame_id is not used)
string[] frame                  # Contact frame ID
uint8[] mode                    # Contact mode

# Contact wrench of world on robot expressed in contact frame
geometry_msgs/Wrench[] wrench