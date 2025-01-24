# A transition to a new stance by repositioning one or more end effectors

# Leave the footholds field empty to request a default foothold, body pose, and
# joint overrides

std_msgs/Header header              # Header with timestamp and base frame

string[] frames                     # Contact frame ID(s)
geometry_msgs/Pose[] footholds      # Desired end effector pose(s)
geometry_msgs/Pose body             # Desired body pose
climb_msgs/JointCommand overrides   # Desired joint override