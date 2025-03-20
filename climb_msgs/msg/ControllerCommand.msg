# A desired behavior for each end-effector of a robot

# Each end effector can be operated in one of 3 modes.
#
# Free mode (default):
#   Move with commanded velocity (projected into twist basis)
# Transition mode:
#   Apply commanded contact force (projected into wrench basis)
# Stance mode:
#   Optimize contact force to maximize stability margin
#
# Vectors must either be empty or the same length as the frame vector.
# Empty vectors are treated as all zeros if used.

# Contact modes
uint8 MODE_FREE = 0             # Not in contact: follow commanded twist
uint8 MODE_TRANSITION = 1       # Changing contact mode: apply commanded wrench
uint8 MODE_CONTACT = 2          # Contacting surface: obey guard constraints
uint8 MODE_STANCE = 3           # Adhered to surface: maximize stability margin

std_msgs/Header header          # Header with timestamp and body frame ID
string[] frame                  # Contact frame ID
uint8[] mode                    # Contact mode
geometry_msgs/Pose body         # Desired body pose
JointCommand overrides          # Joint overrides

# Velocity of free end-effector in contact frame (rad/s or m/s)
geometry_msgs/Twist[] twist

# Contact force on transitioning end-effector in contact frame (N or Nm)
geometry_msgs/Wrench[] wrench