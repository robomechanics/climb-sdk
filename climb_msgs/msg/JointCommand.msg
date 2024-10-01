# Describes a set of desired joint setpoints

# Each joint can be operated in one of 3 modes.
#
# Position mode:
#   Move to commanded position while respecting max velocity and max effort
# Velocity mode:
#   Move at commanded velocity while respecting max effort (ignore position)
# Torque mode:
#   Apply commanded effort (ignore position/velocity)
#
# Max velocity/effort is a limit on the absolute value of velocity/effort.

uint8 POSITION_MODE = 0     # Position mode
uint8 VELOCITY_MODE = 1     # Velocity mode
uint8 EFFORT_MODE = 2       # Effort mode

std_msgs/Header header      # Header with timestamp (frame_id is not used)
string[] name               # The name of each joint
uint8[] mode                # Commanded operating mode of the joint
float64[] position          # Commanded position in rad or m
float64[] velocity          # Commanded or max velocity in rad/s or m/s
float64[] effort            # Commanded or max effort in Nm or N