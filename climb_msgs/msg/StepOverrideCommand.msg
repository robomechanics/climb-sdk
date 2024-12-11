# Command to adjust an in-progress step

uint8 COMMAND_NONE = 0          # Remain in the current state
uint8 COMMAND_RETRY = 1         # Retry the step from the beginning
uint8 COMMAND_ADVANCE = 2       # Advance to the next state
uint8 COMMAND_STOP = 3          # Pause the step
uint8 COMMAND_RESUME = 4        # Unpause the step

std_msgs/Header header          # Header with timestamp and step contact frame
geometry_msgs/Twist offset      # Change to the foothold pose in the map frame
uint8 command                   # Change the current phase of the step