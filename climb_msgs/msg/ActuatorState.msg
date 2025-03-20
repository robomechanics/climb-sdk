# The current status of a set of actuators

# Actuator error status types
uint8 ERROR_NONE = 0            # Fully operational (default)
uint8 ERROR_COMMUNICATION = 1   # Not responding
uint8 ERROR_BAD_COMMAND = 2     # Received invalid command
uint8 ERROR_BAD_MODE = 3        # Entered unsupported operating mode
uint8 ERROR_VOLTAGE = 4         # Incorrect input voltage
uint8 ERROR_TEMPERATURE = 5     # Unsafe temperature
uint8 ERROR_POSITION_SENSOR = 6 # Position sensor malfunction (e.g. encoder)
uint8 ERROR_VELOCITY_SENSOR = 7 # Velocity sensor malfunction (e.g. tachometer)
uint8 ERROR_EFFORT_SENSOR = 8   # Effort sensor malfunction (e.g. strain gauge)
uint8 ERROR_OVERLOAD = 9        # Unsafe load
uint8 ERROR_ELECTRICAL = 10     # Electrical shock
uint8 ERROR_OUTPUT = 11         # No output motion detected
uint8 ERROR_UNDEFINED = 255     # Any other error type

std_msgs/Header header          # Header with timestamp (frame_id is not used)
int32[] id                      # The unique id of each actuator
string[] joint                  # The name of each actuator's associated joint
bool[] enabled                  # Whether each actuator can apply torque
float64[] temperature           # The temperature of each actuator in degrees C
float64[] voltage               # The input voltage of each actuator in volts
uint8[] error                   # The error status of each actuator