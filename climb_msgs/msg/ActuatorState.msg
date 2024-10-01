# Describes the current status of a set of actuators

# Actuator error status types
uint8 NO_ERROR = 0              # Fully operational (default)
uint8 COMMUNICATION_ERROR = 1   # Not responding
uint8 BAD_COMMAND_ERROR = 2     # Received invalid command
uint8 BAD_MODE_ERROR = 3        # Entered unsupported operating mode
uint8 VOLTAGE_ERROR = 4         # Incorrect input voltage
uint8 TEMPERATURE_ERROR = 5     # Unsafe temperature
uint8 POSITION_SENSOR_ERROR = 6 # Position sensor malfunction (e.g. encoder)
uint8 VELOCITY_SENSOR_ERROR = 7 # Velocity sensor malfunction (e.g. tachometer)
uint8 EFFORT_SENSOR_ERROR = 8   # Effort sensor malfunction (e.g. strain gauge)
uint8 OVERLOAD_ERROR = 9        # Unsafe load
uint8 ELECTRICAL_ERROR = 10     # Electrical shock
uint8 OUTPUT_ERROR = 11         # No output motion detected
uint8 UNDEFINED_ERROR = 255     # Any other error type

std_msgs/Header header          # Header with timestamp (frame_id is not used)
int32[] id                      # The unique id of each actuator
string[] joint                  # The name of each actuator's associated joint
bool[] enabled                  # Whether each actuator can apply torque
float64[] temperature           # The temperature of each actuator in degrees C
float64[] voltage               # The input voltage of each actuator in volts
uint8[] error                   # The error status of each actuator