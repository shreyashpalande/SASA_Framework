# Represents high-frequency feedback from one of BOXER's three drive
# units (left, right, lift).

Header header

# Current flowing from battery into the driver.
float32 current

# Instantaneous duty cycle of MOSFET bridge.
float32 duty_cycle

# Set Point commanded by the controller
float32 set_point

# Current operating mode
int8 MODE_NONE=-1
int8 MODE_VELOCITY=0
int8 MODE_PWM=1
int8 MODE_EFFORT=2
int8 MODE_POSITION=3
int8 mode

# Temperatures measured in the MOSFET bridge and on the motor casing, in deg C.
float32 bridge_temperature
float32 motor_temperature

# Voltage rails, in volts
float32 bridge_voltage

# Encoder data.
float32 measured_velocity  # rad/s or m/s
float32 measured_pose      # rad or m

# Motor fault states
uint8 FAULT_DRIVER=1  # underlying driver chip reports a fault condition.
uint8 FAULT_HOME_TIMEOUT=2  # limit switch did not trigger within expected timeframe
uint8 FAULT_CURRENT=4  # Motor overcurrent
uint8 FAULT_OVERHEAT=8  # Motor and/or Bridge overheat
uint8 fault

# True if the underlying driver chip is held in reset.
bool driver_in_reset

# True if precharge is complete on this motor drive channel.
bool precharge_complete

# For position feedback, determines if homed.
bool homing_complete

# Limit switch status, for the lift only.
uint8 LIMIT_NONE=0
uint8 LIMIT_UPPER=1
uint8 LIMIT_LOWER=2
uint8 limit

# Motor controller PID constants
Pid pid
