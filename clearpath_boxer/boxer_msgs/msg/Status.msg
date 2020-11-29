# This message represents BOXER's lower-frequency status updates, which
# are republished on the PC as diagnostic messages. This message originates
# with the LSPCU.

Header header

# Commit of firmware source.
string hardware_id

# Times since MCU power-on and MCU rosserial connection, respectively.
duration mcu_uptime
duration connection_uptime

MCUResetReason reset_reason

# Component temperatures in degC.
float32 pcb_temperature
float32 mcu_temperature
float32[] battery_cell_temperature

# CMU failure status from BMU
bool battery_cmu_failure
# Id of failed battery cells
uint8[] battery_failed_cells
# Balance status
bool battery_balance_status # False: Balance stop, True: Any cell under balance discharge

int8 SONAR_DEVICE_UNKNOWN=-1
int8 SONAR_DEVICE_MB7047=0
int8 SONAR_DEVICE_SRF235=1
int8 sonar_device

# Detected items
int8 CART_ABSENT=0
int8 CART_PRESENT=1
int8 CART_NOT_SUPPORTED=-1
int8 cart

int8 CHARGE_DOCK_ABSENT=0
int8 CHARGE_DOCK_PRESENT=1
int8 CHARGE_DOCK_NOT_SUPPORTED=-1
int8 charge_dock

# Monitoring the estop loop. Changes in these values trigger an immediate
# publish, outside the ordinarily-scheduled 1Hz updates.
# Also if any updates occur
# update boxer_firmware here: http://codesearch/?q=safetyStateSpinOnce&i=nope&files=&repos=boxer_firmware
# and boxer_diagnostics here: http://codesearch/?q=generalDiagnostics&i=nope&files=&repos=boxer_robot
# at a minimum.
uint16 STOP_CLEAR=0
uint16 STOP_ESTOP_PC_COMMS_LOST=1
uint16 STOP_ESTOP_BMU_STATUS=2
uint16 STOP_ESTOP_TRACTION=4
uint16 STOP_ESTOP_RESET_REQUIRED=8
uint16 STOP_ESTOPPED=16
uint16 STOP_PRECHARGING=32
uint16 STOP_CHARGING=64
uint16 STOP_LIDAR=128
uint16 STOP_SONAR=256
uint16 STOP_TEMPERATURE=512
uint16 STOP_CURRENT=1024
uint16 STOP_ESTOP_COMPUTER_COMMANDED=2048
uint16 stop

# Voltage rails, in volts
# Averaged over the message period
float32 measured_battery
float32 measured_24v
float32 measured_12v
float32 measured_5v
float32 measured_appliance

# Current senses available on platform, in amps.
# Averaged over the message period
float32 drive_current
float32 user_current
float32 computer_current
float32 led_array_current
float32 total_current
float32 appliance_current

# Detailed safety loop status for all monitors available to
# the LSPCU to determine the safety loop status of the system.
uint32 SAFETY_LOOP_STATUS_OSSD1_STATUS=1
uint32 SAFETY_LOOP_STATUS_OSSD2_STATUS=2
uint32 SAFETY_LOOP_STATUS_LSPCU_STOP_STATUS=4
uint32 SAFETY_LOOP_STATUS_TDCU_STOP_STATUS=8
uint32 SAFETY_LOOP_STATUS_K1_COIL_MONITOR_STATUS=16
uint32 SAFETY_LOOP_STATUS_K2_COIL_MONITOR_STATUS=32
uint32 SAFETY_LOOP_STATUS_K_COIL_NET_STATUS=64
uint32 SAFETY_LOOP_STATUS_ESTOP_POWER_CHECK_STATUS=128
uint32 SAFETY_LOOP_STATUS_ESTOP_IMPLAUSIBLE_STATUS=256
uint32 SAFETY_LOOP_STATUS_ESTOP_RESET_ENABLED_STATUS=512
uint32 SAFETY_LOOP_STATUS_ESTOP_RESET_BUTTON_STATUS=1024
uint32 SAFETY_LOOP_STATUS_AUX_CHECK_STATUS=2048
uint32 SAFETY_LOOP_STATUS_ESTOP_EXTERNAL_CHANNEL1=4096
uint32 SAFETY_LOOP_STATUS_ESTOP_EXTERNAL_CHANNEL2=8192
uint32 SAFETY_LOOP_STATUS_SAFETY_RELAY_COIL_STATUS_OSSD1=16384
uint32 SAFETY_LOOP_STATUS_SAFETY_RELAY_COIL_STATUS_OSSD2=32768
uint32 SAFETY_LOOP_STATUS_SAFETY_RELAY_GROUND_PATH=65536
uint32 SAFETY_LOOP_STATUS_LSPCU_SAFETY_RELAY_WATCHDOG_STATUS=131072
uint32 SAFETY_LOOP_STATUS_TDCU_SAFETY_RELAY_WATCHDOG_STATUS=262144
uint32 safety_loop_status

# Lidar reset request and command status
bool lidar_reset_requested
bool lidar_reset_in_progress
