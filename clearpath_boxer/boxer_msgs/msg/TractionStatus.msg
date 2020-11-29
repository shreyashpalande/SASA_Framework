# This message represents BOXER's lower-frequency status updates, which
# are republished on the PC as diagnostic messages. This message originates
# with the TDCU.

Header header

# Commit of firmware source.
string hardware_id

# Wheel-Differential Cap Table checksum
uint32 cap_table_checksum

# Times since MCU power-on and MCU rosserial connection, respectively.
duration mcu_uptime
duration connection_uptime

MCUResetReason reset_reason

# Component temperatures in degC.
float32 pcb_temperature
float32 mcu_temperature

# Monitoring the system state. Changes in these values trigger an immediate
# publish, outside the ordinarily-scheduled 1Hz updates.
uint16 STOP_CLEAR=0
uint16 STOP_ESTOP_PC_COMMS_LOST=1
uint16 STOP_DRIVER_FAULT=2
uint16 STOP_OVER_TEMPERATURE=4
uint16 STOP_OVER_CURRENT=8
uint16 STOP_PRECHARGING=16
uint16 STOP_COMMANDS_STALE=32
uint16 STOP_CAP_TABLE_VIOLATION=64
uint16 stop

# Voltage rails, in volts
# Averaged over the message period
float32 measured_battery
float32 measured_switched_battery
float32 measured_24v
float32 measured_12v
float32 measured_5v
