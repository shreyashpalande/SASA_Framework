# Reasons the MCU can indicate it was reset for.
uint8 RESET_LOW_POWER=1
uint8 RESET_WINDOW_WATCHDOG=2
uint8 RESET_INDEPENDENT_WATCHDOG=4
uint8 RESET_SOFTWARE_RESET=8
uint8 RESET_POWER_ON_RESET=16
uint8 RESET_PIN_RESET=32
uint8 RESET_BROWN_OUT_RESET=64

# Reasons can be more than one, make a bit field and OR in the reasons.
uint8 reason
