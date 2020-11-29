# Message to the MCU to command position of the
# lift motor in meters. Ranges from 0.0 to 0.08.
float32 height

# Command the lift to home in a specific direction and stay there
uint8 HOME_NONE=0
uint8 HOME_DOWN=1
uint8 HOME_UP=2

uint8 home_lift
