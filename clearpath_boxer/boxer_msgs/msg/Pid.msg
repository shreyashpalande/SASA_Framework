# This message is used to update the PID controller constants on BOXER

# Proportional gain. Units are 1/ETU
float32 p
# Integral gain. Units are s/ETU
float32 i
# Derivative gain. Units are 1/ETU*s
float32 d
# Integrator limit. Units are s/ETU
float32 i_limit
# Feed forward gain. Units are 1/ETU
float32 feed_forward
# Command threshold at which we use the full P gain. Below this threshold, we linearly scale the P gain
# down to zero. This is done to allow the P gain to reduce integrator overshoot while allowing for low
# P gains at low speeds to reduce oscillations.
float32 p_cmd_threshold
