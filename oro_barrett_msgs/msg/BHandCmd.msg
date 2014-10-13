# All arrays of hand joints are organized as:
#   [f1, f2, f3, spread]

Header header

# Joint commands
float64[4] cmd

# Joint command mode
uint8[4] mode

# Motor command mode
uint8 MODE_IDLE = 0
uint8 MODE_TORQUE = 2
uint8 MODE_PID = 3
uint8 MODE_VELOCITY = 4
uint8 MODE_TRAPEZOIDAL= 5
# Do not change the mode
uint8 MODE_SAME = 6
