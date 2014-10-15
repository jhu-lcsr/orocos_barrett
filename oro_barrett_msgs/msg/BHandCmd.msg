# All arrays of hand joints are organized as:
#   [f1, f2, f3, spread]

Header header

# Joint commands
float64[4] cmd

# Joint command mode
int8[4] mode

# Motor command mode
int8 MODE_IDLE = 0
int8 MODE_TORQUE = 2
int8 MODE_PID = 3
int8 MODE_VELOCITY = 4
int8 MODE_TRAPEZOIDAL= 5
# Do not change the mode
int8 MODE_SAME = -1
