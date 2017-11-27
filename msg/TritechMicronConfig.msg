# Tritech Micron configuration parameters.

# Header message.
Header header

# Whether the sonar head is facing upwards.
bool inverted

# Whether the scan is alternated or continuous.
bool prf_alt

# Whether the scan is clockwise.
bool scanright

# Whether the scan intensity is an 8-bit or 4-bit number.
bool prf_AGC

# Initial gain of the sonar pulse. Ranges between 0 and 1.
float64 gain

# Level of Adaptative Gain Control (%) applied to profiler.
float64 filt_gain
# Analogue detect threshold level on the receiver and controls the sensitivity of the Profiler.
float64 adc_threshold

# Right and left limit of scan if in sector scan mode. Values are in radians
# and range from 0 to 2 pi.
float64 left_limit
float64 right_limit

# Range of scan in meters.
float64 range

# Number of bins per scan slice.
int16 nbins

# Motor step size in radians.
float64 step

# Time in microseconds at which Receiver starts sampling after it has transmitted
int16 lockout

# Enable/disable virtual port
bool port_enabled

# Port baudrate
int16 port_baudrate
