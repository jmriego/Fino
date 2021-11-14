#define SERIAL_BAUD 115200  // Baudrate

// --------------------------
// Joystick related variables
// --------------------------

#define default_gain 1.0
#define friction_gain 0.25
#define damperSplineNumPoints 6

// comment out this line if you don't want to have a spline configuration for the damper
#define damperSplineGain float damperSplinePoints[2][damperSplineNumPoints] = { \
    {0, 0, 2500, 6000, 10000, 10000}, \
    {0, 0, 250, 4000, 5000, 5000}}
