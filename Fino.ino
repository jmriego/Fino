#define NO_DEBUG
#define COMINO
#ifdef _VARIANT_ARDUINO_DUE_X_
#define Serial SerialUSB
#endif

// the digits mean Mmmmrrr (M=Major,m=minor,r=revision)
#define SKETCH_VERSION 3000001

#include "src/Joystick.h"
#include "config.h"
#include "order.h"

// -------------------------
// Various global variables
// -------------------------
unsigned long lastEffectsUpdate;
unsigned long nextJoystickMillis;

// --------------------------
// Joystick related variables
// --------------------------
#define minX -32768
#define maxX 32767
#define minY -32768
#define maxY 32767

bool is_connected = false;
bool forces_requested = false;

int16_t pos[2] = {0, 0};
int lastX;
int lastY;
int lastVelX;
int lastVelY;
int lastAccelX;
int lastAccelY;

EffectParams effects[2];
int32_t forces[2] = {0, 0};

double Kp=0.2, Ki=0.0, Kd=0;
Joystick_ Joystick(
    JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK,
    19, 2, // Button Count, Hat Switch Count
    true, true, false, // X, Y, Z
    false, false, false, // Rx, Ry, Rz
    true, true); // rudder, throttle

void setup() {

    ArduinoSetup();
    setupJoystick();

    // setup communication
    #if defined(COMINO) || defined(DEBUG)
    Serial.begin(SERIAL_BAUD);
    #endif

    // setup timing and run them as soon as possible
    lastEffectsUpdate = 0;
    nextJoystickMillis = 0;
}

void loop(){
    ReadPots();

    #ifdef COMINO
    get_messages_from_serial();
    #endif

    unsigned long currentMillis;
    currentMillis = millis();
    // do not run more frequently than these many milliseconds
    if (currentMillis >= nextJoystickMillis) {
        updateJoystickPos();
        nextJoystickMillis = currentMillis + 2;

        updateEffects(true);
        #ifdef COMINO
        if (forces_requested) {
            sendForces();
            forces_requested = false;
        }
        #endif
    }

    DriveMotors();
}
