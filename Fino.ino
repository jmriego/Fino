#define NODEBUG
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
unsigned long nextEffectsMillis;

// --------------------------
// Joystick related variables
// --------------------------
#define minX -32768
#define maxX 32767
#define minY -32768
#define maxY 32767

bool is_connected = false;
bool forces_requested = false;
bool pos_updated = false;

int16_t pos[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int16_t hat[4] = {0, 0, 0, 0};
int lastX;
int lastY;
int lastVelX;
int lastVelY;
int lastAccelX;
int lastAccelY;

EffectParams effects[2];
int16_t forces[2] = {0, 0};

Joystick_ Joystick(
    JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK,
    19, 2, // Button Count, Hat Switch Count
    true, true, true, // X, Y, Z
    true, true, true, // Rx, Ry, Rz
    true, true); // slider, dial

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
    nextEffectsMillis = 0;
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

        // we calculate condition forces every 100ms or more frequently if we get position updates
        if (currentMillis >= nextEffectsMillis || pos_updated) {
            updateEffects(true);
            nextEffectsMillis = currentMillis + 100;
            pos_updated = false;
        } else {
            // calculate forces without recalculating condition forces
            // this helps having smoother spring/damper/friction
            // if our update rate matches our input device
            updateEffects(false);
        }

        #ifdef COMINO
        if (forces_requested) {
            sendForces();
            forces_requested = false;
        }
        #endif
    }

    DriveMotors();
}
