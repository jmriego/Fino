#define DEBUGNO
//#define COMINO
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
unsigned long nextLedMillis;
bool ledStatus;

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

int16_t pos[2] = {0, 0};
int32_t debug[2] = {0, 0};
int lastX;
int lastY;
int lastVelX;
int lastVelY;
int lastAccelX;
int lastAccelY;

EffectParams effects[2];
int32_t forces[2] = {0, 0};

Joystick_ Joystick(
    JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK,
    19, 2, // Button Count, Hat Switch Count
    true, true, false, // X, Y, Z
    false, false, false, // Rx, Ry, Rz
    true, true); // rudder, throttle

void setup() {

    setupJoystick();

    // setup communication
    #ifdef COMINO
    SerialUSB.begin(SERIAL_BAUD);
    #endif
    
    pinMode(LED_BUILTIN, OUTPUT);
    // setup timing and run them as soon as possible
    lastEffectsUpdate = 0;
    nextJoystickMillis = 0;
    nextEffectsMillis = 0;
    nextLedMillis = 0;
    ledStatus = false;
}

void loop(){
    #ifdef COMINO
    get_messages_from_serial();
    #endif

    unsigned long currentMillis;
    currentMillis = millis();
    if (currentMillis >= nextLedMillis) {
        ledStatus = !ledStatus;
        digitalWrite(LED_BUILTIN, ledStatus ? HIGH : LOW);
        nextLedMillis += 1000;
    }
    // do not run more frequently than these many milliseconds
    if (currentMillis >= nextJoystickMillis) {
        updateJoystickPos();
        nextJoystickMillis = currentMillis + 2;

        // we calculate condition forces every 100ms or more frequently if we get position updates
        if (currentMillis >= nextEffectsMillis || pos_updated) {
            updateEffects(true);
            nextEffectsMillis = currentMillis + 500;
            pos_updated = false;
            
            pos[0] = forces[0];
            pos[1] = debug[1];
            debug[0] = 0;
            debug[1] = 0;
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
}
