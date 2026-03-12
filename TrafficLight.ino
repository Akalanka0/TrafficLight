/* ============================================================
   SMART PEDESTRIAN TRAFFIC LIGHT — Arduino
   Features:
     - Ultrasonic pedestrian detection (HC-SR04)
     - 5-state Finite State Machine with proper pre-green amber phase
     - 3-sample median filter to prevent false triggers
     - pulseIn timeout to prevent state-machine stalls
     - All LED outputs explicitly set in every state
   ============================================================ */

/* ----------- PIN DEFINITIONS ----------- */
#define TRIG        6
#define ECHO        7

#define CAR_RED     8
#define CAR_YELLOW  9
#define CAR_GREEN   10

#define PED_GREEN   11
#define PED_RED     12

/* ----------- TIMING CONSTANTS (ms) ----------- */
#define VEHICLE_GREEN_TIME    5000   // Minimum green before yielding to pedestrian
#define VEHICLE_YELLOW_TIME   2000   // Green → Red amber duration
#define VEHICLE_RED_TIME      1000   // Safety buffer at red (no pedestrian path)
#define PED_CROSS_TIME        4000   // Pedestrian crossing window
#define VEHICLE_PRE_GREEN_TIME 2000  // Red → Green amber duration

/* ----------- PEDESTRIAN DETECTION ----------- */
#define PED_MIN_CM    5    // Minimum detection distance
#define PED_MAX_CM   20    // Maximum detection distance
#define SENSOR_TIMEOUT 30000  // pulseIn timeout in µs (~5 m max range)
#define NO_OBJECT     999     // Sentinel value when nothing detected

/* ----------- STATE DEFINITIONS ----------- */
//
//  VEHICLE_GREEN  ──(ped detected + timer elapsed)──► VEHICLE_YELLOW
//  VEHICLE_YELLOW ──(timer)──────────────────────────► VEHICLE_RED
//  VEHICLE_RED    ──(ped detected)───────────────────► PEDESTRIAN_GREEN
//  VEHICLE_RED    ──(no ped + safety buffer)─────────► VEHICLE_PRE_GREEN
//  PEDESTRIAN_GREEN ──(timer)────────────────────────► VEHICLE_PRE_GREEN
//  VEHICLE_PRE_GREEN ──(timer)───────────────────────► VEHICLE_GREEN
//
enum State {
    VEHICLE_GREEN,
    VEHICLE_YELLOW,
    VEHICLE_RED,
    PEDESTRIAN_GREEN,
    VEHICLE_PRE_GREEN   // Amber phase before returning to green
};

enum State currentState;
unsigned long stateStartTime = 0;

/* ============================================================
   SETUP
   ============================================================ */
void setup() {
    pinMode(CAR_RED,    OUTPUT);
    pinMode(CAR_YELLOW, OUTPUT);
    pinMode(CAR_GREEN,  OUTPUT);
    pinMode(PED_GREEN,  OUTPUT);
    pinMode(PED_RED,    OUTPUT);
    pinMode(TRIG,       OUTPUT);
    pinMode(ECHO,       INPUT);

    // Start in VEHICLE_GREEN state
    currentState   = VEHICLE_GREEN;
    stateStartTime = millis();

    // Set initial LED outputs explicitly
    digitalWrite(CAR_GREEN,  HIGH);
    digitalWrite(CAR_YELLOW, LOW);
    digitalWrite(CAR_RED,    LOW);
    digitalWrite(PED_GREEN,  LOW);
    digitalWrite(PED_RED,    HIGH);
}

/* ============================================================
   ULTRASONIC SENSOR — 3-sample median filter
   Returns distance in cm, or NO_OBJECT if nothing detected.
   pulseIn timeout prevents blocking the main loop.
   ============================================================ */
int getDistance() {
    int samples[3];

    for (int i = 0; i < 3; i++) {
        // Trigger pulse
        digitalWrite(TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG, LOW);

        // Read echo with timeout
        long duration = pulseIn(ECHO, HIGH, SENSOR_TIMEOUT);

        if (duration == 0) {
            samples[i] = NO_OBJECT;   // Timeout → no object in range
        } else {
            samples[i] = (int)(duration * 0.034 / 2);  // Convert to cm
        }

        delay(10);  // Short gap between pings to avoid cross-talk
    }

    // Sort 3 samples and return median
    for (int i = 0; i < 2; i++) {
        for (int j = i + 1; j < 3; j++) {
            if (samples[i] > samples[j]) {
                int tmp    = samples[i];
                samples[i] = samples[j];
                samples[j] = tmp;
            }
        }
    }
    return samples[1];  // Median value
}

/* ============================================================
   MAIN LOOP — Non-blocking state machine
   ============================================================ */
void loop() {
    unsigned long currentTime    = millis();
    int           distance       = getDistance();
    bool          pedestrianWaiting = (distance >= PED_MIN_CM && distance <= PED_MAX_CM);

    switch (currentState) {

        /* --------------------------------------------------
           VEHICLE GREEN — cars may pass
           Waits for minimum green time, then yields if
           a pedestrian is detected.
        -------------------------------------------------- */
        case VEHICLE_GREEN:
            digitalWrite(CAR_GREEN,  HIGH);
            digitalWrite(CAR_YELLOW, LOW);
            digitalWrite(CAR_RED,    LOW);
            digitalWrite(PED_GREEN,  LOW);
            digitalWrite(PED_RED,    HIGH);

            if (pedestrianWaiting &&
                (currentTime - stateStartTime >= VEHICLE_GREEN_TIME)) {
                currentState   = VEHICLE_YELLOW;
                stateStartTime = currentTime;
            }
            break;

        /* --------------------------------------------------
           VEHICLE YELLOW — cars prepare to stop
        -------------------------------------------------- */
        case VEHICLE_YELLOW:
            digitalWrite(CAR_GREEN,  LOW);
            digitalWrite(CAR_YELLOW, HIGH);
            digitalWrite(CAR_RED,    LOW);
            digitalWrite(PED_GREEN,  LOW);
            digitalWrite(PED_RED,    HIGH);

            if (currentTime - stateStartTime >= VEHICLE_YELLOW_TIME) {
                currentState   = VEHICLE_RED;
                stateStartTime = currentTime;
            }
            break;

        /* --------------------------------------------------
           VEHICLE RED — cars stopped
           Two paths:
             a) Pedestrian present → PEDESTRIAN_GREEN
             b) No pedestrian after safety buffer → VEHICLE_PRE_GREEN
        -------------------------------------------------- */
        case VEHICLE_RED:
            digitalWrite(CAR_GREEN,  LOW);
            digitalWrite(CAR_YELLOW, LOW);
            digitalWrite(CAR_RED,    HIGH);
            digitalWrite(PED_GREEN,  LOW);
            digitalWrite(PED_RED,    HIGH);

            if (pedestrianWaiting) {
                currentState   = PEDESTRIAN_GREEN;
                stateStartTime = currentTime;
            } else if (currentTime - stateStartTime >= VEHICLE_RED_TIME) {
                currentState   = VEHICLE_PRE_GREEN;  // ← FIX: was VEHICLE_YELLOW (caused infinite loop)
                stateStartTime = currentTime;
            }
            break;

        /* --------------------------------------------------
           PEDESTRIAN GREEN — pedestrians may cross
           Cars remain red throughout.
        -------------------------------------------------- */
        case PEDESTRIAN_GREEN:
            digitalWrite(CAR_GREEN,  LOW);
            digitalWrite(CAR_YELLOW, LOW);
            digitalWrite(CAR_RED,    HIGH);
            digitalWrite(PED_GREEN,  HIGH);
            digitalWrite(PED_RED,    LOW);

            if (currentTime - stateStartTime >= PED_CROSS_TIME) {
                // End pedestrian phase — transition to pre-green amber
                digitalWrite(PED_GREEN, LOW);
                digitalWrite(PED_RED,   HIGH);

                currentState   = VEHICLE_PRE_GREEN;  // ← FIX: was VEHICLE_YELLOW (caused infinite loop)
                stateStartTime = currentTime;
            }
            break;

        /* --------------------------------------------------
           VEHICLE PRE-GREEN — amber warning before green
           Signals drivers that green is about to resume.
           (Red + Yellow simultaneously = international
            "prepare to go" convention)
        -------------------------------------------------- */
        case VEHICLE_PRE_GREEN:
            digitalWrite(CAR_GREEN,  LOW);
            digitalWrite(CAR_YELLOW, HIGH);
            digitalWrite(CAR_RED,    HIGH);   // Red+Yellow = prepare to go
            digitalWrite(PED_GREEN,  LOW);
            digitalWrite(PED_RED,    HIGH);

            if (currentTime - stateStartTime >= VEHICLE_PRE_GREEN_TIME) {
                currentState   = VEHICLE_GREEN;
                stateStartTime = currentTime;
            }
            break;
    }
}
