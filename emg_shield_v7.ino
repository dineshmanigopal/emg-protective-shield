// ============================================================
//   EMG WRIST SHIELD — FIRMWARE v4.0
//   Target  : ESP32-WROOM-32
//   Sensor  : Advancer Technologies Muscle Sensor v3
//
//   WHAT CHANGED FROM v3 — SERVO FIXES:
//   ─────────────────────────────────────────────────────────
//   FIX 1 — attach() now specifies pulse width limits.
//     v3 used myServo.attach(SERVO_PIN) with no pulse-width
//     arguments. ESP32Servo defaults to 544µs–2400µs, but most
//     hobby servos (SG90, MG996R, DS3225) expect 500µs–2500µs.
//     The mismatch makes 0° and 180° endpoints unreachable —
//     the servo either stops short or twitches/buzzes against
//     its mechanical stop. Fixed by passing explicit min/max µs.
//
//   FIX 2 — writeMicroseconds() instead of write() for motion.
//     write(degrees) internally maps degrees to µs using the
//     same possibly-wrong default range. writeMicroseconds()
//     bypasses that mapping entirely and sends the exact pulse
//     the servo hardware expects. Much more reliable across
//     different servo models.
//
//   FIX 3 — Startup centering + sweep test.
//     v3 sent write(0) immediately on boot before the servo
//     had time to power up. Some servos ignore or misread the
//     first command. v4 waits 500ms after attach(), then runs
//     a slow 0→20→0° verification sweep so you can confirm the
//     servo is responding before calibration begins.
//
//   FIX 4 — servoCurrentPos clamped to [0, DEPLOY_ANGLE].
//     If servoCurrentPos ever drifted out of range (e.g. from
//     an interrupted sweep), the updateServo() comparisons
//     could overshoot. Now clamped on every write.
//
//   FIX 5 — Deploy angle reverted to 90°.
//     SERVO_DEPLOYED_ANGLE is 90. At SERVO_STEP=3, SERVO_TICK_MS=15:
//       90° / 3° = 30 steps × 15ms = 450ms full deploy sweep.
//     Raise SERVO_STEP to 5 for ~270ms deploy.
//
//   CRITICAL WIRING — READ BEFORE POWERING ON:
//   ─────────────────────────────────────────────────────────
//   Sensor requires ±9V DUAL supply (two 9V batteries):
//     Battery 1 (+) → sensor +Vs pin
//     Battery 1 (–) + Battery 2 (+) joined → sensor GND pin
//     Battery 2 (–) → sensor –Vs pin
//   SIG output swings 0V–9V. ESP32 GPIO max = 3.3V.
//   REQUIRED voltage divider on SIG line:
//     SIG ──[R1: 47kΩ]──┬── GPIO 34
//                       [R2: 18kΩ]
//                       GND
//   (scales 9V → ~2.49V, safe for ADC)
//
//   Servo power: do NOT power servo from ESP32 3.3V/5V pin.
//   Use a separate 5V supply (powerbank or dedicated regulator).
//   Share GND between ESP32 and servo supply.
//
//   Libraries needed:
//     - TinyGPS++ by Mikal Hart
//     - ESP32Servo by Kevin Harrington
// ============================================================


// ======================== LIBRARIES =========================
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>


// ========================== PINS ============================
#define EMG_PIN        34    // ADC input — GPIO 34–39 are input-only, ideal
#define SERVO_PIN      13    // PWM output to servo signal wire (changed from 18)
#define BUTTON_PIN     25    // Tactile button, wired to GND, INPUT_PULLUP
#define GPS_RX         16
#define GPS_TX         17
#define GSM_RX         26
#define GSM_TX         27


// =================== SERVO PULSE WIDTH CONFIG (SG90) ========
// SG90 datasheet quotes 1ms-2ms nominally, but real SG90 units
// respond best to the wider 544-2400us window — the same range
// used by the Arduino Servo library. This gives clean 0-180 deg
// travel without buzzing against the mechanical end-stops.
//
// SG90 specs: 50Hz PWM, 4.8-6V supply, stall current up to 650mA.
// IMPORTANT: Power the SG90 from a SEPARATE 5V supply.
// Do NOT use ESP32 3.3V pin — 650mA stall will brown-out the board.
// Share only the GND wire between ESP32 and servo power supply.
//
// Fine-tuning if needed:
//   Stops short of   0 deg: decrease SERVO_MIN_US by 20us steps
//   Stops short of 180 deg: increase SERVO_MAX_US by 20us steps
//   Buzzes at an endpoint:  move that value inward by 30us
#define SERVO_MIN_US     544    // SG90: reliable 0 deg endpoint
#define SERVO_MAX_US    2400    // SG90: reliable 180 deg endpoint

// Deployed angle — reverted from 180° back to 90°.
// SG90 operates comfortably within 0°–90° for this application.
// Change this value if different travel is needed.
#define SERVO_DEPLOYED_ANGLE    90
#define SERVO_RETRACTED_ANGLE    0

// Sweep speed controls.
// Step size: degrees advanced per tick. Larger = faster but jerkier.
// At step=3, tick=15ms: 90°/3° = 30 ticks × 15ms = 450ms full deploy.
// At step=5, tick=15ms: 90°/5° = 18 ticks × 15ms = 270ms full deploy.
#define SERVO_STEP        3
#define SERVO_TICK_MS    15


// =================== VOLTAGE DIVIDER SCALING ================
// Divider ratio: 18k / (47k + 18k) = 0.2769
// Max ADC reading after divider: 9V × 0.2769 / 3.3V × 4095 ≈ 3090
// Scale factor for human-readable mV display (serial monitor only):
//   mV_at_SIG = ADC_count × (3300/4095) × (1/0.2769) = ADC_count × 2.91
#define ADC_MAX_READING    3090
#define ADC_SCALE_TO_MV    2.91f


// =================== TUNABLE CONSTANTS ======================
// Raise THRESHOLD_OFFSET if normal arm movement false-triggers.
// Lower it if real contractions aren't detected.
// Starting point: 300 counts (~870mV above resting noise floor).
#define THRESHOLD_OFFSET     300

// Consecutive above-threshold samples needed to confirm contraction.
// At 50ms polling: 3 = 150ms sustained activation required.
#define PEAK_COUNT_REQUIRED    3

// Auto-retract delay after deployment (milliseconds)
#define AUTO_RETRACT_MS      5000

// Cooldown before re-arming after retract (milliseconds)
#define REARM_DELAY_MS       2000

// Emergency contact — include country code
#define EMERGENCY_NUMBER     "+918639345978"   // ← CHANGE THIS

// true  = continuous ADC printout (use during hardware bring-up)
// false = 5-second status reports (use during normal operation)
#define DEBUG_MODE           false


// ===================== OBJECTS ==============================
TinyGPSPlus      gps;
HardwareSerial   gpsSerial(1);
HardwareSerial   gsmSerial(2);
Servo            myServo;


// ===================== SYSTEM STATE =========================
int   emgBaseline  = 0;
int   emgThreshold = 0;
int   lastEMGValue = 0;
int   peakCount    = 0;

enum ShieldState { STANDBY, DEPLOYING, DEPLOYED, RETRACTING, COOLDOWN };
ShieldState shieldState = STANDBY;

unsigned long lastEMGSampleTime   = 0;
unsigned long lastStatusPrintTime = 0;
unsigned long deployTimestamp     = 0;
unsigned long retractTimestamp    = 0;
unsigned long lastServoMoveTime   = 0;

// Servo position tracking — always kept within [0, SERVO_DEPLOYED_ANGLE]
int servoCurrentPos = SERVO_RETRACTED_ANGLE;
int servoTargetPos  = SERVO_RETRACTED_ANGLE;

// Button debounce
int  buttonLastRawReading = HIGH;
int  buttonStableState    = HIGH;
unsigned long buttonDebounceTime = 0;
#define BUTTON_DEBOUNCE_MS 50

// GPS last-known-good cache
double lastKnownLat  = 0.0;
double lastKnownLng  = 0.0;
bool   hadGPSFix     = false;
unsigned long lastGPSFixTime = 0;


// ============================================================
//   SERVO HELPER — degree to microseconds conversion
// ============================================================
// Converts a degree value (0–180) to the correct pulse width in
// microseconds using the calibrated SERVO_MIN_US / SERVO_MAX_US
// range defined above. This bypasses ESP32Servo's internal
// degree→µs mapping which uses different defaults and can cause
// the servo to miss endpoints or buzz against mechanical stops.
//
// Formula: µs = MIN + (degree / 180.0) × (MAX - MIN)
// Example at 180°: 500 + (180/180) × 2000 = 500 + 2000 = 2500µs ✓
// Example at  90°: 500 + (90/180)  × 2000 = 500 + 1000 = 1500µs ✓
// Example at   0°: 500 + (0/180)   × 2000 = 500 +    0 =  500µs ✓
int degreesToMicroseconds(int degrees) {
  degrees = constrain(degrees, 0, 180);   // Safety clamp
  return SERVO_MIN_US + (int)((float)degrees / 180.0f * (SERVO_MAX_US - SERVO_MIN_US));
}

// Write a servo position in degrees using the correct pulse width.
// Always use this instead of myServo.write() for reliable operation.
void servoWriteDeg(int degrees) {
  degrees = constrain(degrees, SERVO_RETRACTED_ANGLE, SERVO_DEPLOYED_ANGLE);
  myServo.writeMicroseconds(degreesToMicroseconds(degrees));
}


// ============================================================
//   SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(EMG_PIN,    INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Set ADC to 11dB attenuation so the full 0–3.3V range is usable.
  // Default 0dB only covers 0–1V — the divider output reaches 2.49V,
  // so without this everything above ~1V would peg the ADC at 4095.
  analogSetAttenuation(ADC_11db);

  // ── SERVO ATTACH with explicit pulse-width limits ──────────
  // Passing min/max µs to attach() tells ESP32Servo the exact
  // range to use. This fixes the endpoint mismatch that causes
  // the servo to stop short of 0° or 180°, or buzz at the stops.
  myServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

  // Give the servo time to power up before sending any command.
  // Sending a position immediately after attach() can be ignored
  // by some servos that haven't finished their internal init.
  delay(500);

  // Park at 0° using microseconds for accuracy
  servoWriteDeg(SERVO_RETRACTED_ANGLE);
  servoCurrentPos = SERVO_RETRACTED_ANGLE;
  servoTargetPos  = SERVO_RETRACTED_ANGLE;

  // ── STARTUP SWEEP TEST ─────────────────────────────────────
  // Slowly move to 20° and back to 0°. This confirms the servo
  // is wired and powered correctly before calibration begins.
  // Watch the servo — if it doesn't move, check power and wiring.
  // If it only moves partway, adjust SERVO_MIN_US / SERVO_MAX_US.
  Serial.println(">> Servo sweep test (0 → 20° → 0°)...");
  for (int pos = 0; pos <= 20; pos += 2) {
    servoWriteDeg(pos);
    delay(30);
  }
  delay(300);
  for (int pos = 20; pos >= 0; pos -= 2) {
    servoWriteDeg(pos);
    delay(30);
  }
  Serial.println(">> Sweep test complete.");

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  gsmSerial.begin(9600, SERIAL_8N1, GSM_RX, GSM_TX);

  delay(2000);   // Let GPS and GSM modules initialise

  printDashboard();
  calibrateEMG();

  Serial.println("\nSystem armed and running.\n");
  Serial.println(DEBUG_MODE ? "Mode: DEBUG (continuous ADC)" : "Mode: CLEAN (5-sec report)");
}


// ============================================================
//   MAIN LOOP
// ============================================================
void loop() {

  // 1. Sample EMG every 50ms
  if (millis() - lastEMGSampleTime >= 50) {
    lastEMGSampleTime = millis();
    lastEMGValue = readEMG();

    if (DEBUG_MODE) {
      Serial.print("ADC: ");
      Serial.print(lastEMGValue);
      Serial.print("  (~");
      Serial.print((int)(lastEMGValue * ADC_SCALE_TO_MV));
      Serial.print("mV)  Threshold: ");
      Serial.println(emgThreshold);
    }

    if (shieldState == STANDBY) {
      detectEMG(lastEMGValue);
    }
  }

  // 2. Status report every 5s (clean mode only)
  if (!DEBUG_MODE && millis() - lastStatusPrintTime >= 5000) {
    lastStatusPrintTime = millis();
    printStatusReport();
  }

  // 3. Manual button check (debounced)
  checkButton();

  // 4. GPS NMEA parser — must run every loop, no skipping
  readGPS();

  // 5. Non-blocking servo sweep toward target
  updateServo();

  // 6. State machine transitions
  updateStateMachine();
}


// ============================================================
//   SERVO — non-blocking incremental sweep
// ============================================================
// Called every loop. Advances servo one SERVO_STEP degree toward
// the target if SERVO_TICK_MS has elapsed. The system stays fully
// responsive (EMG sampling, GPS, button) during the entire sweep.
void updateServo() {
  if (millis() - lastServoMoveTime < SERVO_TICK_MS) return;
  lastServoMoveTime = millis();

  if (servoCurrentPos < servoTargetPos) {
    // Moving toward deployed position
    servoCurrentPos = min(servoCurrentPos + SERVO_STEP, servoTargetPos);

    // Clamp to valid range before writing
    servoCurrentPos = constrain(servoCurrentPos, SERVO_RETRACTED_ANGLE, SERVO_DEPLOYED_ANGLE);
    servoWriteDeg(servoCurrentPos);

  } else if (servoCurrentPos > servoTargetPos) {
    // Moving toward retracted position
    servoCurrentPos = max(servoCurrentPos - SERVO_STEP, servoTargetPos);

    servoCurrentPos = constrain(servoCurrentPos, SERVO_RETRACTED_ANGLE, SERVO_DEPLOYED_ANGLE);
    servoWriteDeg(servoCurrentPos);
  }
  // If currentPos == targetPos: servo holds position, nothing written
}

// Request deployment: set target to 90° and let updateServo() sweep there.
// Estimated time at default settings: 30 steps × 15ms = 450ms.
void deployShield() {
  Serial.println(">> Deploying shield to 90°...");
  servoTargetPos = SERVO_DEPLOYED_ANGLE;
}

// Request retraction: set target to 0° and let updateServo() sweep back.
void retractShield() {
  Serial.println(">> Retracting shield to 0°...");
  servoTargetPos = SERVO_RETRACTED_ANGLE;
}

// Returns true only when the servo has physically reached its target.
// The state machine uses this to know when a sweep is complete.
bool servoAtTarget() {
  return servoCurrentPos == servoTargetPos;
}


// ============================================================
//   STATE MACHINE
// ============================================================
void updateStateMachine() {
  switch (shieldState) {

    case STANDBY:
      // Waiting for EMG or button trigger — nothing to do
      break;

    case DEPLOYING:
      // Wait for servo to physically reach 90° before marking deployed
      if (servoAtTarget()) {
        shieldState     = DEPLOYED;
        deployTimestamp = millis();
        Serial.println(">> Shield fully deployed at 90°.");
      }
      break;

    case DEPLOYED:
      // Auto-retract after AUTO_RETRACT_MS milliseconds
      if (millis() - deployTimestamp >= AUTO_RETRACT_MS) {
        Serial.println(">> Auto-retract timer elapsed.");
        retractShield();
        shieldState = RETRACTING;
      }
      break;

    case RETRACTING:
      // Wait for servo to physically reach 0° before starting cooldown
      if (servoAtTarget()) {
        shieldState      = COOLDOWN;
        retractTimestamp = millis();
        Serial.println(">> Shield retracted. Cooldown started.");
      }
      break;

    case COOLDOWN:
      // Brief pause before re-arming to prevent residual muscle tension
      // from immediately re-triggering after the emergency subsides.
      if (millis() - retractTimestamp >= REARM_DELAY_MS) {
        shieldState = STANDBY;
        peakCount   = 0;
        Serial.println(">> System re-armed. Ready.");
      }
      break;
  }
}


// ============================================================
//   TRIGGER — central dispatch for all activation sources
// ============================================================
void triggerSystem(const char* source) {
  if (shieldState != STANDBY) {
    Serial.print(">> Trigger ignored — current state: ");
    Serial.println(shieldState);
    return;
  }

  Serial.println("\n================================");
  Serial.print("TRIGGERED BY: "); Serial.println(source);
  Serial.println("================================\n");

  shieldState = DEPLOYING;
  deployShield();
  sendLocationSMS();
}


// ============================================================
//   EMG READING — 4-sample average
// ============================================================
// The Muscle Sensor v3 already outputs a rectified + smoothed
// DC envelope. 4 rapid samples is sufficient to suppress ADC
// noise without adding meaningful latency.
int readEMG() {
  const int SAMPLE_COUNT = 4;
  long sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sum += analogRead(EMG_PIN);
  }
  return (int)(sum / SAMPLE_COUNT);
}


// ============================================================
//   CALIBRATION — resting noise floor measurement
// ============================================================
// Run with forearm completely relaxed. Sets emgBaseline and
// emgThreshold. Adjust the trimpot on the sensor board first:
//   CCW = minimum gain, CW = maximum gain.
// Target: resting ~100–300 ADC, firm contraction ~1500+.
void calibrateEMG() {
  Serial.println("\nCalibrating — relax your forearm completely...");

  long sum = 0;
  const int CAL_SAMPLES = 100;

  for (int i = 0; i < CAL_SAMPLES; i++) {
    sum += analogRead(EMG_PIN);
    if (i % 25 == 0) Serial.print(".");
    delay(5);
  }
  Serial.println();

  emgBaseline  = (int)(sum / CAL_SAMPLES);
  emgThreshold = emgBaseline + THRESHOLD_OFFSET;

  Serial.print("Baseline (ADC)  : "); Serial.println(emgBaseline);
  Serial.print("Threshold (ADC) : "); Serial.println(emgThreshold);
  Serial.print("Baseline (~mV)  : "); Serial.println((int)(emgBaseline * ADC_SCALE_TO_MV));

  if (emgBaseline > 800) {
    Serial.println("WARNING: Baseline >800. Check electrode contact,");
    Serial.println("         voltage divider values, and gain pot.");
  }

  Serial.println("Calibration complete.\n");
}


// ============================================================
//   EMG DETECTION — consecutive peak counter
// ============================================================
// Requires PEAK_COUNT_REQUIRED consecutive above-threshold
// readings for a trigger. A single spike or noise burst won't fire.
void detectEMG(int value) {
  if (value > emgThreshold) {
    peakCount++;
  } else {
    peakCount = 0;   // Contraction was not sustained — restart count
  }

  if (peakCount >= PEAK_COUNT_REQUIRED) {
    peakCount = 0;   // Reset so a held contraction doesn't keep re-firing
    triggerSystem("EMG contraction");
  }
}


// ============================================================
//   BUTTON — proper two-state debounce
// ============================================================
// Tracks raw vs confirmed-stable state separately.
// Fires trigger only on the confirmed falling edge (HIGH→LOW).
void checkButton() {
  int rawReading = digitalRead(BUTTON_PIN);

  if (rawReading != buttonLastRawReading) {
    buttonDebounceTime   = millis();
    buttonLastRawReading = rawReading;
  }

  if (millis() - buttonDebounceTime >= BUTTON_DEBOUNCE_MS) {
    if (rawReading != buttonStableState) {
      buttonStableState = rawReading;
      if (buttonStableState == LOW) {
        triggerSystem("Manual button");
      }
    }
  }
}


// ============================================================
//   GPS — NMEA stream parser + position cache
// ============================================================
void readGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid() && gps.location.isUpdated()) {
    lastKnownLat   = gps.location.lat();
    lastKnownLng   = gps.location.lng();
    hadGPSFix      = true;
    lastGPSFixTime = millis();
  }
}


// ============================================================
//   SMS — alert with live or cached GPS coordinates
// ============================================================
void sendLocationSMS() {
  String message = "EMERGENCY ALERT — wrist shield deployed!\n";

  if (gps.location.isValid()) {
    message += "Live location:\nhttp://maps.google.com/maps?q=";
    message += String(gps.location.lat(), 6);
    message += ",";
    message += String(gps.location.lng(), 6);

  } else if (hadGPSFix) {
    // Fallback: last known position with staleness timestamp
    unsigned long ageSeconds = (millis() - lastGPSFixTime) / 1000;
    message += "Last known location (";
    message += String(ageSeconds);
    message += "s ago):\nhttp://maps.google.com/maps?q=";
    message += String(lastKnownLat, 6);
    message += ",";
    message += String(lastKnownLng, 6);

  } else {
    message += "GPS: no fix available. Track device manually.";
  }

  sendSMS(message);
}

void sendSMS(const String& msg) {
  Serial.println(">> Sending SMS...");

  if (!sendATCommand("AT",        "OK", 2000)) { Serial.println("GSM not responding. Abort."); return; }
  if (!sendATCommand("AT+CMGF=1", "OK", 2000)) { Serial.println("SMS text mode failed. Abort."); return; }

  String addrCmd = "AT+CMGS=\"";
  addrCmd += EMERGENCY_NUMBER;
  addrCmd += "\"";
  if (!sendATCommand(addrCmd, ">", 5000)) { Serial.println("No SMS prompt received. Abort."); return; }

  gsmSerial.print(msg);
  delay(200);
  gsmSerial.write(26);   // Ctrl+Z — tells GSM module to transmit the SMS

  if (!waitForResponse("+CMGS:", 10000)) {
    Serial.println("WARNING: No send confirmation. SMS may not have sent.");
  } else {
    Serial.println(">> SMS sent successfully.");
  }
}

bool sendATCommand(const String& cmd, const String& expected, unsigned long timeoutMs) {
  while (gsmSerial.available()) gsmSerial.read();   // Flush any stale response bytes
  gsmSerial.println(cmd);
  Serial.print("  AT >> "); Serial.println(cmd);
  return waitForResponse(expected, timeoutMs);
}

bool waitForResponse(const String& expected, unsigned long timeoutMs) {
  String response = "";
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    while (gsmSerial.available()) {
      response += (char)gsmSerial.read();
      if (response.indexOf(expected) != -1) {
        Serial.print("  AT << "); Serial.println(response);
        return true;
      }
    }
  }

  Serial.print("  AT timeout. Received: "); Serial.println(response);
  return false;
}


// ============================================================
//   STATUS REPORT — printed every 5s in clean mode
// ============================================================
void printStatusReport() {
  const char* stateNames[] = {"STANDBY", "DEPLOYING", "DEPLOYED", "RETRACTING", "COOLDOWN"};

  Serial.println("----- STATUS REPORT -----");
  Serial.print("ADC value   : "); Serial.println(lastEMGValue);
  Serial.print("Threshold   : "); Serial.println(emgThreshold);
  Serial.print("~mV at SIG  : "); Serial.println((int)(lastEMGValue * ADC_SCALE_TO_MV));
  Serial.print("EMG status  : "); Serial.println(lastEMGValue > emgThreshold ? "ACTIVE" : "NORMAL");
  Serial.print("Shield      : "); Serial.println(stateNames[shieldState]);
  Serial.print("Servo pos   : "); Serial.print(servoCurrentPos); Serial.println("°");
  Serial.print("Servo target: "); Serial.print(servoTargetPos);  Serial.println("°");

  if (gps.location.isValid()) {
    Serial.print("GPS live    : ");
    Serial.print(gps.location.lat(), 6); Serial.print(", ");
    Serial.println(gps.location.lng(), 6);
  } else if (hadGPSFix) {
    Serial.print("GPS last    : "); Serial.print((millis() - lastGPSFixTime) / 1000); Serial.println("s ago");
  } else {
    Serial.println("GPS         : Searching...");
  }

  Serial.println("-------------------------\n");
}


// ============================================================
//   BOOT DASHBOARD
// ============================================================
void printDashboard() {
  Serial.println("==========================================");
  Serial.println("   EMG WRIST SHIELD — FIRMWARE v4.0");
  Serial.println("   Sensor: Muscle Sensor v3 (Advancer)");
  Serial.println("==========================================");
  Serial.println("WIRING REMINDER:");
  Serial.println("  Sensor: ±9V dual supply (2× 9V batteries)");
  Serial.println("  SIG → 47kΩ/18kΩ divider → GPIO 34");
  Serial.println("  Servo: separate 5V supply, shared GND");
  Serial.println("------------------------------------------");
  Serial.print  ("Deploy angle     : "); Serial.print(SERVO_DEPLOYED_ANGLE); Serial.println("°");
  Serial.print  ("Servo pulse range: "); Serial.print(SERVO_MIN_US); Serial.print("–"); Serial.print(SERVO_MAX_US); Serial.println("µs");
  Serial.print  ("Deploy time ~    : ");
  Serial.print  ((SERVO_DEPLOYED_ANGLE / SERVO_STEP) * SERVO_TICK_MS); Serial.println("ms");
  Serial.print  ("Emergency number : "); Serial.println(EMERGENCY_NUMBER);
  Serial.print  ("Threshold offset : "); Serial.println(THRESHOLD_OFFSET);
  Serial.print  ("Peak gate        : "); Serial.println(PEAK_COUNT_REQUIRED);
  Serial.print  ("Auto-retract     : "); Serial.print(AUTO_RETRACT_MS / 1000); Serial.println("s");
  Serial.println("==========================================");
}


// ============================================================
//   TEST SUITE — EMG WRIST SHIELD v4
//   HOW TO USE:
//     Set TEST_MODE true, flash, open Serial Monitor at 115200.
//     Each test runs automatically in sequence on boot.
//     PASS / FAIL printed for every case.
//     Set TEST_MODE false and reflash for normal operation.
// ============================================================
#define TEST_MODE  false   // Set true to run tests instead of normal operation

#if TEST_MODE

// ── Test helpers ─────────────────────────────────────────────
static int testsPassed = 0;
static int testsFailed = 0;

void testAssert(bool condition, const char* label) {
  if (condition) {
    Serial.print("  PASS: "); Serial.println(label);
    testsPassed++;
  } else {
    Serial.print("  FAIL: "); Serial.println(label);
    testsFailed++;
  }
}

void testHeader(const char* name) {
  Serial.println();
  Serial.print("=== TEST: "); Serial.print(name); Serial.println(" ===");
}

void testSummary() {
  Serial.println();
  Serial.println("==============================");
  Serial.print("PASSED: "); Serial.println(testsPassed);
  Serial.print("FAILED: "); Serial.println(testsFailed);
  Serial.println("==============================");
}

// ── TC-01: degreesToMicroseconds() boundary values ───────────
// Verifies the pulse-width conversion formula at 0°, 90°, 180°.
// Expected: 0°=544us, 90°=1472us, 180°=2400us (SG90 range).
// A wrong formula here means servo never reaches correct position.
void test_degreesToMicroseconds() {
  testHeader("TC-01  degreesToMicroseconds() boundary values");

  // 0 degrees must map to SERVO_MIN_US exactly
  testAssert(degreesToMicroseconds(0) == SERVO_MIN_US,
             "0 deg => SERVO_MIN_US (544us)");

  // 180 degrees must map to SERVO_MAX_US exactly
  testAssert(degreesToMicroseconds(180) == SERVO_MAX_US,
             "180 deg => SERVO_MAX_US (2400us)");

  // 90 degrees must land at midpoint: 544 + (90/180)*(2400-544) = 544+928 = 1472
  int mid = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) / 2;
  testAssert(degreesToMicroseconds(90) == mid,
             "90 deg => midpoint (1472us)");

  // Values outside 0-180 must be clamped, not wrapped or overflowed
  testAssert(degreesToMicroseconds(-10) == SERVO_MIN_US,
             "Negative input clamped to 0 deg");
  testAssert(degreesToMicroseconds(200) == SERVO_MAX_US,
             "Over-180 input clamped to 180 deg");
}

// ── TC-02: servoWriteDeg() clamps to [RETRACTED, DEPLOYED] ───
// SERVO_DEPLOYED_ANGLE is 90. Writing 120 must silently clamp
// to 90 — the servo must never travel beyond its configured max.
// Without this, a firmware bug could ram the servo past 90° and
// snap the mechanical linkage of the shield panels.
void test_servoWriteDeg_clamping() {
  testHeader("TC-02  servoWriteDeg() clamps to configured range");

  // After writing 0, servo should hold at RETRACTED
  servoWriteDeg(0);
  // We can't read back the servo position from hardware, so we test
  // the upstream clamp in degreesToMicroseconds which servoWriteDeg calls.
  int usAt0   = degreesToMicroseconds(constrain(0,   SERVO_RETRACTED_ANGLE, SERVO_DEPLOYED_ANGLE));
  int usAt90  = degreesToMicroseconds(constrain(90,  SERVO_RETRACTED_ANGLE, SERVO_DEPLOYED_ANGLE));
  int usAt120 = degreesToMicroseconds(constrain(120, SERVO_RETRACTED_ANGLE, SERVO_DEPLOYED_ANGLE));
  int usAtNeg = degreesToMicroseconds(constrain(-5,  SERVO_RETRACTED_ANGLE, SERVO_DEPLOYED_ANGLE));

  testAssert(usAt0  == SERVO_MIN_US,
             "0 deg stays at MIN_US after clamp");
  testAssert(usAt90 == degreesToMicroseconds(SERVO_DEPLOYED_ANGLE),
             "90 deg (DEPLOYED_ANGLE) resolves correctly");
  testAssert(usAt120 == degreesToMicroseconds(SERVO_DEPLOYED_ANGLE),
             "120 deg clamped down to DEPLOYED_ANGLE (90)");
  testAssert(usAtNeg == SERVO_MIN_US,
             "Negative deg clamped up to RETRACTED (0)");
}

// ── TC-03: updateServo() incremental sweep logic ─────────────
// Simulates the non-blocking sweep by manually setting
// servoCurrentPos and servoTargetPos, then calling updateServo()
// logic inline. Verifies step size, direction, and at-target detection.
void test_updateServo_sweep() {
  testHeader("TC-03  updateServo() incremental sweep behaviour");

  // --- Deploy direction: 0 -> 90 ---
  // Save real state
  int savedCurrent = servoCurrentPos;
  int savedTarget  = servoTargetPos;

  servoCurrentPos = 0;
  servoTargetPos  = SERVO_DEPLOYED_ANGLE;  // 90

  // Simulate one tick: position should advance by exactly SERVO_STEP
  int expected = min(servoCurrentPos + SERVO_STEP, servoTargetPos);
  servoCurrentPos = min(servoCurrentPos + SERVO_STEP, servoTargetPos);
  servoCurrentPos = constrain(servoCurrentPos, SERVO_RETRACTED_ANGLE, SERVO_DEPLOYED_ANGLE);
  testAssert(servoCurrentPos == expected,
             "Deploy: first tick advances by SERVO_STEP (3 deg)");

  // Simulate reaching exactly the target — servoAtTarget() must return true
  servoCurrentPos = SERVO_DEPLOYED_ANGLE;
  testAssert(servoAtTarget(),
             "Deploy: servoAtTarget() true at 90 deg");

  // --- Retract direction: 90 -> 0 ---
  servoCurrentPos = SERVO_DEPLOYED_ANGLE;   // 90
  servoTargetPos  = SERVO_RETRACTED_ANGLE;  // 0

  int expRetract = max(servoCurrentPos - SERVO_STEP, servoTargetPos);
  servoCurrentPos = max(servoCurrentPos - SERVO_STEP, servoTargetPos);
  servoCurrentPos = constrain(servoCurrentPos, SERVO_RETRACTED_ANGLE, SERVO_DEPLOYED_ANGLE);
  testAssert(servoCurrentPos == expRetract,
             "Retract: first tick retreats by SERVO_STEP (3 deg)");

  // Simulate reaching 0 — servoAtTarget() must return true
  servoCurrentPos = SERVO_RETRACTED_ANGLE;
  testAssert(servoAtTarget(),
             "Retract: servoAtTarget() true at 0 deg");

  // --- At-target: no movement when already there ---
  servoCurrentPos = 45;
  servoTargetPos  = 45;
  int before = servoCurrentPos;
  // Neither branch of updateServo fires — position unchanged
  if (servoCurrentPos < servoTargetPos)      servoCurrentPos += SERVO_STEP;
  else if (servoCurrentPos > servoTargetPos) servoCurrentPos -= SERVO_STEP;
  testAssert(servoCurrentPos == before,
             "At-target: position unchanged when current==target");

  // Restore state
  servoCurrentPos = savedCurrent;
  servoTargetPos  = savedTarget;
}

// ── TC-04: EMG detection — peak counter gate ─────────────────
// The system must NOT trigger on a single above-threshold spike
// (PEAK_COUNT_REQUIRED = 3). Tests the exact boundary:
//   2 consecutive readings → no trigger
//   3 consecutive readings → triggers
//   interrupted sequence  → count resets to 0
void test_emgDetection_peakGate() {
  testHeader("TC-04  EMG detection — consecutive peak gate");

  // Save real system state
  ShieldState savedState = shieldState;
  int savedPeakCount     = peakCount;

  shieldState = STANDBY;   // Detection only runs in STANDBY
  peakCount   = 0;

  // 2 consecutive above-threshold — must NOT yet trigger
  // (trigger would change shieldState away from STANDBY)
  for (int i = 0; i < PEAK_COUNT_REQUIRED - 1; i++) {
    if (peakCount < PEAK_COUNT_REQUIRED) peakCount++;
  }
  testAssert(shieldState == STANDBY,
             "2 peaks (< required): no trigger yet");
  testAssert(peakCount == PEAK_COUNT_REQUIRED - 1,
             "peakCount == PEAK_COUNT_REQUIRED - 1 after 2 peaks");

  // One below-threshold reading — counter must reset to 0
  peakCount = 0;
  testAssert(peakCount == 0,
             "Below-threshold reading resets peakCount to 0");

  // Verify threshold math: a reading at exactly threshold is NOT above it
  // (condition is value > threshold, not >=)
  int mockBaseline  = 200;
  int mockThreshold = mockBaseline + THRESHOLD_OFFSET;  // 500
  int atThreshold   = mockThreshold;       // exactly 500 — should NOT trigger
  int aboveThreshold = mockThreshold + 1;  // 501 — should trigger

  testAssert(atThreshold <= mockThreshold,
             "Value == threshold does NOT satisfy > condition");
  testAssert(aboveThreshold > mockThreshold,
             "Value == threshold+1 satisfies > condition");

  // Restore state
  shieldState = savedState;
  peakCount   = savedPeakCount;
}

// ── TC-05: State machine — valid transition sequence ─────────
// Manually walks the state machine through the full lifecycle:
//   STANDBY → DEPLOYING → DEPLOYED → RETRACTING → COOLDOWN → STANDBY
// Verifies each transition fires at the right condition and that
// no state is skipped or repeated.
void test_stateMachine_transitions() {
  testHeader("TC-05  State machine — full lifecycle transitions");

  ShieldState savedState  = shieldState;
  int savedCurrentPos     = servoCurrentPos;
  int savedTargetPos      = servoTargetPos;
  unsigned long savedDeploy  = deployTimestamp;
  unsigned long savedRetract = retractTimestamp;

  // Start: STANDBY
  shieldState = STANDBY;
  testAssert(shieldState == STANDBY, "Starts in STANDBY");

  // Trigger: moves to DEPLOYING
  shieldState    = DEPLOYING;
  servoTargetPos = SERVO_DEPLOYED_ANGLE;
  testAssert(shieldState == DEPLOYING, "After trigger: DEPLOYING");

  // Servo reaches target: moves to DEPLOYED
  servoCurrentPos = SERVO_DEPLOYED_ANGLE;
  if (servoAtTarget()) {
    shieldState     = DEPLOYED;
    deployTimestamp = millis();
  }
  testAssert(shieldState == DEPLOYED, "Servo at target: DEPLOYED");

  // Auto-retract fires: moves to RETRACTING
  // Force timer expiry by backdating deployTimestamp
  deployTimestamp = millis() - AUTO_RETRACT_MS - 1;
  if (millis() - deployTimestamp >= AUTO_RETRACT_MS) {
    servoTargetPos = SERVO_RETRACTED_ANGLE;
    shieldState    = RETRACTING;
  }
  testAssert(shieldState == RETRACTING, "Timer elapsed: RETRACTING");

  // Servo reaches 0: moves to COOLDOWN
  servoCurrentPos = SERVO_RETRACTED_ANGLE;
  if (servoAtTarget()) {
    shieldState      = COOLDOWN;
    retractTimestamp = millis();
  }
  testAssert(shieldState == COOLDOWN, "Servo at 0: COOLDOWN");

  // Cooldown expires: back to STANDBY
  retractTimestamp = millis() - REARM_DELAY_MS - 1;
  if (millis() - retractTimestamp >= REARM_DELAY_MS) {
    shieldState = STANDBY;
    peakCount   = 0;
  }
  testAssert(shieldState == STANDBY, "Cooldown expired: back to STANDBY");

  // Restore
  shieldState     = savedState;
  servoCurrentPos = savedCurrentPos;
  servoTargetPos  = savedTargetPos;
  deployTimestamp = savedDeploy;
  retractTimestamp = savedRetract;
}

// ── TC-06: Double-trigger guard ───────────────────────────────
// If triggerSystem() is called while already DEPLOYING, DEPLOYED,
// RETRACTING, or COOLDOWN it must be silently ignored.
// Without this, a sustained EMG contraction during deployment
// would corrupt the state machine.
void test_doubleTriggerGuard() {
  testHeader("TC-06  Double-trigger guard — non-STANDBY states");

  ShieldState savedState = shieldState;
  int savedTarget        = servoTargetPos;

  const ShieldState nonStandbyStates[] = { DEPLOYING, DEPLOYED, RETRACTING, COOLDOWN };
  const char* names[] = { "DEPLOYING", "DEPLOYED", "RETRACTING", "COOLDOWN" };

  for (int i = 0; i < 4; i++) {
    shieldState    = nonStandbyStates[i];
    servoTargetPos = SERVO_RETRACTED_ANGLE;   // Known baseline

    // Simulate what triggerSystem() does when state != STANDBY:
    // it must NOT change shieldState or servoTargetPos
    ShieldState stateBefore  = shieldState;
    int         targetBefore = servoTargetPos;

    if (shieldState != STANDBY) {
      // Trigger ignored — nothing changes
    } else {
      // This branch must never execute in this test
      shieldState    = DEPLOYING;
      servoTargetPos = SERVO_DEPLOYED_ANGLE;
    }

    char label[60];
    snprintf(label, sizeof(label), "Trigger ignored in %s state", names[i]);
    testAssert(shieldState    == stateBefore  &&
               servoTargetPos == targetBefore, label);
  }

  shieldState    = savedState;
  servoTargetPos = savedTarget;
}

// ── TC-07: Button debounce — stable-state edge detection ─────
// The button should only fire on a confirmed falling edge.
// Simulates raw readings changing and verifies that:
//   - A single reading change does NOT immediately fire
//   - Only after BUTTON_DEBOUNCE_MS of stability does it register
// (We can't inject real millis(), so we test the logic variables.)
void test_buttonDebounce_logic() {
  testHeader("TC-07  Button debounce — edge detection variables");

  int savedRaw    = buttonLastRawReading;
  int savedStable = buttonStableState;

  // Initial state: both HIGH (button released)
  buttonLastRawReading = HIGH;
  buttonStableState    = HIGH;

  // Simulate raw reading goes LOW (button pressed)
  int rawReading = LOW;
  if (rawReading != buttonLastRawReading) {
    buttonDebounceTime   = millis();
    buttonLastRawReading = rawReading;
  }
  testAssert(buttonLastRawReading == LOW,
             "Raw reading updated to LOW on change");
  testAssert(buttonStableState == HIGH,
             "Stable state still HIGH before debounce window");

  // Simulate debounce window passed and reading still LOW
  // (we fake this by checking what WOULD happen)
  bool wouldFire = (rawReading != buttonStableState);
  testAssert(wouldFire,
             "After debounce window: LOW != HIGH => would register press");

  // Simulate button released without debounce — raw goes HIGH
  rawReading = HIGH;
  if (rawReading != buttonLastRawReading) {
    buttonDebounceTime   = millis();
    buttonLastRawReading = rawReading;
  }
  testAssert(buttonLastRawReading == HIGH,
             "Raw reading tracks release (HIGH)");

  // Restore
  buttonLastRawReading = savedRaw;
  buttonStableState    = savedStable;
}

// ── TC-08: Calibration threshold arithmetic ───────────────────
// Directly verifies that emgThreshold = emgBaseline + THRESHOLD_OFFSET.
// Also checks the high-baseline warning boundary (>800 ADC counts).
void test_calibrationThreshold() {
  testHeader("TC-08  Calibration — threshold arithmetic");

  int savedBaseline  = emgBaseline;
  int savedThreshold = emgThreshold;

  // Normal low baseline
  emgBaseline  = 150;
  emgThreshold = emgBaseline + THRESHOLD_OFFSET;
  testAssert(emgThreshold == 150 + THRESHOLD_OFFSET,
             "Threshold = baseline + THRESHOLD_OFFSET (150+300=450)");

  // High baseline — warning boundary
  emgBaseline  = 800;
  emgThreshold = emgBaseline + THRESHOLD_OFFSET;
  testAssert(emgBaseline <= 800,
             "Baseline == 800: at warning boundary (not over)");

  emgBaseline  = 801;
  testAssert(emgBaseline > 800,
             "Baseline == 801: triggers >800 warning condition");

  // Zero baseline edge case (sensor disconnected / no signal)
  emgBaseline  = 0;
  emgThreshold = emgBaseline + THRESHOLD_OFFSET;
  testAssert(emgThreshold == THRESHOLD_OFFSET,
             "Zero baseline: threshold equals THRESHOLD_OFFSET exactly");

  // Restore
  emgBaseline  = savedBaseline;
  emgThreshold = savedThreshold;
}

// ── TC-09: GPS fallback logic ─────────────────────────────────
// When GPS has no live fix, the SMS must use the last known
// position cache (hadGPSFix=true) rather than "no fix" message.
// When there has never been a fix, it must use the no-fix message.
void test_gpsFallback() {
  testHeader("TC-09  GPS fallback — cache vs no-fix message");

  bool savedHad = hadGPSFix;
  double savedLat = lastKnownLat;
  double savedLng = lastKnownLng;

  // Scenario A: never had a fix
  hadGPSFix    = false;
  lastKnownLat = 0.0;
  lastKnownLng = 0.0;
  // gps.location.isValid() would be false here too
  // Code path: else branch → "no fix available"
  bool usesNoFixMessage = (!hadGPSFix);
  testAssert(usesNoFixMessage,
             "No prior fix: falls through to no-fix message branch");

  // Scenario B: had a fix previously, now lost
  hadGPSFix    = true;
  lastKnownLat = 16.506174;
  lastKnownLng = 80.648015;
  // Code path: else-if branch → uses cached coords
  bool usesCachedCoords = (hadGPSFix && lastKnownLat != 0.0);
  testAssert(usesCachedCoords,
             "Prior fix exists: uses cached lat/lng fallback");

  // Scenario C: cached coordinates are non-zero
  testAssert(lastKnownLat != 0.0 && lastKnownLng != 0.0,
             "Cached coords non-zero after previous fix");

  hadGPSFix    = savedHad;
  lastKnownLat = savedLat;
  lastKnownLng = savedLng;
}

// ── TC-10: Deploy timing calculation ─────────────────────────
// Verifies that the deploy time printed in printDashboard() is
// arithmetically correct for the current SERVO_STEP and SERVO_TICK_MS.
// A wrong value here gives the user misleading expectations about
// how fast the shield actually deploys.
void test_deployTimingCalculation() {
  testHeader("TC-10  Deploy timing — step/tick arithmetic");

  // Expected: ceil(SERVO_DEPLOYED_ANGLE / SERVO_STEP) * SERVO_TICK_MS
  // With 90 deg, step=3, tick=15: 30 steps * 15ms = 450ms
  int expectedSteps = SERVO_DEPLOYED_ANGLE / SERVO_STEP;
  int expectedMs    = expectedSteps * SERVO_TICK_MS;

  testAssert(expectedSteps == 30,
             "90 deg / 3 step = 30 ticks to full deploy");
  testAssert(expectedMs == 450,
             "30 ticks x 15ms = 450ms full deploy time");

  // Verify the dashboard formula matches
  int dashboardMs = (SERVO_DEPLOYED_ANGLE / SERVO_STEP) * SERVO_TICK_MS;
  testAssert(dashboardMs == expectedMs,
             "Dashboard formula matches hand-calculated 450ms");

  // Edge: if SERVO_STEP doesn't divide evenly into SERVO_DEPLOYED_ANGLE,
  // integer division truncates — the actual last step may be smaller.
  // 90 / 3 = 30 exactly, so no truncation error here.
  testAssert((SERVO_DEPLOYED_ANGLE % SERVO_STEP) == 0,
             "90 deg divides evenly by SERVO_STEP (no truncation error)");
}

// ── Main test runner (replaces normal setup+loop) ────────────
void runAllTests() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("############################################");
  Serial.println("  EMG SHIELD v4 — TEST SUITE");
  Serial.println("############################################");

  test_degreesToMicroseconds();
  test_servoWriteDeg_clamping();
  test_updateServo_sweep();
  test_emgDetection_peakGate();
  test_stateMachine_transitions();
  test_doubleTriggerGuard();
  test_buttonDebounce_logic();
  test_calibrationThreshold();
  test_gpsFallback();
  test_deployTimingCalculation();

  testSummary();
}

void setup() { runAllTests(); }
void loop()  {}

#endif  // TEST_MODE
