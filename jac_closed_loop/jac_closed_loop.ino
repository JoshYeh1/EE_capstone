#include <math.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// Goniometer control Firmware FINAL VERSION with SCPI Commands implemented

// -------------------- Setup Pins --------------------
const int StepX = 2;
const int DirX  = 5;
const int StepY = 3;
const int DirY  = 6;

const int homeSwitchPin   = 9;
const bool switchActiveLow = true;

// -------------------- Hall Sensors --------------------
const int HALL_X_PIN = A0;
const int HALL_Y_PIN = A1;
const int HALL_Z_PIN = A2;

const uint8_t NUM_SAMPLES = 48;
const uint8_t HALL_SAMPLE_DELAY_MS = 2;

const float X_OFFSET = 502.57f;
const float Y_OFFSET = 505.000;
const float Z_OFFSET = 428.340;

// -------------------- Structs (all declared before any function that references them) --------------------

struct HallData {
  float x;
  float y;
  float z;
};

struct FieldVector {
  float x;
  float y;
  float z;
};


struct HallOrientation {
  float thetaDeg;
  float phiDeg;
  float magnitude;
  bool phiDefined;
  bool valid;
};

struct SavedState {
  uint32_t magic;
  long     stepsX;
  long     stepsY;
  uint8_t  valid;
  uint8_t  inMotion;
  uint16_t checksum;
};

// One compact, persistent Jacobian calibration.  This deliberately stores only
// four floats instead of a large grid so it is friendly to small AVR boards.
struct SavedJacobian {
  uint32_t magic;
  float a, b, c, d;
  uint16_t checksum;
};

// These result types must be declared before Arduino auto-generates function
// prototypes. Keeping them here prevents the IDE from seeing an unknown return
// type for the adaptive recovery helpers below.
enum VerifiedStepResult : uint8_t {
  VERIFIED_STEP_ACCEPTED,
  VERIFIED_STEP_REJECTED,
  VERIFIED_STEP_ERROR
};

enum PhiRecoveryResult : uint8_t {
  PHI_RECOVERY_ACCEPTED,
  PHI_RECOVERY_FAILED,
  PHI_RECOVERY_ERROR
};

// These must be declared near the top because the Arduino IDE auto-generates
// function prototypes before later code sections.


// -------------------- Forward Declarations --------------------
float currentPsiDeg();
float currentOmegaDeg();
float signed180ToUser360(float deg);
float wrapSigned180(float deg);
uint16_t computeChecksum(const SavedState &s);
bool readSavedState(SavedState &s);
void writeSavedState(long stepsX, long stepsY, bool valid, bool inMotion);

// -------------------- Hall Read --------------------
HallData readHallAverage() {
  long xSum = 0, ySum = 0, zSum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    xSum += analogRead(HALL_X_PIN);
    ySum += analogRead(HALL_Y_PIN);
    zSum += analogRead(HALL_Z_PIN);
    delay(HALL_SAMPLE_DELAY_MS);
  }
  HallData result;
  result.x = ((float)xSum / NUM_SAMPLES) - X_OFFSET;
  result.y = ((float)ySum / NUM_SAMPLES) - Y_OFFSET;
  result.z = (((float)zSum / NUM_SAMPLES) - Z_OFFSET) * 0.94f;

  // Small Hall sensor mounting-tilt correction: rotate X/Z about Y axis
  const float TILT_DEG = 4.0f;
  const float TILT_RAD = TILT_DEG * PI / 180.0f;

  float xOld = result.x;
  float zOld = result.z;

  result.x = xOld * cos(TILT_RAD) + zOld * sin(TILT_RAD);
  result.z = -xOld * sin(TILT_RAD) + zOld * cos(TILT_RAD);
  return result;
}

// -------------------- Hall Vector -> Psi/Omega (No Lookup Table) --------------------
const float HALL_X_SCALE = 1.0f;
const float HALL_Y_SCALE = 1.0f;
const float HALL_Z_SCALE = 1.0f;


float vectorMagnitude(const FieldVector &v) {
  return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

FieldVector normalizeVector(const FieldVector &v) {
  float m = vectorMagnitude(v);
  if (m < 0.001f) return {0.0f, 0.0f, 0.0f};
  return {v.x / m, v.y / m, v.z / m};
}


// -------------------- Hall Vector -> Theta/Phi --------------------
// THETA/PHI are measured directly in the Hall/sample coordinate frame.
// Hall +Z means field through the front of the sample:
//   +Z -> THETA near 0 deg, XY plane -> THETA near 90 deg, -Z -> THETA near 180 deg.
// This assumes Hall Z is mounted parallel to the sample normal.
const float PHI_UNDEFINED_THETA_DEG = 8.0f;

HallOrientation calculateThetaPhi(const FieldVector &rawB) {
  HallOrientation out;
  out.thetaDeg = 0.0f;
  out.phiDeg = 0.0f;
  out.magnitude = vectorMagnitude(rawB);
  out.phiDefined = false;
  out.valid = false;
  if (out.magnitude < 0.001f) return out;

  // Use the Hall/sample axes directly.
  FieldVector b = normalizeVector(rawB);
  float z = constrain(b.z, -1.0f, 1.0f);
  out.thetaDeg = acosf(z) * 180.0f / PI;

  float xy = sqrtf(b.x*b.x + b.y*b.y);
  if (xy >= sinf(PHI_UNDEFINED_THETA_DEG * PI / 180.0f)) {
    out.phiDeg = atan2f(b.y, b.x) * 180.0f / PI;
    if (out.phiDeg < 0.0f) out.phiDeg += 360.0f;
    out.phiDefined = true;
  }
  out.valid = true;
  return out;
}

float phiErrorDeg(float actualPhiDeg, float targetPhiDeg) {
  return fabsf(wrapSigned180(actualPhiDeg - targetPhiDeg));
}

// -------------------- Motor Settings --------------------
const int   stepsPerRev   = 200;
const int   microsteps    = 32;
const float DEG_PER_STEP  = 360.0f / (stepsPerRev * microsteps);

const unsigned long PULSE_HIGH_US = 3UL;
const bool invertDirX = true;
const bool invertDirY = false;

// -------------------- Acceleration Ramp --------------------
const float accelSPS2 = 100.0f;
const float minSPS    = 5.0f;

// -------------------- Speed Constants --------------------
const float homingRPM = 2.0f;
const float goZeroRPM = 5.0f;

// -------------------- Cable / Travel Limits --------------------
// These are PHYSICAL, UNWRAPPED stage coordinates used to protect the wiring.
// The mechanism can safely use the full +/-180 degree working range.  The extra
// 20 degrees on each side are reserve travel, not a normal operating target.
// Keep these values inside the cable-twist range you verify experimentally.
const float PSI_MIN_ANGLE_DEG   = -200.0f;
const float PSI_MAX_ANGLE_DEG   =  200.0f;
const float OMEGA_MIN_ANGLE_DEG = -200.0f;
const float OMEGA_MAX_ANGLE_DEG =  200.0f;

// -------------------- Position Memory --------------------
long currentStepsY = 0L;
long currentStepsX = 0L;

// -------------------- EEPROM --------------------
const uint32_t EEPROM_MAGIC = 0x47A10E55UL;
const int      EEPROM_ADDR  = 0;
const uint32_t JACOBIAN_MAGIC = 0x4A41434FUL; // "JACO"
const int EEPROM_JACOBIAN_ADDR = EEPROM_ADDR + (int)sizeof(SavedState);

uint16_t computeChecksum(const SavedState &s) {
  const uint8_t *p = (const uint8_t *)&s;
  uint16_t sum = 0;
  for (size_t i = 0; i < sizeof(SavedState) - sizeof(uint16_t); i++)
    sum = (uint16_t)(sum + p[i]);
  return sum;
}

bool readSavedState(SavedState &s) {
  EEPROM.get(EEPROM_ADDR, s);
  if (s.magic != EEPROM_MAGIC) return false;
  uint16_t expected = s.checksum;
  s.checksum = 0;
  uint16_t actual = computeChecksum(s);
  s.checksum = expected;
  return (expected == actual);
}

void writeSavedState(long stepsX, long stepsY, bool valid, bool inMotion) {
  SavedState s;
  s.magic    = EEPROM_MAGIC;
  s.stepsX   = stepsX;
  s.stepsY   = stepsY;
  s.valid    = valid    ? 1 : 0;
  s.inMotion = inMotion ? 1 : 0;
  s.checksum = 0;
  s.checksum = computeChecksum(s);
  EEPROM.put(EEPROM_ADDR, s);
}

void saveCurrentPositionConfirmed() {
  writeSavedState(currentStepsX, currentStepsY, true, false);
}

void markMotionStarted() {
  writeSavedState(currentStepsX, currentStepsY, true, true);
}

uint16_t computeJacobianChecksum(const SavedJacobian &j) {
  const uint8_t *p = (const uint8_t *)&j;
  uint16_t sum = 0;
  for (size_t i = 0; i < sizeof(SavedJacobian) - sizeof(uint16_t); i++) sum += p[i];
  return sum;
}

bool readSavedJacobian(SavedJacobian &j) {
  EEPROM.get(EEPROM_JACOBIAN_ADDR, j);
  if (j.magic != JACOBIAN_MAGIC) return false;
  uint16_t expected = j.checksum;
  j.checksum = 0;
  uint16_t actual = computeJacobianChecksum(j);
  j.checksum = expected;
  return expected == actual;
}

void writeSavedJacobian(float a, float b, float c, float d) {
  SavedJacobian j;
  j.magic = JACOBIAN_MAGIC; j.a = a; j.b = b; j.c = c; j.d = d;
  j.checksum = 0; j.checksum = computeJacobianChecksum(j);
  EEPROM.put(EEPROM_JACOBIAN_ADDR, j);
}

void clearSavedJacobian() {
  SavedJacobian j;
  memset(&j, 0, sizeof(j));
  EEPROM.put(EEPROM_JACOBIAN_ADDR, j);
}

// -------------------- SCPI Error Queue --------------------
const int ERROR_QUEUE_SIZE  = 4;
const int ERROR_ENTRY_LEN   = 32;

static char errorQueue[ERROR_QUEUE_SIZE][ERROR_ENTRY_LEN];
static int  errorHead  = 0;
static int  errorTail  = 0;
static int  errorCount = 0;

void pushError(int code, const __FlashStringHelper *msg) {
  if (errorCount >= ERROR_QUEUE_SIZE) {
    errorHead = (errorHead + 1) % ERROR_QUEUE_SIZE;
    errorCount--;
  }
  char *slot = errorQueue[errorTail];
  itoa(code, slot, 10);
  uint8_t used = (uint8_t)strlen(slot);
  slot[used++] = ',';
  PGM_P p = reinterpret_cast<PGM_P>(msg);
  uint8_t remaining = ERROR_ENTRY_LEN - used - 1;
  uint8_t i = 0;
  char c;
  while (i < remaining && (c = pgm_read_byte(p + i)) != '\0') {
    slot[used + i] = c;
    i++;
  }
  slot[used + i] = '\0';
  errorTail = (errorTail + 1) % ERROR_QUEUE_SIZE;
  errorCount++;
}

void popErrorToSerial() {
  if (errorCount == 0) {
    Serial.println(F("0,No error"));
    return;
  }
  Serial.println(errorQueue[errorHead]);
  errorHead = (errorHead + 1) % ERROR_QUEUE_SIZE;
  errorCount--;
}

// -------------------- Startup Flags --------------------
bool startupStateLoaded         = false;
bool startupStateIncompleteMove = false;

// -------------------- Angle Conversion --------------------
float wrap360(float deg) {
  while (deg <    0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

float wrapSigned180(float deg) {
  while (deg <= -180.0f) deg += 360.0f;
  while (deg >   180.0f) deg -= 360.0f;
  return deg;
}

float user360ToSigned180(float deg) {
  deg = wrap360(deg);
  if (deg > 180.0f) deg -= 360.0f;
  return deg;
}

float signed180ToUser360(float deg) { return wrap360(deg); }

// -------------------- Position Accessors --------------------
float currentPsiDeg()   { return currentStepsY * DEG_PER_STEP; }
float currentOmegaDeg() { return (currentStepsX - currentStepsY) * DEG_PER_STEP; }

// -------------------- Hall Output --------------------
FieldVector readFieldVector() {
  HallData h = readHallAverage();
  return {
    h.x * HALL_X_SCALE,
    h.y * HALL_Y_SCALE,
    h.z * HALL_Z_SCALE
  };
}

void sendHallData() {
  FieldVector b = readFieldVector();
  HallOrientation o = calculateThetaPhi(b);

  if (!o.valid) {
    Serial.println(F("ERR,FIELD_TOO_SMALL"));
    return;
  }

  Serial.print(F("BX,"));          Serial.print(b.x, 3);
  Serial.print(F(",BY,"));         Serial.print(b.y, 3);
  Serial.print(F(",BZ,"));         Serial.print(b.z, 3);
  Serial.print(F(",BMAG,"));       Serial.print(o.magnitude, 3);
  Serial.print(F(",THETA,"));      Serial.print(o.thetaDeg, 2);
  Serial.print(F(",PHI,"));
  if (o.phiDefined) Serial.print(o.phiDeg, 2);
  else              Serial.print(F("UNDEFINED"));
  Serial.print(F(",MOTOR_PSI,"));  Serial.print(signed180ToUser360(currentPsiDeg()), 2);
  Serial.print(F(",MOTOR_OMEGA,")); Serial.println(signed180ToUser360(currentOmegaDeg()), 2);
}

// -------------------- Utility --------------------
float rpmToSPS(float rpm) {
  return (rpm * (float)(stepsPerRev * microsteps)) / 60.0f;
}

unsigned long spsToPeriodUs(float sps) {
  if (sps <= 0.0f) return 0UL;
  float p = 1000000.0f / sps;
  if (p < 2.0f * PULSE_HIGH_US) p = 2.0f * (float)PULSE_HIGH_US;
  return (unsigned long)(p + 0.5f);
}

float getRampSPS(long i, long N, float rpm) {
  float maxSPS = rpmToSPS(rpm);
  if (maxSPS < minSPS) maxSPS = minSPS;

  long accelSteps = (long)((maxSPS*maxSPS - minSPS*minSPS) / (2.0f * accelSPS2));
  if (accelSteps < 1L) accelSteps = 1L;
  if (2L * accelSteps > N) accelSteps = N / 2L;
  long cruiseSteps = N - 2L * accelSteps;

  float sps;
  if (i < accelSteps) {
    sps = sqrtf(minSPS*minSPS + 2.0f * accelSPS2 * (float)i);
    if (sps > maxSPS) sps = maxSPS;
  } else if (i >= accelSteps + cruiseSteps) {
    long d = (N - 1L) - i;
    if (d < 0L) d = 0L;
    sps = sqrtf(minSPS*minSPS + 2.0f * accelSPS2 * (float)d);
    if (sps > maxSPS) sps = maxSPS;
    if (sps < minSPS) sps = minSPS;
  } else {
    sps = maxSPS;
  }
  return sps;
}

void setDir(int pin, long delta, bool invert) {
  bool wantPos = (delta >= 0L);
  int level = wantPos ? HIGH : LOW;
  if (invert) level = (level == HIGH) ? LOW : HIGH;
  digitalWrite(pin, level);
}

void stepPulse(bool doX, bool doY) {
  if (doX) digitalWrite(StepX, HIGH);
  if (doY) digitalWrite(StepY, HIGH);
  delayMicroseconds(PULSE_HIGH_US);
  if (doX) digitalWrite(StepX, LOW);
  if (doY) digitalWrite(StepY, LOW);
}

bool inPsiRange(float deg) {
  return (deg >= PSI_MIN_ANGLE_DEG && deg <= PSI_MAX_ANGLE_DEG);
}

bool inOmegaRange(float deg) {
  return (deg >= OMEGA_MIN_ANGLE_DEG && deg <= OMEGA_MAX_ANGLE_DEG);
}

bool homeSwitchPressed() {
  int v = digitalRead(homeSwitchPin);
  return switchActiveLow ? (v == LOW) : (v == HIGH);
}

bool homeSwitchPressedDebounced() {
  static bool stableState = false;
  static bool lastReading  = false;
  static unsigned long lastChange = 0;

  bool reading = homeSwitchPressed();
  if (reading != lastReading) {
    lastReading = reading;
    lastChange  = millis();
  }
  if ((millis() - lastChange) > 20) stableState = reading;
  return stableState;
}

// -------------------- SCPI Query Responses --------------------
void sendPositionQuery() {
  float psiUser   = signed180ToUser360(currentPsiDeg());
  float omegaUser = signed180ToUser360(currentOmegaDeg());
  Serial.print(F("PSI,"));        Serial.print(psiUser,   3);
  Serial.print(F(",OMEGA,"));     Serial.print(omegaUser, 3);
  Serial.print(F(",PSI_INT,"));   Serial.print(currentPsiDeg(),   3);
  Serial.print(F(",OMEGA_INT,")); Serial.println(currentOmegaDeg(), 3);
}

void sendStepQuery() {
  Serial.print(F("STEPSX,")); Serial.print(currentStepsX);
  Serial.print(F(",STEPSY,")); Serial.println(currentStepsY);
}

void sendSwitchQuery() {
  Serial.print(F("SWITCH,"));
  Serial.println(homeSwitchPressedDebounced() ? F("PRESSED") : F("OPEN"));
}

void sendEEPROMQuery() {
  SavedState s;
  bool ok = readSavedState(s);
  Serial.print(F("EEPROM,"));
  Serial.print(ok ? F("VALID") : F("INVALID"));
  if (ok) {
    Serial.print(F(",SAVED_X,"));     Serial.print(s.stepsX);
    Serial.print(F(",SAVED_Y,"));     Serial.print(s.stepsY);
    Serial.print(F(",STATE_VALID,")); Serial.print((int)s.valid);
    Serial.print(F(",INMOTION,"));    Serial.println((int)s.inMotion);
  } else {
    Serial.println();
  }
}

void sendGoZeroRPMQuery() {
  Serial.print(F("GOZERORPM,")); Serial.println(goZeroRPM, 3);
}

void sendHomingRPMQuery() {
  Serial.print(F("HOMINGRPM,")); Serial.println(homingRPM, 3);
}

// -------------------- Homing --------------------
void executeHomeOmega(int searchDirSign) {
  markMotionStarted();
  bool wantPos = (searchDirSign >= 0);
  int level = wantPos ? HIGH : LOW;
  if (invertDirX) level = (level == HIGH) ? LOW : HIGH;
  digitalWrite(DirX, level);

  unsigned long periodUs   = spsToPeriodUs(rpmToSPS(homingRPM));
  unsigned long lastStepUs = micros();

  while (!homeSwitchPressedDebounced()) {
    if ((unsigned long)(micros() - lastStepUs) >= periodUs) {
      stepPulse(true, false);
      lastStepUs = micros();
    }
  }
  currentStepsX = currentStepsY;
  saveCurrentPositionConfirmed();
}

void executeHomePsi(int searchDirSign) {
  markMotionStarted();
  bool wantPos = (searchDirSign >= 0);

  int levelX = wantPos ? HIGH : LOW;
  if (invertDirX) levelX = (levelX == HIGH) ? LOW : HIGH;
  digitalWrite(DirX, levelX);

  int levelY = wantPos ? HIGH : LOW;
  if (invertDirY) levelY = (levelY == HIGH) ? LOW : HIGH;
  digitalWrite(DirY, levelY);

  unsigned long periodUs   = spsToPeriodUs(rpmToSPS(homingRPM));
  unsigned long lastStepUs = micros();

  while (!homeSwitchPressedDebounced()) {
    if ((unsigned long)(micros() - lastStepUs) >= periodUs) {
      stepPulse(true, true);
      lastStepUs = micros();
    }
  }
  currentStepsX = currentStepsX - currentStepsY;
  currentStepsY = 0L;
  saveCurrentPositionConfirmed();
}

// -------------------- Move --------------------
bool executeMove(float targetPsiDeg, float targetOmegaDeg, float rpm) {
  if (!inPsiRange(targetPsiDeg)) {
    pushError(-222, F("Psi out of range"));
    return false;
  }
  if (!inOmegaRange(targetOmegaDeg)) {
    pushError(-223, F("Omega out of range"));
    return false;
  }
  if (rpm <= 0.0f) {
    pushError(-224, F("RPM must be positive"));
    return false;
  }

  long targetStepsY = lroundf(targetPsiDeg   / DEG_PER_STEP);
  long targetStepsX = lroundf((targetPsiDeg + targetOmegaDeg) / DEG_PER_STEP);

  long deltaStepsY = targetStepsY - currentStepsY;
  long deltaStepsX = targetStepsX - currentStepsX;

  long absX = abs(deltaStepsX);
  long absY = abs(deltaStepsY);

  if (absX == 0L && absY == 0L) return true;

  markMotionStarted();

  setDir(DirX, deltaStepsX, invertDirX);
  setDir(DirY, deltaStepsY, invertDirY);

  long majorSteps = (absX >= absY) ? absX : absY;
  long minorSteps = (absX < absY) ? absX : absY;
  bool xIsMajor   = (absX >= absY);

  float skipInterval = 0.0f;
  if (majorSteps != minorSteps)
    skipInterval = (float)majorSteps / (float)(majorSteps - minorSteps);

  float skipAccum   = 0.0f;
  unsigned long lastStepUs = micros();

  for (long i = 0; i < majorSteps; i++) {
    bool doMinor = true;
    if (skipInterval > 0.0f) {
      skipAccum += 1.0f;
      if (skipAccum >= skipInterval) {
        doMinor    = false;
        skipAccum -= skipInterval;
      }
    }

    bool doStepX = xIsMajor ? true  : doMinor;
    bool doStepY = xIsMajor ? doMinor : true;
    if (absX == 0L) doStepX = false;
    if (absY == 0L) doStepY = false;

    float sps         = getRampSPS(i, majorSteps, rpm);
    unsigned long periodUs = spsToPeriodUs(sps);

    while ((unsigned long)(micros() - lastStepUs) < periodUs) { /* blocking */ }
    lastStepUs = micros();

    stepPulse(doStepX, doStepY);
  }

  currentStepsX = targetStepsX;
  currentStepsY = targetStepsY;
  saveCurrentPositionConfirmed();
  return true;
}

// -------------------- Move and Measure --------------------
void moveAndMeasure(float psi, float omega, float rpm) {
  if (!executeMove(psi, omega, rpm)) {
    Serial.println(F("MOVE FAILED"));
    return;
  }
  delay(500);
  sendHallData();
}

// -------------------- Closed Loop: adaptive Jacobian controller --------------------
// The controller measures the Hall-derived field orientation, predicts a motor
// correction from a local Jacobian, verifies the result, and learns from good
// moves.  When normal predictions stall, it can empirically probe both omega
// directions before using one staged physical transfer to the other branch.

const float CL_THETA_TOL_DEG = 2.0f;
// Near a theta pole, phi is not meaningful and theta repeatability is worse.
const float CL_THETA_ONLY_TOL_DEG = 6.0f;
const float CL_PHI_TOL_DEG = 4.0f;
const float CL_THETA_WEIGHT = 2.5f;
const float CL_PHI_WEIGHT = 1.0f;

// Extra room is needed for a measured omega sweep and a possible alternate
// branch transfer.  These are discrete move/measure iterations, not a
// continuous servo loop.
const int CL_MAX_ITERATIONS = 25;
const unsigned long CL_SETTLE_MS = 450UL;
const int CL_FAST_READS = 2;
const int CL_VERIFY_READS = 5;

// A very tiny score change is usually Hall noise rather than real progress.
// Normal moves may still be accepted if they do not worsen the score, but
// recovery sweeps require this stronger improvement test.
const float CL_MIN_IMPROVEMENT_ABS = 0.35f;
const float CL_MIN_IMPROVEMENT_FRAC = 0.015f;
const int CL_MAX_STAGNANT_MOVES_BEFORE_RECOVERY = 3;

// Jacobian learning is deliberately conservative.  Larger moves cross more
// nonlinear regions, so their observations are down-weighted.
const float CL_MODEL_LEARNING_RATE = 0.12f;
const float CL_MIN_SLOPE_MAG = 0.10f;

// Phi is ill-conditioned near theta = 0 or 180.  Do not let it steer feedback
// until both the target and measured field direction are far enough from a pole.
const float CL_PHI_ENABLE_THETA_DEG = 10.0f;

// Normal prediction failures or stagnant moves trigger an empirical phi recovery
// stage.  It tries the model-preferred omega sign first, then the opposite sign
// if needed.  Good probe moves update the Jacobian.
const int CL_MAX_REJECTS_BEFORE_RECOVERY = 2;
const float CL_PHI_SWEEP_STEP_DEG = 5.0f;
const int CL_PHI_SWEEP_MAX_STEPS = 8;
const float CL_PHI_SWEEP_EXIT_PHI_ERR_DEG = 18.0f;
const float CL_PHI_SWEEP_MAX_THETA_ERR_DEG = 8.0f;

// Closed-loop motion can freely use the normal +/-180 degree working range.
// The feedback workspace ends 3 degrees inside the physical +/-200 degree
// cable limit.  The hard physical limit remains enforced by executeMove().
const float CL_LIMIT_MARGIN_DEG = 3.0f;

// If omega cannot keep moving outward inside the protected workspace, or if
// both measured omega probe directions fail, this transfer physically travels
// through zero in small stages to the opposite +/-180 degree branch.  It is a
// real move and is followed by a new Hall measurement; it is NOT angle wrapping.
const bool CL_ENABLE_BRANCH_TRANSFER_AFTER_SWEEP_FAILURE = true;
const float CL_TRANSFER_OPPOSITE_OMEGA_DEG = 180.0f;
const float CL_TRANSFER_STAGE_DEG = 15.0f;
const float CL_TRANSFER_MAX_RPM = 2.0f;
const unsigned long CL_TRANSFER_SETTLE_MS = 200UL;

struct LocalJacobian {
  // [ dTheta/dPsi  dTheta/dOmega ]
  // [ dPhi/dPsi    dPhi/dOmega   ]
  float a, b, c, d;
  bool valid;
  int updates;
};

// Based on your measured behavior: negative PSI increased THETA in the
// original tested region, so dTheta/dPsi starts negative.
const LocalJacobian CL_MODEL_DEFAULT = {-0.80f, 0.25f, -0.25f, 0.80f, true, 0};
LocalJacobian clModel = {-0.80f, 0.25f, -0.25f, 0.80f, true, 0};

bool thetaSafelyAwayFromPole(float thetaDeg) {
  return (thetaDeg >= CL_PHI_ENABLE_THETA_DEG &&
          thetaDeg <= 180.0f - CL_PHI_ENABLE_THETA_DEG);
}

bool orientationAtTarget(const HallOrientation &o, float targetThetaDeg,
                         float targetPhiDeg, bool targetPhiDefined) {
  const float thetaTolerance = !targetPhiDefined ? CL_THETA_ONLY_TOL_DEG
                                                  : CL_THETA_TOL_DEG;
  if (fabsf(o.thetaDeg - targetThetaDeg) > thetaTolerance) return false;

  // For an intentional pole target, phi is ignored.
  if (!targetPhiDefined || !thetaSafelyAwayFromPole(targetThetaDeg)) return true;

  return thetaSafelyAwayFromPole(o.thetaDeg) && o.phiDefined &&
         (phiErrorDeg(o.phiDeg, targetPhiDeg) <= CL_PHI_TOL_DEG);
}

bool readStableOrientation(HallOrientation &out, int reads = CL_FAST_READS) {
  float thetaSum = 0.0f, magSum = 0.0f, phiSinSum = 0.0f, phiCosSum = 0.0f;
  int validCount = 0, phiCount = 0;
  if (reads < 1) reads = 1;

  for (int i = 0; i < reads; i++) {
    HallOrientation one = calculateThetaPhi(readFieldVector());
    if (!one.valid) continue;

    thetaSum += one.thetaDeg;
    magSum += one.magnitude;
    validCount++;

    if (one.phiDefined) {
      float r = one.phiDeg * PI / 180.0f;
      phiSinSum += sinf(r);
      phiCosSum += cosf(r);
      phiCount++;
    }

    if (i < reads - 1) delay(20);
  }

  if (validCount == 0) return false;

  out.thetaDeg = thetaSum / validCount;
  out.magnitude = magSum / validCount;
  out.valid = true;
  out.phiDefined = (phiCount >= (reads + 1) / 2);
  out.phiDeg = 0.0f;

  if (out.phiDefined) {
    out.phiDeg = atan2f(phiSinSum, phiCosSum) * 180.0f / PI;
    if (out.phiDeg < 0.0f) out.phiDeg += 360.0f;
  }

  return true;
}

float signedPhiDifference(float targetPhi, float measuredPhi) {
  return wrapSigned180(targetPhi - measuredPhi);
}

float closedLoopScore(const HallOrientation &o, float targetTheta, float targetPhi,
                      bool phiNeeded) {
  float thetaError = fabsf(o.thetaDeg - targetTheta);
  if (!phiNeeded || !o.phiDefined) return CL_THETA_WEIGHT * thetaError;
  return CL_THETA_WEIGHT * thetaError +
         CL_PHI_WEIGHT * phiErrorDeg(o.phiDeg, targetPhi);
}

bool scoreMeaningfullyImproved(float beforeScore, float afterScore) {
  float required = beforeScore * CL_MIN_IMPROVEMENT_FRAC;
  if (required < CL_MIN_IMPROVEMENT_ABS) required = CL_MIN_IMPROVEMENT_ABS;
  return afterScore <= (beforeScore - required);
}

float keepNonZeroSlope(float value) {
  if (fabsf(value) >= CL_MIN_SLOPE_MAG) return value;
  return (value < 0.0f) ? -CL_MIN_SLOPE_MAG : CL_MIN_SLOPE_MAG;
}

int signOf(float x) {
  return (x >= 0.0f) ? 1 : -1;
}

// Scale a two-axis correction as one vector rather than clipping each motor
// independently.  This preserves the psi:omega ratio chosen by the Jacobian.
void scaleCorrectionVector(float &dPsi, float &dOmega,
                           float maxPsi, float maxOmega) {
  if (maxPsi <= 0.0f) dPsi = 0.0f;
  if (maxOmega <= 0.0f) dOmega = 0.0f;

  float scale = 1.0f;
  if (maxPsi > 0.0f && fabsf(dPsi) > maxPsi) {
    scale = min(scale, maxPsi / fabsf(dPsi));
  }
  if (maxOmega > 0.0f && fabsf(dOmega) > maxOmega) {
    scale = min(scale, maxOmega / fabsf(dOmega));
  }

  dPsi *= scale;
  dOmega *= scale;
}

bool closedLoopPsiSafe(float psi) {
  return psi >= (PSI_MIN_ANGLE_DEG + CL_LIMIT_MARGIN_DEG) &&
         psi <= (PSI_MAX_ANGLE_DEG - CL_LIMIT_MARGIN_DEG);
}

bool closedLoopOmegaSafe(float omega) {
  return omega >= (OMEGA_MIN_ANGLE_DEG + CL_LIMIT_MARGIN_DEG) &&
         omega <= (OMEGA_MAX_ANGLE_DEG - CL_LIMIT_MARGIN_DEG);
}

bool closedLoopTargetSafe(float psi, float omega) {
  return closedLoopPsiSafe(psi) && closedLoopOmegaSafe(omega);
}

// Move omega continuously through zero to the opposite working branch.  PSI is
// held fixed, so this is a staged physical transfer rather than a fake
// +180/-180 coordinate jump.  It is allowed once per CL:ORIENT command.
bool runSafeOmegaTransfer(float psi, float startOmega, float rpm) {
  float targetOmega = (startOmega >= 0.0f) ? -CL_TRANSFER_OPPOSITE_OMEGA_DEG
                                            :  CL_TRANSFER_OPPOSITE_OMEGA_DEG;

  if (!closedLoopTargetSafe(psi, targetOmega)) return false;

  float transferRPM = (rpm < CL_TRANSFER_MAX_RPM) ? rpm : CL_TRANSFER_MAX_RPM;
  if (transferRPM <= 0.0f) return false;

  Serial.print(F("CL,OMEGA_TRANSFER,FROM,")); Serial.print(startOmega, 2);
  Serial.print(F(",TO,")); Serial.println(targetOmega, 2);

  float omega = startOmega;
  while (fabsf(targetOmega - omega) > 0.25f) {
    float remaining = targetOmega - omega;
    float step = constrain(remaining, -CL_TRANSFER_STAGE_DEG, CL_TRANSFER_STAGE_DEG);
    float nextOmega = omega + step;

    if (!closedLoopTargetSafe(psi, nextOmega) ||
        !executeMove(psi, nextOmega, transferRPM)) {
      pushError(-320, F("Omega transfer failed"));
      Serial.println(F("CL,OMEGA_TRANSFER,FAIL"));
      return false;
    }

    delay(CL_TRANSFER_SETTLE_MS);
    omega = currentOmegaDeg();
  }

  Serial.println(F("CL,OMEGA_TRANSFER,OK"));
  return true;
}

// -------------------- Initial relative coarse approach --------------------
const bool  CL_ENABLE_INITIAL_COARSE = true;
const float CL_COARSE_START_SCORE = 35.0f;
const float CL_COARSE_FRACTION = 0.88f;
const float CL_COARSE_LIMIT_MARGIN = 4.0f;
const float CL_MIN_DET = 0.08f;

float clampCoarsePsiTarget(float x) {
  return constrain(x, PSI_MIN_ANGLE_DEG + CL_COARSE_LIMIT_MARGIN,
                     PSI_MAX_ANGLE_DEG - CL_COARSE_LIMIT_MARGIN);
}

bool runInitialCoarseApproach(const HallOrientation &current, float targetTheta,
                              float targetPhi, bool targetPhiDefined, float rpm) {
  const float curPsi = currentPsiDeg();
  const float curOmega = currentOmegaDeg();

  const bool usePhi = targetPhiDefined && current.phiDefined &&
                      thetaSafelyAwayFromPole(current.thetaDeg) &&
                      thetaSafelyAwayFromPole(targetTheta);

  const float eTheta = targetTheta - current.thetaDeg;
  const float ePhi = usePhi ? signedPhiDifference(targetPhi, current.phiDeg) : 0.0f;
  float dPsi = 0.0f, dOmega = 0.0f;

  if (usePhi) {
    const float det = clModel.a * clModel.d - clModel.b * clModel.c;
    if (fabsf(det) > CL_MIN_DET) {
      dPsi   = ( clModel.d * eTheta - clModel.b * ePhi) / det;
      dOmega = (-clModel.c * eTheta + clModel.a * ePhi) / det;
    } else {
      dPsi = eTheta / keepNonZeroSlope(clModel.a);
    }
  } else {
    dPsi = eTheta / keepNonZeroSlope(clModel.a);
  }

  dPsi *= CL_COARSE_FRACTION;
  dOmega *= CL_COARSE_FRACTION;

  if (!usePhi) {
    dOmega = 0.0f;
    dPsi = constrain(dPsi, -35.0f, 35.0f);

    // On departure from a theta pole, gently recenter omega before asking phi
    // to become meaningful.  This prevents starting phi correction on an
    // unnecessarily wound cable branch.
    if (targetPhiDefined && !thetaSafelyAwayFromPole(current.thetaDeg)) {
      if (curOmega > 20.0f) dOmega = -min(25.0f, curOmega);
      else if (curOmega < -20.0f) dOmega = min(25.0f, -curOmega);
      if (dOmega != 0.0f) Serial.println(F("CL,POLE_DEPART,RECENTER_OMEGA"));
    }
  } else {
    // Preserve the two-axis Jacobian direction while limiting the coarse jump.
    scaleCorrectionVector(dPsi, dOmega, 12.0f, 22.0f);
  }

  float nextPsi = clampCoarsePsiTarget(curPsi + dPsi);
  float nextOmega = curOmega + dOmega;

  if (!closedLoopTargetSafe(nextPsi, nextOmega)) {
    Serial.println(F("CL,COARSE_LIMIT"));
    return false;
  }

  if (fabsf(nextPsi - curPsi) < 0.5f &&
      fabsf(nextOmega - curOmega) < 0.5f) return false;

  Serial.print(F("CL,COARSE_REL,PSI,")); Serial.print(nextPsi, 2);
  Serial.print(F(",OMEGA,")); Serial.println(nextOmega, 2);
  return executeMove(nextPsi, nextOmega, rpm);
}

// -------------------- Normal Jacobian prediction --------------------
void selectMotionLimits(float score, bool poleMode,
                        float &damping, float &maxPsi, float &maxOmega) {
  if (poleMode) {
    if (score > 30.0f) {
      damping = 0.90f; maxPsi = 10.0f; maxOmega = 0.0f;
    } else if (score > 10.0f) {
      damping = 0.75f; maxPsi = 6.0f; maxOmega = 0.0f;
    } else {
      damping = 0.45f; maxPsi = 2.0f; maxOmega = 0.0f;
    }
  } else if (score > 100.0f) {
    damping = 0.95f; maxPsi = 15.0f; maxOmega = 25.0f;
  } else if (score > 35.0f) {
    damping = 0.85f; maxPsi = 10.0f; maxOmega = 18.0f;
  } else if (score > 12.0f) {
    damping = 0.55f; maxPsi = 4.0f; maxOmega = 8.0f;
  } else {
    damping = 0.30f; maxPsi = 1.5f; maxOmega = 3.0f;
  }
}

void printModel() {
  Serial.print(F("CL,MODEL,DTH_DPSI,")); Serial.print(clModel.a, 3);
  Serial.print(F(",DTH_DOMG,"));         Serial.print(clModel.b, 3);
  Serial.print(F(",DPH_DPSI,"));         Serial.print(clModel.c, 3);
  Serial.print(F(",DPH_DOMG,"));         Serial.print(clModel.d, 3);
  Serial.print(F(",UPDATES,"));          Serial.println(clModel.updates);
}

void saveCurrentJacobian() {
  writeSavedJacobian(clModel.a, clModel.b, clModel.c, clModel.d);
}

bool loadSavedJacobian() {
  SavedJacobian j;
  if (!readSavedJacobian(j)) return false;
  clModel.a = j.a; clModel.b = j.b; clModel.c = j.c; clModel.d = j.d;
  clModel.valid = true; clModel.updates = 0;
  return true;
}

void sendJacobianQuery() {
  Serial.print(F("CAL,JAC,SAVED,"));
  SavedJacobian j;
  Serial.print(readSavedJacobian(j) ? 1 : 0);
  Serial.print(F(","));
  printModel();
}

// Predict one normal correction from the inverse local Jacobian.  The phi error
// is the shortest signed circular difference; alternate physical routes are
// handled later by empirical phi probes and, if needed, one branch transfer.
bool predictCorrection(const HallOrientation &o, float targetTheta, float targetPhi,
                       bool phiNeeded, float score, float &dPsi, float &dOmega) {
  const float eTheta = targetTheta - o.thetaDeg;
  const bool poleMode = !phiNeeded || !o.phiDefined;

  float damping, maxPsi, maxOmega;
  selectMotionLimits(score, poleMode, damping, maxPsi, maxOmega);

  if (poleMode) {
    dPsi = eTheta / keepNonZeroSlope(clModel.a);
    dOmega = 0.0f;
  } else {
    const float ePhi = signedPhiDifference(targetPhi, o.phiDeg);
    const float det = clModel.a * clModel.d - clModel.b * clModel.c;
    if (fabsf(det) < CL_MIN_DET) return false;

    dPsi   = ( clModel.d * eTheta - clModel.b * ePhi) / det;
    dOmega = (-clModel.c * eTheta + clModel.a * ePhi) / det;
  }

  dPsi *= damping;
  dOmega *= damping;

  // Do not separately clip psi and omega; scale both together.
  scaleCorrectionVector(dPsi, dOmega, maxPsi, maxOmega);

  if (fabsf(dPsi) < 0.20f) dPsi = 0.0f;
  if (fabsf(dOmega) < 0.35f) dOmega = 0.0f;

  return dPsi != 0.0f || dOmega != 0.0f;
}

// Learn from verified motions.  Theta is always learned in the middle region;
// phi entries update only when phi is meaningful before and after the move.
void updateModel(const HallOrientation &before, const HallOrientation &after,
                 float dPsi, float dOmega) {
  float denom = dPsi * dPsi + dOmega * dOmega;
  if (denom < 0.01f) return;

  if (!thetaSafelyAwayFromPole(before.thetaDeg) ||
      !thetaSafelyAwayFromPole(after.thetaDeg)) return;

  float moveMag = sqrtf(denom);
  float learningScale = 1.0f;
  if (moveMag > 16.0f)      learningScale = 0.10f;
  else if (moveMag > 8.0f)  learningScale = 0.20f;
  else if (moveMag > 4.0f)  learningScale = 0.50f;

  float observedTheta = after.thetaDeg - before.thetaDeg;
  float predictedTheta = clModel.a * dPsi + clModel.b * dOmega;
  float k = (CL_MODEL_LEARNING_RATE * learningScale) / denom;

  clModel.a += k * (observedTheta - predictedTheta) * dPsi;
  clModel.b += k * (observedTheta - predictedTheta) * dOmega;
  clModel.a = keepNonZeroSlope(clModel.a);

  bool phiReliable = before.phiDefined && after.phiDefined;
  if (phiReliable) {
    float observedPhi = wrapSigned180(after.phiDeg - before.phiDeg);
    float predictedPhi = clModel.c * dPsi + clModel.d * dOmega;
    clModel.c += k * (observedPhi - predictedPhi) * dPsi;
    clModel.d += k * (observedPhi - predictedPhi) * dOmega;
    clModel.d = keepNonZeroSlope(clModel.d);
  }

  clModel.updates++;
}

// Two post-move reads reduce the chance that one Hall outlier is treated as a
// physical jump.  For this control iteration, use whichever read has the
// lower target score.
bool readPostMoveOrientation(HallOrientation &out, float targetTheta, float targetPhi,
                             bool phiNeeded, float &scoreOut) {
  HallOrientation first, second;
  if (!readStableOrientation(first, CL_FAST_READS)) return false;
  delay(150);
  if (!readStableOrientation(second, CL_FAST_READS)) return false;

  float scoreFirst = closedLoopScore(first, targetTheta, targetPhi, phiNeeded);
  float scoreSecond = closedLoopScore(second, targetTheta, targetPhi, phiNeeded);

  if (scoreSecond < scoreFirst) {
    out = second;
    scoreOut = scoreSecond;
  } else {
    out = first;
    scoreOut = scoreFirst;
  }
  return true;
}

// Test one small omega-only step.  A failed test is rolled back immediately,
// so trying the opposite sign begins from the same known motor position.
VerifiedStepResult runVerifiedOmegaProbe(const HallOrientation &before,
                                         float targetTheta, float targetPhi,
                                         bool phiNeeded, float beforeScore,
                                         float dOmega, float rpm,
                                         HallOrientation &afterOut,
                                         float &afterScoreOut) {
  float beforePsi = currentPsiDeg();
  float beforeOmega = currentOmegaDeg();
  float nextOmega = beforeOmega + dOmega;

  if (!closedLoopTargetSafe(beforePsi, nextOmega)) {
    return VERIFIED_STEP_REJECTED;
  }

  Serial.print(F("CL,PHI_SWEEP,TRY,DOMEGA,")); Serial.println(dOmega, 2);

  if (!executeMove(beforePsi, nextOmega, rpm)) {
    pushError(-321, F("Phi sweep move failed"));
    return VERIFIED_STEP_ERROR;
  }
  delay(CL_SETTLE_MS);

  if (!readPostMoveOrientation(afterOut, targetTheta, targetPhi, phiNeeded,
                               afterScoreOut)) {
    pushError(-311, F("Field too small"));
    return VERIFIED_STEP_ERROR;
  }

  if (scoreMeaningfullyImproved(beforeScore, afterScoreOut)) {
    Serial.print(F("CL,PHI_SWEEP,ACCEPT,BEFORE,"));
    Serial.print(beforeScore, 3);
    Serial.print(F(",AFTER,"));
    Serial.println(afterScoreOut, 3);

    // A successful pure-omega probe is high-value local calibration data.
    updateModel(before, afterOut, 0.0f, dOmega);
    return VERIFIED_STEP_ACCEPTED;
  }

  Serial.print(F("CL,PHI_SWEEP,REJECT,BEFORE,"));
  Serial.print(beforeScore, 3);
  Serial.print(F(",AFTER,"));
  Serial.println(afterScoreOut, 3);

  if (!executeMove(beforePsi, beforeOmega, rpm)) {
    pushError(-317, F("Phi sweep rollback failed"));
    return VERIFIED_STEP_ERROR;
  }
  delay(CL_SETTLE_MS);
  return VERIFIED_STEP_REJECTED;
}

// Try the model-preferred omega sign first.  If it does not produce a measured
// improvement, roll back and test the opposite sign.  This lets the controller
// discover that a local response has changed instead of assuming the Jacobian
// direction is always correct.
PhiRecoveryResult runPhiSweepRecovery(const HallOrientation &current,
                                      float targetTheta, float targetPhi,
                                      bool phiNeeded, float currentScore,
                                      float rpm, int &sweepDirection,
                                      int &sweepSteps) {
  if (!phiNeeded || !current.phiDefined) return PHI_RECOVERY_FAILED;

  int preferredDirection = sweepDirection;
  if (preferredDirection == 0) {
    float ePhi = signedPhiDifference(targetPhi, current.phiDeg);
    float phiSlope = keepNonZeroSlope(clModel.d);
    preferredDirection = signOf(ePhi / phiSlope);
  }

  int directions[2];
  directions[0] = preferredDirection;
  directions[1] = -preferredDirection;

  for (int i = 0; i < 2; i++) {
    int dir = directions[i];
    HallOrientation after;
    float afterScore = 0.0f;

    VerifiedStepResult result = runVerifiedOmegaProbe(
      current, targetTheta, targetPhi, phiNeeded, currentScore,
      (float)dir * CL_PHI_SWEEP_STEP_DEG, rpm, after, afterScore);

    if (result == VERIFIED_STEP_ACCEPTED) {
      sweepDirection = dir;
      sweepSteps++;
      return PHI_RECOVERY_ACCEPTED;
    }
    if (result == VERIFIED_STEP_ERROR) return PHI_RECOVERY_ERROR;
  }

  return PHI_RECOVERY_FAILED;
}

bool closedLoopOrient(float targetTheta, float targetPhi, bool targetPhiDefined, float rpm) {
  if (targetTheta < 0.0f || targetTheta > 180.0f || rpm <= 0.0f) {
    pushError(-310, F("Bad theta or RPM"));
    return false;
  }
  if (targetPhiDefined) targetPhi = wrap360(targetPhi);

  // A first relative coarse move gets the stage near the destination.  The
  // Hall-measured correction loop remains responsible for final accuracy.
  if (CL_ENABLE_INITIAL_COARSE) {
    HallOrientation start;
    if (readStableOrientation(start, CL_VERIFY_READS)) {
      bool startPhiNeeded = targetPhiDefined &&
                            thetaSafelyAwayFromPole(targetTheta) &&
                            start.phiDefined &&
                            thetaSafelyAwayFromPole(start.thetaDeg);
      float startScore = closedLoopScore(start, targetTheta, targetPhi,
                                         startPhiNeeded);
      if (startScore > CL_COARSE_START_SCORE) {
        if (!runInitialCoarseApproach(start, targetTheta, targetPhi,
                                      targetPhiDefined, rpm)) {
          Serial.println(F("CL,COARSE_SKIPPED"));
        } else {
          delay(CL_SETTLE_MS);
        }
      }
    }
  }

  float rejectionStepScale = 1.0f;
  int rejectedMoves = 0;
  int stagnantMoves = 0;
  bool omegaTransferUsed = false;
  bool phiSweepActive = false;
  int phiSweepDirection = 0;
  int phiSweepSteps = 0;

  for (int iter = 1; iter <= CL_MAX_ITERATIONS; iter++) {
    HallOrientation current;
    int reads = (iter == 1) ? CL_VERIFY_READS : CL_FAST_READS;
    if (!readStableOrientation(current, reads)) {
      pushError(-311, F("Field too small"));
      return false;
    }

    bool targetAtPole = !thetaSafelyAwayFromPole(targetTheta);
    bool phiReliable = current.phiDefined &&
                       thetaSafelyAwayFromPole(current.thetaDeg);
    bool phiNeeded = targetPhiDefined && !targetAtPole && phiReliable;

    float thetaError = fabsf(current.thetaDeg - targetTheta);
    float phiError = phiNeeded ? phiErrorDeg(current.phiDeg, targetPhi) : 0.0f;
    float score = closedLoopScore(current, targetTheta, targetPhi, phiNeeded);

    Serial.print(F("CL,ITER,"));       Serial.print(iter);
    Serial.print(F(",MODE,"));
    Serial.print(phiSweepActive ? F("PHI_SWEEP") : F("PREDICT"));
    Serial.print(F(",THETA,"));        Serial.print(current.thetaDeg, 2);
    Serial.print(F(",PHI,"));
    if (current.phiDefined) Serial.print(current.phiDeg, 2);
    else Serial.print(F("UNDEFINED"));
    Serial.print(F(",THETA_ERR,"));    Serial.print(thetaError, 2);
    Serial.print(F(",PHI_ERR,"));      Serial.print(phiError, 2);
    Serial.print(F(",SCORE,"));        Serial.println(score, 3);

    if (orientationAtTarget(current, targetTheta, targetPhi, targetPhiDefined)) {
      HallOrientation verified;
      if (readStableOrientation(verified, CL_VERIFY_READS) &&
          orientationAtTarget(verified, targetTheta, targetPhi,
                              targetPhiDefined)) {
        Serial.print(F("CL,FINAL,THETA,"));
        Serial.print(verified.thetaDeg, 2);
        Serial.print(F(",PHI,"));
        if (verified.phiDefined) Serial.print(verified.phiDeg, 2);
        else Serial.print(F("UNDEFINED"));
        Serial.println();
        Serial.println(F("CL,OK"));
        return true;
      }
    }

    // Continue a successful measured omega sweep only while theta remains
    // close enough to target and phi is still appreciably wrong.  Once either
    // condition changes, resume the normal two-axis Jacobian controller with
    // the newly learned local omega slopes.
    if (phiSweepActive &&
        (!phiNeeded ||
         phiError <= CL_PHI_SWEEP_EXIT_PHI_ERR_DEG ||
         thetaError > CL_PHI_SWEEP_MAX_THETA_ERR_DEG ||
         phiSweepSteps >= CL_PHI_SWEEP_MAX_STEPS)) {
      Serial.println(F("CL,PHI_SWEEP,END"));
      phiSweepActive = false;
      phiSweepDirection = 0;
      phiSweepSteps = 0;
    }

    bool recoveryRequested = phiNeeded &&
                             (rejectedMoves >= CL_MAX_REJECTS_BEFORE_RECOVERY ||
                              stagnantMoves >= CL_MAX_STAGNANT_MOVES_BEFORE_RECOVERY);

    // Recovery is empirical: it tries both physical omega directions, accepts
    // only a measured improvement, and learns from the successful direction.
    if (phiNeeded && (phiSweepActive || recoveryRequested) &&
        thetaError <= CL_PHI_SWEEP_MAX_THETA_ERR_DEG) {
      if (!phiSweepActive) {
        phiSweepActive = true;
        phiSweepDirection = 0;
        phiSweepSteps = 0;
        Serial.println(F("CL,PHI_SWEEP,START"));
      }

      PhiRecoveryResult recovery = runPhiSweepRecovery(
        current, targetTheta, targetPhi, phiNeeded, score, rpm,
        phiSweepDirection, phiSweepSteps);

      if (recovery == PHI_RECOVERY_ACCEPTED) {
        rejectionStepScale = 1.0f;
        rejectedMoves = 0;
        stagnantMoves = 0;
        continue;
      }

      if (recovery == PHI_RECOVERY_ERROR) return false;

      // Neither local omega direction helped.  This is the point at which the
      // controller is allowed to test the other physical branch once.
      phiSweepActive = false;
      phiSweepDirection = 0;
      phiSweepSteps = 0;

      if (CL_ENABLE_BRANCH_TRANSFER_AFTER_SWEEP_FAILURE &&
          !omegaTransferUsed) {
        float curPsi = currentPsiDeg();
        float curOmega = currentOmegaDeg();

        Serial.println(F("CL,PHI_SWEEP,NO_LOCAL_PATH"));
        if (!runSafeOmegaTransfer(curPsi, curOmega, rpm)) return false;

        omegaTransferUsed = true;
        rejectionStepScale = 1.0f;
        rejectedMoves = 0;
        stagnantMoves = 0;
        continue;
      }

      pushError(-318, F("Phi path stalled"));
      Serial.println(F("CL,STALLED,PHI_PATH"));
      return false;
    }

    // Theta-only requests cannot use phi sweeps or branch transfers.
    if (!targetPhiDefined &&
        (rejectedMoves >= CL_MAX_REJECTS_BEFORE_RECOVERY ||
         stagnantMoves >= CL_MAX_STAGNANT_MOVES_BEFORE_RECOVERY)) {
      pushError(-319, F("Theta path stalled"));
      Serial.println(F("CL,STALLED,THETA_PATH"));
      return false;
    }

    float dPsi = 0.0f, dOmega = 0.0f;
    float curPsi = currentPsiDeg();
    float curOmega = currentOmegaDeg();
    bool predicted = predictCorrection(current, targetTheta, targetPhi,
                                       phiNeeded, score, dPsi, dOmega);

    if (!predicted) {
      pushError(-316, F("Correction became too small"));
      Serial.println(F("CL,STALLED"));
      return false;
    }

    // A normal prediction that would exceed the protected omega workspace can
    // switch to the opposite branch once, but only if PSI itself remains safe.
    float proposedPsi = curPsi + dPsi;
    float proposedOmega = curOmega + dOmega;
    bool psiSafe = closedLoopPsiSafe(proposedPsi);
    bool omegaSafe = closedLoopOmegaSafe(proposedOmega);
    bool pointsOmegaOutward = (curOmega > 0.0f && dOmega > 0.0f) ||
                              (curOmega < 0.0f && dOmega < 0.0f);

    if (!psiSafe || !omegaSafe) {
      bool canTransfer = targetPhiDefined && phiNeeded &&
                         !omegaTransferUsed &&
                         psiSafe && !omegaSafe && pointsOmegaOutward;

      if (canTransfer) {
        Serial.println(F("CL,OMEGA_TRANSFER,EDGE"));
        if (!runSafeOmegaTransfer(curPsi, curOmega, rpm)) return false;

        omegaTransferUsed = true;
        rejectionStepScale = 1.0f;
        rejectedMoves = 0;
        stagnantMoves = 0;
        phiSweepActive = false;
        continue;
      }

      pushError(-312, F("Target blocked by travel limit"));
      Serial.println(F("CL,STALLED,TRAVEL_LIMIT"));
      return false;
    }

    // After a rejected move, retry a smaller version of the same vector.
    dPsi *= rejectionStepScale;
    dOmega *= rejectionStepScale;

    if (fabsf(dPsi) < 0.15f) dPsi = 0.0f;
    if (fabsf(dOmega) < 0.25f) dOmega = 0.0f;
    if (dPsi == 0.0f && dOmega == 0.0f) {
      pushError(-316, F("Correction became too small"));
      Serial.println(F("CL,STALLED"));
      return false;
    }

    float beforePsi = currentPsiDeg();
    float beforeOmega = currentOmegaDeg();
    float nextPsi = beforePsi + dPsi;
    float nextOmega = beforeOmega + dOmega;

    // The reduced retry is always inside the workspace if the full predicted
    // vector was inside, but retain the check for safety.
    if (!closedLoopTargetSafe(nextPsi, nextOmega)) {
      pushError(-312, F("Retry blocked by travel limit"));
      Serial.println(F("CL,STALLED,TRAVEL_LIMIT"));
      return false;
    }

    Serial.print(F("CL,PREDICT,DPSI,")); Serial.print(dPsi, 2);
    Serial.print(F(",DOMEGA,"));         Serial.print(dOmega, 2);
    Serial.print(F(",PSI,"));            Serial.print(nextPsi, 2);
    Serial.print(F(",OMEGA,"));          Serial.println(nextOmega, 2);

    if (!executeMove(nextPsi, nextOmega, rpm)) {
      pushError(-314, F("Predicted move failed"));
      return false;
    }
    delay(CL_SETTLE_MS);

    HallOrientation after;
    float afterScore = 0.0f;
    if (!readPostMoveOrientation(after, targetTheta, targetPhi, phiNeeded,
                                 afterScore)) {
      pushError(-311, F("Field too small"));
      return false;
    }

    // Reject a truly worse move, return to the known better motor position,
    // reduce the next vector, and do not learn from the rejected attempt.
    if (afterScore > score) {
      Serial.print(F("CL,REJECT,WORSE_SCORE,BEFORE,"));
      Serial.print(score, 3);
      Serial.print(F(",AFTER,"));
      Serial.println(afterScore, 3);

      if (!executeMove(beforePsi, beforeOmega, rpm)) {
        pushError(-317, F("Rollback failed"));
        return false;
      }
      delay(CL_SETTLE_MS);

      rejectionStepScale *= 0.5f;
      if (rejectionStepScale < 0.20f) rejectionStepScale = 0.20f;
      rejectedMoves++;
      stagnantMoves++;

      if (rejectedMoves >= CL_MAX_REJECTS_BEFORE_RECOVERY) {
        Serial.println(F("CL,MODEL_HOLD,REPEATED_REJECTS"));
      }
      continue;
    }

    // Good moves learn.  Meaningful improvement clears the recovery counters;
    // a near-flat result stays accepted but counts toward a later empirical
    // recovery rather than allowing endless barely-changing corrections.
    bool meaningful = scoreMeaningfullyImproved(score, afterScore);
    updateModel(current, after, dPsi, dOmega);

    if (meaningful) {
      rejectionStepScale = min(1.0f, rejectionStepScale * 1.25f);
      rejectedMoves = 0;
      stagnantMoves = 0;
    } else {
      stagnantMoves++;
      rejectionStepScale = min(1.0f, rejectionStepScale * 1.10f);
      Serial.println(F("CL,MODEL_HOLD,LOW_PROGRESS"));
    }
  }

  pushError(-313, F("Closed loop timeout"));
  Serial.println(F("CL,TIMEOUT"));
  return false;
}


// -------------------- EEPROM Restore --------------------
void restorePositionFromEEPROM() {
  SavedState s;
  if (!readSavedState(s)) {
    currentStepsX = 0L;
    currentStepsY = 0L;
    startupStateLoaded         = false;
    startupStateIncompleteMove = false;
    return;
  }
  currentStepsX              = s.stepsX;
  currentStepsY              = s.stepsY;
  startupStateLoaded         = true;
  startupStateIncompleteMove = (s.inMotion != 0);
}

// -------------------- Parsing --------------------
static bool ciEq(const char *a, const char *b) {
  while (*a && *b) {
    if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false;
    a++; b++;
  }
  return (*a == '\0' && *b == '\0');
}

static bool ciStartsWith(const char *str, const char *prefix) {
  while (*prefix) {
    if (tolower((unsigned char)*str) != tolower((unsigned char)*prefix)) return false;
    str++; prefix++;
  }
  return true;
}

static bool parseThreeFloats(const char *s, float &a, float &b, float &c) {
  char *end;
  a = (float)strtod(s, &end);
  if (end == s || *end != ',') return false;
  s = end + 1;
  b = (float)strtod(s, &end);
  if (end == s || *end != ',') return false;
  s = end + 1;
  c = (float)strtod(s, &end);
  if (end == s) return false;
  while (*end == ' ') end++;
  return (*end == '\0');
}

// -------------------- SCPI Command Handler --------------------
void handleSCPI(char *line) {
  // ---- Identification / Reset ----
  if (ciEq(line, "*IDN?")) {
    Serial.println(F("WashU,Goniometer,ThetaPhiClosedLoop,1.4"));
    return;
  }

  if (ciEq(line, "*RST")) {
    currentStepsX = 0L;
    currentStepsY = 0L;
    saveCurrentPositionConfirmed();
    startupStateIncompleteMove = false;
    Serial.println(F("OK"));
    return;
  }

  // ---- Error Queue ----
  if (ciEq(line, "SYST:ERR?")) {
    popErrorToSerial();
    return;
  }

  // ---- Status Queries ----
  if (ciEq(line, "STAT:POS?") || ciEq(line, "MEAS:POS?")) {
    sendPositionQuery(); return;
  }
  if (ciEq(line, "STAT:STEP?")) { sendStepQuery();    return; }
  if (ciEq(line, "STAT:SWIT?")) { sendSwitchQuery();  return; }
  if (ciEq(line, "STAT:EEPR?")) { sendEEPROMQuery();  return; }
  if (ciEq(line, "CONF:GZER?")) { sendGoZeroRPMQuery(); return; }
  if (ciEq(line, "CONF:HOM?"))  { sendHomingRPMQuery(); return; }

  // ---- Persistent Jacobian calibration ----
  if (ciEq(line, "CAL:JAC?")) { sendJacobianQuery(); return; }
  if (ciEq(line, "CAL:JAC:SAVE")) { saveCurrentJacobian(); Serial.println(F("OK")); return; }
  if (ciEq(line, "CAL:JAC:LOAD")) {
    if (loadSavedJacobian()) { Serial.println(F("OK")); printModel(); }
    else { pushError(-318, F("No saved Jacobian")); Serial.println(F("ERR")); }
    return;
  }
  if (ciEq(line, "CAL:JAC:CLEAR")) { clearSavedJacobian(); clModel = CL_MODEL_DEFAULT; Serial.println(F("OK")); return; }
  if (ciEq(line, "CAL:JAC:DEFAULT")) { clModel = CL_MODEL_DEFAULT; Serial.println(F("OK")); return; }

  // ---- Hall Queries ----
  if (ciEq(line, "HALL?") || ciEq(line, "MEAS:FIELD?") || ciEq(line, "HALL:PSIOMEGA?")) {
    sendHallData(); return;
  }

  // ---- Go to Zero ----
  if (ciEq(line, "MOVE:ZERO") || ciEq(line, "MOV:ZER")) {
    if (!executeMove(0.0f, 0.0f, goZeroRPM)) { Serial.println(F("ERR")); return; }
    Serial.println(F("OK"));
    return;
  }

  // ---- Homing ----
  if (ciEq(line, "HOME:OMEGA") || ciEq(line, "HOME:OME")) {
    if (homeSwitchPressedDebounced()) {
      pushError(-230, F("Switch already pressed"));
      Serial.println(F("ERR")); return;
    }
    executeHomeOmega(-1);
    Serial.println(F("OK")); return;
  }

  if (ciEq(line, "HOME:PSI")) {
    if (homeSwitchPressedDebounced()) {
      pushError(-231, F("Switch already pressed"));
      Serial.println(F("ERR")); return;
    }
    executeHomePsi(-1);
    Serial.println(F("OK")); return;
  }

  // ---- Memory Clear ----
  if (ciEq(line, "MEM:CLEAR") || ciEq(line, "MEM:CLE")) {
    writeSavedState(0L, 0L, true, false);
    currentStepsX = 0L;
    currentStepsY = 0L;
    startupStateLoaded         = true;
    startupStateIncompleteMove = false;
    Serial.println(F("OK")); return;
  }

  // ---- MOVE:RAW <psi>,<omega>,<rpm> ----
  // Physical, unwrapped stage coordinates in degrees.  Unlike MOVE:ABS, this
  // command does NOT convert 0..360 values into -180..180.  Use it only when
  // you intentionally need to choose the cable-twist branch, for example:
  // MOVE:RAW -180,180,2
  const char *moveRawArgs = nullptr;
  if (ciStartsWith(line, "MOVE:RAW ")) moveRawArgs = line + 9;
  else if (ciStartsWith(line, "MOV:RAW ")) moveRawArgs = line + 8;

  if (moveRawArgs) {
    while (*moveRawArgs == ' ') moveRawArgs++;
    if (*moveRawArgs == '\0') {
      pushError(-202, F("Missing MOVE:RAW arguments"));
      Serial.println(F("ERR")); return;
    }

    float psiDegRaw, omegaDegRaw, rpm;
    if (!parseThreeFloats(moveRawArgs, psiDegRaw, omegaDegRaw, rpm)) {
      pushError(-203, F("Bad MOVE:RAW format"));
      Serial.println(F("ERR")); return;
    }

    if (!executeMove(psiDegRaw, omegaDegRaw, rpm)) {
      Serial.println(F("ERR")); return;
    }
    Serial.println(F("OK")); return;
  }

  // ---- MOVE:ABS <psi>,<omega>,<rpm> ----
  const char *moveAbsArgs = nullptr;
  if (ciStartsWith(line, "MOVE:ABS ")) moveAbsArgs = line + 9;
  else if (ciStartsWith(line, "MOV:ABS "))  moveAbsArgs = line + 8;

  if (moveAbsArgs) {
    while (*moveAbsArgs == ' ') moveAbsArgs++;
    if (*moveAbsArgs == '\0') {
      pushError(-200, F("Missing MOVE:ABS arguments"));
      Serial.println(F("ERR")); return;
    }

    float psiDegUser, omegaDegUser, rpm;
    if (!parseThreeFloats(moveAbsArgs, psiDegUser, omegaDegUser, rpm)) {
      pushError(-201, F("Bad MOVE:ABS format"));
      Serial.println(F("ERR")); return;
    }

    if (psiDegUser < 0.0f || psiDegUser > 360.0f) {
      pushError(-222, F("Psi input must be 0 to 360"));
      Serial.println(F("ERR")); return;
    }
    if (omegaDegUser < 0.0f || omegaDegUser > 360.0f) {
      pushError(-223, F("Omega input must be 0 to 360"));
      Serial.println(F("ERR")); return;
    }

    float psiInternal   = user360ToSigned180(psiDegUser);
    float omegaInternal = user360ToSigned180(omegaDegUser);

    if (!executeMove(psiInternal, omegaInternal, rpm)) {
      Serial.println(F("ERR")); return;
    }
    Serial.println(F("OK")); return;
  }

  // ---- MEAS:MOVE <psi>,<omega>,<rpm> ----
  if (ciStartsWith(line, "MEAS:MOVE")) {
    const char *args = line + 9;
    while (*args == ' ') args++;
    if (*args == '\0') {
      Serial.println(F("BAD PARAMETERS")); return;
    }
    float psi, omega, rpm;
    if (!parseThreeFloats(args, psi, omega, rpm)) {
      Serial.println(F("BAD PARAMETERS")); return;
    }
    // Convert user 0-360 angles to internal -180..180 before passing to executeMove.
    float psiInternal   = user360ToSigned180(psi);
    float omegaInternal = user360ToSigned180(omega);
    moveAndMeasure(psiInternal, omegaInternal, rpm);
    return;
  }

  // ---- CL:ORIENT <theta>,<phi>,<rpm> ----
  // User directly commands desired field-relative sample orientation.
  if (ciStartsWith(line, "CL:ORIENT") || ciStartsWith(line, "CLO:ORI")) {
    const char *args = ciStartsWith(line, "CL:ORIENT") ? line + 9 : line + 7;
    while (*args == ' ') args++;
    // Accepted formats:
    //   CL:ORIENT <theta>,<phi>,<rpm>     e.g. CL:ORIENT 115,27,2
    //   CL:ORIENT <theta>,UNDEF,<rpm>    e.g. CL:ORIENT 0,UNDEF,2
    // UNDEF means PHI is intentionally ignored; use it near THETA = 0 or 180.
    char argBuf[64];
    strncpy(argBuf, args, sizeof(argBuf) - 1);
    argBuf[sizeof(argBuf) - 1] = '\0';

    char *thetaText = strtok(argBuf, ",");
    char *phiText   = strtok(NULL, ",");
    char *rpmText   = strtok(NULL, ",");
    char *extraText = strtok(NULL, ",");
    if (!thetaText || !phiText || !rpmText || extraText) {
      pushError(-314, F("Bad CL:ORIENT format"));
      Serial.println(F("ERR")); return;
    }
    while (*thetaText == ' ') thetaText++;
    while (*phiText == ' ') phiText++;
    while (*rpmText == ' ') rpmText++;

    float theta = atof(thetaText);
    float rpm = atof(rpmText);
    bool targetPhiDefined = true;
    float phi = 0.0f;
    if (strcasecmp(phiText, "UNDEF") == 0 || strcasecmp(phiText, "UNDEFINED") == 0) {
      targetPhiDefined = false;
    } else {
      phi = atof(phiText);
      if (phi < 0.0f || phi > 360.0f) {
        pushError(-315, F("Phi input must be 0 to 360 or UNDEF"));
        Serial.println(F("ERR")); return;
      }
    }
    if (!closedLoopOrient(theta, phi, targetPhiDefined, rpm)) {
      Serial.println(F("ERR")); return;
    }
    return;
  }

  // ---- Unknown ----
  pushError(-113, F("Undefined header"));
  Serial.println(F("ERR"));
}

// -------------------- Serial Input --------------------
void handleSerialInput() {
  static char lineBuf[64];
  static uint8_t pos = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (pos > 0) {
        lineBuf[pos] = '\0';
        while (pos > 0 && (lineBuf[pos-1] == ' ' || lineBuf[pos-1] == '\t'))
          lineBuf[--pos] = '\0';
        handleSCPI(lineBuf);
        pos = 0;
      }
    } else {
      if (pos < sizeof(lineBuf) - 1)
        lineBuf[pos++] = c;
    }
  }
}

// -------------------- Setup / Loop --------------------
void setup() {
  pinMode(StepX, OUTPUT);
  pinMode(DirX,  OUTPUT);
  pinMode(StepY, OUTPUT);
  pinMode(DirY,  OUTPUT);
  pinMode(homeSwitchPin, INPUT_PULLUP);

  digitalWrite(StepX, LOW);
  digitalWrite(StepY, LOW);

  Serial.begin(115200);
  delay(300);

  restorePositionFromEEPROM();
  if (loadSavedJacobian()) Serial.println(F("CAL,JAC,LOADED"));

  Serial.println(F("Goniometer ready."));
  Serial.println(F("Travel: full +/-180 working range; PSI/OMEGA hard cable limit +/-200 deg."));
  Serial.println(F("CL recovery: measured +/- omega sweep; one alternate branch transfer."));
  Serial.println(F("CL:ORIENT theta,phi,rpm | MOVE:RAW psi,omega,rpm | HALL? | STAT:POS?"));
}

void loop() {
  handleSerialInput();
}