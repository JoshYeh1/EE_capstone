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

const int NUM_SAMPLES = 100;

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

struct HallPsiOmega {
  float psiDeg;
  float omegaDeg;
  float magnitude;
  bool valid;
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

// These must be declared near the top because the Arduino IDE auto-generates
// function prototypes before later code sections.
struct SearchMove {
  float dPsi;
  float dOmega;
};

struct ClosedLoopMemory {
  float lastDPsi;
  float lastDOmega;
  int noImproveCount;
};

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
    delay(5);
  }
  HallData result;
  result.x = ((float)xSum / NUM_SAMPLES) - X_OFFSET;
  result.y = ((float)ySum / NUM_SAMPLES) - Y_OFFSET;
  result.z = (((float)zSum / NUM_SAMPLES) - Z_OFFSET) * 0.94f; //CHANGED. ADDED *0.94

  // Small Hall sensor mounting-tilt correction: rotate X/Z about Y axis
  const float TILT_DEG = 4.0f;
  const float TILT_RAD = TILT_DEG * PI / 180.0f;

  float xOld = result.x;
  float zOld = result.z;

  result.x = xOld * cos(TILT_RAD) + zOld * sin(TILT_RAD);
  result.z = -xOld * sin(TILT_RAD) + zOld * cos(TILT_RAD);
  //END ADD
  return result;
}

// -------------------- Hall Vector -> Psi/Omega (No Lookup Table) --------------------
const float HALL_X_SCALE = 1.0f;
const float HALL_Y_SCALE = 1.0f;
const float HALL_Z_SCALE = 1.0f;

const float ZERO_REF_X = -11.170f;
const float ZERO_REF_Y = 4.670f;
const float ZERO_REF_Z = 26.870f;
// NOTE: ZERO_REF is no longer used for THETA/PHI. It remains only for old
// helper code retained in this file; it can be removed during later cleanup.

const float HALL_PSI_SIGN   = 1.0f;
const float HALL_OMEGA_SIGN = 1.0f;

float vectorMagnitude(const FieldVector &v) {
  return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

FieldVector normalizeVector(const FieldVector &v) {
  float m = vectorMagnitude(v);
  if (m < 0.001f) return {0.0f, 0.0f, 0.0f};
  return {v.x / m, v.y / m, v.z / m};
}

FieldVector crossProduct(const FieldVector &a, const FieldVector &b) {
  return {
    a.y*b.z - a.z*b.y,
    a.z*b.x - a.x*b.z,
    a.x*b.y - a.y*b.x
  };
}

float dotProduct(const FieldVector &a, const FieldVector &b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

FieldVector alignToZeroReference(const FieldVector &v) {
  FieldVector u = normalizeVector({ZERO_REF_X, ZERO_REF_Y, ZERO_REF_Z});
  FieldVector target = {0.0f, 0.0f, 1.0f};
  float d = dotProduct(u, target);
  if (d > 0.999999f) return v;
  if (d < -0.999999f) return {v.x, -v.y, -v.z};

  FieldVector axis = crossProduct(u, target);
  float s = vectorMagnitude(axis);
  axis.x /= s; axis.y /= s; axis.z /= s;

  FieldVector axv = crossProduct(axis, v);
  float adv = dotProduct(axis, v);
  return {
    v.x*d + axv.x*s + axis.x*adv*(1.0f-d),
    v.y*d + axv.y*s + axis.y*adv*(1.0f-d),
    v.z*d + axv.z*s + axis.z*adv*(1.0f-d)
  };
}

float angularDifferenceDeg(float a, float b) {
  return fabsf(wrapSigned180(a - b));
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

  // Do NOT rotate by ZERO_REF here. ZERO_REF would redefine THETA = 0.
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

// Combined orientation error used only by the closed-loop search.
// Phi is naturally ignored near theta = 0 or 180 because the in-plane direction is undefined there.
float orientationError(const HallOrientation &o, float targetThetaDeg, float targetPhiDeg) {
  float thetaError = fabsf(o.thetaDeg - targetThetaDeg);
  if (!o.phiDefined || targetThetaDeg < PHI_UNDEFINED_THETA_DEG ||
      targetThetaDeg > (180.0f - PHI_UNDEFINED_THETA_DEG)) {
    return thetaError;
  }
  float phiWeight = sinf(targetThetaDeg * PI / 180.0f);
  return thetaError + phiWeight * phiErrorDeg(o.phiDeg, targetPhiDeg);
}

HallPsiOmega calculateHallPsiOmega(const FieldVector &rawB) {
  HallPsiOmega out;
  out.magnitude = vectorMagnitude(rawB);
  out.psiDeg = 0.0f;
  out.omegaDeg = 0.0f;
  out.valid = false;
  if (out.magnitude < 0.001f) return out;

  FieldVector unitB = normalizeVector(rawB);
  FieldVector b = alignToZeroReference(unitB);

  float psi1 = atan2f(b.x, sqrtf(b.y*b.y + b.z*b.z)) * 180.0f / PI;
  float omega1 = atan2f(b.y, b.z) * 180.0f / PI;
  psi1 *= HALL_PSI_SIGN;
  omega1 *= HALL_OMEGA_SIGN;
  psi1 = wrapSigned180(psi1);
  omega1 = wrapSigned180(omega1);

  float psi2 = (psi1 >= 0.0f) ? (180.0f - psi1) : (-180.0f - psi1);
  float omega2 = wrapSigned180(omega1 + 180.0f);

  float motorPsi = currentPsiDeg();
  float motorOmega = currentOmegaDeg();
  float score1 = angularDifferenceDeg(psi1, motorPsi) + angularDifferenceDeg(omega1, motorOmega);
  float score2 = angularDifferenceDeg(psi2, motorPsi) + angularDifferenceDeg(omega2, motorOmega);

  if (score2 < score1) {
    out.psiDeg = psi2;
    out.omegaDeg = omega2;
  } else {
    out.psiDeg = psi1;
    out.omegaDeg = omega1;
  }
  out.valid = true;
  return out;
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

// -------------------- Angle Limits --------------------
const float MIN_ANGLE_DEG = -180.0f;
const float MAX_ANGLE_DEG =  180.0f;

// -------------------- Position Memory --------------------
long currentStepsY = 0L;
long currentStepsX = 0L;

// -------------------- EEPROM --------------------
const uint32_t EEPROM_MAGIC = 0x47A10E55UL;
const int      EEPROM_ADDR  = 0;

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

// -------------------- SCPI Error Queue --------------------
const int ERROR_QUEUE_SIZE  = 8;
const int ERROR_ENTRY_LEN   = 48;

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
  // HALL_PSI/HALL_OMEGA intentionally omitted: they are not real motor angles.
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

bool inRange(float deg) {
  return (deg >= MIN_ANGLE_DEG && deg <= MAX_ANGLE_DEG);
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
  if (!inRange(targetPsiDeg)) {
    pushError(-222, F("Psi out of range"));
    return false;
  }
  if (!inRange(targetOmegaDeg)) {
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

// -------------------- Closed Loop: target Theta/Phi --------------------
// Improved local-search controller:
// - scores the current position as a baseline
// - averages multiple Hall orientations at each tested point
// - uses large steps far away and small steps near target
// - requires a meaningful score improvement before moving
// - discourages immediate reversals (helps backlash / belt slack)
// - uses 4-point cross search first; tries diagonals only when needed

const float CL_THETA_TOL_DEG = 2.0f;
const float CL_PHI_TOL_DEG = 5.0f;
const float CL_THETA_STAGE2_BAND_DEG = 5.0f;
const float CL_THETA_HOLD_WEIGHT = 4.0f;

const int CL_MAX_ITERATIONS = 20;
const unsigned long CL_SETTLE_MS = 450UL;
const int CL_ORIENTATION_READS = 5;       // Each read itself averages ADC samples.
const float CL_MIN_IMPROVEMENT_FRAC = 0.03f; // Candidate must improve score by 8%.
const float CL_MIN_IMPROVEMENT_ABS = 0.50f;  // Or at least 0.20 degree-score.
const float CL_REVERSAL_EXTRA_FRAC = 0.15f;  // Extra improvement required to reverse.
const float CL_APPROACH_MARGIN_DEG = 3.0f;
const int CL_STUCK_LIMIT = 2;


bool orientationAtTarget(const HallOrientation &o, float targetThetaDeg,
                         float targetPhiDeg, bool targetPhiDefined) {
  if (fabsf(o.thetaDeg - targetThetaDeg) > CL_THETA_TOL_DEG) return false;
  if (!targetPhiDefined || targetThetaDeg < PHI_UNDEFINED_THETA_DEG ||
      targetThetaDeg > 180.0f - PHI_UNDEFINED_THETA_DEG) return true;
  return o.phiDefined && (phiErrorDeg(o.phiDeg, targetPhiDeg) <= CL_PHI_TOL_DEG);
}

// Average several fully computed orientations. Phi is averaged as a circular quantity.
bool readStableOrientation(HallOrientation &out) {
  float thetaSum = 0.0f;
  float magSum = 0.0f;
  float phiSinSum = 0.0f;
  float phiCosSum = 0.0f;
  int validCount = 0;
  int phiCount = 0;

  for (int i = 0; i < CL_ORIENTATION_READS; i++) {
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
    if (i < CL_ORIENTATION_READS - 1) delay(20);
  }

  if (validCount == 0) return false;
  out.thetaDeg = thetaSum / validCount;
  out.magnitude = magSum / validCount;
  out.valid = true;
  out.phiDefined = (phiCount >= (CL_ORIENTATION_READS + 1) / 2);
  out.phiDeg = 0.0f;
  if (out.phiDefined) {
    out.phiDeg = atan2f(phiSinSum, phiCosSum) * 180.0f / PI;
    if (out.phiDeg < 0.0f) out.phiDeg += 360.0f;
  }
  return true;
}

float closedLoopScore(const HallOrientation &o, float targetTheta, float targetPhi,
                      bool thetaStage) {
  float thetaError = fabsf(o.thetaDeg - targetTheta);
  if (thetaStage || !o.phiDefined) return thetaError;
  float phiError = phiErrorDeg(o.phiDeg, targetPhi);
  return phiError + CL_THETA_HOLD_WEIGHT * thetaError;
}

// Choose separate adaptive psi / omega test sizes from the current error.
void chooseSearchSteps(float thetaError, float phiError, bool thetaStage,
                       float &psiStep, float &omegaStep) {

  // First stage: primarily get THETA close.
  if (thetaStage) {
    if (thetaError > 20.0f) {
      psiStep = 5.0f;
      omegaStep = 8.0f;
    }
    else if (thetaError > 7.0f) {
      psiStep = 2.0f;
      omegaStep = 4.0f;
    }
    else {
      psiStep = 0.5f;
      omegaStep = 1.0f;
    }
  }

  // Second stage: THETA is close, now move toward PHI target.
  else {
    if (phiError > 60.0f) {
      psiStep = 4.0f;
      omegaStep = 15.0f;
    }
    else if (phiError > 25.0f) {
      psiStep = 2.0f;
      omegaStep = 8.0f;
    }
    else if (phiError > 10.0f) {
      psiStep = 1.0f;
      omegaStep = 4.0f;
    }
    else {
      psiStep = 0.5f;
      omegaStep = 1.5f;
    }
  }
}

bool isImmediateReversal(const SearchMove &m, const ClosedLoopMemory &mem) {
  bool psiReverse = (m.dPsi != 0.0f && mem.lastDPsi != 0.0f && m.dPsi * mem.lastDPsi < 0.0f);
  bool omegaReverse = (m.dOmega != 0.0f && mem.lastDOmega != 0.0f && m.dOmega * mem.lastDOmega < 0.0f);
  return psiReverse || omegaReverse;
}

bool hasMeaningfulImprovement(float currentScore, float candidateScore, bool reversal) {
  float required = max(CL_MIN_IMPROVEMENT_ABS, currentScore * CL_MIN_IMPROVEMENT_FRAC);
  if (reversal) required += max(CL_MIN_IMPROVEMENT_ABS, currentScore * CL_REVERSAL_EXTRA_FRAC);
  return candidateScore <= (currentScore - required);
}

// Move to target so final approach is consistent. This reduces belt/gear backlash effects.
bool executeMoveWithConsistentApproach(float targetPsi, float targetOmega, float rpm) {
  if (!inRange(targetPsi) || !inRange(targetOmega)) return false;

  float prePsi = targetPsi - CL_APPROACH_MARGIN_DEG;
  float preOmega = targetOmega - CL_APPROACH_MARGIN_DEG;
  if (prePsi < MIN_ANGLE_DEG) prePsi = targetPsi + CL_APPROACH_MARGIN_DEG;
  if (preOmega < MIN_ANGLE_DEG) preOmega = targetOmega + CL_APPROACH_MARGIN_DEG;
  if (!inRange(prePsi)) prePsi = targetPsi;
  if (!inRange(preOmega)) preOmega = targetOmega;

  if (fabsf(currentPsiDeg() - prePsi) > 0.02f ||
      fabsf(currentOmegaDeg() - preOmega) > 0.02f) {
    if (!executeMove(prePsi, preOmega, rpm)) return false;
    delay(CL_SETTLE_MS);
  }
  if (!executeMove(targetPsi, targetOmega, rpm)) return false;
  delay(CL_SETTLE_MS);
  return true;
}

bool tryCandidateFromBase(float basePsi, float baseOmega,
                          float candidatePsi, float candidateOmega,
                          float rpm, float targetTheta, float targetPhi,
                          bool thetaStage, float &scoreOut,
                          HallOrientation &measurementOut) {
  if (!inRange(candidatePsi) || !inRange(candidateOmega)) return false;
  if (!executeMoveWithConsistentApproach(basePsi, baseOmega, rpm)) return false;
  if (!executeMoveWithConsistentApproach(candidatePsi, candidateOmega, rpm)) return false;
  if (!readStableOrientation(measurementOut)) return false;
  scoreOut = closedLoopScore(measurementOut, targetTheta, targetPhi, thetaStage);
  return true;
}

bool closedLoopOrient(float targetTheta, float targetPhi, bool targetPhiDefined, float rpm) {
  if (targetTheta < 0.0f || targetTheta > 180.0f || rpm <= 0.0f) {
    pushError(-310, F("Bad theta or RPM"));
    return false;
  }
  if (targetPhiDefined) targetPhi = wrap360(targetPhi);

  ClosedLoopMemory memory = {0.0f, 0.0f, 0};

  for (int iter = 1; iter <= CL_MAX_ITERATIONS; iter++) {
    HallOrientation current;
    if (!readStableOrientation(current)) {
      pushError(-311, F("Field too small"));
      return false;
    }

    float thetaError = fabsf(current.thetaDeg - targetTheta);
    bool phiNeeded = targetPhiDefined && !(targetTheta < PHI_UNDEFINED_THETA_DEG ||
                                           targetTheta > 180.0f - PHI_UNDEFINED_THETA_DEG);
    bool thetaStage = (thetaError > CL_THETA_STAGE2_BAND_DEG) || !phiNeeded || !current.phiDefined;
    float currentScore = closedLoopScore(current, targetTheta, targetPhi, thetaStage);

    Serial.print(F("CL,ITER,")); Serial.print(iter);
    Serial.print(F(",STAGE,")); Serial.print(thetaStage ? F("THETA") : F("PHI"));
    Serial.print(F(",THETA,")); Serial.print(current.thetaDeg, 2);
    Serial.print(F(",PHI,"));
    if (current.phiDefined) Serial.print(current.phiDeg, 2); else Serial.print(F("UNDEFINED"));
    Serial.print(F(",SCORE,")); Serial.println(currentScore, 3);

    if (orientationAtTarget(current, targetTheta, targetPhi, targetPhiDefined)) {
      Serial.println(F("CL,OK"));
      return true;
    }

    float psiStep, omegaStep;
    float phiError = targetPhiDefined
      ? phiErrorDeg(current.phiDeg, targetPhi)
      : 0.0f;

    chooseSearchSteps(thetaError, phiError, thetaStage, psiStep, omegaStep);
    float basePsi = currentPsiDeg();
    float baseOmega = currentOmegaDeg();

    // Cross search: four nearby candidates. Diagonals are tried only if no cross move wins.
    SearchMove cross[4] = {
      { psiStep, 0.0f }, { -psiStep, 0.0f },
      { 0.0f, omegaStep }, { 0.0f, -omegaStep }
    };
    SearchMove diagonals[4] = {
      { psiStep, omegaStep }, { psiStep, -omegaStep },
      { -psiStep, omegaStep }, { -psiStep, -omegaStep }
    };

    SearchMove bestMove = {0.0f, 0.0f};
    float bestScore = currentScore;
    bool foundImprovement = false;

    for (int pass = 0; pass < 2 && !foundImprovement; pass++) {
      SearchMove *moves = (pass == 0) ? cross : diagonals;
      int moveCount = 4;
      for (int i = 0; i < moveCount; i++) {
        float cPsi = basePsi + moves[i].dPsi;
        float cOmega = baseOmega + moves[i].dOmega;
        HallOrientation tested;
        float candidateScore;
        if (!tryCandidateFromBase(basePsi, baseOmega, cPsi, cOmega, rpm,
                                  targetTheta, targetPhi, thetaStage,
                                  candidateScore, tested)) continue;

        bool reversal = isImmediateReversal(moves[i], memory);
        Serial.print(F("CL,TRY,PSI,")); Serial.print(cPsi, 2);
        Serial.print(F(",OMEGA,")); Serial.print(cOmega, 2);
        Serial.print(F(",THETA,")); Serial.print(tested.thetaDeg, 2);
        Serial.print(F(",PHI,"));
        if (tested.phiDefined) Serial.print(tested.phiDeg, 2); else Serial.print(F("UNDEFINED"));
        Serial.print(F(",SCORE,")); Serial.print(candidateScore, 3);
        Serial.print(F(",REV,")); Serial.println(reversal ? 1 : 0);

        if (candidateScore < bestScore &&
            hasMeaningfulImprovement(currentScore, candidateScore, reversal)) {
          bestScore = candidateScore;
          bestMove = moves[i];
          foundImprovement = true;
        }
      }
    }

    if (!foundImprovement) {
      // Return to baseline before deciding whether to shrink step size / stop.
      if (!executeMoveWithConsistentApproach(basePsi, baseOmega, rpm)) return false;
      memory.noImproveCount++;
      Serial.print(F("CL,NO_IMPROVE,")); Serial.println(memory.noImproveCount);
      if (memory.noImproveCount >= CL_STUCK_LIMIT) {
        pushError(-312, F("No meaningful improvement"));
        Serial.println(F("CL,STALLED"));
        return false;
      }
      continue;
    }

    memory.noImproveCount = 0;
    float targetPsi = basePsi + bestMove.dPsi;
    float targetOmega = baseOmega + bestMove.dOmega;
    if (!executeMoveWithConsistentApproach(targetPsi, targetOmega, rpm)) return false;
    memory.lastDPsi = bestMove.dPsi;
    memory.lastDOmega = bestMove.dOmega;

    Serial.print(F("CL,ACCEPT,PSI,")); Serial.print(targetPsi, 2);
    Serial.print(F(",OMEGA,")); Serial.print(targetOmega, 2);
    Serial.print(F(",SCORE,")); Serial.println(bestScore, 3);
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
    Serial.println(F("WashU,Goniometer,ThetaPhiClosedLoop,1.2"));
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
    char argBuf[80];
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
  static char lineBuf[80];
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

  Serial.println(F("Goniometer SCPI ready."));
  Serial.println(F("User input angles for MOVE:ABS and MEAS:MOVE are 0 to 360 degrees."));
  Serial.println(F("Internal motion logic remains -180 to 180 degrees."));
  Serial.println(F("Examples:"));
  Serial.println(F("*IDN?"));
  Serial.println(F("STAT:POS?"));
  Serial.println(F("STAT:STEP?"));
  Serial.println(F("STAT:SWIT?"));
  Serial.println(F("STAT:EEPR?"));
  Serial.println(F("CONF:GZER?"));
  Serial.println(F("CONF:HOM?"));
  Serial.println(F("MEAS:FIELD?"));
  Serial.println(F("HALL?"));
  Serial.println(F("MOVE:ABS 270,350,5"));
  Serial.println(F("MEAS:MOVE 270,350,5"));
  Serial.println(F("CL:ORIENT 45,90,3   (target theta,phi,rpm)"));
  Serial.println(F("CL:ORIENT 0,UNDEF,3 (theta-only; ignore phi)"));
  Serial.println(F("MOVE:ZERO"));
  Serial.println(F("HOME:OMEGA"));
  Serial.println(F("HOME:PSI"));
  Serial.println(F("MEM:CLEAR"));
  Serial.println(F("SYST:ERR?"));
}

void loop() {
  handleSerialInput();
}
