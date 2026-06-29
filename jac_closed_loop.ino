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

// One compact, persistent Jacobian calibration.  This deliberately stores only
// four floats instead of a large grid so it is friendly to small AVR boards.
struct SavedJacobian {
  uint32_t magic;
  float a, b, c, d;
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

// -------------------- Closed Loop: adaptive Jacobian controller --------------------
// The controller uses the current Hall reading plus a local Jacobian to make
// fast large moves when far away and smaller verified moves near the target.

const float CL_THETA_TOL_DEG = 2.0f;
const float CL_PHI_TOL_DEG = 5.0f;
const float CL_THETA_WEIGHT = 2.5f;
const float CL_PHI_WEIGHT = 1.0f;

const int CL_MAX_ITERATIONS = 40;
const unsigned long CL_SETTLE_MS = 450UL;
const int CL_FAST_READS = 1;
const int CL_VERIFY_READS = 3;
const float CL_MIN_IMPROVEMENT_ABS = 0.35f;
const float CL_MIN_IMPROVEMENT_FRAC = 0.015f;

// Jacobian learning is deliberately conservative. Large moves traverse a
// nonlinear mechanism, so they are used for motion but not for model learning.
const float CL_MODEL_LEARNING_RATE = 0.12f;
const float CL_MAX_LEARNING_PSI_DEG = 4.0f;
const float CL_MAX_LEARNING_OMEGA_DEG = 6.0f;
const float CL_MIN_SLOPE_MAG = 0.10f;

// Phi is mathematically ill-conditioned near theta = 0 or 180.
// Do not let a newly-defined, noisy phi angle steer the controller until
// both the target and measured theta are safely away from either pole.
const float CL_PHI_ENABLE_THETA_DEG = 20.0f;

// A failed move reduces the next command instead of repeating the same move.
const int CL_MAX_REJECTS_BEFORE_RESET = 2;
const float CL_MIN_REJECTION_SCALE = 0.20f;

// When the shortest PHI direction would drive OMEGA beyond its wire-protection
// limit, try the other wrapped PHI direction instead. This can be a longer
// physical field path, but it keeps the motors inside their safe range.
const float CL_LIMIT_MARGIN_DEG = 1.0f;

// Limit-aware look-ahead planner. This is used before a motor limit is hit.
// It trades a little extra travel for a path that remains smooth and preserves
// clearance from both wire-protection boundaries.

struct LocalJacobian {
  // [ dTheta/dPsi  dTheta/dOmega ]
  // [ dPhi/dPsi    dPhi/dOmega   ]
  float a, b, c, d;
  bool valid;
  int updates;
};

// Based on the measured behavior in your test: negative PSI increased THETA,
// therefore dTheta/dPsi is negative in this local region.
const LocalJacobian CL_MODEL_DEFAULT = {-0.80f, 0.25f, -0.25f, 0.80f, true, 0};
LocalJacobian clModel = {-0.80f, 0.25f, -0.25f, 0.80f, true, 0};

bool thetaSafelyAwayFromPole(float thetaDeg) {
  return (thetaDeg >= CL_PHI_ENABLE_THETA_DEG &&
          thetaDeg <= 180.0f - CL_PHI_ENABLE_THETA_DEG);
}

bool orientationAtTarget(const HallOrientation &o, float targetThetaDeg,
                         float targetPhiDeg, bool targetPhiDefined) {
  if (fabsf(o.thetaDeg - targetThetaDeg) > CL_THETA_TOL_DEG) return false;

  // For a pole target, phi is not meaningful. For non-pole targets, require
  // a reliable measured phi before declaring success.
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
  return CL_THETA_WEIGHT * thetaError + CL_PHI_WEIGHT * phiErrorDeg(o.phiDeg, targetPhi);
}

float keepNonZeroSlope(float value) {
  if (fabsf(value) >= CL_MIN_SLOPE_MAG) return value;
  return (value < 0.0f) ? -CL_MIN_SLOPE_MAG : CL_MIN_SLOPE_MAG;
}

// Fast feed-forward approach. These values are intentionally simple and compact.
// They are based on your measured 90,45 result near psi=-81, omega=-89.
// Keep them as easy-to-tune constants rather than storing a large lookup table.
const bool  CL_ENABLE_INITIAL_COARSE = true;
const float CL_COARSE_START_SCORE = 35.0f;
const float CL_COARSE_PSI_PER_THETA = -0.90f;
const float CL_COARSE_OMEGA_PER_PHI = -1.95f;
const float CL_COARSE_FRACTION = 0.88f;
const float CL_COARSE_LIMIT_MARGIN = 4.0f;

float clampCoarseTarget(float x) {
  return constrain(x, MIN_ANGLE_DEG + CL_COARSE_LIMIT_MARGIN,
                     MAX_ANGLE_DEG - CL_COARSE_LIMIT_MARGIN);
}

// Relative coarse move: calculate from the current Hall orientation rather than
// treating the zero-based coarse map as an absolute motor target. This lets
// fast coarse motion work for target-to-target moves as well as zero-to-target.
const float CL_MIN_DET = 0.08f;  // Minimum safe 2x2 Jacobian determinant

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

  // Use the learned local Jacobian as the fast relative coarse estimate.
  // This is still a coarse move because it is deliberately damped and capped.
  if (usePhi) {
    const float det = clModel.a * clModel.d - clModel.b * clModel.c;
    if (fabsf(det) > CL_MIN_DET) {
      dPsi = ( clModel.d * eTheta - clModel.b * ePhi) / det;
      dOmega = (-clModel.c * eTheta + clModel.a * ePhi) / det;
    } else {
      dPsi = eTheta / keepNonZeroSlope(clModel.a);
      dOmega = 0.0f;
    }
  } else {
    dPsi = eTheta / keepNonZeroSlope(clModel.a);
  }

  // Do not use an unbounded prediction for the first jump.
  if (!usePhi) {
  // Theta-only moves can make a much larger safe coarse psi jump.
  dPsi = constrain(dPsi * CL_COARSE_FRACTION, -35.0f, 35.0f);
  dOmega = 0.0f;
  } else {
    // Two-axis moves stay more conservative.
    dPsi = constrain(dPsi * CL_COARSE_FRACTION, -12.0f, 12.0f);
    dOmega = constrain(dOmega * CL_COARSE_FRACTION, -22.0f, 22.0f);
  }

  float nextPsi = clampCoarseTarget(curPsi + dPsi);
  float nextOmega = clampCoarseTarget(curOmega + dOmega);
  if (fabsf(nextPsi - curPsi) < 0.5f && fabsf(nextOmega - curOmega) < 0.5f) return false;

  Serial.print(F("CL,COARSE_REL,PSI,")); Serial.print(nextPsi, 2);
  Serial.print(F(",OMEGA,")); Serial.println(nextOmega, 2);
  return executeMove(nextPsi, nextOmega, rpm);
}

// Select command size from error. This is the fast/coarse stage, medium stage,
// and fine stage. Pole mode is intentionally smaller because PHI is undefined
// and THETA estimates are more sensitive there.
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
  } else if (score > 35.0f) {
    damping = 0.80f; maxPsi = 8.0f; maxOmega = 15.0f;
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

// Predict a motor correction from the inverse local Jacobian.
// phiErrorOverride is normally NaN. When supplied, it lets the caller use the
// other 360-degree PHI branch if the shortest branch would violate a motor limit.
bool predictCorrectionWithPhiError(const HallOrientation &o, float targetTheta, float targetPhi,
                                   bool phiNeeded, float score, float phiErrorOverride,
                                   float &dPsi, float &dOmega) {
  float eTheta = targetTheta - o.thetaDeg;
  bool poleMode = !phiNeeded || !o.phiDefined;
  float damping, maxPsi, maxOmega;
  selectMotionLimits(score, poleMode, damping, maxPsi, maxOmega);

  if (poleMode) {
    dPsi = eTheta / keepNonZeroSlope(clModel.a);
    dOmega = 0.0f;
  } else {
    float ePhi = isnan(phiErrorOverride) ? signedPhiDifference(targetPhi, o.phiDeg)
                                          : phiErrorOverride;
    float det = clModel.a * clModel.d - clModel.b * clModel.c;
    if (fabsf(det) < 0.08f) return false;
    dPsi = ( clModel.d * eTheta - clModel.b * ePhi) / det;
    dOmega = (-clModel.c * eTheta + clModel.a * ePhi) / det;
  }

  dPsi *= damping;
  dOmega *= damping;
  dPsi = constrain(dPsi, -maxPsi, maxPsi);
  dOmega = constrain(dOmega, -maxOmega, maxOmega);

  if (fabsf(dPsi) < 0.20f) dPsi = 0.0f;
  if (fabsf(dOmega) < 0.35f) dOmega = 0.0f;
  return (dPsi != 0.0f || dOmega != 0.0f);
}

bool predictCorrection(const HallOrientation &o, float targetTheta, float targetPhi,
                       bool phiNeeded, float score, float &dPsi, float &dOmega) {
  return predictCorrectionWithPhiError(o, targetTheta, targetPhi, phiNeeded, score, NAN,
                                       dPsi, dOmega);
}

// Learn from verified motions. Large moves are down-weighted; THETA is still
// learned when PHI is undefined, while PHI entries update only when meaningful.
void updateModel(const HallOrientation &before, const HallOrientation &after,
                 float dPsi, float dOmega) {
  float denom = dPsi * dPsi + dOmega * dOmega;
  if (denom < 0.01f) return;

  // Do not let pole behavior overwrite the general middle-region Jacobian.
  // The controller can still move near the pole; it just does not learn there.
  if (!thetaSafelyAwayFromPole(before.thetaDeg) ||
      !thetaSafelyAwayFromPole(after.thetaDeg)) {
    Serial.println(F("CL,MODEL_SKIP,POLE_REGION"));
    return;
  }

  // Big moves still contain useful information, but they cross more nonlinear
  // parts of the mechanism. Learn from them at reduced weight rather than
  // ignoring every coarse move.
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
// Two post-move reads help avoid treating one Hall outlier as a physical jump.
// When they disagree, use the reading closer to the target for this iteration.
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

bool runFallbackCrossSearch(const HallOrientation &baseMeasure, float basePsi, float baseOmega,
                            float targetTheta, float targetPhi, bool phiNeeded, float rpm,
                            float &acceptedDPsi, float &acceptedDOmega) {
  float thetaError = fabsf(baseMeasure.thetaDeg - targetTheta);
  float phiError = phiNeeded ? phiErrorDeg(baseMeasure.phiDeg, targetPhi) : 0.0f;
  float psiStep = (thetaError > 8.0f) ? 3.0f : 1.0f;
  float omegaStep = !phiNeeded ? 0.0f : ((phiError > 15.0f) ? 8.0f : 3.0f);
  float baseScore = closedLoopScore(baseMeasure, targetTheta, targetPhi, phiNeeded);
  float bestScore = baseScore;
  acceptedDPsi = acceptedDOmega = 0.0f;
  SearchMove moves[8] = {
    { psiStep, 0}, {-psiStep, 0},
    { 0, omegaStep}, {0, -omegaStep},
    { psiStep, omegaStep}, {psiStep, -omegaStep},
    {-psiStep, omegaStep}, {-psiStep, -omegaStep}
  };

  for (int i = 0; i < 8; i++) {
    if (moves[i].dPsi == 0.0f && moves[i].dOmega == 0.0f) continue;
    float p = basePsi + moves[i].dPsi;
    float w = baseOmega + moves[i].dOmega;
    if (!inRange(p) || !inRange(w)) continue;
    if (!executeMove(p, w, rpm)) continue;
    delay(CL_SETTLE_MS);

    HallOrientation trial;
    if (!readStableOrientation(trial, CL_FAST_READS)) continue;
    float s = closedLoopScore(trial, targetTheta, targetPhi, phiNeeded);
    Serial.print(F("CL,FALLBACK,PSI,")); Serial.print(p,2);
    Serial.print(F(",OMEGA,"));          Serial.print(w,2);
    Serial.print(F(",THETA,"));          Serial.print(trial.thetaDeg,2);
    Serial.print(F(",PHI,"));
    if (trial.phiDefined) Serial.print(trial.phiDeg,2); else Serial.print(F("UNDEFINED"));
    Serial.print(F(",SCORE,")); Serial.println(s,3);

    if (s < bestScore) {
      bestScore = s;
      acceptedDPsi = moves[i].dPsi;
      acceptedDOmega = moves[i].dOmega;
    }
    executeMove(basePsi, baseOmega, rpm);
    delay(CL_SETTLE_MS);
  }

  return bestScore < baseScore - max(CL_MIN_IMPROVEMENT_ABS,
                                     baseScore * CL_MIN_IMPROVEMENT_FRAC);
}

bool closedLoopOrient(float targetTheta, float targetPhi, bool targetPhiDefined, float rpm) {
  if (targetTheta < 0.0f || targetTheta > 180.0f || rpm <= 0.0f) {
    pushError(-310, F("Bad theta or RPM"));
    return false;
  }
  if (targetPhiDefined) targetPhi = wrap360(targetPhi);

  // One quick feed-forward move gets the stage near the destination.  The
  // persistent Jacobian then handles the measured final correction.
  if (CL_ENABLE_INITIAL_COARSE) {
    HallOrientation start;
    if (readStableOrientation(start, CL_VERIFY_READS)) {
      bool startPhiNeeded = targetPhiDefined && thetaSafelyAwayFromPole(targetTheta) &&
                            start.phiDefined && thetaSafelyAwayFromPole(start.thetaDeg);
      float startScore = closedLoopScore(start, targetTheta, targetPhi, startPhiNeeded);
      if (startScore > CL_COARSE_START_SCORE) {
        if (!runInitialCoarseApproach(start, targetTheta, targetPhi, targetPhiDefined, rpm)) {
          Serial.println(F("CL,COARSE_SKIPPED"));
        } else {
          delay(CL_SETTLE_MS);
        }
      }
    }
  }

  float rejectionStepScale = 1.0f;
  int rejectedMoves = 0;

  for (int iter = 1; iter <= CL_MAX_ITERATIONS; iter++) {
    HallOrientation current;
    int reads = (iter == 1) ? CL_VERIFY_READS : CL_FAST_READS;
    if (!readStableOrientation(current, reads)) {
      pushError(-311, F("Field too small"));
      return false;
    }

    // Keep phi out of the feedback loop until both the target and the current
    // measured direction are safely away from the theta poles.
    bool targetAtPole = !thetaSafelyAwayFromPole(targetTheta);
    bool phiReliable = current.phiDefined && thetaSafelyAwayFromPole(current.thetaDeg);
    bool phiNeeded = targetPhiDefined && !targetAtPole && phiReliable;
    float thetaError = fabsf(current.thetaDeg - targetTheta);
    float phiError = phiNeeded ? phiErrorDeg(current.phiDeg, targetPhi) : 0.0f;
    float score = closedLoopScore(current, targetTheta, targetPhi, phiNeeded);

    Serial.print(F("CL,ITER,"));       Serial.print(iter);
    Serial.print(F(",MODE,PREDICT"));
    Serial.print(F(",THETA,"));        Serial.print(current.thetaDeg,2);
    Serial.print(F(",PHI,"));
    if (current.phiDefined) Serial.print(current.phiDeg,2); else Serial.print(F("UNDEFINED"));
    Serial.print(F(",THETA_ERR,"));    Serial.print(thetaError,2);
    Serial.print(F(",PHI_ERR,"));      Serial.print(phiError,2);
    Serial.print(F(",SCORE,"));        Serial.println(score,3);

    if (orientationAtTarget(current, targetTheta, targetPhi, targetPhiDefined)) {
      HallOrientation verified;
      if (readStableOrientation(verified, CL_VERIFY_READS) &&
          orientationAtTarget(verified, targetTheta, targetPhi, targetPhiDefined)) {
        Serial.println(F("CL,OK"));
        return true;
      }
    }

    float dPsi = 0.0f, dOmega = 0.0f;
    float curPsi = currentPsiDeg();
    float curOmega = currentOmegaDeg();
    bool predicted = predictCorrection(current, targetTheta, targetPhi, phiNeeded,
                                       score, dPsi, dOmega);
    bool normalInRange = predicted && inRange(curPsi + dPsi) && inRange(curOmega + dOmega);

    // Keep the fast Jacobian path while it is safe. If it would cross a
    // wire-protection limit, use the small physical fallback only then.
    if (!normalInRange) {
      Serial.println(F("CL,PREDICT_LIMIT,FALLBACK"));
      if (!runFallbackCrossSearch(current, curPsi, curOmega,
                                  targetTheta, targetPhi, phiNeeded, rpm,
                                  dPsi, dOmega)) {
        pushError(-312, F("Target blocked by motor limits"));
        Serial.println(F("CL,STALLED,LIMIT"));
        return false;
      }
    }

    // After a rejected move, retry with a smaller version of the prediction.
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

    Serial.print(F("CL,PREDICT,DPSI,")); Serial.print(dPsi,2);
    Serial.print(F(",DOMEGA,"));         Serial.print(dOmega,2);
    Serial.print(F(",PSI,"));            Serial.print(nextPsi,2);
    Serial.print(F(",OMEGA,"));          Serial.println(nextOmega,2);

    if (!executeMove(nextPsi, nextOmega, rpm)) {
      pushError(-314, F("Predicted move failed"));
      return false;
    }
    delay(CL_SETTLE_MS);

    HallOrientation after;
    float afterScore = 0.0f;
    if (!readPostMoveOrientation(after, targetTheta, targetPhi, phiNeeded, afterScore)) {
      pushError(-311, F("Field too small"));
      return false;
    }

    // Reject moves that clearly worsen the score. Restore the known better
    // motor position, reduce the next step, and do not teach the model from it.
    if (afterScore > score) {
      Serial.print(F("CL,REJECT,WORSE_SCORE,BEFORE,"));
      Serial.print(score, 3);
      Serial.print(F(",AFTER,"));
      Serial.println(afterScore, 3);

      if (!executeMove(beforePsi, beforeOmega, rpm)) {
        pushError(-317, F("Rollback failed"));
        return false;
      }
      delay(CL_SETTLE_MS * 2UL);

      rejectionStepScale *= 0.5f;
      if (rejectionStepScale < CL_MIN_REJECTION_SCALE) {
        rejectionStepScale = CL_MIN_REJECTION_SCALE;
      }
      rejectedMoves++;

      if (rejectedMoves >= CL_MAX_REJECTS_BEFORE_RESET) {
        clModel = CL_MODEL_DEFAULT;
        rejectedMoves = 0;
        Serial.println(F("CL,MODEL_RESET,REPEATED_REJECTS"));
      }
      continue;
    }

    // Successful movement: cautiously restore normal command size.
    rejectionStepScale = min(1.0f, rejectionStepScale * 1.25f);
    rejectedMoves = 0;

    updateModel(current, after, dPsi, dOmega);
    printModel();
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
  if (loadSavedJacobian()) Serial.println(F("CAL,JAC,LOADED"));

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
  Serial.println(F("Closed-loop uses relative coarse move and persistent Jacobian."));
  Serial.println(F("MOVE:ZERO"));
  Serial.println(F("HOME:OMEGA"));
  Serial.println(F("HOME:PSI"));
  Serial.println(F("MEM:CLEAR"));
  Serial.println(F("CAL:JAC? / CAL:JAC:SAVE / CAL:JAC:LOAD / CAL:JAC:CLEAR"));
  Serial.println(F("SYST:ERR?"));
}

void loop() {
  handleSerialInput();
}
