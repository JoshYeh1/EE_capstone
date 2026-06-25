// Program uses SCIPI commands and uses three axis hall sensor to determine orientation of sample relative to mag field
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

const float X_OFFSET = 504.0f;
const float Y_OFFSET = 505.0f;
const float Z_OFFSET = 509.0f;

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

struct SavedState {
  uint32_t magic;
  long     stepsX;
  long     stepsY;
  uint8_t  valid;
  uint8_t  inMotion;
  uint16_t checksum;
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
  result.z = ((float)zSum / NUM_SAMPLES) - Z_OFFSET;
  return result;
}

// -------------------- Hall Vector -> Psi/Omega (No Lookup Table) --------------------
const float HALL_X_SCALE = 1.0f;
const float HALL_Y_SCALE = 1.0f;
const float HALL_Z_SCALE = 1.0f;

const float ZERO_REF_X = 1.380f;
const float ZERO_REF_Y = 3.340f;
const float ZERO_REF_Z = 103.750f;

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
  HallPsiOmega h = calculateHallPsiOmega(b);

  if (!h.valid) {
    Serial.println(F("ERR,FIELD_TOO_SMALL"));
    return;
  }

  Serial.print(F("BX,"));          Serial.print(b.x, 3);
  Serial.print(F(",BY,"));         Serial.print(b.y, 3);
  Serial.print(F(",BZ,"));         Serial.print(b.z, 3);
  Serial.print(F(",BMAG,"));       Serial.print(h.magnitude, 3);
  Serial.print(F(",HALL_PSI,"));   Serial.print(signed180ToUser360(h.psiDeg), 2);
  Serial.print(F(",HALL_OMEGA,")); Serial.print(signed180ToUser360(h.omegaDeg), 2);
  Serial.print(F(",HALL_PSI_INT,"));   Serial.print(h.psiDeg, 2);
  Serial.print(F(",HALL_OMEGA_INT,")); Serial.print(h.omegaDeg, 2);
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
    Serial.println(F("WashU,Goniometer,Prototype,1.1"));
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
  Serial.println(F("MOVE:ZERO"));
  Serial.println(F("HOME:OMEGA"));
  Serial.println(F("HOME:PSI"));
  Serial.println(F("MEM:CLEAR"));
  Serial.println(F("SYST:ERR?"));
}

void loop() {
  handleSerialInput();
}
