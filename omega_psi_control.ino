// ============================================================
// TWO-AXIS OMEGA/PSI STEPPER CONTROLLER
// ============================================================
//
// Controls two stepper motors to point a platform to any
// direction on a sphere using the shortest arc path.
//
// MOTOR WIRING:
//   Motor X (beta axis) : Step=pin2, Dir=pin5
//   Motor Y (alpha axis): Step=pin3, Dir=pin6
//   Home switch         : pin9 (active LOW, INPUT_PULLUP)
//
// SERIAL COMMANDS (115200 baud):
//   home omega          -- zero the omega axis (Motor X only)
//   home psi            -- zero the psi   axis (both motors)
//   status              -- print current position and state
//   psi,omega,rpm       -- move to target, e.g.  -30,20,5
//
// COORDINATE SYSTEM:
//   psi   [-180, 180] deg  -- elevation axis(Motor Y)
//   omega [-180, 180] deg  -- azimuth axis(Motor X)
//   alpha = psi            -- physical Y motor angle
//   beta  = psi + omega    -- physical X motor angle
//
// NOTES:
//   - 1/4 microstepping (set on driver DIP switches)
//   - Shortest great-circle path on the sphere
//   - Both motors move simultaneously
//   - Homing stops immediately when switch is triggered
// ============================================================

#include <math.h>

// ---------------- SECTION 1: CONFIGURATION --------

// --- Pin assignments ---
const int PIN_STEP_X     = 2;
const int PIN_DIR_X      = 5;
const int PIN_STEP_Y     = 3;
const int PIN_DIR_Y      = 6;
const int PIN_HOME_SWITCH = 9;   // INPUT_PULLUP; switch pulls LOW when pressed

// --- Motor geometry ---
const int   FULL_STEPS_PER_REV = 200;    // standard 1.8-degree stepper
const int   MICROSTEPS         = 4;      // 1/4 microstepping on driver
const float DEG_PER_STEP = 360.0f / (FULL_STEPS_PER_REV * MICROSTEPS);  // gives us 0.45 deg of roatation at 1/4 microstep

// --- Set motor direction (flip if a motor runs backwards) ---
const bool INVERT_DIR_X = true;
const bool INVERT_DIR_Y = false;

// --- Speed limits ---
const float MIN_STEPS_PER_SEC = 5.0f;   // prevents stall at very low speed (in step/sec)
const float HOMING_RPM        = 2.0f;   // slow, safe speed during calibration (in RPM)

// --- Step pulse width (microseconds) ---
// Most drivers need >= 2 µs HIGH; 3 µs gives comfortable margin.
const unsigned long STEP_PULSE_HIGH_US = 3;

// --- Homing safety: max steps before giving up ---
// At 0.45 deg/step, 1600 steps = 720 deg; more than enough for one full turn.
const long HOMING_MAX_STEPS = 1600;

// --- Angle limits ---
const float MIN_ANGLE_DEG = -180.0f;
const float MAX_ANGLE_DEG =  180.0f;

// --- Debug flags (set true to print verbose info to Serial) ---
const bool DEBUG_MOTION = false;
const bool DEBUG_PATH   = false;

// ============================================================
// ---- SECTION 2: BASIC MATH HELPERS ------------------------
// ============================================================

// Simple 3-component vector
struct Vec3 { float x, y, z; };

// Clamp x to [lo, hi]
float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// Wrap angle to (-180, +180]
float normalizeDeg180(float a) {
  //fmod to avoid unbounded while-loops on extreme inputs
  a = fmodf(a + 180.0f, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a - 180.0f;
}

bool angleInRange(float deg) {
  return deg >= MIN_ANGLE_DEG && deg <= MAX_ANGLE_DEG;
}

float degToRad(float d) { return d * (PI / 180.0f); }
float radToDeg(float r) { return r * (180.0f / PI); }

// Absolute angular distance, accounting for wrap-around
float wrappedAngularDistance(float a, float b) {
  return fabsf(normalizeDeg180(a - b));
}

// --- Vector operations ---
float  vecDot  (Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
float  vecNorm (Vec3 v)         { return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z); }
Vec3   vecScale(Vec3 v, float s){ return {v.x*s, v.y*s, v.z*s}; }
Vec3   vecAdd  (Vec3 a, Vec3 b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
Vec3   vecCross(Vec3 a, Vec3 b) {
  return { a.y*b.z - a.z*b.y,
           a.z*b.x - a.x*b.z,
           a.x*b.y - a.y*b.x };
}
Vec3 vecNormalize(Vec3 v) {
  float n = vecNorm(v);
  if (n < 1e-9f) return {0.0f, 0.0f, 1.0f};   // fallback: +Z
  return vecScale(v, 1.0f / n);
}

// ============================================================
// ---- SECTION 3: COORDINATE CONVERSIONS --------------------
// ============================================================
//
// Internal convention:  theta = -psi,  phi = -omega
// so the sphere's +Z maps to psi=0, omega=0.

// Convert (psi, omega) angles to a unit vector on the sphere
Vec3 psiOmegaToVec3(float psiDeg, float omegaDeg) {
  float theta = degToRad(-psiDeg);
  float phi   = degToRad(-omegaDeg);
  return vecNormalize({
    sinf(theta) * cosf(phi),
    sinf(theta) * sinf(phi),
    cosf(theta)
  });
}

// Result type for the inverse conversion
struct AnglePair { float psi, omega; bool valid; };

// Given a unit vector and the previous (psi, omega), find the
// nearest valid (psi, omega) that maps to that vector.
// There are two solutions for each vector (opposite psi values);
// we pick whichever minimises motor travel, weighting psi more
// heavily because large psi jumps tend to be mechanically expensive.
AnglePair vec3ToPsiOmega(Vec3 n, float prevPsi, float prevOmega) {
  float nz        = clampf(n.z, -1.0f, 1.0f);
  float thetaDeg  = radToDeg(acosf(nz));
  float phiDeg    = radToDeg(atan2f(n.y, n.x));

  // Two families of solutions
  float psiA   = -thetaDeg;
  float omegaA = normalizeDeg180(-phiDeg);
  float psiB   = +thetaDeg;
  float omegaB = normalizeDeg180(-phiDeg - 180.0f);
  float omegaC = normalizeDeg180(-phiDeg + 180.0f);  // same as omegaB mod 360

  struct Candidate { float psi, omega; };
  Candidate candidates[3] = { {psiA, omegaA}, {psiB, omegaB}, {psiB, omegaC} };

  AnglePair best = {0.0f, 0.0f, false};
  float bestScore = 1e15f;

  for (auto &c : candidates) {
    if (!angleInRange(c.psi) || !angleInRange(c.omega)) continue;

    // Weight psi travel 10x more than omega to penalise big psi swings
    float score = 10.0f * fabsf(c.psi - prevPsi)
                +  1.0f * wrappedAngularDistance(c.omega, prevOmega);

    if (!best.valid || score < bestScore) {
      best = {c.psi, c.omega, true};
      bestScore = score;
    }
  }
  return best;
}

// ============================================================
// ---- SECTION 4: MOTOR STEP UTILITIES ----------------------
// ============================================================

// Convert RPM to steps per second
float rpmToSPS(float rpm) {
  return rpm * (FULL_STEPS_PER_REV * MICROSTEPS) / 60.0f;
}

// Convert steps-per-second to the period between steps (µs)
unsigned long spsToPeriodUs(float sps) {
  if (sps <= 0.0f) return 0;
  float period = 1000000.0f / sps;
  float minPeriod = 2.0f * STEP_PULSE_HIGH_US;
  if (period < minPeriod) period = minPeriod;
  return (unsigned long)(period + 0.5f);
}

// Set a DIR pin according to the sign of the desired step,
// accounting for any direction inversion.
void setDir(int dirPin, int stepSign, bool invert) {
  bool wantPos = (stepSign >= 0);
  digitalWrite(dirPin, (wantPos ^ invert) ? HIGH : LOW);
}

// Convert (psi, omega) to absolute stepper positions in steps.
//   alpha = psi        --> Y motor
//   beta  = psi+omega  --> X motor
void anglestoMotorSteps(float psiDeg, float omegaDeg,
                         long &xSteps, long &ySteps) {
  ySteps = lroundf(psiDeg             / DEG_PER_STEP);
  xSteps = lroundf((psiDeg + omegaDeg) / DEG_PER_STEP);
}

// ============================================================
// ---- SECTION 5: SYSTEM STATE ------------------------------
// ============================================================

// --- Current logical position ---
float currentPsi   = 0.0f;   // degrees
float currentOmega = 0.0f;   // degrees

// --- Physical step counters (track actual pulses sent) ---
long physStepsX = 0;
long physStepsY = 0;

// --- Commanded step targets (used by the DDA chaser) ---
long cmdStepsX = 0;
long cmdStepsY = 0;

// Re-derive step counters from the current software angles.
// Call this after homing or any angle snap.
void syncStepCounters() {
  long x, y;
  anglestoMotorSteps(currentPsi, currentOmega, x, y);
  cmdStepsX = physStepsX = x;
  cmdStepsY = physStepsY = y;
}

// ============================================================
// ---- SECTION 6: HOME SWITCH --------------------------------
// ============================================================

bool homeSwitchRaw() {
  return digitalRead(PIN_HOME_SWITCH) == LOW;   // active-LOW
}

// Returns true only after the switch has held the same state
// for 20 ms (simple debounce).
bool homeSwitchDebounced() {
  static bool     stable      = false;
  static bool     lastReading = false;
  static unsigned long lastChange = 0;

  bool reading = homeSwitchRaw();
  if (reading != lastReading) {
    lastReading = reading;
    lastChange  = millis();
  }
  if (millis() - lastChange > 20) stable = reading;
  return stable;
}

// ============================================================
// ---- SECTION 7: SERIAL INPUT PARSING ----------------------
// ============================================================

char    inputBuf[32];
uint8_t inputLen = 0;

// Parse "psi,omega,rpm" into three floats. Returns false on error.
bool parseMotionCommand(const char *line,
                        float &psi, float &omega, float &rpm) {
  const char *p1 = strchr(line, ',');      if (!p1) return false;
  const char *p2 = strchr(p1 + 1, ',');   if (!p2) return false;
  if (strchr(p2 + 1, ',')) return false;  // too many commas

  psi   = atof(line);
  omega = atof(p1 + 1);
  rpm   = atof(p2 + 1);
  return true;
}

// ============================================================
// ---- SECTION 8: MOTION ENGINE -----------------------------
// ============================================================

// What is the motion system doing right now?
enum MotionMode {
  MODE_IDLE,
  MODE_HOMING_OMEGA,   // Motor X only, looking for switch
  MODE_HOMING_PSI,     // Both motors, looking for switch
  MODE_PATH_FOLLOW     // Continuous spherical path following
};

struct MotionState {
  MotionMode mode    = MODE_IDLE;
  bool       active  = false;

  // Current local DDA target (steps)
  long targetX = 0, targetY = 0;

  // Bresenham/DDA error accumulators
  long accumX = 0, accumY = 0;

  // Step direction signs (+1 or -1)
  int  signX = 0, signY = 0;

  // Speed
  float          rpm           = 0.0f;
  unsigned long  periodUs      = 0;
  unsigned long  lastStepUs    = 0;

  // Pulse state (we keep the HIGH for STEP_PULSE_HIGH_US before pulling LOW)
  bool          pulseHigh      = false;
  unsigned long pulseStartUs   = 0;
  bool          pulsedX        = false;
  bool          pulsedY        = false;

  // Homing safety counter
  long homingStepCount = 0;
};

MotionState mot;

void resetMotion() {
  mot = MotionState();   // zero-initialise everything
}

// --- Low-level pulse helpers ---
void beginPulse(bool doX, bool doY) {
  if (doX) { digitalWrite(PIN_STEP_X, HIGH); physStepsX += mot.signX; cmdStepsX += mot.signX; }
  if (doY) { digitalWrite(PIN_STEP_Y, HIGH); physStepsY += mot.signY; cmdStepsY += mot.signY; }
  mot.pulseHigh    = true;
  mot.pulseStartUs = micros();
  mot.pulsedX      = doX;
  mot.pulsedY      = doY;
}

void endPulse() {
  if (mot.pulsedX) digitalWrite(PIN_STEP_X, LOW);
  if (mot.pulsedY) digitalWrite(PIN_STEP_Y, LOW);
  mot.pulseHigh = mot.pulsedX = mot.pulsedY = false;
}

// ============================================================
// ---- SECTION 9: SPHERICAL PATH PLANNER --------------------
// ============================================================

struct PathPlan {
  bool  active       = false;
  Vec3  nStart, nEnd;
  float gammaRad     = 0.0f;   // total arc angle
  int   totalSeg     = 0;      // number of interpolation segments
  int   currentSeg   = 0;      // segments completed so far
  float prevPsi      = 0.0f;
  float prevOmega    = 0.0f;
  float goalPsi      = 0.0f;   // final target (used for snap at end)
  float goalOmega    = 0.0f;
  float rpm          = 0.0f;
  bool  antiparallel = false;  // special case: 180-deg flip
  Vec3  fallbackAxis;           // rotation axis for antiparallel case
};

PathPlan plan;

void resetPlan() { plan = PathPlan(); }

// Pick any axis perpendicular to n (for the antiparallel fallback)
Vec3 perpendicularTo(Vec3 n) {
  Vec3 ref = (fabsf(n.z) < 0.9f) ? Vec3{0,0,1} : Vec3{1,0,0};
  return vecNormalize(vecCross(n, ref));
}

// Rotate vector u around unit axis k by angle thetaRad (Rodrigues)
Vec3 rodrigues(Vec3 u, Vec3 k, float thetaRad) {
  float c = cosf(thetaRad), s = sinf(thetaRad);
  return vecNormalize(
    vecAdd(vecAdd(vecScale(u, c),
                  vecScale(vecCross(k, u), s)),
           vecScale(k, vecDot(k, u) * (1.0f - c)))
  );
}

// Spherical linear interpolation: return point at fraction u in [0,1]
Vec3 slerp(Vec3 a, Vec3 b, float gammaRad, float u,
           bool antiparallel, Vec3 fallback) {
  if (gammaRad < 1e-8f)            return a;
  if (antiparallel)                return rodrigues(a, fallback, u * PI);
  float sg = sinf(gammaRad);
  if (fabsf(sg) < 1e-8f)          return a;
  float w1 = sinf((1.0f - u) * gammaRad) / sg;
  float w2 = sinf(u           * gammaRad) / sg;
  return vecNormalize(vecAdd(vecScale(a, w1), vecScale(b, w2)));
}

// Advance one step along the planned path.
// Returns true if a new local target was loaded, false when done or on error.
bool loadNextSegment() {
  if (!plan.active) return false;
  if (plan.currentSeg >= plan.totalSeg) return false;  // signal done

  int   nextSeg = plan.currentSeg + 1;
  float u       = (float)nextSeg / (float)plan.totalSeg;
  Vec3  nMid    = slerp(plan.nStart, plan.nEnd, plan.gammaRad, u,
                        plan.antiparallel, plan.fallbackAxis);

  AnglePair ap = vec3ToPsiOmega(nMid, plan.prevPsi, plan.prevOmega);
  if (!ap.valid || !angleInRange(ap.psi) || !angleInRange(ap.omega)) {
    Serial.println(F("Path error: invalid intermediate angle. Stopping."));
    return false;
  }

  long xT, yT;
  anglestoMotorSteps(ap.psi, ap.omega, xT, yT);

  mot.targetX = xT;
  mot.targetY = yT;
  mot.accumX  = 0;
  mot.accumY  = 0;

  plan.prevPsi     = ap.psi;
  plan.prevOmega   = ap.omega;
  plan.currentSeg  = nextSeg;

  if (DEBUG_PATH) {
    Serial.print(F("seg ")); Serial.print(nextSeg);
    Serial.print(F("/")); Serial.print(plan.totalSeg);
    Serial.print(F("  psi=")); Serial.print(ap.psi, 2);
    Serial.print(F("  omg=")); Serial.println(ap.omega, 2);
  }
  return true;
}

// Validate the full path before committing, then kick off motion.
bool startPathMove(float goalPsi, float goalOmega, float rpm) {
  resetPlan();

  Vec3  n0  = psiOmegaToVec3(currentPsi, currentOmega);
  Vec3  nf  = psiOmegaToVec3(goalPsi,    goalOmega);
  float dot = clampf(vecDot(n0, nf), -1.0f, 1.0f);
  float arc = acosf(dot);                  // radians
  int   segs = max(1, (int)ceilf(radToDeg(arc) / DEG_PER_STEP));

  plan.nStart      = n0;
  plan.nEnd        = nf;
  plan.gammaRad    = arc;
  plan.totalSeg    = segs;
  plan.currentSeg  = 0;
  plan.prevPsi     = currentPsi;
  plan.prevOmega   = currentOmega;
  plan.goalPsi     = goalPsi;
  plan.goalOmega   = goalOmega;
  plan.rpm         = rpm;
  plan.antiparallel = (fabsf(dot + 1.0f) < 1e-5f);
  plan.fallbackAxis = perpendicularTo(n0);

  if (DEBUG_PATH) {
    Serial.print(F("New path: arc=")); Serial.print(radToDeg(arc), 2);
    Serial.print(F(" deg, segments=")); Serial.println(segs);
  }

  // --- Pre-validate every waypoint before moving ---
  float tPsi = currentPsi, tOmega = currentOmega;
  for (int i = 1; i <= segs; i++) {
    float u = (float)i / (float)segs;
    Vec3  n = slerp(n0, nf, arc, u, plan.antiparallel, plan.fallbackAxis);
    AnglePair ap = vec3ToPsiOmega(n, tPsi, tOmega);
    if (!ap.valid) {
      Serial.println(F("Move rejected: path passes through unreachable angle."));
      resetPlan();
      return false;
    }
    tPsi = ap.psi; tOmega = ap.omega;
  }

  // Load the first waypoint and enable motion
  plan.active = true;
  if (!loadNextSegment()) {
    resetPlan();
    return false;
  }

  resetMotion();
  mot.mode      = MODE_PATH_FOLLOW;
  mot.rpm       = rpm;
  mot.active    = true;
  mot.lastStepUs = micros();
  return true;
}

// ============================================================
// ---- SECTION 10: HOMING ------------------------------------
// ============================================================

void startHomeOmega() {
  resetMotion();
  mot.mode     = MODE_HOMING_OMEGA;
  mot.rpm      = HOMING_RPM;
  mot.signX    = -1;    // search direction; flip to +1 if wrong
  mot.signY    = 0;
  mot.active   = true;
  mot.lastStepUs = micros();
  setDir(PIN_DIR_X, mot.signX, INVERT_DIR_X);
  Serial.println(F("Homing OMEGA (Motor X only)..."));
}

void startHomePsi() {
  resetMotion();
  mot.mode     = MODE_HOMING_PSI;
  mot.rpm      = HOMING_RPM;
  mot.signX    = -1;
  mot.signY    = -1;
  mot.active   = true;
  mot.lastStepUs = micros();
  setDir(PIN_DIR_X, mot.signX, INVERT_DIR_X);
  setDir(PIN_DIR_Y, mot.signY, INVERT_DIR_Y);
  Serial.println(F("Homing PSI (both motors)..."));
}

// Called every loop tick during homing; checks switch and step limit.
void checkHomingDone() {
  if (!homeSwitchDebounced() && mot.homingStepCount < HOMING_MAX_STEPS) return;

  // Stop output immediately
  digitalWrite(PIN_STEP_X, LOW);
  digitalWrite(PIN_STEP_Y, LOW);

  if (mot.homingStepCount >= HOMING_MAX_STEPS) {
    Serial.println(F("ERROR: Homing timed out (switch not found). Check wiring."));
  } else {
    if (mot.mode == MODE_HOMING_OMEGA) {
      currentOmega = 0.0f;
      Serial.println(F("OMEGA homed to 0 deg."));
    } else {
      currentPsi = 0.0f;
      Serial.println(F("PSI homed to 0 deg."));
    }
  }

  syncStepCounters();
  resetMotion();
}

// ============================================================
// ---- SECTION 11: MAIN MOTION UPDATE (called every loop) ---
// ============================================================

void updateMotion() {
  if (!mot.active) return;

  unsigned long nowUs = micros();

  // --- Finish any in-progress step pulse (hold HIGH for STEP_PULSE_HIGH_US) ---
  if (mot.pulseHigh) {
    if (nowUs - mot.pulseStartUs >= STEP_PULSE_HIGH_US) endPulse();
    return;   // don't start another step until this one is done
  }

  // --- Homing modes ---
  if (mot.mode == MODE_HOMING_OMEGA || mot.mode == MODE_HOMING_PSI) {
    checkHomingDone();
    if (!mot.active) return;

    mot.periodUs = spsToPeriodUs(max(rpmToSPS(mot.rpm), MIN_STEPS_PER_SEC));

    if (nowUs - mot.lastStepUs >= mot.periodUs) {
      bool doX = (mot.mode == MODE_HOMING_OMEGA || mot.mode == MODE_HOMING_PSI);
      bool doY = (mot.mode == MODE_HOMING_PSI);
      beginPulse(doX, doY);
      mot.homingStepCount++;
      mot.lastStepUs = nowUs;
    }
    return;
  }

  // --- Path-following mode ---
  if (mot.mode != MODE_PATH_FOLLOW) return;

  long dx = mot.targetX - cmdStepsX;
  long dy = mot.targetY - cmdStepsY;

  // If we've reached the current waypoint, load the next one
  if (dx == 0 && dy == 0) {
    if (!loadNextSegment()) {
      // No more segments: we're done (or an error occurred)
      bool done = (plan.currentSeg >= plan.totalSeg);
      if (done) {
        currentPsi   = plan.goalPsi;
        currentOmega = plan.goalOmega;
        syncStepCounters();
        if (DEBUG_PATH) Serial.println(F("Move complete."));
      }
      resetPlan();
      resetMotion();
    }
    return;
  }

  // --- Bresenham DDA: drive both axes simultaneously ---
  long ax = labs(dx), ay = labs(dy);
  long major = max(ax, ay);

  mot.signX = (dx >= 0) ? +1 : -1;
  mot.signY = (dy >= 0) ? +1 : -1;
  setDir(PIN_DIR_X, mot.signX, INVERT_DIR_X);
  setDir(PIN_DIR_Y, mot.signY, INVERT_DIR_Y);

  mot.periodUs = spsToPeriodUs(max(rpmToSPS(mot.rpm), MIN_STEPS_PER_SEC));

  if (nowUs - mot.lastStepUs >= mot.periodUs) {
    mot.accumX += ax;
    mot.accumY += ay;

    bool doX = (mot.accumX >= major);
    bool doY = (mot.accumY >= major);
    if (doX) mot.accumX -= major;
    if (doY) mot.accumY -= major;

    if (doX || doY) {
      beginPulse(doX, doY);
    }
    mot.lastStepUs = nowUs;
  }
}

bool systemBusy() {
  return mot.active || plan.active;
}

// ============================================================
// ---- SECTION 12: STATUS REPORT ----------------------------
// ============================================================

void printStatus() {
  Serial.print(F("psi     = ")); Serial.println(currentPsi,   3);
  Serial.print(F("omega   = ")); Serial.println(currentOmega, 3);
  Serial.print(F("alpha   = ")); Serial.println(currentPsi,   3);
  Serial.print(F("beta    = ")); Serial.println(currentPsi + currentOmega, 3);
  Serial.print(F("busy    = ")); Serial.println(systemBusy() ? F("YES") : F("NO"));
  Serial.print(F("switch  = ")); Serial.println(homeSwitchDebounced() ? F("PRESSED") : F("open"));
  Serial.print(F("physX   = ")); Serial.println(physStepsX);
  Serial.print(F("physY   = ")); Serial.println(physStepsY);
  Serial.print(F("cmdX    = ")); Serial.println(cmdStepsX);
  Serial.print(F("cmdY    = ")); Serial.println(cmdStepsY);
}

// ============================================================
// ---- SECTION 13: SERIAL COMMAND HANDLER ------------------
// ============================================================

void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      inputBuf[inputLen] = '\0';
      inputLen = 0;
      if (strlen(inputBuf) == 0) continue;

      // "status" works even while busy
      if (strcasecmp(inputBuf, "status") == 0) { printStatus(); continue; }

      if (systemBusy()) { Serial.println(F("Busy. Wait for move to finish.")); continue; }

      if (strcasecmp(inputBuf, "home omega") == 0) {
        homeSwitchDebounced()
          ? Serial.println(F("Switch already pressed — release first."))
          : startHomeOmega();
        continue;
      }

      if (strcasecmp(inputBuf, "home psi") == 0) {
        homeSwitchDebounced()
          ? Serial.println(F("Switch already pressed — release first."))
          : startHomePsi();
        continue;
      }

      float psiT, omegaT, rpmT;
      if (!parseMotionCommand(inputBuf, psiT, omegaT, rpmT)) {
        Serial.println(F("Unknown command. Use: psi,omega,rpm  |  home omega  |  home psi  |  status"));
        continue;
      }

      // Validate inputs
      if (!angleInRange(psiT))   { Serial.println(F("psi out of range [-180,180].")); continue; }
      if (!angleInRange(omegaT)) { Serial.println(F("omega out of range [-180,180].")); continue; }
      if (rpmT <= 0.0f)          { Serial.println(F("rpm must be > 0.")); continue; }

      if (fabsf(psiT - currentPsi) < 1e-6f && fabsf(omegaT - currentOmega) < 1e-6f) {
        Serial.println(F("Already at target.")); continue;
      }

      Serial.print(F("Moving to psi="));    Serial.print(psiT, 2);
      Serial.print(F(" omega="));           Serial.print(omegaT, 2);
      Serial.print(F(" at "));              Serial.print(rpmT, 1);
      Serial.println(F(" rpm"));

      if (!startPathMove(psiT, omegaT, rpmT))
        Serial.println(F("Could not plan move."));

    } else if (isPrintable(c) && inputLen < sizeof(inputBuf) - 1) {
      inputBuf[inputLen++] = c;
    }
  }
}

// ============================================================
// ---- SECTION 14: ARDUINO SETUP / LOOP --------------------
// ============================================================

void setup() {
  pinMode(PIN_STEP_X,      OUTPUT);  digitalWrite(PIN_STEP_X, LOW);
  pinMode(PIN_DIR_X,       OUTPUT);
  pinMode(PIN_STEP_Y,      OUTPUT);  digitalWrite(PIN_STEP_Y, LOW);
  pinMode(PIN_DIR_Y,       OUTPUT);
  pinMode(PIN_HOME_SWITCH, INPUT_PULLUP);

  Serial.begin(115200);
  resetMotion();
  resetPlan();
  syncStepCounters();

  Serial.println(F("=== Omega/Psi controller ready ==="));
  Serial.println(F("Commands: home omega | home psi | status | psi,omega,rpm"));
  Serial.println(F("Example:  -30,20,5"));
}

void loop() {
  handleSerial();
  updateMotion();
}
