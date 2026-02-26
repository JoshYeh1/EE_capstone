// STEP 7: user input with calibration with LED for FUN NOW WITH ACCELERATION STUFF

const int stepX = 2;
const int dirX  = 5;

const int potPin = A0;

const int zeroBtnPin = 9;// button to set zero
const int ledPin = 10;//Fun LED pin

//FOR SMOOOOTH ACCELERATION
bool motionActive = false;
long motionStartSteps = 0;
long motionEndSteps = 0;
unsigned long motionStartTime = 0;
unsigned long motionDuration = 3000; 

const bool btnActiveLow = true;  // INPUT_PULLUP, when pressed = LOW

const int microstep = 8;
const int motor_steps = 200;
const long steps_per_rev = (long)motor_steps * microstep;

// motion timing
unsigned long step_delay = 800;
unsigned long lastStepTime = 0;
bool stepState = false;

// position tracking
long currentPosition = 0;   // motor position in microsteps (tracked)
long targetPosition  = 0;   // commanded target in microsteps

// software zero offset (set by button)
long zeroOffsetSteps = 0;

// serial input
String inputLine = "";

// mode
enum Mode { CALIBRATE_POT, RUN_SERIAL };
Mode mode = CALIBRATE_POT;

// ---------------- helpers ----------------
bool buttonPressed() {
  int v = digitalRead(zeroBtnPin);
  return btnActiveLow ? (v == LOW) : (v == HIGH);
}

// debounced press detect (edge)
bool buttonRisingEdge() {
  static bool stableState = false;         // last stable pressed/unpressed
  static bool lastReading = false;         // last raw reading
  static unsigned long lastChange = 0;

  bool reading = buttonPressed();

  if (reading != lastReading) {
    lastChange = millis();
    lastReading = reading;
  }

  if ((millis() - lastChange) > 30) {      // debounce time
    if (reading != stableState) {
      stableState = reading;
      if (stableState == true) return true; // became pressed
    }
  }

  return false;
}

void setTargetFromAngle(float angleDeg) {

  long angleSteps = (long)((angleDeg / 360.0) * (float)steps_per_rev);
  long newTarget = zeroOffsetSteps + angleSteps;

  motionStartSteps = currentPosition;
  motionEndSteps = newTarget;

  long distance = abs(motionEndSteps - motionStartSteps);

  // Duration proportional to move size
  motionDuration = map(distance, 0, 4000, 500, 4000);
  motionDuration = constrain(motionDuration, 500, 5000);

  motionStartTime = millis();
  motionActive = true;
}

void updateSmoothMotion() {
  if (mode == CALIBRATE_POT) {
    step_delay = 200;
    return;
  }
  if (!motionActive) return;

  unsigned long elapsed = millis() - motionStartTime;

  if (elapsed >= motionDuration) {
    targetPosition = motionEndSteps;
    motionActive = false;
    return;
  }

  float t = (float)elapsed / (float)motionDuration;

  // Smooth cosine interpolation (position)
  float s = 0.5 * (1 - cos(PI * t));

  long delta = motionEndSteps - motionStartSteps;
  targetPosition = motionStartSteps + (long)(delta * s);

  //Velocity shaping

  float v_norm = (PI / (2.0 * motionDuration)) * sin(PI * t);

  float totalSteps = abs(delta);
  float v_steps_per_ms = totalSteps * v_norm;

  float v_steps_per_s = v_steps_per_ms * 1000.0;

  if (v_steps_per_s > 1.0) {
    step_delay = (unsigned long)(1000000.0 / v_steps_per_s);
  }

  // Safety clamp
  step_delay = constrain(step_delay, 200, 3000);
}

// non-blocking step pulses toward targetPosition
void updateMotorTowardTarget() {

  if (abs(currentPosition - targetPosition) <= 2) return;

  bool direction = (targetPosition > currentPosition);
  digitalWrite(dirX, direction ? HIGH : LOW);

  unsigned long currentTime = micros();

  long error = targetPosition - currentPosition;
  long absError = abs(error);

  const unsigned long pulseWidth = 5;  // 5 µs HIGH pulse

  if (!stepState && (currentTime - lastStepTime >= step_delay)) {
    digitalWrite(stepX, HIGH);
    stepState = true;
    lastStepTime = currentTime;
  }

  else if (stepState && (currentTime - lastStepTime >= pulseWidth)) {
    digitalWrite(stepX, LOW);
    stepState = false;
    lastStepTime = currentTime;

    if (direction) currentPosition++;
    else currentPosition--;
  }
}

void setup() {
  pinMode(stepX, OUTPUT);
  pinMode(dirX, OUTPUT);

  pinMode(zeroBtnPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); 
   
  Serial.begin(9600);

  Serial.println("\nUSER INPUT WITH CALIBRATION");
  Serial.println("MODE: CALIBRATE (pot controls motor).");
  Serial.println("Turn pot to desired 'zero' orientation, then press the button.");
  Serial.println("After that: type an angle in degrees ( 90, -45, 360)");
  Serial.println("Type: cal   (to use pot calibration mode)");
  Serial.println("Type: run   (to use serial control mode)");
}

void updateLED() {

  static unsigned long lastBlink = 0;
  static bool ledState = false;

  if (mode == CALIBRATE_POT) {
    // Blink while calibrating
    if (millis() - lastBlink > 250) {   // 2 Hz blink
      ledState = !ledState;
      digitalWrite(ledPin, ledState);
      lastBlink = millis();
    }
  }
  else if (mode == RUN_SERIAL) {
    // Solid ON when zero is set
    digitalWrite(ledPin, HIGH);
  }
}

void loop() {
  // -------- read serial non-blocking --------
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      inputLine.trim();
      if (inputLine.length() > 0) {

        if (inputLine.equalsIgnoreCase("cal")) {
          mode = CALIBRATE_POT;
          Serial.println("MODE: CALIBRATE (pot controls motor). Press button to set zero.");
          inputLine = "";
          break;
        }

        if (inputLine.equalsIgnoreCase("run")) {
          mode = RUN_SERIAL;
          Serial.println("MODE: RUN (serial angles).");
          inputLine = "";
          break;
        }

        if (mode == RUN_SERIAL) {
          float angleDeg = inputLine.toFloat();
          setTargetFromAngle(angleDeg);

          Serial.print("Command angle (deg): ");
          Serial.print(angleDeg);
          Serial.print(" -> target steps: ");
          Serial.println(targetPosition);
        } else {
          Serial.println("You're in CALIBRATE mode. Press button to set zero, or type 'run'.");
        }

        inputLine = "";
      }
    } else {
      if (isPrintable(c)) inputLine += c;
    }
  }

  // -------- button: set zero --------
  if (buttonRisingEdge()) {
    // define current motor position as "0 degrees"
    zeroOffsetSteps = currentPosition;

    Serial.println("\nNEW ZERO SET!");
    Serial.print("zeroOffsetSteps = ");
    Serial.println(zeroOffsetSteps);
    Serial.println("Now type angles in degrees. Switching to RUN mode.\n");

    mode = RUN_SERIAL;
  }

  // -------- mode behavior --------
  if (mode == CALIBRATE_POT) {
    // Read pot and low-pass filter
    static float filteredValue = 0;
    int raw = analogRead(potPin);
    filteredValue = 0.95f * filteredValue + 0.05f * raw;

    // map pot to 0..720 degrees (2 revs). Change to 360 if desired.
    float angle = (filteredValue / 1023.0f) * 720.0f;

    long potSteps = (long)((angle / 360.0f) * (float)steps_per_rev);
    targetPosition = potSteps;

    // ---- FIXED PRINTING ----
    // 1) Pause prints while user is typing (Serial.available() == 0)
    // 2) Print only when pot changed enough (threshold), AND not faster than ~10 Hz
    static int lastRaw = -1;
    static unsigned long lastPrint = 0;

    if (Serial.available() == 0) {
      const int threshold = 10; // raw ADC counts; tune 5-20
      if (abs(raw - lastRaw) >= threshold && (millis() - lastPrint) > 100) {
        lastRaw = raw;
        lastPrint = millis();

        Serial.print("CAL pot raw=");
        Serial.print(raw);
        Serial.print(" filtered=");
        Serial.print(filteredValue, 1);
        Serial.print(" angle≈");
        Serial.print(angle, 1);
        Serial.print(" deg  currentSteps=");
        Serial.println(currentPosition);
      }
    }
  }

  //updates
  updateSmoothMotion();
  updateMotorTowardTarget();
  updateLED();
}