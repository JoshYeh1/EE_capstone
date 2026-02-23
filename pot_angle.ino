//STEP 4: Move motor using potentimeter

const int stepX = 2;
const int dirX  = 5;
const int potPin = A0;

const int microstep = 8; //CHANGE THIS DEPENDING ON MICROSTEPPING
const int motor_steps = 200;
const int steps_per_rev = motor_steps * microstep;

unsigned long step_delay = 800; // variable delay

unsigned long lastStepTime = 0;
bool stepState = false;

long currentPosition = 0;// where motor currently is (in steps)
long targetPosition  = 0;// where we want to go (in steps)

void setup() {
  pinMode(stepX, OUTPUT);
  pinMode(dirX, OUTPUT);
  Serial.begin(9600); 
}

void loop() {
  //Read potentiometer once and smooth using low pass filter
  static float filteredValue = 0;

  int raw = analogRead(potPin);
  filteredValue = 0.95 * filteredValue + 0.05 * raw;

  //Non-blocking serial print in terminal
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    Serial.print("Pot reading: ");
    Serial.println(raw);
  }

  float angle = (filteredValue / 1023.0) * 720.0;  // smoother than map()
  targetPosition = (angle / 360.0) * steps_per_rev;

  //Move motor
  if (abs(currentPosition - targetPosition) > 5) {

    bool direction = (targetPosition > currentPosition);
    digitalWrite(dirX, direction);

    unsigned long currentTime = micros();

    long error = targetPosition - currentPosition;
    long absError = abs(error);

    step_delay = map(absError, 0, 200, 3000, 600);
    step_delay = constrain(step_delay, 600, 3000);

    if (!stepState && (currentTime - lastStepTime >= step_delay)) {
      digitalWrite(stepX, HIGH);
      stepState = true;
      lastStepTime = currentTime;
    }

    else if (stepState && (currentTime - lastStepTime >= step_delay)) {
      digitalWrite(stepX, LOW);
      stepState = false;
      lastStepTime = currentTime;

      if (direction)
        currentPosition++;
      else
        currentPosition--;
    }
  }
}
