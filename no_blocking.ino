// STEP 3: Get the Motor to oscillate __ degree given angle input includes microstepping!
// Non-blocking timing using micros()

const int stepX = 2;
const int dirX  = 5;

const int microstep = 1;
const int motor_steps = 200; //number of steps in motor
const int steps_per_rev = motor_steps * microstep; //scales steps depending on microstepping

const unsigned long step_delay = 800; // changable delay in microseconds 

float oscillation_angle = 360.0; //CHANGE THIS
int oscillation_steps; //empty var 

unsigned long lastStepTime = 0;
bool stepState = false;// HIGH or LOW state of step pin
bool direction = HIGH;// current direction

int stepsTaken = 0;// steps completed in current move
bool moving = true;// motor currently moving

void setup() {
  pinMode(stepX, OUTPUT);
  pinMode(dirX, OUTPUT);

  oscillation_steps = (oscillation_angle / 360.0) * steps_per_rev;

  digitalWrite(dirX, direction);
}

void loop() {

  unsigned long currentTime = micros(); //mircos gives us current time

  if (moving) {

    if (!stepState && (currentTime - lastStepTime >= step_delay)) { //When in ste HIGH
      digitalWrite(stepX, HIGH);
      stepState = true;
      lastStepTime = currentTime;
    }

    else if (stepState && (currentTime - lastStepTime >= step_delay)) { //when in step LOW
      digitalWrite(stepX, LOW);
      stepState = false;
      lastStepTime = currentTime;

      stepsTaken++;//add step to step counter

      if (stepsTaken >= oscillation_steps) {
        moving = false;// finished this move
        stepsTaken = 0;

        direction = !direction;// reverse direction
        digitalWrite(dirX, direction);
      }
    }
  }

  // small pause between oscillations (non-blocking)
  static unsigned long pauseStart = 0;

  if (!moving) {
    if (pauseStart == 0) {
      pauseStart = millis();
    }

    if (millis() - pauseStart >= 200) {
      moving = true;
      pauseStart = 0;
    }
  }
}