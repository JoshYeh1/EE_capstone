//STEP 2: Get the Motor to oscillate __ degree given angle input

const int stepX = 2;
const int dirX = 5;

const int microstep = 1;
const int motor_steps = 200; //number of steps in the motor 
const int steps_per_rev = motor_steps * microstep; //scales steps depending on microsteps

const int step_delay = 800; //changable delay in us

//Given this delay, we can find our RPM.
// 1 period is 800 + 800 = 1600 us
// multiply by 1,000,000 to get in sec  ==> 625 steps/sec
// Divide by steps per rev as found above. 625/200 (assuming no microstep)
// Thus, motor runs at abou 3.125 RPM

float oscillation_angle = 360.0; //degrees

int oscillation_steps; //creating empty var

void setup() {
  // put your setup code here, to run once:
  pinMode(stepX,OUTPUT); //initialize step pin
  pinMode(dirX,OUTPUT); //initialize direction pin
  digitalWrite(dirX,HIGH); //sets spinning direction

  oscillation_steps = (oscillation_angle/360.0) * steps_per_rev; //determines amount of steps needed in order to reach desred angle
}

void moveSteps(int steps, bool direction) { //reasuable motion function with steps and direction and input
  digitalWrite(dirX,direction);

  for (int i = 0; i < steps; i++) {
   digitalWrite(stepX, HIGH);
   delayMicroseconds(step_delay);
   digitalWrite(stepX,LOW);
   delayMicroseconds(step_delay);
  }
}

void loop() {
  moveSteps(oscillation_steps,HIGH); //Calls function and moves motor
  delay(200);
  moveSteps(oscillation_steps,LOW);
  delay(200); //one second delau
}

