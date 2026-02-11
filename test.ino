#include <AccelStepper.h> //loading needed library

int STEP_PIN = 3;//step signal pin on arduino
int DIR_PIN = 2;//direction signal pin on arduino
int POT_PIN = A0;//analog potentiometer pin on arduino
int MICROSTEP = 1;

AccelStepper motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

//PID control
float Kp = 3.0; //Proportional gain constant [Error]
float Ki = 0.002;// Integrated gain constant [Accumulated Error]
float Kd = 8.0; //Derivative gain constant [Error Rate]


//Memory for PID stuff
float integral = 0;
float previousError = 0;

long stepsPerRev = 200*MICROSTEP;
int deadband = 3; //If error is less than value, stop motor.
float maxSpeed = 2000;//This is in steps/sec

void setup() {
  // put your setup code here, to run once:
  motor.setMaxSpeed(maxSpeed);
  motor.setAcceleration(1500); //Limits acceleration
}

void loop() {
  // put your main code here, to run repeatedly:
  int potValue = analogRead(POT_PIN);
  long actualPosition = map(potValue,0,1023,0,stepsPerRev);//Converts ADC reading to motor step scale
  long targetPosition= stepsPerRev/2;//Target is middle. Can be changed with angle input!
  float error = targetPosition - actualPosition;

  if (abs(error) < deadband) {
    motor.setSpeed(0);
    integral = 0;
  }
  else {
   
    float P = Kp * error;

    integral += error;
    integral = constrain(integral,-10000,10000);
    float I = Ki * integral;

    float derivative = error - previousError;
    float D = Kd * derivative;

    float output = P + I + D; //lets goooo

    output = constrain(output,-maxSpeed,maxSpeed);

    motor.setSpeed(output);

    previousError = error;
   
  }

  motor.runSpeed();
}