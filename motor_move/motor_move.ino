//STEP 1: Get the Motor to rotate around.

const int stepX = 2;
const int dirX = 5;

void setup() {
  // put your setup code here, to run once:
  pinMode(stepX,OUTPUT); //initialize step pin
  pinMode(dirX,OUTPUT); //initialize direction pin
  digitalWrite(dirX,HIGH); //sets spinning direction
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int x = 0; x < 200; x++) {
    //This part creates a square input wafe using HIGH LOW and delay.
    digitalWrite(stepX, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepX, LOW);
    delayMicroseconds(1000);

  }
  delay(1000); //one second delau
}
