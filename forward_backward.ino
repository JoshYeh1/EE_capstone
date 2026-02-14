const int StepX = 2;
const int DirX = 5;
const int StepY = 3;
const int DirY = 6;

int angle = 45;
float micro = 1;
float totalSteps = 200 * micro;
float resolution = 360/totalSteps;
float steps = angle/resolution;
int vel = 15000;
int dir1 = HIGH;
int dir2 = LOW;

void setup() {
  pinMode(StepX,OUTPUT);
  pinMode(DirX,OUTPUT);
  pinMode(StepY,OUTPUT);
  pinMode(DirY,OUTPUT);
}


//void motorA(a) {
  
//}

void loop() {

  //motorA(15);
  //motorA(5);
  //motorA(20);
  
  for (int i = 0; i< 2; i++) {
  
    digitalWrite(DirX, dir1); //counterclockwise
  
    for(int x = 0; x<steps; x++) {
      digitalWrite(StepX,HIGH);
      delayMicroseconds(vel);
      digitalWrite(StepX,LOW); 
      delayMicroseconds(vel);
   }
  delay(1000); // delay for 1 second
    
  digitalWrite(DirX, dir2); // clockwise
    
    for(int x = 0; x<steps; x++) {
      digitalWrite(StepX,HIGH);
      delayMicroseconds(vel);
      digitalWrite(StepX,LOW); 
      delayMicroseconds(vel);
   }
  delay(1000); // delay for 1 second
  
    digitalWrite(DirX, dir2); // clockwise
    
    for(int x = 0; x<steps; x++) {
      digitalWrite(StepX,HIGH);
      delayMicroseconds(vel);
      digitalWrite(StepX,LOW); 
      delayMicroseconds(vel);
   }
  delay(1000); // delay for 1 second
  
    digitalWrite(DirX, dir1); // counterclockwise
    
    for(int x = 0; x<steps; x++) {
      digitalWrite(StepX,HIGH);
      delayMicroseconds(vel);
      digitalWrite(StepX,LOW); 
      delayMicroseconds(vel);
   }
  delay(1000); // delay for 1 second




  digitalWrite(DirY, dir2); //counterclockwise
  
    for(int y = 0; y<steps; y++) {
      digitalWrite(StepY,HIGH);
      delayMicroseconds(vel);
      digitalWrite(StepY,LOW); 
      delayMicroseconds(vel);
   }
  delay(1000); // delay for 1 second
    
  digitalWrite(DirY, dir1); // clockwise
    
    for(int y = 0; y<steps; y++) {
      digitalWrite(StepY,HIGH);
      delayMicroseconds(vel);
      digitalWrite(StepY,LOW); 
      delayMicroseconds(vel);
   }
  delay(1000); // delay for 1 second
  
    digitalWrite(DirY, dir1); // clockwise
    
    for(int y = 0; y<steps; y++) {
      digitalWrite(StepY,HIGH);
      delayMicroseconds(vel);
      digitalWrite(StepY,LOW); 
      delayMicroseconds(vel);
   }
  delay(1000); // delay for 1 second
  
    digitalWrite(DirY, dir2); // counterclockwise
    
    for(int y = 0; y<steps; y++) {
      digitalWrite(StepY,HIGH);
      delayMicroseconds(vel);
      digitalWrite(StepY,LOW); 
      delayMicroseconds(vel);
   }
  delay(1000); // delay for 1 second

  }
}
