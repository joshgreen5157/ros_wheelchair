#include<Arduino.h>

//Pins
const int ENA = 8; //Define enable pin for Megamoto 1
const int LMF = 6; //Define pin for Left Motor Forward
const int LMR = 5; //Define pin for Left Motor Reverse
const int RMF = 9; //Define pin for Right Motor Forward
const int RMR = 10; //Define pin for Right Motor Reverse
const int JOYX = 0; const int JOYY = 1;

//Range variables
const int joyRange = 500;
const int turnReduction = 2;
const int motorRange = 100;
const int joyPlay = 10;

//Operational Functions
void brake(); void coast(); void motorLeft(int speed=0); void motorRight(int speed=0);
void writeMotors(int left, int right); void serialRead(); void readJoystick();

//timing variables
int cmdDelayMillis = 0; int readTimer = 500;

void setup() {
  Serial.begin(9600); // start Serial communication
  pinMode(ENA, OUTPUT);
  pinMode(LMF,OUTPUT); //set H-Bridge pins to OUTPUT
  pinMode(LMR,OUTPUT); 
  pinMode(RMF,OUTPUT); 
  pinMode(RMR, OUTPUT); 
  pinMode(JOYX, INPUT);
  pinMode(JOYY, INPUT);
  brake();
}

//wheels braking
void brake(){
  digitalWrite(ENA, HIGH);
  motorLeft();
  motorRight();
}

void coast() {
  digitalWrite(ENA, LOW);
  //bring motors to 0
  motorLeft();
  motorRight();
}

//should be able to turn these into one function
void motorLeft(int speed){
  if (speed == 0) {
      analogWrite(LMR, 0);
      analogWrite(LMF, 0);
    }
    else if (speed < 0 ) {
      analogWrite(LMR, -speed);
      analogWrite(LMF, 0);
    }
    else {
      analogWrite(LMR, 0);
      analogWrite(LMF, speed);
    }
  };

void motorRight(int speed){
  if (speed == 0) {
    analogWrite(RMR, 0);
    analogWrite(RMF, 0);
  }
    else if (speed < 0 ) {
      analogWrite(RMR, -speed);
      analogWrite(RMF, 0);
    }
    else {
      analogWrite(RMR, 0);
      analogWrite(RMF, speed);
    }
};

void writeMotors(int left, int right){
  motorLeft(left);
  motorRight(right);
}

void readJoystick() {
  // Serial.print("JOYX: ");
  // Serial.println(analogRead(JOYX));
  // Serial.print("JOYY: ");
  // Serial.println(analogRead(JOYY));
  //values read in at 0-1000, converting to -500 to 500
  Serial.println("Reading Joystick");
  int xValue; int yValue;
  xValue = analogRead(JOYX)-500;
  yValue = analogRead(JOYY)-500;
  if (yValue / joyPlay == 0) {
    yValue =  0;
  }
  if (xValue / joyPlay == 0) {
    xValue = 0;
  }
  if (xValue != 0) 
  { xValue = xValue / turnReduction;  }
}

void serialRead() { 

  int leftInt = yValue - xValue;
  int rightInt = yValue + xValue;
  leftInt = map(leftInt, -joyRange, joyRange, -motorRange, motorRange);
  rightInt = map(rightInt, -joyRange, joyRange, -motorRange, motorRange);
  writeMotors(leftInt, rightInt);
}

void checkState() {
  //copy current state to temporary variable
  //check pins for status
  //update state value
  //if state != state then do nothing
  //else go to run()

}

void run() {
  //switch case with state variable

}


void loop() {

  //if some signal:
  //run()
  //else delay for milli time

} // end of code
