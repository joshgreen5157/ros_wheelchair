#include<Servo.h>
//Define the H-Bridge components of the L298N Motor Controller
#define ENA 5 //Define enable pin for one set of output pins
#define ENB 6 //Define enable pin for the other set of pins
#define IN1 7 //Define enable for Motor 0 rear right
#define IN2 8 //Define enable for Motor 1 front right
#define IN3 9 //Define enable for Motor 2 rear left
#define IN4 11 //Define enable for Motor 3 front left

//Define Servo
Servo myServo;
#define SRV 3

//Define pin for LED
#define LED 13 //Define 13 pin for LED
bool state = LOW; //The initial state of the function is defined as a low level
char getstr; //Defines a function that receives the Bluetooth character
int stepSize = 500;
bool stepDrive = true; bool impact = true; bool bForward = false; bool movement = false;
unsigned char carSpeed = 220; //define speed for motors (car)
char servoDir = 'm'; 

//Sonar Sensor
int Echo = A4;
int Trig = A5;
int lPos = 180, mPos = 90, rPos = 0;
int scanPos[3] = {60, 90, 120}; int scanCtr = 1;
int dist = 0; int lDist = 0; int rDist = 0;
int currentMillis, previousMillis, cmdMillis, cmdDelayMillis;
bool scan;

void setup() {
Serial.begin(9600); // start Serial communication
pinMode(LED, OUTPUT); //set LED pin to OUTPUT mode
pinMode(IN1,OUTPUT); //set H-Bridge pins to OUTPUT
pinMode(IN2,OUTPUT); // ''
pinMode(IN3,OUTPUT); // ''
pinMode(IN4,OUTPUT); // ''
pinMode(ENA,OUTPUT); // ''
pinMode(ENB,OUTPUT); // ''
pinMode(Echo, INPUT); //sonar receiver
pinMode(Trig,OUTPUT); //sonar sender
myServo.attach(SRV);
stop();
}

//wheels stop subfunction
void stop(){
digitalWrite(ENA,LOW);
digitalWrite(ENB,LOW);
Serial.println("Stop!");
bForward = false;
movement = false;
}

//forward movement sub function
void forward(){
digitalWrite(ENA,HIGH);
digitalWrite(ENB,HIGH);
digitalWrite(IN1,HIGH);
digitalWrite(IN2,LOW);
digitalWrite(IN3,LOW);
digitalWrite(IN4,HIGH);
Serial.println("Forward");
bForward = true;
movement = true;
btRead('v');
}
void backward(){
digitalWrite(ENA,HIGH);
digitalWrite(ENB,HIGH);
digitalWrite(IN1,LOW);
digitalWrite(IN2,HIGH);
digitalWrite(IN3,HIGH);
digitalWrite(IN4,LOW);
Serial.println("Backward");
bForward = false;
movement = true;
}
//forward movement sub function
void right(){
digitalWrite(ENA,HIGH);
digitalWrite(ENB,HIGH);
digitalWrite(IN1,HIGH);
digitalWrite(IN2,LOW);
digitalWrite(IN3,HIGH);
digitalWrite(IN4,LOW);
Serial.println("Right");
bForward = false;
movement = true;
}
//forward movement sub function
void left(){
digitalWrite(ENA,HIGH);
digitalWrite(ENB,HIGH);
digitalWrite(IN1,LOW);
digitalWrite(IN2,HIGH);
digitalWrite(IN3,LOW);
digitalWrite(IN4,HIGH);
Serial.println("Left");
bForward = false;
movement = true;
}
//Control LED sub function
void stateChange() {
state = !state;
digitalWrite(LED, state);
if (state) {
  Serial.println("Light On");}
  else {Serial.println("Light Off");
  }
}

void stepMode() {
  stepDrive = !stepDrive;
  if (!stepDrive) {
  Serial.println("Step Mode Disabled");}
  else {Serial.println("Step Mode Enabled");
  }
}

void moveServo(int ang) {
  switch (ang){
    case 0: Serial.println("Head Right"); break;
    case 90: Serial.println("Head Center"); break;
    case 180: Serial.println("Head Left"); break;
    default: break;
  }
  myServo.write(ang);
}

int distanceCheck(char dir) {
  //generate pulse signal
  digitalWrite(Trig,LOW); //baseline for the trigger command
  delayMicroseconds(2);
  digitalWrite(Trig,HIGH);
  delayMicroseconds(20);
  digitalWrite(Trig,LOW);
  //calculate the distance and convert to cm
  float fDistance = pulseIn(Echo,HIGH); //read the bounced signal
  fDistance = fDistance/58; //conver the time to cm
  switch (dir) {
    case 'r': Serial.println("Right distance is: "); break;
    case 'm': Serial.println("Middle distance is: "); break;
    case 'l': Serial.println("Right distance is: "); break;
  }
  Serial.println((int)fDistance);
  return (int)fDistance; //return as an integer to distance in cm
}
void btRead(char c) { //switch function activated from bluetooth
  switch (c){
  case 'a': stateChange(); break;
  case 'f': forward(); break;
  case 'b': backward(); break;
  case 's': stop(); break;
  case 'l': left(); break;
  case 'r': right(); break;
  case 'e': stepMode(); break;
  case 'm': moveServo(rPos); servoDir = 'r'; impact = false; break;
  case 'v': moveServo(mPos); servoDir = 'm'; impact = true; break;
  case 'z': moveServo(lPos); servoDir = 'l'; impact = false; break;
  case 'd': autoDrive(); break; //initiate autodrive function
  default: break;
  }
}

int sonar() {
  currentMillis = millis();
  if (currentMillis - previousMillis > 150){
    previousMillis = currentMillis;
    dist = distanceCheck(servoDir);
    //if robot is moving forward, sonar is forward, and distance is short then stop
    if (bForward && impact && dist < 20) {
        stop();
     }
  }
  scan = true;
  return dist;
}
void sonarScan() {
  //Modulate 60 degree scan in front of robot if moving forward
  if (bForward && scan = true){
    moveServo(scanPos[scanCtr]);
    scanCtr++;
    scan = false;  
}
void autoDrive() {
bool drive = true;
forward(); 
btRead('v');
  while(drive) {
    sonar();
    sonarScan();
    if (bForward == false) {
        checkDirection();
    }
    //code to break autodrive function
  if (Serial.available() != 0) {
    getstr = Serial.read();
    if (getstr == 'd') {
      drive = false;
    }
  }
  }
  stop();
}
void checkDirection() {
      //Check 5 angles, determine best route, then proceed
      int posArr[5] = {0, 45, 90, 135, 180};
      int distArr[5];
      int maxPos; int mx = 0;
      for (int i = 0; i < 5; i++) {
        moveServo(posArr[i]); delay(250);
        distArr[i] = sonar();
        if (distArr[i] > mx) {
          mx = distArr[i];
          maxPos = i;
        }
      }
      if (mx < 40) {
        backward();
      } else {
        switch (maxPos) {
          case 0: right(); delay(250);
          case 1: right(); delay(250); break;
          case 2: forward(); break;
          case 4: left(); delay(250);
          case 3: left(); delay(250); break;
          default: checkDirection();
        }
      }
}
void loop() {
//The Bluetooth Serial port to receive the data in the function
//If bluetooth is read, then activate switch function and save timestamp
if (Serial.available() > 0) {
  getstr = Serial.read(); 
   //if getstr does not effect movement of robot, do not reset delay
  btRead(getstr);
    if (movement) {
      cmdMillis = millis();
    }
}
//Stepmode time delay function in main loop
//take note of current time, subtract old time and compare to stepsize
if (stepDrive) {
  cmdDelayMillis = millis();
  if (cmdDelayMillis - cmdMillis > stepSize && movement) {
    stop();
  }
}
//Sonar read function
sonar();
sonarScan();

} // end of code
