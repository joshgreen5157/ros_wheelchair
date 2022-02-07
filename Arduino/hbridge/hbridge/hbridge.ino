
//Define the H-Bridge components of the L298N Motor Controller
#define ENA 8 //Define enable pin for Megamoto 1
#define LMF 6 //Define pin for Left Motor Forward
#define LMR 5 //Define pin for Left Motor Reverse
#define RMF 9 //Define pin for Right Motor Forward
#define RMR 10 //Define pin for Right Motor Reverse
#define JOYX A0 //Joystick X value
#define JOYY A1 //Joystick Y value

int joyRange = 1023;
int motorRange = 100;
char getstr; //Defines a function that receives the Bluetooth character


void setup() {
  Serial.begin(9600); // start Serial communication
  pinMode(ENA, OUTPUT);
  pinMode(LMF,OUTPUT); //set H-Bridge pins to OUTPUT
  pinMode(LMR,OUTPUT); 
  pinMode(RMF,OUTPUT); 
  pinMode(RMR, OUTPUT); 
  pinMode(JOYX, INPUT);
  pinMode(JOYY, INPUT);
}

//wheels braking
void brake(){
  digitalWrite(ENA, HIGH);
  motorLeft();
  motoRight();
}

void coast() {
  digitalWrite(ENA, LOW);
  //bring motors to 0
  motorLeft();
  motorRight();
}

//should be able to turn these into one function
void motorLeft(int speed=0){
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
}

void motorRight(int speed=0){
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
}

void writeMotors(int left, int right){
  leftMotor(left);
  rightMotor(right);
}

void serialRead() { //A is left/right, B is forward/back on joystick
  xValue = analogRead(joyX);
  yValue = analogRead(joyY);
  if (xValue != 0) 
  { xValue = xValue / 2;
  }
  leftInt = yValue + xValue;
  rightInt = yValue - xValue
  leftInt = map(leftInt, -joyRange, joyRange, -motorRange, motorRange);
  rightInt = map(rightInt, -joyRange, joyRange, -motorRange, motorRange);
  writeMotors(leftInt, rightInt);
}


//void btRead(char c) { //switch function activated from bluetooth
//  switch (c){
//  case 'a': stateChange(); break;
//  case 'f': forward(); break;
//  case 'b': backward(); break;
//  case 's': stop(); break;
//  case 'l': left(); break;
//  case 'r': right(); break;
//  case 'e': stepMode(); break;
//  case 'm': moveServo(rPos); servoDir = 'r'; impact = false; break;
//  case 'v': moveServo(mPos); servoDir = 'm'; impact = true; break;
//  case 'z': moveServo(lPos); servoDir = 'l'; impact = false; break;
//  case 'd': autoDrive(); break; //initiate autodrive function
//  default: break;
//  }
//}

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
//take note of current time, subtract old time and compare to stepsize
if (stepDrive) {
  cmdDelayMillis = millis();
  if (cmdDelayMillis - cmdMillis > stepSize && movement) {
    stop();
  }
}


} // end of code
