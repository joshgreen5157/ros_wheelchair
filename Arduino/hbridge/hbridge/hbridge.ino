#include<Arduino.h>
#include<String.h>

//Pins
const int ENA = 8; //Define enable pin for Megamoto 1
const int LMF = 5; //Define pin for Left Motor Forward
const int LMR = 6; //Define pin for Left Motor Reverse
const int RMF = 9; //Define pin for Right Motor Forward
const int RMR = 10; //Define pin for Right Motor Reverse
const int JOYX = 0; const int JOYY = 1; //analog 0 and analog 1
const int BRAKED = 13; //brake Input
const int JOYBTN = 3;

//Range variables
int xCal, yCal;
const int turnReduction = 2;
const int motorRange = 100;
const int joyPlay = 30;
const int calibrationlevel = 10;

//Operational Functions
void brake(); void coast(); void motorLeft(int speed=0); void motorRight(int speed=0);
void writeMotors(int left, int right); void serialRead(); 
void readJoystick(); void calibrateJoystick(); void joystickButton();

//timing variables
unsigned long timer; unsigned long timer2; unsigned long cmdTimer = 0; unsigned long joyTimer = 0;
const int cmdDelay1 = 250;  const int cmdDelay2 = 25;
const int brakeDelay = 100; 
const int autoDelay = 500; const int joyDelay = 500; 

//state variables
bool stateChanged = true;
int state = 0; //0: brake, 1: manual, 2: joystick, 3: autonomous;
bool joyReadyBool = false; bool autonomousModeBool = false;
bool joyModeBool = false; int joyBtnState = 0; int lastJoyBtnState = 0;

void setup() {
  Serial.begin(9600); // start Serial communication
  pinMode(ENA, OUTPUT);
  pinMode(LMF,OUTPUT); //set H-Bridge pins to OUTPUT
  pinMode(LMR,OUTPUT); 
  pinMode(RMF,OUTPUT); 
  pinMode(RMR, OUTPUT); 
  pinMode(JOYX, INPUT);
  pinMode(JOYY, INPUT);
  pinMode(BRAKED, INPUT);
  pinMode(JOYBTN, INPUT);
  calibrateJoystick();
}

void calibrateJoystick() {
  unsigned int xTot= 0; unsigned int yTot = 0;
  for (int i = 0; i < calibrationlevel; i++) {
    xTot = xTot + analogRead(JOYX);
    yTot = yTot + analogRead(JOYY);
  }
  unsigned int xAvg = xTot / calibrationlevel;
  unsigned int yAvg = yTot / calibrationlevel;
  //magic numbers, need to make tied to something
  if (xAvg < 475 || xAvg > 575) {
    xAvg = 525;
  }
  if (xAvg < 475 || yAvg > 575) {
    yAvg = 525;
  }
  xCal = xAvg;
  yCal = yAvg;
  joyReadyBool = true;
  joyModeBool = true;
  }

//wheels braking
void brake(){
  digitalWrite(ENA, LOW);
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
      Serial.println((String)"Writing LEFT motor REVERSE: " + speed);
      analogWrite(LMR, -speed);
      analogWrite(LMF, 0);
    }
    else if (speed > 0) {
      analogWrite(LMR, 0);
      analogWrite(LMF, speed);
    }
  };

void motorRight(int speed=0){
  if (speed == 0) {
    analogWrite(RMR, 0);
    analogWrite(RMF, 0);
  }
    else if (speed < 0 ) {
      Serial.println((String)"Writing RIGHT motor REVERSE: " + speed);
      analogWrite(RMR, -speed);
      analogWrite(RMF, 0);
    }
    else if (speed > 0) {
      analogWrite(RMR, 0);
      analogWrite(RMF, speed);
    }
};

void writeMotors(int left, int right){
  digitalWrite(ENA, HIGH);
  motorLeft(left/8);
  motorRight(right/8);
}

void readJoystick() {
  if (timer - joyTimer > joyDelay) {
    joyTimer = millis();;
    int xValue; int yValue;
    xValue = analogRead(JOYX)-xCal;
    yValue = analogRead(JOYY)-yCal;
    if (yValue / joyPlay == 0) {
      yValue =  0;
    }
    if (xValue / joyPlay == 0) {
      xValue = 0;
    }
    if (xValue != 0) 
    { xValue = xValue / turnReduction;  }
     Serial.println((String)"XXXXXX: " + xValue + " || YYYYYY " + yValue);
    int leftPower = yValue + xValue; int rightPower = yValue - xValue;
    Serial.println((String)"Left motor: " + leftPower + " || Right motor: " + rightPower);
    writeMotors(leftPower, rightPower);
}}

void autonomousMode() {
  Serial.read();
}


void checkState() {
  int tempState;
  serialRead();
  joystickButton();
  bool BRAKED=false;
  if (BRAKED== true) {
    tempState = 0;
  }
  else if (joyReadyBool == true && joyModeBool == true) {
    tempState = 2;
  }
  else if (autonomousModeBool == true && joyModeBool == false) {
    tempState = 3;
  }
  else {
    tempState = 1;
  }
  if (tempState != state) {
    stateChanged = true;
    state = tempState;
    Serial.println((String)"***State Changed***    " + state + state + state + state );
    
  }
}

void joystickButton() {
  joyBtnState = digitalRead(JOYBTN);

  if (joyBtnState != lastJoyBtnState) {
    if (joyBtnState == HIGH) {
      joyModeBool = true;
    }
    else {
      joyModeBool = false;
    }
  }
  lastJoyBtnState = joyBtnState;
}

void serialRead() {
  if (Serial.available() > 0) {
    autonomousModeBool = true; 
  }
}

void loop() {
  timer = millis();
  if (timer - timer2 > cmdDelay2) {
    checkState();
    timer2 = millis();
  }
  if (timer - cmdDelay1 > cmdTimer) {
    switch (state) {
      case 1: //manual state
      Serial.println("Manual");
      brake();
      break;
      case 2: //joystick state
      readJoystick();
      break;
      case 3: //autonomous state
      Serial.println("Autonomous");
      break;
      default:
      //brake state
      brake();
      Serial.println("Braked");
      break;
    }
    if (stateChanged) {
      stateChanged = false;
    }
    cmdTimer = millis();
  }
} // end of code
