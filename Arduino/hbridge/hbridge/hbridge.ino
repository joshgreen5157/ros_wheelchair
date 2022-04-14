#include<Arduino.h>

//Pins
const int ENA = 8; //Define enable pin for Megamoto 1
const int LMF = 6; //Define pin for Left Motor Forward
const int LMR = 5; //Define pin for Left Motor Reverse
const int RMF = 10; //Define pin for Right Motor Forward
const int RMR = 9; //Define pin for Right Motor Reverse
const int JOYX = A0; const int JOYY = A1; //analog 0 and analog 1
const int BRAKED = 4; //brake Input
const int JOYPWR = 3;
const int JOYRELAY = 11;

//Range variables
int xCal, yCal;
const int turnReduction = 4;
const int motorRange = 100;
const int joyPlay = 100;
const int calibrationlevel = 10;
const int motorReduction = 3;

//Operational Functions
void brake(); void coast(); void motorLeft(int speed = 0); void motorRight(int speed = 0);
void writeMotors(int left, int right); void serialRead();
void readJoystick(); void calibrateJoystick(); void joystickPwr();

//timing variables
unsigned long timer, timer2, autoTimer; unsigned long cmdTimer = 0;
unsigned long joyTimer = 0; unsigned long autonomousTimeOut = 1000;
const int cmdDelay1 = 25;  const int cmdDelay2 = 10;
const int brakeDelay = 100; 
const int autoDelay = 50; const int joyDelay = 50;

//state variables
bool stateChanged = true;
int state = 0; //0: brake, 1: manual, 2: joystick, 3: autonomous;
bool joyReadyBool = false; bool autonomousModeBool = false;
bool joyModeBool = false; int joyPwrState = 0;

void setup() {
  Serial.begin(9600); // start Serial communication
  pinMode(ENA, OUTPUT);
  pinMode(LMF, OUTPUT); //set H-Bridge pins to OUTPUT
  pinMode(LMR, OUTPUT);
  pinMode(RMF, OUTPUT);
  pinMode(RMR, OUTPUT);
  pinMode(JOYX, INPUT);
  pinMode(JOYY, INPUT);
  pinMode(BRAKED, INPUT);
  pinMode(JOYPWR, INPUT);
  pinMode(JOYRELAY, OUTPUT);
  digitalWrite(JOYRELAY, HIGH);
}

void calibrateJoystick() {
  unsigned int xTot = 0; unsigned int yTot = 0;
  for (int i = 0; i < calibrationlevel; i++) {
    xTot = xTot + analogRead(JOYX);
    yTot = yTot + analogRead(JOYY);
  }
  unsigned int xAvg = xTot / calibrationlevel;
  unsigned int yAvg = yTot / calibrationlevel;
  int x = xAvg;
  int y = yAvg;
  Serial.print(x);
  Serial.print(y);
  //magic numbers, need to make tied to something
  if (xAvg < 475 || xAvg > 575) {
    xAvg = 525;
  }
  if (xAvg < 475 || yAvg > 575) {
    yAvg = 525;
  }
  xCal = xAvg;
  yCal = yAvg;
}

//wheels braking
void brake() {
  digitalWrite(ENA, LOW);
  motorLeft();
  motorRight();
}

//should be able to turn these into one function
void motorLeft(int speed) {
  if (speed == 0) {
    analogWrite(LMR, 0);
    analogWrite(LMF, 0);
  }
  else if (speed < 0 ) {
    analogWrite(LMR, -speed);
    analogWrite(LMF, 0);
  }
  else if (speed > 0) {
    analogWrite(LMR, 0);
    analogWrite(LMF, speed);
  }
};

void motorRight(int speed) {
  if (speed == 0) {
    analogWrite(RMR, 0);
    analogWrite(RMF, 0);
  }
  else if (speed < 0 ) {
    analogWrite(RMR, -speed);
    analogWrite(RMF, 0);
  }
  else if (speed > 0) {
    analogWrite(RMR, 0);
    analogWrite(RMF, speed);
  }
};

void writeMotors(int left, int right) {
  digitalWrite(ENA, HIGH);
  if (left < 255 && right < 255) {
    motorLeft(left);
    motorRight(right);
  }
}

void readJoystick() {
  if (timer - joyTimer > joyDelay) {
    joyTimer = millis();
    int xValue; int yValue;
    xValue = analogRead(JOYX) - xCal;
    yValue = analogRead(JOYY) - yCal;
    if (yValue / joyPlay == 0) {
      yValue =  0;
    }
    if (xValue / joyPlay == 0) {
      xValue = 0;
    }
    if (yValue != 0){ 
      yValue = yValue / turnReduction;
    }
    //     Serial.println((String)"XXXXXX: " + xValue + " || YYYYYY " + yValue);
    int leftPower = (yValue + xValue) / motorReduction; 
    int rightPower = (yValue - xValue) / motorReduction;
    //    Serial.println((String)"Left motor: " + leftPower + " || Right motor: " + rightPower);
    writeMotors(leftPower, rightPower);
  }
};

void autonomousMode() {
  if (Serial.available() > 0) {
    int LM; int RM;
    String cmd = Serial.readStringUntil('*');
    Serial.read();
    sscanf( cmd.c_str(), "%%%d&%d", &LM, &RM );
    writeMotors(LM, RM);
  }
};


void checkState() {
  int tempState;
  serialRead();
  joystickPwr();
  if (digitalRead(BRAKED)) {
    tempState = 0;
  }
  else if (autonomousModeBool) {
    tempState = 3;
  }
  else if (joyReadyBool == true) {
    tempState = 2;
  }
  else {
    tempState = 1;
  }
  if (tempState != state) {
    stateChanged = true;
    state = tempState;
    Serial.println((String)"***State Changed***    " + state);
    Serial.flush();
  };
};

void joystickPwr() {
  joyPwrState = digitalRead(JOYPWR);
  if (joyPwrState != joyReadyBool) {
    if (joyPwrState == HIGH) {
      calibrateJoystick();
      joyReadyBool = HIGH;
    }
    else //joyPwrState == LOW
    {
      joyReadyBool = LOW;
    }
  }
};

void serialRead() {
  if (Serial.available() > 0) {
    autonomousModeBool = true;
    autoTimer = millis();
  }
  else if (autonomousModeBool) {
    if (millis() - autoTimer > autonomousTimeOut) {
      autonomousModeBool = false;
    }
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
      case 0: //brake state
        brake();
        break;
      case 1: //manual state
        //      Serial.println("Manual");
        brake();
        break;
      case 2: //joystick state
        readJoystick();
        break;
      case 3: //autonomous state
        Serial.println("Autonomous");
        autonomousMode();
        break;
      default:
        //brake state
        brake();
        //      Serial.println("Braked");
        break;
    }
    if (stateChanged) {
      stateChanged = false;
    }
    cmdTimer = millis();
  }
} // end of code
