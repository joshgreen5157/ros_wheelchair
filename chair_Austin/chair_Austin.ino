#define joyA 9
#define joyB 10
#define ECHO_PIN A4
#define TRIG_PIN A5
#define TRIG_PIN2 A7
#define ECHO_PIN2 A8

const int ObstacleDetection = 35;
const int powerPin = 4;
const int powerFeedback = 5;
int SW_state = 0;
char mode = 'm';
char data = '*';

int line_A = 130;
int line_B = 130;
int speed = 190;
int neutral = 140;
float kp = 2.5;
float ki = .1;
int wallDist = 50;
int counter;
float distance[]={0,0,0};
bool runOnce = false;

void setup() {
  pinMode(ECHO_PIN, INPUT);    
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);    
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(powerPin,OUTPUT);
  pinMode(powerFeedback,INPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(2, INPUT_PULLUP);

Serial.begin(9600);

// Serial3 is for bluetooth in pins TX3 and RX3
Serial3.begin(9600);
}

//Ultrasonic distance measurement Sub function
void Distance_test() {
  digitalWrite(TRIG_PIN, LOW);  
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(20);
  digitalWrite(TRIG_PIN, LOW);
  distance[0] = pulseIn(ECHO_PIN, HIGH)/74/2;
  
  digitalWrite(TRIG_PIN2, LOW);   
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN2, HIGH); 
  delayMicroseconds(20);
  digitalWrite(TRIG_PIN2, LOW);
  distance[1] = pulseIn(ECHO_PIN2, HIGH)/74/2;
}

//starts up chair on run
void startupProcedure() {
  stop();
  delay(300);
  digitalWrite (powerPin,HIGH);
  delay(100);
  digitalWrite (powerPin,LOW);
  delay(2500);
  }

//F(neutral,speed),B(neutral,30),R(50,neutral),L(220,neutral),S(neutral,neutral)
void move(int lineA, int lineB){       
  analogWrite(joyA,lineA);  
  analogWrite(joyB,lineB);
}

void stop(){
  move(140,140);
//  analogWrite(joyA,neutral);
//  analogWrite(joyB,neutral);
}

//PI controller calculations with limiters
float Pcalc(){
  Distance_test();
  if(distance[0] > 200){
    return 130;
  }

  if (distance[0] > 110){
    return 185;
  }
  
  delayMicroseconds(20);
  if(distance[0] > 70 && distance[0] < 110){
    return 130;
    }
  
  float err = distance[0] - wallDist;
  float alpha = kp * err + (ki * err) + 130;
  if (alpha > 220){
    alpha = 220;
  }
  if (alpha < 30){
    alpha = 30;
  }
//Serial.println(distance[0]);
  return alpha;
}

//toggles between auto and manual
void toggleRead(){
        if (digitalRead(2) == LOW){
          if (mode == 'u'){
            mode = 'm';
          }
          else{
            mode = 'u';
          }
          delay(300);
        }
}

void lineTracking(){
  while (mode == 'a'){
    reading();
    //delay(1);
    readingSerial();
    //Serial.println(data);
    //    Serial.println(mode);
    if(data=='0'){                      //forward
      move(neutral, 190);
    }
    else if(data=='1'){
      move(113,neutral);
    }
    else if(data=='2'){                     
      move(177, neutral);
    }
    else if (data == 's'){
      stop();
    }
    Distance_test();
    objectDetection('a');
   }
}


void objectDetection(char type){
  while(distance[1] < ObstacleDetection && distance[1] > 1 && mode == type){
        stop();
        reading();
        Distance_test();
        delay(10);
        toggleRead();
      }
}

void ultra(){
    move(neutral, speed);
    while(counter<3600 && mode == 'u'){
      Distance_test();
      line_A = Pcalc();
      move(line_A,speed);
      delay(10);
     // Serial.print("line_A: " );
      //Serial.println(line_A);
      counter++;
      //This reads toggle switching for auto or manual
      toggleRead();
      reading();
      // Obstacle detection and stop until object moves
      objectDetection('u');
    }
    stop();
    counter = 0;                                                                                                                                                                                                                               
}

void readingSerial(){
    while(Serial.available()){
      data = Serial.read();
    
    switch(data){
        case 'i': startupProcedure(); data = '*'; break;
        case 'm': mode = 'm'; break;
        case 'a': mode = 'a'; break;
        default:  break;
    }
    }
    //Serial.println(data);
}

void reading(){
    if(Serial3.available()){
      mode = Serial3.read();
    
     switch(mode){
        case 'p': startupProcedure(); mode = '*'; break;
        case 'f': move(neutral,speed); break;
        case 'b': move(neutral,30);   break;
        case 'l': move(180,neutral);   break;
        case 'r': move(80,neutral);  break;
        case 's': stop();
                  mode = 's';
                  break;
        case 'u': ultra(); break;
        case 'a': mode = 'a'; break;
        case 'y': speed += 10;
                  if (speed > 250) {
                  speed = 250;
                  }
                  break;
        case 'z': speed -= 10;
                  if (speed < 180) {
                  speed = 180;
                  }
                  break;
        default:  break;
    }
    }
    //Serial.println(mode);
    //mode = 'm';
}

void manualMode() {
  int xPosition = analogRead(A0);
  line_A = xPosition/8.0+80;
  int yPosition = analogRead(A1);
  line_B = yPosition/4.6+40;
  SW_state = digitalRead(2);
  delay(1);
  move(line_A,line_B);
 // Serial.print("lineA=");
 // Serial.println(line_A);
  }
  
void loop() {
  if(runOnce == false){
  startupProcedure();
  runOnce = true;
  }
  reading();
  //delay(10);
  readingSerial();
  //delay(10);
  //Serial.println(speed);
  toggleRead();
  if(mode == 'm'){
    manualMode(); 
    Distance_test();
   Serial.println(distance[0]);
    Serial.println(distance[1]);
    
  }
  else if(mode == 'u'){
    ultra();
    while(mode == 'u'){
      reading();
      toggleRead();
      delay(300);
    } 
  }
   else if(mode == 'a'){
    lineTracking();
  }
}
