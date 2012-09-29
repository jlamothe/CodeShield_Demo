#include <Servo.h> 

/* Rotary encoder read example */
#define ENCODER_A 14
#define ENCODER_B 15
#define ENCODER_PORT PINC
#define SWITCH 13
#define BUTTON 12
#define RGB_RED 11
#define RGB_GREEN 10
#define RGB_BLUE 9
#define LED 6
#define SERVO 5
#define PIEZO 3
#define RELAY 2
#define POT 2
#define HALL 3
#define THERMISTOR 4
#define PHOTOCELL 5

Servo myservo;  // create servo object to control a servo 
static uint8_t counter = 1;      //this variable will be changed by encoder input
int switchState = LOW;
int lastRotValue;

void setup()
{
  /* Setup encoder pins as inputs */
  pinMode(ENCODER_A, INPUT);
  digitalWrite(ENCODER_A, HIGH);
  pinMode(ENCODER_B, INPUT);
  digitalWrite(ENCODER_B, HIGH);
  pinMode(SWITCH, INPUT);
  pinMode(BUTTON, INPUT);
  pinMode(RGB_RED, OUTPUT);
  pinMode(RGB_BLUE, OUTPUT);
  pinMode(RGB_GREEN, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(PIEZO, OUTPUT);
  pinMode(RELAY, OUTPUT);
 // myservo.attach(SERVO);  // attaches the servo on pin 9 to the servo object 
//  switchState = digitalRead(SWITCH);
  Serial.begin (115200);
  Serial.println("Start");
}
 
void loop()
{
  // Check if switchState has changed
  int switchCurrent = digitalRead(SWITCH);
  if (switchCurrent != switchState) {
    switchState = switchCurrent;
     digitalWrite(RGB_RED, LOW);
     digitalWrite(RGB_BLUE, LOW);
     digitalWrite(RGB_GREEN, LOW);
     if (switchState == HIGH) {
       myservo.attach(SERVO);
     } else {
       myservo.detach();
     } 
  }
  // Deal with switch state
  if (switchState == HIGH) {
  // Get encoder State
    int8_t tmpdata;
    tmpdata = read_encoder();
    if( tmpdata ) {
      counter += tmpdata;
    }
    int rotValue = ((counter / 4) % 4) + 1;
    if (rotValue != lastRotValue) {
      lastRotValue = rotValue;
      Serial.println(rotValue);
      doBeep(rotValue);
    }
    
   // digitalWrite(RGB_RED, HIGH);
    myservo.write(map( analogRead(POT), 0, 1023, 0, 179));                  // sets the servo position according to the scaled value 
    digitalWrite(RELAY, digitalRead(BUTTON));
    digitalWrite(RGB_BLUE, digitalRead(BUTTON));
  } else {
    //Serial.print("Hall: ");
    //Serial.println(analogRead(HALL));
    int tVal =  map(analogRead(THERMISTOR), 555, 590, 0, 255);
    //int tVal =  map(analogRead(POT), 0, 1024, 1, 254);
    if (tVal < 1) tVal = 0;
    if (tVal > 255) tVal = 255;
    Serial.println(tVal);  // 545 - 590
    analogWrite(RGB_RED, tVal);
    analogWrite(RGB_GREEN, 255-tVal);    //analogWrite(RGB_RED, 255 - tVal); 
    delay(100);
  }
}
 
/* returns change in encoder state (-1,0,1) */
int8_t read_encoder()
{
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;
  /**/
  old_AB <<= 2;                   //remember previous state
  old_AB |= ( ENCODER_PORT & 0x03 );  //add current state
  return ( enc_states[( old_AB & 0x0f )]);
}

void doBeep(int beeps) 
{
  Serial.println("doBeeps: "+String(beeps));
  for(int i=0; i<beeps; i++) {
    analogWrite(PIEZO, 128);
    delay(100);
    digitalWrite(PIEZO, LOW);
    delay(100);
  }
}

