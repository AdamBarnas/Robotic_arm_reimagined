#include <Wire.h>
 
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define FREQUENCY             50

#define HAND 11
#define WRIST  12
#define ELBOW 13
#define SHOULDER 14
#define BASE 15

 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int potWrist = A3;
int potElbow = A2;                        //Assign Potentiometers to pins on Arduino Uno
int potShoulder = A1;
int potBase = A0;

int hand = 11;
int wrist = 12;
int elbow = 13;                           //Assign Motors to pins on Servo Driver Board
int shoulder = 14;
int base = 15;


int hand_last = 0;
int wrist_last = 0;
int elbow_last = 0;                      //remember last values for all axis
int shoulder_last = 0;
int base_last = 0;

void setup() 
{
  delay(5000);                            // <-- So I have time to get controller to starting position
  
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  pwm.setPWM(11, 0, 90);                  //Set Gripper to 90 degrees (Close Gripper)
  
  pinMode(13,INPUT_PULLUP);
  
  Serial.begin(9600);
}
 
 
void moveMotor(int controlIn, int motorOut)
{
  int pulse_wide, pulse_width, potVal;
  
  potVal = analogRead(controlIn);                                                   //Read value of Potentiometer
  switch (motorOut) {
    case HAND:
      Serial.print("hand     ");
      if (potVal == 0) potVal = hand_last;
      hand_last = potVal;
      break;

    case WRIST:
      Serial.print("wrist    ");
      if (potVal == 0) potVal = wrist_last;
      wrist_last = potVal;
      break;

    case ELBOW:
      Serial.print("elbow    ");
      if (potVal == 0) potVal = elbow_last;
      elbow_last = potVal;
      break;

    case SHOULDER:
      Serial.print("shoulder ");
      if (potVal == 0) potVal = shoulder_last;
      shoulder_last = potVal;
      break;

    case BASE:
      Serial.print("base     ");
      if (potVal == 0) potVal = base_last;
      base_last = potVal;
      break;  
  }
  Serial.println(potVal);

  if (potVal < 240) potVal = 240;
  if (potVal > 800) potVal = 800;
  
  pulse_wide = map(potVal, 800, 240, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);                //Map Potentiometer position to Motor
  
  pwm.setPWM(motorOut, 0, pulse_width);
 
}
 
void loop() 
{
  moveMotor(potWrist, wrist);
  
  moveMotor(potElbow, elbow);
                                                            //Assign Motors to corresponding Potentiometers
  moveMotor(potShoulder, shoulder);
  
  moveMotor(potBase, base);



  int pushButton = digitalRead(13);
  if(pushButton == LOW)
  {
    pwm.setPWM(hand, 0, 180);                             //Keep Gripper closed when button is not pressed
    Serial.println("Grab");
  }
  else
  {
    pwm.setPWM(hand, 0, 90);                              //Open Gripper when button is pressed
    Serial.println("Release");
  }
  // Serial.println("================");
  // delay(500);
  // Serial.println("================");
}