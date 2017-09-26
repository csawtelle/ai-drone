#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  250 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  430 // this is the 'maximum' pulse length count (out of 4096)
uint16_t incomingByte = SERVOMIN;
uint8_t initFlag = 0;
void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);
  yield();
}
void initRotor() {
    pwm.setPWM(0, 0, SERVOMAX);
    delay(2);
    pwm.setPWM(0, 0, SERVOMIN);
    delay(2);
    pwm.setPWM(0, 0, SERVOMAX);
    delay(2);
    pwm.setPWM(0, 0, SERVOMIN);
    delay(2);
    initFlag = 1;
}
void loop() {
    if (Serial.available() > 0) {
      if(initFlag == 0){
        initRotor();
      }
      incomingByte = Serial.parseInt();
      Serial.print("I received: ");
      Serial.println(incomingByte, DEC);
      pwm.setPWM(0, 0, incomingByte);
      Serial.println(incomingByte);
      delay(2);
    }
}
