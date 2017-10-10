#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Simple_AHRS.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  250 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  430 // this is the 'maximum' pulse length count (out of 4096)
uint16_t incomingByte = SERVOMIN;
uint8_t initFlag = 0;
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
Adafruit_BMP085_Unified       bmp(18001);
Adafruit_Simple_AHRS          ahrs(&accel, &mag);

float input;

typedef struct {
  float kp = .1;
  float ki = .1;
  float kd = .1;
  float lastErr;
  float errSum;
  float setPoint;
  float upperBound = 1300; 
  float lowerBound = 2100;
  unsigned long lastTime;
  int pulseWidth=lowerBound;
} PID;

PID yaw;
PID pitch;
PID roll;

void initRotor() {
  for(int i=0; x < 7; x++) {
    pwm.setPWM(i, 0, SERVOMAX);
    delay(2);
    pwm.setPWM(i, 0, SERVOMIN);
    delay(2);
    pwm.setPWM(i, 0, SERVOMAX);
    delay(2);
    pwm.setPWM(i, 0, SERVOMIN);
    delay(2);
  }
}
void updateRotor(PID &r, PID &p, PID &y) {
  /* ************ MIX ************
  Chan  Thro   Pitc   Roll   Yaw
  1      100    100     50  -100
  2      100    100    -50   100
  3      100      0   -100  -100
  4      100   -100    -50   100
  5      100   -100     50  -100
  6      100      0    100   100
  
  1 = 1*pitch + .5*roll + -1*yaw
  ...etc
  3 = 0*pitch + -1*roll + -1*yaw
  ...etc
  ****************************** */
  int rpw = r.pulseWidth;
  int ppw = p.pulseWidth;
  int ypw = y.pulseWidth;
  
  int one = 1*ppw + .5*rpw + -1*ypw
  int two = 1*ppw + -.5*rpw + 1*ypw
  int three = 0*ppw + -1*rpw + -1*ypw
  int four = -1*ppw + -.5*rpw + 1*ypw
  int five = -1*ppw + .5*rpw + -1*ypw
  int six = 0*ppw + .5*rpw + 1*ypw
  
  pwm.setPWM(1, 0, one);
  pwm.setPWM(2, 0, two);
  pwm.setPWM(3, 0, three);
  pwm.setPWM(4, 0, four);
  pwm.setPWM(5, 0, five);
  pwm.setPWM(6, 0, six);
}
float compute(PID &p, int input) {
   unsigned long now = millis();
   float deltaT = (float)(now - p.lastTime);
   float error = p.setPoint - input;
   p.errSum += (error * deltaT);
   float dErr = (error - p.lastErr) / deltaT;
   p.pulseWidth = p.kp * error + p.ki * p.errSum + p.kd * dErr;
   p.lastErr = error;
   p.lastTime = now;
}

void setup() {
  Serial.begin(115200);
  accel.begin();
  mag.begin();
  bmp.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  initRotor()
  yield();
}

void loop(void) {
  sensors_vec_t   orientation;
  if (ahrs.getOrientation(&orientation)) {
    compute(roll, orientation.roll);
    compute(pitch, orientation.pitch);
    compute(yaw, orientation.heading);
    updateRotor(roll, pitch, yaw);
  }
}
