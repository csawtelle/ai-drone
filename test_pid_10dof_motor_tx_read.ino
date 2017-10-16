#define NUM_RC_CHANNELS 6 //You need to specify how many pins you want to use
#include "PinChangeInt.h"  //If you need pinchangeint you need to include this header
const uint8_t RC_Channel_Pin[NUM_RC_CHANNELS]={8,9,10,11,12,13,};
uint16_t RC_Channel_Value[NUM_RC_CHANNELS];
#include "RCLib.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Simple_AHRS.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define LOWERBOUND  272 // this is the 'minimum' pulse length count (out of 4096)
#define UPPERRBOUND  450 // this is the 'maximum' pulse length count (out of 4096)
uint8_t initFlag = 0;
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
Adafruit_BMP085_Unified       bmp(18001);
Adafruit_Simple_AHRS          ahrs(&accel, &mag);

float input;

typedef struct {
  float kp = 3;
  float ki = .001;
  float kd = .001;
  float lastErr;
  float errSum;
  float setPoint;
  unsigned long lastTime;
  int pulseWidth=0;
} PID;

PID yaw;
PID pitch;
PID roll;

void initRotor() {
  for(int i=0; i < 6; i++) {
    pwm.setPWM(i, 0, UPPERRBOUND);
    delay(2);
    pwm.setPWM(i, 0, LOWERBOUND);
    delay(2);
    pwm.setPWM(i, 0, UPPERRBOUND);
    delay(2);
    pwm.setPWM(i, 0, LOWERBOUND);
    delay(2);
  }
};

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
  
  int one = 1*ppw+ LOWERBOUND;// + .5*rpw + -1*ypw + LOWERBOUND;
  int two = 1*ppw+ LOWERBOUND;// + -.5*rpw + 1*ypw + LOWERBOUND;
  int three = 0*ppw+ LOWERBOUND;// + -1*rpw + -1*ypw + LOWERBOUND;
  int four = -1*ppw+ LOWERBOUND;// + -.5*rpw + 1*ypw + LOWERBOUND;
  int five = -1*ppw+ LOWERBOUND;// + .5*rpw + -1*ypw + LOWERBOUND;
  int six = 0*ppw+ LOWERBOUND;// + .5*rpw + 1*ypw + LOWERBOUND;
  Serial.print(one);
  Serial.print(" ");
  Serial.print(two);
  Serial.print(" ");
  Serial.print(three);
  Serial.print(" ");
  Serial.print(four);
  Serial.print(" ");
  Serial.print(five);
  Serial.print(" ");
  Serial.println(six);
  if (one < LOWERBOUND){
    pwm.setPWM(0, 0, LOWERBOUND);
  } else {
    pwm.setPWM(0, 0, one);
  };
  if (two < LOWERBOUND){
    pwm.setPWM(1, 0, LOWERBOUND);
  } else {
    pwm.setPWM(1, 0, two);
  };
  if (three < LOWERBOUND){
    pwm.setPWM(2, 0, LOWERBOUND);
  } else {
    pwm.setPWM(2, 0, three);
  };
  if (four < LOWERBOUND){
    pwm.setPWM(3, 0, LOWERBOUND);
  } else {
    pwm.setPWM(3, 0, four);
  };
  if (five < LOWERBOUND){
    pwm.setPWM(4, 0, LOWERBOUND);
  } else {
    pwm.setPWM(4, 0, five);
  };
  if (six < LOWERBOUND){
    pwm.setPWM(5, 0, LOWERBOUND);
  } else {
    pwm.setPWM(5, 0, six);
  };
};

float compute(PID &p, int input) {
   unsigned long now = millis();
   //float deltaT = (float)(now - p.lastTime);
   float error = p.setPoint - input;
   //p.errSum += (error * deltaT);
   //float dErr = (error - p.lastErr) / deltaT;
   p.pulseWidth = p.kp * error;// + p.ki * p.errSum + p.kd * dErr;
   p.lastErr = error;
   p.lastTime = now;
};

void plot(int Data1, int Data2, int Data3, int Data4=0, int Data5=0, int Data6=0, int Data7=0, int Data8=0){
  Serial.print(Data1); 
  Serial.print(" ");
  Serial.print(Data2); 
  Serial.print(" ");
  Serial.print(Data3); 
  Serial.print(" ");
  Serial.print(Data4); 
  Serial.print(" ");
  Serial.print(Data5); 
  Serial.print(" ");
  Serial.println(Data6); 
}
void setup(){
  Serial.begin(115200);
  accel.begin();
  mag.begin();
  bmp.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  initRotor();
  sensors_vec_t   orientation;
  if (ahrs.getOrientation(&orientation)) {
    yaw.setPoint = orientation.heading;
    pitch.setPoint = orientation.pitch;
    roll.setPoint = orientation.roll;
  };
  Serial.println(F("Rc serial oscilloscope demo"));
  SetRCInterrupts(); //This method will do all the config foe you.
                    //Note some problems will be reported on the serial monitor
  Serial.println(F("Interrupts Set; starting "));
}

void loop(){
  int flag;
  sensors_vec_t   orientation;
  if (ahrs.getOrientation(&orientation)) {
    compute(roll, orientation.roll);
    compute(pitch, orientation.pitch);
    //compute(yaw, orientation.heading);
    updateRotor(roll, pitch, yaw);
  };
  if(flag=getChannelsReceiveInfo()) {
    plot(
      RC_Channel_Value[0],
      RC_Channel_Value[1],
      RC_Channel_Value[2],
      RC_Channel_Value[3],
      RC_Channel_Value[4],
      RC_Channel_Value[5]
    );
  }
}
