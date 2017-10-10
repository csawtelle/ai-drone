#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Simple_AHRS.h>

// Create sensor instances.
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
Adafruit_BMP085_Unified       bmp(18001);

// Create simple AHRS algorithm using the above sensors.
Adafruit_Simple_AHRS          ahrs(&accel, &mag);

float input;

typedef struct {
  float kp = 1;
  float ki = 1;
  float kd = 1;
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

float compute(PID &p, int input) {
   unsigned long now = millis();
   Serial.print("Now: ");
   Serial.print(now);
   float deltaT = (float)(now - p.lastTime);
   Serial.print("  dt: ");
   Serial.print(deltaT);
   float error = p.setPoint - input;
   Serial.print("  e: ");
   Serial.print(error);
   p.errSum += (error * deltaT);
   Serial.print("  ers: ");
   Serial.println(p.errSum);
   float dErr = (error - p.lastErr) / deltaT;
   p.pulseWidth = p.kp * error + p.ki * p.errSum + p.kd * dErr;
   p.lastErr = error;
   p.lastTime = now;
}

void setup()
{
  Serial.begin(115200);
  Serial.println(F("Adafruit 10 DOF Board AHRS Example")); Serial.println("");
  
  // Initialize the sensors.
  accel.begin();
  mag.begin();
  bmp.begin();
}

void loop(void)
{
  sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    compute(roll, orientation.roll);
    Serial.print("   pulse   ");
    Serial.print(roll.pulseWidth);
    Serial.print("   ");
    /*compute(pitch, orientation.pitch);
    compute(yaw, orientation.heading);*/
  }
  delay(100);
}
