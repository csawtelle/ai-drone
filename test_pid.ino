unsigned long lastTime;
float Input, Output, Setpoint;
float errSum, lastErr;
float kp, ki, kd;

void Compute() {
   unsigned long now = millis();
   float timeChange = (double)(now - lastTime);
   float error = Setpoint - Input;
   errSum += (error * timeChange);
   float dErr = (error - lastErr) / timeChange;
   Output = kp * error + ki * errSum + kd * dErr;
   lastErr = error;
   lastTime = now;
}
void init(float Kp, float Ki, float Kd) {
   kp = Kp;
   ki = Ki;
   kd = Kd;
}
