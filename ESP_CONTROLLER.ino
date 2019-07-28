#include <PID_v1.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include <Wire.h>

/* Connections:
   DIO 2 - Internal LED
   DIO 16 - Safety switch input
   DIO 17 - VESC output

   Operation:
   speed ~linearly related to pitch:
    - pitch tied to ThrTarget
    - ThrValue incremented up to ThrTarget
    - escThrottle (mapped ThrValue) averaged (msoothed) over 75 cycles
   if pitching forward, speed up
   if pitching back, slow down, then reverse
   if foot off safety switch, stop

   Functionality not yet implemented:
   if acceleration consistently upwards (ascending), increase pitch-speed ratio
   if acceleration consistently downwards (descending), decrease pitch-speed ratio
   if pitching back and going down a slope (ie fwdDir = true but pich < 0), brake (or reverse throttle)
   if pitching forward x amount, accelerate to y speed. if pitch returned to ~0, continue at speed y
   if pitching forward 2x amount, accelerate to 2y speed. if pitch returned to ~0, continue at speed y or 2y
   if pitching back, decelerate from speed from y or 2y, deceleration proportional to pitch angle
*/

// Acc/Gyro
MPU6050 mpu6050;

const uint16_t MPU = 0x68;
int16_t ax, ay, az, gx, gy, gz;
float zAccG, pitch, yGyro, rawPitch, gyroPitch, accPitch;
float previousTime, currentTime, elapsedTimeSecs;

// Misc
float thrValue = 0.0;
float thrTarget = 0.0;
int escThrottle = 0;
bool motorEn = false;
int firstRun = 1;

// Consts
const int logSize = 75;
const int LED = 2;
const int SAFE_SW = 16;
const int VESC_THR = 17;
float accLimit = 16384.0;
float gyroLimit = 131.0;
float pitchLog[logSize];
float throttleLog[logSize];

void setup() {
  Serial.begin(115200);
  
  ledcSetup(0, 5000, 8);
  ledcAttachPin(VESC_THR, 0);
  ledcWrite(0, 127);
  
  delay(10000);  
  Wire.begin();
  Wire.setClock(100000);
  Serial.println("I2C");

  // Custom offset
  mpu6050.initialize();  
  mpu6050.setXAccelOffset(-4114);
  mpu6050.setYAccelOffset(-24);
  mpu6050.setZAccelOffset(1667);
  mpu6050.setXGyroOffset(82);
  mpu6050.setYGyroOffset(-33);
  mpu6050.setZGyroOffset(8);
  
  pinMode(SAFE_SW, INPUT_PULLUP); // SAFETY INPUT
  pinMode(VESC_THR, OUTPUT); // VESC OUTPUT
  pinMode(LED, OUTPUT); // ONBOARD LED OUTPUT

  for (int i = 0; i < logSize; i++) {
    throttleLog[i] = 127;
  }
}

void loop() {
  previousTime = currentTime;
  currentTime = millis();
  elapsedTimeSecs = (currentTime - previousTime) / 1000.0;  
  
  // READ ACC & GYRO
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  
  yGyro = (float) gy / gyroLimit;
  yGyro = yGyro + 0.05; // Drift offset
  gyroPitch = gyroPitch + yGyro * elapsedTimeSecs;
  
  zAccG = 10.0 * (float) az / accLimit; // Vertical accel value
  accPitch = (float) atan2((-ax), sqrt(ay * ay + az * az)) * 57.3;
  pitch = 0.9 * gyroPitch + 0.1 * accPitch;
    
  pitchLog[0] = accPitch;
  //pitch = avgData(logSize, pitchLog);
    
  // SET TARGET THROTTLE
  if (digitalRead(SAFE_SW) == 0) {
    digitalWrite(LED, HIGH);
    if (pitch > -2 && pitch < 2 && motorEn == false) { // After leaning forward OK to move off    
      motorEn = true;
      Serial.println("MOTOR EN");
    } else if (motorEn == true) {
      thrTarget = mapf(pitch, -25.0, 25.0, -100.0, 100.0, -100.0, 100.0);
      thrValue = setThrottle(thrTarget, thrValue); // Increase/decrease throttle
    }
  } else {
    digitalWrite(LED, LOW);
    thrValue = 0.0;
    motorEn = false;
  }
  
  // WRITE THROTTLE VALUE
  escThrottle = mapf(thrValue, -100.0, 100.0, 0.0, 255, 0.0, 255.0); // Convert throttle value to DAC range
  throttleLog[0] = escThrottle; 
  escThrottle = avgData(logSize, throttleLog);
  ledcWrite(0, escThrottle); // Output analog data to ESC (0-255, 0-3.3V)
  
  // DEBUG OUTPUT
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print(escThrottle);
  Serial.println("");
    
  shiftData(logSize, throttleLog);
  shiftData(logSize, pitchLog);
  delay(10);
}

// ---------------------------------------------------------------------------------------

float setThrottle(float thrTarget, float thrValue) {
  if (thrTarget < 0) {
    if (thrTarget < thrValue) {
      thrValue = thrValue + ((thrTarget - thrValue)/3.0);
    } else if (thrTarget > thrValue) {
      thrValue = thrValue + (((thrTarget - thrValue)/2.0));
    }
  } else {
    if (thrTarget > thrValue) { // Increase throttle if pitching forward
      thrValue = thrValue + ((thrTarget - thrValue)/3.0);
    } else if (thrTarget < thrValue) { // Decrease throttle if pitching back
      thrValue = thrValue + (((thrTarget - thrValue)/2.0));
    }
  }
  return thrValue;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max, float lowLim, float upLim) {
  float out;
  out = (x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
  if (out >= upLim) {
    out = upLim;
  } else if (out <= lowLim) {
    out = lowLim;
  }
  return out;
}

float avgData(int bufSize, float bufArray[]) {
  float sum = 0.0;
  for (int i = 0; i < bufSize; i++) {
    sum = sum + bufArray[i];
  }
  sum = sum / bufSize;
  return sum;
}

void shiftData(int bufSize, float bufArray[]) {
  for (int i = (bufSize - 1); i > 0; i--) {
    bufArray[i] = bufArray[i - 1];
  }
}
