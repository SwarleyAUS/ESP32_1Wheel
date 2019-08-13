#include <datatypes.h>
#include <VescUart.h>
#include <MPU6050.h>
#include <Kalman.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <Arduino.h>
#include <analogWrite.h>

#define MPU6050_GYRO_FS_1000 0x02
#define MPU6050_GYRO_FS_2000 0x03

/* Connections:
   DIO 2 - Internal LED
   DIO 14 - Safety switch input 1
   DIO 12 - Safety switch input 2
   DIO 27 - VESC output

   Operation:
   speed tied to pitch through PD controller
   if pitching forward, speed up
   if pitching back, slow down, then reverse
   if foot off safety switch, stop
*/

// Acc/Gyro
MPU6050 mpu6050;
const uint16_t MPU = 0x68;
int16_t ax, ay, az, gx, gy, gz;
float zAccG, pitch, yGyro, gyroPitch, accPitch, compPitch;
float previousTime, currentTime, elapsedTimeSecs;

// Kalman & PID
Kalman kalman;
uint32_t timer;
double Kp = 1.75, Kd = 0.05;

// Misc
float thrValue = 96;
float thrTarget = 0.0;
float prevThrottleValue = 0.0;
int escThrottle = 0;
bool motorEn = false;
bool firstRun = true;

// Consts
const int LED = 2;
const int SAFE_SW_1 = 14;
const int SAFE_SW_2 = 12;
const int VESC_THR = 27;
float accLimit = 16384.0;
float gyroLimit = 32.8;

struct bldcMeasure measuredValues;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); // VESC
  Serial2.begin(9600); // BT module
  
  ledcSetup(0, 5000, 8);
  ledcAttachPin(VESC_THR, 0);
  ledcWrite(0, 96);
  
  Wire.begin();
  Wire.setClock(400000);
  
  // Custom offset
  mpu6050.initialize();
  mpu6050.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
  mpu6050.setXAccelOffset(-4203);
  mpu6050.setYAccelOffset(-21);
  mpu6050.setZAccelOffset(1673);
  mpu6050.setXGyroOffset(93);
  mpu6050.setYGyroOffset(-17);
  mpu6050.setZGyroOffset(4);
  
  pinMode(SAFE_SW_1, INPUT); // SAFETY INPUT 1
  pinMode(SAFE_SW_2, INPUT); // SAFETY INPUT 2
  pinMode(VESC_THR, OUTPUT); // VESC OUTPUT
  pinMode(LED, OUTPUT); // ONBOARD LED OUTPUT
  
  kalman.setAngle(pitch);
}

// ---------------------------------------------------------------------------------------

void loop() {
  /*if (VescUartGetValue(measuredValues)) {
    SerialPrint(measuredValues);
  }*/
  
  previousTime = currentTime;
  currentTime = millis();
  elapsedTimeSecs = (currentTime - previousTime) / 1000.0;
  
  yGyro = getGyRaw();
  accPitch = getAccPitch();
  zAccG = getAccVal();
  //zAccG = az * 0.5 + (zAccG * 0.5); // Low pass filter
  
  // If deemed to be level, horizontally, (given by vertical g (m/s) value & accPitch) then begin accepting Gyro data
  if (firstRun == true) {
    if (zAccG >= 9.7 && accPitch >= -0.25 && accPitch <= 0.25) {
      firstRun = false;
      Serial.println("GYRO INIT");
    } else {
      gyroPitch = 0;
      pitch = accPitch;
    }
  } else {
    gyroPitch = getGyPitch(yGyro);
    pitch = getCompPitch(gyroPitch, accPitch, pitch);
    //pitch = getKalPitch(pitch); // Slows by ~1500 microseconds when enabled
  }

  //if (analogRead(SAFE_SW_1) <= 1000 && analogRead(SAFE_SW_2) <= 1000) {
  if (analogRead(SAFE_SW_1) <= 1000) {
    digitalWrite(LED, HIGH);
    if (pitch > -3 && pitch < 3 && motorEn == false) { // After leaning to level OK to move off
      motorEn = true;
    } 
    if (motorEn == true) {
      thrTarget = pitch * Kp + yGyro * Kd;
      // Adjusted range such that VESC ADC floating voltage and throttle voltage centre are matched (~1.25V):
      thrValue = mapf(thrTarget, -20.0, 20.0, 0.0, 192.0, 0.0, 192.0); 
      thrValue = thrValue * 0.5 + prevThrottleValue * 0.5;
    }
  } else {
    digitalWrite(LED, LOW);
    thrValue = 96.0;
    motorEn = false;
    firstRun = true;
  }

  //Serial.print(pitch * Kp);
  //Serial.print("\t");
  //Serial.print(yGyro * Kd);
  //Serial.print("\t");
  //Serial.print(pitch);
  //Serial.print("\t");
  //Serial.print(thrValue);
  //Serial.println();
  
  //Serial2.print(pitch);
  //Serial2.print("\t");
  //Serial2.print(thrValue);
  //Serial2.print("\t");
  //Serial2.print(measuredValues.dutyCycleNow);
  //Serial2.print("\t");
  //Serial2.print(measuredValues.avgInputCurrent);
  //Serial2.println();
  
  // WRITE THROTTLE VALUE
  setThrottle(thrValue);
  prevThrottleValue = thrValue;
}

// ---------------------------------------------------------------------------------------

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

float getGyRaw() {
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  
  yGyro = (float) gy / gyroLimit;
  yGyro = yGyro - 0.08; // Drift offset
  return yGyro;
}

float getGyPitch(float gyro) {
  return gyroPitch = gyroPitch + gyro * elapsedTimeSecs;
}

float getAccPitch() {
  return accPitch = (float) atan2((-ax), sqrt(ay * ay + az * az)) * 57.3;
}

float getAccVal() {
  return zAccG = 9.8 * (float) az / accLimit; // Vertical accel value
}

float getCompPitch(float gyro, float acc, float pitch) {
  return compPitch = 0.8 * gyro + 0.2 * acc;
}

float getKalPitch(float pitch) {
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  float kalAngleY = kalman.getAngle(pitch, (gy / 131.0), dt);
  return kalAngleY;
}

void setThrottle(float throttle) {
  ledcWrite(0, throttle); // Output analog data to ESC (0-255, 0-3.3V)
  //throttle = mapf(throttle, 0.0, 192.0, -1.0, 1.0, -1.0, 1.0);
  //VescUartSetDuty(throttle);
}
