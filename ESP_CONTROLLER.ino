#include <MPU6050.h>
#include <Kalman.h>
#include <I2Cdev.h>
#include <Wire.h>

#define MPU6050_ACCEL_FS_2 0x00
#define MPU6050_ACCEL_FS_4 0x01
#define MPU6050_ACCEL_FS_8 0x02
#define MPU6050_ACCEL_FS_16 0x03

#define MPU6050_GYRO_FS_250 0x00
#define MPU6050_GYRO_FS_500 0x01
#define MPU6050_GYRO_FS_1000 0x02
#define MPU6050_GYRO_FS_2000 0x03

/* Connections:
   DIO 2 - Internal LED
   DIO 15 - MPU interrupt
   DIO 16 - Safety switch input
   DIO 17 - VESC output

   Operation:
   speed tied to pitch through PD controller
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
float zAccG, pitch, yGyro, gyroPitch, accPitch;
float previousTime, currentTime, elapsedTimeSecs;

// Kalman & PID
Kalman kalman;
uint32_t timer;
double Kp = 1.0, Ki = 0.0005, Kd = 0.05;

// Misc
float thrValue = 0.0;
float thrTarget = 0.0;
int escThrottle = 0;
bool motorEn = false;
bool firstRun = true;

// Consts
const int logSize = 25;
const int LED = 2;
const int INT = 15;
const int SAFE_SW = 16;
const int VESC_THR = 17;
float accLimit = 16384.0;
float gyroLimit = 32.8;
float throttleLog[logSize];

void setup() {
  Serial.begin(115200);
  
  ledcSetup(0, 5000, 8);
  ledcAttachPin(VESC_THR, 0);
  ledcWrite(0, 127);
  
  delay(10000);
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
  
  pinMode(SAFE_SW, INPUT_PULLUP); // SAFETY INPUT
  pinMode(VESC_THR, OUTPUT); // VESC OUTPUT
  pinMode(LED, OUTPUT); // ONBOARD LED OUTPUT

  kalman.setAngle(pitch);
}

// ---------------------------------------------------------------------------------------

void loop() {
  
  previousTime = currentTime;
  currentTime = millis();
  elapsedTimeSecs = (currentTime - previousTime) / 1000.0;  
  
  // SET TARGET THROTTLE
  yGyro = getGyRaw();
  accPitch = getAccPitch();
  
  zAccG = getAccVal();
  //zAccG = az * 0.5 + (zAccG * 0.5); // Low pass filter
  
  // If deemed to be level, horizontally, (given by vertical g (m/s) value & accPitch) then begin accepting gyro data
  if (firstRun == true) {
    if (zAccG >= 9.7 && pitch >= -1 && pitch <= 1) {
      firstRun = false;
      Serial.println("GYRO INIT");
    } else {
      pitch = accPitch;
    }
  } else {
    gyroPitch = getGyPitch(yGyro);
    pitch = getCompPitch(gyroPitch, accPitch);
  }
  pitch = getKalPitch(pitch);
  
  if (digitalRead(SAFE_SW) == 0) {
    digitalWrite(LED, HIGH);
    if (pitch > -3 && pitch < 3 && motorEn == false) { // After leaning to level OK to move off
      motorEn = true;
      Serial.println("MOTOR EN");
    } else if (motorEn == true) {
      thrTarget = pitch * Kp + yGyro * Kd;
      thrValue = mapf(thrTarget, -25.0, 25.0, 0.0, 255.0, 0.0, 255.0);
    }
  } else {
    digitalWrite(LED, LOW);
    thrValue = 127.0;
    motorEn = false;
    firstRun = true;
  }

  /*Serial.print(pitch);
  Serial.print("\t");
  Serial.print(thrTarget);
  Serial.println();*/
  
  // WRITE THROTTLE VALUE
  setThrottle(thrValue);
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
  yGyro = yGyro + 0.02; // Drift offset
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

float getCompPitch(float gyro, float acc) {  
  return pitch = 0.75 * gyro + 0.25 * acc;
}

float getKalPitch(float pitch) {
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  float kalAngleY = kalman.getAngle(pitch, (gy / 131.0), dt);
  return kalAngleY;
}

void setThrottle(float throttle) {
  throttleLog[0] = throttle;
  throttle = avgData(logSize, throttleLog);
  shiftData(logSize, throttleLog);
  ledcWrite(0, throttle); // Output analog data to ESC (0-255, 0-3.3V)
}
