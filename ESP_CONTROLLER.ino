#include <MPU6050.h>
#include <Kalman.h>
#include <I2Cdev.h>
#include <Wire.h>

#define MPU6050_GYRO_FS_1000 0x02
#define MPU6050_GYRO_FS_2000 0x03

/* Connections:
   DIO 2 - Internal LED
   DIO 16 - Safety switch input
   DIO 17 - VESC output

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
float zAccG, pitch, yGyro, gyroPitch, accPitch,;
float previousTime, currentTime, elapsedTimeSecs;

// Kalman & PID
Kalman kalman;
uint32_t timer;
double Kp = 1.1, Ki = 0.0005, Kd = 0.8;

// Misc
float thrValue = 0.0;
float thrTarget = 0.0;
int escThrottle = 0;
bool motorEn = false;
bool firstRun = true;

// Consts
const int LED = 2;
const int SAFE_SW_1 = 16; // Change to 12
const int SAFE_SW_2 = 13;
const int VESC_THR = 17;
float accLimit = 16384.0;
float gyroLimit = 32.8;

void setup() {
  Serial.begin(115200);
  
  ledcSetup(0, 5000, 8);
  ledcAttachPin(VESC_THR, 0);
  ledcWrite(0, 127);
  
  //delay(10000);
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
  
  pinMode(SAFE_SW_1, INPUT_PULLUP); // SAFETY INPUT 1
  pinMode(SAFE_SW_2, INPUT_PULLUP); // SAFETY INPUT 2
  pinMode(VESC_THR, OUTPUT); // VESC OUTPUT
  pinMode(LED, OUTPUT); // ONBOARD LED OUTPUT
  
  kalman.setAngle(pitch);
}

// ---------------------------------------------------------------------------------------

void loop() {
  previousTime = currentTime;
  currentTime = millis();
  elapsedTimeSecs = (currentTime - previousTime) / 1000.0;
  
  yGyro = getGyRaw();
  accPitch = getAccPitch();  
  zAccG = getAccVal();
  //zAccG = az * 0.5 + (zAccG * 0.5); // Low pass filter
  
  // If deemed to be level, horizontally, (given by vertical g (m/s) value & accPitch) then begin accepting gyro data
  if (firstRun == true) {
    if (zAccG >= 9.7 && pitch >= -1 && pitch <= 1) {
      firstRun = false;
    } else {
      gyroPitch = 0;
      pitch = accPitch;
    }
  } else {
    gyroPitch = getGyPitch(yGyro);
    pitch = getCompPitch(gyroPitch, accPitch, pitch);
    //pitch = getKalPitch(pitch); // Slows by ~1500 microseconds when enabled
  }

  //if (analogRead(SAFE_SW_1) <= 100 && analogRead(SAFE_SW_2) <= 100) {
  if (digitalRead(SAFE_SW_1) == 0) {
    digitalWrite(LED, HIGH);
    if (pitch > -1 && pitch < 1 && motorEn == false) { // After leaning to level OK to move off
      motorEn = true;
      Serial.println("MOTOR EN");
    } else if (motorEn == true) {
      thrTarget = pitch * Kp + yGyro * Kd;
      thrValue = mapf(thrTarget, -30.0, 30.0, 0.0, 255.0, 0.0, 255.0);
    }
  } else {
    digitalWrite(LED, LOW);
    thrValue = 127.0;
    motorEn = false;
    firstRun = true;
  }

  Serial.print(pitch);
  //Serial.print("\t");
  //Serial.print(thrValue);
  Serial.println();
  
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

float getCompPitch(float gyro, float acc, float pitch) {
  return compPitch = 0.8 * gyro + 0.2 * acc; // Coarse
  //return compPitch = 0.7 * gyro + 0.2 * acc + 0.1 * pitch; // Smooth(er)
}

float getKalPitch(float pitch) {
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  float kalAngleY = kalman.getAngle(pitch, (gy / 131.0), dt);
  return kalAngleY;
}

void setThrottle(float throttle) {
  ledcWrite(0, throttle); // Output analog data to ESC (0-255, 0-3.3V)
}
