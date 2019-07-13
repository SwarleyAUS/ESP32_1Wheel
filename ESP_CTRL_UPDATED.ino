#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/* Connections:
   DIO 16 - Safety switch input
   DIO 17 - VESC output
   DIO 18 - BAT SW output

   Operation:
   speed linearly related to pitch (with smooting)
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
int16_t ax, ay, az;
int16_t gx, gy, gz;
float zAccG, pitch;

// Misc
float thrValue = 0.0;
float thrTarget = 0.0;
int escThrottle = 0;
bool motorEn = false;

// I/Os
const int LED = 2;
const int SAFE_SW = 16;
const int VESC_THR = 17;
const int PWR_SW = 18;
const int logSize = 20;

// Consts
float accLimit = 16384.0;
float gyroLimit = 131.0;
float throttleStep = 2.0;
float throttleLog[logSize];

void setup() {
  ledcSetup(0, 5000, 8);
  ledcAttachPin(VESC_THR, 0);
  
  Serial.begin(115200);
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Serial.println("I2C");
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
      Serial.println("FAST");
  #endif
  
  mpu6050.initialize();  
  mpu6050.setXAccelOffset(-4114);
  mpu6050.setYAccelOffset(-24);
  mpu6050.setZAccelOffset(1667);
  mpu6050.setXGyroOffset(82);
  mpu6050.setYGyroOffset(-33);
  mpu6050.setZGyroOffset(8);
  
  pinMode(SAFE_SW, INPUT_PULLUP); // SAFETY INPUT
  pinMode(VESC_THR, OUTPUT); // VESC OUTPUT
  pinMode(LED, OUTPUT); // VESC OUTPUT
  pinMode(PWR_SW, OUTPUT); // POWER SWITCH
  //digitalWrite(PWR_SW, HIGH);

  for (int i = (logSize - 1); i > 0; i--) {
    throttleLog[i] = 127;
  }
}

void loop() {
  // READ ACC & GYRO
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  
  zAccG = 10.0 * (float) az / accLimit; // Y-axis value  
  pitch = (float) atan2((-ax), sqrt(ay * ay + az * az)) * 57.3;
  
  // SET TARGET THROTTLE
  if (digitalRead(SAFE_SW) == 0) {
    digitalWrite(LED, HIGH);
    Serial.println("SW CLOSED");
    if (pitch > -3 && pitch < 3 && motorEn == false) { // After leaning forward OK to move off    
      motorEn = true;
      Serial.println("MOTOR EN");
      delay(10);
    } else if (motorEn == true) {
      thrTarget = mapf(pitch, -15.0, 15.0, -100.0, 100.0, -100.0, 100.0);
      thrValue = setThrottle(thrTarget, thrValue); // Increase/decrease throttle
    }
  } else {
    digitalWrite(LED, LOW);
    thrValue = 0.0;
  }
  
  // WRITE THROTTLE VALUE
  escThrottle = mapf(thrValue, -100.0, 100.0, 0, 255, 0, 255); // Convert throttle value to DAC range
  throttleLog[0] = escThrottle;
  escThrottle = avgData(logSize, throttleLog);
  ledcWrite(0, escThrottle); // Output analog data to ESC (0-255, 0-3.3V)
  
  // DEBUG OUTPUT
  /*Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("\t ACCG: ");
  Serial.print(zAccG);
  Serial.print("\t ThrTar: ");
  Serial.print(thrTarget);
  Serial.print("\t ThrVal: ");
  Serial.print(thrValue);
  Serial.print("\t ESCThr: ");*/
  Serial.print(escThrottle);
  Serial.println("");
    
  shiftData(logSize, throttleLog);
  delay(50);
}

// ---------------------------------------------------------------------------------------

float setThrottle(float thrTarget, float thrValue) {
  if (thrTarget < 0) {
    if (thrTarget < thrValue - 5) {
      thrValue = thrValue + ((thrTarget - thrValue)/5.0);
    } else if (thrTarget > thrValue + 5) {
      thrValue = thrValue + (((thrTarget - thrValue)/2.0));
    }
  } else {
    if (thrTarget > thrValue + 5) { // Increase throttle if pitching forward (+10% dead zone)
      thrValue = thrValue + ((thrTarget - thrValue)/5.0);
    } else if (thrTarget < thrValue - 5) { // Decrease throttle if pitching back (-10% dead zone)
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
