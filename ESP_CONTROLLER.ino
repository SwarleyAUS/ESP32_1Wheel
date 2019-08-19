#include <MPU6050.h>
#include <Kalman.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <VescUart.h>

#define MPU6050_GYRO_FS_1000 0x02
#define MPU6050_GYRO_FS_2000 0x03

/* Connections:
   DIO 2 - Internal LED
   DIO 16/17 - VESC UART
   DIO 18 - VESC ADC [not used]
   DIO 34 - Safety switch input 1
   DIO 36 - Safety switch input 2
*/

#define ADC_THR 0

// IMU
MPU6050 mpu6050;
const uint16_t MPU = 0x68;
int16_t ax, ay, az, gx, gy, gz;
float zAccG, pitch, yGyro, gyroPitch, accPitch, compPitch;
float previousTime, currentTime, elapsedTimeSecs;
float GYRO_OFFSET = -0.15;

// Kalman & PID
Kalman kalman;
uint32_t timer;
double Kp = 1.25, Kd = 0.07;

// VESC
VescUart Vesc;
uint32_t VescTimer = 0;
int VescCounter = 0;
float VescVolt, VescDuty, VescCurr;
float VESC_ADC_LIM = 192.0;
float VESC_SCALE = 0.0;

// Bluetooth
char pitchString[5], thrString[3], dutyString[4], currString[4];
char sBuffer[30];
BluetoothSerial SerialBT;

// Misc
float thrValue = 0.0;
float thrTarget = 0.0;
float prevThrottleValue;
int escThrottle = 0;
bool motorEn = false;
bool firstRun = true;

// Consts
const int LED = 2;
const int VESC_ADC = 18;
const int SAFE_SW_1 = 34;
const int SAFE_SW_2 = 36;
float accLimit = 16384.0;
float gyroLimit = 32.8;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 16, 17);
  Vesc.setSerialPort(&Serial1);
  SerialBT.begin("ESP32");
  
  ledcSetup(0, 5000, 8);
  ledcAttachPin(VESC_ADC, 0);
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
  
  pinMode(SAFE_SW_1, INPUT_PULLUP); // SAFETY INPUT 1
  pinMode(SAFE_SW_2, INPUT_PULLUP); // SAFETY INPUT 2
  pinMode(VESC_ADC, OUTPUT); // VESC OUTPUT
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
    if (zAccG >= 9.7 && pitch >= -0.5 && pitch <= 0.5) {
      firstRun = false;
      gyroPitch = 0;
    } else {
      pitch = accPitch;
    }
  } else {
    gyroPitch = getGyPitch(yGyro);
    pitch = getCompPitch(gyroPitch, accPitch, pitch);
    //pitch = getKalPitch(pitch); // Slows loop by ~1500 microseconds when enabled
  }

  if ((millis() - VescTimer) >= 10) {
    //if (analogRead(SAFE_SW_1) <= 3250 && analogRead(SAFE_SW_2) <= 3250) {
    if (analogRead(SAFE_SW_1) <= 3000) {
      digitalWrite(LED, HIGH);
      if (pitch > -1 && pitch < 1 && motorEn == false) { // After leaning to level OK to move off
        motorEn = true;
      } else if (motorEn == true) {
        thrTarget = pitch * Kp + yGyro * Kd + VESC_SCALE;
        thrValue = mapf(thrTarget, -30.0, 30.0, 0.0, 255.0, 0.0, 255.0);
        thrValue = thrValue * 0.5 + prevThrottleValue * 0.5; //  Low pass filter
      }
    } else {
      digitalWrite(LED, LOW);
      thrValue = 255.0 / 2.0;
      motorEn = false;
      firstRun = true;
    }
    
    if (ADC_THR == 1) {
      setThrottleADC(thrValue);
    } else {
      setThrottleUART(thrValue);
    }

    if (VescCounter >= 10) {
      if (Vesc.getVescValues() == true) {
        VescVolt = Vesc.data.inpVoltage;
        VescCurr = Vesc.data.ampHours;
        VescDuty = Vesc.data.dutyCycleNow * 100.0;
        
        // Pitch, Throttle, Duty Cycle, Current
        strcpy(sBuffer, dtostrf(pitch, 5, 1, pitchString));
        strcat(sBuffer, " deg, ");
        strcat(sBuffer, dtostrf(thrValue, 3, 0, thrString));
        strcat(sBuffer, ", ");
        strcat(sBuffer, dtostrf(VescDuty, 4, 1, dutyString));
        strcat(sBuffer, "%, ");
        strcat(sBuffer, dtostrf(VescCurr, 4, 1, currString));
        strcat(sBuffer, "A");
      } else {
        strcpy(sBuffer, dtostrf(pitch, 5, 1, pitchString));
        strcat(sBuffer, " deg");
      }
      strcat(sBuffer, "\r\n");
      SerialBT.write((uint8_t *) sBuffer, strlen(sBuffer));
      VescCounter = 0;
    } else {
      VescCounter++;
    }
    prevThrottleValue = thrValue;
    
    //Serial.print(pitch * Kp);
    //Serial.print("\t");
    //Serial.print(yGyro * Kd);
    //Serial.print("\t");
    //Serial.print(pitch);
    //Serial.print("\t");
    //Serial.print(thrValue);    
    //Serial.println();
    
    VescTimer = millis();
  }  
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
  yGyro = yGyro + GYRO_OFFSET; // Drift offset
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

void setThrottleUART(float throttle) {
  throttle = mapf(throttle, 0.0, 255.0, -1.0, 1.0, -1.0, 1.0);
  Vesc.setDuty(throttle);
}

void setThrottleADC(float throttle) {
  throttle = mapf(throttle, 0.0, 255.0, 0.0, VESC_ADC_LIM, 0.0, VESC_ADC_LIM);
  ledcWrite(0, throttle); // Output analog data to ESC
}
