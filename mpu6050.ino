#include <Wire.h>
long looptimer, prevtime;
float dt;
double gyroX, gyroY, gyroZ, roll_G = 0, pitch_G = 0, yaw_G = 0, gyroXcalibration, gyroYcalibration, gyroZcalibration;
double AccX, AccY, AccZ, accXcalibration, accYcalibration, accZcalibration;
double roll, pitch, yaw;
void start_the_transmition() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}
void read_Acc_data() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t ACCX = Wire.read() << 8 | Wire.read();
  int16_t ACCY = Wire.read() << 8 | Wire.read();
  int16_t ACCZ = Wire.read() << 8 | Wire.read();
  AccX = (float)ACCX / 16384;
  AccY = (float)ACCY / 16384;
  AccZ = (float)ACCZ / 16384;
}
void read_gyro() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6, 0x43);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  gyroX = (float)GyroX / 65.5;
  gyroY = (float)GyroY / 65.5;
  gyroZ = (float)GyroZ / 65.5;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  start_the_transmition();
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);            // write to address 26 of the register
  Wire.write(0x02);            // options here are 0x00 which is off, and 0x01, 0x02, 0x03, 0x04, 0x05, 0x06
  Wire.endTransmission(true);  // 0x06 being the highest filter setting

  for (int RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    read_gyro();
    gyroXcalibration += gyroX;
    gyroYcalibration += gyroY;
    gyroZcalibration += gyroZ;
    read_Acc_data();
    accXcalibration += AccX;
    accYcalibration += AccY;
    accZcalibration += (AccZ - 1);
    delay(1);
  }
  gyroXcalibration /= 2000;
  gyroYcalibration /= 2000;
  gyroZcalibration /= 2000;
  accXcalibration /= 2000;
  accYcalibration /= 2000;
  accZcalibration /= 2000;
}

void loop() {
  read_Acc_data();
  AccX -= accXcalibration;
  AccY -= accYcalibration;
  AccZ -= accZcalibration;
  read_gyro();
  gyroX -= gyroXcalibration;
  gyroY -= gyroYcalibration;
  gyroZ -= gyroZcalibration;
  looptimer = millis();
  dt = (looptimer - prevtime) * 0.001;
  prevtime = looptimer;
  roll_G = gyroX * dt;
  pitch_G = gyroY * dt;

  float angleAccX = atan2(AccY, sqrt(AccZ * AccZ + AccX * AccX)) * 57.29578;
  float angleAccY = -atan2(accX, sqrt(accZ * accZ + accY * accY)) * RAD_2_DEG;  // [-180,+180] deg
  roll = 0.98 * (roll + roll_G) + angleAccX * 0.02;
  pitch = 0.98 * (pitch + pitch_G) + angleAccY;
  yaw = gyroZ * dt;
}
