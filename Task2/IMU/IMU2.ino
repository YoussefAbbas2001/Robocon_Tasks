/*
Author : Youssef Abbas
Date   : 31/10/2021
About  : IMU Read DMP
*/
#include <Wire.h>

#define LED 8
#define IMU_ADDRESS 0x68
#define RAD_TO_DEG 180/PI

int16_t minVal=265;
int16_t maxVal=402;
int16_t x_gyro;
int16_t y_gyro;
int16_t z_gyro;
int16_t x_accel;
int16_t y_accel;
int16_t z_accel;
int16_t temp;
int32_t x_gyro_offset = 0;
int32_t y_gyro_offset = 0;
int32_t z_gyro_offset = 0;
int32_t x_accel_offset= 0;
int32_t y_accel_offset= 0;
int32_t z_accel_offset= 0;
int32_t n_iterations;
float  roll;
float  pitch;
float  xang;
float  yang;
float  zang;



void setup()
{
  Serial.begin(9600);
  pinMode(LED,OUTPUT); //Red led to indicate issue
  
  //set IMU configurations
  setImuRegister(0x6B,0x00); //Initializing Sensor
  setImuRegister(0x1B,0x18); //Set Gyro Configurations

  
  calibrationImu(n_iterations); //calibrate IMU
}

void loop()
{
  readImuAll();   //Read Gyro measurments
  computeRoll();
  computePitch();
  computeAngle();
  
  Serial1.print("roll = ");
  Serial1.println(roll,1);
  Serial1.print("pitch = ");
  Serial1.println(pitch,1);
  Serial1.print("x angle = ");
  Serial1.print(xang);
  Serial1.print(" ,y angle = ");
  Serial1.print(yang);
  Serial1.print(" , z angle = ");
  Serial1.print(zang);
  
  
  delay(400);
  
}
void computeAngle()
{
  int xAng = map(x_accel,minVal,maxVal,-90,90);
  int yAng = map(y_accel,minVal,maxVal,-90,90);
  int zAng = map(z_accel,minVal,maxVal,-90,90);
 
  xang= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  yang= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  zang= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
}
void computeRoll()
{
   roll = atan2(y_accel , z_accel) * 180.0 / PI;

}
void computePitch()
{
   pitch = atan2(-x_accel , sqrt(y_accel * y_accel + z_accel * z_accel)) * 180.0 / PI;
 
}

void setImuRegister(uint8_t reg , uint8_t val)
{
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(reg);  //write address of register 
  Wire.write(val);  //write value of register
  Wire.endTransmission();
}

uint8_t readImu1Byte(uint8_t reg)
{ 
  uint8_t data = 0;                    //var to take data in it
  Wire.beginTransmission(IMU_ADDRESS);  
  Wire.write(reg);                     //pit address of register
  Wire.endTransmission();
  
  Wire.requestFrom(IMU_ADDRESS,1);     //request from slave one byte
  while(Wire.available() < 1);         //delay flow of program till 1 byte is ready
  data = Wire.read();                  //read data
  return data;
}
uint16_t readImu2Byte(uint8_t reg)
{
  uint16_t data = 0;                
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(reg);                    //put address of register
  Wire.endTransmission();             
  Wire.requestFrom(IMU_ADDRESS,2);    //request from slave two byte

  while(Wire.available() < 2);        //delay flow of program till 2 byte is ready
  data = Wire.read()<<8 | Wire.read();//read High byte then shifted then read Low byte
  return data;
}

void readImuAll()
{
  /*
  fuction to read all IMU 
  */
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(IMU_ADDRESS,14);
  while(Wire.available() < 14);
  x_accel= Wire.read()<<8 | Wire.read(); //nominal value of x_accel
  y_accel= Wire.read()<<8 | Wire.read(); //nominal value of y_accel
  z_accel= Wire.read()<<8 | Wire.read(); //nominal value of z_accel
  temp   = Wire.read()<<8 | Wire.read(); //nominal value of temp
  x_gyro = Wire.read()<<8 | Wire.read(); //nominal value of x_gyro
  y_gyro = Wire.read()<<8 | Wire.read(); //nominal value of y_gyro
  z_gyro = Wire.read()<<8 | Wire.read(); //nominal value of z_gyro

  x_accel= x_accel - x_accel_offset;   //real value of x_accel
  y_accel= y_accel - y_accel_offset;   //real value of y_accel
  z_accel= z_accel - z_accel_offset;   //real value of z_accel
  x_gyro = x_gyro  - x_gyro_offset ;   //real value of x_gyro
  y_gyro = y_gyro  - y_gyro_offset ;   //real value of y_gyro
  z_gyro = z_gyro  - z_gyro_offset ;   //read value of z_gyro
}
void readImuGyro()
{
  /*
  fuction to read all Gyro 
  */
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(IMU_ADDRESS,8);
  while(Wire.available() < 8);
  x_gyro = Wire.read()<<8 | Wire.read(); //nominal value of x_gyro
  y_gyro = Wire.read()<<8 | Wire.read(); //nominal value of y_gyro
  z_gyro = Wire.read()<<8 | Wire.read(); //nominal value of z_gyro

  x_gyro = x_gyro  - x_gyro_offset;   //real value of x_gyro
  y_gyro = y_gyro  - y_gyro_offset;   //real value of y_gyro
  z_gyro = z_gyro  - z_gyro_offset;   //read value of z_gyro
    
}

void calibrationImu(uint32_t n_iterations)
{
  for(int i = 0;i<n_iterations ;i++)
  {
    x_accel_offset+= readImu2Byte(0x3B);
    y_accel_offset+= readImu2Byte(0x3D);
    z_accel_offset+= readImu2Byte(0x3F);
    
    x_gyro_offset += readImu2Byte(0x43);
    y_gyro_offset += readImu2Byte(0x45);
    z_gyro_offset += readImu2Byte(0x47);  
  }
  x_accel_offset/= n_iterations;
  y_accel_offset/= n_iterations;
  z_accel_offset/= n_iterations;
  x_gyro_offset /= n_iterations;
  y_gyro_offset /= n_iterations;
  z_gyro_offset /= n_iterations;
  
}
