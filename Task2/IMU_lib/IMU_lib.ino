#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>

MPU6050 imu;


const int iAx = 0;    // index of xaxis Accelerameter
const int iAy = 1;    // index of yaxis Accelerameter
const int iAz = 2;    // index of zaxis Accelerameter


uint16_t  x_accel;
uint16_t  y_accel;
uint16_t  z_accel;

int16_t minVal=265;
int16_t maxVal=402;
float  roll;
float  pitch;
float  yaw;
float  xang;
float  yang;
float  zang;


void Initialize()
 {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    imu.initialize();
    imu.CalibrateAccel(6);
    imu.PrintActiveOffsets();
    imu.CalibrateAccel(1);
    imu.PrintActiveOffsets();
    imu.CalibrateAccel(1);
    imu.PrintActiveOffsets();
    imu.CalibrateAccel(1);
    imu.PrintActiveOffsets();
    imu.CalibrateAccel(1);
    imu.PrintActiveOffsets();
 }

void SetOffsets(int TheOffsets[6])
{ 
    imu.setXAccelOffset(TheOffsets [iAx]);  //set the error of set appear in X_axis Acceleratmeter
    imu.setYAccelOffset(TheOffsets [iAy]);  //set the error of set appear in Y_axis Acceleratmeter
    imu.setZAccelOffset(TheOffsets [iAz]);  //set the error of set appear in Z_axis Acceleratmeter

} 
void computeAllAxis()
{
  computeRoll(); 
  computePitch();
  computeYaw();
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
   roll = atan2(y_accel , z_accel) * (180.0/PI);

}
void computePitch()
{
   pitch = atan2(-x_accel , sqrt(y_accel * y_accel + z_accel * z_accel)) * (180.0/PI);
 
}
void computeYaw()
{
   yaw =  atan2(z_accel , sqrt(x_accel*x_accel + z_accel*z_accel)) * (180.0/PI);
}
void setup() 
{
    Serial.begin(9600);
    Initialize();

}

void loop() 
{
  x_accel =  imu.getAccelerationX();
  y_accel =  imu.getAccelerationY();
  z_accel =  imu.getAccelerationZ();
  computeAllAxis();  //Compute Roll, Pitch, and Yaw
  Serial.print("roll  = ");
  Serial.println(roll,1);
  Serial.print("pitch = ");
  Serial.println(pitch,1);
  Serial.print("Yaw   = ");
  Serial.println(yaw,1);
  Serial.print("x angle = ");
  Serial.print(xang);
  Serial.print(" ,y angle = ");
  Serial.print(yang);
  Serial.print(" , z angle = ");
  Serial.print(zang);
  
}
