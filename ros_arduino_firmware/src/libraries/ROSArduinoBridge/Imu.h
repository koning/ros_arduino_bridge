

#ifndef __IMU_H__
#define __IMU_H__

// IMU for ros_arduino_bridge
// Joe Koning, 10/03/2016 joking at pobox dot com
//
// Currently supported IMUs:
//                           MPU6050 through the I2Cdevlib
//
// New IMU definitions are implemented by deriving from the rabImu class.
// functions.
// This code rquires avr-stl from https://github.com/andysworkshop/avr-stl.git

#ifdef USE_IMU
#include <Wire.h>
#include <utility.h>
#include <StandardCplusplus.h>
#include <unwind-cxx.h>
#include <system_configuration.h>
#include <map>

#define GY85_T    1 
#define BNO055_T  2
#define MPU6050_T 3
#define MPU9250_T 4

// Base class used in map
class RabImu
{
  public:
    virtual void read(void) = 0;
};

typedef std::map<long, RabImu* > ImuMap;
typedef std::pair<long, RabImu* > ImuPair;

ImuMap imuMap;

#ifdef IMU_GY85
//https://github.com/sqrtmo/GY-85-arduino.git
#include "GY_85.h"
#include <Wire.h>

class RabGY85 : public RabImu
{
  private:
    long addr;
    GY_85 GY85;     //create the object
    
  public:
    RabGY85(long _addr) : addr(_addr) { GY85.init(); }
    void read (void)
    { 
      int ax = GY85.accelerometer_x( GY85.readFromAccelerometer() );
  int ay = GY85.accelerometer_y( GY85.readFromAccelerometer() );
  int az = GY85.accelerometer_z( GY85.readFromAccelerometer() );
  
  int cx = GY85.compass_x( GY85.readFromCompass() );
  int cy = GY85.compass_y( GY85.readFromCompass() );
  int cz = GY85.compass_z( GY85.readFromCompass() );

  float gx = GY85.gyro_x( GY85.readGyro() );
  float gy = GY85.gyro_y( GY85.readGyro() );
  float gz = GY85.gyro_z( GY85.readGyro() );
  float gt = GY85.temp  ( GY85.readGyro() );
  
  //SERIAL_STREAM.print  ( "accelerometer" );
  //SERIAL_STREAM.print  ( " x:" );
  SERIAL_STREAM.print  ( ax );
  SERIAL_STREAM.print(" ");
  //SERIAL_STREAM.print  ( " y:" );
  SERIAL_STREAM.print  ( ay );
  SERIAL_STREAM.print(" ");
  //SERIAL_STREAM.print  ( " z:" );
  SERIAL_STREAM.print  ( az );
  SERIAL_STREAM.print(" ");
  
  //SERIAL_STREAM.print  ( "  compass" );
  //SERIAL_STREAM.print  ( " x:" );
  SERIAL_STREAM.print  ( cx );
  SERIAL_STREAM.print(" ");
  //SERIAL_STREAM.print  ( " y:" );
  SERIAL_STREAM.print  ( cy );
  SERIAL_STREAM.print(" ");
  //SERIAL_STREAM.print  (" z:");
  SERIAL_STREAM.print  ( cz );
  SERIAL_STREAM.print(" ");
  
  //SERIAL_STREAM.print  ( "  gyro" );
  //SERIAL_STREAM.print  ( " x:" );
  SERIAL_STREAM.print  ( gx );
  SERIAL_STREAM.print(" ");
  //SERIAL_STREAM.print  ( " y:" );
  SERIAL_STREAM.print  ( gy );
  SERIAL_STREAM.print(" ");
  //SERIAL_STREAM.print  ( " z:" );
  SERIAL_STREAM.print  ( gz );
  SERIAL_STREAM.print(" ");
  //SERIAL_STREAM.print  ( " gyro temp:" );
  SERIAL_STREAM.println( gt );
    }
};
#endif


#ifdef IMU_MPU6050

// MPU6050 using I2Cdevlib 
// I2Cdev version MPU6050 https:/github.com/jrowberg/i2devlib.git 
#include "I2Cdev.h"
#include "MPU6050.h"

class RabMPU6050 : public RabImu
{
  private:
    uint8_t addr;
    MPU6050 *mpu;     //create the object
    
  public:
    RabMPU6050(uint8_t _addr) : addr(_addr) 
    { 
      mpu = new MPU6050(addr);
      mpu->initialize(); 
    }
    ~RabMPU6050() { delete mpu; }
    void read (void)
    { 
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  mpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // these methods (and a few others) are also available
  //mpu.getAcceleration(&ax, &ay, &az);
  //mpu.getRotation(&gx, &gy, &gz);

  // display space-separated accel/gyro x/y/z values
  SERIAL_STREAM.print(ax); SERIAL_STREAM.print(" ");
  SERIAL_STREAM.print(ay); SERIAL_STREAM.print(" ");
  SERIAL_STREAM.print(az); SERIAL_STREAM.print(" ");
  SERIAL_STREAM.print(gx); SERIAL_STREAM.print(" ");
  SERIAL_STREAM.print(gy); SERIAL_STREAM.print(" ");
  SERIAL_STREAM.println(gz);
    }
};
#endif

// This function is called if ("j" addr type) is called from python
void initIMU(long imu_addr, long imu_type) 
{
  RabImu* rbi = 0;
  switch(imu_type) {
    case GY85_T:
      #ifdef IMU_GY85
      rbi = new RabGY85(imu_addr);
      #endif
      break;
    case BNO055_T:
      #ifdef IMU_BNO055
      rbi = new RabBNO055(imu_addr);
      #endif
      break;
    case MPU6050_T:
      #ifdef IMU_MPU6050
      rbi = new RabMPU6050(imu_addr);
      #endif
      break;
    case MPU9250_T:
      #ifdef IMU_MPU9250
      rbi = new RabMPU9250(imu_addr);
      #endif
      break;
  }
  if(rbi != 0 ) {
    imuMap.insert(ImuPair(imu_addr, rbi));
  }
}
// This function is called when ("i" addr) is called.
void readIMU(long imu_addr) 
{
  if(imuMap.count(imu_addr)) {
    imuMap[imu_addr]->read();
  }
}
#endif // USE_IMU

#endif // __IMU_H__
