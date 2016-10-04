// IMU for ros_arduino_bridge
// Joe Koning, 10/03/2016 joking at pobox dot com
//
// Currently supported IMUs:
//                           MMA8451 through the I2C-Sensor-Lib
//
// New IMU definitions can be imlemented by defining the initIMU and readIMU
// functions.

#ifdef USE_IMU

// MMA8451 from I2C-Sensor-Lib 
#ifdef IMU_MMA8451

#include <stdint.h>
#include <i2c_MMA8451.h>
MMA8451 mma8451;


// This function is called during setup()
void initIMU() {
  if (mma8451.initialize()) SERIAL_STREAM.println("MMA8451 IMU Sensor found!");
}

//This function is called if an "i" message is received by the loop() function
//The proper array read must be implemented in the arduino_(driver,smbus,serial).py
void readIMU() {
  float xyz_g[3];
  mma8451.getMeasurement(xyz_g);
  //SERIAL_STREAM.print(" X: ");
  SERIAL_STREAM.print(xyz_g[0],2);
  SERIAL_STREAM.print(" ");
  //SERIAL_STREAM.print(" \tY: ");
  SERIAL_STREAM.print(xyz_g[1],2);
  SERIAL_STREAM.print(" ");
  //SERIAL_STREAM.print(" \tZ: ");
  SERIAL_STREAM.println(xyz_g[2],2);
}
#else
void initIMU() {
}
void readIMU() {
}
#endif // IMU TYPES

#endif //USE_IMU
