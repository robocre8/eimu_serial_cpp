#include <sstream>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <iomanip>
#include <cmath>

#include "eimu_serial.hpp"

eimu_serial::EIMUSerialClient imu;

void delay_ms(unsigned long milliseconds)
{
  usleep(milliseconds * 1000);
}

int main(int argc, char **argv)
{
  float toRad = 2 * M_PI / 360;
  float toDeg = 1 / toRad;

  bool success;
  float val0;
  std::vector<float> buffer;

  auto prevTime = std::chrono::system_clock::now();
  std::chrono::duration<double> duration;
  float sampleTime = 0.02; //20ms (50Hz)

  std::string serial_port = "/dev/ttyACM0";
  int serial_baud_rate = 115200;
  int serial_timeout_ms = 18; // < 20ms(for 50Hz comm)
  imu.connect(serial_port, serial_baud_rate, serial_timeout_ms);

  // success = imu.clearDataBuffer();

  int worldFrameId = 1;
  imu.setWorldFrameId(worldFrameId);
  std::tie(success, val0) = imu.getWorldFrameId();
  if (success){
    worldFrameId = (int)val0;
    if(worldFrameId == 1) std::cout << "ENU Frame" << std::endl;
    else if(worldFrameId == 0) std::cout << "NWU Frame" << std::endl;
    else if(worldFrameId == 2) std::cout << "NED Frame" << std::endl;
  } else {
    std::cout << "Could not get world frame ID" << std::endl;
  }

  prevTime = std::chrono::system_clock::now();

  while (true)
  {
    duration = (std::chrono::system_clock::now() - prevTime);
    if (duration.count() > sampleTime)
    {
      // float qw, qx, qy, qz;
      // std::tie(success, buffer) = imu.readQuat();
      // if (success){
      //   qw = buffer[0];
      //   qx = buffer[1];
      //   qy = buffer[2];
      //   qz = buffer[3];
      // }

      float r, p, y, ax, ay, az, gx, gy, gz;
      std::tie(success, buffer) = imu.readImuData();
      if (success){
        r = buffer[0]; p = buffer[1]; y = buffer[2];
        ax = buffer[3]; ay = buffer[4]; az = buffer[5];
        gx = buffer[6]; gy = buffer[7]; gz = buffer[8];

        std::cout << "r: " << r*toDeg << std::fixed << std::setprecision(2);
        std::cout << "\tp: " << p*toDeg << std::fixed << std::setprecision(2);
        std::cout << "\ty: " << y*toDeg << std::fixed << std::setprecision(2) << std::endl;

        std::cout << "ax: " << ax << std::fixed << std::setprecision(4);
        std::cout << "\tay: " << ay << std::fixed << std::setprecision(4);
        std::cout << "\taz: " << az << std::fixed << std::setprecision(4) << std::endl;

        std::cout << "gx: " << gx << std::fixed << std::setprecision(4);
        std::cout << "\tgy: " << gy << std::fixed << std::setprecision(4);
        std::cout << "\tgz: " << gz << std::fixed << std::setprecision(4) << std::endl;

        std::cout << std::endl;
      }
      else {
        std::cerr << "Error reading IMU data" << std::endl;
      }

      prevTime = std::chrono::system_clock::now();
    }
  }
}
