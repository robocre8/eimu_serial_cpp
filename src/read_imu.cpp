#include <sstream>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <iomanip>
#include <cmath>

#include <eimu_serial.hpp>

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
  float r, p, y, ax, ay, az, gx, gy, gz;

  auto prevTime = std::chrono::system_clock::now();
  std::chrono::duration<double> duration;
  float sampleTime = 0.02; //20ms (50Hz)

  std::string serial_port = "/dev/ttyACM0";
  int serial_baud_rate = 115200;
  int serial_timeout_ms = 18; // < 20ms(for 50Hz comm)
  imu.connect(serial_port, serial_baud_rate, serial_timeout_ms);

  // wait for the eimu to fully setup
  for (int i = 1; i <= 4; i += 1)
  {
    delay_ms(1000);
    std::cout << "configuring controller: " << i << " sec" << std::endl;
  }

  // success = imu.clearDataBuffer();

  int worldFrameId = 1;
  imu.setWorldFrameId(worldFrameId);
  std::tie(success, worldFrameId) = imu.getWorldFrameId();
  if (success){
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
      std::tie(success, r, p, y, ax, ay, az, gx, gy, gz) = imu.readImuData();

      if (success){
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
