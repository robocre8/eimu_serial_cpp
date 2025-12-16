
#include <sstream>
#include <iostream>
#include <unistd.h>

#include <chrono>

#include <iomanip>

#include "eimu.hpp"

EIMU eimu;

void delay_ms(unsigned long milliseconds)
{
  usleep(milliseconds * 1000);
}

int main(int argc, char **argv)
{
  bool success;
  float r, p, y, ax, ay, az, gx, gy, gz;

  auto prevTime = std::chrono::system_clock::now();
  std::chrono::duration<double> duration;
  float sampleTime = 0.01;

  std::string port = "/dev/ttyACM0";
  eimu.connect(port);

  // wait for the eimu to fully setup
  for (int i = 1; i <= 4; i += 1)
  {
    delay_ms(1000);
    std::cout << "configuring controller: " << i << " sec" << std::endl;
  }

  // success = eimu.clearDataBuffer();

  int worldFrameId = 1;
  eimu.setWorldFrameId(worldFrameId);
  std::tie(success, worldFrameId) = eimu.getWorldFrameId();
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
      std::tie(success, r, p, y, ax, ay, az, gz, gy, gz) = eimu.readImuData();

      if (success){
        std::cout << "r: " << r << std::fixed << std::setprecision(4);
        std::cout << "\tp: " << p << std::fixed << std::setprecision(4);
        std::cout << "\ty: " << y << std::fixed << std::setprecision(4) << std::endl;

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