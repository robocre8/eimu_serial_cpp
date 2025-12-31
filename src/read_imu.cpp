
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
  float r=0.0, p=0.0, y=0.0;
  float ax=0.0, ay=0.0, az=0.0;
  float gx=0.0, gy=0.0, gz=0.0;

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
      std::tie(success, r, p, y) = eimu.readRPY();
      std::tie(success, ax, ay, az) = eimu.readLinearAcc();
      std::tie(success, gz, gy, gz) = eimu.readGyro();

      if (success){
        std::cout << "r: " << r ;
        std::cout << "\tp: " << p ;
        std::cout << "\ty: " << y << std::endl;

        std::cout << "ax: " << ax ;
        std::cout << "\tay: " << ay ;
        std::cout << "\taz: " << az << std::endl;

        std::cout << "gx: " << gx ;
        std::cout << "\tgy: " << gy ;
        std::cout << "\tgz: " << gz << std::endl;

        std::cout << std::endl;
      }
      else {
        std::cerr << "Error reading IMU data" << std::endl;
      }

      prevTime = std::chrono::system_clock::now();
    }
  }
}