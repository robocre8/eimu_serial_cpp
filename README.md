## Easy IMU (EIMU) Cpp Library (amd64-build) i.e for PC
C++ serial interface for the Easy IMU (EIMU).

> you can use it in your microcomputer robotics project (e.g Raspberry Pi, PC, etc.) running ubuntu

#

## Install
- download and install the eimu-serial-dev pkg. you can also check the latest release [here](https://github.com/robocre8/eimu_serial_cpp/releases/)

**PC (AMD64)**
```shell
wget https://github.com/robocre8/eimu_serial_cpp/releases/download/v1.3.0/eimu-serial-dev_1.3.0_amd64.deb
```
```shell
sudo apt install ./eimu-serial-dev_1.3.0_amd64.deb
```

#

## Uninstall
- uninstall the .deb file any time with this
  ```shell
    sudo apt remove eimu-serial-dev
  ```

#

## How to Use the Library
- Ensure you have the **`EIMU MODULE`** connected to you PC or microcomputer

- Ensure you've already calibrated and setup the EIMU with the [eimu_setup_application](https://github.com/robocre8/eimu_setup_application).

- check the serial port the driver is connected to:
  ```shell
  ls /dev/ttyA*
  ```
  > you should see /dev/ttyACM0 or /dev/ttyACM1 and so on

- use the serial port in your code

- A simple way to get started is simply to try out the example code below

#

## Basic Library functions and usage

- connect to EIMU module
  > eimu_serial::EIMUSerialClient imu;
  >
  > imu.connect("port_name or port_path")

- clear imu, filter, etc. data buffer on the EIMU module
  > imu.clearDataBuffer() // returns bool -> success

- set imu reference frame -> NWU (0), ENU (1), NED (2) 
  > imu.setWorldFrameId(frame_id) 

- get imu reference frame -> NWU (0), ENU (1), NED (2) 
  > imu.getWorldFrameId() // returns std::tuple -> bool, float

- adjust filter gain
  > imu.setFilterGain(gain)

- read filter gain
  > imu.getFilterGain() // returns std::tuple -> bool, float

- read all IMU data (orientation - RPY, linear acceleration, angular velocity)
  > imu.readImuData() // returns std::tuple -> bool, std::vector<float> (r, p, y, ax, ay, az, gx, gy, gz)

- read Oreintation - Quaterninos
  > imu.readQuat() // returns std::tuple -> bool, std::vector<float> (qw, qx, qy, qz)

- read Oreintation - RPY
  > imu.readRPY() // returns std::tuple -> bool, std::vector<float> (r, p, y)

- read Linear Acceleration
  > imu.readLinearAcc() // returns std::tuple -> bool, std::vector<float> (ax, ay, az)

- read Gyro (Angular velocity)
  > imu.readGyro() // returns std::tuple -> bool, std::vector<float> (gx, gy, gz)

- while these function above help communicate with the already configure EIMU module, more examples of advanced funtions usage for parameter tuning can be found in the [eimu_setup_application](https://github.com/robocre8/eimu_setup_application) source code

#

## example code - read_imu.cpp

```
/read_imu
├── include/
└── src
    ├── read_imu.cpp
 CMakeLists.txt
```

```cpp
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <iomanip>
#include <cmath>

#include <eimu_serial/eimu_serial.hpp>

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
```

#

## example CMakeLists.txt
```txt
  cmake_minimum_required(VERSION 3.16)
  project(read_imu LANGUAGES CXX)

  find_package(eimu_serial REQUIRED)

  add_executable(read_imu src/read_imu.cpp)
  target_link_libraries(read_imu eimu_serial::eimu_serial)

```

## build your sample package
- build your read_imu sample.
  > cd into the root directory
  >
  > mkdir build (i.e create a folder named build)
  >
  > enter the following command in the terminal in the root folder:
    ````
    cmake -B ./build/
    ````
    ````
    cmake --build ./build/
    ````
    ````
    ./build/read_imu
    ````
