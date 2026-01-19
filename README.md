## Easy IMU (EIMU) Cpp Library
C++ serial interface for the Easy IMU (EIMU).

> you can use it in your microcomputer robotics project (e.g Raspberry Pi, PC, etc.) running ubuntu

#

## Install
- download the latest eimu-serial-dev `.deb` file form the [release](https://github.com/robocre8/eimu_serial_cpp/releases)

- install the eimu-serial-dev `.deb` file
  ```shell
    sudo apt install ./eimu-serial-dev_<version>_amd64.deb
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
  > imu.clearDataBuffer() # returns bool -> success

- set imu reference frame -> NWU (0), ENU (1), NED (2) 
  > imu.setWorldFrameId(frame_id)

- get imu reference frame -> NWU (0), ENU (1), NED (2) 
  > imu.getWorldFrameId() # returns std::tuple -> (success, frame_id): bool, int

- adjust filter gain
  > imu.setFilterGain(gain)

- read filter gain
  > imu.getFilterGain() # returns std::tuple -> (success, gain): bool, float

- read all IMU data (orientation - RPY, linear acceleration, angular velocity)
  > imu.readImuData() # returns std::tuple -> (success, r, p, y, ax, ay, az, gx, gy, gz): bool, float, float, float, float, float, float, float, float, float

- read Oreintation - Quaterninos
  > imu.readQuat() # returns std::tuple -> (success, qw, qx, qy, qz): bool, float, float, float, float

- read Oreintation - RPY
  > imu.readRPY() # returns std::tuple -> (success, r, p, y): bool, float, float, float

- read Linear Acceleration
  > imu.readLinearAcc() # returns std::tuple -> (success, ax, ay, az): bool, float, float, float

- read Gyro (Angular velocity)
  > imu.readGyro() # returns std::tuple -> (success, gx, gy, gz): bool, float, float, float

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
