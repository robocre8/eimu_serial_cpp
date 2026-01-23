## Easy IMU Cpp Library
This library helps communicate with the already setup **`Easy IMU Module`** in your PC or microcomputer-based cpp projects, after successful setup with the [eimu_setup_application](https://github.com/samuko-things-company/eimu_setup_application).

> you can use it in your microcomputer robotics project **running on linux** (e.g Raspberry Pi, PC, etc.)

A simple way to get started is simply to try out and follow the example code in the src folder


## How to Use the .deb Package

#### Prequisite
- ensure you've already set up your microcomputer or PC system with ROS2

- download and install the eimu-serial-dev pkg. you can also check the [release](https://github.com/robocre8/eimu_serial_cpp/releases/)

[PC (AMD64)](https://github.com/robocre8/eimu_serial_cpp/tree/amd64-build)
```shell
wget https://github.com/robocre8/eimu_serial_cpp/releases/download/v1.1.1/eimu-serial-dev_1.1.1_amd64.deb
```
```shell
sudo apt install ./eimu-serial-dev_1.1.1_amd64.deb
```
[Raspberry Pi (ARM64)](https://github.com/robocre8/eimu_serial_cpp/tree/arm64-build)
```shell
wget https://github.com/robocre8/eimu_serial_cpp/releases/download/v1.1.1/eimu-serial-dev_1.1.1_arm64.deb
```
```shell
sudo apt install ./eimu-serial-dev_1.1.1_arm64.deb
```


## How to Use the Library (build from source)

- you'll need to install the `libserial` library
  ```shell
    sudo apt-get update
    sudo apt install libserial-dev
  ```
-- Download (by clicking on the green Code button above) or clone the repo into your PC using **`git clone`**
> [!NOTE]  
> you can use this command if you want to clone the repo:
> 
>  ```git clone https://github.com/robocre8/eimu_serial_cpp.git```

- Ensure you have the **Easy IMU Module** is already calibrated.

- Connect the **Easy IMU Module** to your PC or microcomputer

- A simple way to get started is simply to try out and follow the example `read_imu.cpp` code in the src folder.

- make, build and run the example code.
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

- You can follow the pattern used in the example `read_imu.cpp` in your own code and use the cpp library as fit.

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
