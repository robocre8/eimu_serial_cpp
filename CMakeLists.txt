cmake_minimum_required(VERSION 3.16)
project(eimu_cpp LANGUAGES CXX)



set(INCLUDE_LIB include/eimu_serial.hpp)
add_executable(read_imu src/read_imu.cpp ${INCLUDE_LIB})
target_include_directories(read_imu PUBLIC include)
target_link_libraries(read_imu serial)