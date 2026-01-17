cmake_minimum_required(VERSION 3.16)

project(eimu_serial
    VERSION 1.0.0
    LANGUAGES CXX
)

# --------------------------------------------------
# Header-only library
# --------------------------------------------------
add_library(eimu_serial INTERFACE)

target_include_directories(eimu_serial INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(eimu_serial INTERFACE cxx_std_17)

# --------------------------------------------------
# Dependency: LibSerial
# --------------------------------------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

target_include_directories(eimu_serial INTERFACE
    ${LIBSERIAL_INCLUDE_DIRS}
)

target_link_libraries(eimu_serial INTERFACE
    ${LIBSERIAL_LIBRARIES}
)

target_compile_options(eimu_serial INTERFACE
    ${LIBSERIAL_CFLAGS_OTHER}
)

# Namespaced alias
add_library(eimu_serial::eimu_serial ALIAS eimu_serial)

# --------------------------------------------------
# Examples
# --------------------------------------------------
# option(EIMU_SERIAL_BUILD_EXAMPLES "Build EIMU Serial examples" OFF)

# if(EIMU_SERIAL_BUILD_EXAMPLES)
#     add_executable(read_imu examples/read_imu.cpp)
#     target_link_libraries(read_imu PRIVATE eimu_serial::eimu_serial)
# endif()

# --------------------------------------------------
# Install
# --------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    TARGETS eimu_serial
    EXPORT eimu_serialTargets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT eimu_serialTargets
    NAMESPACE eimu_serial::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/eimu_serial
)

# --------------------------------------------------
# Package config files
# --------------------------------------------------
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/eimu_serialConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/eimu_serialConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/eimu_serialConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/eimu_serial
)

install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/eimu_serialConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/eimu_serialConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/eimu_serial
)
