project(NitroMavlink)
cmake_minimum_required(VERSION 3.9.0)

set (PACKAGE_NAME NitroMavlink)
set (CMAKE_VERBOSE_MAKEFILE ON)
set (CMAKE_CXX_STANDARD 20)

# Define C++ library and add all sources
add_library(${PACKAGE_NAME} SHARED
        src/main/cpp/cpp-adapter.cpp
        ../cpp/HybridMAVLink.cpp
        ../cpp/vehicle/VehicleState.cpp
        ../cpp/connection/ConnectionManager.cpp
        ../cpp/commands/CommandExecutor.cpp
        ../cpp/parameters/ParameterManager.cpp
)

# Add Nitrogen specs :)
include(${CMAKE_SOURCE_DIR}/../nitrogen/generated/android/NitroMavlink+autolinking.cmake)

# Set up local includes
include_directories(
        "src/main/cpp"
        "../cpp"
        "../cpp/vehicle"
        "../cpp/connection"
        "../cpp/commands"
        "../cpp/parameters"
        "../cpp/MAVLink/v2.0"
)

find_library(LOG_LIB log)

# Link all libraries together
target_link_libraries(
        ${PACKAGE_NAME}
        ${LOG_LIB}
        android                                   # <-- Android core
)
