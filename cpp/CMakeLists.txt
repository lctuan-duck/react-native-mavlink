# CMakeLists.txt for react-native-mavlink C++ module

cmake_minimum_required(VERSION 3.9.0)

project(ReactNativeMAVLink)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find Nitrogen
find_package(react-native-nitro-modules REQUIRED CONFIG)

# MAVLink include path
set(MAVLINK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/MAVLink/v2.0")

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${MAVLINK_INCLUDE_DIR}
)

# Source files
set(SOURCES
    HybridMAVLink.cpp
    vehicle/VehicleState.cpp
    connection/ConnectionManager.cpp
    commands/CommandExecutor.cpp
)

# Create shared library
add_library(
    ${PROJECT_NAME}
    SHARED
    ${SOURCES}
)

# Link against Nitrogen
target_link_libraries(
    ${PROJECT_NAME}
    NitroModules
)

# Platform-specific linking
if(WIN32)
    target_link_libraries(${PROJECT_NAME} ws2_32)
elseif(UNIX AND NOT APPLE)
    target_link_libraries(${PROJECT_NAME} pthread)
elseif(APPLE)
    target_link_libraries(${PROJECT_NAME} "-framework Foundation")
endif()

# Install target
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})
