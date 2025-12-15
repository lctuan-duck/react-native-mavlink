project(NitroMavlink)
cmake_minimum_required(VERSION 3.9.0)

set (PACKAGE_NAME NitroMavlink)
set (CMAKE_VERBOSE_MAKEFILE ON)
set (CMAKE_CXX_STANDARD 20)

# Define C++ library and add all sources
add_library(${PACKAGE_NAME} SHARED
        src/main/cpp/cpp-adapter.cpp
        ../cpp/HybirdMAVLink.cpp
        ../cpp/bridge/transport/Transport.cpp
        ../cpp/bridge/commands/Commands.cpp
        ../cpp/bridge/parameters/Parameters.cpp
        ../cpp/bridge/events/Events.cpp
        ../cpp/bridge/mission/Mission.cpp
        ../cpp/bridge/logging/Logging.cpp
        ../cpp/core/MAVLinkCore.cpp
        ../cpp/transport/UDPTransport.cpp
        ../cpp/transport/TCPTransport.cpp
        ../cpp/parser/MessageParser.cpp
        ../cpp/telemetry/TelemetryManager.cpp
        ../cpp/events/EventEmitter.cpp
)

# Add Nitrogen specs :)
include(${CMAKE_SOURCE_DIR}/../nitrogen/generated/android/NitroMavlink+autolinking.cmake)

# Set up local includes
include_directories(
        "src/main/cpp"
        "../cpp"
)

find_library(LOG_LIB log)

# Link all libraries together
target_link_libraries(
        ${PACKAGE_NAME}
        ${LOG_LIB}
        android                                   # <-- Android core
)
