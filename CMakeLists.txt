cmake_minimum_required(VERSION 3.15)
project(IMUBundleAdjustment)

set(CMAKE_CXX_STANDARD 14)

include_directories(src)

add_executable(IMUBundleAdjustment
        src/BundleAdjustment.cpp
        src/BundleAdjustment.h
        src/IMUIntegration.cpp
        src/IMUIntegration.h
        src/KeypointDetection.cpp
        src/KeypointDetection.h
        src/main.cpp)
