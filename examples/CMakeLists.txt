cmake_minimum_required(VERSION 3.10)
project(examples)

include_directories(../include)
include_directories(../../GpsUtils/include)
set(CMAKE_CXX_STANDARD 11)

add_executable(timestamp_lidar_samples timestamp_lidar_samples.cpp ../src/velodyne_packet_reader.cpp ../../GpsUtils/src/gps_utils.cpp)