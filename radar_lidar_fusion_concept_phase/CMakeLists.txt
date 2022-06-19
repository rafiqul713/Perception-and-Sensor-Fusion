project(extended_ekf)

cmake_minimum_required(VERSION 3.6)
add_definitions(-std=c++0x)

include_directories(headers)

set(SOURCES 	extended_kalman_filter/src/extended_kalman_filter.cpp
			extended_kalman_filter/src/data.cpp
			extended_kalman_filter/src/fusion_with_ekf.cpp
            	extended_kalman_filter/src/kalman_filter.cpp
			extended_kalman_filter/src/utility.cpp)

add_executable(extended_ekf ${SOURCES})