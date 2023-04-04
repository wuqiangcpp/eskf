cmake_minimum_required(VERSION 3.10)
project(gps_imu_fusion)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CXX_STANDARD 11)


#add_compile_definitions(_USE_MATH_DEFINES)

#include(cmake/eigen.cmake)
#include(cmake/YAML.cmake)
#include(cmake/global_defination.cmake)

include_directories(include/)
include_directories(3rd/GeographicLib/include/)
include_directories(3rd/)
include_directories(3rd/yaml-cpp/include/)

add_subdirectory(3rd/yaml-cpp)


add_library(DEPS 
    src/eskf.cpp
    src/gps_flow.cpp
    src/imu_flow.cpp
    src/eskf_flow.cpp
    3rd/GeographicLib/src/LocalCartesian.cpp
    3rd/GeographicLib/src/Geocentric.cpp
    3rd/GeographicLib/src/Math.cpp
)

target_link_libraries(DEPS  yaml-cpp)

set(target gps_imu_fusion)

add_executable(${target} src/gps_imu_fusion.cpp)
target_link_libraries(${target}  yaml-cpp DEPS )

#################  TEST  #################
#add_executable(test_gps test/test_gps.cpp)
#target_link_libraries(test_gps DEPS)

#add_executable(test_imu test/test_imu.cpp)
#target_link_libraries(test_imu DEPS)


add_custom_command(TARGET ${PROJECT_NAME} 
   POST_BUILD 
   COMMAND ${CMAKE_COMMAND} -E 
       copy_if_different  
        "${CMAKE_BINARY_DIR}/$<$<CONFIG:Release>:${target}.exe>"  
#        "${CMAKE_BINARY_DIR}/$<$<CONFIG:Debug>:${target}_debug.exe>"  
        "${PROJECT_SOURCE_DIR}/" 
)


