# Copyright: (C) 2017 iCub Facility
# Author: Nicolo' Genesio <nicolo.genesio@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
cmake_minimum_required(VERSION 3.2.0)
set(PROJECT_NAME YarpViconBridge)
PROJECT(${PROJECT_NAME})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(YARP REQUIRED)

find_package(ViconSDK REQUIRED)

set(source src/YarpViconBridge.cpp src/main.cpp)
set(header include/YarpViconBridge.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

if(ViconSDK_FOUND)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS} ${ViconSDK_INCLUDE_DIR})
  add_executable(${PROJECT_NAME} ${source} ${header})
  target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${ViconSDK_LIBRARY})
  target_compile_features(${PROJECT_NAME} PRIVATE cxx_range_for) # forcing the compiler to use c++11
else()
  message("ViconSDK not found, please download it and set ViconSDK_ROOR env var")
  endif()

# install(TARGETS ${PROJECTNAME} DESTINATION bin)
