# Copyright: (C) 2019 iCub Facility
# Author: Nicolo' Genesio <nicolo.genesio@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 3.2.0)
set(PROJECT_NAME YarpViconBridge)
project(${PROJECT_NAME})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

include(FeatureSummary)

find_package(YARP 3.3.2 REQUIRED
                        COMPONENTS os
                                   sig
                                   dev
                                   math)

find_package(ViconSDK REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${ViconSDK_INCLUDE_DIR})
set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL "yarp_vicon is always built with dynamic plugins")
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
# Warning: the <package> option of yarp_configure_plugins_installation should be different from the plugin name
yarp_configure_plugins_installation(vicon_device)

yarp_prepare_plugin(vicon
                    CATEGORY device
                    TYPE yarp::dev::YarpViconBridge
                    INCLUDE include/YarpViconBridge.h
                    DEFAULT ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
yarp_add_plugin(yarp_vicon include/YarpViconBridge.h
                           src/YarpViconBridge.cpp)

target_link_libraries(yarp_vicon PRIVATE YARP::YARP_os
                                         YARP::YARP_sig
                                         YARP::YARP_dev
                                         YARP::YARP_math
                                         ${ViconSDK_LIBRARY})
list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS YARP::YARP_os
                                                    YARP::YARP_sig
                                                    YARP::YARP_dev
                                                    YARP::YARP_math
                                                    ${ViconSDK_LIBRARY})
yarp_install(TARGETS yarp_vicon
             EXPORT YARP_${YARP_PLUGIN_MASTER}
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

set_property(TARGET yarp_vicon PROPERTY FOLDER "Plugins/Device")
