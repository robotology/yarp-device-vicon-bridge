/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Nicolo' Genesio
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */
#ifndef YARPVICONBRIDGE_H
#define YARPVICONBRIDGE_H
// YARP includes
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/math/Math.h>
#include <yarp/math/FrameTransform.h>
#include <yarp/os/LogStream.h>

// ViconDataStreamSDK includes
#include <DataStreamClient.h>

// Std includes
#include <iostream>
#include <fstream>
#include <cassert>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <string.h>
#include <unistd.h> // For sleep()
#include <time.h>

namespace yarp {
    namespace dev {
        class YarpViconBridge;
    }
}

class yarp::dev::YarpViconBridge: public yarp::os::RFModule {
private:
    std::string hostname;
    std::string logFile;
    std::string multicastAddress ;
    bool inversion;
    bool connectToMultiCast;
    bool enableMultiCast;
    bool bReadCentroids;
    bool bReadRayData;
    bool interrupted;
    bool publish_unlabeled_markers;
    bool publish_segments;
    std::ofstream ofs;
    unsigned int clientBufferSize;
    std::string axisMapping;
    int rate;
    
    std::string subject_string;
    std::string segment_string;
    std::string viconroot_string;
    std::string unlabeled_marker_string;
    std::string test_frame;
    
    size_t frameRateWindow; // frames
    size_t counter;
    clock_t lastTime;
    yarp::dev::PolyDriver* poly;
    yarp::dev::IFrameTransform* itf;
    ViconDataStreamSDK::CPP::Client viconClient;
public:

    /**
     * @brief yarp::dev::YarpViconBridge
     */
    YarpViconBridge();

    /**
     * @brief yarp::dev::YarpViconBridge
     * @param _hostname
     */
    YarpViconBridge(std::string _hostname);

    /**
     * @brief configure
     * @param rf
     * @return
     */
    bool configure(yarp::os::ResourceFinder &rf);

    /**
     * @brief updateModule
     * @return
     */
    bool updateModule();

    /**
     * @brief getPeriod
     * @return
     */
    double getPeriod();

    /**
     * @brief interruptModule
     * @return
     */
    bool interruptModule();

    /**
     * @brief close
     * @return
     */
    bool close();


};
#endif // YARPVICONBRIDGE_H

