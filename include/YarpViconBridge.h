/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Nicolo' Genesio
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */
#ifndef YARPVICONBRIDGE_H
#define YARPVICONBRIDGE_H
// YARP includes
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameSource.h>
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
#include <mutex>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h> // For sleep()
#endif
#include <time.h>

namespace yarp {
    namespace dev {
        class YarpViconBridge;
    }
}

class yarp::dev::YarpViconBridge: public yarp::dev::DeviceDriver,
                                  public yarp::os::PeriodicThread,
                                  public yarp::dev::ImplementIFrameSource
{
private:
    std::string hostname;
    std::string logFile;
    std::string multicastAddress;
    bool inversion;
    bool connectToMultiCast;
    bool enableMultiCast;
    bool bReadCentroids;
    bool bReadRayData;
    bool interrupted;
    bool publish_unlabeled_markers;
    bool publish_segments;
    bool silent{false};
    std::ofstream ofs;
    unsigned int clientBufferSize;
    std::string axisMapping;
    int rate;
    
    std::string subject_string;
    std::string segment_string;
    std::string viconroot_string;
    std::string unlabeled_marker_string;
    std::string test_frame_name;

    std::mutex m;
    std::vector<yarp::math::FrameTransform> frames;
    
    size_t frameRateWindow; // frames
    size_t counter;
    clock_t lastTime;
    ViconDataStreamSDK::CPP::Client viconClient;
protected:
    void updateFrameContainer(FrameEditor& frameContainer) override;
    bool callbackPrepare() override { return true; };
    bool callbackDismiss() override { return true; };
public:

    /**
     * @brief yarp::dev::YarpViconBridge
     * @param _hostname
     */
    YarpViconBridge(std::string _hostname="localhost:801");

    /**
     * @brief open
     * @param config
     * @return
     */
    bool open(yarp::os::Searchable &config) override;

    /**
     * @brief run
     * @return
     */
    void run() override;

    /**
     * @brief close
     * @return
     */
    bool close() override;


};
#endif // YARPVICONBRIDGE_H

