/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Nicolo' Genesio
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include<YarpViconBridge.h>
#include<yarp/os/Time.h>
#include<cmath>

using namespace ViconDataStreamSDK::CPP;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;

#define output_stream if(!logFile.empty()) ; else yInfo()

constexpr uint32_t default_rate = 120;

yarp::math::Quaternion viconQ2yarpQ(const ViconDataStreamSDK::CPP::Output_GetSegmentGlobalRotationQuaternion& rop)
{
    yarp::math::Quaternion ret;
    ret.x() = rop.Rotation[0];
    ret.y() = rop.Rotation[1];
    ret.z() = rop.Rotation[2];
    ret.w() = rop.Rotation[3];
    return ret;
}

YarpViconBridge::YarpViconBridge(std::string _hostname) : PeriodicThread(1.0/default_rate),
                                                          hostname(_hostname),
                                                          inversion(false),
                                                          rate(default_rate),
                                                          subject_string("Subject_"),
                                                          segment_string("::Segm_"),
                                                          viconroot_string("Vicon_ROOT"),
                                                          unlabeled_marker_string("UnlMarker#"),
                                                          logFile(""),
                                                          multicastAddress("244.0.0.0:44801"),
                                                          connectToMultiCast(false),
                                                          enableMultiCast(false),
                                                          bReadCentroids(false),
                                                          bReadRayData(false),
                                                          clientBufferSize(0),
                                                          axisMapping("ZUp"),
                                                          interrupted(false),
                                                          publish_segments(true),
                                                          publish_unlabeled_markers(true)
{
}

bool YarpViconBridge::open(Searchable &config) {

    if (config.check("log_file"))
    {
        logFile = config.find("log_file").asString();
        yInfo()<< "Using log file <"<< logFile << "> ...";
    }

    silent = config.check("silent");

    if(config.check("hostname"))
    {
        hostname = config.find("hostname").asString();
        hostname = hostname + ":801";
    }
    
    if(config.check("inversion"))
    {
        inversion = true;
    }

    if (config.check("enable_multicast"))
    {
        enableMultiCast = true;
        multicastAddress = config.find("multicastAddress").asString();
        yInfo() << "Enabling multicast address <"<< multicastAddress << "> ...";
    }
    if (config.check("connect_to_multicast"))
    {
        connectToMultiCast = true;
        multicastAddress = config.find("connect_to_multicast").asString();
        yInfo() << "connecting to multicast address <"<< multicastAddress << "> ...";
    }

    if(config.check("publish_segments"))
    {
        publish_segments=config.find("publish_segments").asInt()==1;
    }
  
    if(config.check("publish_unlabeled_markers"))
    {
        publish_unlabeled_markers=config.find("publish_unlabeled_markers").asInt()==1;
    }
      
    if(config.check("subject_string"))
    {
        subject_string=config.find("subject_string").asString();
    }

    if(config.check("segment_string"))
    {
        segment_string=config.find("segment_string").asString();
    }

    if(config.check("test_frame"))
    {
        test_frame_name = config.find("test_frame").asString();
    }
 
    if(config.check("viconroot_string"))
    {
        viconroot_string = config.find("viconroot_string").asString();
    }

    if(config.check("unlabeled_marker_string"))
    {
        unlabeled_marker_string = config.find("unlabeled_marker_string").asString();
    }

    if(config.check("centroids"))
    {
        bReadCentroids = true;
    }

    if(config.check("rays"))
    {
        bReadRayData = true;
    }

    if(config.check("client-buffer-size"))
    {
        clientBufferSize = config.find("client-buffer-size").asInt();
    }

    if(config.check("set-axis-mapping"))
    {
        axisMapping = config.find("set-axis-mapping").asString();
        if( axisMapping == "XUp" || axisMapping == "YUp" || axisMapping == "ZUp" )
        {
            yInfo() << "Setting Axis to "<< axisMapping;
        }
        else
        {
            yInfo() << "Unknown axis setting: "<<  axisMapping <<" . Should be XUp, YUp, or ZUp";
            return false;
        }
    }

    if(!logFile.empty())
    {
    ofs.open(logFile.c_str());
    if(!ofs.is_open())
    {
      yError() << "YarpViconBridge: Could not open log file <" << logFile << ">...exiting";
      return false;
    }
    }
    // Connect to a server
    yInfo() << "Connecting to " << hostname << " ...";
    while( !viconClient.IsConnected().Connected )
    {
        // Direct connection
        if(interrupted)
            return false;

        bool ok = false;
        if(connectToMultiCast)
        {
            // Multicast connection
            ok = ( viconClient.ConnectToMulticast( hostname, multicastAddress ).Result == ViconDataStreamSDK::CPP::Result::Success );

        }
        else
        {
            ok =( viconClient.Connect( hostname ).Result == ViconDataStreamSDK::CPP::Result::Success );
        }
        if(!ok)
        {
            yWarning() << "Connect failed...";
        }

        #ifdef WIN32
            Sleep( 1000 );
        #else
            sleep(1);
        #endif
    }

    // Enable some different data types
    viconClient.EnableSegmentData();
    viconClient.EnableMarkerData();
    viconClient.EnableUnlabeledMarkerData();
    viconClient.EnableMarkerRayData();
    viconClient.EnableDeviceData();
    viconClient.EnableDebugData();
    if( bReadCentroids )
    {
    viconClient.EnableCentroidData();
    }
    if( bReadRayData )
    {
    viconClient.EnableMarkerRayData();
    }

    // Set the streaming mode
    //viconClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );
    // viconClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
    viconClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );

    // Set the global up axis
    viconClient.SetAxisMapping( Direction::Forward,
                           Direction::Left,
                           Direction::Up ); // Z-up

    if( axisMapping == "YUp")
    {
        viconClient.SetAxisMapping( Direction::Forward,
                             Direction::Up,
                             Direction::Right ); // Y-up
    }
    else if( axisMapping == "XUp")
    {
        viconClient.SetAxisMapping( Direction::Up,
                             Direction::Forward,
                             Direction::Left ); // X-up
    }


    if( clientBufferSize > 0 )
    {
        viconClient.SetBufferSize( clientBufferSize );
        yInfo() << "Setting client buffer size to " << clientBufferSize ;
    }

    if( enableMultiCast )
    {
    assert( viconClient.IsConnected().Connected );
    viconClient.StartTransmittingMulticast( hostname, multicastAddress );
    }

    frameRateWindow = 1000; // frames
    counter = 0;
    lastTime = clock();
    return PeriodicThread::start();

}

void YarpViconBridge::run()
{
    // Get a frame
    if(!silent)
        output_stream << "Waiting for new frame...";
    if(interrupted)
        this->close();
    while( viconClient.GetFrame().Result != ViconDataStreamSDK::CPP::Result::Success)
    {
        // Sleep a little so that we don't lumber the CPU with a busy poll
        #ifdef WIN32
        Sleep( 200 );
        #else
        sleep(1);
        #endif
        if(!silent)
            output_stream << ".";
    }
    if(++counter == frameRateWindow)
    {
        clock_t Now = clock();
        double FrameRate = (double)(frameRateWindow * CLOCKS_PER_SEC) / (double)(Now - lastTime);
        if(!logFile.empty())
        {
            time_t rawtime;
            struct tm * timeinfo;
            time ( &rawtime );
            timeinfo = localtime ( &rawtime );

            ofs << "Frame rate = " << FrameRate << " at " <<  asctime (timeinfo)<< std::endl;
        }

        lastTime = Now;
        counter = 0;
    }

    // Get the frame number
    Output_GetFrameNumber _Output_GetFrameNumber = viconClient.GetFrameNumber();

    Output_GetFrameRate Rate = viconClient.GetFrameRate();

    if (!silent)
    {
        yInfo() << "Frame rate: " << Rate.FrameRateHz;
        output_stream << "Frame Number: " << _Output_GetFrameNumber.FrameNumber;
        output_stream << "Latency: " << viconClient.GetLatencyTotal().Total << "s";
    }

    // Get the latency
    for( unsigned int LatencySampleIndex = 0 ; LatencySampleIndex < viconClient.GetLatencySampleCount().Count ; ++LatencySampleIndex )
    {
        std::string SampleName  = viconClient.GetLatencySampleName( LatencySampleIndex ).Name;
        double      SampleValue = viconClient.GetLatencySampleValue( SampleName ).Value;

        if (!silent)
            output_stream << "  " << SampleName << " " << SampleValue << "s" ;
    }

    Output_GetHardwareFrameNumber _Output_GetHardwareFrameNumber = viconClient.GetHardwareFrameNumber();
    if (!silent)
    {
        output_stream;
        output_stream << "Hardware Frame Number: " << _Output_GetHardwareFrameNumber.HardwareFrameNumber;
    }

    if (test_frame_name!="")
    {
        FrameTransform test_frame;
        yarp::sig::Matrix test_matrix (4,4);
        static double start_test_time = yarp::os::Time::now();
        double        test_time = yarp::os::Time::now();
        test_matrix.eye();
        test_matrix[0][4]      = sin(test_time-start_test_time);
        test_frame.parentFrame = viconroot_string;
        test_frame.frameId     = test_frame_name;
        test_frame.fromMatrix(test_matrix);
        cacheValid = false;
        m.lock();
        frames.emplace_back(test_frame);
        m.unlock();
    }

    // Count the number of subjects
    unsigned int SubjectCount = viconClient.GetSubjectCount().SubjectCount;
    if(!silent)
        output_stream << "Subjects (" << SubjectCount << "):";
    for( unsigned int SubjectIndex = 0 ; SubjectIndex < SubjectCount ; ++SubjectIndex )
    {
        std::string SubjectName   = viconClient.GetSubjectName( SubjectIndex ).SubjectName;
        std::string RootSegment   = viconClient.GetSubjectRootSegmentName( SubjectName ).SegmentName;
        unsigned int SegmentCount = viconClient.GetSegmentCount( SubjectName ).SegmentCount;

        if (!silent)
        {
            output_stream << "  Subject #" << SubjectIndex ;
            output_stream << "    Name: " << SubjectName ;
            output_stream << "    Root Segment: " << RootSegment ;
            output_stream << "    Segments (" << SegmentCount << "):" ;
        }

        for( unsigned int SegmentIndex = 0 ; SegmentIndex < SegmentCount ; ++SegmentIndex )
        {
            std::string  SegmentName       = viconClient.GetSegmentName( SubjectName, SegmentIndex ).SegmentName;
            std::string  SegmentParentName = viconClient.GetSegmentParentName( SubjectName, SegmentName ).SegmentName;
            unsigned int ChildCount       = viconClient.GetSegmentChildCount( SubjectName, SegmentName ).SegmentCount;

            if (!silent)
            {
                output_stream << "      Segment #"  << SegmentIndex;
                output_stream << "        Name: "   << SegmentName;
                output_stream << "        Parent: " << SegmentParentName;
                output_stream << "     Children ("  << ChildCount << "):";
                for( unsigned int ChildIndex = 0 ; ChildIndex < ChildCount ; ++ChildIndex )
                {
                    output_stream << "       " << viconClient.GetSegmentChildName( SubjectName, SegmentName, ChildIndex ).SegmentName;
                }
            }

            FrameTransform tf;
            auto           pos = viconClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );

            tf.rotation = viconQ2yarpQ(viconClient.GetSegmentGlobalRotationQuaternion(SubjectName, SegmentName));
            tf.transFromVec(pos.Translation[0]/1000.0, pos.Translation[1]/1000.0, pos.Translation[2]/1000.0);

            if (publish_segments)
            {
                std::string tf_name;

                if (subject_string!="")
                {
                    tf_name = subject_string+SubjectName+segment_string+SegmentName;
                }
                else
                {
                    tf_name = segment_string != "" ? segment_string + SegmentName : SegmentName;
                }

                tf.parentFrame = viconroot_string;
                tf.frameId     = tf_name;
                cacheValid     = false;
                m.lock();
                frames.emplace_back(tf);
                m.unlock();
            }
        }

        // Count the number of markers
        if (!silent)
        {
            unsigned int MarkerCount = viconClient.GetMarkerCount(SubjectName).MarkerCount;
            output_stream << "    Markers (" << MarkerCount << "):";
            for (unsigned int MarkerIndex = 0; MarkerIndex < MarkerCount; ++MarkerIndex)
            {
                std::string MarkerName = viconClient.GetMarkerName(SubjectName, MarkerIndex).MarkerName;
                if (!bReadRayData)
                    continue;

                auto contributionCount = viconClient.GetMarkerRayContributionCount(SubjectName, MarkerName);

                if (contributionCount.Result != ViconDataStreamSDK::CPP::Result::Success)
                    continue;
                output_stream << "      Contributed to by: ";
                for (unsigned int ContributionIndex = 0; ContributionIndex < contributionCount.RayContributionsCount; ++ContributionIndex)
                {
                    auto rayContribution = viconClient.GetMarkerRayContribution(SubjectName, MarkerName, ContributionIndex);
                    output_stream << "ID:" << rayContribution.CameraID << " Index:" << rayContribution.CentroidIndex << " ";
                }
            }
        }
    }

    if (publish_unlabeled_markers)
    {
        // Get the unlabeled markers
        unsigned int UnlabeledMarkerCount = viconClient.GetUnlabeledMarkerCount().MarkerCount;
        cacheValid = false;
        m.lock();
        for (unsigned int UnlabeledMarkerIndex = 0; UnlabeledMarkerIndex < UnlabeledMarkerCount; ++UnlabeledMarkerIndex)
        {
            // Get the global marker translation
            FrameTransform tf;
            auto           pos = viconClient.GetUnlabeledMarkerGlobalTranslation(UnlabeledMarkerIndex);
            tf.transFromVec(pos.Translation[0] / 1000.0, pos.Translation[1] / 1000.0, pos.Translation[2] / 1000.0);
            tf.frameId = unlabeled_marker_string + std::to_string(UnlabeledMarkerIndex);
            tf.parentFrame = viconroot_string;
            frames.emplace_back(tf);
        }
        m.unlock();
    }

    callAllCallbacks();
}

void YarpViconBridge::updateFrameContainer(FrameEditor& frameContainer)
{
    m.lock();
    for (auto& f : frames)
        frameContainer.insertUpdate(f);
    cacheValid = true;
    m.unlock();
}

bool YarpViconBridge::close()
{
    if (yarp::os::PeriodicThread::isRunning())
        yarp::os::PeriodicThread::stop();
    interrupted = true;

    if( enableMultiCast )
    {
      viconClient.StopTransmittingMulticast();
    }
    viconClient.DisableSegmentData();
    viconClient.DisableMarkerData();
    viconClient.DisableUnlabeledMarkerData();
    viconClient.DisableDeviceData();
    if( bReadCentroids )
    {
      viconClient.DisableCentroidData();
    }
    if( bReadRayData )
    {
      bReadRayData = false;
    }

    // Disconnect and dispose
    int t = clock();
    yInfo() << " Disconnecting..." ;
    viconClient.Disconnect();
    int dt = clock() - t;
    double secs = (double) (dt)/(double)CLOCKS_PER_SEC;
    yInfo() << " Disconnect time = " << secs << " secs" ;

    return true;
}


