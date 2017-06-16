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

#define output_stream if(!logFile.empty()) ; else yInfo()

YarpViconBridge::YarpViconBridge() : hostname("localhost:801"), inversion(false), poly(YARP_NULLPTR), itf(YARP_NULLPTR), rate(120),
    logFile(""), multicastAddress("244.0.0.0:44801"), connectToMultiCast(false), enableMultiCast(false),
    bReadCentroids(false), bReadRayData(false), clientBufferSize(0), axisMapping("ZUp"), interrupted(false), publish_segments(true), publish_unlabeled_markers(true)
{
    
    subject_string="Subject_";
    segment_string="::Segm_";
    viconroot_string="Vicon_ROOT";
    unlabeled_marker_string="UnlMarker#";
}

YarpViconBridge::YarpViconBridge(std::string _hostname) : hostname(_hostname), inversion(false), poly(YARP_NULLPTR), itf(YARP_NULLPTR), rate(120),
    logFile(""), multicastAddress("244.0.0.0:44801"), connectToMultiCast(false), enableMultiCast(false),
    bReadCentroids(false), bReadRayData(false), clientBufferSize(0), axisMapping("ZUp"), interrupted(false), publish_segments(true), publish_unlabeled_markers(true)
{
    
    subject_string="Subject_";
    segment_string="::Segm_";
    viconroot_string="Vicon_ROOT";
    unlabeled_marker_string="UnlMarker#";
}

bool YarpViconBridge::configure(yarp::os::ResourceFinder &rf){
    if (rf.check("log_file"))
    {
        logFile = rf.find("log_file").asString();
        yInfo()<< "Using log file <"<< logFile << "> ...";
    }

    if(rf.check("hostname"))
    {
        hostname = rf.find("hostname").asString();
        hostname = hostname + ":801";
    }
    
    if(rf.check("inversion"))
    {
        inversion = true;
    }

    if (rf.check("enable_multicast"))
    {
        enableMultiCast = true;
        multicastAddress = rf.find("multicastAddress").asString();
        yInfo() << "Enabling multicast address <"<< multicastAddress << "> ...";
    }
    if (rf.check("connect_to_multicast"))
    {
        connectToMultiCast = true;
        multicastAddress = rf.find("connect_to_multicast").asString();
        yInfo() << "connecting to multicast address <"<< multicastAddress << "> ...";
    }

    if(rf.check("publish_segments"))
    {
        publish_segments=rf.find("publish_segments").asInt()==1;
    }
  
    if(rf.check("publish_unlabeled_markers"))
    {
        publish_unlabeled_markers=rf.find("publish_unlabeled_markers").asInt()==1;
    }
      
    if(rf.check("subject_string"))
    {
        subject_string=rf.find("subject_string").asString();
    }
  
    if(rf.check("segment_string"))
    {
        segment_string=rf.find("segment_string").asString();
    }
    
    if(rf.check("test_frame"))
    {
        test_frame = rf.find("test_frame").asString();
    }
 
    if(rf.check("viconroot_string"))
    {
        viconroot_string = rf.find("viconroot_string").asString();
    }
    
    if(rf.check("unlabeled_marker_string"))
    {
        unlabeled_marker_string = rf.find("unlabeled_marker_string").asString();
    }
    
    if(rf.check("centroids"))
    {
        bReadCentroids = true;
    }

    if(rf.check("rays"))
    {
        bReadRayData = true;
    }

    if(rf.check("client-buffer-size"))
    {
        clientBufferSize = rf.find("client-buffer-size").asInt();
    }

    if(rf.check("set-axis-mapping"))
    {
        axisMapping = rf.find("set-axis-mapping").asString();
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
            ok = ( viconClient.ConnectToMulticast( hostname, multicastAddress ).Result == Result::Success );

        }
        else
        {
            ok =( viconClient.Connect( hostname ).Result == Result::Success );
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

    // configuring transformClient
    poly = new PolyDriver;
    Property prop;
    prop.put("device", "transformClient");
    prop.put("local", "/transformClientVicon");
    prop.put("remote", "/transformServer");
    if(!poly->open(prop))
    {
        yError()<<"YarpViconBridge: unable to open `transformClient` device";
        return false;
    }

    poly->view(itf);
    if(!itf)
    {
        yError()<<"YarpViconBridge: unable to open `transformClient` device";
        return false;
    }

    frameRateWindow = 1000; // frames
    counter = 0;
    lastTime = clock();
    return true;

}

bool YarpViconBridge::updateModule()
{
    // Get a frame
    output_stream << "Waiting for new frame...";
    if(interrupted)
        return false;
    while( viconClient.GetFrame().Result != Result::Success)
    {
        // Sleep a little so that we don't lumber the CPU with a busy poll
        #ifdef WIN32
        Sleep( 200 );
        #else
        sleep(1);
        #endif

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
    output_stream << "Frame Number: " << _Output_GetFrameNumber.FrameNumber ;

    Output_GetFrameRate Rate = viconClient.GetFrameRate();
    yInfo() << "Frame rate: "           << Rate.FrameRateHz          ;

    // Get the latency
    output_stream << "Latency: " << viconClient.GetLatencyTotal().Total << "s" ;

    for( unsigned int LatencySampleIndex = 0 ; LatencySampleIndex < viconClient.GetLatencySampleCount().Count ; ++LatencySampleIndex )
    {
        std::string SampleName  = viconClient.GetLatencySampleName( LatencySampleIndex ).Name;
        double      SampleValue = viconClient.GetLatencySampleValue( SampleName ).Value;

        output_stream << "  " << SampleName << " " << SampleValue << "s" ;
    }
    output_stream ;

    Output_GetHardwareFrameNumber _Output_GetHardwareFrameNumber = viconClient.GetHardwareFrameNumber();
    output_stream << "Hardware Frame Number: " << _Output_GetHardwareFrameNumber.HardwareFrameNumber ;


    if (test_frame!="")
    {
        yarp::sig::Matrix test_matrix (4,4);
        test_matrix.eye();
        static double start_test_time = yarp::os::Time::now();
        double test_time = yarp::os::Time::now();
        test_matrix[0][4] = sin(test_time-start_test_time);
        itf->setTransform(viconroot_string, test_frame, test_matrix);
    }

    // Count the number of subjects
    unsigned int SubjectCount = viconClient.GetSubjectCount().SubjectCount;
    output_stream << "Subjects (" << SubjectCount << "):" ;
    for( unsigned int SubjectIndex = 0 ; SubjectIndex < SubjectCount ; ++SubjectIndex )
    {
        output_stream << "  Subject #" << SubjectIndex ;

        // Get the subject name
        std::string SubjectName = viconClient.GetSubjectName( SubjectIndex ).SubjectName;
        output_stream << "    Name: " << SubjectName ;

        // Get the root segment
        std::string RootSegment = viconClient.GetSubjectRootSegmentName( SubjectName ).SegmentName;
        output_stream << "    Root Segment: " << RootSegment ;

        // Count the number of segments
        unsigned int SegmentCount = viconClient.GetSegmentCount( SubjectName ).SegmentCount;
        output_stream << "    Segments (" << SegmentCount << "):" ;
        for( unsigned int SegmentIndex = 0 ; SegmentIndex < SegmentCount ; ++SegmentIndex )
        {
            output_stream << "      Segment #" << SegmentIndex ;

            // Get the segment name
            std::string SegmentName = viconClient.GetSegmentName( SubjectName, SegmentIndex ).SegmentName;
            output_stream << "        Name: " << SegmentName ;

            // Get the segment parent
            std::string SegmentParentName = viconClient.GetSegmentParentName( SubjectName, SegmentName ).SegmentName;
            output_stream << "        Parent: " << SegmentParentName ;

            // Get the segment's children
            unsigned int ChildCount = viconClient.GetSegmentChildCount( SubjectName, SegmentName ).SegmentCount;
            output_stream << "     Children (" << ChildCount << "):" ;
            for( unsigned int ChildIndex = 0 ; ChildIndex < ChildCount ; ++ChildIndex )
            {
              std::string ChildName = viconClient.GetSegmentChildName( SubjectName, SegmentName, ChildIndex ).SegmentName;
              output_stream << "       " << ChildName ;
            }

            // Get the global segment rotation as a matrix
            Output_GetSegmentGlobalRotationMatrix _Output_GetSegmentGlobalRotationMatrix =
                    viconClient.GetSegmentGlobalRotationMatrix( SubjectName, SegmentName );
            Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation =
                    viconClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );
            yarp::sig::Matrix m1(4, 4);
            m1[0][0] = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 0 ]; m1[0][1] = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 1 ]; m1[0][2] = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 2 ]; m1[0][3] = _Output_GetSegmentGlobalTranslation.Translation[ 0 ]/1000.0;
            m1[1][0] = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 3 ]; m1[1][1] = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 4 ]; m1[1][2] = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 5 ]; m1[1][3] = _Output_GetSegmentGlobalTranslation.Translation[ 1 ]/1000.0;
            m1[2][0] = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 6 ]; m1[2][1] = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 7 ]; m1[2][2] = _Output_GetSegmentGlobalRotationMatrix.Rotation[ 8 ]; m1[2][3] = _Output_GetSegmentGlobalTranslation.Translation[ 2 ]/1000.0;
            m1[3][0] = 0; m1[3][1] = 0; m1[3][2] = 0; m1[3][3] = 1;

            if (publish_segments)
            {
                std::string tf_name;
                if (subject_string!="")
                {
                    tf_name = subject_string+SubjectName+segment_string+SegmentName;
                }
                else
                {
                    if (segment_string !="")
                    {
                        tf_name = segment_string + SegmentName;
                    }
                    else
                    {
                        tf_name = SegmentName;
                    }
                }
                if(inversion)
                {
                  m1=yarp::math::SE3inv(m1);
                  itf->setTransform(viconroot_string, tf_name, m1);
                  
                }
                else
                {
                  itf->setTransform(tf_name, viconroot_string, m1);
                }
            }
      }

      // Count the number of markers
      unsigned int MarkerCount = viconClient.GetMarkerCount( SubjectName ).MarkerCount;
      output_stream << "    Markers (" << MarkerCount << "):" ;
      for( unsigned int MarkerIndex = 0 ; MarkerIndex < MarkerCount ; ++MarkerIndex )
      {
          // Get the marker name
          std::string MarkerName = viconClient.GetMarkerName( SubjectName, MarkerIndex ).MarkerName;

          // Get the marker parent
          std::string MarkerParentName = viconClient.GetMarkerParentName( SubjectName, MarkerName ).SegmentName;

          // Get the global marker translation
          Output_GetMarkerGlobalTranslation _Output_GetMarkerGlobalTranslation =
          viconClient.GetMarkerGlobalTranslation( SubjectName, MarkerName );

          if( bReadRayData )
          {
              Output_GetMarkerRayContributionCount _Output_GetMarkerRayContributionCount =
              viconClient.GetMarkerRayContributionCount(SubjectName, MarkerName);

              if( _Output_GetMarkerRayContributionCount.Result == Result::Success )
              {
                  output_stream << "      Contributed to by: ";
                  for( unsigned int ContributionIndex = 0; ContributionIndex < _Output_GetMarkerRayContributionCount.RayContributionsCount; ++ContributionIndex )
                  {
                      Output_GetMarkerRayContribution _Output_GetMarkerRayContribution =
                      viconClient.GetMarkerRayContribution( SubjectName, MarkerName, ContributionIndex );
                      output_stream << "ID:" << _Output_GetMarkerRayContribution.CameraID << " Index:" << _Output_GetMarkerRayContribution.CentroidIndex << " ";
                  }
              }
          }
      }
    }

    // Get the unlabeled markers
    unsigned int UnlabeledMarkerCount = viconClient.GetUnlabeledMarkerCount().MarkerCount;
    for( unsigned int UnlabeledMarkerIndex = 0 ; UnlabeledMarkerIndex < UnlabeledMarkerCount ; ++UnlabeledMarkerIndex )
    {
        // Get the global marker translation
        Output_GetUnlabeledMarkerGlobalTranslation _Output_GetUnlabeledMarkerGlobalTranslation =
        viconClient.GetUnlabeledMarkerGlobalTranslation( UnlabeledMarkerIndex );

        yarp::sig::Matrix m1(4, 4);
        m1[0][0] = 1; m1[0][1] = 0; m1[0][2] = 0; m1[0][3] = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 0 ]/1000.0;
        m1[1][0] = 0; m1[1][1] = 0; m1[1][2] = 0; m1[1][3] = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 1 ]/1000.0;
        m1[2][0] = 0; m1[2][1] = 0; m1[2][2] = 1; m1[2][3] = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 2 ]/1000.0;
        m1[3][0] = 0; m1[3][1] = 0; m1[3][2] = 0; m1[3][3] = 1;

        if (publish_unlabeled_markers)
        {  
            std::string tf_name = unlabeled_marker_string + std::to_string(UnlabeledMarkerIndex);
            if(inversion)
            {
              m1=yarp::math::SE3inv(m1);
              itf->setTransform(viconroot_string, tf_name, m1);
            }
            else
            {
              itf->setTransform(tf_name, viconroot_string, m1);
            }
          
        }
    }
    
    return true;
}

double YarpViconBridge::getPeriod()
{
    return 1.0/rate;
}

bool YarpViconBridge::interruptModule()
{
    interrupted=true;
    return true;
}

bool YarpViconBridge::close()
{
    if(poly)
    {
        poly->close();
        delete poly;
        poly = YARP_NULLPTR;
    }
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


