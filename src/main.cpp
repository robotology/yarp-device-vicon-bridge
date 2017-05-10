/*
 * Copyright (C) 2017 iCub Facility
 * Authors: Nicolo' Genesio
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include<YarpViconBridge.h>

int main( int argc, char* argv[] )
{
    yarp::dev::YarpViconBridge yvb;
    yarp::os::ResourceFinder rf;
    rf.configure(argc,argv);
    yvb.configure(rf);
    yvb.runModule();
    return 0;
}


