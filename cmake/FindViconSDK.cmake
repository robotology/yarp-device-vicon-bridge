#
# Find the ViconSDK includes and library
#

# This module defines
# ViconSDK_INCLUDE_DIR, where to find ViconSDK.h
# ViconSDK_LIBRARIES, the libraries to link against
# ViconSDK_FOUND, if false, you cannot build anything that requires ViconSDK

######################################################################## 

find_path(ViconSDK_INCLUDE_DIR NAMES DataStreamClient.h HINTS $ENV{ViconSDK_ROOT})
if(WIN32)        
        find_library(ViconSDK_LIBRARY NAMES libViconDataStreamSDK_CPP.dll libViconDataStreamSDK_CPP.lib HINTS $ENV{ViconSDK_ROOT})
endif(WIN32)

if(UNIX AND NOT APPLE)
        find_library(ViconSDK_LIBRARY NAMES libViconDataStreamSDK_CPP.so HINTS $ENV{ViconSDK_ROOT})
endif(UNIX AND NOT APPLE)
 
if(APPLE)
        find_library(ViconSDK_LIBRARY NAMES libViconDataStreamSDK_CPP.dylib HINTS $ENV{ViconSDK_ROOT})
endif(APPLE)

 
if (ViconSDK_INCLUDE_DIR AND ViconSDK_LIBRARY) 
	set(ViconSDK_FOUND TRUE) 
else () 
	set(ViconSDK_FOUND FALSE) 
endif () 
 
 

# TSS: backwards compatibility
set(ViconSDK_LIBRARIES ${ViconSDK_LIBRARY}) 
