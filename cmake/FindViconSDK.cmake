################################################################################
#                                                                              #
# Copyright (C) 2019 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

# @author Nicolo' Genesio <nicolo.genesio@iit.it>

#
# Find the ViconSDK includes and library
#

# This module defines
# ViconSDK_INCLUDE_DIR, where to find ViconSDK.h
# ViconSDK_LIBRARIES, the libraries to link against
# ViconSDK_FOUND, if false, you cannot build anything that requires ViconSDK

############################################################################

find_path(ViconSDK_INCLUDE_DIR NAMES DataStreamClient.h
                               HINTS $ENV{ViconSDK_DIR})

find_library(ViconSDK_LIBRARY NAMES ViconDataStreamSDK_CPP
                              HINTS $ENV{ViconSDK_DIR})

if (ViconSDK_INCLUDE_DIR AND ViconSDK_LIBRARY)
  set(ViconSDK_FOUND TRUE)
else () 
  set(ViconSDK_FOUND FALSE)
endif () 

# TSS: backwards compatibility
set(ViconSDK_LIBRARIES ${ViconSDK_LIBRARY})
