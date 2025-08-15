# File:   FindAMPLSOLVER.cmake
# Author: Christian Kirches
# Date:   2014
#
# This file is part of tproprietary software of
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
# Copyright (C) 2014 by the authors. All rights reserved.
#
####################################################################################################
#
# Find an AMPLSOLVER installation
#
# This file fulfils the CMAKE modules guidelines:
# <http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup>
#
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines:
#   AMPLSOLVER_CONFIG          - Full path of the CMake config file of an AMPLSOLVER installation
#   AMPLSOLVER_DIR    [cached] - Full path of the dir holding the CMake config file, cached
#
###################################################################################################

INCLUDE( DefaultSearchPaths )

MESSAGE( STATUS "Looking for AMPLSOLVER" "" )

IF( AMPLSOLVER_FIND_QUIETLY )
	SET( DEMAND_TYPE QUIET )
	MARK_AS_ADVANCED( AMPLSOLVER_DIR )
ENDIF()

IF( AMPLSOLVER_FIND_REQUIRED )
	SET( DEMAND_TYPE REQUIRED )
ENDIF()

FIND_PACKAGE( AMPLSOLVER ${DEMAND_TYPE} NO_MODULE PATHS ${DEFAULT_PACKAGE_DIRS} )

IF( AMPLSOLVER_FOUND )
	MESSAGE( STATUS "Looking for AMPLSOLVER (${AMPLSOLVER_CONFIG})" )
	MARK_AS_ADVANCED( AMPLSOLVER_DIR )
ENDIF()
