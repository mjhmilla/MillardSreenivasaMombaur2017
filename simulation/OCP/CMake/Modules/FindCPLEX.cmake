# File:   FindCPLEX.cmake
# Author: Janick Frasch
# Date:   2010
#
# This file is part of tproprietary software of
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
# Copyright (C) 2010 by the authors. All rights reserved.
#
####################################################################################################
#
# Find a CPLEX installation
#
# #This file fulfils the CMAKE modules guidelines:
# #<http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup>
#
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines:
# #  QPOPT_CONFIG          - Full path of the CMake config file of a QPOPT installation
# #  QPOPT_DIR    [cached] - Full path of the dir holding the CMake config file, cached
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )

MESSAGE( STATUS "Looking for CPLEX: " "" )

####################################################################################################
#### SEARCH PACKAGE COMPONENTS
####################################################################################################

SET( CPLEX_DIR "" CACHE PATH "Path to a CPLEX installation" )

MESSAGE( STATUS "Looking for CPLEX library" )
FIND_LIBRARY( CPLEX_LIBRARY
	NAMES 
	    cplex
	    cplex121
	PATHS	
	    ${CPLEX_DIR}
	    ${DEFAULT_PACKAGE_DIRS}
	    ${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES 
	    bin/x86-64_debian4.0_4.1
	    bin/x86-64_sles9.0_3.3
	    lib
	    lib64
	    LIB
)
MESSAGE( STATUS "Looking for CPLEX library (${CPLEX_LIBRARY})" )

MESSAGE( STATUS "Looking for CPLEX include directory" )
FIND_PATH( CPLEX_INCLUDE_DIR cplex.h
	PATHS	
	    ${CPLEX_DIR}
	    ${DEFAULT_PACKAGE_DIRS}
	    ${DEFAULT_INCLUDE_DIRS}
	    #/opt/ilog/cplex121/include/ilcplex
	PATH_SUFFIXES include/ilcplex
)
MESSAGE( STATUS "Looking for CPLEX include directory (${CPLEX_INCLUDE_DIR})" )

####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################

FIND_PACKAGE_HANDLE_STANDARD_ARGS( CPLEX DEFAULT_MSG
	CPLEX_LIBRARY
	CPLEX_INCLUDE_DIR
)

IF( CPLEX_FOUND )
	SET( CPLEX_INCLUDE_DIRS ${CPLEX_INCLUDE_DIR} )
	SET( CPLEX_LIBRARIES "${CPLEX_LIBRARY}"	)
ENDIF()


IF( CPLEX_FOUND OR CPLEX_FIND_QUIETLY )
	MARK_AS_ADVANCED(
		CPLEX_DIR
		CPLEX_INCLUDE_DIR
		CPLEX_LIBRARY
	)
ENDIF()

