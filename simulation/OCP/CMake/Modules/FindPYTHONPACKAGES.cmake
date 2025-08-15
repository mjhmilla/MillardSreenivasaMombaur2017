# File:   FindPYTHONMODULE.cmake
# Author: Felix Lenders
# Date:   2013
#
# This file is part of proprietary software of
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
# Copyright (C) 2007--2009 by the authors. All rights reserved.
#
####################################################################################################
#
# Detect if a certain Python Module is installed
#
# This file fulfils the CMAKE modules guidelines:
# <http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup>
#
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines:
#   XXX_CONFIG          - Full path of the CMake config file of a XXX installation
#   XXX_DIR    [cached] - Full path of the dir holding the CMake config file, cached
#
###################################################################################################

INCLUDE( DefaultSearchPaths )

FUNCTION( FIND_PYTHON_MODULE MODULE )

        STRING( TOUPPER ${MODULE} MODULE_UPPER )

        IF( NOT PY_${MODULE_UPPER} )

                MESSAGE( STATUS "Looking for Python Package ${MODULE}" )

                IF( ARGC GREATER 1 AND ARGV1 STREQUAL "REQUIRED" )
                        SET(${MODULE}_FIND_REQUIRED TRUE)
                ENDIF( ARGC GREATER 1 AND ARGV1 STREQUAL "REQUIRED" )

                EXECUTE_PROCESS( COMMAND "${PYTHON_EXECUTABLE}" "-c"
                        "import ${MODULE}; print ${MODULE}.__file__"
                        RESULT_VARIABLE _${MODULE}_STATUS
                        OUTPUT_VARIABLE _${MODULE}_LOCATION
                        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

                IF( NOT _${MODULE}_STATUS )
                        SET(PY_${MODULE_UPPER} ${_${MODULE}_LOCATION} CACHE STRING
                                "Location of Python module ${MODULE}")
                ENDIF( NOT _${MODULE}_STATUS )

        ENDIF(NOT PY_${MODULE_UPPER})

        FIND_PACKAGE_HANDLE_STANDARD_ARGS(PY_${MODULE} DEFAULT_MSG PY_${MODULE_UPPER})

ENDFUNCTION( FIND_PYTHON_MODULE )


FIND_PACKAGE( PythonInterp )

IF( PYTHONINTERP_FOUND )
        FIND_PACKAGE( PythonLibsNew 2.7 REQUIRED )
        IF( UNIX )
            MESSAGE( ${PYTHON_LIBRARY} )
            IF( ${PYTHON_LIBRARY} STREQUAL "PYTHON_LIBRARY-NOTFOUND" )
                SET( PYTHON_LIBRARY "" )
		SET( PYTHON_LIBRARIES "" )
            ENDIF( ${PYTHON_LIBRARY} STREQUAL "PYTHON_LIBRARY-NOTFOUND" )
        ENDIF( UNIX )
        FIND_PYTHON_MODULE( cython )
        FIND_PYTHON_MODULE( numpy )
ENDIF( PYTHONINTERP_FOUND )

FIND_PACKAGE_HANDLE_STANDARD_ARGS(PYTHONPACKAGES PY_CYTHON PY_NUMPY )

