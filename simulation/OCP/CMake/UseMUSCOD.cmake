# File:   UseMUSCOD.cmake
# Author: Christian Hoffmann
# Date:   2007
#
# $LastChangedBy: chris $
# $LastChangedDate: 2009-07-25 13:39:53 +0200 (Sat, 25 Jul 2009) $
# $LastChangedRevision: 3382 $
# $Id: UseMUSCOD.cmake 3382 2009-07-25 11:39:53Z chris $
#
# This file is part of the MUSCOD package. MUSCOD is proprietary software of
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
#
# Copyright (C) 2009 Christian Hoffmann, christian.hoffmann@iwr.uni-heidelberg.de
#
###################################################################################################

IF( NOT _USEMUSCOD_ )
	SET( _USEMUSCOD_ TRUE )

	INCLUDE( CMakeImportBuildSettings )
	INCLUDE( ImportIncludeDirs )

	CMAKE_IMPORT_BUILD_SETTINGS( ${MUSCOD_BUILD_SETTINGS_FILE} )
	IMPORT_INCLUDE_DIRS( ${MUSCOD_INCLUDE_DIRS_FILE} )
    LINK_DIRECTORIES( ${MUSCOD_LIBRARY_DIRS} )
	INCLUDE( ${MUSCOD_LIBRARY_DEPENDENCIES_FILE} )

	MESSAGE( STATUS "Using MUSCOD" )

ENDIF()
