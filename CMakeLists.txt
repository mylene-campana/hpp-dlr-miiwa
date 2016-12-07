# Copyright (c) 2015 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-dlr-miiwa.
# hpp-dlr-miiwa is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-dlr-miiwa is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-dlr-miiwa.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME dlr_miiwa)
SET(PROJECT_DESCRIPTION "Data specific to dlr miiwa robot")
SET(PROJECT_URL "")

SETUP_PROJECT()

FINDPYTHON()
ADD_REQUIRED_DEPENDENCY ("hpp-corbaserver >= 3")

SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

#ADD_REQUIRED_DEPENDENCY (miiwa)
#ADD_REQUIRED_DEPENDENCY (schunk_pw70)
#ADD_REQUIRED_DEPENDENCY (dlr_miiwa)
#ADD_REQUIRED_DEPENDENCY (schunk_wsg50)
#ADD_REQUIRED_DEPENDENCY (lbr_iiwa_description)
#ADD_REQUIRED_DEPENDENCY (mako_3d)
#ADD_REQUIRED_DEPENDENCY (xacro)

#SET (XACRO_EXEC ${XACRO_PREFIX}/share/xacro/xacro.py)
#MESSAGE (STATUS "XACRO_EXEC=${XACRO_EXEC}")

#execute_process(
#  COMMAND ${XACRO_EXEC} -o dlr_miiwa.urdf
#  ${DLR_MIIWA_PREFIX}/share/dlr_miiwa/urdf/dlr_miiwa.urdf.xacro
#  )

#INSTALL(FILES
#  ${CMAKE_CURRENT_BINARY_DIR}/dlr_miiwa.urdf
#  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/dlr_miiwa/urdf
#)


INSTALL(FILES
  urdf/dlr_miiwa.urdf
  urdf/floor.urdf
  urdf/wall.urdf
  urdf/table.urdf
  urdf/large_wall.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

INSTALL(FILES
  srdf/dlr_miiwa.srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
)


ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
