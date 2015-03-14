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

from hpp.corbaserver.robot import Robot as Parent

class Robot (Parent):
    packageName = "dlr_miiwa"
    urdfName = "dlr_miiwa"
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__ (self, robotName, load = True):
        Parent.__init__ (self, robotName, "anchor", load)
        self.rightWrist = ""
        self.leftWrist  = ""

