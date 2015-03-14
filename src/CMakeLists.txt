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

FINDPYTHON()
PYTHON_INSTALL_ON_SITE (hpp/corbaserver/dlr_miiwa robot.py)
PYTHON_INSTALL_ON_SITE (hpp/corbaserver/dlr_miiwa __init__.py)
PYTHON_INSTALL_ON_SITE (hpp/corbaserver/manipulation/dlr_miiwa robot.py)
PYTHON_INSTALL_ON_SITE (hpp/corbaserver/manipulation/dlr_miiwa __init__.py)
