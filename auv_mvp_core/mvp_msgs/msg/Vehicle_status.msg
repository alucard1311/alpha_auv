#
#   This file is part of MVP-Mission program.
#
#   MVP-Mission is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   MVP-Mission is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with MVP-Mission.  If not, see <https://www.gnu.org/licenses/>.
#
#   Author: Mingxi Zhou
#   Email: mzhou@uri.edu
#   Year: 2022
#
#   Copyright (C) 2022 Smart Ocean Systems Laboratory
#

#global information

string  timenow         #SS-DD-MM-YY
string  timegps         #SS-DD-MM-YY
float64 latitude
float64 longitude

#important vehicle pose

geometry_msgs/Vector3 rpy   #degrees
float64 depth               # meters

# instant current and voltage of the main power source

float32 voltage         #volt
float32 current         #amp



