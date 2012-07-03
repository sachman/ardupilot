/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * location.cpp
 * Copyright (C) Andrew Tridgell 2011
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  this module deals with calculations involving struct Location
 */

#include <FastSerial.h>
#include "AP_Math.h"

static float longitude_scale(const struct Location *loc)
{
	static uint32_t last_lat;
	static float scale = 1.0;
	if (abs(last_lat - loc->lat) < 100000) {
		// we are within 0.01 degrees (about 1km) of the 
		// same latitude. We can avoid the cos() and return
		// the same scale factor.  
		return scale;
	}
	scale = cos((fabs((float)loc->lat)/1.0e7) * 0.0174532925);
	return scale;
}



// return distance in meters to between two locations, or -1
// if one of the locations is invalid
int32_t get_distance(const struct Location *loc1, const struct Location *loc2)
{
	if (loc1->lat == 0 || loc1->lng == 0)
		return -1;
	if(loc2->lat == 0 || loc2->lng == 0)
		return -1;
	float dlat 		= (float)(loc2->lat - loc1->lat);
	float dlong		= ((float)(loc2->lng - loc1->lng)) * longitude_scale(loc2);
	return sqrt(sq(dlat) + sq(dlong)) * .01113195;
}

// return bearing in centi-degrees between two locations
int32_t get_bearing(const struct Location *loc1, const struct Location *loc2)
{
	int32_t off_x = loc2->lng - loc1->lng;
	int32_t off_y = (loc2->lat - loc1->lat) / longitude_scale(loc2);
	int32_t bearing = 9000 + atan2(-off_y, off_x) * 5729.57795;
	if (bearing < 0) bearing += 36000;
	return bearing;
}

// see if location is past a line perpendicular to 
// the line between point1 and point2. If point1 is 
// our previous waypoint and point2 is our target waypoint
// then this function returns true if we have flown past
// the target waypoint
bool location_passed_point(struct Location &location, 
			   struct Location &point1, 			   
			   struct Location &point2)
{
	// the 3 points form a triangle. If the angle between lines
	// point1->point2 and location->point2 is greater than 90
	// degrees then we have passed the waypoint
	Vector2f loc1(location.lat, location.lng);
	Vector2f pt1(point1.lat, point1.lng);
	Vector2f pt2(point2.lat, point2.lng);
	float angle = (loc1 - pt2).angle(pt1 - pt2);
	if (angle == 0) {
		// if we are exactly on the line between point1 and
		// point2 then we are past the waypoint if the
		// distance from location to point1 is greater then
		// the distance from point2 to point1
		return get_distance(&location, &point1) > 
			get_distance(&point2, &point1);
		
	}
	if (degrees(angle) > 90) {
		return true;
	}
	return false;	
}

