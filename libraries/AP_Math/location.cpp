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
 *  this module deals with calculations involving struct Location
 */
#include <stdlib.h>
#include "AP_Math.h"

// radius of earth in meters
#define RADIUS_OF_EARTH 6378100

float longitude_scale(const struct Location *loc)
{
    static int32_t last_lat;
    static float scale = 1.0;
    if (labs(last_lat - loc->lat) < 100000) {
        // we are within 0.01 degrees (about 1km) of the
        // same latitude. We can avoid the cos() and return
        // the same scale factor.
        return scale;
    }
    scale = cosf((fabsf((float)loc->lat)/1.0e7f) * 0.0174532925f);
    last_lat = loc->lat;
    return scale;
}



// return distance in meters to between two locations
float get_distance(const struct Location *loc1, const struct Location *loc2)
{
    float dlat              = (float)(loc2->lat - loc1->lat);
    float dlong             = ((float)(loc2->lng - loc1->lng)) * longitude_scale(loc2);
    return pythagorous2(dlat, dlong) * 0.01113195f;
}

// return distance in centimeters to between two locations
uint32_t get_distance_cm(const struct Location *loc1, const struct Location *loc2)
{
    return get_distance(loc1, loc2) * 100;
}

// return bearing in centi-degrees between two locations
int32_t get_bearing_cd(const struct Location *loc1, const struct Location *loc2)
{
    int32_t off_x = loc2->lng - loc1->lng;
    int32_t off_y = (loc2->lat - loc1->lat) / longitude_scale(loc2);
    int32_t bearing = 9000 + atan2f(-off_y, off_x) * 5729.57795f;
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
    if (isinf(angle)) {
        // two of the points are co-located.
        // If location is equal to point2 then say we have passed the
        // waypoint, otherwise say we haven't
        if (get_distance(&location, &point2) == 0) {
            return true;
        }
        return false;
    } else if (angle == 0) {
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

/*
 *  extrapolate latitude/longitude given bearing and distance
 *  thanks to http://www.movable-type.co.uk/scripts/latlong.html
 *
 *  This function is precise, but costs about 1.7 milliseconds on an AVR2560
 */
void location_update(struct Location *loc, float bearing, float distance)
{
    float lat1 = radians(loc->lat*1.0e-7f);
    float lon1 = radians(loc->lng*1.0e-7f);
    float brng = radians(bearing);
    float dr = distance/RADIUS_OF_EARTH;

    float lat2 = asinf(sinf(lat1)*cosf(dr) +
                       cosf(lat1)*sinf(dr)*cosf(brng));
    float lon2 = lon1 + atan2f(sinf(brng)*sinf(dr)*cosf(lat1),
                               cosf(dr)-sinf(lat1)*sinf(lat2));
    loc->lat = degrees(lat2)*1.0e7f;
    loc->lng = degrees(lon2)*1.0e7f;
}

/*
 *  extrapolate latitude/longitude given distances north and east
 *  This function costs about 80 usec on an AVR2560
 */
void location_offset(struct Location *loc, float ofs_north, float ofs_east)
{
    if (ofs_north != 0 || ofs_east != 0) {
        float dlat = ofs_north * 89.831520982f;
        float dlng = (ofs_east * 89.831520982f) / longitude_scale(loc);
        loc->lat += dlat;
        loc->lng += dlng;
    }
}

/*
 *  Generate turn center, entry, and exit waypoints based on three waypoints, in accordance with a "fly-by" waypoint type.
 *  http://www.faa.gov/air_traffic/publications/atpubs/aim/aim0102.html
 */
bool generate_WP_flyby(const float radius, const struct Location &wpA, const struct Location &wpB, const struct Location &wpC,
                       struct Location &wpB1, struct Location &wpB2, struct Location &wpB3, int8_t &dir)
{
    Vector2f b1, b2, b3, q1, q2, q_diff;
    float ang;

    Vector2f A(0, 0);

    Vector2f Atmp(wpA.lat/1.0e7,wpA.lng/1.0e7);
    Vector2f B(wpB.lat/1.0e7,wpB.lng/1.0e7);
    Vector2f C(wpC.lat/1.0e7,wpC.lng/1.0e7);

    B=geo2planar(Atmp,B);
    C=geo2planar(Atmp,C);

    B=B*RADIUS_OF_EARTH;
    C=C*RADIUS_OF_EARTH;

    q1=(B-A).normalized();
    q2=(C-B).normalized();
    q_diff=(q1-q2).normalized();

    if( (q1.x * q2.y - q1.y * q2.x) > 0) {
        dir = 1;        //Clockwise (Right)
    } else {
        dir = -1;       //Counterclockwise (Left)
    }

    ang=acos(-q1.x*q2.x-q1.y*q2.y)/2.0;
    if(ang < 0.17 || ang > 1.39)      //radians
    {
        return false;
    } else {

        b2.x=B.x-(radius/sin(ang))*q_diff.x;
        b2.y=B.y-(radius/sin(ang))*q_diff.y;

        b1.x=B.x-(radius/tan(ang))*q1.x;
        b1.y=B.y-(radius/tan(ang))*q1.y;

        b3.x=B.x+(radius/tan(ang))*q2.x;
        b3.y=B.y+(radius/tan(ang))*q2.y;

        b1=b1/RADIUS_OF_EARTH;
        b2=b2/RADIUS_OF_EARTH;
        b3=b3/RADIUS_OF_EARTH;

        b1=planar2geo(Atmp,b1);
        b2=planar2geo(Atmp,b2);
        b3=planar2geo(Atmp,b3);

        wpB1=wpB;
        wpB2=wpB;
        wpB3=wpB;

        wpB1.lat=b1.x*1e7;
        wpB1.lng=b1.y*1e7;
        wpB2.lat=b2.x*1e7;
        wpB2.lng=b2.y*1e7;
        wpB3.lat=b3.x*1e7;
        wpB3.lng=b3.y*1e7;

    }

}

Vector2f geo2planar(Vector2f &ref, Vector2f &wp)
{
    //x-North (latitude), y-East (Longitude)
    Vector2f out;
    out.x=radians((wp.x-ref.x));
    out.y=radians((wp.y-ref.y)*cos(radians(ref.x)));

    return out;
}

Vector2f planar2geo(Vector2f &ref, Vector2f &wp)
{
    //x-North (latitude), y-East (Longitude)
    Vector2f out;
    out.x=degrees(wp.x)+ref.x;
    out.y=degrees(wp.y*(1/cos(radians(ref.x))))+ref.y;

    return out;
}
