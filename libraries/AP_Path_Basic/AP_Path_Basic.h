// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_Path_Basic.h
/// @brief   Basic waypoint manager. This is a instance of an
/// AP_Path class

#ifndef AP_PATH_BASIC_H
#define AP_PATH_BASIC_H

#include <AP_Math.h>
#include <AP_AHRS.h>
#include <AP_Param.h>
#include <AP_Path.h>
#include <AP_Navigation.h>

class AP_Path_Basic : public AP_Path {
public:
	AP_Path_Basic(AP_AHRS *ahrs, AP_Navigation *nav ) :
		_ahrs(ahrs),
        _nav(nav)
		{
			AP_Param::setup_object_defaults(this, var_info);
		}
	
	bool generate_segment(const struct Location &prev_WP, const struct Location &next_WP);
    bool segment_complete(void);
	
    // this supports the user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
	// reference to the AHRS object
    AP_AHRS *_ahrs;
    AP_Navigation *_nav;
    AP_Float _turnfactor;
    Location _next_WP, _prev_WP;

};

#endif //AP_PATH_BASIC_H