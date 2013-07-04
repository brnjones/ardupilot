// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_Path_Basic.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_Path_Basic::var_info[] PROGMEM = {
    // @Param: TURNFACTOR
    // @DisplayName: Turn Factor
    // @Description: Test
	// @Units: Ratio
	// @Range: 1-60
	// @Increment: 1
    AP_GROUPINFO("TURNFACTOR",    0, AP_Path_Basic, _turnfactor, 25),
	
    AP_GROUPEND
};

bool AP_Path_Basic::generate_segment(const struct Location &prev_WP, const struct Location &next_WP) 
{
    _next_WP=next_WP;
    _prev_WP=prev_WP;
    return true;
}

bool AP_Path_Basic::segment_complete(void)
{
    struct Location _current_loc;
    uint32_t _wp_distance;
    
    _ahrs->get_position(&_current_loc);
    
    _wp_distance=get_distance(&_current_loc, &_next_WP);
    
    if (_wp_distance <= _nav->turn_distance(g.waypoint_radius)) {
        //gcs_send_text_fmt(PSTR("Reached Waypoint #%i dist %um"),
        //                  (unsigned)nav_command_index,
        //                  (unsigned)get_distance(&_current_loc, &next_WP));
        return true;
	}

    // have we flown past the waypoint?
    if (location_passed_point(_current_loc, _prev_WP, _next_WP)) {
        //gcs_send_text_fmt(PSTR("Passed Waypoint #%i dist %um"),
        //                  (unsigned)nav_command_index,
        //                  (unsigned)get_distance(&_current_loc, &next_WP));
        return true;
    }

    return false;
    
}