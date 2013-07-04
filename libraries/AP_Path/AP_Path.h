// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_Path.h
/// @brief   generic path manager interface

/*
  This defines a generic interface for path managers. Each
  specific manager should be a subclass of this generic
  interface. All variables used by managers should be in their
  own class.
 */

#ifndef AP_PATH_H
#define AP_PATH_H

#include <AP_Common.h>

class AP_Path {
public:

    virtual bool generate_segment(const struct Location &prev_WP, const struct Location &next_WP) = 0;
	virtual bool segment_complete(void) = 0;
	
/*

	// add new path managers to this enum. Users can then
	// select which path manager to use by setting the
	// PATH_MANAGER parameter
	enum PathType {
		BASIC     = 1
	};
    */
};


#endif // AP_PATH_H
