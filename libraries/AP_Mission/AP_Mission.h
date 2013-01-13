// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AP_Mission.h
/// @brief   Handles the MAVLINK command mission stack.  Reads and writes mission to storage.

/* 
    The AP_Mission library:
    - Holds a three element array with the previous, current, and after waypoints.
    - Conditional and Do commands associated with the current leg may be requested
    - Reads and writes the mission MAVLINK commands to and from storage.
    - Performs error checking on waypoint values.
    - Keeps track of the indicies of current nav command and non_nav_waypoints.
    - Accounts for the DO_JUMP command in how is sequences waypoints.
    
*/
#ifndef AP_Mission_h
#define AP_Mission_h

#include <GCS_MAVLink.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_HAL.h>

#define WP_SIZE 15
#define WP_START_BYTE 0x500
#define CMD_BLANK 0

#define SEARCH_FORWARD 1
#define SEARCH_REVERSE 0

// Command/Waypoint/Location Options Bitmask
//--------------------
#define MASK_OPTIONS_RELATIVE_ALT    (1<<0)        // 1 = Relative altitude

/// @class    AP_Mission
/// @brief    Object managing one Mission
class AP_Mission {
public:

    AP_Mission();

    /*---------Basic Functions and variables--------*/
    
    /* Initialize the mission index and waypoint array with data in storage.
       Do this first. */
    void init_commands();
    
    /* nav_waypoints is a 3 element array that contain navigation waypoints.
       nav_waypoints[0]: Previous Waypoint
       nav_waypoints[1]: Current Waypoint, the vehicle is navigating towards this.
       nav_waypoints[2]: After Waypoint  
       The vehicle is typically located between waypoints corresponding to indicies 0 and 1 */
    struct Location nav_waypoints[3];
    
    /*Sequencies the entire waypoint queue to the next waypoint.
      Returns false if there is an error.  */
    bool increment_waypoint_index();
    
    /* Forces a reset of the command queue to a specified waypoint.  
       If home is commanded, nav_waypoints[0] is set to last waypoint in mission,
       otherwise nav_waypoints[0] is not changed.
       Returns false if a non_nav command is requested or error. */
    bool change_waypoint_index(const uint8_t &new_index);
    
    /*Gets a new command associated with current leg of the mission.
      Each time this is called a new command is returned.
      Returns false if error or if there are no more commands.  */
    bool get_new_cmd(struct Location &new_cmd);
        
    
    /*---------------------Utility Functions-------------------*/
    
	/*returns the waypoint index, a pointer to a 3 element array
      that contains the indicies of the waypoints corresponding to 
      those in nav_waypoints.  */
    uint8_t * waypoint_index()           {return _index;    }; 
        
    //gets the total number of commands in the mission.
    uint8_t command_total();
    
    /*Sets the total number of commands in the mission.
      Used when a new mission is uploaded by a planner. */
    void set_command_total(uint8_t max_index);
    
    /*Sets the home location, and writes it to storage */
    void set_home(const struct Location &home);
    
     /*Returns home location */
    const struct Location get_home()            {return _home;};
    
    //Low(er) level functions to store commands and waypoints into storage.
    struct Location get_cmd_with_index(int16_t inx);
    struct Location get_cmd_with_index_raw(int16_t inx);
    void set_cmd_with_index(struct Location &temp, uint16_t inx);
    
private:

    /*Starts from search_index, and searches forward (=1) or
      backward (=0) for the next navigation waypoint.  This function is mindful of DO_JUMP 
      commands.  Returns the index of the found navigation waypoint.  */
    uint8_t _find_nav_index(uint8_t search_index, bool forward);
    
    //Checks a navigation command for validity.
    bool _check_nav_valid(const struct Location &temp);
    
    /* Takes the input index, sets as _index[1] and rebases index[2] to that index.
       Returns false if it fails to rebase to the index.  */
    bool _sync_waypoint_index(const uint8_t &new_index);
    
    //Synchronizes the nav_waypoints array with the indicies in _index
    void _sync_nav_waypoints();
    
    //Current index for non navigation commands
    uint8_t _cmd_index;
    
    //Total number of commands.
    uint8_t _max_index;
    uint8_t _index[3];
    struct Location _home;
    
	// pointer to command index and totals parameter in g
	AP_Int8 *_command_index;
    AP_Int8 *_command_total;
};

#endif
