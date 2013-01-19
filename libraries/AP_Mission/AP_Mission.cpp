// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AP_Mission.cpp
/// @brief   Handles the MAVLINK command mission stack.  Reads and writes mission to storage.

#include "AP_Mission.h"

extern const AP_HAL::HAL& hal;

AP_Mission::AP_Mission()
{
    /* get a pointer to COMMAND_INDEX and COMMAND_TOTAL so we can
     *  read and write to these parameters from within the library.
     *  This is is required for startup and in-air restart functionality */
    enum ap_var_type var_type;
    _command_index = (AP_Int8 *)AP_Param::find("CMD_INDEX", &var_type);
    _command_total = (AP_Int8 *)AP_Param::find("CMD_TOTAL", &var_type);

    _index[0]=0; _index[1]=0; _index[2]=0;
}

void AP_Mission::init_commands()
{
    uint8_t initial_index=_command_index->get();
    _max_index= _command_total->get();

    //Ensure the index loaded from storage is valid
    initial_index=_find_nav_index(initial_index, SEARCH_FORWARD);
    change_waypoint_index(initial_index);
}

bool AP_Mission::increment_waypoint_index()
{
    _index[0]=_index[1];
    if(_sync_waypoint_index(_index[2])) {
        return true;
    } else {
        return false;
    }
}

bool AP_Mission::change_waypoint_index(const uint8_t &new_index)
{
    //Current index is requested, no change.
    if(new_index == _index[1]) {
        return false;
    }

    Location tmp=get_cmd_with_index(new_index);
    if(_check_nav_valid(tmp)) {
        if(_sync_waypoint_index(new_index)) {
            return true;
        }
    }
    return false;
}

bool AP_Mission::get_new_cmd(struct Location &new_CMD)
{
    struct Location temp;
    temp = get_cmd_with_index(_cmd_index);

    if(temp.id <= MAV_CMD_NAV_LAST) {
        return false;                       //no more commands for this leg
    } else {

        //This is required when there is a conditional command prior to a DO_JUMP
        if(temp.id == MAV_CMD_DO_JUMP) {
            if(change_waypoint_index(temp.p1)) {
                temp.lat--;
                set_cmd_with_index(temp, _cmd_index);
            }
        }
        new_CMD=temp;
        _cmd_index++;
        return true;
    }
}

uint8_t AP_Mission::command_total()
{
    return _max_index;
}

void AP_Mission::set_command_total(uint8_t max_index)
{
    _max_index=max_index;
    _command_total->set_and_save(_max_index);
}

void AP_Mission::set_home(const struct Location &home)
{
    _home=home;
    set_cmd_with_index(_home,0);
}

bool AP_Mission::_sync_waypoint_index(const uint8_t &new_index)
{
    Location tmp=get_cmd_with_index(new_index);
    if(new_index <= _max_index) {
        if(_check_nav_valid(tmp)) {
            /* Requested change to home.
             *  -Set previous waypoint as last waypoint in mission.
             *  -Mission Complete, return false.                     */
            if(new_index == 0) {
                _index[0]=_find_nav_index(_max_index, SEARCH_REVERSE);
                _index[1]=0;
                _index[2]=0;
                return false;
                /* Requested last waypoint in mission.
                 *  -Set the after point as home.                        */
            } else if(new_index == _max_index) {
                _index[1]=_max_index;
                _index[2]=0;
            } else {
                _index[1]= new_index;
                _index[2]=_find_nav_index(_index[1]+1, SEARCH_FORWARD);
            }
            _command_index->set_and_save(_index[1]);
            _cmd_index=_index[0]+1; //Reset command index to read commands associated with current mission leg.
            _sync_nav_waypoints();
            return true;
        }
    }
    return false;
}

void AP_Mission::_sync_nav_waypoints(){

    //TODO: this could be optimimzed by making use of the fact some waypoints are already loaded.
    for(int i=0; i<3; i++) {
        nav_waypoints[i]=get_cmd_with_index(_index[i]);
    }
}

/*
 *  Search for the next navigation index in the stack.  Designed to only return a valid index.
 */
uint8_t AP_Mission::_find_nav_index(uint8_t search_index, bool forward)
{
    Location tmp;
    bool condition_cmd=false;

    while(search_index <= _max_index && search_index >= 0) {

        tmp = get_cmd_with_index(search_index);

        //check to see if there is a condition command.
        //don't want to do_jump before the condition command is executed.
        if(tmp.id > MAV_CMD_NAV_LAST && tmp.id < MAV_CMD_CONDITION_LAST) {
            condition_cmd=true;
        }

        //if there is a do_jump without a condition command preceding it, jump now.
        if(tmp.id == MAV_CMD_DO_JUMP && !condition_cmd) {
            if(tmp.p1 <= command_total() && tmp.lat > 0) {
                Location tmp_jump_to;
                tmp_jump_to=get_cmd_with_index(tmp.p1);

                if (_check_nav_valid(tmp_jump_to)) {
                    tmp.lat--;
                    set_cmd_with_index(tmp, search_index);
                    return tmp.p1;
                }
            }
        }

        //otherwise, if we come across a nav command, just pass that index along.
        if(_check_nav_valid(tmp)) {
            return search_index;
        }

        if(forward) {
            search_index++;
        } else {
            search_index--;
        }
    }
    return 0;
}

bool AP_Mission::_check_nav_valid(const struct Location &temp)
{
    /* Other checks could be added such as:
     *   -Distance and altitude diff, if the new waypoint is hundreds of miles away from others, reject.
     *   -others?
     */
    if (temp.id > MAV_CMD_NAV_LAST) {
        return false;
    }

    if ((temp.lat < -900000000 || temp.lat > 900000000) ||
        (temp.lng < -1800000000 || temp.lng > 1800000000)) {
        return false;
    }

    return true;
}

struct Location AP_Mission::get_cmd_with_index_raw(int16_t i)
{
    struct Location temp;
    uint16_t mem;

    // Find out proper location in memory by using the start_byte position + the index
    // --------------------------------------------------------------------------------
    if (i > _max_index) {
        // we do not have a valid command to load
        // return a WP with a "Blank" id
        memset(&temp, 0, sizeof(temp));
        temp.id = CMD_BLANK;

    }else{
        // we can load a command, we don't process it yet
        // read WP position
        mem = (WP_START_BYTE) + (i * WP_SIZE);

        temp.id = hal.storage->read_byte(mem);

        mem++;
        temp.options = hal.storage->read_byte(mem);

        mem++;
        temp.p1 = hal.storage->read_byte(mem);

        mem++;
        temp.alt = hal.storage->read_dword(mem);  // alt is stored in CM! Alt is stored relative!

        mem += 4;
        temp.lat = hal.storage->read_dword(mem); // lat is stored in decimal * 1E7

        mem += 4;
        temp.lng = hal.storage->read_dword(mem); // lon is stored in decimal * 1E7
    }

    return temp;
}

/*
 *  Fetch a mission item from storage. Adjust altitude to be absolute
 */
struct Location AP_Mission::get_cmd_with_index(int16_t i)
{
    struct Location temp;

    temp = get_cmd_with_index_raw(i);

    // Add on home altitude if we are a nav command (or other command with altitude) and stored alt is relative
    if ((temp.id < MAV_CMD_NAV_LAST || temp.id == MAV_CMD_CONDITION_CHANGE_ALT) &&
        (temp.options & MASK_OPTIONS_RELATIVE_ALT) &&
        (temp.lat != 0 || temp.lng != 0 || temp.alt != 0)) {
        temp.alt += _home.alt;
    }

    return temp;
}

void AP_Mission::set_cmd_with_index(struct Location &temp, uint16_t i)
{
    i = constrain_int16(i, 0, command_total());
    uint16_t mem = WP_START_BYTE + (i * WP_SIZE);

    // Set altitude options bitmask
    // XXX What is this trying to do?
    if ((temp.options & MASK_OPTIONS_RELATIVE_ALT) && i != 0) {
        temp.options = MASK_OPTIONS_RELATIVE_ALT;
    } else {
        temp.options = 0;
    }

    hal.storage->write_byte(mem, temp.id);

    mem++;
    hal.storage->write_byte(mem, temp.options);

    mem++;
    hal.storage->write_byte(mem, temp.p1);

    mem++;
    hal.storage->write_dword(mem, temp.alt);

    mem += 4;
    hal.storage->write_dword(mem, temp.lat);

    mem += 4;
    hal.storage->write_dword(mem, temp.lng);
}
