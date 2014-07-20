/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_plane_fbw.pde - init and run calls for loiter flight mode
 */

// plane_init - initialise plane controller
static bool plane_fbw_init(bool ignore_checks)
{
    if ( (!RC_Channel_aux::function_assigned(RC_Channel_aux::k_aileron)) ||
         (!RC_Channel_aux::function_assigned(RC_Channel_aux::k_elevator)) ) {
         
         return false;

    } else {

        return true;
    }
}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
static void plane_fbw_run()
{
        RC_Channel_aux::set_radio(RC_Channel_aux::k_aileron, g.rc_1.radio_in);
        RC_Channel_aux::set_radio(RC_Channel_aux::k_elevator, g.rc_2.radio_in);
        RC_Channel_aux::set_radio(RC_Channel_aux::k_rudder, g.rc_4.radio_in);

        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
}
