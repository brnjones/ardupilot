/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_plane_fbw.pde - init and run calls for loiter flight mode
 */

// plane_init - initialise plane controller
static bool plane_fbw_init(bool ignore_checks)
{
    rollController.reset_I();
    pitchController.reset_I();
    yawController.reset_I(); 

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

    // set nav_roll and nav_pitch using sticks
    nav_roll_cd  = g.rc_1.norm_input() * roll_limit_cd;
    nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
    float pitch_input = g.rc_2.norm_input();
    if (pitch_input > 0) {
        nav_pitch_cd = pitch_input * plane_aparm.pitch_limit_max_cd;
    } else {
        nav_pitch_cd = -(pitch_input * pitch_limit_min_cd);
    }
    nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min_cd, plane_aparm.pitch_limit_max_cd.get());
    // TODO(BrandonJ): Uncomment this on next commit
    // stabilize_plane();
    
    /*
    if (failsafe.ch3_failsafe && g.short_fs_action == 2) {
        // FBWA failsafe glide
        nav_roll_cd = 0;
        nav_pitch_cd = 0;
        channel_throttle->servo_out = 0;
    }
    */
    
   
    /*
      see if we should zero the attitude controller integrators. 
     */

     /*
    if (channel_throttle->control_in == 0 &&
        relative_altitude_abs_cm() < 500 && 
        fabsf(barometer.get_climb_rate()) < 0.5f &&
        gps.ground_speed() < 3) {
        // we are low, with no climb rate, and zero throttle, and very
        // low ground speed. Zero the attitude controller
        // integrators. This prevents integrator buildup pre-takeoff.
        rollController.reset_I();
        pitchController.reset_I();
        yawController.reset_I();
    }
    */
/*
        RC_Channel_aux::set_radio(RC_Channel_aux::k_aileron, g.rc_1.radio_in);
        RC_Channel_aux::set_radio(RC_Channel_aux::k_elevator, g.rc_2.radio_in);
        RC_Channel_aux::set_radio(RC_Channel_aux::k_rudder, g.rc_4.radio_in);
*/
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
}
