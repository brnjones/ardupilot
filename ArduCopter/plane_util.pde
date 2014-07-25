/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * plane_util.pde - plane utility functions
 */

/*
  get a speed scaling number for control surfaces. This is applied to
  PIDs to change the scaling of the PID with speed. At high speed we
  move the surfaces less, and at low speeds we move them more.
 */
static float get_speed_scaler(void)
{
	/*
    float aspeed, speed_scaler;
    if (ahrs.airspeed_estimate(&aspeed)) {
        if (aspeed > auto_state.highest_airspeed) {
            auto_state.highest_airspeed = aspeed;
        }
        if (aspeed > 0) {
            speed_scaler = g.scaling_speed / aspeed;
        } else {
            speed_scaler = 2.0;
        }
        speed_scaler = constrain_float(speed_scaler, 0.5, 2.0);
    } else {
        if (channel_throttle->servo_out > 0) {
            speed_scaler = 0.5f + ((float)THROTTLE_CRUISE / channel_throttle->servo_out / 2.0f);                 // First order taylor expansion of square root
            // Should maybe be to the 2/7 power, but we aren't goint to implement that...
        }else{
            speed_scaler = 1.67f;
        }
        // This case is constrained tighter as we don't have real speed info
        speed_scaler = constrain_float(speed_scaler, 0.6, 1.67);
    }
    return speed_scaler;
    */
    return 1.0;
}

/*
  this is the main roll stabilization function. It takes the
  previously set nav_roll calculates roll servo_out to try to
  stabilize the plane at the given roll
 */
static void stabilize_roll(float speed_scaler)
{


    bool disable_integrator = false;
    if (control_mode == STABILIZE && channel_roll->control_in != 0) {
        disable_integrator = true;
    }
    channel_roll->servo_out = rollController.get_servo_out(nav_roll_cd - ahrs.roll_sensor, 
                                                           speed_scaler, 
                                                           disable_integrator);
}

/*
  this is the main pitch stabilization function. It takes the
  previously set nav_pitch and calculates servo_out values to try to
  stabilize the plane at the given attitude.
 */
static void stabilize_pitch(float speed_scaler)
{
	/*
    int8_t force_elevator = takeoff_tail_hold();
    if (force_elevator != 0) {
        // we are holding the tail down during takeoff. Just covert
        // from a percentage to a -4500..4500 centidegree angle
        channel_pitch->servo_out = 45*force_elevator;
        return;
    }
    */

    int32_t demanded_pitch = nav_pitch_cd + g.pitch_trim_cd; // + channel_throttle->servo_out * g.kff_throttle_to_pitch;
    bool disable_integrator = false;
    if (control_mode == STABILIZE && channel_pitch->control_in != 0) {
        disable_integrator = true;
    }
    channel_pitch->servo_out = pitchController.get_servo_out(demanded_pitch - ahrs.pitch_sensor, 
                                                             speed_scaler, 
                                                             disable_integrator);
}
/*
  stabilize the yaw axis. There are 3 modes of operation:

    - hold a specific heading with ground steering
    - rate controlled with ground steering
    - yaw control for coordinated flight    
 */


static void stabilize_yaw(float speed_scaler)
{
	//todo

}

static void stabilize_plane() {
    float speed_scaler = get_speed_scaler();

    stabilize_roll(speed_scaler);
    stabilize_pitch(speed_scaler);
    stabilize_yaw(speed_scaler);
}

static void scale_roll_limit() {
	// calculate a scaled roll limit based on current pitch
    roll_limit_cd = g.roll_limit_cd * cosf(ahrs.pitch);
    pitch_limit_min_cd = plane_aparm.pitch_limit_min_cd * fabsf(cosf(ahrs.roll));
}