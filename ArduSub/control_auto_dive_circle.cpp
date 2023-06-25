#include "Sub.h"

/*
 * Init and run calls for auto dive mode
 */


bool Sub::auto_dive_circle_init(bool ignore_checks)
{
    if (!position_ok() && !ignore_checks) {
        return false;
    }
    
    circle_pilot_yaw_override = false;

    // initialize speeds and accelerations
    pos_control.set_max_speed_accel_xy(wp_nav.get_default_speed_xy(), wp_nav.get_wp_acceleration());
    pos_control.set_correction_speed_accel_xy(wp_nav.get_default_speed_xy(), wp_nav.get_wp_acceleration());
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control.set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise circle controller including setting the circle center based on vehicle speed
    circle_nav.init();
    return true;
}

void Sub::auto_dive_circle_run()
{
    // there is a pressure sensor for reading depth instead of barometer.

    read_barometer();
    float current_depth = barometer.get_altitude();

    // when vehicle is not armed....
     if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        // initialise velocity controller
        pos_control.init_z_controller();
        pos_control.init_xy_controller();
        return;
    }

    float target_yaw_rate = 0;
    if (!failsafe.pilot_input) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // Set target depth(cm)  
    float target_z = -1000;
    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // when motors are spooling up.... running the attitude control 
    if (motors.get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        pos_control.relax_velocity_controller_xy();
        pos_control.update_xy_controller();
        pos_control.relax_z_controller(0.0f);  
        pos_control.update_z_controller();
        attitude_control.reset_yaw_target_and_rate();
        attitude_control.reset_rate_controller_I_terms();
        attitude_control.input_thrust_vector_rate_heading(pos_control.get_thrust_vector(), 0.0);
        target_z = inertial_nav.get_position_z_up_cm() + -1000;
        return;
    }

    // target depth 
    if (target_z < current_depth) {
        pos_control.set_pos_target_z_cm(target_z);
        pos_control.update_z_controller();
    }
    if (target_z > current_depth){
        control_depth();
        return;
    }
}

