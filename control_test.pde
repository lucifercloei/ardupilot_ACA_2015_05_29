/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_test.pde - init and run calls for loiter flight mode
 */

// test_init - initialise test controller
static bool test_init(bool ignore_checks)
{
    if (position_ok() || optflow_position_ok() || ignore_checks) {

        // set target to current position
        wp_nav.init_loiter_target();

        // initialize vertical speed and accelerationj
        pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control.set_accel_z(g.pilot_accel_z);

        // initialise altitude target to stopping point
        pos_control.set_target_to_stopping_point_z();

        return true;
    }else{
        return false;
    }
}

// test_run - runs the loiter controller
// should be called at 100hz or more
static void test_run()
{
}
