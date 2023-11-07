#include "Rover.h"
float loiter_radius0 = 2.1;
float loiter_radius00 = 2.1;
float loiter_ratio = 2.1;

bool ModeLoiter::_enter()
{
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "loitering mode started");
    // set _destination to reasonable stopping point
    if (!g2.wp_nav.get_stopping_location(_destination)) {
        return false;
    }

    // initialise desired speed to current speed
    if (!attitude_control.get_forward_speed(_desired_speed)) {
        _desired_speed = 0.0f;
    }

    // initialise heading to current heading
    _desired_yaw_cd = ahrs.yaw_sensor;

    loiter_radius0   = g2.loit_radius;
    loiter_ratio     = g2.loit_radius_rat;
    loiter_radius00  = g2.loit_radius_rat*g2.loit_radius;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "g2.loit_radius: %f", loiter_radius0);
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "g2.loit_radius_rat: %f", loiter_ratio);
    
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "inner radius: %f", loiter_radius0);
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "outer radius: %f", loiter_radius00);

    return true;
}

void ModeLoiter::update()
{
    // get distance (in meters) to destination
    _distance_to_destination = rover.current_loc.get_distance(_destination);
    // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Best number ever: %f", 42.4242424242);
    // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "_distance_to_destination: %f", _distance_to_destination);

    //const float loiter_radius = rover.g2.sailboat.tack_enabled() ? g2.sailboat.get_loiter_radius() : g2.loit_radius;
    
    const float loiter_radius1 = g2.loit_radius;
    const float loiter_radius2 = g2.loit_radius_rat*g2.loit_radius;
  
    // multiple radius work well, go back to within first radius if greater than second radius
    // if _distance_to_destination <= loiter_radius2, then change to if _distance_to_destination <= loiter_radius1,
    // if _distance_to_destination <= loiter_radius1, then change to if _distance_to_destination <= loiter_radius2,
    if (_distance_to_destination <= loiter_radius1) {
        if (loiter_radius0 < loiter_radius2) {
            loiter_radius0 = loiter_radius2;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "within loiter radius 1, %d m, motors off", (int)loiter_radius1);
        }
    }
    if (_distance_to_destination > loiter_radius2) {
        if (loiter_radius0 > loiter_radius1) {
            loiter_radius0 = loiter_radius1;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "out of loiter radius 2, %d m, motors on", (int)loiter_radius2);    
        }
    }
    const float loiter_radius = loiter_radius0;
   
    

    // if within loiter radius slew desired speed towards zero and use existing desired heading
    if (_distance_to_destination <= loiter_radius) {
        // sailboats should not stop unless motoring
        //const float desired_speed_within_radius = rover.g2.sailboat.tack_enabled() ? 0.1f : 0.0f;
        //_desired_speed = attitude_control.get_desired_speed_accel_limited(desired_speed_within_radius, rover.G_Dt);

        g2.motors.set_throttle(0); // turn off throttle
        g2.motors.set_steering(0); // turn off steering
        
        // if we want to drift (not point in the same direction) then reinitialise heading to current heading
        //_desired_yaw_cd = ahrs.yaw_sensor;
        

        // if we have a sail but not trying to use it then point into the wind
       //if (!rover.g2.sailboat.tack_enabled() && rover.g2.sailboat.sail_enabled()) {
       //     _desired_yaw_cd = degrees(g2.windvane.get_true_wind_direction_rad()) * 100.0f;
       // }
    } else {
        // P controller with hard-coded gain to convert distance to desired speed
        _desired_speed = MIN((_distance_to_destination - loiter_radius) * g2.loiter_speed_gain, g2.wp_nav.get_default_speed());

        // calculate bearing to destination
        _desired_yaw_cd = rover.current_loc.get_bearing_to(_destination);
        float yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
        // if destination is behind vehicle, reverse towards it
        if ((fabsf(yaw_error_cd) > 9000 && g2.loit_type == 0) || g2.loit_type == 2) {
            _desired_yaw_cd = wrap_180_cd(_desired_yaw_cd + 18000);
            yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
            _desired_speed = -_desired_speed;
        }

        // reduce desired speed if yaw_error is large
        // 45deg of error reduces speed to 75%, 90deg of error reduces speed to 50%
        float yaw_error_ratio = 1.0f - constrain_float(fabsf(yaw_error_cd / 9000.0f), 0.0f, 1.0f) * 0.5f;
        _desired_speed *= yaw_error_ratio;
    

        
        // 0 turn rate is no limit (I changed this to 1.0, very very slow, DLC 2022)
        float turn_rate = 0.0;

        // make sure sailboats don't try and sail directly into the wind
        if (g2.sailboat.use_indirect_route(_desired_yaw_cd)) {
            _desired_yaw_cd = g2.sailboat.calc_heading(_desired_yaw_cd);
            if (g2.sailboat.tacking()) {
                // use pivot turn rate for tacks
                turn_rate = g2.wp_nav.get_pivot_rate();
            }
        }

        // run steering and throttle controllers
        calc_steering_to_heading(_desired_yaw_cd, turn_rate);
        calc_throttle(_desired_speed, true);

    }
    //float throttle_num = g2.motors.get_throttle();
    //GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "_distance_to_destination: %f, loiter_radius: %f, desired_speed: %f, throttle: %f", _distance_to_destination,loiter_radius,_desired_speed,throttle_num);
}

// get desired location
bool ModeLoiter::get_desired_location(Location& destination) const
{
    destination = _destination;
    return true;
}
