// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Rover.h"

/*
  calculate the apparent and true wind direction and speed
  */
void Rover::update_wind(void)
{
    static int16_t ch6_in;
    static int16_t ch7_in;

    ch6_in = RC_Channel::rc_channel(5)->read();
    ch7_in = RC_Channel::rc_channel(6)->read();

    //gcs_send_text_fmt(PSTR("CH6 %d CH7 %d"), ch6_in, ch7_in);

    apparent_wind_dir_sail = (max((ch6_in - 900), 0) * 10) / 3; // should be 1/3 deg per calibration of windsensor
    apparent_wind_speed = max((ch7_in - 900), 0); // should be cm/s per calibration of windsensor
    apparent_wind_dir_boat = approximate_apparent_boat(apparent_wind_dir_sail, last_sail);

    int32_t course = (((ahrs.yaw_sensor / 10) + 1800) % 3600); // note rotated board (1800), check actual compass
    float speed = ahrs.groundspeed();
    //true _wind_dir =
    //true_wind_speed =
}

/*-----------------------------------------------------------------------------------------------
internal_control_mode 1
-----------------------------------------------------------------------------------------------*/

/*
  get apparent with calibration through boat, depreciated
 */
int32_t Rover::get_apparent(void)
{
    int32_t apparent_raw = RC_Channel::rc_channel(5)->read();
    if (apparent_raw < g.wind_min) {
        apparent_raw = g.wind_min;
    }
    if (apparent_raw > g.wind_max) {
        apparent_raw = g.wind_max;
    }
    const int32_t apparent_calib = ((((apparent_raw - g.wind_mid) * 3600) / (g.wind_max - g.wind_min)) + 3600) % 3600;
    //gcs_send_text_fmt(PSTR("Wind: %d"), apparent_calib);
    return apparent_calib;
}

/*
  get difference between apparent and target apparent
 */
int32_t Rover::get_apparent_diff(int32_t apparent)
{
    int32_t diff = (apparent - g.target_apparent);
    if (diff > 1800) {
        diff = - (3600 - diff);
    }
    if (diff < -1800) {
        diff = diff + 3600;
    }
    diff = -diff;
    //Angle between 180 and -180 degrees. Clockwise positive
    return diff;
}

/*
  get rudder value based on apparent
  note fix limit to 40 deg = 400
  ToDo: better controller
 */
int32_t Rover::get_apparent_rudder(int32_t diff)
{
    //int32_t diff = get_apparent_diff(get_apparent());
    diff = diff / 2;

    int32_t result = diff;
    /*
    int32_t result = 0;

    // don't attempt controling if diff small
    int32_t smallDiff = 60;
    bool diffIsBig = ((diff > smallDiff) || (diff < -smallDiff));
    if (diffIsBig) {
        result = diff;
    }
    */

    const int32_t maxAngle = 400;
    result = limit_value(diff, maxAngle);

    if (g.r_servo_upright == 0) {
        result = (int32_t)g.rudder_mid - (((int32_t)result * ((int32_t)g.rudder_mid - (int32_t)g.rudder_min)) / maxAngle);
    } else {
        result = (int32_t)g.rudder_mid + (((int32_t)result * ((int32_t)g.rudder_mid - (int32_t)g.rudder_min)) / maxAngle);
    }

    if (result < g.rudder_min) {
        result = g.rudder_min;
    }
    if (result > g.rudder_max) {
        result = g.rudder_max;
    }

    return result;
}


bool Rover::rudder_diff_is_big(int32_t diff)
{
    // don't attempt controling if diff small
    const int32_t smallDiff = 30;
    const bool diff_is_big = ((diff > smallDiff) || (diff < -smallDiff));
    return diff_is_big ;
}


/*
  get diff
  note fix limit to 65 deg = 650
 */
int32_t Rover::get_apparent_sail(void)
{
    //gcs_send_text_fmt(PSTR("Sail Angle %d"), g.sail_angle);
    const int32_t maxAngle = 650;
    g.sail_angle = limit_value(g.sail_angle, maxAngle);
    const int32_t sailServo = (int32_t)g.sail_mid + (((int32_t)g.sail_angle * ((int32_t)g.sail_max - (int32_t)g.sail_mid)) / maxAngle);
    //gcs_send_text_fmt(PSTR("AS: sa %.1f max %.1f mid %.1f res %.1f"), (double)g.sail_angle, (double)g.sail_max, (double)g.sail_mid, (double)result);
    return sailServo;
}

/*----------------------------------------------------------------------------------------------------
internal_control_mode 2
-----------------------------------------------------------------------------------------------------*/
int32_t Rover::approximate_apparent_boat(int32_t apparent, int32_t sail)
{
    return -((((sail - g.sail_mid) * (60000 / (g.sail_max - g.sail_mid))) / 100) - apparent);
}


int32_t Rover::approximate_sail(int32_t apparent_boat)
{
    int32_t result = 0;

    if (apparent_boat < 1800) {
        result = (int32_t)g.sail_mid - (((int32_t)apparent_boat * 100) / (180000 / (int32_t)(g.sail_max - g.sail_mid)));
    } else {
        result = (int32_t)g.sail_mid + (((3600 - (int32_t)apparent_boat) * 100) / (180000 / (int32_t)(g.sail_max - g.sail_mid)));
    }
    return result;
}


int32_t Rover::get_course_sail(int32_t apparent, int32_t sail)
{
    int32_t result = sail;
    const int32_t apparent_boat = approximate_apparent_boat(apparent,sail);
    if ((apparent_boat < 1700) || (apparent_boat > 1900)) {
        result = approximate_sail(apparent_boat);
    } else {
        if (sail < g.sail_mid) {
            result = g.sail_min;
        } else {
            result = g.sail_max;
        }
    }

    if (result > sail) {
        if ((result - sail) < g.sail_gap) {
            result = sail;
        }
        if ((result - sail) > g.sail_step) {
            result = sail + g.sail_step;
        }
    } else {
        if ((result - sail) > -g.sail_gap) {
            result = sail;
        }
        if ((result - sail) < -g.sail_step) {
            result = sail - g.sail_step;
        }
    }

    if (result > g.sail_max) result = g.sail_max;
    if (result < g.sail_min) result = g.sail_min;

    return result;
}


/*
  get course / heading, note rotated board (180 deg)
 */
int32_t Rover::get_course(void)
{
    return (int32_t) (((ahrs.yaw_sensor / 10) + 1800) % 3600);
}


/*
  get diff
 */
int32_t Rover::get_course_diff(int32_t course)
{
    int32_t result = (course - g.target_course);
    if (result > 1800) {
        result = - (3600 - result);
    }
    return result;
}


/*
  get diff
  ToDo: define parameters in g for correct scaling
  1012 should be replaced by parameter for full circle
 */
int32_t Rover::get_course_rudder(void)
{
    int32_t diff = get_course_diff(get_course());
    diff = diff / 2;

    int32_t result = 0;

    // don't attempt controling if diff small
    const int32_t smallDiff = 60;
    const bool diffIsBig = ((diff > smallDiff) || (diff < -smallDiff));
    if (diffIsBig) {
        result = diff;
    }

    const int32_t maxAngle = 400;
    result = limit_value(diff, maxAngle);

    return g.rudder_mid - result;
}


/*----------------------------------------------------------------------------------------------------
Opens the sail if the boat is on its side, to counter roll
-----------------------------------------------------------------------------------------------------*/
bool Rover::roll_is_normal(void)
{
    const int16_t current_roll = (int16_t)(degrees(ahrs.roll) * 10);
    const int16_t max_roll_angle = 700;
    if (abs(current_roll) > max_roll_angle) {
        return false;
    }
    return true;
}

void Rover::react_extreme_roll(void)
{
    const int16_t current_roll = (int16_t)(degrees(ahrs.roll) * 10);
    if (current_roll < 0) {
        //RC_Channel::rc_channel(2)->radio_out = g.sail_max;
        //RC_Channel::rc_channel(2)->radio_out = sail_pid.get_pid(g.sail_max);
        RC_Channel::rc_channel(2)->radio_out = servoRamp(last_sail, g.sail_max, last_step_sail);
    } else {
        //RC_Channel::rc_channel(2)->radio_out = g.sail_min;
        //RC_Channel::rc_channel(2)->radio_out = sail_pid.get_pid(g.sail_min);
        RC_Channel::rc_channel(2)->radio_out = servoRamp(last_sail, g.sail_min, last_step_sail);
    }
}

/*----------------------------------------------------------------------------------------------------
Called in scheduler
-----------------------------------------------------------------------------------------------------*/
/*
  calculate the sail and rudder servo angles
  */
void Rover::update_servos(void)
{
    //gcs_send_text_fmt(PSTR("last_sail: %d"), RC_Channel::rc_channel(2)->radio_out);
    static int16_t last_throttle;
    static int16_t ch1_in;
    static int16_t ch2_in;
    static int16_t ch3_in;
    int16_t internal_control_mode = 0;

    // support a separate steering channel
    RC_Channel_aux::set_servo_out(RC_Channel_aux::k_steering, channel_steer->pwm_to_angle_dz(0));

	if (control_mode == MANUAL || control_mode == LEARNING) {
        // boat
        ch1_in = RC_Channel::rc_channel(0)->read(); // rudder
        ch2_in = RC_Channel::rc_channel(1)->read(); // used to indicate override
        ch3_in = RC_Channel::rc_channel(2)->read(); // sail

        // check if channel 2 is 'pulled' --> manual override
        if (ch2_in > 1700) {
            internal_control_mode = 0;
        } else {
            internal_control_mode = g.target_mode;
        }

        if (internal_control_mode == 1) {
            //gcs_send_text_fmt(PSTR("Sail_max: %d"), g.sail_max);
            //RC_Channel::rc_channel(2)->radio_out = 2000;
            //RC_Channel::rc_channel(0)->radio_out = g.rudder_max;
            //RC_Channel::rc_channel(2)->radio_out = target_sail;
            if (roll_is_normal()) {
                //gcs_send_text_fmt(PSTR("Roll normal"));
                const int16_t target_sail = get_apparent_sail();
                RC_Channel::rc_channel(2)->radio_out = servoRamp(last_sail, target_sail, last_step_sail);
                //RC_Channel::rc_channel(2)->radio_out = sail_pid.get_pid(get_apparent_sail());
            } else {
                //gcs_send_text_fmt(PSTR("Roll normal"));
                react_extreme_roll();
            }
            const int32_t rudder_diff = get_apparent_diff(get_apparent());
            if (rudder_diff_is_big(rudder_diff)) {
                //gcs_send_text_fmt(PSTR("Change Rudder"));
                const int16_t target_rudder = get_apparent_rudder(rudder_diff);
                RC_Channel::rc_channel(0)->radio_out = servoRamp(last_rudder, target_rudder, last_step_rudder);
            }
            //RC_Channel::rc_channel(0)->radio_out = target_rudder;
            //RC_Channel::rc_channel(0)->radio_out = rudder_pid.get_pid(get_apparent_rudder());
        } else if (internal_control_mode == 2) {
            RC_Channel::rc_channel(0)->radio_out = get_course_rudder();
            RC_Channel::rc_channel(2)->radio_out = get_course_sail(get_apparent(),last_sail);
        } else if (internal_control_mode == 3) {
            RC_Channel::rc_channel(0)->radio_out = RC_Channel::rc_channel(0)->read();
            RC_Channel::rc_channel(2)->radio_out = RC_Channel::rc_channel(2)->read();
        } else {
            calc_scaled_manual_steering();
        }
        const int32_t sail_tmp = RC_Channel::rc_channel(2)->radio_out;
        const int32_t rudder_tmp = RC_Channel::rc_channel(0)->radio_out;
        last_step_sail = last_sail - sail_tmp;
        last_step_rudder = last_rudder - rudder_tmp;
        last_rudder = rudder_tmp;
        last_sail = sail_tmp;
        gcs_send_message(MSG_RADIO_OUT);
        //gcs_send_text_fmt(PSTR("last rudder: %d"), last_rudder);
        /*
        if (failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
            // suppress throttle if in failsafe and manual
            channel_throttle->radio_out = channel_throttle->radio_trim;
        }*/

	} else {
        channel_steer->calc_pwm();
        if (in_reverse) {
            channel_throttle->servo_out = constrain_int16(channel_throttle->servo_out,
                                                          -g.throttle_max,
                                                          -g.throttle_min);
        } else {
            channel_throttle->servo_out = constrain_int16(channel_throttle->servo_out,
                                                          g.throttle_min.get(),
                                                          g.throttle_max.get());
        }

        if ((failsafe.bits & FAILSAFE_EVENT_THROTTLE) && control_mode < AUTO) {
            // suppress throttle if in failsafe
            channel_throttle->servo_out = 0;
        }

        // convert 0 to 100% into PWM
        channel_throttle->calc_pwm();

        // limit throttle movement speed
        throttle_slew_limit(last_throttle);
    }

    // record last throttle before we apply skid steering
    last_throttle = channel_throttle->radio_out;

    if (g.skid_steer_out) {
        // convert the two radio_out values to skid steering values
          //mixing rule:
          //steering = motor1 - motor2
          //throttle = 0.5*(motor1 + motor2)
          //motor1 = throttle + 0.5*steering
          //motor2 = throttle - 0.5*steering
        float steering_scaled = channel_steer->norm_output();
        float throttle_scaled = channel_throttle->norm_output();
        float motor1 = throttle_scaled + 0.5f*steering_scaled;
        float motor2 = throttle_scaled - 0.5f*steering_scaled;
        channel_steer->servo_out = 4500*motor1;
        channel_throttle->servo_out = 100*motor2;
        channel_steer->calc_pwm();
        channel_throttle->calc_pwm();
    }


#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
	// send values to the PWM timers for output
	// ----------------------------------------
    channel_steer->output();
    channel_throttle->output();
    RC_Channel_aux::output_ch_all();
#endif
}

int32_t Rover::limit_value(int32_t value, int32_t limit)
{
    if (value > limit) {
        value = limit;
    } else if (value < -limit) {
        value = -limit;
    }
    return value;
}