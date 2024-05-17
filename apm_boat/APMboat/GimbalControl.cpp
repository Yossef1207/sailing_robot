/*
This file includes functions for controlling the camera gimbal.
*/

#include "Rover.h"

// helper function to check if a calculated servo value lays inside the reachable range
int Rover::in_servo_range(int target_value, int servo_min, int servo_max){
    if(target_value <= servo_min){
        return servo_min;
    }else if(target_value >= servo_max){
        return servo_max;
    }else{
        return target_value;
    }
}

// calculate the mean over all values in an array
int Rover::array_mean(int *arr, int array_length){
    int sum = 0;
    for(int i = 0; i < array_length; i++){
        sum += arr[i];
    }
    return round((float)sum / array_length);
}

// shift the values in an array by one
void Rover::shift_array (int *arr, int array_length){
    for(int i = (array_length-1); i >= 1; i--){
        arr[i] = arr[i-1];
    }
}

// calculate a PWM value for a target angle

int Rover::angle_to_PWM(int current_value, int range_of_motion_degrees, int servo_min, int servo_mid, int servo_max, int target){
    int error = target - current_value;
    if(error > 180){
        error -= 360;
    }else if(error < -180){
        error += 360;
    }
    float PWM_per_angle = (float) (servo_max-servo_min)/range_of_motion_degrees;
    float out = (float) servo_mid - error*PWM_per_angle;
    return round(out);
}

// update function for adjusting the roll servo
void Rover::gimbal_adjust_roll(void){
    int16_t output;
    int target_roll = 0;

    last_roll_servo_PWM_values[0] = angle_to_PWM(degrees(ahrs.roll), g.roll_range, g.camera_roll_min, g.camera_roll_mid, g.camera_roll_max, target_roll);
    output = array_mean(last_roll_servo_PWM_values, sizeof(last_roll_servo_PWM_values)/sizeof(*last_roll_servo_PWM_values));
    shift_array(last_roll_servo_PWM_values, sizeof(last_roll_servo_PWM_values)/sizeof(*last_roll_servo_PWM_values));
    output = in_servo_range(output, g.camera_roll_min, g.camera_roll_max);

    RC_Channel::rc_channel(5)->radio_out=output;
}

// update function for adjusting the yaw servo
void Rover::gimbal_adjust_yaw(void){
    int16_t output;

    last_yaw_servo_PWM_values[0] = angle_to_PWM(round(degrees(ahrs.yaw)), g.yaw_range, g.camera_yaw_min, g.camera_yaw_mid, g.camera_yaw_max, g.cam_yaw_target);
    output = array_mean(last_yaw_servo_PWM_values, sizeof(last_yaw_servo_PWM_values)/sizeof(*last_yaw_servo_PWM_values));
    shift_array(last_yaw_servo_PWM_values, sizeof(last_yaw_servo_PWM_values)/sizeof(*last_yaw_servo_PWM_values));
    output = in_servo_range(output, g.camera_yaw_min, g.camera_yaw_max);

    RC_Channel::rc_channel(4)->radio_out=output;
}
