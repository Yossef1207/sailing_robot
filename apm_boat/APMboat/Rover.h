/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
   main Rover class, containing all vehicle specific state
*/

#ifndef _ROVER_H_
#define _ROVER_H_

#define THISFIRMWARE "ArduRover v2.52-beta"
#define FIRMWARE_VERSION 2,52,0,FIRMWARE_VERSION_TYPE_BETA

#include <math.h>
#include <stdarg.h>

// Libraries
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Menu/AP_Menu.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_GPS/AP_GPS.h>         // ArduPilot GPS library
#include <AP_ADC/AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_InertialSensor/AP_InertialSensor.h> // Inertial Sensor (uncalibated IMU) Library
#include <AP_AHRS/AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_Mission/AP_Mission.h>     // Mission command library
#include <AP_Rally/AP_Rally.h>
#include <AP_Terrain/AP_Terrain.h>
#include <PID/PID.h>            // PID library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include <AP_RangeFinder/AP_RangeFinder.h>	// Range finder library
#include <Filter/Filter.h>			// Filter library
#include <Filter/Butter.h>			// Filter library - butterworth filter
#include <AP_Buffer/AP_Buffer.h>      // FIFO buffer library
#include <Filter/ModeFilter.h>		// Mode Filter from Filter library
#include <Filter/AverageFilter.h>	// Mode Filter from Filter library
#include <AP_Relay/AP_Relay.h>       // APM relay
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
#include <AP_Mount/AP_Mount.h>		// Camera/Antenna mount
#include <AP_Camera/AP_Camera.h>		// Camera triggering
#include <GCS_MAVLink/GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_SerialManager/AP_SerialManager.h>   // Serial manager library
#include <AP_Airspeed/AP_Airspeed.h>    // needed for AHRS build
#include <AP_Vehicle/AP_Vehicle.h>     // needed for AHRS build
#include <DataFlash/DataFlash.h>
#include <AP_RCMapper/AP_RCMapper.h>        // RC input mapping library
#include <SITL/SITL.h>
#include <AP_Scheduler/AP_Scheduler.h>       // main loop scheduler
#include <stdarg.h>
#include <AP_Navigation/AP_Navigation.h>
#include <APM_Control/APM_Control.h>
#include <AP_L1_Control/AP_L1_Control.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Frsky_Telem/AP_Frsky_Telem.h>

#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_HAL_SITL/AP_HAL_SITL.h>
#include <AP_HAL_PX4/AP_HAL_PX4.h>
#include <AP_HAL_VRBRAIN/AP_HAL_VRBRAIN.h>
#include <AP_HAL_FLYMAPLE/AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include "compat.h"

#include <AP_Notify/AP_Notify.h>      // Notify library
#include <AP_BattMonitor/AP_BattMonitor.h> // Battery monitor library
#include <AP_OpticalFlow/AP_OpticalFlow.h>     // Optical Flow library

// Configuration
#include "config.h"

// Local modules
#include "defines.h"
#include "Parameters.h"
#include <GCS_MAVLink/GCS.h>

#include <AP_Declination/AP_Declination.h> // ArduPilot Mega Declination Helper Library

class Rover {
public:
    friend class GCS_MAVLINK;
    friend class Parameters;

    Rover(void);

    // public member functions
    void setup(void);
    void loop(void);

//####################################################
//Below: variabels for the fuctions are defined:

private:
    // ########################################
    int last_roll_servo_PWM_values[3]={0,g.camera_roll_mid,g.camera_roll_mid};
    int last_yaw_servo_PWM_values[3]={0,g.camera_yaw_mid, g.camera_yaw_mid};
    //#########################################


    AP_HAL::BetterStream* cliSerial;

    // must be the first AP_Param variable declared to ensure its
    // constructor runs before the constructors of the other AP_Param
    // variables
    AP_Param param_loader;

    // the rate we run the main loop at
    const AP_InertialSensor::Sample_rate ins_sample_rate;

    // all settable parameters
    Parameters g;

    // main loop scheduler
    AP_Scheduler scheduler;

    // mapping between input channels
    RCMapper rcmap;

    // board specific config
    AP_BoardConfig BoardConfig;

    // primary control channels
    RC_Channel *channel_steer;
    RC_Channel *channel_throttle;
    RC_Channel *channel_learn;
    RC_Channel *channel_wind;

    DataFlash_Class DataFlash;

    bool in_log_download;

    // sensor drivers
    AP_GPS gps;
    AP_Baro barometer;
    Compass compass;
    AP_InertialSensor ins;
    RangeFinder sonar;

    // flight modes convenience array
    AP_Int8	*modes;

// Inertial Navigation EKF
#if AP_AHRS_NAVEKF_AVAILABLE
    NavEKF EKF{&ahrs, barometer, sonar};
    AP_AHRS_NavEKF ahrs {ins, barometer, gps, sonar, EKF};
#else
    AP_AHRS_DCM ahrs {ins, barometer, gps};
#endif

    AP_L1_Control L1_controller;

    // selected navigation controller
    AP_Navigation *nav_controller;

    // steering controller
    AP_SteerController steerController;

    // Mission library
    AP_Mission mission;

    OpticalFlow optflow;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL sitl;
#endif

    // GCS handling
    AP_SerialManager serial_manager;
    const uint8_t num_gcs;
    GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];

    // a pin for reading the receiver RSSI voltage. The scaling by 0.25
    // is to take the 0 to 1024 range down to an 8 bit range for MAVLink
    AP_HAL::AnalogSource *rssi_analog_source;

    // relay support
    AP_Relay relay;

    AP_ServoRelayEvents ServoRelayEvents;

    // Camera
#if CAMERA == ENABLED
    AP_Camera camera;
#endif

    // The rover's current location
    struct Location current_loc;

    // Camera/Antenna mount tracking and stabilisation stuff
#if MOUNT == ENABLED
    // current_loc uses the baro/gps soloution for altitude rather than gps only.
    AP_Mount camera_mount;
#endif

    // if USB is connected
    bool usb_connected;

    // Radio
    // This is the state of the flight control system
    // There are multiple states defined such as MANUAL, FBW-A, AUTO
    enum mode control_mode;

    // Used to maintain the state of the previous control switch position
    // This is set to -1 when we need to re-read the switch
    uint8_t oldSwitchPosition;

    // These are values received from the GCS if the user is using GCS joystick
    // control and are substituted for the values coming from the RC radio
    int16_t rc_override[8];

    // A flag if GCS joystick control is in use
    bool rc_override_active;

    // Failsafe
    // A tracking variable for type of failsafe active
    // Used for failsafe based on loss of RC signal or GCS signal. See
    // FAILSAFE_EVENT_*
    struct {
        uint8_t bits;
        uint32_t rc_override_timer;
        uint32_t start_time;
        uint8_t triggered;
        uint32_t last_valid_rc_ms;
    } failsafe;

    // notification object for LEDs, buzzers etc (parameter set to false disables external leds)
    AP_Notify notify;

    // A counter used to count down valid gps fixes to allow the gps estimate to settle
    // before recording our home position (and executing a ground start if we booted with an air start)
    uint8_t ground_start_count;

    // true if we have a position estimate from AHRS
    bool have_position;

    bool rtl_complete;

    // angle of our next navigation waypoint
    int32_t next_navigation_leg_cd;

    // ground speed error in m/s
    float groundspeed_error;

    // 0-(throttle_max - throttle_cruise) : throttle nudge in Auto mode using top 1/2 of throttle stick travel
    int16_t     throttle_nudge;

    // receiver RSSI
    uint8_t receiver_rssi;

    // the time when the last HEARTBEAT message arrived from a GCS
    uint32_t last_heartbeat_ms;

    // obstacle detection information
    struct {
        // have we detected an obstacle?
        uint8_t detected_count;
        float turn_angle;
        uint16_t sonar1_distance_cm;
        uint16_t sonar2_distance_cm;

        // time when we last detected an obstacle, in milliseconds
        uint32_t detected_time_ms;
    } obstacle;

    // this is set to true when auto has been triggered to start
    bool auto_triggered;

    // Ground speed
    // The amount current ground speed is below min ground speed.  meters per second
    float 	ground_speed;
    int16_t throttle_last;
    int16_t throttle;

    // CH7 control
    // Used to track the CH7 toggle state.
    // When CH7 goes LOW PWM from HIGH PWM, this value will have been set true
    // This allows advanced functionality to know when to execute
    bool ch7_flag;

    // Battery Sensors
    AP_BattMonitor battery;

    // Battery Sensors
#if FRSKY_TELEM_ENABLED == ENABLED
    AP_Frsky_Telem frsky_telemetry;
#endif

    // Navigation control variables
    // The instantaneous desired lateral acceleration in m/s/s
    float lateral_acceleration;

    // Waypoint distances
    // Distance between rover and next waypoint.  Meters
    float wp_distance;
    // Distance between previous and next waypoint.  Meters
    int32_t wp_totalDistance;

    // Conditional command
    // A value used in condition commands (eg delay, change alt, etc.)
    // For example in a change altitude command, it is the altitude to change to.
    int32_t condition_value;
    // A starting value used to check the status of a conditional command.
    // For example in a delay command the condition_start records that start time for the delay
    int32_t	condition_start;

    // 3D Location vectors
    // Location structure defined in AP_Common
    // The home location used for RTL.  The location is set when we first get stable GPS lock
    const struct Location &home;

    // Flag for if we have gps lock and have set the home location
    bool home_is_set;
    // The location of the previous waypoint.  Used for track following and altitude ramp calculations
    struct Location prev_WP;
    // The location of the current/active waypoint.  Used for track following
    struct Location next_WP;
    // The location of the active waypoint in Guided mode.
    struct Location guided_WP;

    // IMU variables
    // The main loop execution time.  Seconds
    // This is the time between calls to the DCM algorithm and is the Integration time for the gyros.
    float G_Dt;

    // Performance monitoring
    // Timer used to accrue data and trigger recording of the performanc monitoring log message
    int32_t	perf_mon_timer;
    // The maximum main loop execution time recorded in the current performance monitoring interval
    uint32_t G_Dt_max;

    // System Timers
    // Time in microseconds of start of main control loop.
    uint32_t fast_loopTimer_us;
    // Number of milliseconds used in last main loop cycle
    uint32_t delta_us_fast_loop;
    // Counter of main loop executions.  Used for performance monitoring and failsafe processing
    uint16_t mainLoop_count;

    // set if we are driving backwards
    bool in_reverse;

    static const AP_Scheduler::Task scheduler_tasks[];

    // use this to prevent recursion during sensor init
    bool in_mavlink_delay;

    // true if we are out of time in our event timeslice
    bool gcs_out_of_time;

    static const AP_Param::Info var_info[];
    static const LogStructure log_structure[];

    // Loiter control
    uint16_t loiter_duration; // How long we should loiter at the nav_waypoint (time in seconds)
    uint32_t loiter_start_time;     // How long have we been loitering - The start time in millis
    bool active_loiter; // TRUE if we actively return to the loitering waypoint if we drift off
    bool previously_reached_wp;  // set to true if we have EVER reached the waypoint
    float distance_past_wp; // record the distance we have gone past the wp

    // boat state
    int16_t last_sail = 1300;
    int16_t last_rudder = 1300;
    int16_t last_step_sail = 0;
    int16_t last_step_rudder = 0;
    int16_t calm_down_counter = 0;
    int16_t apparent_wind_dir_boat = 0;
    int16_t apparent_wind_dir_sail = 0;
    int16_t apparent_wind_speed = 0;
    int16_t true_wind_dir = 0;
    int16_t true_wind_speed = 0;
    PID rudder_pid;
    PID sail_pid;

//####################################################
//Below: Tasks for the scheduler are defined:

private:
    // private member functions

    int  in_servo_range(int target_value, int servo_min, int servo_max);
    int angle_to_PWM(int current_value, int range_of_motion_degrees, int servo_min, int servo_mid, int servo_max, int target);
    void shift_array (int *arr, int array_length);
    int array_mean(int *arr, int array_length);
    void gimbal_adjust_roll(void);
    void gimbal_adjust_yaw(void);
//####################################################
    void ahrs_update();
    void mount_update(void);
    void update_alt();
    void gcs_failsafe_check(void);
    void compass_accumulate(void);
    void update_compass(void);
    void update_logging1(void);
    void update_logging2(void);
    void update_aux(void);
    void one_second_loop(void);
    void update_GPS_50Hz(void);
    void update_GPS_10Hz(void);
    void update_current_mode(void);
    void update_navigation();
    void send_heartbeat(mavlink_channel_t chan);
    void send_attitude(mavlink_channel_t chan);
    void send_extended_status1(mavlink_channel_t chan);
    void send_location(mavlink_channel_t chan);
    void send_nav_controller_output(mavlink_channel_t chan);
    void send_servo_out(mavlink_channel_t chan);
    void send_radio_out(mavlink_channel_t chan);
    void send_vfr_hud(mavlink_channel_t chan);
    void send_simstate(mavlink_channel_t chan);
    void send_hwstatus(mavlink_channel_t chan);
    void send_pid_tuning(mavlink_channel_t chan);
    void send_rangefinder(mavlink_channel_t chan);
    void send_current_waypoint(mavlink_channel_t chan);
    void send_statustext(mavlink_channel_t chan);
    bool telemetry_delayed(mavlink_channel_t chan);
    void gcs_send_message(enum ap_message id);
    void gcs_send_mission_item_reached_message(uint16_t mission_index);
    void gcs_data_stream_send(void);
    void gcs_update(void);
    void gcs_send_text_P(gcs_severity severity, const prog_char_t *str);
    void gcs_retry_deferred(void);
    void do_erase_logs(void);
    void Log_Write_Performance();
    void Log_Write_Steering();
    void Log_Write_Startup(uint8_t type);
    void Log_Write_Control_Tuning();
    void Log_Write_Nav_Tuning();
    void Log_Write_Sonar();
    void Log_Write_Current();
    void Log_Write_Attitude();
    void Log_Write_RC(void);
    void Log_Write_Baro(void);
    void Log_Write_Home_And_Origin();
    void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page);
    void log_init(void);
    void start_logging() ;
    void load_parameters(void);
    void throttle_slew_limit(int16_t last_throttle);
    bool auto_check_trigger(void);
    bool use_pivot_steering(void);
    void calc_throttle(float target_speed);
    void calc_lateral_acceleration();
    void calc_nav_steer();
    void set_servos(void);
    void set_next_WP(const struct Location& loc);
    void set_guided_WP(void);
    void init_home();
    void restart_nav();
    void exit_mission();
    void do_RTL(void);
    bool verify_RTL();
    bool verify_wait_delay();
    bool verify_within_distance();
    void do_take_picture();
    void log_picture();
    void update_commands(void);
    void delay(uint32_t ms);
    void mavlink_delay(uint32_t ms);
    uint32_t millis();
    uint32_t micros();
    void read_control_switch();
    uint8_t readSwitch(void);
    void reset_control_switch();
    void read_trim_switch();
    void update_events(void);
    void navigate();
    void set_control_channels(void);
    void init_rc_in();
    void init_rc_out();
    void read_radio();
    void control_failsafe(uint16_t pwm);
    void trim_control_surfaces();
    void trim_radio();
    void init_barometer(void);
    void init_sonar(void);
    void read_battery(void);
    void read_receiver_rssi(void);
    void read_sonars(void);
    void report_batt_monitor();
    void report_radio();
    void report_gains();
    void report_throttle();
    void report_compass();
    void report_modes();
    void print_PID(PID * pid);
    void print_radio_values();
    void print_switch(uint8_t p, uint8_t m);
    void print_done();
    void print_blanks(int num);
    void print_divider(void);
    int8_t radio_input_switch(void);
    void zero_eeprom(void);
    void print_enabled(bool b);
    void init_ardupilot();
    void startup_ground(void);
    void set_reverse(bool reverse);
    void set_mode(enum mode mode);
    bool mavlink_set_mode(uint8_t mode);
    void failsafe_trigger(uint8_t failsafe_type, bool on);
    void startup_INS_ground(void);
    void update_notify();
    void resetPerfData(void);
    void check_usb_mux(void);
    uint8_t check_digital_pin(uint8_t pin);
    bool should_log(uint32_t mask);
    void frsky_telemetry_send(void);
    void print_hit_enter();
    void gcs_send_text_fmt(const prog_char_t *fmt, ...);
    void print_mode(AP_HAL::BetterStream *port, uint8_t mode);
    bool start_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd);
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
    void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    void do_loiter_time(const AP_Mission::Mission_Command& cmd);
    bool verify_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    bool verify_loiter_time(const AP_Mission::Mission_Command& cmd);
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    void do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    void do_digicam_configure(const AP_Mission::Mission_Command& cmd);
    void do_digicam_control(const AP_Mission::Mission_Command& cmd);
    void init_capabilities(void);
    bool in_stationary_loiter(void);
    void set_loiter_active(const AP_Mission::Mission_Command& cmd);
    // for APMBoat
    int16_t servoRamp(int16_t currentServoValue, int16_t targetServoValue, int16_t rcChannel);
    void init_servo_pids(void);
    void update_wind(void);
    void update_servos(void);
    void calc_scaled_manual_steering(void);
    bool roll_is_normal(void);
    void react_extreme_roll(void);
    int32_t get_apparent(void);
    int32_t get_apparent_diff(int32_t apparent);
    int32_t get_apparent_rudder(int32_t diff);
    int32_t get_apparent_sail(void);
    int32_t approximate_apparent_boat(int32_t apparent, int32_t sail);
    int32_t approximate_sail(int32_t apparent_boat);
    int32_t get_course_sail(int32_t apparent, int32_t sail);
    int32_t get_course(void);
    int32_t get_course_diff(int32_t apparent);
    int32_t get_course_rudder(void);
    bool rudder_diff_is_big(int32_t diff);
    int32_t limit_value(int32_t value, int32_t limit);


public:
    bool print_log_menu(void);
    int8_t dump_log(uint8_t argc, const Menu::arg *argv);
    int8_t erase_logs(uint8_t argc, const Menu::arg *argv);
    int8_t select_logs(uint8_t argc, const Menu::arg *argv);
    int8_t process_logs(uint8_t argc, const Menu::arg *argv);
    int8_t setup_erase(uint8_t argc, const Menu::arg *argv);
    int8_t setup_mode(uint8_t argc, const Menu::arg *argv);
    int8_t reboot_board(uint8_t, const Menu::arg*);
    int8_t main_menu_help(uint8_t argc, const Menu::arg *argv);
    int8_t test_mode(uint8_t argc, const Menu::arg *argv);
    void run_cli(AP_HAL::UARTDriver *port);
    void mavlink_delay_cb();
    void failsafe_check();
    int8_t test_radio_pwm(uint8_t argc, const Menu::arg *argv);
    int8_t test_passthru(uint8_t argc, const Menu::arg *argv);
    int8_t test_radio(uint8_t argc, const Menu::arg *argv);
    int8_t test_failsafe(uint8_t argc, const Menu::arg *argv);
    int8_t test_relay(uint8_t argc, const Menu::arg *argv);
    int8_t test_wp(uint8_t argc, const Menu::arg *argv);
    void test_wp_print(const AP_Mission::Mission_Command& cmd);
    int8_t test_modeswitch(uint8_t argc, const Menu::arg *argv);
    int8_t test_logging(uint8_t argc, const Menu::arg *argv);
    int8_t test_gps(uint8_t argc, const Menu::arg *argv);
    int8_t test_ins(uint8_t argc, const Menu::arg *argv);
    int8_t test_mag(uint8_t argc, const Menu::arg *argv);
    int8_t test_sonar(uint8_t argc, const Menu::arg *argv);
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    int8_t test_shell(uint8_t argc, const Menu::arg *argv);
#endif
};

#define MENU_FUNC(func) FUNCTOR_BIND(&rover, &Rover::func, int8_t, uint8_t, const Menu::arg *)

extern const AP_HAL::HAL& hal;
extern Rover rover;

#endif // _ROVER_H_
