//This file primarily defines local angle encoder (in 16 bit number) values to provide some direction and software limits for the antenna- these will be define differently on different antennas.

//of primary importance are the locations of the azimuth and elevation limit switches. Originally a single software limit was introduced to prevent command angles greater than certain values from being passed. This did not prevent the antenna momentum carrying the antenna into limits. 

//The definition of a 'Go slow zone' near the limits was introduced in an effort to counteract this momentum problem, while still allowing the antenna to operate near the limit zones. The actual max (min) speed in these zones is defined int the soft_lim() function. This was mainly introduced to avoid the need to throw motors into reverse with the possibility that this may damage the gears.

//I also use this file to define the common structures used to contain the TCP values moved between PC and ARM- these are defined at the bottom
#define ALT_PID 0
#define AZ_PID 1

//the following two definitions should be adjusted if the angle encoder is replaced- these are very important as the define the rough zero point of the encoders used to translate from encoder to telescope azimuth and elevation
//before 17/9/2015 Klerefontein#define AZIMUTH_ZERO 78243 //angle encoder position at due north (approximate)
//changed to 111010 on 17/9/2015 //angle encoder position at due north (approximate)
#define AZIMUTH_ZERO 114563 //angle encoder position at due north (approximate)
//#define ELEVATION_ZERO 30416 //angle encoder position at horizon (approximate) [should be calculated from the 'relatively well defined' stow lock position by subtracting 16384- it should be ensured that the encoder is physically positioned so that there is not possibility of an encoder overflow i.e 65535->0. so elevation zero >0 and elevation hi >16384 < 65535]
#define ELEVATION_ZERO 30709 //angle encoder position at horizon (approximate) [should be calculated from the 'relatively well defined' stow lock position by subtracting 16384- it should be ensured that the encoder is physically positioned so that there is not possibility of an encoder overflow i.e 65535->0. so elevation zero >0 and elevation hi >16384 < 65535]





//NOTE WELL:approximate positions of limits at the hartrao setup (this will be different depending on the orientation)
//lo azimuth limit ~AZ Zero -75000
//hi azimuth limit ~ AZ_Zero + 24000
//HI elevation limit = lo_elevation limit /=16384 (i.e 90 degrees)


//NOTE THE FOLLOWING ARE CHANGED AFTER AN ENCODER IS SWOPPED OUT. THESE DEFINE SAFETY ANGLES USED IN THE USER SPACE SOFTWARE AS A SOFTWARE LIMIT CHECKER. PLEASE BE CAREFUL WITH THESE.
//Note 8 September 2009: These are not currently implemented per se. The are defined on the startup of the antenna, however there is provision to change the software limits from user
//space. This should really be rectified to prevent driving into limits. 
//Note 8 September 2009: The program DOES use the Constants NAME_SAFETY_LO. NAME_SAFETY_HI in the soft limit checking and these cannot be changed by Userspace! However they do not currently
//Provide an absolute limit and only really limit the speed to a slower speed than in other regions
//1. these are for the kernel limit checking and also define the maximum limit values used in check_limit_values() which in turn provides a safety check for control.limits[i] used in the soft_lim() algorithm. This is the final software limit check. USE ELEVATION_LIMIT_HI AS THE POSITION AT STOW LOCK THEN ELEVATION_LIMIT_LO = ELEVATION_LIMIT_HI-16384 (090DEGREES)
#define AZIMUTH_LIMIT_LO 30000 
#define AZIMUTH_LIMIT_HI 106000
#define ELEVATION_LIMIT_LO 30600
#define ELEVATION_LIMIT_HI 46700
//2. these are for the user space limit checking- i.e a second order safety mechanism to slow the antenna velocity command (i.e pid output) to some predetermined speed when inside some encoder angle ranges.soft_lim()
#define AZIMUTH_SLOW_ZONE_HI 135000
#define AZIMUTH_SLOW_ZONE_LO 32000
#define ELEVATION_SLOW_ZONE_HI 46500
#define ELEVATION_SLOW_ZONE_LO 30900
//3. these define the initial maximum drivable encoder positions of the user space program (initialised in init_control_struct())). These can be adjusted dynamically while the program is running after this.
// They also are permanently (i.e changing them requires recompiling the program)  used for the slow zone regions
//so the soft_limit() routine will use these values to define 'dangerous' zones i.e near to the limits and cap the antenna speed in these zones. These need to be changed after changing the encoder.
#define AZIMUTH_SAFETY_LO 28500 
#define AZIMUTH_SAFETY_HI 135000 //current hard limit is at 141000
#define ELEVATION_SAFETY_LO 30900 
#define ELEVATION_SAFETY_HI 46500

//these define the maximum speeds in both directions allowed when in the safety zone defined by AZIMUTH_SLOW_ZONE_HI (see 2.) etc. These should only apply when the controller tried to drive 'deeper' into the zone. Driving out of the zone should not effected by the routine- 28 March this has not been implemented in the soft_lim() function yet. Should be trivial
#define AZ_SLOW_SPEED_POSITIVE 1000
#define AZ_SLOW_SPEED_NEGATIVE -1000
#define EL_SLOW_SPEED_POSITIVE 1000
#define EL_SLOW_SPEED_NEGATIVE -1000





#define MAX_TACHO_ALLOWABLE 9000
#define MIN_TACHO_ALLOWABLE -9000
//this section defines constants used when excepting commands from the network- it also defines ports that are used
#define LOCAL_SERVER_PORT_OVRO 1503
#define LOCAL_SERVER_PORT_STRUCT 1502
#define LOCAL_SERVER_PORT 1501
#define REMOTE_SERVER_PORT 1500

//The following are not currently used
#define AZIMUTH_START 101000 //default start command position for the Azimuth axis
//these are for the user space limit checking
#define ELEVATION_START 40000 //default start command position for the ELEVATION axis
//these define the initial maximum drivable encoder positions of the user space program (initialised in init_control_struct())) These are used in soft_lim() routine and can be changed through a suitable TCP command 


#define MAX_AZ_POS_PID 8000
#define MIN_AZ_POS_PID -8000
#define MAX_ALT_POS_PID 7000
#define MIN_ALT_POS_PID -7000
//absolute maximum motor speed output after pid- in arbitrary units just fiddle with this till more or less right
#define MAX_AZ_SPEED 12000
#define MIN_AZ_SPEED -12000
#define MAX_ALT_SPEED 5000
#define MIN_ALT_SPEED -5000

//these will define the maximum space between successive control positions (in deg/s)
#define MAX_AZ_POS_SPACE_DEG 5.0
#define MIN_AZ_POS_SPACE_DEG -5.0
#define MAX_ALT_POS_SPACE_DEG 1
#define MIN_ALT_POS_SPACE_DEG -1
//these define the maximum permitted azimuth accelerations that can be commanded to the antenna using the 117 enum (i.e the control that should be used for all user type control- defined as EQUATORAIL2 for historical reasons )-these should be values in mdeg/s/s i.e 1000 means a maximum acceleration of 1000mdeg/s/s or 1deg/s/s- so for a typical cross scan going between 1deg/s and -1 deg/s we would want a turnaround of say 5 seconds giving a acceleration of 2deg/s / 5seconds =0.4 deg/s/s so a value of 400 is appropriate
#define MAX_AZ_ACCEL 80
#define MIN_AZ_ACCEL -80
#define MAX_ALT_ACCEL 20
#define MIN_ALT_ACCEL -20
//maximum ramp speed (this controls the antenna acceleration)
#define MAX_AZ_RAMP 30
#define MAX_ALT_RAMP 30
//floating point encoder position limits
#define MAX_AZ_VAL_FLOAT 140 //change these when the encoders are changed
#define MIN_AZ_VAL_FLOAT -280 //change these when the encoders are changed.
#define AZWRAPSWITCH -80.0
#define MAX_ALT_VAL_FLOAT 87
#define MIN_ALT_VAL_FLOAT 5

#define HORIZONTAL 100
#define EQUATORIAL 101
#define ENCODER_COORDS 102
#define AZ1_PID_COEFFICIENTS 103
#define AZ2_PID_COEFFICIENTS 104
#define ALT1_PID_COEFFICIENTS 105
#define ALT2_PID_COEFFICIENTS 106
#define IOCTL_COMMANDS 107
#define SAFE_DRIVE_LIMITS 108
#define EQUATORIAL2 109	
#define CONTACTORS 110
#define CLUTCHBRAKE 111
#define EQUATORIAL2_WITH_POINTING 112	
#define AZ1_PID_COEFFICIENTS_ADAPTIVE 113
#define AZ2_PID_COEFFICIENTS_ADAPTIVE 114
#define ALT1_PID_COEFFICIENTS_ADAPTIVE 115
#define ALT2_PID_COEFFICIENTS_ADAPTIVE 116
#define POLYNOMIAL_POINT 117
#define HORIZONTAL_LIST 118
#define STS_VEC_SIZE 5 //size of the status vector read from the max7301


struct new_pid_coefficient_structure{
//this structure (implemented 6 August 2009) allows adaptive PID control of the telescope. i.e different PID coefficients are used at different control conditions (defined by the command_angle-actual_position_angle). This allows different PID coefficients to be used when near the desired angle to when the telescope is slewing.
    //store the total position error for the adaptive (fuzzy logic) PID 
    int position_error[20];
    //store the P coefficients corresponding
    float p[20];
    //store the I coefficients corresponding
    float i[20];
    //store the D coefficients corresponding
    float d[20];
    float kf[20];
    float vf[20];
    float p_2[20];
    float i_2[20];
    float d_2[20];
    //store the motor offset for positive direction
    long motor_plus[20];
    //store the motor offset for negative direction
    long motor_minus[20];
    long hello[20];
    char adaptive;
    char table_length;
    char table_position;

};

struct new_message_parsing_struct{
    int coordinate_type; //i.e identifier attached to the message
    char commands[100][100]; //the actual command strings if necessary
    int command_vals[100];
    float fcommand_vals[100];
    double dcommand_vals[100];
    long lcommand_vals[100];
    struct new_pid_coefficient_structure pid_vals;
    float az_commands[40],alt_commands[40];
    int message_size;
}; 

//this defined the structures used to contain data loaded in the 1 command/second grid type coordinate commandupdatez

struct command_struct{
    long lcomvals[100];
    float comaz[100];
    float comalt[100];
    long eq2_time_end;
    long eq2_time_begin;
};

struct readout_struct{
    float az_position[10];
    float alt_position[10];
    float az_pos_err[10];
    float alt_pos_err[10];
    long int time[10];
    long int timeuSeconds[10];
    long azimuth_time_table[10];
    float azimuth_position_table[10];
    float altitude_position_table[10];
    float az_pos_err_table[10];
    float alt_pos_err_table[10];
    int az_tacho1_table[10];
    int az_tacho2_table[10];
    int alt_tacho1_table[10];
    int alt_tacho2_table[10];
    int az_pid1_table[10];
    int az_pid2_table[10];
    int el_pid1_table[10];
    int el_pid2_table[10];
    double calc_time[10];
    long sample_rate;
    long sample_number;
    long current_value;
    volatile float instantAzErr,instantAltErr;
    volatile float instantCommandAz,instantCommandAlt;
    volatile int azZone;	
    volatile float az_ready_to_read[5];
    volatile float alt_ready_to_read[5];
    volatile float az_err_ready_to_read[5];
    volatile float alt_err_ready_to_read[5];
    volatile int az_tacho1_ready_to_read[5];
    volatile int az_tacho2_ready_to_read[5];
    volatile int alt_tacho1_ready_to_read[5];
    volatile int alt_tacho2_ready_to_read[5];
    volatile int az_pid1_ready_to_read[5];
    volatile int az_pid2_ready_to_read[5];
    volatile int el_pid1_ready_to_read[5];
    volatile int el_pid2_ready_to_read[5];
    volatile int time_ready_to_read[5];
    volatile int timeuSeconds_ready_to_read[5];
    int volatile ready;
    long samples_per_second;
    int softLimitStatus;	
    struct timeval ppsTime;
    };
