//these definitions are primarily to define the primary control structure as well as kernel commands used when using ioctl to communicate with the kernel

//ioctl commands!
#define DEV_IOCTL_MAG_NUM 0xF5
#define DEV_IOCTL_WRITE_CONTROL_STRUCTURE 900
#define DEV_IOCTL_READ_CONTROL_STRUCTURE 901
#define DEV_IOCTL_WRITE_PID 902
#define DEV_IOCTL_WAIT 903
#define DEV_IOCTL_ENABLE_DAC 904
#define DEV_IOCTL_DISABLE_DAC 905
#define DEV_IOCTL_SET_DAC_REGISTERS 906
#define DEV_IOCTL_WRITE_AZIMUTH_ZONE 907
#define DEV_IOCTL_READ_AZIMUTH_ZONE 908
#define DEV_IOCTL_WRITE_MAX7301 909
#define DEV_IOCTL_READ_MAX7301 910
#define DEV_IOCTL_WRITE_AD5362 911
#define DEV_IOCTL_READ_AD5362 912
#define DEV_IOCTL_READ_AD7367 913
#define DEV_IOCTL_READ_ENCODER 914


// #define HORIZONTAL 100
// #define EQUATORIAL 101
// #define ENCODER_COORDS 102
// #define AZ1_PID_COEFFICIENTS 103
// #define AZ2_PID_COEFFICIENTS 104
// #define ALT1_PID_COEFFICIENTS 105
// #define ALT2_PID_COEFFICIENTS 106
// #define IOCTL_COMMANDS 107
// #define SAFE_DRIVE_LIMITS 108
// #define EQUATORIAL2 109	
// #define CONTACTORS 110
// #define CLUTCHBRAKE 111
// #define EQUATORIAL2_WITH_POINTING 112	
// #define AZ1_PID_COEFFICIENTS_ADAPTIVE 113
// #define AZ2_PID_COEFFICIENTS_ADAPTIVE 114
// #define ALT1_PID_COEFFICIENTS_ADAPTIVE 115
// #define ALT2_PID_COEFFICIENTS_ADAPTIVE 116
// #define POLYNOMIAL_POINT 117
// #define HORIZONTAL_LIST 118



#define LONG_TERM_KERNEL_MONITOR_SECONDS 300
#define SLOW_INT 63934


struct pid_coefficient_structure{
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

struct message_parsing_struct{
    int coordinate_type; //i.e identifier attached to the message
    char commands[100][100]; //the actual command strings if necessary
    int command_vals[100];
    float fcommand_vals[100];
    double dcommand_vals[100];
    long lcommand_vals[100];
    struct pid_coefficient_structure pid_vals;
};

struct pid_structure{
	long alt_adc,az_adc;
	int error;
	int read_status;
	int azimuth_zone;
	int interrupt_number;
	unsigned short alt_encoder,az_encoder;
//	unsigned int az_encoder_large;
//	unsigned int az_command_large;
	unsigned short alt_encoder_test[5],az_encoder_test[5];
	unsigned short alt_command,az_command;
	unsigned short prev_alt_encoder,prev_az_encoder;
	unsigned int alt_encoder_long,az_encoder_long;
	unsigned int az_command_long,alt_command_long;
	short alt1_dac,alt2_dac,az1_dac,az2_dac;
	volatile char update,updatepos;
	long cbuffer[8];
	long mbuffer[8];
	unsigned int cbuffer_crc[8];
	unsigned int mbuffer_crc[8];
	unsigned int cbuffer_crc_read[8];
	unsigned int mbuffer_crc_read[8];
	unsigned int cbuffer_read_ret[8];
	unsigned int mbuffer_read_ret[8];
	unsigned int alt1_dac_control,alt2_dac_control,az1_dac_control,az2_dac_control;
	long alt_err,az_err;
	long alt_err1,alt_err2,az_err1,az_err2;
	long alt_p,az_p;
	long alt_i,az_i;
	long alt_i1,alt_i2,az_i1,az_i2;
	long alt_d,az_d;
	long alt_err_prev,az_err_prev;
	long alt_pid,az_pid;
	long alt_pid1,alt_pid2,az_pid1,az_pid2;
	int tacho1,tacho2,tacho3,tacho4;
	double az_p1,az_p2,az_ic1,az_ic2,az_d1,az_d2;
	double az_p1_vel,az_p2_vel,az_ic1_vel,az_ic2_vel,az_d1_vel,az_d2_vel;
	double alt_p1,alt_p2,alt_ic1,alt_ic2,alt_d1,alt_d2;
	double alt_p1_vel,alt_p2_vel,alt_ic1_vel,alt_ic2_vel,alt_d1_vel,alt_d2_vel;
	double pcoeffs[4],icoeffs[4],dcoeffs[4],kfcoeffs[4],vfcoeffs[4];
	double pcoeffs_vel[4],icoeffs_vel[4],dcoeffs_vel[4];
	long motor_plus[4],motor_minus[4];
	long limits[4];
	unsigned short time;
	unsigned long counter,counter2;
	short alt_simulation,az_simulation;
	char encoder_error;
	int encoder_wavelength;
	char interrupt_rate;
	unsigned long time_diff;
	int az_encoder_offset;
	int	ioctl_num;
	struct timeval time_struct;
	volatile unsigned int coordinate_command_type;
	volatile double RA_double,DEC_double,AZ_double,ALT_double;
	long motor_az1_plus,motor_az1_minus;
	long motor_az2_plus,motor_az2_minus;
	long motor_alt1_plus,motor_alt1_minus;
	long motor_alt2_plus,motor_alt2_minus;
	char DAC_Output;
	double a[10],b[10];
	time_t eq2_time_end,eq2_time_begin;
	int eq2_length;
	volatile unsigned long counter3;
	double azimuth_encoder_double,altitude_encoder_double;
	double azimuth_command_double,altitude_command_double;
	double delta_az,delta_alt;
	double vel_of_az,vel_of_alt;
	unsigned int status_vec[20];
	struct timeval ppsTime;
};

