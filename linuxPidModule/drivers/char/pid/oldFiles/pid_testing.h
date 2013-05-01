//these definitions are primarily to define the primary control structure as well as kernel commands used when using ioctl to communicate with the kernel


#define DEV_IOCTL_MAG_NUM 0xF5
#define DEV_IOCTL_WRITE_CONTROL_STRUCTURE 0x56
#define DEV_IOCTL_READ_CONTROL_STRUCTURE 0x57
#define DEV_IOCTL_WRITE_PID 0x58

static struct pid_structure{
	long alt_adc,az_adc;
	int error;
	int azimuth_zone;
	int interrupt_number;
	unsigned short alt_encoder,az_encoder;
//	unsigned int az_encoder_large;
//	unsigned int az_command_large;
	unsigned short alt_encoder_test[5],az_encoder_test[5];
	unsigned short alt_command,az_command;
	unsigned short prev_alt_encoder,prev_az_encoder;
	unsigned int alt_encoder_long,az_encoder_long,az_command_long,alt_command_long;
	short alt1_dac,alt2_dac,az1_dac,az2_dac;
	char update,updatepos;
	unsigned short cbuffer[8];
	unsigned short mbuffer[8];
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
	unsigned short time;
	unsigned long counter;
	short alt_simulation,az_simulation;
	char encoder_error;
	int encoder_wavelength;
	char interrupt_rate;
	unsigned long time_diff;
	int az_encoder_offset;
	int	ioctl_num;
	struct timeval time_struct;
	unsigned int coordinate_command_type;
	double RA_double,DEC_double,AZ_double,ALT_double;
}control;

