//compile with arm-unknown-linux-gnu-gcc antenna_status.c /usr/lib/libarmcsla.a -o antenna_status -lpthread -lm



#include <stdio.h> 
#include <stdlib.h> 
#include <sys/types.h> 
#include <sys/stat.h> 
#include <sys/time.h>
#include <fcntl.h> 
#include <unistd.h>
#include <time.h>
#include <tgmath.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string.h> /* memset() */
#include <sys/time.h> /* select() */
#include <pthread.h>
//this defines the structure used to copy data between kernel and userspace- check that this file is the same in both ALWAYS!!
#include "pid.h"
#include "slalib.h"
#include "ad5362_def.h"
#include "crc_gen.h"


#define REMOTE_SERVER_PORT 1500
#define ALT_LIM_HI 40500
#define ALT_LIM_LO 26000
#define ALT_LIM_SPREAD 2000
#define LIMIT_ZONE_ADD 1000

//#define MOTOR_SLACK 650
#define MAX_I 50000
#define MAX_SPEED 10000
#define ALT_PID 0
#define AZ_PID 1
#define ALT_PID1 0
#define ALT_PID2 1
#define AZ_PID1 2
#define AZ_PID2 3
#define LOCAL_SERVER_PORT 1501
#define MAX_MSG 2000
#define PI 3.141592653589793238462643
#define R2D (180.0/PI) /* radians to degrees */
#define BUFSIZE 100

int fd;
struct pid_structure control;
float *allocate(size_t length);
int MOTOR_SLACK = 2000;
int LIMIT_ZONE_SPEED = 650;
static char buffer[30];
static char cnt[200];
static char udpcat[1000000];
static char *udpcatptr;
static char udpbuf[1000000];
float encoder[10],*encoder_ptr,omega[5],domega[5],d2omega[5];
int order[10];
float La,Il,kt,La,B,Ra,Kb;

int update_pid(char);
int update_pid1(char);
int alt_soft_lim(void);
unsigned int get_crc(unsigned int);
unsigned int ad5362_crc_pack(unsigned int,unsigned int,unsigned int);
struct pidvals{
	double p,i,d,err;
}pidloop;

struct pid_structure update;

void Get_angles(time_t time,double right_ascension_mean,double declination_mean,double Epoch,double *azimuth, double *altitude);
unsigned int testmax7301_high(void);
unsigned int testmax7301_low(void);
unsigned int max7301_init(void);


int main(int argc, char *argv[]){
	
	//int fd;
	int i;
	int unsigned long time_diff;
	int read_ret;
	int test_dac;
	int err_count_alt,err_count_az;
	unsigned short DAC_VAL=-33000;
	long DAC_VAL2 = -33000;
	signed short DAC_VAL3=-10000;
	unsigned short value1;
	int VAL[5];
	signed short t1,t2,t3,t4,t5;
	unsigned int ad5362[10];

	fd = open("/dev/pid",O_RDWR);
	if(fd ==-1){
	perror("failed to open PID Control Module\n");
	//rc = rd;
	exit(-1);
	}
	
	DAC_VAL = ad5362_crc_pack(XREGISTER_WRITE,ALL,-10000);	
	printf("DAC_VAL %ld %lx\n",DAC_VAL);
	ioctl(fd,DEV_IOCTL_DISABLE_DAC, &control); 
	DAC_VAL2=-33000;
	ioctl(fd,DEV_IOCTL_ENABLE_DAC, &control); 
	while(DAC_VAL2<33000){
	
	read_ret = read(fd,&control,sizeof(control));
	DAC_VAL2+=500;
	DAC_VAL3+=200;
	control.az_pid1 = DAC_VAL2;
	control.az1_dac_control = ad5362_crc_pack(XREGISTER_READ,CH5,control.az_pid1);
	control.az2_dac_control = ad5362_crc_pack(XREGISTER_READ,CH6,control.az_pid1);
	control.alt1_dac_control = ad5362_crc_pack(XREGISTER_READ,CH7,control.az_pid1);
	control.alt2_dac_control =ad5362_crc_pack(XREGISTER_READ,CH8,control.az_pid1);
	read_ret = write(fd,&control,sizeof(control));
	//sleep(1);
	//printf("%ld %d %d %d %d\n",control.az_pid1,control.tacho1<<1,control.tacho2<<2,control.tacho3,control.tacho4);
	
	
	printf("%7ld %04d %04d %04d %04d %04d %04d\n",control.az_pid1,control.tacho1,control.tacho2,control.tacho3,control.tacho4);
	
	//printf("%ld %04x %04x %04x %04x\n",control.az_pid1,control.tacho3,VAL[1],VAL[2],((VAL[0]<<18)&0x000001fff));
	//printf("CONTROL_STATUS,%6u,%6u,%2i,%6i,%6d,%6d,%6d,%6d,%6i,%6i,%6i,%6i,%6i,%6i,%6i,%6i,%6i,%6i,%6i,%6i,%6i,%7ld,%7ld,%5u,%5ld end\n",control.az_encoder_long,control.alt_encoder_long,control.azimuth_zone,test_dac,control.tacho1,control.tacho2,control.tacho3,control.tacho4,control.az_pid1,control.az_pid2,control.alt_pid1,con4xtrol.alt_pid2,control.alt1_dac_control,control.alt2_dac_control,control.encoder_error,err_count_alt,err_count_az,control.az_command_long,control.alt_command_long,control.time_struct.tv_sec,control.time_struct.tv_usec,control.time,control.time_struct.tv_usec-time_diff);
	//sleep(1);
	
	}
	printf("-----------------------------\nEND\n---------------------------\n");
	value1 = -32000;
	ioctl(fd,DEV_IOCTL_ENABLE_DAC, &control); 
	value1=000;
	ioctl(fd,DEV_IOCTL_READ_AD5362,ad5362);
	while(1){
	//	ioctl(fd,DEV_IOCTL_READ_AD5362,ad5362);

		read_ret = read(fd,&control,sizeof(control));
		usleep(100);	
		//value1+=100;
		control.az_pid1= value1;
		control.az_pid2 = value1-500;
		control.alt_pid1 = value1-200;
		control.alt_pid2 = value1+400;
		control.az1_dac_control = ad5362_crc_pack(XREGISTER_WRITE,CH5,control.az_pid1);
		control.az2_dac_control = ad5362_crc_pack(XREGISTER_WRITE,CH6,control.az_pid2);
		control.alt1_dac_control = ad5362_crc_pack(XREGISTER_WRITE,CH7,control.alt_pid1);
		 control.alt2_dac_control =ad5362_crc_pack(XREGISTER_WRITE,CH8,control.alt_pid2);
		 if(value1>20000){
		 // value1 =-10000;
		}
		read_ret = write(fd,&control,sizeof(control));
		printf("%7ld %04d %04d %04d %04d %04d %04d\n",control.az_pid1,control.tacho1,control.tacho2,control.tacho3,control.tacho4);
		usleep(10000);	
		
	}
	control.az_pid1 = -30000;
	control.az1_dac_control = ad5362_crc_pack(XREGISTER_READ,CH5,control.az_pid1);
	control.az2_dac_control = ad5362_crc_pack(XREGISTER_READ,CH6,control.az_pid1);
	control.alt1_dac_control = ad5362_crc_pack(XREGISTER_READ,CH7,control.az_pid1);
	control.alt2_dac_control =ad5362_crc_pack(XREGISTER_READ,CH8,control.az_pid1);
	read_ret = write(fd,&control,sizeof(control));	
	
// 	control.az1_dac_control = ad5362_crc_pack(XREGISTER_WRITE,CH5,0);
// 	control.az2_dac_control = ad5362_crc_pack(XREGISTER_WRITE,CH6,0);
// 	control.alt1_dac_control = ad5362_crc_pack(XREGISTER_WRITE,CH7,0);
// 	control.alt2_dac_control =ad5362_crc_pack(XREGISTER_WRITE,CH8,0);
// 	printf("\n%d %d %d %d %08x %08x %08x %08x\n",control.az_pid1,control.az_pid2,control.alt_pid1,control.alt_pid2,control.az1_dac_control,control.az2_dac_control,control.alt1_dac_control,control.alt2_dac_control);

	printf("Testing Max7301 Digital I/O\n");
	max7301_init();
	while(1){
	printf("MAX7301 all ports LO!\n");
	testmax7301_low();
	sleep(1);
	 printf("MAX7301 all ports HI!\n");
	testmax7301_high();
	sleep(1);
	}

// 		DAC_VAL2 = 0x0000;
// 	while(1){
// 	DAC_VAL2+=100;
// 	read_ret = read(fd,&control,sizeof(control));
// 
// 	for(i=0;i<=7;i++){
// 	control.cbuffer[i] = ad5362_crc_pack(CREGISTER_WRITE,ALL,DAC_VAL2);
// 	control.mbuffer[i] = ad5362_crc_pack(MREGISTER_WRITE,ALL,DAC_VAL2);
// 	}
// 	ioctl(fd,DEV_IOCTL_SET_DAC_REGISTERS, &control); 
// 	printf("%7ld %04d %04d %04d %04d  %08x %08x %08x %08x\n",control.az_pid1,control.tacho1,control.tacho2,control.tacho3,control.tacho4,control.az1_dac_control,control.az2_dac_control,control.alt1_dac_control,control.alt2_dac_control);
// 	sleep(1);
// 	}


	close(fd);

}
unsigned int max7301_init(void){
	unsigned int max7301[5];
	unsigned int max7301RX[5];	
	max7301[0] = 0x0401;
	max7301[1] = 0x0401;
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);

	max7301[0] = 0x0955;
	max7301[1] = 0x0955;
	max7301[2] = 2;
	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	max7301[0] = 0x0a55;
	max7301[1] = 0x0a55;
	max7301[2] = 2;
	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);

	//set port 20-23 as output
	max7301[0] = 0x0b55;
	max7301[1] = 0x0b55;
	max7301[2] = 2;
 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);

	//set port 24-27 as output
	max7301[0] = 0x0c55;
	max7301[1] = 0x0c55;
	max7301[2] = 2;
 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	max7301[0] = 0x0d55;
	max7301[1] = 0x0d55;
	max7301[2] = 2;
 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	max7301[0] = 0x0e55;
	max7301[1] = 0x0e55;
	max7301[2] = 2;
 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	max7301[0] = 0x0f55;
	max7301[1] = 0x0f55;
	max7301[2] = 2;
 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	
}

unsigned int testmax7301_high(void){
	unsigned int max7301[5];
	unsigned int max7301RX[5];	
		//printf("TEST I/O\n");
		max7301[0] = 0x4cff;
		max7301[1] = 0x4cff;
		max7301[2]=2;
		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
		max7301[0] = 0x54ff;
		max7301[1] = 0x54ff;
		max7301[2]=2;
		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
// 		max7301[0] = 0x53ff;
// 		max7301[1] = 0x54ff;
// 		max7301[2]=2;
// 		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
 		max7301[0] = 0x5cff;
 		max7301[1] = 0x5cff;
 		max7301[2]=2;
 		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	

	
}

unsigned int testmax7301_low(void){
	unsigned int max7301[5];
	unsigned int max7301RX[5];	
		//printf("TEST I/O\n");
		max7301[0] = 0x4c00;
		max7301[1] = 0x4c00;
		max7301[2]=2;
		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
		max7301[0] = 0x5400;
		max7301[1] = 0x5400;
		max7301[2]=2;
		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
// 		max7301[0] = 0x5400;
// 		max7301[1] = 0x5400;
// 		max7301[2]=2;
// 		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
 		max7301[0] = 0x5c00;
 		max7301[1] = 0x5c00;
		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
// 		max7301[2]=2;
// 		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	

	
}

unsigned int ad5362_crc_pack(unsigned int command,unsigned int channel,unsigned int value){
	//this function takes inputs and prepares a suitable packet to be transmitted to the ad5362 DAC. Includeds a CRC for safety so will produce a four byte value stored in the unsigned int.
	unsigned char test_vec[10];
	unsigned int retval;
	unsigned int packet;
	 crc_t crc;
	packet = (command<<20)|(channel<<16)|(((unsigned short)value+32767)&(0x00ffff));
	test_vec[0]=(packet&0xff000000)>>24;
	test_vec[1]=(packet&0x00ff0000)>>16;
	test_vec[2]=(packet&0x0000ff00)>>8;
	test_vec[3]=(packet&0x000000ff);
	crc = crc_init();
   	crc = crc_update(crc, test_vec, 4);
	crc = crc_finalize(crc);
	retval = (packet<<8)|(crc&(0x000000ff));
	return retval;
}

unsigned int ad5362_crc_pack1(unsigned int command,unsigned int channel,unsigned int value){
	//this function takes inputs and prepares a suitable packet to be transmitted to the ad5362 DAC. Includeds a CRC for safety so will produce a four byte value stored in the unsigned int.
	unsigned char test_vec[10];
	unsigned int retval;
	unsigned int packet;
	 crc_t crc;
	packet = (command<<20)|(channel<<16)|(((unsigned short)value)&(0x00ffff));
	test_vec[0]=(packet&0xff000000)>>24;
	test_vec[1]=(packet&0x00ff0000)>>16;
	test_vec[2]=(packet&0x0000ff00)>>8;
	test_vec[3]=(packet&0x000000ff);
	crc = crc_init();
   	crc = crc_update(crc, test_vec, 4);
	crc = crc_finalize(crc);
	retval = (packet<<8)|(crc&(0x000000ff));
	return retval;
}