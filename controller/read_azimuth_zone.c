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
#include <sys/ioctl.h>
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


unsigned int get_crc(unsigned int);
unsigned int ad5362_crc_pack(unsigned int,unsigned int,unsigned int);
struct pidvals{
	double p,i,d,err;
}pidloop;

struct pid_structure update;




int main(int argc, char *argv[]){
	int fd,i,read_ret;
	int az_zone;
	/* check command line args */
	if(argc<1) {
	printf("usage : %s <server> <data1> ... <dataN> AZIMUTH_ZONE\n", argv[0]);
	exit(1);
	}


	fd = open("/dev/pid",O_RDWR);
	if(fd ==-1){
	perror("failed to open PID Control Module\n");
	//rc = rd;
	exit(-1);
	}

	
	ioctl(fd,DEV_IOCTL_READ_AZIMUTH_ZONE, &az_zone);    
	printf("\n\n---------------TELESCOPE AZIMUTH--------------\n\nZONE %i\n",az_zone);

	printf("\n\n\n--------------FURTHER INFORMATION------------\n\n\nThe control kernel module thinks that the telescope is in ZONE %i.\n\n If the module has just been inserted into the kernel it will start with a value of 100.\n\n The ZONE can be changed by using the program \n\n ./azimuth_zone [ZONE VAL]\n\n e.g ./azimuth_zone 1\n This will set the zone to ZONE 1\n\n\n Valid Zone Values are 0, 1, 2. \n\n------------------ZONE INFORMATION-----------------\n\n ZONE 0 :At minimum azimuth position \n ZONE 1: Park position \n ZONE 2: Maximum azimuth position\n\n\n----------------------------\n\n\n Once the zone information has been uploaded the controller will keep track of its location. This should only need to be done once. If the telescope was parked when the controller was rebooted then the appropriate Zone is 1\n ",az_zone);
	
	ioctl(fd,DEV_IOCTL_READ_CONTROL_STRUCTURE,&control);
	update = control;

	printf("Encoder %d %d\n",update.az_encoder_long,update.alt_encoder_long);
	printf("Encoder %d %d\n",update.az_encoder_long,update.alt_encoder_long);
	close(fd);

}
