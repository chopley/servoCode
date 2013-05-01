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


struct pid_structure control;



int main(int argc, char *argv[]){
	
	int fd;
	unsigned int max7301[5];
	unsigned int max7301RX[5];	
	unsigned int command;
	unsigned char value;
	struct pid_structure user;
	int read_ret;
	int i=0;
	int j;

	fd = open("/dev/pid",O_RDWR);
	if(fd ==-1){
	perror("failed to open PID Control Module\n");
	//rc = rd;
	exit(-1);
	}
	printf("MAX7301 Hello hhhhhhh\n");
	//read in the module data from Kernel space when you start
	//set the 'virtual ports' to output as detailed in datasheet
	while(1){
	//sleep(1);
	max7301[0] = 0xd000;
	max7301[1] = 0xd200;
	max7301[2] = 2;

// 	printf("Reading from MAX7301\n");
	// read_ret = read(fd,&user,sizeof(user));
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	for(j=0;j<=5;j++){
	  max7301RX[j]=max7301[j];
	}
	printf("max Returns %04x %04x %04x %04x %04x %04x\n",max7301RX[0],max7301RX[1],max7301RX[2],max7301RX[3],max7301RX[4],max7301RX[5]);
	usleep(1000);
	//printf("Reading from MAX7301\n");
	/*max7301[0] = 0xd800;
	max7301[1] = 0xd800;
	max7301[2] = 2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	for(j=0;j<=5;j++){
	  max7301RX[j]=max7301[j];
	}
	printf("max Returns %04x %04x %04x %04x %04x %04x\n",max7301RX[0],max7301RX[1],max7301RX[2],max7301RX[3],max7301RX[4],max7301RX[5]);*/
	}

	close(fd);

}
