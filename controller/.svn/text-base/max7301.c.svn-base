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
	ioctl(fd,DEV_IOCTL_READ_CONTROL_STRUCTURE, &control); 
//	while(1){
	max7301[0] = 0x0401;
	max7301[1] = 0x0401;
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	//usleep(10000);
	printf("max Returns %04x %04x %04x %04x %04x %04x\n",max7301[0],max7301[1],max7301[2],max7301[3],max7301[4],max7301[5]);
      //set as schmidtt logic input with pullup
	max7301[0] = 0x0955;
	max7301[1] = 0x0955;
	max7301[2] = 2;
	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	//usleep(10000);
	printf("max Returns %04x %04x %04x %04x %04x %04x\n",max7301[0],max7301[1],max7301[2],max7301[3],max7301[4],max7301[5]);
	
	
	max7301[0] = 0x0a55;
	max7301[1] = 0x0a55;
	max7301[2] = 2;
	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	//usleep(10000);
	//printf("max Returns %04x %04x %04x %04x %04x %04x\n",max7301[0],max7301[1],max7301[2],max7301[3],max7301[4],max7301[5]);
	//set port 20-23 as output
	max7301[0] = 0x0bff;
	max7301[1] = 0x0bf5;
	max7301[2] = 2;
 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	//usleep(10000);
	//printf("max Returns %04x %04x %04x %04x %04x %04x\n",max7301[0],max7301[1],max7301[2],max7301[3],max7301[4],max7301[5]);
	//set port 24-27 as output
	max7301[0] = 0x0cf5;
	max7301[1] = 0x0cff;
	max7301[2] = 2;
	

 	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	for(j=0;j<=5;j++){
	  max7301RX[j]=max7301[j];
	}
	printf("max Returns %04x %04x %04x %04x %04x %04x\n",max7301RX[0],max7301RX[1],max7301RX[2],max7301RX[3],max7301RX[4],max7301RX[5]);
	max7301[0] = 0xd000;
	max7301[1] = 0xd000;
	max7301[2] = 2;

// 	printf("Reading from MAX7301\n");
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	for(j=0;j<=5;j++){
	  max7301RX[j]=max7301[j];
	}
	printf("max Returns %04x %04x %04x %04x %04x %04x\n",max7301RX[0],max7301RX[1],max7301RX[2],max7301RX[3],max7301RX[4],max7301RX[5]);
	usleep(100000);
	//printf("Reading from MAX7301\n");
	max7301[0] = 0xd800;
	max7301[1] = 0xd800;
	max7301[2] = 2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	for(j=0;j<=5;j++){
	  max7301RX[j]=max7301[j];
	}
	printf("max Returns %04x %04x %04x %04x %04x %04x\n",max7301RX[0],max7301RX[1],max7301RX[2],max7301RX[3],max7301RX[4],max7301RX[5]);
	max7301[0] = 0x0d55;
	max7301[1] = 0x0d55;
	max7301[2] = 2;
 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	usleep(100000);
	//printf("max Returns %04x %04x %04x %04x %04x %04x\n",max7301[0],max7301[1],max7301[2],max7301[3],max7301[4],max7301[5]);
	
// 	//max7301[0]=0x20;
//  // 	command = 0x56;	
// //  	value =0x00;
// 	//set port 25 high
// 	max7301[0]=0x39ff;
//  	max7301[1] = 0x39ff;
//  	max7301[2] = 2;
//  	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
// 	//set port 24 high
// 	max7301[0]=0x38ff;
//  	max7301[1] = 0x38ff;
//  	max7301[2] = 2;
//  	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	//set pin 20-27 logic high
	max7301[0] = 0x4cff;
	max7301[1] = 0x58ff;
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	//printf("max Returns %04x %04x %04x %04x %04x %04x\n",max7301[0],max7301[1],max7301[2],max7301[3],max7301[4],max7301[5]);
	max7301[0] = 0x4fff;
	max7301[1] = 0x58ff;
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	//printf("max Returns %04x %04x %04x %04x %04x %04x\n",max7301[0],max7301[1],max7301[2],max7301[3],max7301[4],max7301[5]);
	//bring chips out of shutdown mode
	
//	usleep(1000000);
//	}
// 	usleep(1000);
 	value = 0xff;
/*
	while(1){
// 	max7301[0] = 0x0401;
// 	max7301[1] = 0x0401;
// 	max7301[2]=2;
// 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
// 	max7301[0] = 0x0000|value;
// 	max7301[1] = 0x3700|value;
// 	max7301[2]=2;
// 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	if(value ==0xff){
		value = 0x00;
	}
	else if(value==0x00){
		value=0xff;
	}

	max7301[0] = 0x5400|value;
	max7301[1] = 0x5400|value;
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	usleep(10000);
	}*/
//  	
// // 	

	close(fd);

}
