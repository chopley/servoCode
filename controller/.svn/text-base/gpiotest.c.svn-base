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

unsigned int max7301_init(void);
unsigned int contactors(unsigned int command);
unsigned int clutchbrake(unsigned int command);

struct pid_structure control;
int fd;


int main(int argc, char *argv[]){
	
	
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

	max7301_init();
	contactors(0);
	clutchbrake(0);
	sleep(2);
	clutchbrake(0);
	sleep(2);
	clutchbrake(1);
	sleep(2);
	clutchbrake(1);
	sleep(2);
	clutchbrake(0);
 //	contactors(0);
// 	sleep(2);
// 	contactors(0);
// 	sleep(2);
// 	contactors(1);
// 	sleep(2);
// 	contactors(1);
// 	sleep(2);
// 	contactors(0);
// 	close(fd);

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
	max7301[1] = 0x0bff;
	max7301[2] = 2;
 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);

	//set port 24-27 as output
	max7301[0] = 0x0c55;
	max7301[1] = 0x0cff;
	max7301[2] = 2;
 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	max7301[0] = 0x0d55;
	max7301[1] = 0x0dff;
	max7301[2] = 2;
 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	max7301[0] = 0x0eff;
	max7301[1] = 0x0eff;
	max7301[2] = 2;
 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	
	max7301[0] = 0x0fff;
	max7301[1] = 0x0fff;
	max7301[2] = 2;
 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	
}	
	
unsigned int contactors(unsigned int command){
	unsigned int max7301[5];
	unsigned int max7301RX[5];	
	
	max7301[0]=0xcc00;
	max7301[1]=0xd800;
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	printf("CONTACTORS START Returns %04x %04x \n",max7301[2],max7301[3]);
	sleep(1);
	switch(command){
	
		
	case 0:
		printf("Contactors Off\n");
		max7301[0] = 0x3400;
		max7301[1] = 0;
		max7301[2]=2;
		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
		max7301[0] = 0x3500;
		max7301[1] = 0;
		max7301[2]=2;
		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
		max7301[0] = 0x3600;
		max7301[1] = 0;
		max7301[2]=2;
		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
		max7301[0] = 0x3700;
		max7301[1] = 0;
		max7301[2]=2;
		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	break;
	
	case 1:
		printf("Contactors On\n");
		max7301[0] = 0x34ff;
		max7301[1] = 0;
		max7301[2]=2;
		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
		max7301[0] = 0x35ff;
		max7301[1] = 0;
		max7301[2]=2;
		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
		max7301[0] = 0x36ff;
		max7301[1] = 0;
		max7301[2]=2;
		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
		max7301[0] = 0x37ff;
		max7301[1] = 0;
		max7301[2]=2;
		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
// 		printf("Contactors are Engaged- Wait ~15 seconds for Warning Siren\n");
// 		sleep(10);
// 		
// 		printf("Warning Siren completed\n");
	break;

	}
	sleep(1);
	max7301[0]=0xcc00;
	max7301[1]=0xd800;
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	printf("CONTACTORS END Returns %04x %04x \n",max7301[2],max7301[3]);
	
}



unsigned int clutchbrake(unsigned int command){
	unsigned int max7301[5];
	unsigned int max7301RX[5];
	
	
	max7301[0]=0xcc00;
	max7301[1]=0xd800;
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	printf("CLUTCH BRAKE START 1Returns %04x %04x \n",max7301[2],max7301[3]);
	max7301[0]=0xd400;
	max7301[1]=0xd800;
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	printf("CLUTCH BRAKE START 2 Returns %04x %04x \n",max7301[2],max7301[3]);
	max7301[0]=0xdc00;
	max7301[1]=0xd800;
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	printf("CLUTCH BRAKE START 3 Returns %04x %04x \n",max7301[2],max7301[3]);
	sleep(1);
	
	switch(command){
	case 0:
		printf("Clutch Brake off\n");
		max7301[0] = 0x4c00;
		max7301[1] = 0x0000;
		max7301[2]=2;
		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	break;
	
	case 1:
		printf("Clutch Brake on\n");
		max7301[0] = 0x4cff;
		max7301[1] = 0x0000;
		max7301[2]=2;
		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	break;

	}
	
	sleep(1);
	max7301[0]=0xcc00;
	max7301[1]=0xd800;
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	printf("CLUTCH BRAKE END 1Returns %04x %04x \n",max7301[2],max7301[3]);
	max7301[0]=0xd400;
	max7301[1]=0xd800;
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	printf("CLUTCH BRAKE END 2Returns %04x %04x \n",max7301[2],max7301[3]);
	max7301[0]=0xdc00;
	max7301[1]=0xd800;
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	printf("CLUTCH BRAKE END 3 Returns %04x %04x \n",max7301[2],max7301[3]);
	
}
