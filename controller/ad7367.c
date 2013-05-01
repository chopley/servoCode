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
unsigned int get_crc(unsigned int packet);
unsigned int ad5362_crc_pack(unsigned int command,unsigned int channel,unsigned int value);

int main(int argc, char *argv[]){
	
	int fd;
	unsigned int ad5362[10];
	
	unsigned int command;
	unsigned short value1;
	unsigned char value;
	int i=0;

	fd = open("/dev/pid",O_RDWR);
	if(fd ==-1){
	perror("failed to open PID Control Module\n");
	//rc = rd;
	exit(-1);
	}
	value1=10000;
	printf("AD5362 Hello\n");
	//read in the module data from Kernel space when you start
	//set the 'virtual ports' to output as detailed in datasheet
	ioctl(fd,DEV_IOCTL_READ_AD7367,ad5362);
	i=0;
	while(i<10){
		i=i+1;
	//	ioctl(fd,DEV_IOCTL_READ_AD5362,ad5362);
		usleep(10000);	
		value1+=1000;
		ioctl(fd,DEV_IOCTL_READ_AD7367,ad5362);
		printf("%04d %04d %04d %04d %04d %04d\n",ad5362[0],ad5362[1],ad5362[2],ad5362[3],ad5362[4],ad5362[5],ad5362[6]);
		//usleep(10000);	
	}
	close(fd);

}


unsigned int get_crc(unsigned int packet){
	unsigned char test_vec[10];
	unsigned int retval;
	 crc_t crc;
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
