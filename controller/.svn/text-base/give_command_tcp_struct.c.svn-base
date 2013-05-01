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
#include "telescope_constants.h"
#include "pointing.h"
//#include "command_struct.h"


// #define REMOTE_SERVER_PORT 1500
// 
// #define LOCAL_SERVER_PORT 1501
// #define LOCAL_SERVER_PORT_STRUCT 1502
#define MAX_MSG 2000
#define PI 3.141592653589793238462643
#define R2D (180.0/PI) /* radians to degrees */

/* 
 * tcpclient.c - A simple TCP client
 * usage: tcpclient <host> <port>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <slalib.h>

#define BUFSIZE 1024

/* 
 * error - wrapper for perror
 */
void error(char *msg) {
    perror(msg);
    exit(0);
}

int quad(double *x, double *y,double *Yret,int length);
int equatorial2(int RAh,int RAm, double RAs,int DECd,int DECm,double DECs,time_t *timeout,int *lengthout,double *Yretalt,double *Yretaz);
int equatorial2_withpoint(int RAh,int RAm, double RAs,int DEC_deg,int DEC_min,double DECs,time_t *timeout,int *lengthout,double *Yretalt,double *Yretaz,struct new_message_parsing_struct *message);
int polynomial_withpoint(double AZStart, double AZEnd,double ALTStart,double ALTEnd,time_t *timeout,int length,double *Yretalt,double *Yretaz);

struct new_message_parsing_struct tcp_message,tcp_message_reply;

int main(int argc, char **argv) {
	FILE *pid_file;
	char *char_ptr;
	int fd,i,read_ret;
	int sd, rc;
	int coordinate_type;
	struct sockaddr_in cliAddr, remoteServAddr;
	struct hostent *h;
	char commands[100][100];
	char testbuff[100000];
	char udpbuf[1000000];
	double *Yretazin,*Yretaltin;
	int ra_hour,ra_min,ra_second;
	int RA[5],DEC[5];
	unsigned int command[10];
	int dec_deg,dec_min,dec_second;
	double RAd,DECd,DEC_sign;
	double RAdb[5],DECdb[5];
	double AZpoly[5],ALTpoly[5];
	double PID[4];
	long MOTOR[4];
	int j;
	double AZ,ALT,rac,dec,deltaaz,deltaalt;
	unsigned int AZ_ENCODER,ALT_ENCODER;
	time_t equatorial2_time[5];
	time_t current_time;
	int length;


    int sockfd, portno, n;
    struct sockaddr_in serveraddr;
    struct hostent *server;
    char *hostname;
    char buf[BUFSIZE];

    /* check command line arguments */
   

	coordinate_type = atoi(argv[2]);

	printf("Coordinate type %d\n",coordinate_type);

	switch(coordinate_type){

	case SAFE_DRIVE_LIMITS:
		printf("SAFE LIMITS COMMAND TO KERNEL \n");
		tcp_message.coordinate_type=coordinate_type;
		tcp_message.command_vals[0] = atoi(argv[3]);
		tcp_message.command_vals[1] = atoi(argv[4]);
		tcp_message.command_vals[2] = atoi(argv[5]);
		tcp_message.command_vals[3] = atoi(argv[6]);
		sprintf(udpbuf,"%5d, %5d,%5d,%5d,%5d,0\n",atoi(argv[3]),atoi(argv[4]),coordinate_type,atoi(argv[5]),atoi(argv[6]));
	break;

	case IOCTL_COMMANDS:
		printf("IOCTL COMMAND TO KERNEL \n");
		tcp_message.coordinate_type=coordinate_type;
		tcp_message.command_vals[0] = atoi(argv[3]);
		sprintf(udpbuf,"%5d, 0,%5d,0,0,0\n",atoi(argv[3]),coordinate_type);
	break;

	
	case HORIZONTAL_LIST:
		printf("Horizontal List\n");
		pid_file = fopen("hz_list.txt","r");
		if (pid_file == NULL) {
		  printf("I couldn't open hz_list.txt for writing.\n");
		  exit(0);
		}
		j=0;
		printf("Sending a Horizontal coordinate list %d\n",coordinate_type);
		tcp_message.coordinate_type=coordinate_type;
		time(&current_time);
		//pack the azimuth values
		while(fgets(testbuff, 150, pid_file) != NULL){
		//char_ptr =fgets(testbuff,50,pid_file);
		  printf("%s",testbuff);
		  sscanf(testbuff,"%[^','],%[^',']\n",&commands[0],&commands[1]);
		  tcp_message.az_commands[j] = atof(commands[0]);
		  tcp_message.alt_commands[j] = atof(commands[1]);
		  printf("Values %f %f\n",tcp_message.az_commands[j],tcp_message.alt_commands[j]);
		  j=j+1;
		}
		
		
		//for(i=0; i<=100; i++){
		  tcp_message.lcommand_vals[0]=current_time+j;
		  tcp_message.lcommand_vals[1]=current_time;
                 // tcp_message.az_commands[i]=i;
		 // tcp_message.alt_commands[i] = i;
		//  printf("i = %d \n");
		//}
		//sprintf(udpbuf,"%5d, 0,%5d,0,0,0\n",atoi(argv[3]),coordinate_type);
		fclose(pid_file);
	break;
	
	case AZ1_PID_COEFFICIENTS_ADAPTIVE:
		printf("AZ1 PID Adaptive\n");
		pid_file = fopen("az1pid.txt","r");
		if (pid_file == NULL) {
		  printf("I couldn't open az1pid.txt for writing.\n");
		  exit(0);
		}
		j=0;
		while(fgets(testbuff, 150, pid_file) != NULL){
		//char_ptr =fgets(testbuff,50,pid_file);
		//printf("%s",testbuff);
		sscanf(testbuff,"%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^',']\n",&commands[0],&commands[1],&commands[2],&commands[3],&commands[4],&commands[5],&commands[6],&commands[7],&commands[8],&commands[9],&commands[10]);
		//printf("%s %s\n",&commands[0],&commands[1]);
		tcp_message.coordinate_type=coordinate_type;
		tcp_message.pid_vals.position_error[j] = atoi(commands[0]);
		tcp_message.pid_vals.p[j] = atof(commands[1]);
		tcp_message.pid_vals.i[j] = atof(commands[2]);
		tcp_message.pid_vals.d[j] = atof(commands[3]);
		tcp_message.pid_vals.kf[j] = atof(commands[4]);
		tcp_message.pid_vals.vf[j] = atof(commands[5]);
		tcp_message.pid_vals.motor_plus[j] = atoi(commands[6]);
		tcp_message.pid_vals.motor_minus[j] = atoi(commands[7]);
		tcp_message.pid_vals.p_2[j] = atof(commands[8]);
		tcp_message.pid_vals.i_2[j] = atof(commands[9]);
		tcp_message.pid_vals.d_2[j] = atof(commands[10]);
		printf("%d %f %f %f vf %f %d %d p_2 %f %f %f\n",tcp_message.pid_vals.position_error[j],tcp_message.pid_vals.p[j],tcp_message.pid_vals.i[j],tcp_message.pid_vals.d[j],tcp_message.pid_vals.vf[j],tcp_message.pid_vals.motor_plus[j],tcp_message.pid_vals.motor_minus[j],tcp_message.pid_vals.p_2[j],tcp_message.pid_vals.i_2[j],tcp_message.pid_vals.d_2[j]);
		j++;
		}
		
		tcp_message.pid_vals.table_length=j;
		printf("Length %d %d\n",j,tcp_message.pid_vals.table_length);
		 fclose(pid_file);


		
	break;

	case AZ2_PID_COEFFICIENTS_ADAPTIVE:
		printf("AZ2 PID Adaptive\n");
		pid_file = fopen("az2pid.txt","r");
		if (pid_file == NULL) {
		  printf("I couldn't open az2pid.txt for writing.\n");
		  exit(0);
		}
		j=0;
		while(fgets(testbuff, 150, pid_file) != NULL){
		//char_ptr =fgets(testbuff,50,pid_file);
		//printf("%s",testbuff);
		sscanf(testbuff,"%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^',']\n",&commands[0],&commands[1],&commands[2],&commands[3],&commands[4],&commands[5],&commands[6],&commands[7],&commands[8],&commands[9],&commands[10]);
		//printf("%s %s\n",&commands[0],&commands[1]);
		tcp_message.coordinate_type=coordinate_type;
		tcp_message.pid_vals.position_error[j] = atoi(commands[0]);
		tcp_message.pid_vals.p[j] = atof(commands[1]);
		tcp_message.pid_vals.i[j] = atof(commands[2]);
		tcp_message.pid_vals.d[j] = atof(commands[3]);
		tcp_message.pid_vals.kf[j] = atof(commands[4]);
		tcp_message.pid_vals.vf[j] = atof(commands[5]);
		tcp_message.pid_vals.motor_plus[j] = atoi(commands[6]);
		tcp_message.pid_vals.motor_minus[j] = atoi(commands[7]);
		tcp_message.pid_vals.p_2[j] = atof(commands[8]);
		tcp_message.pid_vals.i_2[j] = atof(commands[9]);
		tcp_message.pid_vals.d_2[j] = atof(commands[10]);
		printf("%d %f %f %f vf %f %d %d p_2 %f %f %f\n",tcp_message.pid_vals.position_error[j],tcp_message.pid_vals.p[j],tcp_message.pid_vals.i[j],tcp_message.pid_vals.d[j],tcp_message.pid_vals.vf[j],tcp_message.pid_vals.motor_plus[j],tcp_message.pid_vals.motor_minus[j],tcp_message.pid_vals.p_2[j],tcp_message.pid_vals.i_2[j],tcp_message.pid_vals.d_2[j]);
		j++;
		}
		
		tcp_message.pid_vals.table_length=j;
		printf("Length %d %d\n",j,tcp_message.pid_vals.table_length);
		 fclose(pid_file);

		
	break;

	case ALT1_PID_COEFFICIENTS_ADAPTIVE:
		printf("ALT PID Adaptive\n");
		pid_file = fopen("alt1pid.txt","r");
		if (pid_file == NULL) {
		  printf("I couldn't open alt1pid.txt for writing.\n");
		  exit(0);
		}
		j=0;
		while(fgets(testbuff, 150, pid_file) != NULL){
		//char_ptr =fgets(testbuff,50,pid_file);
		//printf("%s",testbuff);
		sscanf(testbuff,"%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^',']\n",&commands[0],&commands[1],&commands[2],&commands[3],&commands[4],&commands[5],&commands[6],&commands[7],&commands[8],&commands[9],&commands[10]);
		//printf("%s %s\n",&commands[0],&commands[1]);
		tcp_message.coordinate_type=coordinate_type;
		tcp_message.pid_vals.position_error[j] = atoi(commands[0]);
		tcp_message.pid_vals.p[j] = atof(commands[1]);
		tcp_message.pid_vals.i[j] = atof(commands[2]);
		tcp_message.pid_vals.d[j] = atof(commands[3]);
		tcp_message.pid_vals.kf[j] = atof(commands[4]);
		tcp_message.pid_vals.vf[j] = atof(commands[5]);
		tcp_message.pid_vals.motor_plus[j] = atoi(commands[6]);
		tcp_message.pid_vals.motor_minus[j] = atoi(commands[7]);
		tcp_message.pid_vals.p_2[j] = atof(commands[8]);
		tcp_message.pid_vals.i_2[j] = atof(commands[9]);
		tcp_message.pid_vals.d_2[j] = atof(commands[10]);
		printf("%d %f %f %f vf %f %d %d p_2 %f %f %f\n",tcp_message.pid_vals.position_error[j],tcp_message.pid_vals.p[j],tcp_message.pid_vals.i[j],tcp_message.pid_vals.d[j],tcp_message.pid_vals.vf[j],tcp_message.pid_vals.motor_plus[j],tcp_message.pid_vals.motor_minus[j],tcp_message.pid_vals.p_2[j],tcp_message.pid_vals.i_2[j],tcp_message.pid_vals.d_2[j]);
		j++;
		}
		
		tcp_message.pid_vals.table_length=j;
		printf("Length %d %d\n",j,tcp_message.pid_vals.table_length);
		 fclose(pid_file);


		
	break;
	
	case ALT2_PID_COEFFICIENTS_ADAPTIVE:
		printf("ALT PID Adaptive\n");
		pid_file = fopen("alt2pid.txt","r");
		if (pid_file == NULL) {
		  printf("I couldn't open alt2pid.txt for writing.\n");
		  exit(0);
		}
		j=0;
		while(fgets(testbuff, 150, pid_file) != NULL){
		//char_ptr =fgets(testbuff,50,pid_file);
		//printf("%s",testbuff);
		sscanf(testbuff,"%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^',']\n",&commands[0],&commands[1],&commands[2],&commands[3],&commands[4],&commands[5],&commands[6],&commands[7],&commands[8],&commands[9],&commands[10]);
		//printf("%s %s\n",&commands[0],&commands[1]);
		tcp_message.coordinate_type=coordinate_type;
		tcp_message.pid_vals.position_error[j] = atoi(commands[0]);
		tcp_message.pid_vals.p[j] = atof(commands[1]);
		tcp_message.pid_vals.i[j] = atof(commands[2]);
		tcp_message.pid_vals.d[j] = atof(commands[3]);
		tcp_message.pid_vals.kf[j] = atof(commands[4]);
		tcp_message.pid_vals.vf[j] = atof(commands[5]);
		tcp_message.pid_vals.motor_plus[j] = atoi(commands[6]);
		tcp_message.pid_vals.motor_minus[j] = atoi(commands[7]);
		tcp_message.pid_vals.p_2[j] = atof(commands[8]);
		tcp_message.pid_vals.i_2[j] = atof(commands[9]);
		tcp_message.pid_vals.d_2[j] = atof(commands[10]);
		printf("%d %f %f %f vf %f %d %d p_2 %f %f %f\n",tcp_message.pid_vals.position_error[j],tcp_message.pid_vals.p[j],tcp_message.pid_vals.i[j],tcp_message.pid_vals.d[j],tcp_message.pid_vals.vf[j],tcp_message.pid_vals.motor_plus[j],tcp_message.pid_vals.motor_minus[j],tcp_message.pid_vals.p_2[j],tcp_message.pid_vals.i_2[j],tcp_message.pid_vals.d_2[j]);
		j++;
		}
		
		tcp_message.pid_vals.table_length=j;
		printf("Length %d %d\n",j,tcp_message.pid_vals.table_length);
		 fclose(pid_file);

		
	break;
	



	case AZ1_PID_COEFFICIENTS:
		printf("AZ1 PID command\n");
		if(argc<5) {
			printf("Use this to give AZ1 PID command : %s <server> <data1> ... <dataN> AZIMUTH_ZONE\n", argv[0]);
		exit(1);
		}
		tcp_message.coordinate_type=coordinate_type;
		tcp_message.fcommand_vals[0] = atof(argv[3]);
		tcp_message.fcommand_vals[1] = atof(argv[4]);
		tcp_message.fcommand_vals[2] = atof(argv[5]);
		tcp_message.command_vals[0] = atoi(argv[6]);
		tcp_message.command_vals[1] = atoi(argv[7]);
		PID[0] = atof(argv[3]);
		PID[1] = atof(argv[4]);
		PID[2] = atof(argv[5]);
		MOTOR[0] = atoi(argv[6]);
		MOTOR[1] = atoi(argv[7]);
		sprintf(udpbuf,"%3.5f,%3.5f,%5d,%3.5f,%5d,%5d\n",PID[0],PID[1],coordinate_type,PID[2],MOTOR[0],MOTOR[1]);
	break;


	case AZ2_PID_COEFFICIENTS:
		printf("AZ1 PID command\n");
		if(argc<5) {
			printf("Use this to give AZ1 PID command : %s <server> <data1> ... <dataN> AZIMUTH_ZONE\n", argv[0]);
		exit(1);
		}
		tcp_message.coordinate_type=coordinate_type;
		tcp_message.fcommand_vals[0] = atof(argv[3]);
		tcp_message.fcommand_vals[1] = atof(argv[4]);
		tcp_message.fcommand_vals[2] = atof(argv[5]);
		tcp_message.command_vals[0] = atoi(argv[6]);
		tcp_message.command_vals[1] = atoi(argv[7]);
		PID[0] = atof(argv[3]);
		PID[1] = atof(argv[4]);
		PID[2] = atof(argv[5]);
		MOTOR[0] = atoi(argv[6]);
		MOTOR[1] = atoi(argv[7]);
		sprintf(udpbuf,"%3.5f,%3.5f,%5d,%3.5f,%5d,%5d\n",PID[0],PID[1],coordinate_type,PID[2],MOTOR[0],MOTOR[1]);
	break;

	case ALT1_PID_COEFFICIENTS:
		printf("AZ1 PID command\n");
		if(argc<5) {
			printf("Use this to give AZ1 PID command : %s <server> <data1> ... <dataN> AZIMUTH_ZONE\n", argv[0]);
		exit(1);
		}
		tcp_message.coordinate_type=coordinate_type;
		tcp_message.fcommand_vals[0] = atof(argv[3]);
		tcp_message.fcommand_vals[1] = atof(argv[4]);
		tcp_message.fcommand_vals[2] = atof(argv[5]);
		tcp_message.command_vals[0] = atoi(argv[6]);
		tcp_message.command_vals[1] = atoi(argv[7]);
		PID[0] = atof(argv[3]);
		PID[1] = atof(argv[4]);
		PID[2] = atof(argv[5]);
		MOTOR[0] = atoi(argv[6]);
		MOTOR[1] = atoi(argv[7]);
		sprintf(udpbuf,"%3.5f,%3.5f,%5d,%3.5f,%5d,%5d\n",PID[0],PID[1],coordinate_type,PID[2],MOTOR[0],MOTOR[1]);
	break;

	case ALT2_PID_COEFFICIENTS:
		printf("AZ1 PID command\n");
		if(argc<5) {
			printf("Use this to give AZ1 PID command : %s <server> <data1> ... <dataN> AZIMUTH_ZONE\n", argv[0]);
		exit(1);
		}
		tcp_message.coordinate_type=coordinate_type;
		tcp_message.fcommand_vals[0] = atof(argv[3]);
		tcp_message.fcommand_vals[1] = atof(argv[4]);
		tcp_message.fcommand_vals[2] = atof(argv[5]);
		tcp_message.command_vals[0] = atoi(argv[6]);
		tcp_message.command_vals[1] = atoi(argv[7]);
		PID[0] = atof(argv[3]);
		PID[1] = atof(argv[4]);
		PID[2] = atof(argv[5]);
		MOTOR[0] = atoi(argv[6]);
		MOTOR[1] = atoi(argv[7]);
		sprintf(udpbuf,"%3.5f,%3.5f,%5d,%3.5f,%5d,%5d\n",PID[0],PID[1],coordinate_type,PID[2],MOTOR[0],MOTOR[1]);
	break;

	case HORIZONTAL:
		printf("HZ\n");
		if(argc<4) {
			printf("Use this to give AZ/ALT command : %s <server> <data1> ... <dataN> AZIMUTH_ZONE\n", argv[0]);
		exit(1);
		}
		tcp_message.coordinate_type=coordinate_type;
		
		tcp_message.fcommand_vals[0] = atof(argv[3]);
		tcp_message.fcommand_vals[1] = atof(argv[4]);
		AZ = (double)tcp_message.fcommand_vals[0];
		ALT = (double)tcp_message.fcommand_vals[1];
		pointing(AZ,ALT,&deltaaz, &deltaalt);
		AZ += deltaaz;
		ALT+= deltaalt;
		tcp_message.fcommand_vals[0] = (float) AZ;
		tcp_message.fcommand_vals[1] = (float) ALT;
		printf("%f %f \n",tcp_message.fcommand_vals[0],tcp_message.fcommand_vals[1]);
		AZ = atof(argv[3]);
		ALT = atof(argv[4]);
		sprintf(udpbuf,"%3.5f,%3.5f,%5d,0,0,0\n",AZ,ALT,coordinate_type);
		
	break;
	
	case EQUATORIAL:
		printf("EQ\n");
		if(argc<8) {
		printf("Use this to give RA/DEC Command : %s <server> <data1> ... <dataN> AZIMUTH_ZONE\n", argv[0]);
		exit(1);
		}




		for(i=0;i<3;i++){
	
		RA[i]=atoi(argv[i+3]);
		printf("RA %d %d\n",i,RA[i]);
		}

		for(i=0;i<3;i++){
	
		DEC[i]=atoi(argv[i+6]);
		printf("DEC %d %d\n",i,DEC[i]);
		}


		if(DEC[0]<0){
			DEC[0] = 360+DEC[0];
		}
		slaDtf2r(RA[0],RA[1],RAd,&rac,&j);
		if(j)
			printf("Error with Dtf2r\n");
	

		slaDaf2r(DEC[0],DEC[1],DECd,&dec,&j);
		if(j)
			printf("Error with Daf2r\n");	

		
		printf("RAd %f\n",rac);
		printf("DECd %f\n",dec);
		tcp_message.coordinate_type=coordinate_type;
		tcp_message.fcommand_vals[0] = rac*R2D;
		tcp_message.fcommand_vals[1] = dec*R2D;
		
		sprintf(udpbuf,"%3.5f,%3.5f,%5d,0,0,0\n",rac*R2D,dec*R2D,coordinate_type);
		
	
	break;

	case ENCODER_COORDS:
		AZ_ENCODER=atoi(argv[3]);
		ALT_ENCODER=atoi(argv[4]);
		tcp_message.coordinate_type=coordinate_type;
		tcp_message.command_vals[0] = AZ_ENCODER;
		tcp_message.command_vals[1] = ALT_ENCODER;
		sprintf(udpbuf,"%6d,%6d,%5d,0,0,0\n",AZ_ENCODER,ALT_ENCODER,coordinate_type);
	break;

	case EQUATORIAL2:
		printf("Equaotrials 2\n");
		Yretazin= (double *)malloc(20*sizeof(double));
		if(Yretazin == NULL){
			printf("Problem allocating Yretaz memory!\n");
		}
	
		Yretaltin= (double *)malloc(20*sizeof(double));
		if(Yretaltin == NULL){
			printf("Problem allocating Yretalt memory!\n");
		}
		RA[0] = atoi(argv[3]);
		RA[1] = atoi(argv[4]);
		RAdb[0] = atof(argv[5]);
		DEC[0] = atoi(argv[6]);
		DEC[1] = atoi(argv[7]);
		DECdb[0] = atof(argv[8]);
		printf("Equaotrials 2\n");
		equatorial2(RA[0],RA[1],RAdb[0],DEC[0],DEC[1],DECdb[0],equatorial2_time,&length,Yretaltin,Yretazin);
		printf("Yvals %16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f\n",Yretazin[0],*(Yretazin+1),*(Yretazin+2),*(Yretazin+3),Yretaltin[0],Yretaltin[1],Yretaltin[2],Yretaltin[3]);
		printf("Equaotrials 2\n");
		printf("%6d,%6d,%5d,%6d,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f\n",equatorial2_time[0],equatorial2_time[1],coordinate_type,length,*(Yretazin),*(Yretazin+1),*(Yretazin+2),*(Yretazin+3),Yretaltin[0],Yretaltin[1],Yretaltin[2],Yretaltin[3]);
	
		sprintf(udpbuf,"%6d,%6d,%5d,%6d,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f\n",equatorial2_time[0],equatorial2_time[1],coordinate_type,length,*(Yretazin),*(Yretazin+1),*(Yretazin+2),*(Yretazin+3),Yretaltin[0],Yretaltin[1],Yretaltin[2],Yretaltin[3]);
		tcp_message.coordinate_type=coordinate_type;
		tcp_message.lcommand_vals[0] = equatorial2_time[0];
		tcp_message.lcommand_vals[1] = equatorial2_time[1];
		tcp_message.fcommand_vals[0] = (float)*(Yretazin);
		tcp_message.fcommand_vals[1] = (float)*(Yretazin+1);
		tcp_message.fcommand_vals[2] = (float)*(Yretazin+2);
		tcp_message.fcommand_vals[3] = (float)*(Yretazin+3);
		tcp_message.fcommand_vals[4] = (float)*(Yretaltin+0);
		tcp_message.fcommand_vals[5] = (float)*(Yretaltin+1);
		tcp_message.fcommand_vals[6] = (float)*(Yretaltin+2);
		tcp_message.fcommand_vals[7] = (float)*(Yretaltin+3);
		
		free(Yretazin);
		free(Yretaltin);
		//sprintf(udpbuf,"%6d,%6d,%5d,0,0,0\n",AZ_ENCODER,ALT_ENCODER,coordinate_type);
		
	break;

	case EQUATORIAL2_WITH_POINTING:
		printf("Equaotrials 2 with pointing model\n");
		Yretazin= (double *)malloc(20*sizeof(double));
		if(Yretazin == NULL){
			printf("Problem allocating Yretaz memory!\n");
		}
	
		Yretaltin= (double *)malloc(20*sizeof(double));
		if(Yretaltin == NULL){
			printf("Problem allocating Yretalt memory!\n");
		}
		coordinate_type = EQUATORIAL2;
		RA[0] = atoi(argv[3]);
		RA[1] = atoi(argv[4]);
		RAdb[0] = atof(argv[5]);
		DEC[0] = atoi(argv[6]);
		DEC[1] = atoi(argv[7]);
		DECdb[0] = atof(argv[8]);
		printf("Equatorials 2\n");
		equatorial2_withpoint(RA[0],RA[1],RAdb[0],DEC[0],DEC[1],DECdb[0],equatorial2_time,&length,Yretaltin,Yretazin,&tcp_message);
		//printf("Yvals %16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f\n",Yretazin[0],*(Yretazin+1),*(Yretazin+2),*(Yretazin+3),Yretaltin[0],Yretaltin[1],Yretaltin[2],Yretaltin[3]);
		//printf("Equaotrials 2\n");
		//printf("%6d,%6d,%5d,%6d,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f\n",equatorial2_time[0],equatorial2_time[1],coordinate_type,length,*(Yretazin),*(Yretazin+1),*(Yretazin+2),*(Yretazin+3),Yretaltin[0],Yretaltin[1],Yretaltin[2],Yretaltin[3]);
		for(i=0;i<=length;i++){
		// printf("Struct %f %f %ld %ld\n",tcp_message.az_commands[i],tcp_message.alt_commands[i],tcp_message.lcommand_vals[0],tcp_message.lcommand_vals[1]) ;
		}
		
		
		
		sprintf(udpbuf,"%6d,%6d,%5d,%6d,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f\n",equatorial2_time[0],equatorial2_time[1],coordinate_type,length,*(Yretazin),*(Yretazin+1),*(Yretazin+2),*(Yretazin+3),Yretaltin[0],Yretaltin[1],Yretaltin[2],Yretaltin[3]);
		tcp_message.coordinate_type=HORIZONTAL_LIST;
		tcp_message.message_size = sizeof(tcp_message);
		//tcp_message.coordinate_type=coordinate_type;
		//tcp_message.lcommand_vals[0] = equatorial2_time[0];
		//tcp_message.lcommand_vals[1] = equatorial2_time[1];
		//tcp_message.command_vals[0] = length;
		//tcp_message.fcommand_vals[0] = (float)*(Yretazin);
		//tcp_message.fcommand_vals[1] = (float)*(Yretazin+1);
		//tcp_message.fcommand_vals[2] = (float)*(Yretazin+2);
		//tcp_message.fcommand_vals[3] = (float)*(Yretazin+3);
		//tcp_message.fcommand_vals[4] = (float)*(Yretaltin+0);
		//tcp_message.fcommand_vals[5] = (float)*(Yretaltin+1);
		//tcp_message.fcommand_vals[6] = (float)*(Yretaltin+2);
		//tcp_message.fcommand_vals[7] = (float)*(Yretaltin+3);
		//printf("struct vals %16.9f %16.9f\n",tcp_message.dcommand_vals[0],tcp_message.dcommand_vals[1]);
		free(Yretazin);
		free(Yretaltin);
	break;
	
	case POLYNOMIAL_POINT:
		printf("POLYNOMIAL COEFFICIENTS\n");
		Yretazin= (double *)malloc(20*sizeof(double));
		if(Yretazin == NULL){
			printf("Problem allocating Yretaz memory!\n");
		}
	
		Yretaltin= (double *)malloc(20*sizeof(double));
		if(Yretaltin == NULL){
			printf("Problem allocating Yretalt memory!\n");
		}
		coordinate_type = EQUATORIAL2;
		AZpoly[0] = atof(argv[3]);
		AZpoly[1] = atof(argv[4]);
		ALTpoly[0] = atof(argv[5]);
		ALTpoly[1] = atof(argv[6]);
		length = atoi(argv[7]);
		//length=100;
		printf("POLY POINT\n");
		polynomial_withpoint(AZpoly[0],AZpoly[1],ALTpoly[0],ALTpoly[1],equatorial2_time,length,Yretaltin,Yretazin);
		printf("Values from parsing %6d,%6d,%5d,%6d,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f\n",equatorial2_time[0],equatorial2_time[1],coordinate_type,length,*(Yretazin),*(Yretazin+1),*(Yretazin+2),*(Yretazin+3),Yretaltin[0],Yretaltin[1],Yretaltin[2],Yretaltin[3]);
	
		
		tcp_message.coordinate_type=coordinate_type;
		tcp_message.lcommand_vals[0] = equatorial2_time[0];
		tcp_message.lcommand_vals[1] = equatorial2_time[1];
		tcp_message.command_vals[0] = length;
		tcp_message.fcommand_vals[0] = (float)*(Yretazin);
		tcp_message.fcommand_vals[1] = (float)*(Yretazin+1);
		tcp_message.fcommand_vals[2] = (float)*(Yretazin+2);
		tcp_message.fcommand_vals[3] = (float)*(Yretazin+3);
		tcp_message.fcommand_vals[4] = (float)*(Yretaltin+0);
		tcp_message.fcommand_vals[5] = (float)*(Yretaltin+1);
		tcp_message.fcommand_vals[6] = (float)*(Yretaltin+2);
		tcp_message.fcommand_vals[7] = (float)*(Yretaltin+3);
		
		//printf("Poly Values Sent %6d,%6d,%5d,%6d,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f\n",tcp_message.lcommand_vals[0],tcp_message.lcommand_vals[1],tcp_message.coordinate_type,tcp_message.command_vals[0],tcp_message.fcommand_vals[0],tcp_message.fcommand_vals[1],tcp_message.fcommand_vals[2],tcp_message.fcommand_vals[3],tcp_message.fcommand_vals[4],tcp_message.fcommand_vals[5],tcp_message.fcommand_vals[6]);
		//printf("struct vals %16.9f %16.9f\n",tcp_message.dcommand_vals[0],tcp_message.dcommand_vals[1]);
		free(Yretazin);
		free(Yretaltin);
	break;

	case CONTACTORS:
		printf("Contactors\n");
		command[0] = atoi(argv[3]);
		printf("%6d,%6d,%6d\n",command[0],0,coordinate_type);
		sprintf(udpbuf,"%6d,%6d,%6d\n",command[0],0,coordinate_type);
		tcp_message.coordinate_type=coordinate_type;
		tcp_message.command_vals[0] = command[0];
	break;

	case CLUTCHBRAKE:
		printf("Clutch Brake\n");
		command[0] = atoi(argv[3]);
		printf("%6d,%6d,%6d\n",command[0],0,coordinate_type);
		sprintf(udpbuf,"%6d,%6d,%6d\n",command[0],0,coordinate_type);
		tcp_message.coordinate_type=coordinate_type;
		tcp_message.command_vals[0] = command[0];
	break;
	
	default:
	  printf("Failed to get correct case here\n");
	break;

	}



  	h = gethostbyname(argv[1]);
	if(h==NULL) {
	printf("%s: unknown host '%s' \n", argv[0], argv[1]);
	exit(1);
	}

	printf("%s: sending data to '%s' (IP : %s) \n", argv[0], h->h_name,
	inet_ntoa(*(struct in_addr *)h->h_addr_list[0]));

   //	portno = atoi(argv[2]);

 	remoteServAddr.sin_family = h->h_addrtype;
	memcpy((char *) &remoteServAddr.sin_addr.s_addr,
	h->h_addr_list[0], h->h_length);
	remoteServAddr.sin_port = htons(LOCAL_SERVER_PORT_STRUCT);
	
	/* socket creation */
	sd = socket(AF_INET,SOCK_STREAM,0);
	if(sd<0) {
	printf("%s: cannot open socket \n",argv[0]);
	exit(1);
	}

//     /* build the server's Internet address */
//   	bzero((char *) &serveraddr, sizeof(serveraddr));
//   	serveraddr.sin_family = AF_INET;
//     	bcopy((char *)server->h_addr, 
// 	(char *)&serveraddr.sin_addr.s_addr, server->h_length);
//     	serveraddr.sin_port = htons(portno);

    /* connect: create a connection with the server */
   	if (connect(sd, &remoteServAddr, sizeof(remoteServAddr)) < 0) 
	  error("ERROR connecting");

    /* get message line from the user */
  
   // bzero(buf, BUFSIZE);
    //fgets(buf, BUFSIZE, stdin);




    /* send the message line to the server */
    n = send(sd, &tcp_message, sizeof(tcp_message),0);
    if (n < 0) 
      error("ERROR writing to socket");

    /* print the server's reply */
    bzero(&tcp_message_reply, sizeof(tcp_message_reply));
    length=0;
    n=1;
    while(length<sizeof(tcp_message_reply.message_size) && n>0){
      n = recv(sd, &tcp_message_reply.message_size, sizeof(tcp_message_reply.message_size),0);
      if (n < 0) 
	error("ERROR reading from socket");
      length=length+n;
      printf("Received %d %d %d\n",n,length,n);
    }
    printf("tcp_message_reply %d %d %d %d\n",tcp_message.message_size,tcp_message_reply.message_size,n,tcp_message_reply.coordinate_type);
    length=0;
    n=1;
   while(n>1 && length<=5 && (tcp_message_reply.message_size != tcp_message.message_size)){
     length++; 
     printf("Message did not make it through properly %d %d %d %d\n",tcp_message.message_size,tcp_message_reply.message_size,n,length);
      n = send(sd, &tcp_message, sizeof(tcp_message),0);
      if (n < 0) 
	error("ERROR writing to socket");
      bzero(&tcp_message_reply, sizeof(tcp_message_reply));
      n = recv(sd, &tcp_message_reply.message_size, sizeof(tcp_message_reply.message_size),0);
      if (n < 0) 
	error("ERROR reading from socket");
    }
    
	//printf("Echo from server: %s\n",udpbuf);
	sleep(1);
    close(sd);

	
    return 0;
	
}

int equatorial2(int RAh,int RAm, double RAs,int DEC_deg,int DEC_min,double DECs,time_t *timeout,int *lengthout,double *Yretalt,double *Yretaz){
  // CIpars ciprms;
	double rc, dc, pr, pd, px, rv, date, xp, yp, dut, dx, dy,
	elong, phi, hm, tk, pmb, rh, wl, tlr, eo,
	ri, di, *aob, *zob, *hob, *dob, *rob, ra, da,*elob,*aob1,ra1, da1,  elong1, phi1, daz;
	double *djutc,*fdutc,djtt,djut1,stl;
	
	double ax,a_az,a_alt;
	int j,i;
	double *mdate;
	int RA[5],DEC[5];
	//int dec_deg,dec_min,dec_second;
	double RAd,DECd,DEC_sign;
	double RAdb[5],DECdb[5];
	int length=100;
	int length2;

	
	struct timezone tzp;
	time_t *timev;
	time_t *fittimev;
	double *timevd;
	struct timeval time_struct;
	struct tm *time_ptr;

	length2=length+10;
	//printf("Here\n");
	djutc= (double *)malloc(length2*sizeof(double));
	if(djutc == NULL){
		printf("Problem allocating djutc memory!\n");
	}
	
	fdutc= (double *)malloc(length2*sizeof(double));
	if(fdutc == NULL){
		printf("Problem allocating fdutc memory!\n");
	}

	


	mdate= (double *)malloc(length2*sizeof(double));
	if(mdate == NULL){
		printf("Problem allocating mdate memory!\n");
	}
	
	timevd= (double *)malloc(length2*sizeof(double));
	if(timevd == NULL){
		printf("Problem allocating timevd memory!\n");
	}

	timev= (time_t *)malloc(length2*sizeof(time_t));
	if(timev == NULL){
		printf("Problem allocating timev memory!\n");
	}

	fittimev= (time_t *)malloc(length2*sizeof(time_t));
	if(fittimev == NULL){
		printf("Problem allocating fittimev memory!\n");
	}

	elob= (double *)malloc(length2*sizeof(double));
	if(elob == NULL){
		printf("Problem allocating elob memory!\n");
	}
	
	aob1= (double *)malloc(length2*sizeof(double));
	if(aob1 == NULL){
		printf("Problem allocating aob1 memory!\n");
	}

	aob= (double *)malloc(length2*sizeof(double));
	if(aob == NULL){
		printf("Problem allocating aob memory!\n");
	}

	zob= (double *)malloc(length2*sizeof(double));
	if(zob == NULL){
		printf("Problem allocating zob memory!\n");
	}

	hob= (double *)malloc(length2*sizeof(double));
	if(hob == NULL){
		printf("Problem allocating hob memory!\n");
	}

	rob= (double *)malloc(length2*sizeof(double));
	if(rob == NULL){
		printf("Problem allocating rob memory!\n");
	}

	dob= (double *)malloc(length2*sizeof(double));
	if(dob == NULL){
		printf("Problem allocating dob memory!\n");
	}

	//printf("Here\n");
	RA[0] = RAh;
	RA[1]=	RAm;
	RAd = RAs;

	DEC[0]=DEC_deg;
	DEC[1] = DEC_min;
	DECd = DECs;

	if(DEC[0]<0){
		DEC[0] = 360+DEC[0];
	}
	slaDtf2r(RA[0],RA[1],RAd,&rc,&j);
	if(j)
		printf("Error with Dtf2r\n");
	

	slaDaf2r(DEC[0],DEC[1],DECd,&dc,&j);
	if(j)
		printf("Error with Daf2r\n");		

	//dc = 360./R2D-dc	;
	pr = 0;     /* RA proper motion (radians/year) */
	pr=0;  
	pd =0;             /* Dec proper motion (radians/year) */
	pd=0;
	px = 0;                 /* parallax (arcsec) */
	rv = 0;                   /* radial velocity (km/s) */
	/* Date (UTC, but also used as approximate TDB). */
	//   date = 53569.4;               /* 2005 July 18 09:36, as an MJD */
	/* Earth orientation parameters (from IERS Bulletin B). */
	xp = 0;            /* polar motion x (radians) */
	yp = 0;           /* polar motion y (radians) */
	dut = 0;              /* UT1-UTC (s) */
	dx = 0;              /* nutation adjustment dX */
	dy = 0;             /* nutation adjustment dY */
	/* Local circumstances (Gemini North). */
	elong =27.6853/R2D;     /* east longitude (radians) */
	phi = -25.8897/R2D;        /* latitude (radians) */
		
		//elong = -155.4690472/R2D; /* east longitude (radians) */
		//phi = 19.82380278/R2D;    /* latitude (radians) */
	
	hm = 1415.821;                  /* height a.s.l. (m) */
	tk = 296.13;                     /* temperature (K) */
	pmb = 1013.3;                  /* pressure (hPa) */
	rh = 0.45;                    /* relative humidity (0-1) */
	wl = 200.;                     /* observing wavelength (microns) */
	tlr = 0.0065;                 /* lapse rate (K/m) */
	
	//printf("Here3\n");
	time(&timev[0]);
	for(i=0;i<length;i++){
		//sleep(1);
		//time(&timev);
		if(i!=0){
			timev[i] = timev[i-1]+1;
		}
		time_ptr = gmtime(&(timev[i]));
		//printf("Here %d\n",i);
		
		slaCldj(time_ptr->tm_year+1900,time_ptr->tm_mon+1,time_ptr->tm_mday,&djutc[i],&j);
		if(j){
			printf("error with slaCldj\n");
		}
		slaDtf2d(time_ptr->tm_hour,time_ptr->tm_min,(double)time_ptr->tm_sec,&fdutc[i],&j);
		if(j){
			printf("error with slaDtf2d\n");
		}
		//printf("%5d %5d %5d %5d %5d %5d %8ld ",time_ptr->tm_year+1900,time_ptr->tm_mon+1,time_ptr->tm_mday,time_ptr->tm_hour,time_ptr->tm_min,time_ptr->tm_sec,timev[i]);
// 	//slaDtf2d(18,27,50,&fdutc,&j);
// 	if(j!=0){
// 		printf("Error in second time conversion->fdutc!\n");
// 	}	
		djutc[i]+=fdutc[i];
		mdate[i]=djutc[i];
		//printf("Here %d\n",i);
		slaMap ( rc, dc, pr, pd, px, rv, 2000.0, mdate[i], &ra1, &da1 );
		if(j){
			printf("error with slaMAP\n");
		}
		//printf("Here %d\n",i);
		slaAop ( ra1, da1, mdate[i], dut, elong, phi, hm, xp, yp,tk, pmb, rh, wl, tlr, &aob[i], &zob[i], &hob[i], &dob[i], &rob[i] );
		if(j){
			printf("error with slaAop\n");
		}
		//printf("Here %d\n",i);
		//slaPolmo ( elong, phi, xp, yp, &elong1, &phi1, &daz );
		//if(j){
		//	printf("error with slaPolmo\n");
		//}
		//printf("Here %d\n",i);
		//aob[i] += daz;
	//	printf("Observed Azimuth %+16.9f Observed Elevation %+16.9f \n",slaDranrm(aob[i])*R2D,90.0-zob[i]*R2D);
	}
	//printf("Here\n");
	for(i=0;i<length;i++){
		fittimev[i] = timev[i]-timev[0];
		//printf("fit time %ld \n",fittimev[i]); 
		timevd[i] = (double)fittimev[i];
		aob1[i] = slaDranrm(aob[i])*R2D;
		elob[i] = 90.0-zob[i]*R2D;
		
	}
	
	quad(timevd,aob1,Yretaz,length);
	quad(timevd,elob,Yretalt,length);

	for(i=0;i<length-10;i++){
		ax=*(timevd+i);
		a_az = *(Yretaz) + ax*(*(Yretaz+1)) + ax*(*(Yretaz+2));
		//ay = *(Yret) + ax*(*(Yret+1)) + ax*(*(Yret+2) + ax*(*(Yret+3)))
		a_alt = *(Yretalt) + ax*(*(Yretalt+1)) + ax*(*(Yretalt+2));
		//printf("i = %d time %+16.9f az %+16.9f %+16.9f %+16.9f alt %+16.9f %+16.9f %+16.9f \n",i,*(timevd+i),*(aob1+i),a_az,*(aob1+i)-a_az,*(elob+i),a_alt,*(elob+i)-a_alt);	
	}
	i=0;
	ax=*(timevd+i);
	a_az = *(Yretaz) + ax*(*(Yretaz+1)) + ax*(*(Yretaz+2));
		//ay = *(Yret) + ax*(*(Yret+1)) + ax*(*(Yret+2) + ax*(*(Yret+3)))
	a_alt = *(Yretalt) + ax*(*(Yretalt+1)) + ax*(*(Yretalt+2));
	printf("i = %d time %+16.9f az %+16.9f %+16.9f %+16.9f alt %+16.9f %+16.9f %+16.9f \n",i,*(timevd+i),*(aob1+i),a_az,*(aob1+i)-a_az,*(elob+i),a_alt,*(elob+i)-a_alt);
	//printf("Yvals %16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f\n",*(Yretaz),*(Yretaz+1),*(Yretaz+2),*(Yretaz+3),Yretalt[0],Yretalt[1],Yretalt[2],Yretalt[3]);
//	printf("end\n");
	free(djutc);
	free(fdutc);
	free(mdate);
	free(timevd);
	free(timev);
	free(fittimev);
	free(elob);
	free(aob1);
	free(aob);
	free(zob);
	free(hob);
	free(rob);
	//printf("end2\n");
	*timeout = timev[0]+length;
	*(timeout+1) = timev[0];
	*lengthout = length;
	//;
	//printf("end2\n");
	return 0;
};


int equatorial2_withpoint(int RAh,int RAm, double RAs,int DEC_deg,int DEC_min,double DECs,time_t *timeout,int *lengthout,double *Yretalt,double *Yretaz,struct new_message_parsing_struct *message){
  // CIpars ciprms;
	double rc, dc, pr, pd, px, rv, date, xp, yp, dut, dx, dy,
	elong, phi, hm, tk, pmb, rh, wl, tlr, eo,
	ri, di, *aob, *zob, *hob, *dob, *rob, ra, da,*elob,*aob1,ra1, da1,  elong1, phi1, daz;
	double *djutc,*fdutc,djtt,djut1,stl;
	double ax,a_az,a_alt;
	double delta_az,delta_alt;
	int j,i;
	double *mdate;
	int RA[5],DEC[5];
	//int dec_deg,dec_min,dec_second;
	double RAd,DECd,DEC_sign;
	double RAdb[5],DECdb[5];
	int length=40;
	int length2;

	
	struct timezone tzp;
	time_t *timev;
	time_t *fittimev;
	double *timevd;
	struct timeval time_struct;
	struct tm *time_ptr;
	time_t current_time;
	
	length2=length+10;
	//printf("Here\n");
	djutc= (double *)malloc(length2*sizeof(double));
	if(djutc == NULL){
		printf("Problem allocating djutc memory!\n");
	}
	
	fdutc= (double *)malloc(length2*sizeof(double));
	if(fdutc == NULL){
		printf("Problem allocating fdutc memory!\n");
	}

	


	mdate= (double *)malloc(length2*sizeof(double));
	if(mdate == NULL){
		printf("Problem allocating mdate memory!\n");
	}
	
	timevd= (double *)malloc(length2*sizeof(double));
	if(timevd == NULL){
		printf("Problem allocating timevd memory!\n");
	}

	timev= (time_t *)malloc(length2*sizeof(time_t));
	if(timev == NULL){
		printf("Problem allocating timev memory!\n");
	}

	fittimev= (time_t *)malloc(length2*sizeof(time_t));
	if(fittimev == NULL){
		printf("Problem allocating fittimev memory!\n");
	}

	elob= (double *)malloc(length2*sizeof(double));
	if(elob == NULL){
		printf("Problem allocating elob memory!\n");
	}
	
	aob1= (double *)malloc(length2*sizeof(double));
	if(aob1 == NULL){
		printf("Problem allocating aob1 memory!\n");
	}

	aob= (double *)malloc(length2*sizeof(double));
	if(aob == NULL){
		printf("Problem allocating aob memory!\n");
	}

	zob= (double *)malloc(length2*sizeof(double));
	if(zob == NULL){
		printf("Problem allocating zob memory!\n");
	}

	hob= (double *)malloc(length2*sizeof(double));
	if(hob == NULL){
		printf("Problem allocating hob memory!\n");
	}

	rob= (double *)malloc(length2*sizeof(double));
	if(rob == NULL){
		printf("Problem allocating rob memory!\n");
	}

	dob= (double *)malloc(length2*sizeof(double));
	if(dob == NULL){
		printf("Problem allocating dob memory!\n");
	}

	//printf("Here\n");
	RA[0] = RAh;
	RA[1]=	RAm;
	RAd = RAs;

	DEC[0]=DEC_deg;
	DEC[1] = DEC_min;
	DECd = DECs;

	if(DEC[0]<0){
		DEC[0] = 360+DEC[0];
	}
	slaDtf2r(RA[0],RA[1],RAd,&rc,&j);
	if(j)
		printf("Error with Dtf2r\n");
	

	slaDaf2r(DEC[0],DEC[1],DECd,&dc,&j);
	if(j)
		printf("Error with Daf2r\n");		

	//dc = 360./R2D-dc	;
	pr = 0;     /* RA proper motion (radians/year) */
	pr=0;  
	pd =0;             /* Dec proper motion (radians/year) */
	pd=0;
	px = 0;                 /* parallax (arcsec) */
	rv = 0;                   /* radial velocity (km/s) */
	/* Date (UTC, but also used as approximate TDB). */
	//   date = 53569.4;               /* 2005 July 18 09:36, as an MJD */
	/* Earth orientation parameters (from IERS Bulletin B). */
	xp = 0;            /* polar motion x (radians) */
	yp = 0;           /* polar motion y (radians) */
	dut = 0;              /* UT1-UTC (s) */
	dx = 0;              /* nutation adjustment dX */
	dy = 0;             /* nutation adjustment dY */
	/* Local circumstances (Gemini North). */
	elong =27.6853/R2D;     /* east longitude (radians) */
	phi = -25.8897/R2D;        /* latitude (radians) */
		
		//elong = -155.4690472/R2D; /* east longitude (radians) */
		//phi = 19.82380278/R2D;    /* latitude (radians) */
	
	hm = 1415.821;                  /* height a.s.l. (m) */
	tk = 296.13;                     /* temperature (K) */
	pmb = 1013.3;                  /* pressure (hPa) */
	rh = 0.45;                    /* relative humidity (0-1) */
	wl = 200.;                     /* observing wavelength (microns) */
	tlr = 0.0065;                 /* lapse rate (K/m) */
	
	//printf("Here3\n");
	time(&timev[0]);
	for(i=0;i<length;i++){
		//sleep(1);
		//time(&timev);
		if(i!=0){
			timev[i] = timev[i-1]+1;
		}
		time_ptr = gmtime(&(timev[i]));
		//printf("Here %d\n",i);
		
		slaCldj(time_ptr->tm_year+1900,time_ptr->tm_mon+1,time_ptr->tm_mday,&djutc[i],&j);
		if(j){
			printf("error with slaCldj\n");
		}
		slaDtf2d(time_ptr->tm_hour,time_ptr->tm_min,(double)time_ptr->tm_sec,&fdutc[i],&j);
		if(j){
			printf("error with slaDtf2d\n");
		}
		//printf("%5d %5d %5d %5d %5d %5d %8ld ",time_ptr->tm_year+1900,time_ptr->tm_mon+1,time_ptr->tm_mday,time_ptr->tm_hour,time_ptr->tm_min,time_ptr->tm_sec,timev[i]);
// 	//slaDtf2d(18,27,50,&fdutc,&j);
// 	if(j!=0){
// 		printf("Error in second time conversion->fdutc!\n");
// 	}	
		djutc[i]+=fdutc[i];
		mdate[i]=djutc[i];
		//printf("Here %d\n",i);
		slaMap ( rc, dc, pr, pd, px, rv, 2000.0, mdate[i], &ra1, &da1 );
		if(j){
			printf("error with slaMAP\n");
		}
		//printf("Here %d\n",i);
		slaAop ( ra1, da1, mdate[i], dut, elong, phi, hm, xp, yp,tk, pmb, rh, wl, tlr, &aob[i], &zob[i], &hob[i], &dob[i], &rob[i] );
		if(j){
			printf("error with slaAop\n");
		}
		//printf("Here %d\n",i);
		//slaPolmo ( elong, phi, xp, yp, &elong1, &phi1, &daz );
		//if(j){
		//	printf("error with slaPolmo\n");
		//}
		//printf("Here %d\n",i);
		//aob[i] += daz;
	//	printf("Observed Azimuth %+16.9f Observed Elevation %+16.9f \n",slaDranrm(aob[i])*R2D,90.0-zob[i]*R2D);
	}
	//printf("Here\n");
	for(i=0;i<length;i++){
		fittimev[i] = timev[i]-timev[0];
		//printf("fit time %ld \n",fittimev[i]); 
		timevd[i] = (double)fittimev[i];
		aob1[i] = slaDranrm(aob[i])*R2D;
		elob[i] = 90.0-zob[i]*R2D;
		//apply pointing model defined in pointing.c
		pointing(aob1[i],elob[i],&delta_az, &delta_alt);
		aob1[i] +=delta_az;
		elob[i] += delta_alt;
		message->az_commands[i]=(float)aob1[i];
		message->alt_commands[i]=(float)elob[i];
	}
	message->lcommand_vals[0] = timev[length-1];
	message->lcommand_vals[1] = timev[0];
	
	 
	quad(timevd,aob1,Yretaz,length);
	quad(timevd,elob,Yretalt,length);

	for(i=0;i<length-10;i++){
		ax=*(timevd+i);
		a_az = *(Yretaz) + ax*(*(Yretaz+1)) + ax*(*(Yretaz+2));
		//ay = *(Yret) + ax*(*(Yret+1)) + ax*(*(Yret+2) + ax*(*(Yret+3)))
		a_alt = *(Yretalt) + ax*(*(Yretalt+1)) + ax*(*(Yretalt+2));
		//printf("i = %d time %+16.9f az %+16.9f %+16.9f %+16.9f alt %+16.9f %+16.9f %+16.9f \n",i,*(timevd+i),*(aob1+i),a_az,*(aob1+i)-a_az,*(elob+i),a_alt,*(elob+i)-a_alt);	
	}
	i=0;
	ax=*(timevd+i);
	a_az = *(Yretaz) + ax*(*(Yretaz+1)) + ax*(*(Yretaz+2));
		//ay = *(Yret) + ax*(*(Yret+1)) + ax*(*(Yret+2) + ax*(*(Yret+3)))
	a_alt = *(Yretalt) + ax*(*(Yretalt+1)) + ax*(*(Yretalt+2));
	printf("i = %d time %+16.9f az %+16.9f %+16.9f %+16.9f alt %+16.9f %+16.9f %+16.9f \n",i,*(timevd+i),*(aob1+i),a_az,*(aob1+i)-a_az,*(elob+i),a_alt,*(elob+i)-a_alt);
	//printf("Yvals %16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f,%16.9f\n",*(Yretaz),*(Yretaz+1),*(Yretaz+2),*(Yretaz+3),Yretalt[0],Yretalt[1],Yretalt[2],Yretalt[3]);
//	printf("end\n");
	free(djutc);
	free(fdutc);
	free(mdate);
	free(timevd);
	free(timev);
	free(fittimev);
	free(elob);
	free(aob1);
	free(aob);
	free(zob);
	free(hob);
	free(rob);
	//printf("end2\n");
	*timeout = timev[0]+length;
	*(timeout+1) = timev[0];
	*lengthout = length;
	//;
	//printf("end2\n");
	return 0;
};


int quad(double *x, double *y,double *Yret,int length){
	double s,sx,sxx,sxxx,sxxxx,sy,sxy,sxxy,sxxxxx,sxxxy;
	double ax,ay;
	double A[3][3],Y[3];
	double adeter[10];
	int i,IW[3],JF;	
		s = 0;
		sx = 0;
		sxx = 0;
		sxxx = 0 ;
		sxxxx = 0;
		sxxxxx = 0;
		sy = 0;
		sxy = 0;
		sxxy = 0;
		sxxxy=0;
	printf("Calling Quad()\n");
	for(i=0;i<length;i++){
		ax=*(x+i);
		ay=*(y+i);
		//ay = slaDranrm(ay)*R2D;
		s=s+1;
		sx=sx+ax;
		sxx=sxx+pow(ax,2);
		sxxx=sxxx+pow(ax,3);
		sxxxx=sxxxx+pow(ax,4);
		sxxxxx = sxxxxx + pow(ax,5);
		sy=sy+ay;
		sxy=sxy + ax*ay;
		sxxy=sxxy+(pow(ax,2))*ay;
		sxxxy=sxxxy+(pow(ax,3))*ay;
	}
	A[0][0]=s;
	A[0][1]=sx;
	A[0][2]=sxx;
	//A[0][3] = sxxx;
	A[1][0]=sx;
	A[1][1]=sxx;
	A[1][2]=sxxx;
	//A[1][3] = sxxxx;
	A[2][0]=sxx;
	A[2][1]=sxxx;
	A[2][2]=sxxxx;
	//A[2][3] = sxxxxx; 
	*(Yret)=sy;
	*(Yret+1)=sxy;
	*(Yret+2)=sxxy;
	//*(Yret+3)=sxxxy;
	
	slaDmat(3,A,Yret,adeter,&JF,IW);
	for(i=0;i<=3;i++){
		//printf("%f \n",Y[i]);
	}
	//*(Yret+3)=0;
	for(i=0;i<length;i++){
		ax=*(x+i);
		ay = *(Yret) + ax*(*(Yret+1) + ax*(*(Yret+2)  )) ;
		//printf("ax %+16.9f ay %+16.9f ayin %+16.9f %+16.9f\n",ax,ay,*(y+i),ay-*(y+i));
	}
		
};


int polynomial_withpoint(double AZStart, double AZEnd,double ALTStart,double ALTEnd,time_t *timeout,int length,double *Yretalt,double *Yretaz){
  // CIpars ciprms;
	double rc, dc, pr, pd, px, rv, date, xp, yp, dut, dx, dy,
	elong, phi, hm, tk, pmb, rh, wl, tlr, eo,
	ri, di, *aob, *zob, *hob, *dob, *rob, ra, da,*elob,*aob1,ra1, da1,  elong1, phi1, daz;
	double *djutc,*fdutc,djtt,djut1,stl;
	double ax,a_az,a_alt;
	double delta_az,delta_alt;
	double azstep,altstep;
	int j,i;
	double *mdate;
	
	//int dec_deg,dec_min,dec_second;
	
	//int length=*(timeout+1)-*timeout;
	
	int length2;

	
	struct timezone tzp;
	time_t *timev;
	
	time_t *fittimev;
	double *timevd;
	struct timeval time_struct;
	struct tm *time_ptr;

	length2=length+10;
	//printf("Here\n");
	djutc= (double *)malloc(length2*sizeof(double));
	if(djutc == NULL){
		printf("Problem allocating djutc memory!\n");
	}
	
	fdutc= (double *)malloc(length2*sizeof(double));
	if(fdutc == NULL){
		printf("Problem allocating fdutc memory!\n");
	}

	


	mdate= (double *)malloc(length2*sizeof(double));
	if(mdate == NULL){
		printf("Problem allocating mdate memory!\n");
	}
	
	timevd= (double *)malloc(length2*sizeof(double));
	if(timevd == NULL){
		printf("Problem allocating timevd memory!\n");
	}

	timev= (time_t *)malloc(length2*sizeof(time_t));
	if(timev == NULL){
		printf("Problem allocating timev memory!\n");
	}

	fittimev= (time_t *)malloc(length2*sizeof(time_t));
	if(fittimev == NULL){
		printf("Problem allocating fittimev memory!\n");
	}

	elob= (double *)malloc(length2*sizeof(double));
	if(elob == NULL){
		printf("Problem allocating elob memory!\n");
	}
	
	

	aob= (double *)malloc(length2*sizeof(double));
	if(aob == NULL){
		printf("Problem allocating aob memory!\n");
	}
	azstep = (AZEnd - AZStart)/length;
	altstep = (ALTEnd - ALTStart)/length;
	time(&timev[0]);
	
	for(i=0;i<=length;i++){
	    if(i!=0){
			timev[i] = timev[i-1]+1;
		}
	    *(aob+i) = AZStart + i*azstep;
	     *(elob+i) = ALTStart + i*altstep;
	//     printf("%ld AZ %d %f %f \n",timev[i],i,*(aob+i),*(elob+i));
	}

	for(i=0;i<length;i++){
		fittimev[i] = timev[i]-timev[0];
		//printf("fit time %ld \n",fittimev[i]); 
		timevd[i] = (double)fittimev[i];
		//apply pointing model defined in pointing.c
		pointing(aob[i],elob[i],&delta_az, &delta_alt);
		aob[i] +=delta_az;
		elob[i] += delta_alt;
	}
	quad(timevd,aob,Yretaz,length);
	quad(timevd,elob,Yretalt,length);
	
	for(i=0;i<length-10;i++){
		ax=*(timevd+i);
		a_az = *(Yretaz) + ax*(*(Yretaz+1)) + ax*(*(Yretaz+2));
		//ay = *(Yret) + ax*(*(Yret+1)) + ax*(*(Yret+2) + ax*(*(Yret+3)))
		a_alt = *(Yretalt) + ax*(*(Yretalt+1)) + ax*(*(Yretalt+2));
	//	printf("i = %d time %+16.9f az %+16.9f %+16.9f %+16.9f alt %+16.9f %+16.9f %+16.9f \n",i,*(timevd+i),*(aob+i),a_az,*(aob+i)-a_az,*(elob+i),a_alt,*(elob+i)-a_alt);	
	}
	
	free(djutc);
	free(fdutc);
	free(mdate);
	free(timevd);
	free(timev);
	free(fittimev);
	free(elob);
	free(aob);
	
	//printf("end2\n");
	*timeout = timev[0]+length;
	*(timeout+1) = timev[0];
	//*lengthout = length;
	//;
	//printf("end2\n");
	return 0;
};


