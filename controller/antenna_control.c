//compile with arm-unknown-linux-gnu-gcc antenna_control.c pointing.c crc_gen.c /usr/lib/libarmcsla.a -o antenna_control -lpthread -lm
//requires an arm cross-compiler and the slalib library needs to be cross-compiled with this as well.
//Summary: This code is the SA C-BASS antenna controller program. It provides an interface between higher level control program and the low-level hardware (DACs,encoders, ADCs, digital IO). This low-level hardware is actually controller by a kernel level program (pid.ko)

//REVISION HISTORY
//----------------------------------------
//9 August 2009
//Began to upgrade the command parser from the original ascii character- the new version will use binary structures to interchange data rather than the less flexible text based scripts.
//Under SVN revision control from February 2010

//README notes.. The GPIO layout of the MAX7301 is as follows:
//CHIP 1  CON5		GPIO	Description
//  	  12		OUT	CLUTCH/BRAKE (AZ) (Drive HI to activate the Clutch or release the Brake)
//	  13		OUT	CLUTCH/BRAKE(AZ)
//	  14		OUT	CLUTCH/BRAKE(AZ)
//	  15		OUT	CLUTCH/BRAKE(ALT)
//	  16		OUT	CLUTCH/BRAKE(ALT)
//	  17		OUT	CLUTCH/BRAKE(ALT)
//	  18		N/A
//	  19		N/A
//	  20		OUT	Contactor MC1 (AZ1) (Drive HI to engage the contactor and thus turn on the servos)
//	  21		OUT	Contactor MC2 (AZ2)
//	  CON6
//	  22		OUT	Contactor MC3 (EL1)
//	  23		OUT	Contactor MC4 (EL2)
//	  24		IN	A
 //	  25		IN	THR1+
 //	  26		IN	MC1+		(Active LO)
 //	  27		IN	MB1+
 //	  28		IN	B
 //	  29		IN	THR2+
   //	  30		IN	MC2+		(Active LO)
   //	  31		IN	MB2+

//CHIP 2  CON7		GPIO	Description
//  	  12		IN	C
//	  13		IN	THR3+
//	  14		IN	MC3+		(Active LO)
//	  15		IN	MB3+
//	  16		IN	D
//	  17		IN	THR4+
//	  18		IN	MC4+		(Active LO)
//	  19		IN	MB4+
//	  20		IN	
//	  21		IN	
//	  CON8
//	  22		IN	
//	  23		IN	
//	  24		IN	
 //	  25		IN	
 //	  26		IN	
 //	  27		IN	
 //	  28		IN	
 //	  29		IN	
   //	  30		IN	
   //	  31		IN	


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
//#include "command_struct.h"
//this defines the structure used to copy data between kernel and userspace- check that this file is the same in both ALWAYS!!
#include "pid.h"
#include "slalib.h"
#include "ad5362_def.h"
#include "crc_gen.h"
#include "telescope_constants.h"
#include "pointing.h"

#define PI 3.141592653589793238462643
#define R2D (180.0/PI) /* radians to degrees */




pthread_mutex_t mutexsum;
pthread_mutex_t pid_coefficients;



//function to initiate the various structure values
int init_control_struct(struct pid_structure *control_ptr);
//function to control (via software) various limit positions etc. Uses values stored in telescope_constants.h
int soft_lim(struct pid_structure *control_struct,unsigned int limit_hi,unsigned int limit_lo,unsigned int limit_slow_zone_hi,unsigned int limit_slow_zone_lo,int slow_speed_hi, int slow_speed_lo,unsigned int encoder, unsigned int *command,long *pid,long *pid1,long *pid2);
//crc values for the ad5362 DAC which provides command voltages to the servo systems
unsigned int get_crc(unsigned int);
unsigned int ad5362_crc_pack(unsigned int,unsigned int,unsigned int);
unsigned int ad5362_crc_pack1(unsigned int,unsigned int,unsigned int);

//copy the pid structure from kernel space into the userspace domain
int get_structure(int pid_handle,struct pid_structure *userspace);

int control_loop(int pid_handle,struct pid_structure *userspace);
//function to calculate the position PID loop
int update_pid(char select, unsigned int command, unsigned int prev_encoder,unsigned int encoder,double p_gain, double i_gain, double d_gain,double kfgain,double vel_com,long time_diff,int MOTOR_SLACK_PLUS, int MOTOR_SLACK_MINUS,int MAX_MOTOR_SPEED_PLUS,int MAX_MOTOR_SPEED_MINUS,int MAX_I_WINDUP,int MIN_I_WINDUP, long *current_error_ptr, long *current_i_ptr,long *pid_return_ptr);
//same as above but uses the azimuth values rather than the raw encoder values as the input to the PID loop
int update_pid_double(char select, double command, double prev_encoder,double encoder,double p_gain, double i_gain, double d_gain,double kfgain,double vel_com,long time_diff,int MOTOR_SLACK_PLUS, int MOTOR_SLACK_MINUS,int MAX_MOTOR_SPEED_PLUS,int MAX_MOTOR_SPEED_MINUS,int MAX_I_WINDUP,int MIN_I_WINDUP, double *current_error_ptr, double *current_i_ptr,long *pid_return_ptr);


int update_pid2_double(char select, double command, double prev_feedback,double feedback,double p_gain, double i_gain, double d_gain,double feedback_gain, long time_diff,int MAX_MOTOR_SPEED_PLUS,int MAX_MOTOR_SPEED_MINUS,int MAX_I_WINDUP,int MIN_I_WINDUP, long *current_error_ptr, double *current_i_ptr,long *pid_return_ptr);
//function to limit the acceleration of the antenna
void ramp(long *pid_return_old,long *az_pid1,long *az_pid2,long *alt_pid1,long *alt_pid2,int MAX_AZ,int MIN_AZ,int MAX_ALT,int MIN_ALT,int AZ_INTERVAL,int ALT_INTERVAL,int MAX_AZ_INTERVAL,int MAX_ALT_INTERVAL);

//function to limit the acceleration of the antenna using the tachometer feedback
void tacho_ramp(long *tacho_old,long *tacho_new,long *pid,int MAX_AZ,int MIN_AZ,int MAX_ALT,int MIN_ALT);

//function to sort a vector of double values
void sort(double *unsorted,double *sorted,int length);

//function to convert from a RA/DEC value to the azimuth and altitude values which are the antenna native coordinates.
void Get_angles(time_t time,double right_ascension_mean,double declination_mean,double Epoch,double *altitude,double *azimuth,double *amprms_ptr,double *aoprms_ptr);

//function to provide a pointing correction if neccessary. The idea is actually to provide this at a higher level
int pointing_model_correction(double Azimuth,double Altitude, unsigned int *AZ_ENCODER, unsigned int *ALT_ENCODER);

//function to load up necessary parameters to allow the fast conversion from RA/DEC to ALT/AZ. Needs to be updated every couple of hours
void get_parameters(double *amprms, double *aoprms);

//function to implement an anti-backlash when tracking. This backlash would be caused by wind. The best way to avoid it is to have one of the motors driving slightly in opposition so as to keep the gear teeth locked on both sides
int backlash(unsigned int command,unsigned int encoder,double velocity,unsigned int backlash_motor_offset,unsigned int backlash_position_offset,unsigned int maintain_position_offset1,unsigned int maintain_position_offset2,long *pid1,long *pid2);
//function to handle the dynamic PID coefficient 
//void pidadaptive_update(unsigned int encoder, unsigned int command, struct pid_coefficient_structure *pid_struct,double *pcoeff, double *icoeff, double *dcoeff,double *kfcoeff,double *vfcoeff,long *MOTOR_PLUS, long *MOTOR_MINUS);
void pidadaptive_update(unsigned int type,unsigned int encoder, unsigned int command, struct pid_coefficient_structure *pid_struct,double *pcoeff, double *icoeff, double *dcoeff,double *kfcoeff,double *vfcoeff,long *MOTOR_PLUS, long *MOTOR_MINUS,double *icoeff_vel, double *dcoeff_vel,double *kfcoeff_vel);

//function used to control the contactors (i.e contactors on or contactors off)
unsigned int contactors(unsigned int command);
//similarly for the clutch and brakes
unsigned int clutchbrake(unsigned int command);
//initiates the max7301 GPIO controller
unsigned int max7301_init(void);

//the pid loop for velocity
void velocity_pid(long vel_tacho, double vel_command, double Kfa, double Pd, double Id, double Dd, long Mx_I, long Mn_I, int MAX_OUT,int MIN_OUT,double *current_i_ptr,long *velpid_out, long *pid_return_ptr);


//the status query for checking the breakers etc
unsigned int status_query(unsigned int *return_vec);

//declare the overall PID structure used to pass information between different threads.
struct pid_structure control;

int fd,sd_command;
struct sockaddr_in cliAddr1, remoteServAddr;
//the adaptive control structures for the four motors

//declare the dynamic pid coefficient structures
struct new_pid_coefficient_structure azimuth_pid1,azimuth_pid2,altitude_pid1,altitude_pid2;
struct pid_structure tcp_control_structs[5];
struct command_struct command;


//struct command_position_struct command;

//velocity PID loop


void *command_thread(void *arg){
	//this thread sits on a incoming socket and listens for incoming commands- the commands are held in a common C struct which allows a simple binary transfer of the data to be done on both sides of the connection
	int sd,rc,n, cliLen,read_ret;
	int i,ij;
	double RAd,DECd,AZd,ALTd;
	double azstart,azend,elstart,elend,time;
	unsigned int AZ_ENCODER,ALT_ENCODER;
	 int childfd;
	 struct sockaddr_in cliAddr, servAddr;
	 int optval;
	 struct hostent *hostp; /* client host info */
	 char *hostaddrp;
	  struct new_message_parsing_struct tcp_message;
	  int coordinate_type;
	struct pid_structure controlc;
	//define the type of communication (tcp) that will be used by the socket
	sd=socket(AF_INET, SOCK_STREAM, 0);
	 if(sd<0) {
     	 printf(" cannot open socket \n");
     	 exit(1);
  	 }

	optval = 1;
	//set the socket options
  	setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, 
	     (const void *)&optval , sizeof(int));

	  /*
   * build the server's Internet address
   */
   //make sure that everything is clear in memory.
  bzero((char *) &servAddr, sizeof(servAddr));


	//begin setting connection options
	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servAddr.sin_port = htons(LOCAL_SERVER_PORT_STRUCT);
	

	 //bind to the socket and prepare to listen for incoming communications
	rc = bind (sd, (struct sockaddr *) &servAddr,sizeof(servAddr));
	if(rc<0) {
	printf(": cannot bind port number %d \n",LOCAL_SERVER_PORT_STRUCT);
	exit(1);
	}
	
	printf(": waiting for data on port UDP %u\n",
	LOCAL_SERVER_PORT_STRUCT);

	while(1){
	//listen!!!
	  if (listen(sd, 5) < 0) /* allow 5 requests to queue up */ 
	      error("ERROR on listen");

	//do necessary housekeeping to set up the communications
	cliLen = sizeof(cliAddr);

	 childfd = accept(sd, (struct sockaddr *) &cliAddr, &cliLen);
   	 if (childfd < 0) 
     	 error("ERROR on accept");

	 hostp = gethostbyaddr((const char *)&cliAddr.sin_addr.s_addr, 
			  sizeof(cliAddr.sin_addr.s_addr), AF_INET);
  	  if (hostp == NULL)
     	 error("ERROR on gethostbyaddr");
   	 hostaddrp = inet_ntoa(cliAddr.sin_addr);
   	 if (hostaddrp == NULL)
   	   error("ERROR on inet_ntoa\n");
   	 printf("server established connection with %s (%s)\n", 
	   hostp->h_name, hostaddrp);

	  //first clear the incoming structure and then receive the binary data into it!
	  bzero(&tcp_message, sizeof(tcp_message));
   	 n = recv(childfd, &tcp_message, sizeof(tcp_message),0);
   	 if (n < 0) 
     	 error("ERROR reading from socket");
   	 printf("server received %d bytes: %d", n, tcp_message.coordinate_type);

	//n = recvfrom(sd, msg, sizeof(msg), 0,	(struct sockaddr *) &cliAddr, &cliLen);
	
	printf("Received Structure\n");
	 n = send(childfd, &tcp_message, sizeof(tcp_message),0);
    	if (n < 0) 
     	error("ERROR writing to socket");

	ioctl(fd,DEV_IOCTL_READ_CONTROL_STRUCTURE, &controlc);
	coordinate_type = tcp_message.coordinate_type;
	printf("Coordinate Type %d\n",coordinate_type);
	switch(coordinate_type){
	//now we simply go through- decide the type of command and take appropriate action making sure to use mutexex to prevent conflicting variable updates
	case HORIZONTAL:
	//an azimuth/elevation command is given
		printf("yippee HORIZONTAL Updated\n");
		//commands[0] = atof(command_string1);
		//commands[1] = atof(command_string2);
		//coordinate_type_extra = HORIZONTAL;
		AZd = tcp_message.fcommand_vals[0];
		ALTd = tcp_message.fcommand_vals[1];
		printf("AZ %f ALT %f\n",AZd,ALTd);
		pthread_mutex_lock (&mutexsum);
			//read_ret = read(fd,&control,sizeof(control));
			
			control.AZ_double=AZd;
			control.ALT_double=ALTd;
			control.coordinate_command_type=coordinate_type;
			//read_ret = write(fd,&control,sizeof(control));
		pthread_mutex_unlock (&mutexsum);
		break;
	case EQUATORIAL:
	//a RA/DEC command is given- this should be used sparingly as the onboard RA/DEC->ALT/AZ is slow due to the lack of floating point unit
		printf("yippee EQUATORIAL\n");
		//commands[0] = atof(command_string1);
		//commands[1] = atof(command_string2);
		//coordinate_type_extra = EQUATORIAL;
		RAd = tcp_message.fcommand_vals[0];
		DECd= tcp_message.fcommand_vals[1];
		printf("RA %f DEC %f\n",RAd,DECd);
		pthread_mutex_lock (&mutexsum);
			//read_ret = read(fd,&control,sizeof(control));
			control.RA_double=RAd;
			control.DEC_double=DECd;
			control.coordinate_command_type=coordinate_type;
			//read_ret = write(fd,&control,sizeof(control));	
		pthread_mutex_unlock (&mutexsum);
		break;

		
	
	case EQUATORIAL2:
	//This uses a set of second order polynomial coefficients to update the required ALT/AZ- these are calculated by the high level control PC and uploaded onto the ARM to allow quicker calculation of the appropriate ALT/AZ command angles
		printf("yippee EQUATORIAL2\n");
		//int_commands[0] = atoi(command_string1); //Tend
		//int_commands[1] = atoi(command_string2); //T0
		//coordinate_type_extra = EQUATORIAL; 	
		//int_commands[2] = atoi(command_string3);//Length
		//commands[0] = atof(command_string4); //a1
		//commands[1] = atof(command_string5); //a2
		//commands[2] = atof(command_string6); //a3
		//commands[3] = atof(command_string7); //a4
		//commands[4] = atof(command_string8); //b1
		//commands[5] = atof(command_string9); //b2
		//commands[6] = atof(command_string10); //b3
		//commands[7] = atof(command_string11); //b4
		
		printf("Tend %d T0 %d Length %d\n",tcp_message.lcommand_vals[0],tcp_message.lcommand_vals[1],tcp_message.command_vals[0]);
		printf("a0 %lf a1 %lf a2 %lf a3 %lf b1 %lf b2 %lf b3 %lf b4 %lf\n",tcp_message.fcommand_vals[0],tcp_message.fcommand_vals[1],tcp_message.fcommand_vals[2],tcp_message.fcommand_vals[3],tcp_message.fcommand_vals[4],tcp_message.fcommand_vals[5],tcp_message.fcommand_vals[6],tcp_message.fcommand_vals[7]);
		
		
		
		pthread_mutex_lock (&mutexsum);
			//read_ret = read(fd,&control,sizeof(control));
			control.a[0]=tcp_message.fcommand_vals[0];
			control.a[1]=tcp_message.fcommand_vals[1];
			control.a[2]=tcp_message.fcommand_vals[2];
			control.a[3]=tcp_message.fcommand_vals[3];
			control.b[0]=tcp_message.fcommand_vals[4];
			control.b[1]=tcp_message.fcommand_vals[5];
			control.b[2]=tcp_message.fcommand_vals[6];
			control.b[3]=tcp_message.fcommand_vals[7];
			//control.b[0]=tcp_message.dcommand_vals[8];
			control.eq2_time_end = tcp_message.lcommand_vals[0];
			control.eq2_time_begin = tcp_message.lcommand_vals[1];
			control.coordinate_command_type=coordinate_type;
		pthread_mutex_unlock (&mutexsum);
		time = tcp_message.command_vals[0];
		azstart = control.a[0];
		azend = control.a[0] + time*(control.a[1] + time*control.a[2]);
		elstart = control.b[0];
		elend = control.b[0] + time*(control.b[1] + time*control.b[2]);
		//calculate the speed to display for user
		printf("Azimuth Start : %5.2f End : %5.2f Distance : %5.2f Velocity : %5.2f \nElevation Start : %5.2f End : %5.2f Distance : %5.2f Velocity : %5.2f\n",azstart,azend,azend-azstart,(azend-azstart)/time,elstart,elend,elend-elstart,(elend-elstart)/time);
		break;
	
	case HORIZONTAL_LIST:
	  	printf("Horizontal Time Series control positions\n");
		pthread_mutex_lock (&mutexsum);
		  control.eq2_time_end = tcp_message.lcommand_vals[0];
		  control.eq2_time_begin = tcp_message.lcommand_vals[1];
		  control.coordinate_command_type=coordinate_type;
		pthread_mutex_unlock (&mutexsum);
		for(i=0; i<=100; i++){
		  command.comaz[i] = tcp_message.az_commands[i];
		  command.comaz[i]=100-command.comaz[i];
		  command.comalt[i] = tcp_message.alt_commands[i];
		  printf("i = %d Az %f Alt %f %ld %ld\n",i,command.comaz[i],command.comalt[i],control.eq2_time_end,control.eq2_time_begin);
		}
	break;
		
	case ENCODER_COORDS:
	//very low level to allow driving antenna to a specific angle encoder positions (i.e uses 16bit number)
		printf("Encoder Coordinate\n");
		//commands[0] = atof(command_string1);
		//commands[1] = atof(command_string2);
		//coordinate_type_extra = ENCODER_COORDS;
		
		AZ_ENCODER = tcp_message.command_vals[0];
		ALT_ENCODER = tcp_message.command_vals[1];
		pthread_mutex_lock (&mutexsum);
			//read_ret = read(fd,&control,sizeof(control));
			control.az_command_long=AZ_ENCODER;
			control.alt_command_long=ALT_ENCODER;
			//azalt2encoder(azimuth_val,control.delta_az, altitude_val,control.delta_alt, &control.az_command_long, &control.alt_command_long);
			
			//control.AZ_double=AZd;
			//control.ALT_double=ALTd;
			control.coordinate_command_type=coordinate_type;
			//read_ret = write(fd,&control,sizeof(control));
		pthread_mutex_unlock (&mutexsum);
	break;

	case AZ1_PID_COEFFICIENTS_ADAPTIVE:
	//updated the AZ1 pid coefficients with a table of pid values which can be changed at different error values

	      pthread_mutex_lock(&pid_coefficients);
		printf("Adaptive AZ1 PID coefficient update\n");
		printf("%f %f %f %d\n",tcp_message.pid_vals.i[0],tcp_message.pid_vals.d[0],azimuth_pid1.table_length);
		azimuth_pid1 = tcp_message.pid_vals;
		printf("AZ1 Length %d\n",azimuth_pid1.table_length);
		azimuth_pid1.adaptive=1;
		i=0;
		while(i<azimuth_pid1.table_length){
		printf("Az %d %f %f %f kf %f vf %f %d %d %f %f %f\n",azimuth_pid1.position_error[i],azimuth_pid1.p[i],azimuth_pid1.i[i],azimuth_pid1.d[i],azimuth_pid1.kf[i],azimuth_pid1.vf[i],azimuth_pid1.motor_plus[i],azimuth_pid1.motor_minus[i],azimuth_pid1.p_2[i],azimuth_pid1.i_2[i],azimuth_pid1.d_2[i]);
		i++;
		}
	      pthread_mutex_unlock(&pid_coefficients);
		
	break;
	
	case AZ2_PID_COEFFICIENTS_ADAPTIVE:
	      pthread_mutex_lock(&pid_coefficients);
		printf("Adaptive AZ2 PID coefficient update\n");
		printf("%f %f %f %d\n",tcp_message.pid_vals.i[0],tcp_message.pid_vals.d[0],tcp_message.pid_vals.table_length);
		azimuth_pid2 = tcp_message.pid_vals;
		azimuth_pid2.adaptive=1;
		i=0;
		while(i<azimuth_pid2.table_length){
		printf("Az %d %f %f %f kf %f vf %f %d %d %f %f %f\n",azimuth_pid2.position_error[i],azimuth_pid2.p[i],azimuth_pid2.i[i],azimuth_pid2.d[i],azimuth_pid2.kf[i],azimuth_pid2.vf[i],azimuth_pid2.motor_plus[i],azimuth_pid2.motor_minus[i],azimuth_pid2.p_2[i],azimuth_pid2.i_2[i],azimuth_pid2.d_2[i]);
		i++;
		}
	      pthread_mutex_unlock(&pid_coefficients);
	      
		
	break;
	
	case ALT1_PID_COEFFICIENTS_ADAPTIVE:
	      pthread_mutex_lock(&pid_coefficients);
		printf("Adaptive ALT1 PID coefficient update\n");
		printf("%f %f %f %d\n",tcp_message.pid_vals.i[0],tcp_message.pid_vals.d[0],tcp_message.pid_vals.table_length);
		altitude_pid1 = tcp_message.pid_vals;
		altitude_pid1.adaptive=1;
		i=0;
		while(i<altitude_pid1.table_length){
 		printf("Alt1pid_ad %d %f %f %f kf %f vf %f %d %d %f %f %f\n",altitude_pid1.position_error[i],altitude_pid1.p[i],altitude_pid1.i[i],altitude_pid1.d[i],altitude_pid1.kf[i],altitude_pid1.vf[i],altitude_pid1.motor_plus[i],altitude_pid1.motor_minus[i],altitude_pid1.p_2[i],altitude_pid1.i_2[i],altitude_pid1.d_2[i]);
		i++;
		}
	      pthread_mutex_unlock(&pid_coefficients);
		
	break;
	
	case ALT2_PID_COEFFICIENTS_ADAPTIVE:
	      pthread_mutex_lock(&pid_coefficients);
		printf("Adaptive ALT2 PID coefficient update\n");
		printf("%f %f %f %d\n",tcp_message.pid_vals.i[0],tcp_message.pid_vals.d[0],tcp_message.pid_vals.table_length);
		altitude_pid2 = tcp_message.pid_vals;
		altitude_pid2.adaptive=1;
		i=0;
		while(i<altitude_pid2.table_length){
		printf("Alt2 %d %f %f %f kf %f vf %f %d %d %f %f %f\n",altitude_pid2.position_error[i],altitude_pid2.p[i],altitude_pid2.i[i],altitude_pid2.d[i],altitude_pid2.kf[i],altitude_pid2.vf[i],altitude_pid2.motor_plus[i],altitude_pid2.motor_minus[i],altitude_pid2.p_2[i],altitude_pid2.i_2[i],altitude_pid2.d_2[i]);
		i++;
		}
	      pthread_mutex_unlock(&pid_coefficients);
	      
		
	break;


	case AZ1_PID_COEFFICIENTS:
	//non-dynamic pid coefficients
		printf("PID AZ 1 Command Encoder Coordinate\n");
		//commands[0] = atof(command_string1);
		//commands[1] = atof(command_string2);
		//coordinate_type_extra = AZ1_PID_COEFFICIENTS;
		pthread_mutex_lock (&mutexsum);
		  //read_ret = read(fd,&control,sizeof(control));
		  control.pcoeffs[0] = tcp_message.fcommand_vals[0];
		  control.icoeffs[0] = tcp_message.fcommand_vals[1];
		  control.dcoeffs[0] = tcp_message.fcommand_vals[2];
		  control.motor_plus[0] = tcp_message.command_vals[0];
		  control.motor_minus[0] = tcp_message.command_vals[1];
		  control.update =1;
		  pthread_mutex_lock (&pid_coefficients);
		  azimuth_pid1.adaptive=0;
		pthread_mutex_unlock (&pid_coefficients);
	//	control.coordinate_command_type=coordinate_type;
		printf("P %f I %f D %f M+ %ld M- %ld\n",control.pcoeffs[0],control.icoeffs[0],control.dcoeffs[0],control.motor_plus[0],control.motor_minus[0]);
		//read_ret = write(fd,&control,sizeof(control));
		pthread_mutex_unlock (&mutexsum);
	break;
	
	case AZ2_PID_COEFFICIENTS:
		printf("PID AZ 2 Command Encoder Coordinate\n");
		//commands[0] = atof(command_string1);
		//commands[1] = atof(command_string2);
		//coordinate_type_extra = AZ2_PID_COEFFICIENTS;
		pthread_mutex_lock (&mutexsum);
	  //	read_ret = read(fd,&control,sizeof(control));
		  control.pcoeffs[1] = tcp_message.fcommand_vals[0];
		  control.icoeffs[1] = tcp_message.fcommand_vals[1];
		  control.dcoeffs[1] = tcp_message.fcommand_vals[2];
		  control.motor_plus[1] = tcp_message.command_vals[0];
		  control.motor_minus[1] = tcp_message.command_vals[1];
	  //	control.coordinate_command_type=coordinate_type;
		  control.update =1;
		  pthread_mutex_lock (&pid_coefficients);
		    azimuth_pid2.adaptive=0;
		  pthread_mutex_unlock (&pid_coefficients);
		  printf("P %f I %f D %f M+ %ld M- %ld\n",control.pcoeffs[1],control.icoeffs[1],control.dcoeffs[1],control.motor_plus[1],control.motor_minus[1]);
		//read_ret = write(fd,&control,sizeof(control));
		//read_ret = write(fd,&control,sizeof(control));
		pthread_mutex_unlock (&mutexsum);
	break;

	case ALT1_PID_COEFFICIENTS:
		printf("PID ALT 1 Command Encoder Coordinate\n");
		//commands[0] = atof(command_string1);
		//commands[1] = atof(command_string2);
		//coordinate_type_extra = ALT1_PID_COEFFICIENTS;
		pthread_mutex_lock (&mutexsum);
		//read_ret = read(fd,&control,sizeof(control));
		control.pcoeffs[2] = tcp_message.fcommand_vals[0];
		control.icoeffs[2] = tcp_message.fcommand_vals[1];
		control.dcoeffs[2] = tcp_message.fcommand_vals[2];
		control.motor_plus[2] = tcp_message.command_vals[0];
		control.motor_minus[2] = tcp_message.command_vals[1];
		control.update =1;
		pthread_mutex_lock (&pid_coefficients);
		altitude_pid1.adaptive=0;
		pthread_mutex_unlock (&pid_coefficients);
	//	control.coordinate_command_type=coordinate_type;
		printf("P %f I %f D %f M+ %ld M- %ld\n",control.pcoeffs[2],control.icoeffs[2],control.dcoeffs[2],control.motor_plus[2],control.motor_minus[2]);
		//read_ret = write(fd,&control,sizeof(control));
		//read_ret = write(fd,&control,sizeof(control));
		pthread_mutex_unlock (&mutexsum);
	break;

	case ALT2_PID_COEFFICIENTS:
		printf("PID ALT 2 Command Encoder Coordinate\n");
		//commands[0] = atof(command_string1);
		//commands[1] = atof(command_string2);
		//coordinate_type_extra = ALT2_PID_COEFFICIENTS;
		pthread_mutex_lock (&mutexsum);
		//read_ret = read(fd,&control,sizeof(control));
		control.pcoeffs[3] = tcp_message.fcommand_vals[0];
		control.icoeffs[3] = tcp_message.fcommand_vals[1];
		control.dcoeffs[3] = tcp_message.fcommand_vals[2];
		control.motor_plus[3] = tcp_message.command_vals[0];
		control.motor_minus[3] = tcp_message.command_vals[1];
		control.update =1;
		pthread_mutex_lock (&pid_coefficients);
		altitude_pid2.adaptive=1;
		pthread_mutex_unlock (&pid_coefficients);
	//	control.coordinate_command_type=coordinate_type;
		printf("P %f I %f D %f M+ %ld M- %ld\n",control.pcoeffs[3],control.icoeffs[3],control.dcoeffs[3],control.motor_plus[3],control.motor_minus[3]);
		//read_ret = write(fd,&control,sizeof(control));
		//read_ret = write(fd,&control,sizeof(control));
		pthread_mutex_unlock (&mutexsum);
	break;
	
	case IOCTL_COMMANDS:
	//handles IO control commands to various hardware devices- these are defined in pid.h
		printf("IOCTL COMMAND %04x\n");
	//	pthread_mutex_lock (&mutexsum);
//		read_ret = read(fd,&control,sizeof(control));
		//read_ret = read(fd,&control,sizeof(control));
		//printf("Change DAC State\n");
		//printf("Control.DAC_Output = %d\n",control.DAC_Output);
		//control.DAC_Output = atoi(command_string1);
		
		ioctl(fd,tcp_message.command_vals[0], &controlc);
	//	printf("Control.DAC_Output = %d\n",control.DAC_Output);
		//read_ret = read(fd,&control,sizeof(control));
	//	printf("Control.DAC_Output = %d\n",control.DAC_Output);
	//	pthread_mutex_unlock (&mutexsum);
	break;

	case SAFE_DRIVE_LIMITS:
		printf("Change the Safety Zone Drive Limits\n");
		pthread_mutex_lock (&mutexsum);
		control.limits[0] = tcp_message.command_vals[0];
		control.limits[1] =tcp_message.command_vals[1];
		control.limits[2] = tcp_message.command_vals[2];
		control.limits[3] = tcp_message.command_vals[3];
		control.update =1;
		//control.coordinate_command_type=coordinate_type;
		pthread_mutex_unlock (&mutexsum);
	break;

	case CONTACTORS:
		//int_commands[0] = atoi(command_string1);
		printf("WORK WITH CONTACTORS %d\n",tcp_message.command_vals[0]);
		
		contactors(tcp_message.command_vals[0]);
	break;

	case CLUTCHBRAKE:
		//int_commands[0] = atoi(command_string1);
		printf("WORK WITH CLUTCHBRAKE %d\n",tcp_message.command_vals[0]);
		printf("Disabling DAC Since working with Contactors- must be restarted Manually\n");
		ioctl(fd,DEV_IOCTL_DISABLE_DAC, &controlc);
		clutchbrake(tcp_message.command_vals[0]);
	break;

	default:
	  printf("Failed to get correct case here\n");
	break;
	
	}


	
	
//	pthread_mutex_lock (&mutexsum);

	pthread_mutex_lock (&mutexsum);

		

	
	
	
//	read_ret = write(fd,&controlc,sizeof(controlc));
		

	
	printf("regoing through thread control.coordinate_command_type %d\n",control.coordinate_command_type);
	printf("RA %f DEC %f AZ %f ALT %f %d %d\n",RAd,DECd,AZd,ALTd, control.az_command_long,control.alt_command_long); 
	pthread_mutex_unlock (&mutexsum);
	}


	close(childfd);
	
	




};

void *coordinate_thread (void *arg){
	//this thread handles the updates of command angles to the main loop. It will 'tick' about once a second to give a new angle if required.
	double right_ascension,declination;
	double amprms[21],*amprms_ptr,aoprms[21],*aoprms_ptr;
	double epoch;
	double avevel_az[10],avevel_alt[10],avevel_azsorted[10],avevel_altsorted[10];
	double vel_az,vel_alt;
	double altitude_val,azimuth_val;
	unsigned int az_test[5];
	unsigned int az_temp;
	long az_test2[5];
	long tmp,tmp2;
	int i,j;
	int average_counter=0;
	unsigned int read_ret;
	unsigned int command_alt,command_az;
	struct timeval time_struct;
	struct timezone tzp;
	struct pid_structure controlc;
	time_t eq2_time_end,eq2_time_beg,current_time;
	int t;
	double tdouble,tdouble1,td;
	unsigned int status_vec[20];



	

//	ioctl(fd,DEV_IOCTL_READ_CONTROL_STRUCTURE, &control_coordinate);
	
//	ioctl(fd,DEV_IOCTL_READ_CONTROL_STRUCTURE, &controlc);
	
	status_query(&control.status_vec);
	pthread_mutex_lock (&mutexsum);
	ioctl(fd,DEV_IOCTL_READ_CONTROL_STRUCTURE, &controlc);
	control.az_command_long = controlc.az_encoder_long;
	control.alt_command_long = controlc.alt_encoder_long;
	encoder2azalt(control.az_command_long,0.,control.alt_command_long,0.,&control.azimuth_command_double, &control.altitude_command_double);
	azimuth_angle_no_sort(control.azimuth_command_double,controlc.az_encoder_long,AZIMUTH_ZERO,AZIMUTH_SAFETY_LO,AZIMUTH_SAFETY_HI,&control.azimuth_command_double);
	
	control.coordinate_command_type =ENCODER_COORDS;
	control.az_command_long = controlc.az_encoder_long;
	control.alt_command_long = controlc.alt_encoder_long;
	printf("Command Angles AZ %d ALT %d\n",control.az_command_long,control.alt_command_long);
	pthread_mutex_unlock (&mutexsum);


	printf("HERE1");

	epoch=2000.;
	get_parameters(amprms, aoprms);

	while(1){
	ioctl(fd,DEV_IOCTL_READ_CONTROL_STRUCTURE, &controlc);
	usleep(10000000);
	status_query(&control.status_vec);
	
	//printf("Status Query : \nChip1 12-19 %04x\nChip1 20-27 %04x\nChip1 P24-31 %04x\nChip2 P12-19 %04x\nChip2 P20-27 %04x\nChip2 P28-31 %04x\n",*(status_vec),*(status_vec+2),*(status_vec+4),*(status_vec+3),*(status_vec+5),*(status_vec+6));
//	ioctl(fd,DEV_IOCTL_READ_CONTROL_STRUCTURE, &controlc);
//	control_coordinate = control_writer;
	
//	ioctl(fd,DEV_IOCTL_READ_CONTROL_STRUCTURE, &controlc);   
	
	//printf("get angles %d\n",control.coordinate_command_type);

	if (control.coordinate_command_type==HORIZONTAL){
		pthread_mutex_lock (&mutexsum);
			printf("HZ in coordinate thread\n");
			azimuth_val = control.AZ_double;
			altitude_val = control.ALT_double;
			control.azimuth_command_double = azimuth_val;
			control.altitude_command_double=altitude_val;
			control.delta_az =0.;
			control.delta_alt=0.;
			control.vel_of_az = 0.;
			control.vel_of_alt = 0.;
			//pointing(azimuth_val,altitude_val,&control.delta_az,&control.delta_alt);
			//pointing_model_correction(azimuth_val,altitude_val, &control.az_command_long, unsigned int *ALT_ENCODER);
			
		//	control.az_command_long= (unsigned int)(65535.*(azimuth_val/360.));
		//	control.az_command_long+=AZIMUTH_ZERO;
			//azimuth_val+=control.delta_az;
			//altitude_val +=control.delta_alt;
			//azalt2encoder(azimuth_val, altitude_val, &control.az_command_long, &control.alt_command_long);
			azalt2encoder(azimuth_val,control.delta_az, altitude_val,control.delta_alt, &control.az_command_long, &control.alt_command_long);
			//printf("HORIZONTAL get angles AZ %f %d ALT %f %d\n",control.AZ_double,control.az_command_long,control.ALT_double,control.alt_command_long);
			sort_azimuth(controlc.az_encoder_long,control.az_command_long,AZIMUTH_SAFETY_HI,AZIMUTH_SAFETY_LO,&control.az_command_long);
			azimuth_angle(control.azimuth_command_double,controlc.az_encoder_long,AZIMUTH_ZERO,AZIMUTH_SAFETY_LO,AZIMUTH_SAFETY_HI,&control.azimuth_command_double);
			
			//sort_azimuth_double(controlc.az_encoder_long,control.azimuth_command_double,170., -370.,&control.azimuth_command_double);
	
		//	control.alt_command_long = (unsigned int)(65535.*(altitude_val/360.));
		//	control.alt_command_long+=ELEVATION_ZERO;
		//	control.az_command_long = command_az;
		//	control.alt_command_long = command_alt;
		pthread_mutex_unlock (&mutexsum);
	//	printf("HORIZONTAL get angles AZ %f %d ALT %f %d\n",control.AZ_double,control.az_command_long,control.ALT_double,control.alt_command_long);
	}
	else if(control.coordinate_command_type==EQUATORIAL){
		pthread_mutex_lock (&mutexsum);
			right_ascension = control.RA_double;
			declination = control.DEC_double;
			gettimeofday(&(time_struct),&tzp);
		pthread_mutex_unlock (&mutexsum);
			Get_angles(time_struct.tv_sec,right_ascension,declination,epoch,&altitude_val,&azimuth_val,amprms,aoprms);
			printf("Equatorial Az %f ALT %f \n",azimuth_val,altitude_val);
			control.azimuth_command_double = azimuth_val;
			control.altitude_command_double=altitude_val;
			control.delta_az =0.;
			control.delta_alt=0.;
			//pointing(azimuth_val,altitude_val,&control.delta_az,&control.delta_alt);
			//azimuth_val+=control.delta_az;
			//altitude_val +=control.delta_alt;
			//azimuth_val = 110.5;
			//altitude_val = 56.6;
		pthread_mutex_lock (&mutexsum);
			//control.az_command_long= (unsigned int)(65535.*(azimuth_val/360.));
			//control.az_command_long+=AZIMUTH_ZERO;
			azalt2encoder(azimuth_val,control.delta_az, altitude_val,control.delta_alt, &control.az_command_long, &control.alt_command_long);
			//azalt2encoder(azimuth_val, altitude_val, &control.az_command_long, &control.alt_command_long);
			sort_azimuth(controlc.az_encoder_long,control.az_command_long,AZIMUTH_SAFETY_HI, AZIMUTH_SAFETY_LO,&control.az_command_long);
			sort_azimuth_double(controlc.az_encoder_long,control.azimuth_command_double,170., -370.,&control.azimuth_command_double);
			//control.alt_command_long = (unsigned int)(65535.*(altitude_val/360.));
			//control.alt_command_long+=ELEVATION_ZERO;
			
		pthread_mutex_unlock (&mutexsum);
		//printf("EQUATORIAL RA %f DEC %f AZ %f %d ALT %f %d\n",right_ascension,declination,azimuth_val,control.az_command_long,altitude_val,control.alt_command_long);
	}

	else if(control.coordinate_command_type==ENCODER_COORDS){
		//printf("Angle encoder coordinates\n");
		pthread_mutex_lock (&mutexsum);
		control.vel_of_az = 0.;
		control.vel_of_alt = 0.;
		//control.az_command_long = 
		encoder2azalt(control.az_command_long,0.,control.alt_command_long,0.,&control.azimuth_command_double, &control.altitude_command_double);	
		printf("control az command11 %f %d\n",control.azimuth_command_double,control.az_command_long);
		azimuth_angle_no_sort(control.azimuth_command_double,controlc.az_encoder_long,AZIMUTH_ZERO,AZIMUTH_SAFETY_LO,AZIMUTH_SAFETY_HI,&control.azimuth_command_double);
		//printf("control az command22 %f\n",control.azimuth_command_double);
		//encoder2azaltprime(unsigned int AZ_Encoder, double delta_AZ, unsigned int ALT_Encoder,double delta_ALT,double *AZ, double *ALT)
		pthread_mutex_unlock (&mutexsum);
		//printf("ENCODER Coordinates RA %f DEC %f AZ %f %d ALT %f %d\n",right_ascension,declination,azimuth_val,control.az_command_long,altitude_val,control.alt_command_long);
	}

	//else if(control.coordinate_command_type==EQUATORIAL2){
	else if(control.coordinate_command_type==500){
		//printf("EQ2n\n");
		pthread_mutex_lock (&mutexsum);
			control.delta_az =0.;
			control.delta_alt=0.;
			gettimeofday(&(time_struct),&tzp);
			tdouble1=time_struct.tv_sec;
			tdouble = time_struct.tv_usec;
			time(&current_time);
			if(current_time<=control.eq2_time_end){
				td = tdouble1 + tdouble/1000000.;
				tdouble = (control.eq2_time_begin);
				td = td - tdouble;
				//t = current_time - control.eq2_time_begin;
				//average_counter++;
 				avevel_az[6] = avevel_az[5];
 				avevel_az[5] = avevel_az[4];
 				avevel_az[4] = avevel_az[3];
 				avevel_az[3] = avevel_az[2];
 				avevel_az[2] = avevel_az[1];
 				avevel_az[1] = avevel_az[0];
				
				avevel_alt[6] = avevel_alt[5];
 				avevel_alt[5] = avevel_alt[4];
 				avevel_alt[4] = avevel_alt[3];
 				avevel_alt[3] = avevel_alt[2];
 				avevel_alt[2] = avevel_alt[1];
 				avevel_alt[1] = avevel_alt[0];
				
				//avevel_az[5] = avevel_az[5];
				//avevel_alt[5] = avevel_alt[5]/5;
				
				vel_az = control.azimuth_command_double;
				vel_alt = control.altitude_command_double;
				azimuth_val = control.a[0] + td*(control.a[1] + td*control.a[2]);
				altitude_val = control.b[0] + td*(control.b[1] + td*control.b[2]);
				control.azimuth_command_double = azimuth_val;
				control.altitude_command_double=altitude_val;
				//Calculate the velocity of the the commands by using the previous commanded angle and the current
				vel_az = control.azimuth_command_double-vel_az;
				vel_alt = control.altitude_command_double - vel_alt;
				
// 				avevel_az[6] = 1.335;
// 				avevel_az[5] = 1.235;
// 				avevel_az[4] = 25.235;
// 				avevel_az[3] = 5.235;
// 				avevel_az[2] = 6.245;
// 				avevel_az[1] = 2.345;
// 				avevel_az[0] = 7.89;
				avevel_az[0] = vel_az;
				avevel_alt[0] = vel_alt;
				
				sort(avevel_az,avevel_azsorted,7);
				sort(avevel_alt,avevel_altsorted,7);
				//for(i=0;i<10;i++){
				//  printf("AZ VEL Vals unsorted %f Sorted %f \n",avevel_az[i],avevel_azsorted[i]);
				//}
				//for(i=0;i<10;i++){
				//    printf("ALT VEL Vals unsorted %f Sorted %f \n",avevel_alt[i],avevel_altsorted[i]);
				//}
				vel_az = avevel_azsorted[3];
				vel_alt = avevel_altsorted[3];
				//vel_az = avevel_azsorted[2];
// 				if(average_counter >10){
// 				  average_counter =10;
// 				}
// 				if(average_counter >5){
// 				  if(avevel_az[6]<0.9 || avevel_az[6] >1.1){
// 				    vel_az = avevel_az[5];
// 				  }
// 				  if(avevel_alt[6]<0.9 || avevel_alt[6] >1.1){
// 				    vel_alt = avevel_alt[5];
// 				  }
// 				}
				//simple filter of rubbish using averaging
				//vel_az = (avevel_az[5] + vel_az)/6.;
				//vel_alt = (avevel_alt[5] + vel_alt)/6.;
				control.vel_of_az = 1000.*vel_az;
				control.vel_of_alt = 1000.*vel_alt;
				//printf("Equatorial 2 Az %f ALT %f Vel AZ %f VEL ALT %f\n",azimuth_val,altitude_val,vel_az,vel_alt);
				//pointing(azimuth_val,altitude_val,&control.delta_az,&control.delta_alt);
				//azimuth_val+=control.delta_az;
				//altitude_val +=control.delta_alt;
				//control.az_command_long= (unsigned int)(65535.*(azimuth_val/360.));
				//control.az_command_long+=AZIMUTH_ZERO;
				azalt2encoder(azimuth_val,control.delta_az, altitude_val,control.delta_alt, &control.az_command_long, &control.alt_command_long);
				sort_azimuth(controlc.az_encoder_long,control.az_command_long,AZIMUTH_LIMIT_HI, AZIMUTH_LIMIT_LO,&control.az_command_long);
				azimuth_angle(control.azimuth_command_double,controlc.az_encoder_long,AZIMUTH_ZERO,AZIMUTH_SAFETY_LO,AZIMUTH_SAFETY_HI,&control.azimuth_command_double);
				//control.alt_command_long = (unsigned int)(65535.*(altitude_val/360.));
				//control.alt_command_long+=ELEVATION_ZERO;
				//printf("Az %f ALT %f \n",azimuth_val,altitude_val);
			}

	
			

			else {
				printf("Eq2 Error-Need new quadratic coefficients to maintain pointing accuracy. Tend = %ld T= %ld\n",control.eq2_time_end,current_time);
				control.a[0]=0;
				control.a[1]=0;
				control.a[2]=0;
				control.b[0]=0;
				control.b[1]=0;
				control.b[2]=0;
				
				
			}
						
		
			
			
		
			//printf("Here in EQ2\n");
		//control.az_command_long = 
		pthread_mutex_unlock (&mutexsum);
		//printf("ENCODER Coordinates RA %f DEC %f AZ %f %d ALT %f %d\n",right_ascension,declination,azimuth_val,control.az_command_long,altitude_val,control.alt_command_long);
	}
	
// 	else if(control.coordinate_command_type==EQUATORIAL2){
// 	
// 	}
	else if(control.coordinate_command_type==EQUATORIAL2){
			
			}
			
	else if(control.coordinate_command_type==HORIZONTAL_LIST){
			
			}
	else 
		printf("Error in Coordinate Command\n");

	pthread_mutex_lock (&mutexsum);	
		control.updatepos =1;
	pthread_mutex_unlock (&mutexsum);
	}

}

void *pid_thread (void *arg){
	int read_ret;
	read_ret = control_loop(fd,&control);
	
	exit(1);
}

int main(int argc, char *argv[]){
	//this routine starts everything up and then hands control over to the threads and the control loop which is handled in the getstructure() function
	unsigned long time_diff;
	unsigned long loop_time;
	struct pid_structure userspace;
	pthread_t writer_id;
	pthread_t command_id;
	pthread_t coordinate_update_id;
	pthread_t pidloop_id;
	char udpbuffer[10000];
	struct hostent *h;
	int read_ret,rc,i;
	unsigned int DAC_VAL;

	fd = open("/dev/pid",O_RDWR);
	if(fd ==-1){
	perror("failed to open PID Control Module\n");
	//rc = rd;
	exit(-1);
	}
	init_control_struct(&control);
	ioctl(fd,DEV_IOCTL_DISABLE_DAC, &control); 



	if(argc<2) {
	printf("usage : %s <server> <data1> ... <dataN> \n", argv[0]);
	exit(1);
	}
	
	h = gethostbyname(argv[1]);
	if(h==NULL) {
	printf("%s: unknown host '%s' \n", argv[0], argv[1]);
	exit(1);
	}
	
	printf("%s: sending data to '%s' (IP : %s) \n", argv[0], h->h_name,
	inet_ntoa(*(struct in_addr *)h->h_addr_list[0]));
	
	remoteServAddr.sin_family = h->h_addrtype;
	memcpy((char *) &remoteServAddr.sin_addr.s_addr,
	h->h_addr_list[0], h->h_length);
	remoteServAddr.sin_port = htons(REMOTE_SERVER_PORT);
	
	/* socket creation */
	sd_command = socket(AF_INET,SOCK_STREAM,0);
	if(sd_command<0) {
	printf("%s: cannot open socket \n",argv[0]);
	exit(1);
	}
	
	/* bind any port */
	cliAddr1.sin_family = AF_INET;
	cliAddr1.sin_addr.s_addr = htonl(INADDR_ANY);
	cliAddr1.sin_port = htons(0);
	

	if (connect(sd_command, &remoteServAddr, sizeof(remoteServAddr)) < 0) 
     	 error("ERROR connecting");


	//sprintf(udpbuffer,"Hello\n");
// 	rc = write(sd_command, udpbuffer, strlen(udpbuffer));
// 	
// 		if(rc<0) {
// 			printf("%s: cannot send data %d \n",argv[0],i-1);
// 			close(sd_command);
// 			exit(1);
// 		}

	
	
	DAC_VAL = ad5362_crc_pack(XREGISTER_WRITE,ALL,0);	
	printf("DAC_VAL %ld\n",DAC_VAL);

	 max7301_init();
	pthread_mutex_init(&mutexsum, NULL);
	pthread_mutex_init(&pid_coefficients,NULL);
	//pthread_create(&writer_id, NULL, writer_thread, NULL); 
	pthread_create(&coordinate_update_id, NULL, coordinate_thread, NULL);
	pthread_create(&command_id, NULL, command_thread, NULL);

	loop_time = 5;

	printf("Hello66\n");
	read_ret = control_loop(fd,&userspace);

	

	pthread_mutex_destroy(&mutexsum);
	pthread_mutex_destroy(&pid_coefficients);

	 

	close(fd);

}



//this is the subroutine where its all at- any editing should be done here
int control_loop(int pid_handle,struct pid_structure *userspace){
		int read_ret; //define handle for the read return function
		int fd2;
		int azimuth_double_zone=0;
		unsigned int prev_azencoder,prev_altencoder;
		double azerr1;
		double tachoaz1_prev,tachoaz2_prev,tachoalt1_prev,tachoalt2_prev;
		int aztacho1,aztacho2,alttacho1,alttacho2;
		long aztacho1v[20],aztacho2v[20],alttacho1v[20],alttacho2v[20];
		double fir[20];
		long aztacho1d,aztacho2d,alttacho1d,alttacho2d;
		struct pid_structure user,init,loop;
		long pid_return_old[4];
		long pid_return_new[4];
		long tacho_old[4],tacho_new[4];
		char buffer[5000];
		char cnt[20000];
		char udpcat[1000000];
		char *udpcatptr;
		char udpbuf[1000000];
		int counter;
		int pointing_counter;
		int packets;
		int firlen;
		int err_count_alt,err_count_az;
		long time_diff;
		int countera=0;
		int counterb=0;
		unsigned long time_diff2;
		int test_dac=1;
		int  rc, i,j;
		unsigned int DAC_val;
		double delta_az,delta_alt;
		double prev_azencoderd,prev_altencoderd;
		time_t current_time;
		unsigned int status_vec[20];
		double accel_az=0,accel_alt=0;
		double az_vel_vec[5],alt_vel_vec[5],az_acc_vec[5],alt_acc_vec[5],az_pos_vec[5],alt_pos_vec[5];
		double prev_azencoder_pos[20];
		double azencoder_vel[20],altencoder_vel[20];
		extern int sd_command;
		double tdouble,tdouble1,td;
		double avevel_az[10],avevel_alt[10],avevel_azsorted[10],avevel_altsorted[10];
		double vel_az=0,vel_alt=0;
		double altitude_val=0,azimuth_val=0;
		unsigned int az_test[5];
		unsigned int az_temp;
		long az_test2[5];
		long velpid_out_az,velpid_out_alt;
		long aztacho[5],alttacho[5];
		double azlim[5],ellim[5];
		double kfa_azimuth,kfa_altitude;
		
		
		printf("Starting\n");

		
		user = *userspace;
		fd2=pid_handle;
		read_ret = read(fd2,&init,sizeof(init));
		loop = control;
		
		printf("Command Angles LOOP AZ %d ALT %d\n",loop.az_command_long,loop.alt_command_long);
		printf("%d %d %d %d %d %d %d %d\n",(user.tacho1),(user.tacho2),(user.tacho3),(user.tacho4),(user.azimuth_zone));
		printf("HERE");
	err_count_alt=5000;
	if(test_dac==1){
	  //this is the start of the routine.
	  //first zero all necessary variables.
	  packets=0;
	  counter =0;
	  delta_az = 0;
	  delta_alt = 0;
	  pointing_counter = 0;	
	  //fir tap filter coefficients
	   for(j=0;j<=10;j++){
		fir[j] = 0.2;
	 
	    }
	  
	   for(j=0;j<=19;j++){
		aztacho1v[j]=0;
		aztacho2v[j]=0;
		alttacho1v[j]=0;
		alttacho2v[j]=0;
	    }
	    firlen=1;
	    
//LOOP BEGINS HERE!!-------------------------------	    
	    
	  while(err_count_alt>0){ //probably don't need this while statement anymore? Can just be while(1) ?
	  
	    //increment relevent counters
	    packets++;
	    counter++;
	    pointing_counter++;
	    user.counter =0;
	    //save the time from the last loop so that a value for the 'tick' interval can be estimated accurately
	    time_diff = user.time_struct.tv_usec;
	    prev_azencoder = (unsigned int)user.az_encoder_long;
	    prev_altencoder=(unsigned int)user.alt_encoder_long;
	    prev_azencoderd = loop.azimuth_encoder_double;
	    prev_altencoderd = loop.altitude_encoder_double;

//-----------------------------------------------------------
	   //This section is an attempt to try and get stable velocity values from the encoders themselves- need to expand to the altitude encoders too
 	    prev_azencoder_pos[0] = prev_azencoder_pos[1];
 	    prev_azencoder_pos[1] = prev_azencoder_pos[2];
 	    prev_azencoder_pos[2] = prev_azencoder_pos[3];
 	    prev_azencoder_pos[3] = prev_azencoder_pos[4];
 	    prev_azencoder_pos[4] = prev_azencoder_pos[5];
 	    prev_azencoder_pos[5] = prev_azencoder_pos[6];
 	    prev_azencoder_pos[6] = loop.azimuth_encoder_double;
// 	    
 	    azencoder_vel[0] = azencoder_vel[1];
 	    azencoder_vel[1] = azencoder_vel[2];
 	    azencoder_vel[2] = azencoder_vel[3];
 	    azencoder_vel[3] = azencoder_vel[4];
 	    azencoder_vel[4] = azencoder_vel[5];
 	    azencoder_vel[5] = azencoder_vel[6];
 	    azencoder_vel[6] = 1000*100*(prev_azencoder_pos[6]-prev_azencoder_pos[0])/6; //in deg/s
// 	    //azencoder_vel[7] = (prev_azencoder_pos[6]-prev_azencoder_pos[0])/(6*0.011);
// 	    
// 	    
 	    azencoder_vel[7] = (azencoder_vel[6]+0.5*azencoder_vel[5] + 0.25*azencoder_vel[4] +0.25*azencoder_vel[3]+0.1*azencoder_vel[2]+0.1*azencoder_vel[1]+0.1*azencoder_vel[0])/2.3; //in mdeg/s
//------------------------------------------------------------------------	    
	    tachoaz1_prev = (double)aztacho1;
	    tachoaz2_prev = (double)aztacho2;
	    tachoalt1_prev=(double)alttacho1;
	    tachoalt2_prev=(double)alttacho2;
	    
	    //read from the kernel
	    
	    read_ret = read(fd,&user,sizeof(user));//THIS IS WHERE THE TICK INTERVAL IS IMPLEMENTED. The kernel blocks the read and this is released by an onboard timer overflow interrupt routine. This gives an consistent timing mechanism to the pid control loop.
	     //tacho offsets as there is a slight negative bias here!
	
	    //user.tacho1+=88;
	    //user.tacho2+=41;
	    //user.tacho3+=62;
	    //user.tacho4+=68;
	   
	   
	   for(j=0;j<=firlen-1;j++){
		aztacho1v[firlen-j]=aztacho1v[firlen-j-1];
		aztacho2v[firlen-j]=aztacho2v[firlen-j-1];
		alttacho1v[firlen-j]=alttacho1v[firlen-j-1];
		alttacho2v[firlen-j]=alttacho2v[firlen-j-1];
	    }
	    
	    aztacho1v[0] = (long)user.tacho2;
	    aztacho2v[0]= (long)user.tacho3;
	    alttacho1v[0]=(long)user.tacho4;
	    alttacho2v[0]=(long)user.tacho1;
// 	   
	    
	    aztacho1d =0;
	    aztacho2d = 0;
	    alttacho1d = 0;
	    alttacho2d = 0;
	
	    for(j=0;j<=firlen-1;j++){

	      aztacho1d +=aztacho1v[j];
	      aztacho2d += aztacho2v[j];
	      alttacho1d += alttacho1v[j];
	      alttacho2d += alttacho2v[j];
	    }
 	    aztacho1 =aztacho1d/firlen;
 	    aztacho2=aztacho2d/firlen;
 	    alttacho1=alttacho1d/firlen;
 	    alttacho2=alttacho2d/firlen;  
	    
	    aztacho2+=40;

	 
	 
	    
	    if(pointing_counter >0){//this is for pointing corrections etc. I have actually found it to be problematic to do any significant pointing corrections in this loop as it messes up the timing since it can take variable amounts of time to apply pointing corrections. I think what is better is to keep all these coordinates in native telescope coordinates and correct them at the control PC.
		   //this routine is important as it updates the azimuth/elevation encoders from the raw encoder value to the floating point representation-the function is defined in pointing.c- the zero point of the azimuth and elevation 
		   //should be defined in telescope_constants.h if (for example) the encoders are changed. This should be done manually and is only required once.
		    encoder2azaltprime(user.az_encoder_long,delta_az,user.alt_encoder_long,delta_alt,&loop.azimuth_encoder_double, &loop.altitude_encoder_double);
		    
		   
		    pointing_counter = 0;
	    }
    
	    

	    //pointing2(user.azimuth_encoder_double,user.altitude_encoder_double,&delta_az,&delta_alt);

	    
	    time_diff2 =  user.time_struct.tv_usec ;//calculate the time interval between loops.
	    if(time_diff2>time_diff){
		    time_diff = time_diff2-time_diff;
	    }
	    else if(time_diff2<=time_diff){
		    time_diff = time_diff2+1000000 - time_diff;
	    }
		
	    
	    if(user.read_status){
		    printf("Error in Reading the antenna Status\n");
	    }

	    
// 			      if(control.coordinate_command_type==HORIZONTAL_LIST){
// 				  for(i=0;i<=99;i++){
// 				    time(&current_time);
// 				    i = current_time-control.eq2_time_begin;
// 				   // printf("update %d %f %f %ld %ld\n",i,command.comaz[i],command.comalt[i],control.eq2_time_end,control.eq2_time_begin);
// 				  }
// 				}
	    
	    //-------------TRY THIS HERE RATHER THAN IN THE COORDINATE INTERRUPT-This relies on one type of command being given to the controller-
						
			if((control.coordinate_command_type==EQUATORIAL2) || control.coordinate_command_type==HORIZONTAL_LIST ){
				
				tdouble1=user.time_struct.tv_sec;
				tdouble = user.time_struct.tv_usec;
				
				time(&current_time);
				//check if the time is still valid
				if(current_time<=control.eq2_time_end){
				counterb=0;
				//time(&current_time);
	    
	    
				td = tdouble1 + tdouble/1000000.;
				tdouble = (control.eq2_time_begin);
				td = td - tdouble;
				//save the previous command velocities in preparation to calculate the accelerations
				
				
				
				
				//save the previous command positions in preparation to calculate the velocities
				vel_az = azimuth_val;
				vel_alt = altitude_val;
				if(control.coordinate_command_type==EQUATORIAL2){
				  azimuth_val = control.a[0] + td*(control.a[1] + td*control.a[2]);
				  altitude_val = control.b[0] + td*(control.b[1] + td*control.b[2]);
				}
				
				 if(control.coordinate_command_type==HORIZONTAL_LIST){
				   i = current_time-control.eq2_time_begin;
				   td = user.time_struct.tv_usec/1000000.;
				   azimuth_val=command.comaz[i]+td*(command.comaz[i+1]-command.comaz[i]);
				   altitude_val = command.comalt[i]+td*(command.comalt[i+1]-command.comalt[i]);
				 
				  }
				
				
				//need the software limit checked to go in this position- it needs to 
				//1. make sure the command encoder values do not go past this zone
				//2. make sure the velocity command is zeroed past this value
				//3. Also (preferably) define a slow zone that should prevent the elevation from going wrong and allow the antenna to be parked at zenith
				
				
				loop.azimuth_command_double = azimuth_val;
				loop.altitude_command_double= altitude_val;
				//Calculate the velocity of the the commands by using the previous commanded angle and the current
				vel_az = azimuth_val-vel_az;
				vel_alt = altitude_val-vel_alt;
				
// 				
				
				loop.vel_of_az = 1000.*vel_az*100; //get into mdeg/s
				loop.vel_of_alt = 1000.*vel_alt*100;//get into mdeg/s
	    
	    
				azalt2encoder(azimuth_val,control.delta_az, altitude_val,control.delta_alt, &loop.az_command_long, &loop.alt_command_long);
				sort_azimuth(loop.az_encoder_long,loop.az_command_long,AZIMUTH_LIMIT_HI, AZIMUTH_LIMIT_LO,&loop.az_command_long);
				//azimuth_angle(loop.azimuth_command_double,loop.az_encoder_long,AZIMUTH_ZERO,AZIMUTH_SAFETY_LO,AZIMUTH_SAFETY_HI,&loop.azimuth_command_double);
				}

			if(control.coordinate_command_type==HORIZONTAL){

				    printf("HORIZONTAL");
}
				
				else {
				  
				  if(counterb==0){
				    printf("Eq2 Error-Need new quadratic coefficients to maintain pointing accuracy. Tend = %ld T= %ld\n",control.eq2_time_end,current_time);
				  counterb=1;
				  }
				  loop.vel_of_az = 0;
				  loop.vel_of_alt = 0;
				  loop.azimuth_command_double = azimuth_val;
				  loop.altitude_command_double= altitude_val;
				  azalt2encoder(azimuth_val,control.delta_az, altitude_val,control.delta_alt, &loop.az_command_long, &loop.alt_command_long);
				  sort_azimuth(loop.az_encoder_long,loop.az_command_long,AZIMUTH_LIMIT_HI, AZIMUTH_LIMIT_LO,&loop.az_command_long);
				  //azimuth_angle(loop.azimuth_command_double,loop.az_encoder_long,AZIMUTH_ZERO,AZIMUTH_SAFETY_LO,AZIMUTH_SAFETY_HI,&loop.azimuth_command_double);
				
				
				}
	    
			}
	    
	    
	    
	    
	    //---------THIS LOOP IS ON IF THE COMPARE TO VALUE IS 1 OTHERWISE IT IS NOT ENABLED AND WILL BE SET AS 5------------------------------------
	    if(control.updatepos ==1){//update the control angles- these are done in the coordinate thread once a second after which the flag control.updatepos is set. This if statement checks whether the flag is set (i.e whether commands need to be updated?) and then proceeds to copy relevent values into relevent places.
		    control.updatepos=0;
	    	printf("update %d %d %d\n",control.update,control.az_command_long,control.alt_command_long);
		    loop.az_command_long = control.az_command_long;
		    loop.alt_command_long = control.alt_command_long;
		    loop.azimuth_command_double = control.azimuth_command_double;
		    loop.altitude_command_double	= control.altitude_command_double;
		    loop.delta_az = control.delta_az;
		    loop.delta_alt = control.delta_alt;
		    loop.vel_of_az = control.vel_of_az;
		    loop.vel_of_alt = control.vel_of_alt;
		   
		    
	    }
	    
	    if(control.update ==1){//The PID coefficients might also be updates in which case they need to be changed in the loop.
		    control.update =0;
		    for(i=0;i<=4;i++){
			    printf("UPDATE LIMITS\n");
			    //loop.pcoeffs[i]=control.pcoeffs[i];
			    //loop.icoeffs[i]=control.icoeffs[i];
			    //loop.dcoeffs[i]=control.dcoeffs[i];
			    //loop.motor_plus[i] = control.motor_plus[i];
			    //loop.motor_minus[i] = control.motor_minus[i];
			    loop.pcoeffs[i]=2000.;
			    loop.icoeffs[i]=0.5;
			    loop.dcoeffs[i]=0.;
			    loop.kfcoeffs[i]=0.1;
			loop.vfcoeffs[i]=0.1;
			loop.pcoeffs_vel[i]=1;
			loop.icoeffs_vel[i]=0.;
			loop.dcoeffs_vel[i]=0.;
			    loop.motor_plus[i] = 0.;
			    loop.motor_minus[i] = 0.;
			 //   loop.limits[i] = control.limits[i];
		    }	
	    
	    }
		    
		//pthread_mutex_lock(&pid_coefficients);    
/*		 if(azimuth_pid1.adaptive==1){
		  // printf("Adaptive PID on AZ1\n");
		    pidadaptive_update(0,(unsigned int)user.az_encoder_long, (unsigned int)loop.az_command_long, &azimuth_pid1,&loop.pcoeffs[0],&loop.icoeffs[0],&loop.dcoeffs[0],&loop.kfcoeffs[0],&loop.vfcoeffs[0],&loop.motor_plus[0],&loop.motor_minus[0],&loop.pcoeffs_vel[0],&loop.icoeffs_vel[0],&loop.dcoeffs_vel[0]);
		}
		 if(azimuth_pid2.adaptive==1){
		    pidadaptive_update(1,(unsigned int)user.az_encoder_long, (unsigned int)loop.az_command_long, &azimuth_pid2,&loop.pcoeffs[1], &loop.icoeffs[1], &loop.dcoeffs[1],&loop.kfcoeffs[1],&loop.vfcoeffs[1],&loop.motor_plus[1],&loop.motor_minus[1],&loop.pcoeffs_vel[1],&loop.icoeffs_vel[1],&loop.dcoeffs_vel[1]);
		   }
		 if(altitude_pid1.adaptive==1){
		  pidadaptive_update(2,(unsigned int)user.alt_encoder_long, (unsigned int)loop.alt_command_long,&altitude_pid1,&loop.pcoeffs[2], &loop.icoeffs[2], &loop.dcoeffs[2],&loop.kfcoeffs[2],&loop.vfcoeffs[2],&loop.motor_plus[2],&loop.motor_minus[2],&loop.pcoeffs_vel[2],&loop.icoeffs_vel[2],&loop.dcoeffs_vel[2]);
		 }
		    
		 if(altitude_pid2.adaptive==1){
		    pidadaptive_update(3,(unsigned int)user.alt_encoder_long,(unsigned int)loop.alt_command_long,&altitude_pid2,&loop.pcoeffs[3], &loop.icoeffs[3], &loop.dcoeffs[3],&loop.kfcoeffs[3],&loop.vfcoeffs[3],&loop.motor_plus[3],&loop.motor_minus[3],&loop.pcoeffs_vel[3],&loop.icoeffs_vel[3],&loop.dcoeffs_vel[3]);
		}
		//pthread_mutex_unlock(&pid_coefficients);  // 
	    
//		    printf("%f %f %f %f %f %f %f %f %f %f \n", loop.pcoeffs[3], &loop.icoeffs[3], &loop.dcoeffs[3],&loop.kfcoeffs[3],&loop.vfcoeffs[3],&loop.motor_plus[3],&loop.motor_minus[3],&loop.pcoeffs_vel[3],&loop.icoeffs_vel[3],&loop.dcoeffs_vel[3]);
*/
	    if(packets==5 && user.azimuth_zone<=2){//every 5 packets send data to the control PC
		    sprintf(udpbuf,"");
		    sprintf(udpbuf,"%s",udpcat);
		    sprintf(udpcat,"");
		    packets=0;
		    rc = send(sd_command, udpbuf, strlen(udpbuf),0);
		//    rc = send(sd_command, tcp_control_structs, 5*sizeof(tcp_control_structs),0);
		    if(rc<0) {
			    printf(": cannot send data in pid loop \n");
			    close(sd_command);
			    exit(1);
		    }
	    }
	    //Save the pid return value so that the ramp clipping does not throw away valuable information.
	    pid_return_old[0] = pid_return_new[0];
	    pid_return_old[1] = pid_return_new[1];
	    pid_return_old[2] = pid_return_new[2];
	    pid_return_old[3] = pid_return_new[3];

	    
	    
	     
	    
	    //pid output calculation
	    update_pid_double(AZ_PID, loop.azimuth_command_double,prev_azencoderd, loop.azimuth_encoder_double,loop.pcoeffs[0],loop.icoeffs[0],loop.dcoeffs[0],loop.kfcoeffs[0],loop.vel_of_az,time_diff,loop.motor_plus[0],loop.motor_minus[0],MAX_AZ_SPEED,MIN_AZ_SPEED,3000/loop.icoeffs[0],-3000/loop.icoeffs[0], &azerr1, &loop.az_ic1,&loop.az_pid1);
	    
	    // update_pid_double(AZ_PID, loop.azimuth_command_double,prev_azencoderd, loop.azimuth_encoder_double,loop.pcoeffs[1],loop.icoeffs[1],loop.dcoeffs[1],loop.kfcoeffs[1],loop.vel_of_az,time_diff,loop.motor_plus[1],loop.motor_minus[1],MAX_AZ_SPEED,MIN_AZ_SPEED,3000/loop.icoeffs[1],-3000/loop.icoeffs[1], &loop.az_err2, &loop.az_ic2,&loop.az_pid2);
	    
	 
	    update_pid(ALT_PID, (unsigned int)loop.alt_command_long,prev_altencoder, (unsigned int)user.alt_encoder_long,loop.pcoeffs[2],loop.icoeffs[2],loop.dcoeffs[2],loop.kfcoeffs[2],loop.vel_of_alt,time_diff,loop.motor_plus[2],control.motor_minus[2],MAX_ALT_SPEED,MIN_ALT_SPEED,2000/loop.icoeffs[2],-2000/loop.icoeffs[2], &loop.alt_err1, &loop.alt_i1,&loop.alt_pid1);
	   // update_pid(ALT_PID, (unsigned int)loop.alt_command_long, prev_altencoder,(unsigned int)user.alt_encoder_long,loop.pcoeffs[3],loop.icoeffs[3],loop.dcoeffs[3],loop.kfcoeffs[3],loop.vel_of_alt,time_diff,loop.motor_plus[3],control.motor_minus[3],MAX_ALT_SPEED,MIN_ALT_SPEED,2000/loop.icoeffs[3],-2000/loop.icoeffs[3], &loop.alt_err2, &loop.alt_i2,&loop.alt_pid2);
	    
	    
	    if(countera<10){
	      countera++;
	     
	    }
	    else if(countera>=10){
	    
	    
	    countera=0;
	    }
	    
	    pid_return_new[0] = loop.az_pid1;
 	    pid_return_new[1] = 0.5*loop.az_pid1;
 	    pid_return_new[2] = loop.alt_pid1;
 	    pid_return_new[3] = 0.5*loop.alt_pid1;
	   
//	VELOCITY PID LOOP----------------------	   
	   
	 // if((loop.kfcoeffs[0]*loop.vel_of_az<10000.) && (loop.kfcoeffs[0]*loop.vel_of_az>-10000.)){
	 
	     aztacho[1] = aztacho1;
	     aztacho[2] = aztacho2;
	     aztacho[3] = (long)(loop.kfcoeffs[0]*loop.vel_of_az);
	     alttacho[1] = alttacho1;
	     alttacho[2] = alttacho2;
	     alttacho[3] = (long)(loop.kfcoeffs[2]*loop.vel_of_alt);
	     
	    switch((aztacho[3]<10000 && aztacho[3]>-10000)){
	     
	      case 1:	//i.e the low velocity zone
		aztacho[0] = (long)aztacho2; //use the low velocity value from the ADC i.e channel 1
 		kfa_azimuth = loop.kfcoeffs[0];
		//if(countera==9){
 		 // printf("Aztacho1 Zone %ld\n",aztacho[3]);
 		 //}
		break;
	      case 0: //i.e the high velocity zone
		aztacho[0] = (long)aztacho2;
 		//aztacho[0]=aztacho[0]; //adjust for the lower ADC ratings
		kfa_azimuth = loop.kfcoeffs[0];
		//if(countera==9){
 		  //printf("Aztacho2 Zone\n");
 		// }
		break;
	      default:
		printf("Error in the AZ tachometer switching\n");
			      
	    }
	    
	   switch((alttacho[3]<10000 && alttacho[3]>-10000)){
	     
	      case 1:	//i.e the low velocity zone
		alttacho[0] = (long)alttacho1; //use the low velocity value from the ADC i.e channel 1
		break;
	      case 0: //i.e the high velocity zone
		alttacho[0] = (long)alttacho2;
		break;
	      default:
		printf("Error in the ALT tachometer switching\n");
			      
	    }
	    
	    //do the velocity pid loop using the switched velocities depending on what speed you actually want to go
	    
	    velocity_pid(aztacho[0],loop.vel_of_az,kfa_azimuth,loop.pcoeffs_vel[0],loop.icoeffs_vel[0], 0.,1500/loop.icoeffs_vel[0],-1500/loop.icoeffs_vel[0],7000,-7000,&loop.az_ic1_vel,&velpid_out_az,&loop.az_pid1);
	    
	    velocity_pid(alttacho[0],loop.vel_of_alt,loop.kfcoeffs[2],loop.pcoeffs_vel[2], loop.icoeffs_vel[2], 0.,1000,-1000,2000,-2000,&loop.alt_ic1_vel,&velpid_out_alt,&loop.alt_pid1);
	      if(countera==9){
	      // printf("%7ld \n",loop.az_err1);
	      // printf("PID pos %7d vel %7d Tot %7d AZ %7lf %7lf %7d %7lf %7lf %7lf \n",pid_return_new[0],velpid_out_az,loop.az_pid1,loop.vel_of_az,loop.kfcoeffs[0]*loop.vel_of_az,aztacho1,azencoder_vel[7],loop.vel_of_alt,azerr1);
	    //  printf("Az Encoder Velocity Tacho %d %f\n",aztacho1,loop.kfcoeffs[0]*loop.vel_of_az);
	      }
	  // }
// 	   else if((loop.kfcoeffs[0]*loop.vel_of_az>=10000.) || (loop.kfcoeffs[0]*loop.vel_of_az<=-10000.)){
// 	    if(countera==6){
// 	    velocity_pid(azencoder_vel[7],loop.vel_of_az,loop.kfcoeffs[0],loop.pcoeffs_vel[0],loop.icoeffs_vel[0], 0.,1000/loop.icoeffs_vel[0],-1000/loop.icoeffs_vel[0],2000,-2000,&loop.az_ic1_vel,&velpid_out_az,&loop.az_pid1);
// 	    
// 	    velocity_pid((long)alttacho1,loop.vel_of_alt,loop.kfcoeffs[2],loop.pcoeffs_vel[2], loop.icoeffs_vel[2], 0.,1000,-1000,2000,-2000,&loop.alt_ic1_vel,&velpid_out_alt,&loop.alt_pid1);
// 	    }
// 	      if(countera==9){
// 	      // printf("%7ld \n",loop.az_err1);
// 	      // printf("PID pos %7d vel %7d Tot %7d AZ %7lf %7lf %7d %7lf %7lf %7lf \n",pid_return_new[0],velpid_out_az,loop.az_pid1,loop.vel_of_az,loop.kfcoeffs[0]*loop.vel_of_az,aztacho1,azencoder_vel[7],loop.vel_of_alt,azerr1);
// 	      printf("Az Encoder Velocity Encoder %f %f\n",loop.kfcoeffs[0]*azencoder_vel[7],loop.kfcoeffs[0]*loop.vel_of_az);
// 	      }
// 	   }
//-------------------------------------------------	    
	   
	    
 	    pid_return_new[0] = loop.az_pid1;
 	    pid_return_new[1] = 0.5*loop.az_pid1;
 	    pid_return_new[2] = loop.alt_pid1;
 	    pid_return_new[3] = 0.5*loop.alt_pid1;
	    
	    //might requires a different bit of additional damping if only the pid loop is running (i.e command_velocity is zero)
	    if(loop.vel_of_az==0){
	    pid_return_new[0] -= loop.vfcoeffs[0]*aztacho1;
	    pid_return_new[1] -= loop.vfcoeffs[1]*aztacho2;
	    }
	    if(loop.vel_of_alt==0){
	    pid_return_new[2] -= loop.vfcoeffs[2]*alttacho1;
	    pid_return_new[3] -= loop.vfcoeffs[3]*alttacho2;
	    }
  //----------------------Implement acceleration limits
				az_pos_vec[0]=az_pos_vec[1];
				az_pos_vec[1]=az_pos_vec[2];
				az_pos_vec[2]=loop.azimuth_encoder_double;
				alt_pos_vec[0]=alt_pos_vec[1];	
				alt_pos_vec[1]=alt_pos_vec[2];
				alt_pos_vec[2]=loop.altitude_encoder_double;
				
				az_vel_vec[0]=az_vel_vec[1];
				az_vel_vec[1]=az_vel_vec[2];
				az_vel_vec[2] = az_pos_vec[2]-az_pos_vec[1];
				alt_vel_vec[0]=alt_vel_vec[1];
				alt_vel_vec[1]=alt_vel_vec[2];
				alt_vel_vec[2] = alt_pos_vec[2]-alt_pos_vec[1];
				
				
				az_acc_vec[0]=az_acc_vec[1];
				az_acc_vec[1]=az_acc_vec[2];
				az_acc_vec[2] = az_pos_vec[2] - 2*az_pos_vec[1] + az_pos_vec[0];
				
				alt_acc_vec[0]=alt_acc_vec[1];
				alt_acc_vec[1]=alt_acc_vec[2];
				alt_acc_vec[2] = alt_pos_vec[2] - 2*alt_pos_vec[1] + alt_pos_vec[0];
				
				accel_az = az_acc_vec[2];
				accel_alt = alt_acc_vec[2];

				//put in maximum accelerations permitted- the enums are defined in telescope_constants.h
				if(accel_az>(double)MAX_AZ_ACCEL/1000.){
				  //calculate maximum allowable velocity
				  
				    //printf("Too much Acceleration pos %f %f %f %f\n",az_pos_vec[2],az_pos_vec[1],az_pos_vec[0],accel_az);
				     pid_return_new[0] = 0.5*pid_return_new[0];
				     pid_return_new[1] = 0.5*pid_return_new[1];
			
				}
				
				if(accel_az<(double)MIN_AZ_ACCEL/1000.){
				  //calculate maximum allowable velocity
		
				    pid_return_new[0] = 0.5*pid_return_new[0];
				    pid_return_new[1] = 0.5*pid_return_new[1];
				
				}
				
				//put in maximum accelerations permitted- the enums are defined in telescope_constants.h
				if(accel_alt>(double)MAX_ALT_ACCEL/1000.){
		
				     pid_return_new[2] = 0.5*pid_return_new[2];
				     pid_return_new[3] = 0.5*pid_return_new[3];
			
				}
				
				if(accel_alt<(double)MIN_ALT_ACCEL/1000.){
				  //calculate maximum allowable velocity
				    pid_return_new[2] = 0.5*pid_return_new[2];
				    pid_return_new[3] = 0.5*pid_return_new[3];
	
				}
//-------------------------------------------------------------------------------				
	  ramp(&pid_return_old,&pid_return_new[0],&pid_return_new[1],&pid_return_new[2],&pid_return_new[3],MAX_AZ_SPEED,MIN_AZ_SPEED,MAX_ALT_SPEED,MIN_ALT_SPEED,0,0,0,0);//the acceleration of the antenna needs to be limited. This is handled by this ramp() routine

	   
    //	backlash(user.alt_command_long,user.alt_encoder_long,2000,0,0,0,&user.alt_pid1,&user.alt_pid2);


	   
	 
	  
	  soft_lim(&loop,loop.limits[1],loop.limits[0],AZIMUTH_SAFETY_HI,AZIMUTH_SAFETY_LO,AZ_SLOW_SPEED_POSITIVE,AZ_SLOW_SPEED_NEGATIVE,user.az_encoder_long,&(loop.az_command_long),&(loop.az_pid),&(pid_return_new[0]),&(pid_return_new[1]));//the software limit checking to prevent antenna driving into its own hard limits.

	    soft_lim(&loop,loop.limits[3],loop.limits[2],ELEVATION_SAFETY_HI,ELEVATION_SAFETY_LO,EL_SLOW_SPEED_POSITIVE,EL_SLOW_SPEED_NEGATIVE,user.alt_encoder_long,&(loop.alt_command_long),&(loop.alt_pid),&(pid_return_new[2]),&(pid_return_new[3]));
	   
	   
	    backlash(loop.az_command_long,user.az_encoder_long,loop.vel_of_az,500,100,0,0,&pid_return_new[0],&pid_return_new[1]);
	      

	       printf("PID pos %7d %7d %7d %7d \n",pid_return_new[0],pid_return_new[1],pid_return_new[2],pid_return_new[3]);

	    //calculate the data that needs to be written to the DACS. These include a CRC for robustness.
	    loop.az1_dac_control = ad5362_crc_pack(XREGISTER_WRITE,CH5,pid_return_new[0]);	
	    loop.az2_dac_control = ad5362_crc_pack(XREGISTER_WRITE,CH6,pid_return_new[1]);
	    loop.alt1_dac_control = ad5362_crc_pack(XREGISTER_WRITE,CH7,pid_return_new[2]);
	    loop.alt2_dac_control = ad5362_crc_pack(XREGISTER_WRITE,CH8,pid_return_new[3]);

	    
	   // status_vec=control.status_vec;
	    
	    
	   // printf("Status Query : \nChip1 12-19 %04x\nChip1 20-27 %04x\nChip1 P24-31 %04x\nChip2 P12-19 %04x\nChip2 P20-27 %04x\nChip2 P28-31 %04x\n",control.status_vec[0],control.status_vec[2],control.status_vec[4],control.status_vec[3],control.status_vec[5],control.status_vec[6]);
	    //printf("%d %d %d %d %08x %08x %08x %08x",user.az_pid1,user.az_pid2,user.alt_pid1,user.alt_pid2,user.az1_dac_control,user.az2_dac_control,user.alt1_dac_control,user.alt2_dac_control);
	    if(!user.read_status){
	    //ioctl(fd,DEV_IOCTL_WRITE_PID, &loop);
	    //write the relevent voltage commands (after pid, ramp and software limits) to the kernel board. This then updated the DACs, gives the voltage command to the YASKAWA servopacks and the antenna can begin to drive.
	    //NOTE there is further control over the voltage output at the kernel level and certain flags need to be set before there will actually be voltage output. This is done using the IOCTL to the kernel with command value DEV_IOCTL_ENABLE_DAC or DEV_IOCTL_DISABLE_DAC respectively. Handled in the command thread.
		    read_ret = write(fd,&loop,sizeof(loop));
	    }
	    else 
	    printf("error writing\n");
    //	read_ret = write(fd,&user,sizeof(user));
    //	read_ret = write(fd,&control_struct,sizeof(control_struct));

	    if(user.azimuth_zone<=2){//and now we simply send the date through- this might be better done by sending the entire structure since the ascii string packing is inherently slow. But sending structures causes 2 problemss:
	    //1. Need to send the data each time loop. Lots of overhead (i.e it is more difficult to keep the data building up and then only actually send the data every now and again). This can actually be overcome fairly easily if I tried!
	    //2. This locks us into using C on the other side of the channel since other programming languages have different memorey layout. This is the main reason I have not pursued this cours.
	   //  tcp_control_structs[packets] = user;
	     sprintf(cnt,"%10.3f,%10.3f,%8u,%8u,%8.3f,%8.3f,%6u,%6u,%2i,%6i,%6i,%6i,%6i,%6i,%6i,%6i,%6i,%6i,%10lu,%10lu,%6i,%6i,%6i,%6i,%6i,%8ld,%8ld,%7ld,%7ld,%5u,%5ld,%04x,%04x,%04x,%04x,%04x,%04x,%5.3f,%5.3f,end \n",loop.azimuth_encoder_double,loop.azimuth_command_double,user.az_encoder_long,loop.az_command_long,loop.altitude_encoder_double,loop.altitude_command_double,user.alt_encoder_long,loop.alt_command_long,user.azimuth_zone,test_dac,aztacho1,aztacho[0],alttacho1,alttacho2,pid_return_new[0],pid_return_new[1],pid_return_new[2],pid_return_new[3],loop.alt1_dac_control,loop.alt2_dac_control,user.encoder_error,err_count_alt,err_count_az,loop.az_command_long,loop.alt_command_long,user.time_struct.tv_sec,user.time_struct.tv_usec,user.time,time_diff,user.counter3,user.counter2,control.status_vec[0],control.status_vec[2],control.status_vec[4],control.status_vec[3],control.status_vec[5],control.status_vec[6],loop.vel_of_az*loop.kfcoeffs[0],loop.vel_of_alt*loop.kfcoeffs[2]);
	     //loop.vel_of_az*loop.kfcoeffs[0],loop.vel_of_alt*loop.kfcoeffs[2]
	    }
	    else {	
		    printf("\n\n\n ----------------------------ERROR USING THE CONTROL SOFTWARE-------------------------\n\n\nPROBLEM WITH AZIMUTH ZONE INFORMATION... UPDATE THE PID STRUCTURE WITH A VALUE BEFORE RUNNING THIS\n\n./azimuth_zone 1 \n\n The Zone information can be read using \n\n./read_azimuth_zone \n\n This should only need to be done on bootup or if the kernel module has been recently removed and re-inserted. \n\n ----------------------ZONE INFORMATION-------------\n\n If the Telescope was parked then you are in Zone 1. If the telescope has reached its minimum azimuth limit then you are in Zone 0 and if it has reached maximum azimuth limit then you are in Zone 2\n\n\n\n");
		    
		    sprintf(cnt,"\n\n\nPROBLEM WITH AZIMUTH ZONE INFORMATION... UPDATE THE PID STRUCTURE WITH A VALUE BEFORE RUNNING THIS\n\n./azimuth_zone 1 \n\n The Zone information can be read using \n\n./read_azimuth_zone \n\n This should only need to be done on bootup. If the Telescope was parked then you are in Zone 1. If the telescope has reached its minimum azimuth limit then you are in Zone 0 and if it has reached maximum azimuth limit then you are in Zone 2\n\n\n\n");
		    rc = send(sd_command, udpbuf, strlen(udpbuf),0);
		    exit(1);
	    }
	    
	    strcat(udpcat,cnt);
	    sprintf(cnt,"");
	    
	   // strcat(udpcat,"\n");
	    }
	}
	return 0;

}
//void velocity_pid(int vel_tacho,double vel_command,double Kf, double P, double I, double D, long MAX_I, long MIN_I, double *current_i_ptr, long *pid_return_ptr)

void velocity_pid(long vel_tacho, double vel_command, double Kfa, double Pd, double Id, double Dd, long Mx_I, long Mn_I, int MAX_OUT,int MIN_OUT,double *current_i_ptr,long *velpid_out, long *pid_return_ptr){
  double corrected_vel_command;
  double error;
  double output_p,output_i;
  double ival,return_val_double;
  long return_val;
    //Kf is the conversion factor to get the tacho velocity to the same "units" as the velocity command
    corrected_vel_command = vel_command*Kfa;
   // vel_tacho=vel_tacho*Kfa;
    // ival is the old integrator value
    
    if(corrected_vel_command==0){
      *current_i_ptr=0;
    }
    
    
    
    ival =0.;
    error =0.;
    ival = *current_i_ptr;
    //Calculate the velocity error
    //error = corrected_vel_command-vel_tacho;
   // error = (double)*pid_return_ptr;
    error =corrected_vel_command-vel_tacho;
    //integral
    ival += error;
    if(ival > Mx_I){
      ival = Mx_I;
    }
    
    if(ival < Mn_I){
      ival = Mn_I;
    }
    
    //update the integral
    *current_i_ptr=ival;
    
    output_p = error*Pd;
    output_i = ival * Id;
    
    return_val_double = output_p + output_i;
    
    return_val = (long)return_val_double;
    //here we make sure the velocity outputs don't dwarf the position output
    if(return_val>MAX_OUT){
      return_val=(long)MAX_OUT;
    }
    if(return_val<MIN_OUT){
      return_val=(long)MIN_OUT;
    }
    if(corrected_vel_command==0){
      return_val=0;
    }
    *velpid_out=return_val;
    //printf("Velocity Command %d Velocity Tacho %f Vel PID %ld\n",vel_tacho,corrected_vel_command,return_val);
    *pid_return_ptr += return_val;


}

void tacho_ramp(long *tacho_old,long *tacho_new,long *pid,int MAX_AZ,int MIN_AZ,int MAX_ALT,int MIN_ALT){


}
		
		
int init_control_struct(struct pid_structure *control_ptr){
	//the initiation of the control structure 
	struct pid_structure controlj;
	int i;
	int read_ret;
	
	
	read_ret = read(fd,&controlj,sizeof(controlj));
	//pthread_mutex_lock (&mutexsum);
		//controlj = *control_ptr;
	//pthread_mutex_unlock (&mutexsum);
	printf("AZIMUTH ZONE ON STARTUP = %d\n",controlj.azimuth_zone);
	
	controlj.DAC_Output=0;
	controlj.alt_err_prev=0;
	controlj.alt_p = 0;
	controlj.alt_i = 0;
	controlj.alt_d = 0 ;
	controlj.az_p = 0;
	controlj.az_i = 0;
	controlj.az_d = 0 ;
	controlj.time = 0;
	controlj.az_i2 =0;
	controlj.az_i1 = 0;
	controlj.alt_i1=0;
	controlj.alt_i2=0;
	controlj.az_err2 =0;
	controlj.az_err1 = 0;
	controlj.alt_err1=0;
	controlj.alt_err2=0;
	controlj.az_p1=0.;
	controlj.az_p2=0.;
	controlj.az_ic1=0.;
	controlj.az_ic2=0.;
	controlj.az_d1=0.;
	controlj.az_d2=0.;
	controlj.az_p1_vel=0.;
	controlj.az_p2_vel=0.;
	controlj.az_ic1_vel=0.;
	controlj.az_ic2_vel=0.;
	controlj.az_d1_vel=0.;
	controlj.az_d2_vel=0.;
	
	

	controlj.alt_p1=0.;
	controlj.alt_p2=0.;
	controlj.alt_ic1=0.;
	controlj.alt_ic2=0.;
	controlj.alt_d1=0.;
	controlj.alt_d2=0.;
	controlj.alt_p1_vel=0.;
	controlj.alt_p2_vel=0.;
	controlj.alt_ic1_vel=0.;
	controlj.alt_ic2_vel=0.;
	controlj.alt_d1_vel=0.;
	controlj.alt_d2_vel=0.;
	
	
	
	//these define the absolute maximum encoder angles that the antenna will drive to
	controlj.limits[0] =  AZIMUTH_SAFETY_LO;
	controlj.limits[1] =  AZIMUTH_SAFETY_HI;
	controlj.limits[2] = ELEVATION_SAFETY_LO;
	controlj.limits[3] = ELEVATION_SAFETY_HI;
	for(i=0;i<=19;i++){
		control.pcoeffs[i]=0.;
		control.icoeffs[i]=0.;
		control.dcoeffs[i]=0.;
		control.kfcoeffs[i]=0.;
		control.vfcoeffs[i]=0.;
		control.motor_plus[i]=0;
		control.motor_minus[i]=0;
		
	}

	controlj.az_command_long = controlj.az_encoder_long;
	controlj.alt_command_long = controlj.alt_encoder_long;
	encoder2azalt(controlj.az_command_long,0.,controlj.alt_command_long,0.,&controlj.azimuth_command_double, &controlj.altitude_command_double);
	printf("Initial commands = %d %d\n", controlj.az_command_long,controlj.alt_command_long);
	for(i=0;i<=7;i++){
	      //this gives a nominal zero value- adjust later for small offsets.
		controlj.cbuffer[i]=0x8000;
		controlj.mbuffer[i]=0xffff;
	}
	
	//use these to adjust the DAC output by using small trim values
	controlj.cbuffer[0]+=0;
	controlj.cbuffer[1]+=0;
	controlj.cbuffer[2]+=0;
	controlj.cbuffer[3]+=0;
	controlj.cbuffer[4]+=185;
	controlj.cbuffer[5]-=50;
	controlj.cbuffer[6]+=50;
	controlj.cbuffer[7]-=10;
 	controlj.cbuffer_crc[0]=ad5362_crc_pack1(CREGISTER_WRITE,CH1,controlj.cbuffer[0]);
 	controlj.cbuffer_crc[1]=ad5362_crc_pack1(CREGISTER_WRITE,CH2,controlj.cbuffer[1]);
	controlj.cbuffer_crc[2]=ad5362_crc_pack1(CREGISTER_WRITE,CH3,controlj.cbuffer[2]);
	controlj.cbuffer_crc[3]=ad5362_crc_pack1(CREGISTER_WRITE,CH4,controlj.cbuffer[3]);
 	controlj.cbuffer_crc[4]=ad5362_crc_pack1(CREGISTER_WRITE,CH5,controlj.cbuffer[4]);
	controlj.cbuffer_crc[5]=ad5362_crc_pack1(CREGISTER_WRITE,CH6,controlj.cbuffer[5]);
	controlj.cbuffer_crc[6]=ad5362_crc_pack1(CREGISTER_WRITE,CH7,controlj.cbuffer[6]);
	controlj.cbuffer_crc[7]=ad5362_crc_pack1(CREGISTER_WRITE,CH8,controlj.cbuffer[7]);

	controlj.mbuffer_crc[0]=ad5362_crc_pack1(MREGISTER_WRITE,CH1,controlj.mbuffer[0]);
	controlj.mbuffer_crc[1]=ad5362_crc_pack1(MREGISTER_WRITE,CH2,controlj.mbuffer[1]);
	controlj.mbuffer_crc[2]=ad5362_crc_pack1(MREGISTER_WRITE,CH3,controlj.mbuffer[2]);
	controlj.mbuffer_crc[3]=ad5362_crc_pack1(MREGISTER_WRITE,CH4,controlj.mbuffer[3]);
 	controlj.mbuffer_crc[4]=ad5362_crc_pack1(MREGISTER_WRITE,CH5,controlj.mbuffer[4]);
 	controlj.mbuffer_crc[5]=ad5362_crc_pack1(MREGISTER_WRITE,CH6,controlj.mbuffer[5]);
	controlj.mbuffer_crc[6]=ad5362_crc_pack1(MREGISTER_WRITE,CH7,controlj.mbuffer[6]);
	controlj.mbuffer_crc[7]=ad5362_crc_pack1(MREGISTER_WRITE,CH8,controlj.mbuffer[7]);
	ioctl(fd,DEV_IOCTL_SET_DAC_REGISTERS, &controlj); 
	
	controlj.interrupt_rate = 18;
	controlj.encoder_wavelength = 10;
	read_ret = write(fd,&controlj,sizeof(controlj));
 	//controlj.alt_command_long = controlj.alt_encoder_long;
	//controlj.az_command_long = controlj.az_encoder_long;
	pthread_mutex_lock(&pid_coefficients);
	  azimuth_pid1.adaptive=0;
	  azimuth_pid2.adaptive=0;
	  altitude_pid1.adaptive=0;
	  altitude_pid2.adaptive=0;
	  azimuth_pid1.table_length=0;
	  azimuth_pid2.table_length=0;
	  altitude_pid1.table_length=0;
	  altitude_pid2.table_length=0;
	   azimuth_pid1.table_position=0;
	  azimuth_pid2.table_position=0;
	  altitude_pid1.table_position=0;
	  altitude_pid2.table_position=0;
	pthread_mutex_unlock(&pid_coefficients);
	
	pthread_mutex_lock (&mutexsum);
		control = controlj;
	pthread_mutex_unlock (&mutexsum);

}



void ramp(long *pid_return_old,long *az_pid1,long *az_pid2,long *alt_pid1,long *alt_pid2,int MAX_AZ,int MIN_AZ,int MAX_ALT,int MIN_ALT,int AZ_INTERVAL,int ALT_INTERVAL,int MAX_AZ_INTERVAL,int MAX_ALT_INTERVAL){

      long pid_return_old_az1;
      long pid_return_old_az2;
      long pid_return_old_alt1;
      long pid_return_old_alt2;
      long current_ramp_az1;
      long current_ramp_az2;
      long current_ramp_alt1;
      long current_ramp_alt2;
      
      pid_return_old_az1 = pid_return_old[0];
      pid_return_old_az2 = pid_return_old[1];
      pid_return_old_alt1 = pid_return_old[2];
      pid_return_old_alt2 = pid_return_old[3];
      
      current_ramp_az1 = *az_pid1-pid_return_old_az1;
      current_ramp_az2 = *az_pid2-pid_return_old_az2;
      current_ramp_alt1 = *alt_pid1-pid_return_old_alt1;
      current_ramp_alt2 = *alt_pid2-pid_return_old_alt2;
      
      
      if(!((current_ramp_az1 <MAX_AZ_RAMP) && (current_ramp_az1>-MAX_AZ_RAMP))){
	 // printf("AZ 1 over Ramp range- adjusting\n");
	  if(current_ramp_az1<0){
	    *az_pid1 = pid_return_old_az1 - MAX_AZ_RAMP;
	   // printf("Ramp az1 overload: Rounding down to %i\n,",*az_pid1);
	  }
	  if(current_ramp_az1>=0){
	    *az_pid1 = pid_return_old_az1 + MAX_AZ_RAMP;
	    //printf("Ramp az1 overload: Rounding up to %i\n,",*az_pid1);
	  }
	   
      }
      
      if(!((current_ramp_az2 <MAX_AZ_RAMP) && (current_ramp_az2>-MAX_AZ_RAMP))){
	  //printf("AZ 2 over Ramp range- adjusting out %i %i %i \n",pid_return_old_az2,*az_pid2,current_ramp_az2);
	   if(current_ramp_az2<0){
	    *az_pid2 = pid_return_old_az2 - MAX_AZ_RAMP;
	    //printf("Ramp az2 overload: Rounding down to %i\n,",*az_pid2);
	  }
	  if(current_ramp_az2>=0){
	    *az_pid2 = pid_return_old_az2 + MAX_AZ_RAMP;
	    //printf("Ramp az2 overload: Rounding up to %i\n,",*az_pid2);
	  }
      }
      
      if(!((current_ramp_alt1 <MAX_ALT_RAMP) && (current_ramp_alt1>-MAX_ALT_RAMP))){
	  
	   if(current_ramp_alt1<0){
	    *alt_pid1 = pid_return_old_alt1 - MAX_ALT_RAMP;
	  //  printf("Ramp alt1 overload: Rounding down to %i\n,",*alt_pid1);
	  }
	  if(current_ramp_alt1>=0){
	    *alt_pid1 = pid_return_old_alt1 + MAX_ALT_RAMP;
	    //printf("Ramp alt1 overload: Rounding up to %i\n,",*alt_pid1);
	  }
      }

      if(!((current_ramp_alt1 <MAX_ALT_RAMP) && (current_ramp_alt1>-MAX_ALT_RAMP))){
	  //printf("ALT 2 over Ramp range- adjusting\n");
	   
	   if(current_ramp_alt2<0){
	    *alt_pid2 = pid_return_old_alt2 - MAX_ALT_RAMP;
	    //printf("Ramp alt2 overload: Rounding down to %i\n,",*alt_pid2);
	  }
	  if(current_ramp_alt2>=0){
	    *alt_pid2 = pid_return_old_alt2 + MAX_ALT_RAMP;
	    //printf("Ramp alt2 overload: Rounding up to %i\n,",*alt_pid2);
	  }
      }
      
      if(!((*az_pid1<MAX_AZ) && (*az_pid1>MIN_AZ))){
	if(*az_pid1<MIN_AZ){
	  *az_pid1 =MIN_AZ;
	}
	if(*az_pid1>MAX_AZ){
	  *az_pid1 =MAX_AZ;
	}
      }
      if(!((*az_pid2<MAX_AZ) && (*az_pid2>MIN_AZ))){
	if(*az_pid2<MIN_AZ){
	  *az_pid2 =MIN_AZ;
	}
	if(*az_pid2>MAX_AZ){
	  *az_pid2 =MAX_AZ;
	}
      }
      
        if(!((*alt_pid1<MAX_ALT) && (*alt_pid1>MIN_ALT))){
	if(*alt_pid1<MIN_ALT){
	  *alt_pid1 =MIN_ALT;
	}
	if(*alt_pid1>MAX_ALT){
	  *alt_pid1 =MAX_ALT;
	}
      }
      
      if(!((*alt_pid2<MAX_ALT) && (*alt_pid2>MIN_ALT))){
	if(*alt_pid2<MIN_ALT){
	  *alt_pid2 =MIN_ALT;
	}
	if(*alt_pid2>MAX_ALT){
	  *alt_pid2 =MAX_ALT;
	}
      }
      

}

int soft_lim(struct pid_structure *control_struct,unsigned int limit_hi,unsigned int limit_lo,unsigned int limit_slow_zone_hi,unsigned int limit_slow_zone_lo,int slow_speed_hi, int slow_speed_lo,unsigned int encoder, unsigned int *command,long *pid,long *pid1,long *pid2){
	//limit hi defines the maximum allowable command angle
	//limit lo defines the minimum allowable command angle
	//limit_slow_zone_hi defines the coordinates that the antenna will drive slowly in
	//limit_slow_zone_lo define the coordinates that the antenna will drive slwoly in
	

	if(*command>limit_hi){
		*command = limit_hi;
	}
	
	if((encoder>limit_hi) && ((*command)>limit_hi)) {
		*pid = 0;
		*pid1 = 0;
		*pid2 = 0;
		*command = limit_hi;
	}
	
// 	if((encoder>limit_hi) ) {
// 		*pid = 0;
// 		*pid1 = 0;
// 		*pid2 = 0;
// 		*command = limit_hi;
// 	}
	
		//printf("LIMIT_ZONE\n");
	if(encoder >limit_slow_zone_hi) {
		if(*pid1 > slow_speed_hi){
			//printf("%d adjusting speed1\n",control_limits.alt_pid);
			//printf("adjusting speed1\n");
			*pid1 = slow_speed_hi;
			
		}
		if(*pid2 > slow_speed_hi){
			//printf("%d adjusting speed1\n",control_limits.alt_pid);
			//printf("adjusting speed1\n");
			*pid2 = slow_speed_hi;
			
		}
	}
	
	if(*command<limit_lo){
		*command = limit_lo;
	}

	if((encoder<limit_lo) && ((*command)<limit_lo)) {
		*pid = 0;
		*pid1 = 0;
		*pid2 = 0;
		*command = limit_lo;
	}

// 	if((encoder<limit_lo)) {
// 		*pid = 0;
// 		*pid1 = 0;
// 		*pid2 = 0;
// 		*command = limit_lo;
// 	}

	
	if(encoder <limit_slow_zone_lo) {
		if(*pid1 < slow_speed_lo){
			//printf("adjusting speed2\n");
			*pid1 = slow_speed_lo;
		}
		if(*pid2 < slow_speed_lo){
			//printf("adjusting speed2\n");
			*pid2 = slow_speed_lo;
		}
	}
	
}


int update_pid(char select, unsigned int command, unsigned int prev_encoder,unsigned int encoder,double p_gain, double i_gain, double d_gain,double kfgain,double vel_com,long time_diff,int MOTOR_SLACK_PLUS, int MOTOR_SLACK_MINUS,int MAX_MOTOR_SPEED_PLUS,int MAX_MOTOR_SPEED_MINUS,int MAX_I_WINDUP,int MIN_I_WINDUP, long *current_error_ptr, long *current_i_ptr,long *pid_return_ptr){

	long error1,error2,error3,error1_prev,error2_prev,error3_prev;
	long p,i,d;
	long previous_error,current_error,pid_return,previous_encoder;	
	double test1,test2,test3;
	double relative_time_diff,pd,id,dd,current_errord;
	double command_velocityff;
	double pg,ig,dg,kfg;
	previous_encoder = (long)prev_encoder; //store the previous encoder value for the D calculation
	i = *current_i_ptr; //store the integral value for calculations
	previous_error = *current_error_ptr; //store the previous error value (i.e command-position)
	p_gain=1000;
	i_gain=1;
	d_gain= 0;
 	kfgain=1;
	pg=p_gain;
	ig=i_gain;
	dg= d_gain;
 	kfg=kfgain;
	relative_time_diff = (double)time_diff/10000.;
	test1=0.;
	test2=0.;
	test3=0.;
	
	printf("PID %f %f \n",kfgain,vel_com);
	command_velocityff = kfgain*vel_com;
	//command_velocityff=0.;
	printf("ff %f %f %f \n",kfgain,vel_com,command_velocityff);
	error1 = command-encoder; //calculate the new error value
	error2 = 65535 - encoder + ( command); //in case of encoder overflow- not a problem provided the encoders do not overflow- make sure this is the case when they are installed especially with the altitude encoder- much easier!
	error3 = -(65535 - (command) +encoder);

	//turn clockwise
	
// 	if((fabs(error1) >fabs(error2)) && (fabs(error3) >fabs(error2))) {
// 	current_error = error2;
// 	//printf("Error2 AntiClockwise %f %f %f\n",fabs(error1),fabs(error2),fabs(error3);
// 	//direction=+1;
// 	}
// 	//anticlockwise
// 	else if ((fabs(error1) >fabs(error3)) && (fabs(error2) >fabs(error3))) {
// 	current_error = error3;
// 	//printf("Error3 AntiClockwise %f %f %f\n",fabs(error1),fabs(error2),fabs(error3));
// 	//direction=-1;
// 	}
// 	
// 	else if ((fabs(error2) >fabs(error1)) && (fabs(error3) >fabs(error1))) {
// 	current_error = error1;
// 	//printf("Error1 AntiClockwise %f %f %f\n",fabs(error1),fabs(error2),fabs(error3));
// 	//direction=-1;
// 	}
// 	
	current_error = error1; //the current error is error1
	current_errord=(double)current_error; //convert to a double for the calculations with floating point values


	p = current_error; //store the value to be used with the p calculations
	
	//id = current_errord+(double)i*(relative_time_diff);
	if(i_gain>0.00001){
	  i = current_error + i; //add the error to the previous i value (i.e pseudo integration)
	}
	else if (i_gain<=0.00001){
	 i= 0.;
	}
	//printf("i = %d id=%f\n",i,id);
	//i = (long)id;
	//i = (double)current_error + (double)i*((double)time_diff/10000.);
	//check the integral windup
	if(i >MAX_I_WINDUP)
		i = MAX_I_WINDUP;
	else if(i < MIN_I_WINDUP)
		i = MIN_I_WINDUP;

	//d = current_error - previous_error;
	//See PID-without PHD by Tim Westcottfor explanation of the D term. Suggests it is better to use derivative of the position rather than the error.
	d= ((long)encoder -previous_encoder) ;
//	d=5000;
      //calculate the actual pid values (i.e output = kp*p + ki*i - kd*d)
	test1 = p_gain * (double)p;
	test2 = i_gain*i;
	//printf("Test2 %f %d %f\n",p_gain,i,test2);
	//test2=0;
	test3 = d_gain*(double)d;
	//printf("test1 %f test2 %f test3 %f\n",test1,test2,test3);
	//if(test3!=0.)
		//printf("test1 %f test2 %f test3 %f\n",test1,test2,test3);
	pid_return = (long)test1 + (long)test2-(long)test3+command_velocityff;
	//printf("current error %d p %d i %d d %d test1 %f test2 %f test3 %f pidreturn %d\n",current_error,p,i,d,test1,test2,test3,pid_return);
	//take into account the motor slack- i.e the antenna does not begin to move until a certain voltage is reached on the output.
	if(previous_error<0 && current_error>=0){
	
		//simple anti-windup
		//if(i<0){
		//  i+=(double)MOTOR_SLACK_PLUS/i_gain;
		//}
		//need a small offset for the motor slack
		//if(pid_return<MOTOR_SLACK_PLUS){
		  //pid_return=(long)MOTOR_SLACK_PLUS;
		//}
	}
	if(previous_error>0 && current_error<=0){
		//simple anti-windup
		//if(i>0){
		 // i+=(double)MOTOR_SLACK_MINUS/i_gain;
		//}
		//if(pid_return>MOTOR_SLACK_MINUS){
		 // pid_return=(long)MOTOR_SLACK_MINUS;
		//}
	}
	
	if(current_error>0){
		pid_return+=MOTOR_SLACK_PLUS;
	}
	if(current_error<0){
		pid_return+=MOTOR_SLACK_MINUS;
	}
	//set the maximum pid value that can be output to limit maximum speed
	if(pid_return >MAX_MOTOR_SPEED_PLUS){
	  pid_return = MAX_MOTOR_SPEED_PLUS;
	}

	if(pid_return <MAX_MOTOR_SPEED_MINUS){
	  pid_return = MAX_MOTOR_SPEED_MINUS;
	}
//	printf("command %d encoder %d i %d prev error %d current error %d pid_return %d\n",command,encoder,i,previous_error,current_error,pid_return);
	//store data from this loop to the control structure
	*current_i_ptr = i;
	//*current_error_ptr = current_error;
	*current_error_ptr = current_error;
	*pid_return_ptr = pid_return;
//	control.alt_d = control.alt_err - control.alt_err_prev;
//	control.alt_pid = (pidloop.p * control.alt_p + pidloop.i*control.alt_i + pidloop.d*control.alt_d);
//	control.alt_simulation = 0.2*control.alt_pid;


	return 1;


}


int update_pid_double(char select, double command, double prev_encoder,double encoder,double p_gain, double i_gain, double d_gain,double kfgain,double vel_com,long time_diff,int MOTOR_SLACK_PLUS, int MOTOR_SLACK_MINUS,int MAX_MOTOR_SPEED_PLUS,int MAX_MOTOR_SPEED_MINUS,int MAX_I_WINDUP,int MIN_I_WINDUP, double *current_error_ptr, double *current_i_ptr,long *pid_return_ptr){

	double error1,error2,error3,error1_prev,error2_prev,error3_prev;
	double p,i,d;
	double previous_error,current_error,previous_encoder;	
	long pid_return;
	double test1,test2,test3;
	double relative_time_diff,pd,id,dd,current_errord;
	double command_velocityff;
	double pg,ig,dg,kfg;
	previous_encoder = prev_encoder; //store the previous encoder value for the D calculation
	i = *current_i_ptr; //store the integral value for calculations
	previous_error = (double)*current_error_ptr; //store the previous error value (i.e command-position)
	p_gain=1000;
	i_gain=1;
	d_gain= 0;
 	kfgain=1;
	pg=p_gain;
	ig=i_gain;
	dg= d_gain;
 	kfg=kfgain;
	relative_time_diff = (double)time_diff/10000.;
	test1=0.;
	test2=0.;
	test3=0.;
	
	printf("PID %f %f \n",kfgain,vel_com);
	command_velocityff = kfgain*vel_com;
	//command_velocityff=0.;
	printf("vel ff %f %f %f \n",kfgain,vel_com,command_velocityff);
	error1 = command-encoder; //calculate the new error value
	//printf("Error1 %lf \n",error1);
	error2 = 65535 - encoder + ( command); //in case of encoder overflow- not a problem provided the encoders do not overflow- make sure this is the case when they are installed especially with the altitude encoder- much easier!
	error3 = -(65535 - (command) +encoder);
	//printf("Error %f %f %f\n",error1,command,encoder);
	//turn clockwise
	
// 	if((fabs(error1) >fabs(error2)) && (fabs(error3) >fabs(error2))) {
// 	current_error = error2;
// 	//printf("Error2 AntiClockwise %f %f %f\n",fabs(error1),fabs(error2),fabs(error3);
// 	//direction=+1;
// 	}
// 	//anticlockwise
// 	else if ((fabs(error1) >fabs(error3)) && (fabs(error2) >fabs(error3))) {
// 	current_error = error3;
// 	//printf("Error3 AntiClockwise %f %f %f\n",fabs(error1),fabs(error2),fabs(error3));
// 	//direction=-1;
// 	}
// 	
// 	else if ((fabs(error2) >fabs(error1)) && (fabs(error3) >fabs(error1))) {
// 	current_error = error1;
// 	//printf("Error1 AntiClockwise %f %f %f\n",fabs(error1),fabs(error2),fabs(error3));
// 	//direction=-1;
// 	}
// 	
	current_error = error1; //the current error is error1
	current_errord=(double)current_error; //convert to a double for the calculations with floating point values


	p = current_error; //store the value to be used with the p calculations
	
	//id = current_errord+(double)i*(relative_time_diff);
	if(i_gain>0.00001){
	  i = current_error + i; //add the error to the previous i value (i.e pseudo integration)
	}
	else if (i_gain<=0.00001){
	 i= 0.;
	}
	//printf("i = %d id=%f\n",i,id);
	//i = (long)id;
	//i = (double)current_error + (double)i*((double)time_diff/10000.);
	//check the integral windup
	if(i >MAX_I_WINDUP)
		i = MAX_I_WINDUP;
	else if(i < MIN_I_WINDUP)
		i = MIN_I_WINDUP;

	//d = current_error - previous_error;
	//See PID-without PHD by Tim Westcottfor explanation of the D term. Suggests it is better to use derivative of the position rather than the error.
	d= (encoder -previous_encoder) ;
//	d=5000;
      //calculate the actual pid values (i.e output = kp*p + ki*i - kd*d)
	test1 = p_gain * (double)p;
	test2 = i_gain*i;
	//printf("Test2 %f %d %f\n",p_gain,i,test2);
	//test2=0;
	test3 = d_gain*(double)d;
	//printf("test1 %f test2 %f test3 %f %f\n",test1,test2,test3,i);
	//if(test3!=0.)
		//printf("test1 %f test2 %f test3 %f\n",test1,test2,test3);
	//so use this return if there is no velocity pid loop afterwards
	//pid_return = (long)test1 + (long)test2-(long)test3+command_velocityff;
	//so use this return if there is  velocity pid loop afterwards
 	pid_return = (long)test1 + (long)test2-(long)test3;
	
	//printf("current error %d p %d i %d d %d test1 %f test2 %f test3 %f pidreturn %d\n",current_error,p,i,d,test1,test2,test3,pid_return);
	//take into account the motor slack- i.e the antenna does not begin to move until a certain voltage is reached on the output.
	if(previous_error<0 && current_error>=0){
	
		//simple anti-windup
		//if(i<0){
		//  i+=(double)MOTOR_SLACK_PLUS/i_gain;
		//}
		//need a small offset for the motor slack
		//if(pid_return<MOTOR_SLACK_PLUS){
		  //pid_return=(long)MOTOR_SLACK_PLUS;
		//}
	}
	if(previous_error>0 && current_error<=0){
		//simple anti-windup
		//if(i>0){
		 // i+=(double)MOTOR_SLACK_MINUS/i_gain;
		//}
		//if(pid_return>MOTOR_SLACK_MINUS){
		 // pid_return=(long)MOTOR_SLACK_MINUS;
		//}
	}
	
	if(current_error>0){
		pid_return+=MOTOR_SLACK_PLUS;
	}
	if(current_error<0){
		pid_return+=MOTOR_SLACK_MINUS;
	}
	//set the maximum pid value that can be output to limit maximum speed
	if(pid_return >MAX_MOTOR_SPEED_PLUS){
	  pid_return = MAX_MOTOR_SPEED_PLUS;
	}

	if(pid_return <MAX_MOTOR_SPEED_MINUS){
	  pid_return = MAX_MOTOR_SPEED_MINUS;
	}
//	printf("command %d encoder %d i %d prev error %d current error %d pid_return %d\n",command,encoder,i,previous_error,current_error,pid_return);
	//store data from this loop to the control structure
	*current_i_ptr = i;
	//*current_error_ptr = current_error;
	*current_error_ptr = current_error;
	*pid_return_ptr = pid_return;
//	control.alt_d = control.alt_err - control.alt_err_prev;
//	control.alt_pid = (pidloop.p * control.alt_p + pidloop.i*control.alt_i + pidloop.d*control.alt_d);
//	control.alt_simulation = 0.2*control.alt_pid;


	return 1;


}


update_pid2_double(char select, double command, double prev_feedback,double feedback,double p_gain, double i_gain, double d_gain,double feedback_gain,long time_diff,int MAX_MOTOR_SPEED_PLUS,int MAX_MOTOR_SPEED_MINUS,int MAX_I_WINDUP,int MIN_I_WINDUP, long *current_error_ptr, double *current_i_ptr,long *pid_return_ptr){
	double error1,error2,error3,error1_prev,error2_prev,error3_prev;
	double p,i,d;
	double previous_error,current_error,previous_encoder;	
	long pid_return;
	double test1,test2,test3;
	double relative_time_diff,pd,id,dd,current_errord;
	double command_velocityff;
	double pg,ig,dg,kfg;
	//previous_encoder = prev_encoder; //store the previous encoder value for the D calculation
	i = *current_i_ptr; //store the integral value for calculations
	previous_error = (double)*current_error_ptr; //store the previous error value (i.e command-position)
	
 	
	relative_time_diff = (double)time_diff/10000.;
	test1=0.;
	test2=0.;
	test3=0.;
	

	
	
	error1 = command-feedback*feedback_gain; //calculate the new error value
	//error2 = 65535 - encoder + ( command); //in case of encoder overflow- not a problem provided the encoders do not overflow- make sure this is the case when they are installed especially with the altitude encoder- much easier!
	//error3 = -(65535 - (command) +encoder);

// 	
	current_error = error1; //the current error is error1
	current_errord=(double)current_error; //convert to a double for the calculations with floating point values


	p = current_error; //store the value to be used with the p calculations
	

	if(i_gain>0.00001){
	  i = current_error + i; //add the error to the previous i value (i.e pseudo integration)
	}
	else if (i_gain<=0.00001){
	 i= 0.;
	}

	//check the integral windup
	if(i >MAX_I_WINDUP)
		i = MAX_I_WINDUP;
	else if(i < MIN_I_WINDUP)
		i = MIN_I_WINDUP;

	
	//See PID-without PHD by Tim Westcottfor explanation of the D term. Suggests it is better to use derivative of the position rather than the error.
	d= (feedback -prev_feedback) ;

      //calculate the actual pid values (i.e output = kp*p + ki*i - kd*d)
	test1 = p_gain * (double)p;
	test2 = i_gain*i;

	test3 = d_gain*(double)d;

	pid_return = (long)test1 + (long)test2-(long)test3;

	//take into account the motor slack- i.e the antenna does not begin to move until a certain voltage is reached on the output.
	if(previous_error<0 && current_error>=0){
	
		//simple anti-windup
		//if(i<0){
		//  i+=(double)MOTOR_SLACK_PLUS/i_gain;
		//}
		//need a small offset for the motor slack
		//if(pid_return<MOTOR_SLACK_PLUS){
		  //pid_return=(long)MOTOR_SLACK_PLUS;
		//}
	}
	if(previous_error>0 && current_error<=0){
		//simple anti-windup
		//if(i>0){
		 // i+=(double)MOTOR_SLACK_MINUS/i_gain;
		//}
		//if(pid_return>MOTOR_SLACK_MINUS){
		 // pid_return=(long)MOTOR_SLACK_MINUS;
		//}
	}
	
	
	//set the maximum pid value that can be output to limit maximum speed
	if(pid_return >MAX_MOTOR_SPEED_PLUS){
	  pid_return = MAX_MOTOR_SPEED_PLUS;
	}

	if(pid_return <MAX_MOTOR_SPEED_MINUS){
	  pid_return = MAX_MOTOR_SPEED_MINUS;
	}

	//store data from this loop to the control structure
	*current_i_ptr = i;
	*current_error_ptr = current_error;
	*pid_return_ptr = pid_return;



	return 1;

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

float *allocate(size_t length){
	float *array;
	if (( array = (float*) malloc(length*sizeof(float))) == NULL)
   {  printf( "Not enough memory to allocate buffer\n");
      exit(1);
   }
   printf( "String was allocated!\n");
	return array;
}


void get_parameters(double *amprms, double *aoprms){
	struct tm time_structure,*time_ptr;
	double djutc,fdutc,djtt,djut1,stl;
	double right_ascension,declination;
	struct timeval time_struct;
	struct timezone tzp;
	double epoch;
	double polx,poly,dut,longitude,latitude,height,tambient,pressure,humidity,wavelength,troplapse;
	longitude = 27.6853/R2D; //longitude of HARTRAO in radians
	latitude = -25.8897/R2D; //latitude of HARTRAO in radians
	height = 1415.821;	//height of HARTRAO in metres
	tambient = 296.13;	//pseudo ambient temperature
	pressure = 1013.3;	//pseudo pressure
	humidity = 0.45;	//pseudo relative humidity
	wavelength = 200.0;	//wavelength greater than 200 means radio wavelengths!@
	troplapse = 0.0065;	//tropospheric lapse rate
	dut=0.13;		//UTC-UT1
	polx = 0.0;		//polar motion of HARTRAO in x
	poly = 0.0;		//polar motion of HARTRAO in y
	int i,j;

	//amprms_ptr = &amprms;
	//aoprms_ptr = &aoprms;
	right_ascension = 300.567;
	declination = -25.8897;
	epoch=2000.;	
	gettimeofday(&(time_struct),&tzp);
	time_ptr = gmtime(&(time_struct.tv_sec));
	slaCldj(time_ptr->tm_year+1900,time_ptr->tm_mon+1,time_ptr->tm_mday,&djutc,&j);
	
	if(j!=0){
	printf("Error in first time conversion->djutc!\n");
	}
	slaDtf2d(time_ptr->tm_hour,time_ptr->tm_min,(double)time_ptr->tm_sec,&fdutc,&j);
	if(j!=0){
	printf("Error in second time conversion->fdutc!\n");
	}
	printf("time in seconds %lu %u\n",time,time_ptr->tm_sec);
	printf("%d %d %d %d %d %d %d\n",time_ptr->tm_year,time_ptr->tm_mon+1,time_ptr->tm_mday,time_ptr->tm_hour,time_ptr->tm_min);
	djutc+=fdutc;
	printf("djutc %f \n",djutc);
	djtt = djutc + slaDat(djutc)/86400.0;
	slaMappa( 2000., djtt, amprms );
	

	printf("Here4\n");
	slaAoppa( djutc, dut, longitude, latitude, height, polx, poly,tambient, pressure, humidity, wavelength, troplapse, aoprms);
	for(i=0;i<=21;i++){
		printf("amprms %d = %f aoprms = %f \n",i,amprms[i],aoprms[i]);
	}

}


void pidadaptive_update(unsigned int type,unsigned int encoder, unsigned int command, struct pid_coefficient_structure *pid_struct,double *pcoeff, double *icoeff, double *dcoeff,double *kfcoeff,double *vfcoeff,long *MOTOR_PLUS, long *MOTOR_MINUS,double *pcoeff_vel, double *icoeff_vel,double *dcoeff_vel){
      long error;
      int i,j,length;
      j=0;
      length = pid_struct->table_length-1;
      i = (pid_struct->table_position);
      error = abs(command-encoder);
	 
	// if(type==3){
		//printf("Altitude PID check %d %d %d\n",error,pid_struct->position_error[0],length);
	     // }
   
   
	while(error<=pid_struct->position_error[j] && (j < length)){
	   //
	   //if(type==3){
	   //printf("Error %ld %ld \n",error,pid_struct->position_error[j]);
	   //}
	    j++;
	     
	}
	
	if(j!=i){
	  pid_struct->table_position =j;
	 // printf("Adaptive PID Error %d changing to %d on table %f %f %f\n",error,pid_struct->table_position,pid_struct->p[j],pid_struct->i[j],pid_struct->d[j] );
	  *pcoeff = pid_struct->p[j];
	  *icoeff = pid_struct->i[j];
	  *dcoeff = pid_struct->d[j];
	  *kfcoeff = pid_struct->kf[j];
	  *vfcoeff = pid_struct->vf[j];
	  *pcoeff_vel = pid_struct->p_2[j];
	  *icoeff_vel = pid_struct->i_2[j];
	  *dcoeff_vel = pid_struct->d_2[j];
	  *MOTOR_PLUS = pid_struct->motor_plus[j];
	  *MOTOR_MINUS = pid_struct->motor_minus[j];
	  switch(type){
	    case 0:
	     // printf("Azimuth 1\n");
	    break;
	    case 1:
	     // printf("Azimuth 2\n");
	    break;
	    case 2:
	   //   printf("Altitude 1\n");
	    break;
	    case 3:
	  //    printf("Altitude 2\n");
	    break;
	   }
	  
	}
	
	
   //   printf("Length %d\n",pid_struct->table_length);
   //   printf("%d\n%d\n",pid_struct->position_error[i],pid_struct->position_error[i+1]);


}

void Get_angles(time_t time,double right_ascension_mean,double declination_mean,double Epoch,double *altitude,double *azimuth,double *amprms_ptr,double *aoprms_ptr){
	//function to determine what angles to send! it accepts right ascension and declination in degrees and returns a 16bit number to represent the altitude and azimuth
	struct tm *time_ptr;
	struct timeval time_struct;
	struct timezone tzp;
	int j;
	double right_ascension_rad,declination_rad;
	double djutc,fdutc,djtt,djut1,stl;
	double ra_apparent,dec_apparent;
	double polx,poly,dut,longitude,latitude,height,tambient,pressure,humidity,wavelength,troplapse;
	double aobs,zobs,hobs,dobs,robs;
	double dr1950,dd1950,p1950,v1950,dr2000,dd2000,p2000,v2000,r1950,d1950,r2000,d2000;
	double check_coords,check_coords1;
	double amprms[21],aoprms[21];
	//double dut;
 	longitude = 27.6853/R2D; //longitude of HARTRAO in radians
 	latitude = -25.8897/R2D; //latitude of HARTRAO in radians
// 	height = 1415.821;	//height of HARTRAO in metres
// 	tambient = 296.13;	//pseudo ambient temperature
// 	pressure = 1013.3;	//pseudo pressure
// 	humidity = 0.45;	//pseudo relative humidity
// 	wavelength = 200.0;	//wavelength greater than 200 means radio wavelengths!@
// 	troplapse = 0.0065;	//tropospheric lapse rate
// 	dut=0.13;		//UTC-UT1
// 	polx = 0.0;		//polar motion of HARTRAO in x
// 	poly = 0.0;		//polar motion of HARTRAO in y
	dut = 0. ;
	right_ascension_rad = right_ascension_mean/R2D;
	declination_rad = declination_mean/R2D;
//	printf("RADs ra %f dec %f \n",right_ascension_rad,declination_rad);
	dr1950 =0.0;
	dd1950 =0.0;
	p1950=0.0;
	v1950=0.0;
	dr2000=0.0;
	dd2000=0.0;
	p2000=0.0;
	v2000=0.0;
	
	if(Epoch == 1950.){
		//change from FK4 to FK5
		r1950 = right_ascension_rad;
		d1950 = declination_rad;
		dr1950 = 0.0; //proper motion in right ascension
		dd1950 = 0.0;//proper motion in declination
		p1950 =0.0; //parallax in arcseconds
		v1950 =0.0;//radial velocity +'v moving away
		slaFk425(r1950,d1950,dr1950,dd1950,p1950,v1950,&r2000,&d2000,&dr2000,&dd2000,&p2000,&v2000);
		right_ascension_rad = r2000;
		declination_rad =d2000;
	}
	 
	gettimeofday(&(time_struct),&tzp);
	 //convert from seconds since Jan 1970 to a time structure
	//time_ptr = localtime(&time);
	time_ptr = gmtime(&(time_struct.tv_sec));
	slaCldj(time_ptr->tm_year+1900,time_ptr->tm_mon+1,time_ptr->tm_mday,&djutc,&j);
	if(j!=0){
		printf("Error in first time conversion->djutc!\n");
	}
	slaDtf2d(time_ptr->tm_hour,time_ptr->tm_min,(double)time_ptr->tm_sec,&fdutc,&j);
	if(j!=0){
		printf("Error in second time conversion->fdutc!\n");
	}
//	printf("time in seconds %lu %u\n",time,time_ptr->tm_sec);
	
	djutc+=fdutc;
	
//	printf("djutc %f \n",djutc);
	
	djtt = djutc + slaDat(djutc)/86400.0;
//	printf("djtt %f \n",djtt);
	//transform mean coordinates in J2000 to apparent coordinates in J2000 epoch 2000.
	slaMapqk( right_ascension_rad, declination_rad, 0., 0., 0., 0., amprms_ptr, &ra_apparent, &dec_apparent );
	
	//slaMap(right_ascension_rad,declination_rad,0.,0.,0.,0.,2000.,djtt,&ra_apparent,&dec_apparent);
	ra_apparent = right_ascension_rad;
	dec_apparent = declination_rad;
	//printf("RA App %f DEC App %f \n",ra_apparent,dec_apparent);
	//get the correction from the web in the final version!
	djut1=djutc+(dut)/86400;
	stl=slaGmst(djut1)+longitude+slaEqeqx(djtt);
	stl = slaDranrm(stl);
	//printf("Sidereal Time %f \n",stl*R2D);
// 	
// 
// 	
	check_coords = abs(R2D*(stl-ra_apparent));
	check_coords1  = abs(R2D*(stl-ra_apparent)+360.);
//	printf("check1 %f check2 %f\n",check_coords,check_coords1);
	

 	if(check_coords < 90. || check_coords1 <90.){
		slaAoppat( djutc, aoprms_ptr );
		slaAopqk(ra_apparent,dec_apparent, aoprms_ptr, &aobs,&zobs,&hobs,&dobs,&robs);
	//slaAop(ra_apparent,dec_apparent,djutc,dut,longitude,latitude,height,polx,poly,tambient,pressure,humidity,wavelength,troplapse,&aobs,&zobs,&hobs,&dobs,&robs);
	}
	else {
		printf("Coordinate problem\n");
	}
//	slaDe2h((50./R2D),100./R2D,latitude,&aobs,&zobs);
// 	
// 
// 	
 	//printf("observed : Azimuth %f Zenith %f Hour Angle %f Declination %f Right Ascension %f\n",aobs*R2D,zobs*R2D,hobs*R2D,dobs*R2D,robs*R2D);
// 	//put azimuth into 0-2pi range
 	aobs = slaDranrm(aobs);
//	zobs = abs(zobs);
 	*altitude = 90.0 - zobs*R2D;
 	*azimuth = aobs*R2D;
	
}


int backlash(unsigned int command,unsigned int encoder,double velocity,unsigned int backlash_motor_offset,unsigned int backlash_position_offset,unsigned int maintain_position_offset1,unsigned int maintain_position_offset2,long *pid1,long *pid2){
	int hi_val,lo_val;
	int az,alt;
	int error,sign;
	error=(long)command-(long)encoder;
	//printf("Backlash\n");
	//printf("Command %d Encoder %d\n",command,encoder);
	if(velocity<0.){
	  sign = +1;
	 // printf("Backlash neg\n");
	}
	else if(velocity>0.){
	  sign = -1;
	 // printf("Backlash pos\n");
	}
	else if(velocity==0.){
	  sign =0;
	}
	error =abs(error);
	//printf("Error %d\n",error);
	if(error<backlash_position_offset){
		
		*pid2 = sign*backlash_motor_offset;
		//printf("Backlash %ld\n",*pid2);
		
	}

}

int pointing_model_correction(double Azimuth,double Altitude, unsigned int *AZ_ENCODER, unsigned int *ALT_ENCODER){


	return 0;
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


unsigned int status_query(unsigned int *return_vec){
  unsigned int max7301[5];
  unsigned int max7301RX[5];
  //this function calls a general read of the status of the mechanical switches



	//printf("Status Query\n");
	max7301[0]=0xcc00; //chip1 12-19
	max7301[1]=0xcc00;//chip2 12-19
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	*(return_vec) = max7301[2]; //chip1 12-19
	*(return_vec+1) = max7301[3]; //chip2 12-19
	
	max7301[0]=0xd400; //chip1 20-27
	max7301[1]=0xcc00;//chip2 12-19
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	//printf("A- [Chip1 P24-31 and Chip2 P12-19] Returns %04x %04x\n",max7301[2],max7301[3]);

	*(return_vec+2) = max7301[2]; //chip1 20-27
	*(return_vec+3) = max7301[3]; //chip2 12-19
	max7301[0]=0xd800; //chip1 24-31
	max7301[1]=0xd400;//chip2 20-27
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	//printf("B- [Chip1 P24-31] and Chip2 20-27] Returns %04x %04x\n",max7301[2],max7301[3]);
	*(return_vec+4) = max7301[2];//chip2 24-31
	*(return_vec+5) = max7301[3];//chip2 20-27


	max7301[0]=0xd800;//chip1 24-31
	max7301[1]=0xdc00;//chip1 28-31
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	//printf("C- [Chip1 P24-31] and Chip2 28-31] Returns %04x %04x\n",max7301[2],max7301[3]);
	*(return_vec+6) = max7301[3]; //chip2 28-31
	//printf("Status Query : \nChip1 12-19 %04x\nChip1 20-27 %04x\nChip1 P24-31 %04x\nChip2 P12-19 %04x\nChip2 P20-27 %04x\nChip2 P28-31 %04x\n",*(return_vec),*(return_vec+2),*(return_vec+4),*(return_vec+3),*(return_vec+5),*(return_vec+6));

}

unsigned int contactors(unsigned int command){
	unsigned int max7301[5];
	unsigned int max7301RX[5];	
	
//	Contactors occupy Max7301 Chip 1 20,21,22,23 to engage- i.e turn 20,21,22,23 hi to engage the contactors
//	The feedback from the contactors is in Chip1 pin 26,30 and Chip2 pin 14 and 18- If the contactor is engaged the pins are pulled low and if the contactor is disengaged the pins are pulled high. So when contactors are engaged we expect Chip 1 24-31 to be 00000000 (0x0000) and Chip 2 12-19 to be 00000000 (0x0000) and with them disengaged we expect Chip1 24-31 to be 00100010 (0x44) and Chip 2 12-19 to be 00100010 (0x44)
//SEE THE MAX7301 DATASHEET FOR INFORMATION ABOUT HOW TO READ AND WRITE TO THE MAX7301
	
	printf("CONTACTORS OFF Return Should be \"CONTACTORS END 1 Returns 0x0044 0x0044\n");
	printf("CONTACTORS ON Return Should be \"CONTACTORS END 1 0x0000 0x0000\n");
	max7301[0]=0xd800; //chip1 24-31
	max7301[1]=0xcc00;//chip1 12-19
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	printf("CONTACTORS START 1 [Chip1 P24-31 and Chip2 P12-19] Returns %04x %04x\n",max7301[2],max7301[3]);
	
	max7301[0]=0xd800; //chip1 24-31
	max7301[1]=0xd400;//chip1 20-27
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	printf("CONTACTORS START 2 [Chip1 P24-31] and Chip2 20-27] Returns %04x %04x\n",max7301[2],max7301[3]);
	
	max7301[0]=0xd800;//chip1 24-31
	max7301[1]=0xdc00;//chip1 28-31
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	printf("CONTACTORS START 3 [Chip1 P24-31] and Chip2 28-31] Returns %04x %04x\n",max7301[2],max7301[3]);
	
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
		printf("Contactors are Engaged- Wait ~15 seconds for Warning Siren\n");
		sleep(10);
		
		printf("Warning Siren completed\n");
		
	break;

	}
	sleep(1);
	
	max7301[0]=0xd800; //chip1 24-31
	max7301[1]=0xcc00; //chip2 12-19
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	printf("CONTACTORS END 1 [Chip1 P24-31 and Chip2 P12-19] Returns  %04x %04x\n",max7301[2],max7301[3]);
	
	max7301[0]=0xd800;//chip1 24-31
	max7301[1]=0xd400;//chip2 20-27
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	printf("CONTACTORS END 2 [Chip1 P24-31 and Chip2 P20-27] Returns  %04x %04x\n",max7301[2],max7301[3]);
	
	max7301[0]=0xd800;//chip1 24-31
	max7301[1]=0xdc00;//chip2 28-31
	max7301[2]=2;
	ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
	printf("CONTACTORS END 3 [Chip1 P24-31 and Chip2 P28-31] Returns  %04x %04x\n",max7301[2],max7301[3]);
	
}


unsigned int clutchbrake(unsigned int command){
	unsigned int max7301[5];
	unsigned int max7301RX[5];	
	switch(command){
	case 0:
		printf("Clutch Brake off\n");
		max7301[0] = 0x4c00;
		max7301[1] = 0x0000;
		max7301[2]=2;
		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);

// 		max7301[0] = 0x2c00;
//  		max7301[1] = 0x0000;
//  		max7301[2]=2;
// 		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
// 		max7301[0] = 0x2d00;
//  		max7301[1] = 0x0000;
//  		max7301[2]=2;
// 		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
// 		max7301[0] = 0x2e00;
//  		max7301[1] = 0x0000;
//  		max7301[2]=2;
// 		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
// 		max7301[0] = 0x2f00;
//  		max7301[1] = 0x0000;
//  		max7301[2]=2;
// 		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
// 		max7301[0] = 0x3000;
//  		max7301[1] = 0x0000;
//  		max7301[2]=2;
// 		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
// 		max7301[0] = 0x3100;
//  		max7301[1] = 0x0000;
//  		max7301[2]=2;
// 		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	break;
	
	case 1:
		printf("Clutch Brake on\n");
 		max7301[0] = 0x4cff;
 		max7301[1] = 0x0000;
 		max7301[2]=2;
		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
		
		
// 		max7301[0] = 0x2cff;
// 		max7301[1] = 0x0000;
// 		max7301[2]=2;
// 		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301); //BRAKE AZimuth
// 		max7301[0] = 0x2dff;
//  		max7301[1] = 0x0000;
//  		max7301[2]=2;
// 		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
// 		max7301[0] = 0x2eff;
// 		max7301[1] = 0x0000;
// 		max7301[2]=2;
// 		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
// 		max7301[0] = 0x2fff;
//  		max7301[1] = 0x0000;
//  		max7301[2]=2;
// 		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
// 		max7301[0] = 0x30ff;
//  		max7301[1] = 0x0000;
//  		max7301[2]=2;
// 		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
// 		max7301[0] = 0x31ff;
//  		max7301[1] = 0x0000;
//  		max7301[2]=2;
// 		ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	break;

	}
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

	//set Chip 1 port 12-15 as output Chip 2 port 12-15 as Schmitt logic input with pullup 
	max7301[0] = 0x0b55;
	max7301[1] = 0x0bff;
	max7301[2] = 2;
 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);

	//set Chip 1 port 16-19 as output Chip 2 port 16-19 as Schmitt logic input with pullup 
	max7301[0] = 0x0c55;
	max7301[1] = 0x0cff;
	max7301[2] = 2;
 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	//set Chip 1 port 20-23 as output Chip 2 port 20-23 as Schmitt logic input with pullup 
	max7301[0] = 0x0d55;
	max7301[1] = 0x0dff;
	max7301[2] = 2;
 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	//set Chip 1 port 24-27 as as Schmitt logic input with pullup  Chip 2 port 24-27 as Schmitt logic input with pullup 
	max7301[0] = 0x0eff;
	max7301[1] = 0x0eff;
	max7301[2] = 2;
 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	//set Chip 1 port 28-31 as as Schmitt logic input with pullup Chip 2 port 28-31 as as Schmitt logic input with pullup 
	max7301[0] = 0x0fff;
	max7301[1] = 0x0fff;
	max7301[2] = 2;
 	ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	
}	
void sort(double *unsorted,double *sorted,int length){

	
	double tmp,tmp2;
	double working[10],sorting[10];
	int i,j;
	
	//i=length;
	for(i=0;i<length;i++){
			*(working+i) = *(unsorted+i);
			*(sorting+i) = *(unsorted+i);
			//printf("Starting Unsorted %d %f Sorted %f \n",i,*(working+i),*(sorting+i));
			//printf("AZ TEST SORTING %d\n",az_test2[i]);
			
		}
     // printf("Sorting Vals\n");
      for (i=0; i<length; i++) {
	//printf("Unsorted %f Sorted %f \n",*(working+i),*(sorting+i));
  			for (j=0; j<length-i; j++)
   				 if (*(sorting+j+1) < *(sorting+j)) {  /* compare the two neighbors */
    				  	tmp = *(sorting+j);         /* swap a[j] and a[j+1]      */
					tmp2 = *(working+j);
     				 	*(sorting+j) = *(sorting+j+1);
					*(working+j) = *(working+j+1);
     				 	*(sorting+j+1) = tmp;
					*(working+j+1)= tmp2;
  				}
			//printf("22 Unsorted %f Sorted %f \n",*(unsorted+i),*(sorted+i));	
			}
		
      for (i=0; i<length; i++) {
	//printf("Ending %d Unsorted %f Sorted %f \n",i,*(working+i),*(sorting+i));
	*(sorted+i) = *(sorting+i);
	}

}
