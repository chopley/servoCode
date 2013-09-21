//compile with arm-unknown-linux-gnu-gcc antenna_control.c pointing.c crc_gen.c /usr/lib/libarmcsla.a -o antenna_control -lpthread -lm
//requires an arm cross-compiler and the slalib library needs to be cross-compiled with this as well.
//Summary: This code is the SA C-BASS antenna controller program. It provides an interface between higher level control program and the low-level hardware (DACs,encoders, ADCs, digital IO). This low-level hardware is actually controller by a kernel level program (pid.ko)
//usage Notes:
//./antenna_control_sa optional_server_name
//the controller accepts an optional_server_name for a server name e.g 
//./antenna_control_sa c-bass
//when given the program will attempt to connect to a tcp server on the machine specified and will dump low-level servo data to that machine- if the server 
//is not open the program will fail to load-
//if the control process is started without the server argument then data will not be dumped



//REVISION HISTORY
//----------------------------------------
//9 August 2009
//Began to upgrade the command parser from the original ascii character- the new version will use binary structures to interchange data rather than the less flexible text based scripts.
//Under SVN revision control from February 2010

//README notes.. The GPIO layout of the MAX7301 is as follows:
//NB REMEMBER THAT THE MAX7301 RETURNS THE STATUS IN ACTIVE LOW AS WELL AS HGHEST BIT BEING MOST SIGNIFICANT
//I.E FOR A READ FROM CHIP 2 20-27 A RETURN OF 0Xf7 MEANS 
//27=1 (NOT CLOSED)
//26=1 (N/C)
//25=1 (N/C)
//24=1 (N/C)
//23=0 (CLOSED)
//22=1 (N/C)
//21=1 (N/C)
//20=1 (N/C)
//
//CHIP 1  CON5          GPIO    Description
//        12            OUT     CLUTCH/BRAKE (AZ) (Drive HI to activate the Clutch or release the Brake)
//        13            OUT     CLUTCH/BRAKE(AZ)
//        14            OUT     CLUTCH/BRAKE(AZ)
//        15            OUT     CLUTCH/BRAKE(ALT)
//        16            OUT     CLUTCH/BRAKE(ALT)
//        17            OUT     CLUTCH/BRAKE(ALT)
//        18            N/A
//        19            N/A
//        20            OUT     Contactor MC1 (AZ1) (Drive HI to engage the contactor and thus turn on the servos)
//        21            OUT     Contactor MC2 (AZ2)
//        CON6
//        22            OUT     Contactor MC3 (EL1)
//        23            OUT     Contactor MC4 (EL2)
//        24            IN      A
 //       25            IN      THR1+
 //       26            IN      MC1+            (Active LO)
 //       27            IN      MB1+
 //       28            IN      B
 //       29            IN      THR2+
   //     30            IN      MC2+            (Active LO)
   //     31            IN      MB2+

//CHIP 2  CON7          GPIO    Description
//        12            IN      C
//        13            IN      THR3+
//        14            IN      MC3+            (Active LO)
//        15            IN      MB3+
//        16            IN      D
//        17            IN      THR4+
//        18            IN      MC4+            (Active LO)
//        19            IN      MB4+
//        20            IN      
//        21            IN      
//        CON8
//        22            IN      
//        23            IN      
//        24            IN      
 //       25            IN      
 //       26            IN      
 //       27            IN      
 //       28            IN      
 //       29            IN      
   //     30            IN      
   //     31            IN      


#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
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
#include <string.h>		/* memset() */
#include <sys/time.h>		/* select() */
#include <pthread.h>
#include <stdlib.h>
//#include "command_struct.h"
//this defines the structure used to copy data between kernel and userspace- check that this file is the same in both ALWAYS!!
#include "pid.h"
#include "slalib.h"
#include "ad5362_def.h"
#include "crc_gen.h"
#include "telescope_constants.h"
#include "pointing.h"
#include "cbass_control_commands.h"	/*definition */
#include <sys/types.h>
#include <sys/wait.h>


#define PI 3.141592653589793238462643
#define R2D (180.0/PI)		/* radians to degrees */




pthread_mutex_t mutexsum;
pthread_mutex_t pid_coefficients;
pthread_mutex_t readout_lock;

void shutdownServo(void);
//function to initiate the various structure values
int init_control_struct (struct pid_structure *control_ptr);
//function to control (via software) various limit positions etc. Uses values stored in telescope_constants.h
int soft_lim (volatile struct pid_structure *control_struct,
	      unsigned int limit_hi, unsigned int limit_lo,
	      unsigned int limit_slow_zone_hi,
	      unsigned int limit_slow_zone_lo, int slow_speed_hi,
	      int slow_speed_lo, unsigned int encoder,
	      volatile unsigned int *command, volatile long *pid, long *pid1,
	      long *pid2);
//crc values for the ad5362 DAC which provides command voltages to the servo systems
unsigned int get_crc (unsigned int);
unsigned int ad5362_crc_pack (unsigned int, unsigned int, unsigned int);
unsigned int ad5362_crc_pack1 (unsigned int, unsigned int, unsigned int);

//copy the pid structure from kernel space into the userspace domain
int get_structure (int pid_handle, struct pid_structure *userspace);

int control_loop (int pid_handle, struct pid_structure *userspace);

//same as above but uses the azimuth values rather than the raw encoder values as the input to the PID loop
int update_pid_double (char select, double command, double prev_encoder,
		       double encoder, double p_gain, double i_gain,
		       double d_gain, double kfgain, double vel_com,
		       long time_diff, int MOTOR_SLACK_PLUS,
		       int MOTOR_SLACK_MINUS, int MAX_MOTOR_SPEED_PLUS,
		       int MAX_MOTOR_SPEED_MINUS, int MAX_I_WINDUP,
		       int MIN_I_WINDUP, double *current_error_ptr,
		       volatile double *current_i_ptr,
		       volatile long *pid_return_ptr);


//function to limit the acceleration of the antenna
void ramp (volatile long *pid_return_old, long *az_pid1, long *az_pid2,
	   long *alt_pid1, long *alt_pid2, int MAX_AZ, int MIN_AZ,
	   int MAX_ALT, int MIN_ALT, int AZ_INTERVAL, int ALT_INTERVAL,
	   int MAX_AZ_INTERVAL, int MAX_ALT_INTERVAL);


//function to sort a vector of double values
void sort (double *unsorted, double *sorted, int length);



//function to implement an anti-backlash when tracking. This backlash would be caused by wind. The best way to avoid it is to have one of the motors driving slightly in opposition so as to keep the gear teeth locked on both sides
int backlash (unsigned int command, unsigned int encoder, double velocity,
	      unsigned int backlash_motor_offset,
	      unsigned int backlash_position_offset,
	      unsigned int maintain_position_offset1,
	      unsigned int maintain_position_offset2, long *pid1, long *pid2);
//function to handle the dynamic PID coefficient 
void pidadaptive_update (unsigned int type, unsigned int encoder,
			 unsigned int command,
			 struct new_pid_coefficient_structure *,
			 volatile double *pcoeff, volatile double *icoeff,
			 volatile double *dcoeff, volatile double *kfcoeff,
			 volatile double *vfcoeff, volatile long *MOTOR_PLUS,
			 volatile long *MOTOR_MINUS,
			 volatile double *pcoeff_vel,
			 volatile double *icoeff_vel,
			 volatile double *dcoeff_vel);


void allPidsUpdate(struct new_pid_coefficient_structure *az1,struct new_pid_coefficient_structure *az2,struct new_pid_coefficient_structure *el1, struct new_pid_coefficient_structure *el2, volatile double *pcoeff, volatile double *icoeff, volatile double *dcoeff, volatile double *kcoeff, volatile double *vfcoeff, volatile long *MOTOR_PLUS, volatile long *MOTOR_MINUS,volatile double *pcoeff_vel,volatile double *icoeff_vel, volatile double *dcoeff_vel);


//function used to control the contactors (i.e contactors on or contactors off)
unsigned int contactors (unsigned int command);
//similarly for the clutch and brakes
unsigned int clutchbrake (unsigned int command, unsigned *sts_return);
//initiates the max7301 GPIO controller
unsigned int max7301_init (void);

//the pid loop for velocity
void velocity_pid (long vel_tacho, double vel_command, double position_error,
		   double Kfa, double Pd, double Id, double Dd, long Mx_I,
		   long Mn_I, int MAX_OUT, int MIN_OUT, long time_diff,
		   volatile double *current_i_ptr, long *velpid_out,
		   volatile long *pid_return_ptr);
void byte_to_binary (int x, char *ret);

//the status query for checking the breakers etc
unsigned int status_query (unsigned int *return_vec);

//declare the overall PID structure used to pass information between different threads.
struct pid_structure control;

int fd, sd_command;
struct sockaddr_in cliAddr1, remoteServAddr;
//the adaptive control structures for the four motors

//declare the dynamic pid coefficient structures
struct new_pid_coefficient_structure azimuth_pid1, azimuth_pid2,
  altitude_pid1, altitude_pid2, *azimuth_pid1_ptr;
struct pid_structure tcp_control_structs[5];
struct command_struct command;

volatile struct readout_struct readout;
void readoutStructUpdate(double azerr1,double alterr1, volatile struct readout_struct *readout,volatile struct pid_structure *loop,struct pid_structure *user); 
//struct command_position_struct command;

//velocity PID loop

void shutdownServo(){
	unsigned int STS_VEC[5];

        printf("Shutting down the Servo\n\n");
	//turn on the brakes in azimuth
	clutchbrake (2, STS_VEC);
	usleep(20000);
	//turn on the brakes in elevation
	clutchbrake (4, STS_VEC);
	usleep(20000);
	//release the clutches in both az an el
	clutchbrake (6, STS_VEC);
	usleep(20000);
	clutchbrake (8, STS_VEC);
	usleep(20000);
	contactors (2);
	usleep(20000);
	contactors (4);
	usleep(20000);
	clutchbrake (11, STS_VEC);
	usleep(20000);
	printf("Exiting the Servo Loop\n");
	exit(0);




}

void sigINT(int sig)
{ //this is called when the Servo is closed using ctrl-C

        printf("You have presses Ctrl-C");
	shutdownServo();	

}


void
byte_to_binary (int x, char *ret)
{
  //this function is to create the status return string that the main c-bass controller required

  char b[9];
  bzero (b, 9);
  int z;
  z = 1;
  while (z < 256)
    {
      strcat (b, ((x & z) == z) ? "1" : "0");
      z <<= 1;
    }
  strncpy (ret, b, 8);

}




void *
servlet (void *childfd) /* servlet thread */  
{			
 /*This thread handles the TCP socket established when control to the control program is required*/	
  int n, j,azZone;
  char buf[1024];
  char commands[100][100];
  float commandsd[10];		//for temporary storage of doubles commands received
  long commandsl[10];		//for temp storage of long commands
  char return_string[1024];
  char sts_return_string[1024];
  char bin_temp[20];
  unsigned int max7301[5], max7301RX[5];
  unsigned int STS_VEC[5], STS_TEMP_VEC[5];
  unsigned long STS_RET;
  struct sockaddr_in cliAddr;	/* structure to hold client's address */
  int cliLen = sizeof (cliAddr);	/* length of address     
					   //char s[100]; */
  struct hostent *hostp;	/* client host info */
  char *hostaddrp;
  struct pid_structure status, control_servlet;
  double mjd, fd;
  int IY, IM, ID, second_correction;
  int i,stat;
  struct tm *time_ptr;
  time_t timev, time_contactors_engaged;


  /* proc client's requests */
    bzero (buf, 1024);
  bzero (return_string, 1024);
  n = 1;
  printf ("Starting Servlet\n");
  n = recv ((int) childfd, buf, 1024, 0);
  if (n < 0)
    {
      printf ("Error reading from servlet socket\n");
      //error("ERROR reading from servlet socket");
    }
  while (n > 0)
    {
      //ioctl (fd, DEV_IOCTL_READ_CONTROL_STRUCTURE, &status);
      time (&timev);		//get the localtime and store this in timev
      bzero (commands[0], 100);
      bzero (commands[1], 100);
      bzero (commands[2], 100);
      bzero (commands[3], 100);
      bzero (commands[4], 100);
      bzero (commands[5], 100);
      bzero (commands[6], 100);
      bzero (commands[7], 100);
      //printf ("Received String %s\n", buf);


      sscanf (buf,
	      "%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^',']\n",
	      &commands[0], &commands[1], &commands[2], &commands[3],
	      &commands[4], &commands[5], &commands[6], &commands[7],
	      &commands[8], &commands[9], &commands[10]);
      //printf("Parsing string %s %s %s \n",commands[0],commands[1],commands[2]);

	//go through the different command options and respond as necessary
      if (!strcmp (commands[0], GAE))
	{
	  pthread_mutex_lock (&readout_lock);
	  	sprintf (return_string, "%s,%f,%f\r", GAE,
			   readout.az_ready_to_read[0], readout.alt_ready_to_read[0]);
	  pthread_mutex_unlock (&readout_lock);
	}
      else if(!strcmp(commands[0],ERR))
	{		
	
	  pthread_mutex_lock (&readout_lock);
		sprintf(return_string,"ERR,%f,%f\r",1000*readout.instantAzErr,1000*readout.instantAltErr);
	 printf("ERR,%f,%f\r",1000*readout.instantAzErr,1000*readout.instantAltErr);
	  pthread_mutex_unlock (&readout_lock);


	}	
	
      else if (!strcmp (commands[0], GIM))
	{
	  //printf("GIM command received\n");
	  j = 0;

	  pthread_mutex_lock (&readout_lock);
	  while ((readout.ready != 1) && (j <= 10))
	    {
	      if (j == 10)
		{
		  printf ("GIM Encoder Readout not ready %d-TIMEOUT\n", j);
		}
	      usleep (1000);
	      j++;
	    }

	  //put together the string to reply with
	  if (readout.ready == 1)
	    {
	      sprintf (return_string,
		       "%s,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r",
		       GIM, readout.az_ready_to_read[0],
		       readout.az_ready_to_read[1],
		       readout.az_ready_to_read[2],
		       readout.az_ready_to_read[3],
		       readout.az_ready_to_read[4],
		       readout.alt_ready_to_read[0],
		       readout.alt_ready_to_read[1],
		       readout.alt_ready_to_read[2],
		       readout.alt_ready_to_read[3],
		       readout.alt_ready_to_read[4],
		       readout.az_err_ready_to_read[0],
		       readout.az_err_ready_to_read[1],
		       readout.az_err_ready_to_read[2],
		       readout.az_err_ready_to_read[3],
		       readout.az_err_ready_to_read[4],
		       readout.alt_err_ready_to_read[0],
		       readout.alt_err_ready_to_read[1],
		       readout.alt_err_ready_to_read[2],
		       readout.alt_err_ready_to_read[3],
		       readout.alt_err_ready_to_read[4]);
	 /*     printf ("%s,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
		       GIM, readout.az_ready_to_read[0],
		       readout.az_ready_to_read[1],
		       readout.az_ready_to_read[2],
		       readout.az_ready_to_read[3],
		       readout.az_ready_to_read[4],
		       readout.alt_ready_to_read[0],
		       readout.alt_ready_to_read[1],
		       readout.alt_ready_to_read[2],
		       readout.alt_ready_to_read[3],
		       readout.alt_ready_to_read[4],
		       readout.az_err_ready_to_read[0],
		       readout.az_err_ready_to_read[1],
		       readout.az_err_ready_to_read[2],
		       readout.az_err_ready_to_read[3],
		       readout.az_err_ready_to_read[4],
		       readout.alt_err_ready_to_read[0],
		       readout.alt_err_ready_to_read[1],
		       readout.alt_err_ready_to_read[2],
		       readout.alt_err_ready_to_read[3],
		       readout.alt_err_ready_to_read[4]); */
	      readout.ready = 0;
	    }
	  else if (readout.ready != 1)
	    {
	      sprintf (return_string, "NODATA,\r");
	      readout.ready = 0;
	    }
	  pthread_mutex_unlock (&readout_lock);

	}
      else if (!strcmp (commands[0], AEL))
	{
	  printf("AEL command received\n\n");
      		printf ("Received String %s\n", buf);
	  commandsd[0] = atof (commands[1]);	//az1
	  commandsd[1] = atof (commands[2]);	//el1
	  commandsd[2] = atof (commands[3]);	//az2..
	  commandsd[3] = atof (commands[4]);	//el2
	  commandsd[4] = atof (commands[5]);	//az3
	  commandsd[5] = atof (commands[6]);	//el3
	  commandsl[0] = atof (commands[7]);	//tstart in MJD seconds since last MJD
	  commandsl[1] = atof (commands[8]);	//tend
	  //check for nan
	  j=0;
	  stat=0;
	  for(i=0;i<=5;i++){
	  j=isnan(commandsd[0]);
   	  stat=stat+j;
	  }
	  //now for the time conversions!!
	  //time(&timev); //get the localtime and store this in timev
	  //printf ("Local Linux Time %ld \n", timev);
	  time_ptr = gmtime (&timev);	//convert to gmtime and store in time_ptr
	  //printf ("GMT Linux Time %ld \n", timev);
	  slaCldj (time_ptr->tm_year + 1900, time_ptr->tm_mon + 1,
		   time_ptr->tm_mday, &mjd, &j);
	  if (j != 0)
	    {
	      printf
		("Unnacceptable conversion to MJD in AEL string ovro servlet\n");
	    }
	  //printf ("MJD for today is %f \n", mjd);
	  slaDtf2d (time_ptr->tm_hour, time_ptr->tm_min,
		    (double) time_ptr->tm_sec, &fd, &j);
	  if (j != 0)
	    {
	      printf
		("Unnacceptable conversion to MJD in AEL string ovro servlet\n");
	    }
	  second_correction = fd * 86400;
	/*  printf("Seconds since MJD calculated with local time %d Sent %ld %ld\n",
	     second_correction, commandsl[0], commandsl[1]);*/
	  second_correction = commandsl[0] - fd * 86400;	//calculate a potential latency in the command sent  
	  control.eq2_time_begin = timev + second_correction;
	  //second_correction = commandsl[1] - fd*86400;
	  control.eq2_time_end =
	    control.eq2_time_begin + (commandsl[1] - commandsl[0]);
	  control.coordinate_command_type = HORIZONTAL_LIST;
	  //if(stat=0){ //check for nan
	  command.comaz[0] = commandsd[0];
	  command.comalt[0] = commandsd[1];
	  command.comaz[1] = commandsd[2];
	  command.comalt[1] = commandsd[3];
	  command.comaz[2] = commandsd[4];
	  command.comalt[2] = commandsd[5];
	  //}
	  //printf("Time start %ld Time end %ld \n",control.eq2_time_begin,control.eq2_time_end);
	  sprintf (return_string, "%s\r", AEL);
	}
      else if (!strcmp (commands[0], STS))
	{
	  //printf("STS command received\n");
	  j = 0;
	  //I go through the loop at least twice to make sure the return is self-consistent- I print an error message if this requires more than two loops
	 
	  pthread_mutex_lock (&readout_lock);
		azZone=readout.azZone;
	  pthread_mutex_unlock (&readout_lock);
	
	 do
	    {
	      j++;
	      strcpy (sts_return_string, return_string);
	      STS_VEC[0] = 0;
	      STS_VEC[1] = 0;
	      STS_VEC[2] = 0;
	      STS_VEC[3] = 0;
	      STS_TEMP_VEC[0] = 0;
	      STS_TEMP_VEC[1] = 0;
	      STS_TEMP_VEC[2] = 0;
	      STS_TEMP_VEC[3] = 0;
	      max7301[0] = 0;
	      max7301[1] = 0;
	      max7301[2] = 0;
	      max7301[3] = 0;
	      clutchbrake (20, STS_VEC);	//this function returns a STS_VEC by default
	      //printf("STS_VEC return from clutchbrake %04x %04x %04x %04x %04x\n",STS_VEC[0],STS_VEC[1],STS_VEC[2],STS_VEC[3],STS_VEC[4]);
	      bzero (return_string, 1024);
	      sprintf (return_string, "STS,");
	      //here we change the binary format into a string and repeatedly store in the bin_temp string space
	      byte_to_binary (STS_VEC[0], bin_temp);
	      strcat (return_string, bin_temp);
	      byte_to_binary (STS_VEC[1], bin_temp);
	      strcat (return_string, bin_temp);
	      byte_to_binary (STS_VEC[2], bin_temp);
	      bin_temp[2] = '\0';	//end the string here i.e allow 2 bits in the last status return
	      strcat (return_string, bin_temp);
	      sprintf (bin_temp, "%d", azZone);
	      strcat (return_string, bin_temp);
	      if ((timev - time_contactors_engaged) < 15)
		{
		  sprintf (bin_temp, "%d", 1);	//need to report that the contactors were started less than 15 seconds ago
		}
	      else
		sprintf (bin_temp, "%d", 0);	//need to report that the contactors were started more than 15 seconds ago

	      strcat (return_string, bin_temp);
	      strcat (return_string, "\r");	//we need a carriage return at the end of the strings to parse on the far side
	      //now we have a string of the following form: A THR1 MC1 MCB1 B THR2 MC2 MCB2 C THR3 MC3 MCB3 D THR4 MC4 MCB4 BRAKES DRIVE_LIDS AZ_ZONE CONTACTOR TIMER- these are made up of 1's and zeros with A,THR# having 0 as standard operating return, MC# and MCB# having 1 for disengaged (i.e not operating value) and 0 for engaged (i.e operating values), BRAKE has 0 for disengaged (i.e operating value) and 1 for engaged (i.e don't drive telescope), drive_lids is 0 for operating value (lids are closed) and 1 when one of the lids is open, and azimuth zone is the kernel azimuth zone which records which zone the antenna is in (0,1,2)- there is a final value which determines whether the contactor turn on was issues in the last 15 seconds or not- 0 for more than 15 seconds and 1 for less than 15 seconds
	      if (strcmp (sts_return_string, return_string) && j >= 2)
		{
		  printf ("STS has changed j= %d\n", j);
		}

	    }
	  while (j < 5 && strcmp (return_string, sts_return_string));

	}
      else if (!strcmp (commands[0], ABF))
	{			//NOTE THAT BRAKES OFF IS WHAT WE WANT TO DO DURING INITIALISING
	  //az brake on
	  //printf("ABn\n");
	  clutchbrake (3, STS_VEC);
	  sprintf (return_string, "%s\r", ABF);
	}
      else if (!strcmp (commands[0], ABN))
	{
	  //az brake off
	  //printf("ABn\n");
	  clutchbrake (2, STS_VEC);
	  sprintf (return_string, "%s\r", ABN);
	}
      else if (!strcmp (commands[0], EBF))
	{
	  //el brake on
	  //printf("ABn\n");
	  clutchbrake (5, STS_VEC);
	  sprintf (return_string, "%s\r", EBF);
	}
      else if (!strcmp (commands[0], EBN))
	{
	  //el brake off
	  //printf("ABn\n");
	  clutchbrake (4, STS_VEC);
	  sprintf (return_string, "%s\r", EBN);
	}
      else if (!strcmp (commands[0], CLN))
	{
	  //clutches on
	  //printf("ABn\n");
	  clutchbrake (7, STS_VEC);
	  clutchbrake (9, STS_VEC);
	  sprintf (return_string, "%s\r", CLN);
	}
      else if (!strcmp (commands[0], CLF))
	{
	  //clutches off
	  //printf("ABn\n");
	  clutchbrake (6, STS_VEC);
	  clutchbrake (8, STS_VEC);
	  sprintf (return_string, "%s\r", CLF);
	}
      else if (!strcmp (commands[0], AMN))
	{
	  //azimuth contactors engage
	  //printf("ABn\n");
	  contactors (3);
	  sprintf (return_string, "%s\r", AMN);
	  time_contactors_engaged = timev;

	}
      else if (!strcmp (commands[0], AMF))
	{
	  //printf("AMF command received\n");
	  //printf("ABn\n");
	  //azimuth contactors disengage
	  contactors (2);
	  //ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
	  sprintf (return_string, "%s\r", AMF);
	}
      else if (!strcmp (commands[0], EMN))
	{
	  //printf("EMN command received\n");
	  //elevation contactors engage
	  //      printf("ABn\n");
	  contactors (5);
	  time_contactors_engaged = timev;
	  sprintf (return_string, "%s\r", EMN);
	}
      else if (!strcmp (commands[0], EMF))
	{
	  //printf("EMF command received\n");
	  //elevation contactors disengage
	  //printf("ABn\n");
	  contactors (4);
	  sprintf (return_string, "%s\r", EMF);
	}
      else if (!strcmp (commands[0], LPA))
	{
	  //printf("LPA command received\n");
	  //printf("ABn\n");
	  azimuth_pid1.p[0] = atof (commands[1]);
	  azimuth_pid1.i[0] = atof (commands[2]);
	  azimuth_pid1.d[0] = atof (commands[3]);
	  azimuth_pid1.kf[0] = atof (commands[4]);
	  azimuth_pid1.vf[0] = atof (commands[5]);
	  azimuth_pid1.p_2[0] = atof (commands[6]);
	  azimuth_pid1.i_2[0] = atof (commands[7]);
	  azimuth_pid1.adaptive = 1;

	  sprintf (return_string, "%s\r", LPA);
	}
      else if (!strcmp (commands[0], LPB))
	{
	  //printf("LPB command received\n");
	  //printf("ABn\n");
	  azimuth_pid2.p[0] = atof (commands[1]);
	  azimuth_pid2.i[0] = atof (commands[2]);
	  azimuth_pid2.d[0] = atof (commands[3]);
	  azimuth_pid2.kf[0] = atof (commands[4]);
	  azimuth_pid2.vf[0] = atof (commands[5]);
	  azimuth_pid2.p_2[0] = atof (commands[6]);
	  azimuth_pid2.i_2[0] = atof (commands[7]);
	  azimuth_pid2.adaptive = 1;

	  sprintf (return_string, "%s\r", LPB);
	}
      else if (!strcmp (commands[0], LPC))
	{
	  //printf("LPC command received\n");
	  //printf("ABn\n");
	  altitude_pid1.p[0] = atof (commands[1]);
	  altitude_pid1.i[0] = atof (commands[2]);
	  altitude_pid1.d[0] = atof (commands[3]);
	  altitude_pid1.kf[0] = atof (commands[4]);
	  altitude_pid1.vf[0] = atof (commands[5]);
	  altitude_pid1.p_2[0] = atof (commands[6]);
	  altitude_pid1.i_2[0] = atof (commands[7]);
	  altitude_pid1.adaptive = 1;

	  sprintf (return_string, "%s\r", LPC);
	}
      else if (!strcmp (commands[0], LPD))
	{
	  //printf("LPD command received\n");
	  //printf("ABn\n");
	  altitude_pid2.p[0] = atof (commands[1]);
	  altitude_pid2.i[0] = atof (commands[2]);
	  altitude_pid2.d[0] = atof (commands[3]);
	  altitude_pid2.kf[0] = atof (commands[4]);
	  altitude_pid2.vf[0] = atof (commands[5]);
	  altitude_pid2.p_2[0] = atof (commands[6]);
	  altitude_pid2.i_2[0] = atof (commands[7]);
	  altitude_pid2.adaptive = 1;

	  sprintf (return_string, "%s\r", LPD);
	}
      else if (!strcmp (commands[0], SVE))
	{
	  j = atoi (commands[1]);
	  switch (j)
	    {
	    case 1:
	  //    printf ("Received engage servo command\n");
	      clutchbrake (10, STS_VEC);
	      break;
	    case 0:
	   //   printf ("Received disengage servo command\n");
	      clutchbrake (11, STS_VEC);
	      break;

	    }
	  sprintf (return_string, "%s\r", SVE);
	}




      //parse the string that we've received


      //printf("server received %d bytes: %s \n", n, buf);
      //printf ("Returning a string made up like this %s \n", return_string);
      n = send ((int) childfd, return_string, 1024, 0);
      if (n < 0)
	{
	  printf ("Servlet Socket writing error\n");
	  //#error ("ERROR writing to Servlet socket");
	  close((int)childfd);
	  pthread_exit(0);
	}
      //printf("End loop childfd\n");

      bzero (buf, 1024);
      bzero (return_string, 1024);
	//no we wait for a reply from the control system
      //printf("Waiting for reply from control system...\n");
      n = recv ((int) childfd, buf, 1024, 0);
      //printf("Reply received from control system...\n");


      if (n < 0)
	{
	  printf ("Error reading from servlet socket\n");
	  close((int) childfd);
  	  printf ("Closing the Servlet thread\n");
	  pthread_exit(0);  
	//error ("ERROR reading from servlet socket");
	}

    }
  printf ("Closing the Servlet thread\n");
  close ((int) childfd);	/* close the client's channel */
  pthread_exit (0);		/* terminate the thread */
}

void *
ovro_thread (void *arg)
{
  //this thread sits on a incoming socket and listens for incoming commands- the commands are held in a common C struct which allows a simple binary transfer of the data to be done on both sides of the connection
  int sd, rc, n, read_ret, childpid, newsockfd;
  unsigned int cliLen;
  int i, ij;
  int childfd;
  struct sockaddr_in cliAddr, servAddr;
  int optval;
  struct hostent *hostp;	/* client host info */
  char *hostaddrp;
  struct new_message_parsing_struct tcp_message;
  int coordinate_type;
  char buf[1024];
  char commands[100][100];
  char return_string[1024];
  pthread_t child;
  FILE *fp;

  //define the type of communication (tcp) that will be used by the socket
  printf ("OVRO thread starting\n");
  sd = socket (AF_INET, SOCK_STREAM, 0);

  if (sd < 0)
    {
      printf (" cannot open socket \n");
      exit (1);
    }

  optval = 1;
  //set the socket options
  setsockopt (sd, SOL_SOCKET, SO_REUSEADDR,
	      (const void *) &optval, sizeof (int));

  /*
   * build the server's Internet address
   */
  //make sure that everything is clear in memory.
  bzero ((char *) &servAddr, sizeof (servAddr));


  //begin setting connection options
  servAddr.sin_family = AF_INET;
  servAddr.sin_addr.s_addr = htonl (INADDR_ANY);
  servAddr.sin_port = htons (LOCAL_SERVER_PORT_OVRO);


  //bind to the socket and prepare to listen for incoming communications
  rc = bind (sd, (struct sockaddr *) &servAddr, sizeof (servAddr));
  //error("Errror testing\n");
  if (rc < 0)
    {
      printf (": cannot bind port number %d \n", LOCAL_SERVER_PORT_OVRO);
      exit (1);
    }
  printf (": waiting for OVRO_THREAD data on port TCP %u\n",
	  LOCAL_SERVER_PORT_OVRO);
  if (listen (sd, 5) < 0)
    {				/* allow 5 requests to queue up */
      printf ("ERROR on listen");
      perror ("ERROR on listen");
    }
  cliLen = sizeof (cliAddr);

  while (1)
    {
      printf ("Listening\n");
      childfd = accept (sd, (struct sockaddr *) &cliAddr, &cliLen);
      if (childfd < 0)
	{
	  perror ("ERROR on accept");
	}

	//no we open up the servlet thread to handle the communications outside of here
      if (pthread_create (&child, NULL, servlet, (void *) childfd) != 0)
	{
	  printf ("pthread_create We create the servlet thread to handle communications with control pc \n");
	  exit (1);
	}


    }


  printf ("Exiting the OVRO loop\n");
  close (sd);
}

void *
command_thread (void *arg)
{ //this is the old control interface thread- We no use the OVRO control thread- this code is deprecated
  //this thread sits on a incoming socket and listens for incoming commands- the commands are held in a common C struct which allows a simple binary transfer of the data to be done on both sides of the connection
  int sd, rc, n, read_ret;
  unsigned int cliLen;
  int i, ij;
  unsigned int STS_VEC[5];
  double RAd, DECd, AZd, ALTd;
  double azstart, azend, elstart, elend, time;
  unsigned int AZ_ENCODER, ALT_ENCODER;
  int childfd;
  struct sockaddr_in cliAddr, servAddr;
  int optval;
  struct hostent *hostp;	/* client host info */
  char *hostaddrp;
  struct new_message_parsing_struct tcp_message;
  int coordinate_type;
  int length;
  struct pid_structure controlc;
  //define the type of communication (tcp) that will be used by the socket
  sd = socket (AF_INET, SOCK_STREAM, 0);
  if (sd < 0)
    {
      printf (" cannot open socket \n");
      exit (1);
    }

  optval = 1;
  //set the socket options
  setsockopt (sd, SOL_SOCKET, SO_REUSEADDR,
	      (const void *) &optval, sizeof (int));

  /*
   * build the server's Internet address
   */
  //make sure that everything is clear in memory.
  bzero ((char *) &servAddr, sizeof (servAddr));


  //begin setting connection options
  servAddr.sin_family = AF_INET;
  servAddr.sin_addr.s_addr = htonl (INADDR_ANY);
  servAddr.sin_port = htons (LOCAL_SERVER_PORT_STRUCT);


  //bind to the socket and prepare to listen for incoming communications
  rc = bind (sd, (struct sockaddr *) &servAddr, sizeof (servAddr));
  if (rc < 0)
    {
      printf (": cannot bind port number %d \n", LOCAL_SERVER_PORT_STRUCT);
      exit (1);
    }

  printf (": waiting for COMMAND_THREAD1 data on port TCP %u\n",
	  LOCAL_SERVER_PORT_STRUCT);
  //perror("Hello Error\n");
  while (1)
    {
      //listen!!!
      if (listen (sd, 5) < 0)	/* allow 5 requests to queue up */
	perror ("ERROR on listen");

      //do necessary housekeeping to set up the communications
      cliLen = sizeof (cliAddr);

      childfd = accept (sd, (struct sockaddr *) &cliAddr, &cliLen);
      if (childfd < 0)
	perror ("ERROR on accept");

      hostp = gethostbyaddr ((const char *) &cliAddr.sin_addr.s_addr,
			     sizeof (cliAddr.sin_addr.s_addr), AF_INET);
      if (hostp == NULL)
	perror ("ERROR on gethostbyaddr");
      hostaddrp = inet_ntoa (cliAddr.sin_addr);
      if (hostaddrp == NULL)
	perror ("ERROR on inet_ntoa\n");
      printf ("server established connection with %s (%s)\n",
	      hostp->h_name, hostaddrp);

      //first clear the incoming structure and then receive the binary data into it!
      bzero (&tcp_message, sizeof (tcp_message));
      length = 0;
      while (length < sizeof (tcp_message) && n > 0)
	{
	  n = recv (childfd, &tcp_message, sizeof (tcp_message), 0);
	  if (n < 0)
	    perror ("ERROR reading from socket");
	  length = length + n;
	  printf ("Received %d %d %d\n", n, length, tcp_message.message_size);
	}
     //printf ("server received %d bytes: %d\n", n,tcp_message.coordinate_type);

      //n = recvfrom(sd, msg, sizeof(msg), 0, (struct sockaddr *) &cliAddr, &cliLen);

      printf ("Received Structure %d\n", sizeof (tcp_message));
      n =
	send (childfd, &tcp_message.message_size,
	      sizeof (tcp_message.message_size), 0);
      if (n < 0)
	perror ("ERROR writing to socket");
      printf ("Returned %d Bytes to the control program\n", n);
      if (n != sizeof (tcp_message.message_size))
	{
	  printf ("Error with Message Size %d\n", n);
	}
      ioctl (fd, DEV_IOCTL_READ_CONTROL_STRUCTURE, &controlc);
      coordinate_type = tcp_message.coordinate_type;
      printf ("Coordinate Type %d\n", coordinate_type);
      switch (coordinate_type)
	{
	  //now we simply go through- decide the type of command and take appropriate action making sure to use mutexex to prevent conflicting variable updates
	case HORIZONTAL:
	  //an azimuth/elevation command is given
	  printf ("yippee HORIZONTAL Updated\n");
	  //commands[0] = atof(command_string1);
	  //commands[1] = atof(command_string2);
	  //coordinate_type_extra = HORIZONTAL;
	  AZd = tcp_message.fcommand_vals[0];
	  ALTd = tcp_message.fcommand_vals[1];
	  printf ("AZ %f ALT %f\n", AZd, ALTd);
	  pthread_mutex_lock (&mutexsum);
	  //read_ret = read(fd,&control,sizeof(control));

	  control.AZ_double = AZd;
	  control.ALT_double = ALTd;
	  control.coordinate_command_type = coordinate_type;
	  //read_ret = write(fd,&control,sizeof(control));
	  pthread_mutex_unlock (&mutexsum);
	  break;
	case EQUATORIAL:
	  //a RA/DEC command is given- this should be used sparingly as the onboard RA/DEC->ALT/AZ is slow due to the lack of floating point unit
	  printf ("yippee EQUATORIAL\n");
	  //commands[0] = atof(command_string1);
	  //commands[1] = atof(command_string2);
	  //coordinate_type_extra = EQUATORIAL;
	  RAd = tcp_message.fcommand_vals[0];
	  DECd = tcp_message.fcommand_vals[1];
	  printf ("RA %f DEC %f\n", RAd, DECd);
	  pthread_mutex_lock (&mutexsum);
	  //read_ret = read(fd,&control,sizeof(control));
	  control.RA_double = RAd;
	  control.DEC_double = DECd;
	  control.coordinate_command_type = coordinate_type;
	  //read_ret = write(fd,&control,sizeof(control));        
	  pthread_mutex_unlock (&mutexsum);
	  break;



	case EQUATORIAL2:
	  //This uses a set of second order polynomial coefficients to update the required ALT/AZ- these are calculated by the high level control PC and uploaded onto the ARM to allow quicker calculation of the appropriate ALT/AZ command angles
	  printf ("yippee EQUATORIAL2\n");
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

	  printf ("Tend %d T0 %d Length %d\n", tcp_message.lcommand_vals[0],
		  tcp_message.lcommand_vals[1], tcp_message.command_vals[0]);
	  printf ("a0 %lf a1 %lf a2 %lf a3 %lf b1 %lf b2 %lf b3 %lf b4 %lf\n",
		  tcp_message.fcommand_vals[0], tcp_message.fcommand_vals[1],
		  tcp_message.fcommand_vals[2], tcp_message.fcommand_vals[3],
		  tcp_message.fcommand_vals[4], tcp_message.fcommand_vals[5],
		  tcp_message.fcommand_vals[6], tcp_message.fcommand_vals[7]);



	  pthread_mutex_lock (&mutexsum);
	  //read_ret = read(fd,&control,sizeof(control));
	  control.a[0] = tcp_message.fcommand_vals[0];
	  control.a[1] = tcp_message.fcommand_vals[1];
	  control.a[2] = tcp_message.fcommand_vals[2];
	  control.a[3] = tcp_message.fcommand_vals[3];
	  control.b[0] = tcp_message.fcommand_vals[4];
	  control.b[1] = tcp_message.fcommand_vals[5];
	  control.b[2] = tcp_message.fcommand_vals[6];
	  control.b[3] = tcp_message.fcommand_vals[7];
	  //control.b[0]=tcp_message.dcommand_vals[8];
	  control.eq2_time_end = tcp_message.lcommand_vals[0];
	  control.eq2_time_begin = tcp_message.lcommand_vals[1];
	  control.coordinate_command_type = coordinate_type;
	  pthread_mutex_unlock (&mutexsum);
	  time = tcp_message.command_vals[0];
	  azstart = control.a[0];
	  azend = control.a[0] + time * (control.a[1] + time * control.a[2]);
	  elstart = control.b[0];
	  elend = control.b[0] + time * (control.b[1] + time * control.b[2]);
	  //calculate the speed to display for user
	  printf
	    ("Azimuth Start : %5.2f End : %5.2f Distance : %5.2f Velocity : %5.2f \nElevation Start : %5.2f End : %5.2f Distance : %5.2f Velocity : %5.2f\n",
	     azstart, azend, azend - azstart, (azend - azstart) / time,
	     elstart, elend, elend - elstart, (elend - elstart) / time);
	  break;

	case HORIZONTAL_LIST:
	  printf ("Horizontal Time Series control positions\n");
	  pthread_mutex_lock (&mutexsum);
	  //check if the data has come through successfully
	  if (tcp_message.lcommand_vals[0] > 0)
	    {
	      control.eq2_time_end = tcp_message.lcommand_vals[0];
	      control.eq2_time_begin = tcp_message.lcommand_vals[1];
	      control.coordinate_command_type = coordinate_type;
	      for (i = 0; i < control.eq2_time_end - control.eq2_time_begin;
		   i++)
		{
		  command.comaz[i] = tcp_message.az_commands[i];
		  command.comalt[i] = tcp_message.alt_commands[i];
		}
	    }
	  else
	    {
	      printf ("Failed to get command positions from packet\n");
	    }
	  pthread_mutex_unlock (&mutexsum);
	  for (i = 0; i < 5; i++)
	    {
	      printf ("i = %d Az %f %f Alt %f %f %ld %ld\n", i,
		      command.comaz[i], tcp_message.az_commands[i],
		      command.comalt[i], tcp_message.alt_commands[i],
		      control.eq2_time_end, control.eq2_time_begin);
	    }
	  break;

	case ENCODER_COORDS:
	  //very low level to allow driving antenna to a specific angle encoder positions (i.e uses 16bit number)
	  printf ("Encoder Coordinate\n");
	  //commands[0] = atof(command_string1);
	  //commands[1] = atof(command_string2);
	  //coordinate_type_extra = ENCODER_COORDS;

	  AZ_ENCODER = tcp_message.command_vals[0];
	  ALT_ENCODER = tcp_message.command_vals[1];
	  pthread_mutex_lock (&mutexsum);
	  //read_ret = read(fd,&control,sizeof(control));
	  control.az_command_long = AZ_ENCODER;
	  control.alt_command_long = ALT_ENCODER;
	  //azalt2encoder(azimuth_val,control.delta_az, altitude_val,control.delta_alt, &control.az_command_long, &control.alt_command_long);

	  //control.AZ_double=AZd;
	  //control.ALT_double=ALTd;
	  control.coordinate_command_type = coordinate_type;
	  //read_ret = write(fd,&control,sizeof(control));
	  pthread_mutex_unlock (&mutexsum);
	  break;

	case AZ1_PID_COEFFICIENTS_ADAPTIVE:
	  //updated the AZ1 pid coefficients with a table of pid values which can be changed at different error values

	  pthread_mutex_lock (&pid_coefficients);
	  printf ("Adaptive AZ1 PID coefficient update\n");
	  printf ("%f %f %f %d\n", tcp_message.pid_vals.i[0],
		  tcp_message.pid_vals.d[0], azimuth_pid1.table_length);
	  azimuth_pid1 = tcp_message.pid_vals;
	  printf ("AZ1 Length %d\n", azimuth_pid1.table_length);
	  azimuth_pid1.adaptive = 1;
	  i = 0;
	  while (i < azimuth_pid1.table_length)
	    {
	      printf ("Az %d %f %f %f kf %f vf %f %d %d %f %f %f\n",
		      azimuth_pid1.position_error[i], azimuth_pid1.p[i],
		      azimuth_pid1.i[i], azimuth_pid1.d[i],
		      azimuth_pid1.kf[i], azimuth_pid1.vf[i],
		      azimuth_pid1.motor_plus[i], azimuth_pid1.motor_minus[i],
		      azimuth_pid1.p_2[i], azimuth_pid1.i_2[i],
		      azimuth_pid1.d_2[i]);
	      i++;
	    }
	  pthread_mutex_unlock (&pid_coefficients);

	  break;

	case AZ2_PID_COEFFICIENTS_ADAPTIVE:
	  pthread_mutex_lock (&pid_coefficients);
	  printf ("Adaptive AZ2 PID coefficient update\n");
	  printf ("%f %f %f %d\n", tcp_message.pid_vals.i[0],
		  tcp_message.pid_vals.d[0],
		  tcp_message.pid_vals.table_length);
	  azimuth_pid2 = tcp_message.pid_vals;
	  azimuth_pid2.adaptive = 1;
	  i = 0;
	  while (i < azimuth_pid2.table_length)
	    {
	      printf ("Az %d %f %f %f kf %f vf %f %d %d %f %f %f\n",
		      azimuth_pid2.position_error[i], azimuth_pid2.p[i],
		      azimuth_pid2.i[i], azimuth_pid2.d[i],
		      azimuth_pid2.kf[i], azimuth_pid2.vf[i],
		      azimuth_pid2.motor_plus[i], azimuth_pid2.motor_minus[i],
		      azimuth_pid2.p_2[i], azimuth_pid2.i_2[i],
		      azimuth_pid2.d_2[i]);
	      i++;
	    }
	  pthread_mutex_unlock (&pid_coefficients);


	  break;

	case ALT1_PID_COEFFICIENTS_ADAPTIVE:
	  pthread_mutex_lock (&pid_coefficients);
	  printf ("Adaptive ALT1 PID coefficient update\n");
	  printf ("%f %f %f %d\n", tcp_message.pid_vals.i[0],
		  tcp_message.pid_vals.d[0],
		  tcp_message.pid_vals.table_length);
	  altitude_pid1 = tcp_message.pid_vals;
	  altitude_pid1.adaptive = 1;
	  i = 0;
	  while (i < altitude_pid1.table_length)
	    {
	      printf ("Alt1pid_ad %d %f %f %f kf %f vf %f %d %d %f %f %f\n",
		      altitude_pid1.position_error[i], altitude_pid1.p[i],
		      altitude_pid1.i[i], altitude_pid1.d[i],
		      altitude_pid1.kf[i], altitude_pid1.vf[i],
		      altitude_pid1.motor_plus[i],
		      altitude_pid1.motor_minus[i], altitude_pid1.p_2[i],
		      altitude_pid1.i_2[i], altitude_pid1.d_2[i]);
	      i++;
	    }
	  pthread_mutex_unlock (&pid_coefficients);

	  break;

	case ALT2_PID_COEFFICIENTS_ADAPTIVE:
	  pthread_mutex_lock (&pid_coefficients);
	  printf ("Adaptive ALT2 PID coefficient update\n");
	  printf ("%f %f %f %d\n", tcp_message.pid_vals.i[0],
		  tcp_message.pid_vals.d[0],
		  tcp_message.pid_vals.table_length);
	  altitude_pid2 = tcp_message.pid_vals;
	  altitude_pid2.adaptive = 1;
	  i = 0;
	  while (i < altitude_pid2.table_length)
	    {
	      printf ("Alt2 %d %f %f %f kf %f vf %f %d %d %f %f %f\n",
		      altitude_pid2.position_error[i], altitude_pid2.p[i],
		      altitude_pid2.i[i], altitude_pid2.d[i],
		      altitude_pid2.kf[i], altitude_pid2.vf[i],
		      altitude_pid2.motor_plus[i],
		      altitude_pid2.motor_minus[i], altitude_pid2.p_2[i],
		      altitude_pid2.i_2[i], altitude_pid2.d_2[i]);
	      i++;
	    }
	  pthread_mutex_unlock (&pid_coefficients);


	  break;


	case AZ1_PID_COEFFICIENTS:
	  //non-dynamic pid coefficients
	  printf ("PID AZ 1 Command Encoder Coordinate\n");
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
	  control.update = 1;
	  pthread_mutex_lock (&pid_coefficients);
	  azimuth_pid1.adaptive = 0;
	  pthread_mutex_unlock (&pid_coefficients);
	  //      control.coordinate_command_type=coordinate_type;
	  printf ("P %f I %f D %f M+ %ld M- %ld\n", control.pcoeffs[0],
		  control.icoeffs[0], control.dcoeffs[0],
		  control.motor_plus[0], control.motor_minus[0]);
	  //read_ret = write(fd,&control,sizeof(control));
	  pthread_mutex_unlock (&mutexsum);
	  break;

	case AZ2_PID_COEFFICIENTS:
	  printf ("PID AZ 2 Command Encoder Coordinate\n");
	  //commands[0] = atof(command_string1);
	  //commands[1] = atof(command_string2);
	  //coordinate_type_extra = AZ2_PID_COEFFICIENTS;
	  pthread_mutex_lock (&mutexsum);
	  //    read_ret = read(fd,&control,sizeof(control));
	  control.pcoeffs[1] = tcp_message.fcommand_vals[0];
	  control.icoeffs[1] = tcp_message.fcommand_vals[1];
	  control.dcoeffs[1] = tcp_message.fcommand_vals[2];
	  control.motor_plus[1] = tcp_message.command_vals[0];
	  control.motor_minus[1] = tcp_message.command_vals[1];
	  //    control.coordinate_command_type=coordinate_type;
	  control.update = 1;
	  pthread_mutex_lock (&pid_coefficients);
	  azimuth_pid2.adaptive = 0;
	  pthread_mutex_unlock (&pid_coefficients);
	  printf ("P %f I %f D %f M+ %ld M- %ld\n", control.pcoeffs[1],
		  control.icoeffs[1], control.dcoeffs[1],
		  control.motor_plus[1], control.motor_minus[1]);
	  //read_ret = write(fd,&control,sizeof(control));
	  //read_ret = write(fd,&control,sizeof(control));
	  pthread_mutex_unlock (&mutexsum);
	  break;

	case ALT1_PID_COEFFICIENTS:
	  printf ("PID ALT 1 Command Encoder Coordinate\n");
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
	  control.update = 1;
	  pthread_mutex_lock (&pid_coefficients);
	  altitude_pid1.adaptive = 0;
	  pthread_mutex_unlock (&pid_coefficients);
	  //      control.coordinate_command_type=coordinate_type;
	  printf ("P %f I %f D %f M+ %ld M- %ld\n", control.pcoeffs[2],
		  control.icoeffs[2], control.dcoeffs[2],
		  control.motor_plus[2], control.motor_minus[2]);
	  //read_ret = write(fd,&control,sizeof(control));
	  //read_ret = write(fd,&control,sizeof(control));
	  pthread_mutex_unlock (&mutexsum);
	  break;

	case ALT2_PID_COEFFICIENTS:
	  printf ("PID ALT 2 Command Encoder Coordinate\n");
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
	  control.update = 1;
	  pthread_mutex_lock (&pid_coefficients);
	  altitude_pid2.adaptive = 1;
	  pthread_mutex_unlock (&pid_coefficients);
	  //      control.coordinate_command_type=coordinate_type;
	  printf ("P %f I %f D %f M+ %ld M- %ld\n", control.pcoeffs[3],
		  control.icoeffs[3], control.dcoeffs[3],
		  control.motor_plus[3], control.motor_minus[3]);
	  //read_ret = write(fd,&control,sizeof(control));
	  //read_ret = write(fd,&control,sizeof(control));
	  pthread_mutex_unlock (&mutexsum);
	  break;

	case IOCTL_COMMANDS:
	  //handles IO control commands to various hardware devices- these are defined in pid.h
	  printf ("IOCTL COMMAND %04x\n");
	  //      pthread_mutex_lock (&mutexsum);
//              read_ret = read(fd,&control,sizeof(control));
	  //read_ret = read(fd,&control,sizeof(control));
	  //printf("Change DAC State\n");
	  //printf("Control.DAC_Output = %d\n",control.DAC_Output);
	  //control.DAC_Output = atoi(command_string1);

	  ioctl (fd, tcp_message.command_vals[0], &controlc);
	  //      printf("Control.DAC_Output = %d\n",control.DAC_Output);
	  //read_ret = read(fd,&control,sizeof(control));
	  //      printf("Control.DAC_Output = %d\n",control.DAC_Output);
	  //      pthread_mutex_unlock (&mutexsum);
	  break;

	case SAFE_DRIVE_LIMITS:
	  printf ("Change the Safety Zone Drive Limits\n");
	  pthread_mutex_lock (&mutexsum);
	  control.limits[0] = tcp_message.command_vals[0];
	  control.limits[1] = tcp_message.command_vals[1];
	  control.limits[2] = tcp_message.command_vals[2];
	  control.limits[3] = tcp_message.command_vals[3];
	  control.update = 1;
	  //control.coordinate_command_type=coordinate_type;
	  pthread_mutex_unlock (&mutexsum);
	  break;

	case CONTACTORS:
	  //int_commands[0] = atoi(command_string1);
	  printf ("WORK WITH CONTACTORS %d\n", tcp_message.command_vals[0]);

	  contactors (tcp_message.command_vals[0]);
	  break;

	case CLUTCHBRAKE:
	  //int_commands[0] = atoi(command_string1);
	  printf ("WORK WITH CLUTCHBRAKE %d\n", tcp_message.command_vals[0]);
	  printf
	    ("Disabling DAC Since working with Contactors- must be restarted Manually\n");
	  ioctl (fd, DEV_IOCTL_DISABLE_DAC, &controlc);
	  clutchbrake (tcp_message.command_vals[0], STS_VEC);
	  break;

	default:
	  printf ("Failed to get correct case here\n");
	  break;

	}




//      pthread_mutex_lock (&mutexsum);

      pthread_mutex_lock (&mutexsum);






//      read_ret = write(fd,&controlc,sizeof(controlc));



      printf ("regoing through thread control.coordinate_command_type %d\n",
	      control.coordinate_command_type);
      printf ("RA %f DEC %f AZ %f ALT %f %d %d\n", RAd, DECd, AZd, ALTd,
	      control.az_command_long, control.alt_command_long);
      pthread_mutex_unlock (&mutexsum);
      printf ("server closing connection with %s (%s) %u\n",
	      hostp->h_name, hostaddrp, LOCAL_SERVER_PORT_STRUCT);


      close (childfd);
      printf ("server closed connection with %s (%s) %u\n",
	      hostp->h_name, hostaddrp, LOCAL_SERVER_PORT_STRUCT);

    }
  printf ("Outside loop in the Command_thread\n");






};

int
main (int argc, char *argv[])
{
  //this routine starts everything up and then hands control over to the threads and the control loop which is handled in the getstructure() function
  unsigned long time_diff;
  unsigned long loop_time;
  struct pid_structure userspace;
  pthread_t writer_id;
  pthread_t command_id;
  pthread_t ovro_thread_id;
  pthread_t coordinate_update_id;
  pthread_t pidloop_id;
  struct sigaction new_action;
  char udpbuffer[10000];
  struct hostent *h;
  int read_ret, rc, i;
  unsigned int DAC_VAL;

  fd = open ("/dev/pid", O_RDWR);
  if (fd == -1)
    {
      perror ("failed to open PID Control Module\n");
      //rc = rd;
      exit (-1);
    }
  init_control_struct (&control);
  ioctl (fd, DEV_IOCTL_DISABLE_DAC, &control);

//here I'm trying to establish a routine that is run when the programme exits
  new_action.sa_handler = sigINT;
  sigemptyset (&new_action.sa_mask);
  new_action.sa_flags = 0;
  if(sigaction (SIGINT, &new_action, NULL)==-1)
                printf("Error with signal handler\n");
////////////////

	
//  if (argc < 2)
//    {
//      printf ("usage : %s <server> <data1> ... <dataN> \n", argv[0]);
//      exit (1);
//    }

  if(argc==2){
  	h = gethostbyname (argv[1]);
  	if (h == NULL)
    	{
      		printf ("%s: unknown host '%s' \n", argv[0], argv[1]);
      		exit (1);
    	}

  	printf ("%s: sending data to '%s' (IP : %s) \n", argv[0], h->h_name,
	  inet_ntoa (*(struct in_addr *) h->h_addr_list[0]));

  	remoteServAddr.sin_family = h->h_addrtype;
  	memcpy ((char *) &remoteServAddr.sin_addr.s_addr,
	  h->h_addr_list[0], h->h_length);
  	remoteServAddr.sin_port = htons (REMOTE_SERVER_PORT);

  	/* socket creation */
  	sd_command = socket (AF_INET, SOCK_STREAM, 0);
  	if (sd_command < 0)
    	{
      		printf ("%s: cannot open socket \n", argv[0]);
      		exit (1);
    	}

  	/* bind any port */
  	cliAddr1.sin_family = AF_INET;
  	cliAddr1.sin_addr.s_addr = htonl (INADDR_ANY);
  	cliAddr1.sin_port = htons (0);


  	if (connect
      		(sd_command, (struct sockaddr *) &remoteServAddr,
       		sizeof (remoteServAddr)) < 0)
    		error ("ERROR connecting");

	}
  //sprintf(udpbuffer,"Hello\n");
//      rc = write(sd_command, udpbuffer, strlen(udpbuffer));
//      
//              if(rc<0) {
//                      printf("%s: cannot send data %d \n",argv[0],i-1);
//                      close(sd_command);
//                      exit(1);
//              }



  DAC_VAL = ad5362_crc_pack (XREGISTER_WRITE, ALL, 0);
  printf ("DAC_VAL %ld\n", DAC_VAL);

  max7301_init ();
  pthread_mutex_init (&mutexsum, NULL);
  pthread_mutex_init (&pid_coefficients, NULL);
  pthread_mutex_init (&readout_lock, NULL);
  //pthread_create(&writer_id, NULL, writer_thread, NULL); 
  //pthread_create(&coordinate_update_id, NULL, coordinate_thread, NULL);
  pthread_create (&command_id, NULL, command_thread, NULL);
  pthread_create (&ovro_thread_id, NULL, ovro_thread, NULL);
  printf("Starting the signal handler\n");  
  printf("Started the signal handler\n");

  loop_time = 5;

  printf ("Hello66\n");
  read_ret = control_loop (fd, &userspace);
  printf ("Goodbye66\n");


  pthread_mutex_destroy (&mutexsum);
  pthread_mutex_destroy (&pid_coefficients);
  pthread_mutex_destroy (&readout_lock);



  close (fd);
  printf("Closed everything\n");

}
















//this is the subroutine where its all at- any editing should be done here
int
control_loop (int pid_handle, struct pid_structure *userspace)
{
  int read_ret;			//define handle for the read return function
  int fd2;
  int azimuth_double_zone = 0;
  unsigned int prev_azencoder, prev_altencoder;
  double azerr1, alterr1;
  double tachoaz1_prev, tachoaz2_prev, tachoalt1_prev, tachoalt2_prev;
  int aztacho1, aztacho2, alttacho1, alttacho2;
  long aztacho1v[20], aztacho2v[20], alttacho1v[20], alttacho2v[20];
  double fir[20];
  long aztacho1d, aztacho2d, alttacho1d, alttacho2d;
  struct pid_structure user, init;
  volatile struct pid_structure loop;
  long pid_return_old[4];
  long pid_return_new[4];
  long tacho_old[4], tacho_new[4];
  char buffer[5000];
  char cnt[20000];
  char udpcat[1000000];
  char *udpcatptr;
  char udpbuf[1000000];
  int counter;
  int pointing_counter;
  int packets;
  int firlen;
  int err_count_alt, err_count_az;
  long time_diff;
  int countera = 0;
  int counterb = 0;
  unsigned long time_diff2;
  int test_dac = 1;
  int rc, i, j;
  unsigned int DAC_val;
  double delta_az, delta_alt;
  double prev_azencoderd, prev_altencoderd;
  time_t current_time,oldTime;
  struct timeval oldTimeVal,currentTimeVal;
  unsigned int status_vec[20];
  double az_vel_vec[5], alt_vel_vec[5], az_acc_vec[5], alt_acc_vec[5],
    az_pos_vec[5], alt_pos_vec[5];
  double prev_azencoder_pos[20];
  double azencoder_vel[20], altencoder_vel[20];
  extern int sd_command;
  double tdouble, tdouble1, td;
  double avevel_az[10], avevel_alt[10], avevel_azsorted[10],
    avevel_altsorted[10];
  double pos_az = 0, pos_alt = 0;
  double pos_az_past = 0, pos_alt_past = 0;
  double vel_az = 0, vel_alt = 0;
  double vel_az_past = 0, vel_alt_past = 0;
  double accel_az = 0, accel_alt = 0;
  double accel_az_past = 0, accel_alt_past = 0;
  double altitude_val = 0, azimuth_val = 0;
  double azimuth_val_uncorrected,altitude_val_uncorrected;
  double azerr_uncorrected,alterr_uncorrected;
  unsigned int az_test[5];
  unsigned int az_temp;
  long az_test2[5];
  long velpid_out_az, velpid_out_alt;
  long aztacho[5], alttacho[5];
  double azlim[6], ellim[5];
  double kfa_azimuth, kfa_altitude;
  double temp_double;
  double temporary_double[5];
  double pos_az_dot, pos_alt_dot;
  //initialise the readout structure
  readout.sample_rate = 200000;	//sample rate of the interpolated angle positions
  readout.current_value = 0;
  readout.sample_number = 0;
  readout.samples_per_second = 5;
  //
  //azimuth_pid1_ptr=&azimuth_pid1;               
  printf ("Starting\n");

  user = *userspace;
  fd2 = pid_handle;
  read_ret = read (fd2, &init, sizeof (init));
  loop = control;

  printf ("Command Angles LOOP AZ %d ALT %d\n", loop.az_command_long,
	  loop.alt_command_long);
  printf ("%d %d %d %d %d %d %d %d\n", (user.tacho1), (user.tacho2),
	  (user.tacho3), (user.tacho4), (user.azimuth_zone));
  printf ("HERE");
  err_count_alt = 5000;
  if (test_dac == 1)
    {
      //this is the start of the routine.
      //first zero all necessary variables.
      packets = 0;
      counter = 0;
      delta_az = 0;
      delta_alt = 0;
      pointing_counter = 0;
      //fir tap filter coefficients
      for (j = 0; j <= 10; j++)
	{
	  fir[j] = 0.2;

	}

      for (j = 0; j <= 19; j++)
	{
	  aztacho1v[j] = 0;
	  aztacho2v[j] = 0;
	  alttacho1v[j] = 0;
	  alttacho2v[j] = 0;
	}
      firlen = 1;

//LOOP BEGINS HERE!!-------------------------------         
      //initialise the temporary command position doubles
      azimuth_val = loop.azimuth_command_double;
      altitude_val = loop.altitude_command_double;
      oldTime=0;
      gettimeofday(&currentTimeVal,NULL);
      gettimeofday(&oldTimeVal,NULL);
      while (1)
	{			//probably don't need this while statement anymore? Can just be while(1) ?

	  //increment relevent counters
	  packets++;
	  counter++;
	  user.counter = 0;
	  //save the time from the last loop so that a value for the 'tick' interval can be estimated accurately
	  time_diff = user.time_struct.tv_usec;
	  prev_azencoder = (unsigned int) user.az_encoder_long;
	  prev_altencoder = (unsigned int) user.alt_encoder_long;
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
	  azencoder_vel[6] = 1000 * 100 * (prev_azencoder_pos[6] - prev_azencoder_pos[0]) / 6;	//in deg/s
//          //azencoder_vel[7] = (prev_azencoder_pos[6]-prev_azencoder_pos[0])/(6*0.011);
//          
//          
	  azencoder_vel[7] = (azencoder_vel[6] + 0.5 * azencoder_vel[5] + 0.25 * azencoder_vel[4] + 0.25 * azencoder_vel[3] + 0.1 * azencoder_vel[2] + 0.1 * azencoder_vel[1] + 0.1 * azencoder_vel[0]) / 2.3;	//in mdeg/s
//------------------------------------------------------------------------          
	  tachoaz1_prev = (double) aztacho1;
	  tachoaz2_prev = (double) aztacho2;
	  tachoalt1_prev = (double) alttacho1;
	  tachoalt2_prev = (double) alttacho2;

	pos_az_past=loop.azimuth_command_double;
	pos_alt_past=loop.altitude_command_double;
	vel_az_past=vel_az;
	vel_alt_past=vel_alt;
	accel_az_past=accel_az;
	accel_alt_past=accel_alt;
	  //read from the kernel

	  read_ret = read (fd, &user, sizeof (user));	//THIS IS WHERE THE TICK INTERVAL IS IMPLEMENTED. The kernel blocks the read and this is released by an onboard timer overflow interrupt routine. This gives an consistent timing mechanism to the pid control loop.
	  //tacho offsets as there is a slight negative bias here!

	  //user.tacho1+=88;
	  //user.tacho2+=41;
	  //user.tacho3+=62;
	  //user.tacho4+=68;


	  for (j = 0; j <= firlen - 1; j++)
	    {
	      aztacho1v[firlen - j] = aztacho1v[firlen - j - 1];
	      aztacho2v[firlen - j] = aztacho2v[firlen - j - 1];
	      alttacho1v[firlen - j] = alttacho1v[firlen - j - 1];
	      alttacho2v[firlen - j] = alttacho2v[firlen - j - 1];
	    }

	  aztacho1v[0] = (long) user.tacho2;
	  aztacho2v[0] = (long) user.tacho3;
	  alttacho1v[0] = (long) user.tacho4;
	  alttacho2v[0] = (long) user.tacho1;
//         

	  aztacho1d = 0;
	  aztacho2d = 0;
	  alttacho1d = 0;
	  alttacho2d = 0;

	  for (j = 0; j <= firlen - 1; j++)
	    {

	      aztacho1d += aztacho1v[j];
	      aztacho2d += aztacho2v[j];
	      alttacho1d += alttacho1v[j];
	      alttacho2d += alttacho2v[j];
	    }
	  aztacho1 = aztacho1d / firlen;
	  aztacho2 = aztacho2d / firlen;
	  alttacho1 = alttacho1d / firlen;
	  alttacho2 = alttacho2d / firlen;

	  aztacho1 += 45;
	  aztacho2 += 40;




	  //this routine is important as it updates the azimuth/elevation encoders from the raw encoder value to the floating point representation-the function is defined in pointing.c- the zero point of the azimuth and elevation 
	  //should be defined in telescope_constants.h if (for example) the encoders are changed. This should be done manually and is only required once.
	  encoder2azaltprime (user.az_encoder_long, delta_az,
			      user.alt_encoder_long, delta_alt,
			      &loop.azimuth_encoder_double,
			      &loop.altitude_encoder_double);




	  pos_az_dot = loop.azimuth_encoder_double - prev_azencoderd;
	  pos_alt_dot = loop.altitude_encoder_double - prev_altencoderd;
	



	  time_diff2 = user.time_struct.tv_usec;	//calculate the time interval between loops.
	  if (time_diff2 > time_diff)
	    {
	      time_diff = time_diff2 - time_diff;
	    }
	  else if (time_diff2 <= time_diff)
	    {
	      time_diff = time_diff2 + 1000000 - time_diff;
	    }




//                  //-------------TRY THIS HERE RATHER THAN IN THE COORDINATE INTERRUPT-This relies on one type of command being given to the controller-

	  if (control.coordinate_command_type == HORIZONTAL_LIST)
	{

	      tdouble1 = user.time_struct.tv_sec;
	      tdouble = user.time_struct.tv_usec;

	      time (&current_time);
	      gettimeofday(&currentTimeVal,NULL);
		if((currentTimeVal.tv_sec-oldTimeVal.tv_sec)>=1){
			printf("Next Second %ld %ld:%ld \n",(currentTimeVal.tv_sec-oldTimeVal.tv_sec),currentTimeVal.tv_sec,currentTimeVal.tv_usec);
		      gettimeofday(&oldTimeVal,NULL);
		}
	      //check if the time is still valid
	      if(countera==15){
			//printf("Current Time %ld  Time End %ld \n",current_time,control.eq2_time_end);
		}
	      if (current_time <= control.eq2_time_end)
		{
		  counterb = 0;
		  //time(&current_time);


		  td = tdouble1 + tdouble / 1000000.;
		  tdouble = (control.eq2_time_begin);
		  td = td - tdouble;
		  //save the previous command velocities in preparation to calculate the accelerations

		  //save the previous command positions in preparation to calculate the velocities
	
		    
		      //printf("Horizontal List\n"); 
		      i = current_time - control.eq2_time_begin;
		      if (i >
			  (control.eq2_time_end - control.eq2_time_begin - 1))
			{
				i =0;
				azimuth_val=loop.azimuth_command_double;
				altitude_val=loop.altitude_command_double;
		  		//loop.azimuth_command_double = azimuth_val;
		  		//loop.altitude_command_double = altitude_val;
		    		loop.vel_of_az = 0;
		    		loop.vel_of_alt = 0;
				printf("Encoder positions not received in time %i %ld %ld\n",i,control.eq2_time_end,control.eq2_time_begin);
			}
			else
			{
		      		td = user.time_struct.tv_usec / 1000000.;
		    	        temporary_double[0] = azimuth_val;
		      		temporary_double[1] = altitude_val;
		      		//correct for maximum allowable command speeds
		      			if(countera==20){
						//printf("command %f %f \n",command.comaz[i],command.comaz[i+1]);
					}
					
				azimuth_val =
					command.comaz[i] + td * (command.comaz[i + 1] -
						 command.comaz[i]);
		      		altitude_val =
					command.comalt[i] + td * (command.comalt[i + 1] -
						  command.comalt[i]);
				azimuth_val_uncorrected=azimuth_val;
				altitude_val_uncorrected=altitude_val;
				azerr_uncorrected = azimuth_val_uncorrected-loop.azimuth_encoder_double;
				alterr_uncorrected = altitude_val_uncorrected-loop.altitude_encoder_double;
					      		if (azimuth_val > MAX_AZ_VAL_FLOAT)
				{
			  		while (azimuth_val > MAX_AZ_VAL_FLOAT)
			    		{
			      			azimuth_val -= 360;
			      			printf ("reducing AZ VAL %f\n", azimuth_val);
			    		}
			  		//printf("Wrap issue\n");
				}
		      		if (azimuth_val < MIN_AZ_VAL_FLOAT)
				{
			  		while (azimuth_val < MIN_AZ_VAL_FLOAT)
			    		{
			      			azimuth_val += 360;
			      			printf ("Increasing az Val %f azimuth_val\n");
			    		}
				}

		      		if (altitude_val > MAX_ALT_VAL_FLOAT)
				{
			  		altitude_val = MAX_ALT_VAL_FLOAT;
			  		//printf("Elevation too high\n");
				}
		      		if (altitude_val < MIN_ALT_VAL_FLOAT)
				{
			  		altitude_val = MIN_ALT_VAL_FLOAT;
			  		//printf("Elevation too low\n");
				}
				if(azimuth_val > (temporary_double[0] + MAX_AZ_POS_SPACE_DEG/100.)){
		      		//	printf("Speed limit az positive Initial new AZ %f ",azimuth_val);
		      		 	azimuth_val = temporary_double[0] + MAX_AZ_POS_SPACE_DEG/100. ;
		      		//	printf("Corrected new AZ %f \n",azimuth_val);
		      		}
		      		if(azimuth_val < (temporary_double[0] + MIN_AZ_POS_SPACE_DEG/100.)){
			      	//	printf("Speed limit az positive Initial new AZ %f ",azimuth_val);
			      		azimuth_val = temporary_double[0] + MIN_AZ_POS_SPACE_DEG/100. ;
			      	//	printf("Corrected new AZ %f \n",azimuth_val);
		      		}
		      		if(altitude_val > (temporary_double[1] + MAX_ALT_POS_SPACE_DEG/100.)){
		      			altitude_val = temporary_double[1] + MAX_ALT_POS_SPACE_DEG/100.; 
		      		}
		      		if(altitude_val < (temporary_double[1] + MIN_ALT_POS_SPACE_DEG/100.)){
		      			altitude_val = temporary_double[1] + MIN_ALT_POS_SPACE_DEG/100. ;
		      		}



		    	
				

		  	//need the software limit checked to go in this position- it needs to 
		  	//1. make sure the command encoder values do not go past this zone
		  	//2. make sure the velocity command is zeroed past this value
		  	//3. Also (preferably) define a slow zone that should prevent the elevation from going wrong and allow the antenna to be parked at zenith

		  	loop.azimuth_command_double = azimuth_val;
		  	loop.altitude_command_double = altitude_val;
		  	//Calculate the velocity of the the commands by using the previous commanded angle and the current
		  	vel_az = loop.azimuth_command_double - pos_az_past;
		  	vel_alt = loop.altitude_command_double - pos_alt_past;

			accel_az=vel_az-vel_az_past;
			accel_alt=vel_alt-vel_alt_past;
//-------------------------------------------------         



	  //put in maximum accelerations permitted- the enums are defined in telescope_constants.h
	   if ((accel_az > (double) MAX_AZ_ACCEL / 1000.)
		   || (accel_az < (double) MIN_AZ_ACCEL / 1000.))
	    {
	      printf ("Azimuth Acceleration Limit%f %f %f %f %f\n",loop.azimuth_command_double,pos_az_past,accel_az,vel_az,vel_az_past);
	      if (accel_az > (double) MAX_AZ_ACCEL / 1000.)
		{
		loop.azimuth_command_double = loop.azimuth_command_double-0.01 ;
		}
	      else if (accel_az < (double) MIN_AZ_ACCEL / 1000.)
		{
		loop.azimuth_command_double = loop.azimuth_command_double+0.01;
		}
	    }


	   if ((accel_alt > (double) MAX_ALT_ACCEL / 1000.)
		   || (accel_alt < (double) MIN_ALT_ACCEL / 1000.))
	    {
	      printf ("Altitude Acceleration Limit%f %f %f %f %f\n",loop.altitude_command_double,pos_alt_past,accel_alt,vel_alt,vel_alt_past);
	      if (accel_alt > (double) MAX_ALT_ACCEL / 1000.)
		{
		loop.altitude_command_double -=0.01 ;
		}
	      else if (accel_alt < (double) MIN_ALT_ACCEL / 1000.)
		{
		loop.altitude_command_double += 0.01;
		}
	    }
	//recalculate the command velocity///
		  	vel_az = loop.azimuth_command_double - pos_az_past;
		  	vel_alt = loop.altitude_command_double - pos_alt_past;

			accel_az=vel_az-vel_az_past;
			accel_alt=vel_alt-vel_alt_past;
		  	
		azimuth_val=loop.azimuth_command_double;
		  altitude_val=loop.altitude_command_double;
//-------------------------------------------------------------------------------       

		  	//if (vel_az > MAX_AZ_POS_SPACE_DEG)
		    	//{
		      		//azimuth_val =
				//azimuth_val - vel_az + MAX_AZ_POS_SPACE_DEG;
		    	//}

//                              

		  	loop.vel_of_az = 1000. * vel_az * 100;	//get into mdeg/s
		  	loop.vel_of_alt = 1000. * vel_alt * 100;	//get into mdeg/s

			}
//convert the command position to an encoder position
		  azalt2encoder (loop.azimuth_command_double, control.delta_az, loop.altitude_command_double,
				 control.delta_alt, &loop.az_command_long,
				 &loop.alt_command_long);
		  sort_azimuth (loop.az_encoder_long, loop.az_command_long,
				AZIMUTH_LIMIT_HI, AZIMUTH_LIMIT_LO,
				&loop.az_command_long);
			
		}
	      else
		{
		 //haven't received any command angles from the control system in nearly 15 seconds- I need to take some action 
		  	if (counterb == 0)
		    	{
		      		printf
				("Eq2 Error-Need new quadratic coefficients to maintain pointing accuracy-Counterb. Tend = %ld T= %ld\n",
			 	control.eq2_time_end, current_time);
				printf("Haven't received new command angles from the control sytstem-stopping the antenna\n");
		    	}	
		      	counterb = 1;
			azimuth_val=loop.azimuth_encoder_double;
			altitude_val=loop.altitude_encoder_double;
		  	loop.azimuth_command_double = azimuth_val;
		  	loop.altitude_command_double = altitude_val;







		    
		  	loop.vel_of_az = 0;
		  	loop.vel_of_alt = 0;
		  azalt2encoder (loop.azimuth_command_double, control.delta_az, loop.altitude_command_double,control.delta_alt, &loop.az_command_long,&loop.alt_command_long);
		  	sort_azimuth (loop.az_encoder_long, loop.az_command_long,
				AZIMUTH_LIMIT_HI, AZIMUTH_LIMIT_LO,
				&loop.az_command_long);
			
			//printf("Not talking to cbassControl for 15 seconds-SHUTDOWN THE SERVO\n");
			//shutdownServo();
		  //azimuth_angle(loop.azimuth_command_double,loop.az_encoder_long,AZIMUTH_ZERO,AZIMUTH_SAFETY_LO,AZIMUTH_SAFETY_HI,&loop.azimuth_command_double);


		}
	}
	    

	
	allPidsUpdate(&azimuth_pid1,&azimuth_pid2,&altitude_pid1,&altitude_pid2,loop.pcoeffs,loop.icoeffs,loop.dcoeffs,loop.kfcoeffs,loop.vfcoeffs,loop.motor_plus,loop.motor_minus,loop.pcoeffs_vel,loop.icoeffs_vel,loop.dcoeffs_vel);

	 	  if (packets == 5 && user.azimuth_zone <= 2)
	    {			//every 5 packets send data to the control PC
	      sprintf (udpbuf, "");
	      sprintf (udpbuf, "%s", udpcat);
	      sprintf (udpcat, "");
	      packets = 0;
		if(sd_command){
	      		rc = send (sd_command, udpbuf, strlen (udpbuf), 0);
	      		if (rc < 0)
			{
		  		printf (": cannot send data in pid loop \n");
		  		close (sd_command);
		  		exit (1);
			}
		}
	    }
	  //Save the pid return value so that the ramp clipping does not throw away valuable information.
	  pid_return_old[0] = pid_return_new[0];
	  pid_return_old[1] = pid_return_new[1];
	  pid_return_old[2] = pid_return_new[2];
	  pid_return_old[3] = pid_return_new[3];





	  //here we have a counter that can be used for slow printf statements to stop the printfs from overloading the pid loop
	  if (countera < 30)
	    {
	      countera++;

	    }
	  else if (countera >= 30)
	    {
	      countera = 0;
	    }


	  //This section calculates the PID loop depending on what speed we are tracking at 

	  aztacho[1] = aztacho1;
	  aztacho[2] = aztacho2;
	  aztacho[3] = aztacho1;	//uses this as the check against the speed zone
	  alttacho[1] = alttacho1;
	  alttacho[2] = alttacho2;
	  alttacho[3] = alttacho1;	//checks for the speed zone

	  switch ((loop.vel_of_az < 50. && loop.vel_of_az > -50.))
	    {

	    case 1:		//i.e the low velocity zone
	      //pid output calculation
	      update_pid_double (AZ_PID, loop.azimuth_command_double,
				 prev_azencoderd, loop.azimuth_encoder_double,
				 loop.pcoeffs[0], loop.icoeffs[0],
				 loop.dcoeffs[0], loop.kfcoeffs[0],
				 loop.vel_of_az, time_diff,
				 loop.motor_plus[0], loop.motor_minus[0],
				 MAX_AZ_POS_PID, MIN_AZ_POS_PID, 3000, -3000,
				 &azerr1, &loop.az_ic1, &loop.az_pid1);
	      pid_return_new[0] = loop.az_pid1;
	      pid_return_new[1] = 0.5 * loop.az_pid1;
	      aztacho[0] = (long) aztacho1;	//use the low velocity value from the ADC i.e channel 1
	      kfa_azimuth = loop.vfcoeffs[0];
	      velocity_pid (aztacho[0], (double) loop.az_pid1, azerr1,
			    kfa_azimuth, loop.pcoeffs_vel[0],
			    loop.icoeffs_vel[0], 0., 2000, -2000,
			    MAX_AZ_SPEED, MIN_AZ_SPEED, time_diff,
			    &loop.az_ic1_vel, &velpid_out_az, &loop.az_pid1);
	      pid_return_new[0] = loop.az_pid1;
	      pid_return_new[1] = 0.5 * loop.az_pid1;


	      break;

	    case 0:		//i.e the high velocity zone
	      update_pid_double (AZ_PID, loop.azimuth_command_double,
				 prev_azencoderd, loop.azimuth_encoder_double,
				 loop.pcoeffs[1], loop.icoeffs[1],
				 loop.dcoeffs[1], loop.kfcoeffs[1],
				 loop.vel_of_az, time_diff,
				 loop.motor_plus[1], loop.motor_minus[1],
				 MAX_AZ_POS_PID, MIN_AZ_POS_PID, 3000, -3000,
				 &azerr1, &loop.az_ic1, &loop.az_pid1);
	      //printf("bb pid_return %d %d\n",loop.az_pid1,loop.alt_pid1);

	      pid_return_new[0] = loop.az_pid1;
	      pid_return_new[1] = 0.5 * loop.az_pid1;

	      aztacho[0] = (long) aztacho2;
	      kfa_azimuth = loop.vfcoeffs[1];
	      velocity_pid (aztacho[0], (double) loop.az_pid1, azerr1,
			    kfa_azimuth, loop.pcoeffs_vel[1],
			    loop.icoeffs_vel[1], 0., 2000, -2000,
			    MAX_AZ_SPEED, MIN_AZ_SPEED, time_diff,
			    &loop.az_ic1_vel, &velpid_out_az, &loop.az_pid1);
	      //printf("Aztacho2 Zone %d %f\n",aztacho[0],loop.vel_of_az);

	      pid_return_new[0] = loop.az_pid1;
	      pid_return_new[1] = 0.5 * loop.az_pid1;
	      break;
	    default:
	      printf ("Error in the AZ tachometer switching\n");

	    }

	  switch ((loop.vel_of_alt < 50. && loop.vel_of_alt > -50.))
	    {

	    case 1:		//i.e the low velocity zone
	      update_pid_double (ALT_PID, loop.altitude_command_double,
				 prev_altencoderd,
				 loop.altitude_encoder_double,
				 loop.pcoeffs[2], loop.icoeffs[2],
				 loop.dcoeffs[2], loop.kfcoeffs[2],
				 loop.vel_of_alt, time_diff,
				 loop.motor_plus[2], loop.motor_minus[2],
				 MAX_ALT_POS_PID, MIN_ALT_POS_PID,
				 3000 ,
				 -3000 , &alterr1,
				 &loop.alt_ic1, &loop.alt_pid1);
	      pid_return_new[2] = loop.alt_pid1;
	      pid_return_new[3] = 0.5 * loop.alt_pid1;


	      alttacho[0] = (long) alttacho1;	//use the low velocity value from the ADC i.e channel 1
	      velocity_pid (alttacho[0], loop.alt_pid1, alterr1,
			    loop.vfcoeffs[2], loop.pcoeffs_vel[2],
			    loop.icoeffs_vel[2], 0., 2000, -2000,
			    MAX_ALT_SPEED, MIN_ALT_SPEED, time_diff,
			    &loop.alt_ic1_vel, &velpid_out_alt,
			    &loop.alt_pid1);

	      pid_return_new[2] = loop.alt_pid1;
	      pid_return_new[3] = 0.5 * loop.alt_pid1;





	      break;
	    case 0:		//i.e the high velocity zone
	      update_pid_double (ALT_PID, loop.altitude_command_double,
				 prev_altencoderd,
				 loop.altitude_encoder_double,
				 loop.pcoeffs[3], loop.icoeffs[3],
				 loop.dcoeffs[3], loop.kfcoeffs[3],
				 loop.vel_of_alt, time_diff, loop.motor_plus[3],
				 loop.motor_minus[3], MAX_ALT_POS_PID,
				 MIN_ALT_POS_PID, 3000 ,
				 -3000 , &alterr1,
				 &loop.alt_ic1, &loop.alt_pid1);
	      pid_return_new[2] = loop.alt_pid1;
	      pid_return_new[3] = 0.5 * loop.alt_pid1;


	      alttacho[0] = (long) alttacho2;
	      velocity_pid (alttacho[0], loop.alt_pid1, alterr1,
			    loop.vfcoeffs[3], loop.pcoeffs_vel[3],
			    loop.icoeffs_vel[3], 0., 2000, -2000,
			    MAX_ALT_SPEED, MIN_ALT_SPEED, time_diff,
			    &loop.alt_ic1_vel, &velpid_out_alt,
			    &loop.alt_pid1);

	      pid_return_new[2] = loop.alt_pid1;
	      pid_return_new[3] = 0.5 * loop.alt_pid1;



	      break;
	    default:
	      printf ("Error in the ALT tachometer switching\n");

	    }



readout.instantAzErr = readout.instantCommandAz-loop.azimuth_encoder_double;
readout.instantAltErr = readout.instantCommandAlt-loop.altitude_encoder_double;
//f(readout.instantAzErr >180.){
//readout.instantAzErr=readout.instantAzErr-360.;
// 
//	if(azerr_uncorrected >180.){
//		azerr_uncorrected=azerr_uncorrected-360.;
//	} 
		      		
//	readout.instantAzErr = azerr1;
//	readout.instantAltErr = alterr1;
//azerr_uncorrected is the error compared to the actual given command from the control PC- I correct this on the servo to limit accelerations etc.
	  readoutStructUpdate(azerr_uncorrected,alterr_uncorrected,&readout,&loop,&user); 
//	  readoutStructUpdate(azerr1,alterr1,&readout,&loop,&user); 



	  if (countera == 9)
	    {
	      printf("Command pos %lf Error %lf\n",loop.azimuth_command_double,azerr1);
	     // printf("PID pos %7d vel %7d Tot %7d AZ %7lf %7lf %7d %7lf %7lf %7lf ",pid_return_new[0],velpid_out_az,loop.az_pid1,loop.vel_of_az,loop.kfcoeffs[1]*loop.vel_of_az,aztacho1,azencoder_vel[7],loop.vel_of_alt,azerr1);
	      //printf("PID pos %7d vel %7d Tot %7d AZ %7lf %7lf %7d %7lf %7lf %7lf \n",pid_return_new[2],velpid_out_alt,loop.alt_pid1,loop.vel_of_alt,loop.kfcoeffs[3]*loop.vel_of_alt,alttacho1,altencoder_vel[7],loop.vel_of_alt,alterr1);
	      //printf("Az Encoder Velocity Tacho %d %f %f\n",aztacho[0],loop.vel_of_az,loop.kfcoeffs[0]*loop.vel_of_az);

	    }





	  //might requires a different bit of additional damping if only the pid loop is running (i.e command_velocity is zero)
	  if (loop.vel_of_az < 50. && loop.vel_of_az > -50.)
	    {
	      if (azerr1 < 6. && azerr1 > -6.)
		{
		  if (countera == 20)
		    {
		      //printf("Correcting pid_return_new 1\n");
		    }
		  pid_return_new[0] -=
		    loop.dcoeffs[0] * (aztacho2);
		  pid_return_new[1] -=
		    loop.dcoeffs[0] * (aztacho2);
		}
	    }

	  if (loop.vel_of_alt < 50. && loop.vel_of_alt > -50.)
	    {
	      if (alterr1 < 6. && alterr1 > -6.)
		{
		  temp_double = loop.dcoeffs[3] *(alttacho2);
		  /////////////////////////////////////
		  //adding this section just for stability while the antenna is out of balance in elevation
		 //if (temp_double > 500)
		   //{
		     // temp_double = 500;
		    //}
		  //if (temp_double < -500)
		    //{
		      //temp_double = -500;
		    //}
		  ///////////////////////////////

		  pid_return_new[2] -= temp_double;
		  pid_return_new[3] -= temp_double;
		}

	    }



	  ramp (pid_return_old, &pid_return_new[0], &pid_return_new[1], &pid_return_new[2], &pid_return_new[3], MAX_AZ_SPEED, MIN_AZ_SPEED, MAX_ALT_SPEED, MIN_ALT_SPEED, 0, 0, 0, 0);	//the acceleration of the antenna needs to be limited. This is handled by this ramp() routine



	  soft_lim (&loop, loop.limits[1], loop.limits[0], AZIMUTH_SAFETY_HI, AZIMUTH_SAFETY_LO, AZ_SLOW_SPEED_POSITIVE, AZ_SLOW_SPEED_NEGATIVE, user.az_encoder_long, &(loop.az_command_long), &(loop.az_pid), &(pid_return_new[0]), &(pid_return_new[1]));	//the software limit checking to prevent antenna driving into its own hard limits.

	  soft_lim (&loop, loop.limits[3], loop.limits[2],
		    ELEVATION_SAFETY_HI, ELEVATION_SAFETY_LO,
		    EL_SLOW_SPEED_POSITIVE, EL_SLOW_SPEED_NEGATIVE,
		    user.alt_encoder_long, &(loop.alt_command_long),
		    &(loop.alt_pid), &(pid_return_new[2]),
		    &(pid_return_new[3]));

	  if (loop.vel_of_az < 50. && loop.vel_of_az > -50.)
	    {
	      if (azerr1 < 3. && azerr1 > -3.)
		{
		  backlash (loop.az_command_long, user.az_encoder_long,
			    loop.vel_of_az, 500, 100, 0, 0,
			    &pid_return_new[0], &pid_return_new[1]);
		}
	    }
	if (loop.vel_of_alt < 50. && loop.vel_of_alt > -50.)
	    {
	      if (alterr1 < 3. && alterr1 > -3.)
		{
		
	  backlash (loop.alt_command_long, user.alt_encoder_long,
		    loop.vel_of_alt, 500, 100, 0, 0, &pid_return_new[2],
		    &pid_return_new[3]);
		}
		}

	  if (countera == 20)
	    {
	      //printf("PID RETURN %d %d \n",pid_return_new[0],pid_return_new[1]);
	    }


//	if(aztacho2 > MAX_TACHO_ALLOWABLE){
//		 pid_return_new[0] -=
//		    loop.dcoeffs[0] * (pos_az_dot + aztacho2); 
//		pid_return_new[1] -=
//		    loop.dcoeffs[0] * (pos_az_dot + aztacho2);
//	}
//	else if(aztacho2 < MIN_TACHO_ALLOWABLE){
//		 pid_return_new[0] -=
//		    loop.dcoeffs[0] * (pos_az_dot + aztacho2); 
//		pid_return_new[1] -=
//		    loop.dcoeffs[0] * (pos_az_dot + aztacho2);
//	}
//	if(alttacho2 > MAX_TACHO_ALLOWABLE){
//		pid_return_new[2] = pid_return_old[2] - 0.05*(alttacho2-MAX_TACHO_ALLOWABLE);
//		pid_return_new[3] = pid_return_old[3]- 0.05*(alttacho2 - MAX_TACHO_ALLOWABLE);
//	}
//	else if(alttacho2 < MIN_TACHO_ALLOWABLE){
//		pid_return_new[2] = pid_return_old[2] -0.05*(alttacho2-MIN_TACHO_ALLOWABLE);
//		pid_return_new[3] = pid_return_old[3] -0.05*(alttacho2-MIN_TACHO_ALLOWABLE);
//	}

  //int softLimitStatus;
	
	//soft limit check-will eventually be a bit more intelligent but this will prevent embarassing limit overruns
	if((loop.azimuth_encoder_double > MAX_AZ_VAL_FLOAT)   &&    ((pid_return_new[0]>0)||(pid_return_new[1]>0))){
		pid_return_new[0]=0;
		pid_return_new[1]=0;
		readout.softLimitStatus=1;
		//printf("Out Zone 1\n");
	}
	else if((loop.azimuth_encoder_double < MIN_AZ_VAL_FLOAT)   &&    ((pid_return_new[0]<0)||(pid_return_new[1]<0))){
		pid_return_new[0]=0;
		pid_return_new[1]=0;
		readout.softLimitStatus=2;
		//printf("Out Zone 2\n");
	}
	else if((loop.altitude_encoder_double > MAX_ALT_VAL_FLOAT)   &&    ((pid_return_new[2]>0)||(pid_return_new[3]>0))){
		pid_return_new[2]=0;
		pid_return_new[3]=0;
		readout.softLimitStatus=3;
		//printf("Out Zone 3\n");
	}
	else if((loop.altitude_encoder_double < MIN_ALT_VAL_FLOAT)   &&    ((pid_return_new[2]<0)||(pid_return_new[3]<0))){
		pid_return_new[2]=0;
		pid_return_new[3]=0;
		readout.softLimitStatus=4;
		//printf("Out Zone 4\n");
	}
	else{
		readout.softLimitStatus=0;
	}


//	if((loop.altitude_encoder_double > MAX_ALT_VAL_FLOAT)||(loop.altitude_encoder_double < MIN_ALT_VAL_FLOAT)){
//		pid_return_new[2]=0;
//		pid_return_new[3]=0;
//	}



//printf("PID OUT %ld %ld %ld %ld \n",pid_return_new[0],pid_return_new[1],pid_return_new[2],pid_return_new[3]);
	  //calculate the data that needs to be written to the DACS. These include a CRC for robustness.
	  loop.az1_dac_control =
	    ad5362_crc_pack (XREGISTER_WRITE, CH5, pid_return_new[0]);
	  loop.az2_dac_control =
	    ad5362_crc_pack (XREGISTER_WRITE, CH6, pid_return_new[1]);
	  loop.alt1_dac_control =
	    ad5362_crc_pack (XREGISTER_WRITE, CH7, pid_return_new[2]);
	  loop.alt2_dac_control =
	    ad5362_crc_pack (XREGISTER_WRITE, CH8, pid_return_new[3]);


	  // status_vec=control.status_vec;


	  if (!user.read_status)
	    {
	      //write the relevent voltage commands (after pid, ramp and software limits) to the kernel board. This then updated the DACs, gives the voltage command to the YASKAWA servopacks and the antenna can begin to drive.
	      //NOTE there is further control over the voltage output at the kernel level and certain flags need to be set before there will actually be voltage output. This is done using the IOCTL to the kernel with command value DEV_IOCTL_ENABLE_DAC or DEV_IOCTL_DISABLE_DAC respectively. Handled in the command thread.
	      read_ret =
		write (fd, (struct pid_struct *) &loop, sizeof (loop));
	    }
	  else
	    printf ("error writing\n");

	  if (user.azimuth_zone <= 2)
	    {			//and now we simply send the date through- this might be better done by sending the entire structure since the ascii string packing is inherently slow. But sending structures causes 2 problemss:
	      //1. Need to send the data each time loop. Lots of overhead (i.e it is more difficult to keep the data building up and then only actually send the data every now and again). This can actually be overcome fairly easily if I tried!
	      //2. This locks us into using C on the other side of the channel since other programming languages have different memorey layout. This is the main reason I have not pursued this cours.
	      //  tcp_control_structs[packets] = user;
	      sprintf (cnt,
		       "%10.3f,%10.3f,%8u,%8u,%8.3f,%8.3f,%6u,%6u,%2i,%6i,%6i,%6i,%6i,%6i,%6i,%6i,%6i,%6i,%10lu,%10lu,%6i,%6i,%6i,%6i,%6i,%8ld,%8ld,%7ld,%7ld,%5u,%5ld,%04x,%04x,%04x,%04x,%04x,%04x,%5.3f,%5.3f,end \n",
		       loop.azimuth_encoder_double,
		       loop.azimuth_command_double, user.az_encoder_long,
		       loop.az_command_long, loop.altitude_encoder_double,
		       loop.altitude_command_double, user.alt_encoder_long,
		       loop.alt_command_long, user.azimuth_zone, test_dac,
		       aztacho1, aztacho[0], alttacho1, alttacho[0],
		       pid_return_new[0], pid_return_new[1],
		       pid_return_new[2], pid_return_new[3],
		       loop.alt1_dac_control, loop.alt2_dac_control,
		       user.encoder_error, err_count_alt, err_count_az,
		       loop.az_command_long, loop.alt_command_long,
		       user.time_struct.tv_sec, user.time_struct.tv_usec,
		       user.time, time_diff, user.counter3, user.counter2,
		       control.status_vec[0], control.status_vec[2],
		       control.status_vec[4], control.status_vec[3],
		       control.status_vec[5], control.status_vec[6],
		       loop.vel_of_az * loop.kfcoeffs[0],
		       loop.vel_of_alt * loop.kfcoeffs[2]);
	      //loop.vel_of_az*loop.kfcoeffs[0],loop.vel_of_alt*loop.kfcoeffs[2]
	    }
	  else
	    {
	      printf
		("\n\n\n ----------------------------ERROR USING THE CONTROL SOFTWARE-------------------------\n\n\nPROBLEM WITH AZIMUTH ZONE INFORMATION... UPDATE THE PID STRUCTURE WITH A VALUE BEFORE RUNNING THIS\n\n./azimuth_zone 1 \n\n The Zone information can be read using \n\n./read_azimuth_zone \n\n This should only need to be done on bootup or if the kernel module has been recently removed and re-inserted. \n\n ----------------------ZONE INFORMATION-------------\n\n If the Telescope was parked then you are in Zone 1. If the telescope has reached its minimum azimuth limit then you are in Zone 0 and if it has reached maximum azimuth limit then you are in Zone 2\n\n\n\n");

	      sprintf (cnt,
		       "\n\n\nPROBLEM WITH AZIMUTH ZONE INFORMATION... UPDATE THE PID STRUCTURE WITH A VALUE BEFORE RUNNING THIS\n\n./azimuth_zone 1 \n\n The Zone information can be read using \n\n./read_azimuth_zone \n\n This should only need to be done on bootup. If the Telescope was parked then you are in Zone 1. If the telescope has reached its minimum azimuth limit then you are in Zone 0 and if it has reached maximum azimuth limit then you are in Zone 2\n\n\n\n");
	      if(sd_command){
		rc = send (sd_command, udpbuf, strlen (udpbuf), 0);
		}
	      exit (1);
		
	    }

	  strcat (udpcat, cnt);
	  sprintf (cnt, "");

	  // strcat(udpcat,"\n");
	}
    }
  printf ("Exited the main while loop for some reason\n");
  return 0;

}


void readoutStructUpdate(double azerr1,double alterr1, volatile struct readout_struct *readout,volatile struct pid_structure *loop,struct pid_structure *user){ 
		int j; 


//-----------------PREPARE to interpolate to 5 position /second on a regular interval

	  readout->azimuth_time_table[0] = readout->azimuth_time_table[1];
	  readout->azimuth_time_table[1] = readout->azimuth_time_table[2];
	  readout->azimuth_time_table[2] = (long) user->time_struct.tv_usec;
	  //store azimuth positions
	  readout->azimuth_position_table[0] =
	    readout->azimuth_position_table[1];
	  readout->azimuth_position_table[1] =
	    readout->azimuth_position_table[2];
	  readout->azimuth_position_table[2] = loop->azimuth_encoder_double;
	  //store altitude positions
	  readout->altitude_position_table[0] =
	    readout->altitude_position_table[1];
	  readout->altitude_position_table[1] =
	    readout->altitude_position_table[2];
	  readout->altitude_position_table[2] = loop->altitude_encoder_double;
	  //store azimuth errors//
	  readout->az_pos_err_table[0] = readout->az_pos_err_table[1];
	  readout->az_pos_err_table[1] = readout->az_pos_err_table[2];
	  readout->az_pos_err_table[2] = azerr1;
		//store azimuth elevation//
	  readout->alt_pos_err_table[0] = readout->alt_pos_err_table[1];
	  readout->alt_pos_err_table[1] = readout->alt_pos_err_table[2];
	  readout->alt_pos_err_table[2] = alterr1;


	  //correct for rollover
	  if (readout->azimuth_time_table[2] < readout->azimuth_time_table[1])
	    {
	      readout->azimuth_time_table[1] -= 1000000;
	      readout->azimuth_time_table[0] -= 1000000;
	    }

	  //at the end of the second move the data to the ready to read section

	pthread_mutex_lock (&readout_lock);
		readout->instantAzErr = (float) azerr1;	//store for instantaneous errors
		readout->instantAltErr = (float) alterr1;	//store for instantaneous errors
		readout->azZone = (int) user->azimuth_zone;
		if (readout->current_value >= 1000000)
		  {
		    for (j = 0; j < readout->sample_number; j++)
		      {
			readout->az_ready_to_read[j] = readout->az_position[j];
			readout->alt_ready_to_read[j] = readout->alt_position[j];
/*			readout->az_err_ready_to_read[j] = readout->az_pos_err[j];
			readout->alt_err_ready_to_read[j] = readout->alt_pos_err[j];*/
		// I did this on 29 March 2013 because there seemed to be a problem with the way the errors were either being caluclated or loaded into the readout_structure
			readout->az_err_ready_to_read[j] = (float) azerr1 ;
			readout->alt_err_ready_to_read[j] = (float) alterr1;
		      }
		    if (readout->ready == 0)
		      {
			readout->ready = 1;
		      }
		    else
		      {
			printf ("Encoder positions not read from device in time\n");
			printf("In all likelihood the control Program isn't running \n");
			//shutdownServo();
		      }
		    if(readout->softLimitStatus!=0){
			printf("Soft Limit engaged %d\n",readout->softLimitStatus);
			}
		    readout->current_value = 0;
		    readout->sample_number = 0;
		  }
	pthread_mutex_unlock (&readout_lock);


	  if (readout->azimuth_time_table[1] <=
	      readout->current_value & readout->azimuth_time_table[2] >
	      readout->current_value)
	    {
	      //DO THE AZIMUTH INTERPOLATION FIRST
	      readout->calc_time[0] = readout->current_value;	//time we want to interpolate to
	      readout->calc_time[1] = (double) readout->azimuth_time_table[1];	//first time
	      readout->calc_time[2] = (double) readout->azimuth_time_table[2];	//second time
	      readout->calc_time[3] = readout->calc_time[2] - readout->calc_time[1];	//total time difference
	      readout->calc_time[4] = readout->calc_time[0] - readout->calc_time[1];	//time difference to interpolation point
	      readout->calc_time[5] = readout->calc_time[4] / readout->calc_time[3];	//fraction of time to interpolation point
	      //NOw calculate the azimuth positions
	      readout->calc_time[6] =
		(double) readout->azimuth_position_table[1];
	      readout->calc_time[7] =
		(double) readout->azimuth_position_table[2];
	      readout->calc_time[8] =
		readout->calc_time[7] - readout->calc_time[6];
	      readout->calc_time[9] =
		readout->calc_time[8] * readout->calc_time[5] +
		readout->calc_time[6];
	      readout->az_position[readout->sample_number] = (float) readout->calc_time[9];	//and store the azimuth value
	      //AND NOW THE ELEVATION INTERPOLATION- WE DON"T NEED TO REDO THE TIME CALCS ONLY THE POSITION
	      readout->calc_time[6] =
		(double) readout->altitude_position_table[1];
	      readout->calc_time[7] =
		(double) readout->altitude_position_table[2];
	      readout->calc_time[8] =
		readout->calc_time[7] - readout->calc_time[6];
	      readout->calc_time[9] =
		readout->calc_time[8] * readout->calc_time[5] +
		readout->calc_time[6];
	      readout->alt_position[readout->sample_number] = (float) readout->calc_time[9];	//and store the altitude valu
	      //AND NOW THE AZIMUTH ERROR INTERPOLATION- 
	      readout->calc_time[6] = (double) readout->az_pos_err_table[1];
	      readout->calc_time[7] = (double) readout->az_pos_err_table[2];
	      readout->calc_time[8] =
		readout->calc_time[7] - readout->calc_time[6];
	      readout->calc_time[9] =
		readout->calc_time[8] * readout->calc_time[5] +
		readout->calc_time[6];
	      readout->az_pos_err[readout->sample_number] = (float) readout->calc_time[9];	//and store the az err valu
	      //readout->az_pos_err[readout->sample_number] = (float) azerr1;	//and store the az err valu
	      //AND FINALLY the ALTITUDE ERROR INTERPOLATION- 
	      readout->calc_time[6] = (double) readout->alt_pos_err_table[1];
	      readout->calc_time[7] = (double) readout->alt_pos_err_table[2];
	      readout->calc_time[8] =
		readout->calc_time[7] - readout->calc_time[6];
	      readout->calc_time[9] =
		readout->calc_time[8] * readout->calc_time[5] +
		readout->calc_time[6];
	      readout->alt_pos_err[readout->sample_number] = (float) readout->calc_time[9];	//and store the az err valu
	      //readout->alt_pos_err[readout->sample_number] = (float) alterr1;	//and store the az err valu


	      readout->sample_number++;	//increment the sample counter
	      readout->current_value =
		readout->sample_number * readout->sample_rate;
	      readout->time[0] = (long) user->time_struct.tv_sec;
	    }

} 

void allPidsUpdate(struct new_pid_coefficient_structure *az1,struct new_pid_coefficient_structure *az2,struct new_pid_coefficient_structure *el1, struct new_pid_coefficient_structure *el2, volatile double *pcoeffs, volatile double *icoeffs, volatile double *dcoeffs, volatile double *kfcoeffs, volatile double *vfcoeffs, volatile long *motor_plus, volatile long *motor_minus,volatile double *pcoeffs_vel,volatile double *icoeffs_vel, volatile double *dcoeffs_vel){
	  if (az1->adaptive == 1)
	    {
	      pidadaptive_update (0, 0,0,
				  az1, pcoeffs+0,
				  icoeffs+0, dcoeffs+0,
				  kfcoeffs+0, vfcoeffs+0,
				  motor_plus+0, motor_minus+0,
				  pcoeffs_vel+0, icoeffs_vel+0,
				  dcoeffs_vel+0);
	    }
	  if (az2->adaptive == 1)
	    {     
		pidadaptive_update (1, 0,0,
				  az2, pcoeffs+1,
				  icoeffs+1, dcoeffs+1,
				  kfcoeffs+1, vfcoeffs+1,
				  motor_plus+1, motor_minus+1,
				  pcoeffs_vel+1, icoeffs_vel+1,
				  dcoeffs_vel+1);
	   }
	  if (el1->adaptive == 1)
	    {	pidadaptive_update (2, 0,0,
				  el1, pcoeffs+2,
				  icoeffs+2, dcoeffs+2,
				  kfcoeffs+2, vfcoeffs+2,
				  motor_plus+2, motor_minus+2,
				  pcoeffs_vel+2, icoeffs_vel+2,
				  dcoeffs_vel+2);
	   
	   }

	  if (el2->adaptive == 1)
	    {
	      	pidadaptive_update (3, 0,0,
				  el2, pcoeffs+3,
				  icoeffs+3, dcoeffs+3,
				  kfcoeffs+3, vfcoeffs+3,
				  motor_plus+3, motor_minus+3,
				  pcoeffs_vel+3, icoeffs_vel+3,
				  dcoeffs_vel+3);
	   }
	 

}


void
velocity_pid (long vel_tacho, double vel_command, double position_error,
	      double Vfa, double Pd, double Id, double Dd, long Mx_I,
	      long Mn_I, int MAX_OUT, int MIN_OUT, long time_diff,
	      volatile double *current_i_ptr, long *velpid_out,
	      volatile long *pid_return_ptr)
{
  double corrected_vel_command, relative_time_diff;
  double error;
  double output_p, output_i;
  double ival, return_val_double;
  long return_val;
  relative_time_diff = (double) time_diff / 10000.;
  //Vf is the conversion factor to get the tacho velocity to the same "units" as the velocity command
  corrected_vel_command = vel_command;	//*Vfa;
  // vel_tacho=vel_tacho*Kfa;
  // ival is the old integrator value

  if (corrected_vel_command == 0)
    {
      *current_i_ptr = 0;
    }



  ival = 0.;
  error = 0.;
  ival = *current_i_ptr;
  //Calculate the velocity error
  //error = corrected_vel_command-vel_tacho;
  // error = (double)*pid_return_ptr;
  error = corrected_vel_command - (double) vel_tacho *Vfa;
  //integral
  ival += error * relative_time_diff;
  if (ival > Mx_I)
    {
      ival = Mx_I;
    }

  if (ival < Mn_I)
    {
      ival = Mn_I;
    }

  //update the integral
  *current_i_ptr = ival;

  output_p = error * Pd;
  output_i = ival * Id;

  return_val_double = output_p + output_i;

  return_val = (long) return_val_double;
  //here we make sure the velocity outputs don't dwarf the position output
  if (return_val > MAX_OUT)
    {
      return_val = (long) MAX_OUT;
      //printf ("Velocity Loop Return Maxed %d\n", return_val);
    }
  if (return_val < MIN_OUT)
    {
      return_val = (long) MIN_OUT;
      //printf ("Velocity loop Return Mined %d\n", return_val);
    }
  //if(corrected_vel_command==0){
  // return_val+=0;
  //}
  *velpid_out = return_val;
  //printf("Velocity Command %d Velocity Tacho %f Vel PID %ld\n",vel_tacho,corrected_vel_command,return_val);
  //if(position_error <2 &&position_error>-2){
  *pid_return_ptr = return_val;
  //}

}



int
init_control_struct (struct pid_structure *control_ptr)
{
  //the initiation of the control structure 
  struct pid_structure controlj;
  int i;
  int read_ret;


  read_ret = read (fd, &controlj, sizeof (controlj));
  //pthread_mutex_lock (&mutexsum);
  //controlj = *control_ptr;
  //pthread_mutex_unlock (&mutexsum);
  printf ("AZIMUTH ZONE ON STARTUP = %d\n", controlj.azimuth_zone);

  controlj.DAC_Output = 0;
  controlj.alt_err_prev = 0;
  controlj.alt_p = 0;
  controlj.alt_i = 0;
  controlj.alt_d = 0;
  controlj.az_p = 0;
  controlj.az_i = 0;
  controlj.az_d = 0;
  controlj.time = 0;
  controlj.az_i2 = 0;
  controlj.az_i1 = 0;
  controlj.alt_i1 = 0;
  controlj.alt_i2 = 0;
  controlj.az_err2 = 0;
  controlj.az_err1 = 0;
  controlj.alt_err1 = 0;
  controlj.alt_err2 = 0;
  controlj.az_p1 = 0.;
  controlj.az_p2 = 0.;
  controlj.az_ic1 = 0.;
  controlj.az_ic2 = 0.;
  controlj.az_d1 = 0.;
  controlj.az_d2 = 0.;
  controlj.az_p1_vel = 0.;
  controlj.az_p2_vel = 0.;
  controlj.az_ic1_vel = 0.;
  controlj.az_ic2_vel = 0.;
  controlj.az_d1_vel = 0.;
  controlj.az_d2_vel = 0.;



  controlj.alt_p1 = 0.;
  controlj.alt_p2 = 0.;
  controlj.alt_ic1 = 0.;
  controlj.alt_ic2 = 0.;
  controlj.alt_d1 = 0.;
  controlj.alt_d2 = 0.;
  controlj.alt_p1_vel = 0.;
  controlj.alt_p2_vel = 0.;
  controlj.alt_ic1_vel = 0.;
  controlj.alt_ic2_vel = 0.;
  controlj.alt_d1_vel = 0.;
  controlj.alt_d2_vel = 0.;



  //these define the absolute maximum encoder angles that the antenna will drive to
  controlj.limits[0] = AZIMUTH_SAFETY_LO;
  controlj.limits[1] = AZIMUTH_SAFETY_HI;
  controlj.limits[2] = ELEVATION_SAFETY_LO;
  controlj.limits[3] = ELEVATION_SAFETY_HI;
  for (i = 0; i <= 19; i++)
    {
      control.pcoeffs[i] = 0.;
      control.icoeffs[i] = 0.;
      control.dcoeffs[i] = 0.;
      control.kfcoeffs[i] = 0.;
      control.vfcoeffs[i] = 0.;
      control.motor_plus[i] = 0;
      control.motor_minus[i] = 0;

    }

  controlj.az_command_long = controlj.az_encoder_long;
  controlj.alt_command_long = controlj.alt_encoder_long;
  encoder2azalt (controlj.az_command_long, 0., controlj.alt_command_long, 0.,
		 &controlj.azimuth_command_double,
		 &controlj.altitude_command_double);
  printf ("Initial commands = %d %d\n", controlj.az_command_long,
	  controlj.alt_command_long);
  for (i = 0; i <= 7; i++)
    {
      //this gives a nominal zero value- adjust later for small offsets.
      controlj.cbuffer[i] = 0x8000;
      controlj.mbuffer[i] = 0xffff;
    }

  //use these to adjust the DAC output by using small trim values
  controlj.cbuffer[0] += 0;
  controlj.cbuffer[1] += 0;
  controlj.cbuffer[2] += 0;
  controlj.cbuffer[3] += 0;
  controlj.cbuffer[4] += 185;
  controlj.cbuffer[5] -= 50;
  controlj.cbuffer[6] += 50;
  controlj.cbuffer[7] -= 10;
  controlj.cbuffer_crc[0] =
    ad5362_crc_pack1 (CREGISTER_WRITE, CH1, controlj.cbuffer[0]);
  controlj.cbuffer_crc[1] =
    ad5362_crc_pack1 (CREGISTER_WRITE, CH2, controlj.cbuffer[1]);
  controlj.cbuffer_crc[2] =
    ad5362_crc_pack1 (CREGISTER_WRITE, CH3, controlj.cbuffer[2]);
  controlj.cbuffer_crc[3] =
    ad5362_crc_pack1 (CREGISTER_WRITE, CH4, controlj.cbuffer[3]);
  controlj.cbuffer_crc[4] =
    ad5362_crc_pack1 (CREGISTER_WRITE, CH5, controlj.cbuffer[4]);
  controlj.cbuffer_crc[5] =
    ad5362_crc_pack1 (CREGISTER_WRITE, CH6, controlj.cbuffer[5]);
  controlj.cbuffer_crc[6] =
    ad5362_crc_pack1 (CREGISTER_WRITE, CH7, controlj.cbuffer[6]);
  controlj.cbuffer_crc[7] =
    ad5362_crc_pack1 (CREGISTER_WRITE, CH8, controlj.cbuffer[7]);

  controlj.mbuffer_crc[0] =
    ad5362_crc_pack1 (MREGISTER_WRITE, CH1, controlj.mbuffer[0]);
  controlj.mbuffer_crc[1] =
    ad5362_crc_pack1 (MREGISTER_WRITE, CH2, controlj.mbuffer[1]);
  controlj.mbuffer_crc[2] =
    ad5362_crc_pack1 (MREGISTER_WRITE, CH3, controlj.mbuffer[2]);
  controlj.mbuffer_crc[3] =
    ad5362_crc_pack1 (MREGISTER_WRITE, CH4, controlj.mbuffer[3]);
  controlj.mbuffer_crc[4] =
    ad5362_crc_pack1 (MREGISTER_WRITE, CH5, controlj.mbuffer[4]);
  controlj.mbuffer_crc[5] =
    ad5362_crc_pack1 (MREGISTER_WRITE, CH6, controlj.mbuffer[5]);
  controlj.mbuffer_crc[6] =
    ad5362_crc_pack1 (MREGISTER_WRITE, CH7, controlj.mbuffer[6]);
  controlj.mbuffer_crc[7] =
    ad5362_crc_pack1 (MREGISTER_WRITE, CH8, controlj.mbuffer[7]);
  ioctl (fd, DEV_IOCTL_SET_DAC_REGISTERS, &controlj);

  controlj.interrupt_rate = 18;
  controlj.encoder_wavelength = 10;
  read_ret = write (fd, &controlj, sizeof (controlj));
  //controlj.alt_command_long = controlj.alt_encoder_long;
  //controlj.az_command_long = controlj.az_encoder_long;
  pthread_mutex_lock (&pid_coefficients);
  azimuth_pid1.adaptive = 0;
  azimuth_pid2.adaptive = 0;
  altitude_pid1.adaptive = 0;
  altitude_pid2.adaptive = 0;
  azimuth_pid1.table_length = 0;
  azimuth_pid2.table_length = 0;
  altitude_pid1.table_length = 0;
  altitude_pid2.table_length = 0;
  azimuth_pid1.table_position = 0;
  azimuth_pid2.table_position = 0;
  altitude_pid1.table_position = 0;
  altitude_pid2.table_position = 0;
  pthread_mutex_unlock (&pid_coefficients);

  pthread_mutex_lock (&mutexsum);
  control = controlj;
  pthread_mutex_unlock (&mutexsum);

}



void
ramp (volatile long *pid_return_old, long *az_pid1, long *az_pid2,
      long *alt_pid1, long *alt_pid2, int MAX_AZ, int MIN_AZ, int MAX_ALT,
      int MIN_ALT, int AZ_INTERVAL, int ALT_INTERVAL, int MAX_AZ_INTERVAL,
      int MAX_ALT_INTERVAL)
{

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

  current_ramp_az1 = *az_pid1 - pid_return_old_az1;
  current_ramp_az2 = *az_pid2 - pid_return_old_az2;
  current_ramp_alt1 = *alt_pid1 - pid_return_old_alt1;
  current_ramp_alt2 = *alt_pid2 - pid_return_old_alt2;


      if (current_ramp_az1 < -MAX_AZ_RAMP)
	{
	 // *az_pid1 = pid_return_old_az1 - MAX_AZ_RAMP;
	    *az_pid1 = pid_return_old_az1-MAX_AZ_RAMP;
	  // printf("Ramp az1 overload: Rounding down to %i\n,",*az_pid1);
	}
      if (current_ramp_az1 > MAX_AZ_RAMP)
	{
	   *az_pid1 = pid_return_old_az1+MAX_AZ_RAMP;
	 // *az_pid1 =  MAX_AZ_RAMP;
	  //printf("Ramp az1 overload: Rounding up to %i\n,",*az_pid1);
	}
      
     if (current_ramp_az2 < -MAX_AZ_RAMP)
	{
	 // *az_pid1 = pid_return_old_az1 - MAX_AZ_RAMP;
	    *az_pid2 = pid_return_old_az2-MAX_AZ_RAMP;
	  // printf("Ramp az1 overload: Rounding down to %i\n,",*az_pid1);
	}
      if (current_ramp_az2 > MAX_AZ_RAMP)
	{
	  *az_pid2 = pid_return_old_az2+ MAX_AZ_RAMP;
	  //printf("Ramp az1 overload: Rounding up to %i\n,",*az_pid1);
	}

      if (current_ramp_alt1 < -MAX_ALT_RAMP)
	{
	    *alt_pid1 = pid_return_old_alt1-MAX_ALT_RAMP;
	}
      if (current_ramp_alt1 > MAX_ALT_RAMP)
	{
	  *alt_pid1 =pid_return_old_alt1+ MAX_ALT_RAMP;
	}
      if (current_ramp_alt2 < -MAX_ALT_RAMP)
	{
	    *alt_pid2 = pid_return_old_alt2-MAX_ALT_RAMP;
	}
      if (current_ramp_alt2 > MAX_ALT_RAMP)
	{
	  *alt_pid2 =pid_return_old_alt2+ MAX_ALT_RAMP;
	}


}

int
soft_lim (volatile struct pid_structure *control_struct,
	  unsigned int limit_hi, unsigned int limit_lo,
	  unsigned int limit_slow_zone_hi, unsigned int limit_slow_zone_lo,
	  int slow_speed_hi, int slow_speed_lo, unsigned int encoder,
	  volatile unsigned int *command, volatile long *pid, long *pid1,
	  long *pid2)
{
  //limit hi defines the maximum allowable command angle
  //limit lo defines the minimum allowable command angle
  //limit_slow_zone_hi defines the coordinates that the antenna will drive slowly in
  //limit_slow_zone_lo define the coordinates that the antenna will drive slwoly in


  if (*command > limit_hi)
    {
      *command = limit_hi;
    }

  if ((encoder > limit_hi) && ((*command) > limit_hi))
    {
      *pid = 0;
      *pid1 = 0;
      *pid2 = 0;
      *command = limit_hi;
    }

//      if((encoder>limit_hi) ) {
//              *pid = 0;
//              *pid1 = 0;
//              *pid2 = 0;
//              *command = limit_hi;
//      }

  //printf("LIMIT_ZONE\n");
  if (encoder > limit_slow_zone_hi)
    {
      if (*pid1 > slow_speed_hi)
	{
	  //printf("%d adjusting speed1\n",control_limits.alt_pid);
	  //printf("adjusting speed1\n");
	  *pid1 = slow_speed_hi;

	}
      if (*pid2 > slow_speed_hi)
	{
	  //printf("%d adjusting speed1\n",control_limits.alt_pid);
	  //printf("adjusting speed1\n");
	  *pid2 = slow_speed_hi;

	}
    }

  if (*command < limit_lo)
    {
      *command = limit_lo;
    }

  if ((encoder < limit_lo) && ((*command) < limit_lo))
    {
      *pid = 0;
      *pid1 = 0;
      *pid2 = 0;
      *command = limit_lo;
    }

//      if((encoder<limit_lo)) {
//              *pid = 0;
//              *pid1 = 0;
//              *pid2 = 0;
//              *command = limit_lo;
//      }


  if (encoder < limit_slow_zone_lo)
    {
      if (*pid1 < slow_speed_lo)
	{
	  //printf("adjusting speed2\n");
	  *pid1 = slow_speed_lo;
	}
      if (*pid2 < slow_speed_lo)
	{
	  //printf("adjusting speed2\n");
	  *pid2 = slow_speed_lo;
	}
    }

}



int
update_pid_double (char select, double command, double prev_encoder,
		   double encoder, double p_gain, double i_gain,
		   double d_gain, double kfgain, double vel_com,
		   long time_diff, int MOTOR_SLACK_PLUS,
		   int MOTOR_SLACK_MINUS, int MAX_MOTOR_SPEED_PLUS,
		   int MAX_MOTOR_SPEED_MINUS, int MAX_I_WINDUP,
		   int MIN_I_WINDUP, double *current_error_ptr,
		   volatile double *current_i_ptr,
		   volatile long *pid_return_ptr)
{

  double error1, error2, error3, error1_prev, error2_prev, error3_prev;
  double p, i, d;
  double previous_error, current_error, previous_encoder;
  long pid_return;
  double test1, test2, test3;
  double relative_time_diff, pd, id, dd, current_errord;
  double command_velocityff;
  double pg, ig, dg, kfg;
  previous_encoder = prev_encoder;	//store the previous encoder value for the D calculation
  i = *current_i_ptr;		//store the integral value for calculations
  previous_error = (double) *current_error_ptr;	//store the previous error value (i.e command-position)
  pg = p_gain;
  ig = i_gain;
  dg = d_gain;
  kfg = kfgain;
  relative_time_diff = (double) time_diff / 10000.;
  test1 = 0.;
  test2 = 0.;
  test3 = 0.;

  //printf("PID %f %f \n",kfgain,vel_com);
  command_velocityff = kfgain * vel_com;
  //command_velocityff=0.;
  //printf("vel ff %f %f %f \n",kfgain,vel_com,command_velocityff);
  error1 = command - encoder;	//calculate the new error value
  //printf("Error1 %lf \n",error1);
  error2 = 65535 - encoder + (command);	//in case of encoder overflow- not a problem provided the encoders do not overflow- make sure this is the case when they are installed especially with the altitude encoder- much easier!
  error3 = -(65535 - (command) + encoder);
  //printf("Error %f %f %f\n",error1,command,encoder);
  //turn clockwise

//      if((fabs(error1) >fabs(error2)) && (fabs(error3) >fabs(error2))) {
//      current_error = error2;
//      //printf("Error2 AntiClockwise %f %f %f\n",fabs(error1),fabs(error2),fabs(error3);
//      //direction=+1;
//      }
//      //anticlockwise
//      else if ((fabs(error1) >fabs(error3)) && (fabs(error2) >fabs(error3))) {
//      current_error = error3;
//      //printf("Error3 AntiClockwise %f %f %f\n",fabs(error1),fabs(error2),fabs(error3));
//      //direction=-1;
//      }
//      
//      else if ((fabs(error2) >fabs(error1)) && (fabs(error3) >fabs(error1))) {
//      current_error = error1;
//      //printf("Error1 AntiClockwise %f %f %f\n",fabs(error1),fabs(error2),fabs(error3));
//      //direction=-1;
//      }
//      
  current_error = error1;	//the current error is error1
  current_errord = (double) current_error;	//convert to a double for the calculations with floating point values


  p = current_error;		//store the value to be used with the p calculations

  //id = current_errord+(double)i*(relative_time_diff);
  if (i_gain > 0.00001)
    {
      i = current_errord * (relative_time_diff) + i;	//add the error to the previous i value (i.e pseudo integration)
    }
  else if (i_gain <= 0.00001)
    {
      i = 0.;
      //printf("Igain is Zero\n");
    }
  //printf("i = %d id=%f\n",i,id);
  //i = (long)id;
  //i = (double)current_error + (double)i*((double)time_diff/10000.);
  //check the integral windup


  //d = current_error - previous_error;
  //See PID-without PHD by Tim Westcottfor explanation of the D term. Suggests it is better to use derivative of the position rather than the error.
  d = (encoder - previous_encoder);
//      d=5000;
  //calculate the actual pid values (i.e output = kp*p + ki*i - kd*d)
  test1 = p_gain * (double) p;
  test2 = i_gain * i;
  if (test2 > (double) MAX_I_WINDUP)
    {
      i = (double) MAX_I_WINDUP / i_gain;
      //printf("Max I windup %f %f\n",i,i*i_gain);
    }
  else if (test2 < (double) MIN_I_WINDUP)
    {
      i = (double) MIN_I_WINDUP / i_gain;
      //printf("Min I Windup %f %f\n",i,i*i_gain);
    }
  //printf("Test2 %f %d %f\n",p_gain,i,test2);
  //test2=0;
  test3 = d_gain * (double) d;
  //printf("test1 %f test2 %f test3 %f %f\n",test1,test2,test3,i);
  //if(test3!=0.)
  //printf("test1 %f test2 %f test3 %f\n",test1,test2,test3);
  //so use this return if there is no velocity pid loop afterwards
  //pid_return = (long)test1 + (long)test2-(long)test3+command_velocityff;
  //so use this return if there is  velocity pid loop afterwards
  pid_return = (long) test1 + (long) test2 + command_velocityff;

  //printf("current error %d p %d i %d d %d test1 %f test2 %f test3 %f pidreturn %d\n",current_error,p,i,d,test1,test2,test3,pid_return);
  //take into account the motor slack- i.e the antenna does not begin to move until a certain voltage is reached on the output.
  if (previous_error < 0 && current_error >= 0)
    {

      //simple anti-windup
      //if(i<0){
      //  i+=(double)MOTOR_SLACK_PLUS/i_gain;
      //}
      //need a small offset for the motor slack
      //if(pid_return<MOTOR_SLACK_PLUS){
      //pid_return=(long)MOTOR_SLACK_PLUS;
      //}
    }
  if (previous_error > 0 && current_error <= 0)
    {
      //simple anti-windup
      //if(i>0){
      // i+=(double)MOTOR_SLACK_MINUS/i_gain;
      //}
      //if(pid_return>MOTOR_SLACK_MINUS){
      // pid_return=(long)MOTOR_SLACK_MINUS;
      //}
    }

  if (current_error > 0)
    {
      pid_return += MOTOR_SLACK_PLUS;
    }
  if (current_error < 0)
    {
      pid_return += MOTOR_SLACK_MINUS;
    }
  //set the maximum pid value that can be output to limit maximum speed
  if (pid_return > MAX_MOTOR_SPEED_PLUS)
    {
      pid_return = MAX_MOTOR_SPEED_PLUS;
    }

  if (pid_return < MAX_MOTOR_SPEED_MINUS)
    {
      pid_return = MAX_MOTOR_SPEED_MINUS;
    }
//      printf("command %d encoder %d i %d prev error %d current error %d pid_return %d\n",command,encoder,i,previous_error,current_error,pid_return);
  //store data from this loop to the control structure
  *current_i_ptr = i;
  //*current_error_ptr = current_error;
  *current_error_ptr = current_error;
  *pid_return_ptr = pid_return;
//      control.alt_d = control.alt_err - control.alt_err_prev;
//      control.alt_pid = (pidloop.p * control.alt_p + pidloop.i*control.alt_i + pidloop.d*control.alt_d);
//      control.alt_simulation = 0.2*control.alt_pid;


  return 1;


}




unsigned int
get_crc (unsigned int packet)
{
  unsigned char test_vec[10];
  unsigned int retval;
  crc_t crc;
  test_vec[0] = (packet & 0xff000000) >> 24;
  test_vec[1] = (packet & 0x00ff0000) >> 16;
  test_vec[2] = (packet & 0x0000ff00) >> 8;
  test_vec[3] = (packet & 0x000000ff);
  crc = crc_init ();
  crc = crc_update (crc, test_vec, 4);
  crc = crc_finalize (crc);
  retval = (packet << 8) | (crc & (0x000000ff));
  return retval;
}

unsigned int
ad5362_crc_pack (unsigned int command, unsigned int channel,
		 unsigned int value)
{
  //this function takes inputs and prepares a suitable packet to be transmitted to the ad5362 DAC. Includeds a CRC for safety so will produce a four byte value stored in the unsigned int.
  unsigned char test_vec[10];
  unsigned int retval;
  unsigned int packet;
  crc_t crc;
  packet =
    (command << 20) | (channel << 16) | (((unsigned short) value + 32767) &
					 (0x00ffff));
  test_vec[0] = (packet & 0xff000000) >> 24;
  test_vec[1] = (packet & 0x00ff0000) >> 16;
  test_vec[2] = (packet & 0x0000ff00) >> 8;
  test_vec[3] = (packet & 0x000000ff);
  crc = crc_init ();
  crc = crc_update (crc, test_vec, 4);
  crc = crc_finalize (crc);
  retval = (packet << 8) | (crc & (0x000000ff));
  return retval;
}

float *
allocate (size_t length)
{
  float *array;
  if ((array = (float *) malloc (length * sizeof (float))) == NULL)
    {
      printf ("Not enough memory to allocate buffer\n");
      exit (1);
    }
  printf ("String was allocated!\n");
  return array;
}





void
pidadaptive_update (unsigned int type, unsigned int encoder,
		    unsigned int command,
		    struct new_pid_coefficient_structure *pid_struct,
		    volatile double *pcoeff, volatile double *icoeff,
		    volatile double *dcoeff, volatile double *kfcoeff,
		    volatile double *vfcoeff, volatile long *MOTOR_PLUS,
		    volatile long *MOTOR_MINUS, volatile double *pcoeff_vel,
		    volatile double *icoeff_vel, volatile double *dcoeff_vel)
{
  long error;
  int i, j, length;
  j = 0;
  //length = pid_struct->table_length-1;
  //i = (pid_struct->table_position);
  //error = abs(command-encoder);

  // if(type==3){
  //printf("Altitude PID check %d %d %d\n",error,pid_struct->position_error[0],length);
  // }


  //while(error<=pid_struct->position_error[j] && (j < length)){
  //
  //if(type==3){
  //printf("Error %ld %ld \n",error,pid_struct->position_error[j]);
  //}
  //  j++;

  //}

  //if(j!=i){
  j = 0;
  pid_struct->table_position = j;
  // printf("Adaptive PID Error %d changing to %d on table %f %f %f\n",error,pid_struct->table_position,pid_struct->p[j],pid_struct->i[j],pid_struct->d[j] );
  *pcoeff = pid_struct->p[j];
  *icoeff = pid_struct->i[j];
  *dcoeff = pid_struct->d[j];
  *kfcoeff = pid_struct->kf[j];
  *vfcoeff = pid_struct->vf[j];
  *pcoeff_vel = pid_struct->p_2[j];
  *icoeff_vel = pid_struct->i_2[j];
  *dcoeff_vel = pid_struct->d_2[j];
  //*MOTOR_PLUS = pid_struct->motor_plus[j];
  //*MOTOR_MINUS = pid_struct->motor_minus[j];
  *MOTOR_PLUS = 0;
  *MOTOR_MINUS = 0;

  switch (type)
    {
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

  //}


  //   printf("Length %d\n",pid_struct->table_length);
  //   printf("%d\n%d\n",pid_struct->position_error[i],pid_struct->position_error[i+1]);


}




int
backlash (unsigned int command, unsigned int encoder, double velocity,
	  unsigned int backlash_motor_offset,
	  unsigned int backlash_position_offset,
	  unsigned int maintain_position_offset1,
	  unsigned int maintain_position_offset2, long *pid1, long *pid2)
{
  int hi_val, lo_val;
  int az, alt;
  int error, sign;
  error = (long) command - (long) encoder;
  //printf("Backlash\n");
  //printf("Command %d Encoder %d\n",command,encoder);
  if (velocity < 0.)
    {
      sign = +1;
      // printf("Backlash neg\n");
    }
  else if (velocity > 0.)
    {
      sign = -1;
      // printf("Backlash pos\n");
    }
  else if (velocity == 0.)
    {
      sign = 0;
    }
  error = abs (error);
  //printf("Error %d\n",error);
  if (error < backlash_position_offset)
    {

      *pid2 = sign * backlash_motor_offset;
      //printf("Backlash %ld\n",*pid2);

    }

}

int
pointing_model_correction (double Azimuth, double Altitude,
			   unsigned int *AZ_ENCODER,
			   unsigned int *ALT_ENCODER)
{


  return 0;
}


unsigned int
ad5362_crc_pack1 (unsigned int command, unsigned int channel,
		  unsigned int value)
{
  //this function takes inputs and prepares a suitable packet to be transmitted to the ad5362 DAC. Includeds a CRC for safety so will produce a four byte value stored in the unsigned int.
  unsigned char test_vec[10];
  unsigned int retval;
  unsigned int packet;
  crc_t crc;
  packet =
    (command << 20) | (channel << 16) | (((unsigned short) value) &
					 (0x00ffff));
  test_vec[0] = (packet & 0xff000000) >> 24;
  test_vec[1] = (packet & 0x00ff0000) >> 16;
  test_vec[2] = (packet & 0x0000ff00) >> 8;
  test_vec[3] = (packet & 0x000000ff);
  crc = crc_init ();
  crc = crc_update (crc, test_vec, 4);
  crc = crc_finalize (crc);
  retval = (packet << 8) | (crc & (0x000000ff));
  return retval;
}


unsigned int
status_query (unsigned int *return_vec)
{
  unsigned int max7301[5];
  unsigned int max7301RX[5];
  //this function calls a general read of the status of the mechanical switches



  //printf("Status Query\n");
  max7301[0] = 0xcc00;		//chip1 12-19
  max7301[1] = 0xcc00;		//chip2 12-19
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_READ_MAX7301, max7301);
  *(return_vec) = max7301[2];	//chip1 12-19
  *(return_vec + 1) = max7301[3];	//chip2 12-19

  max7301[0] = 0xd400;		//chip1 20-27
  max7301[1] = 0xcc00;		//chip2 12-19
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_READ_MAX7301, max7301);
  //printf("A- [Chip1 P24-31 and Chip2 P12-19] Returns %04x %04x\n",max7301[2],max7301[3]);

  *(return_vec + 2) = max7301[2];	//chip1 20-27
  *(return_vec + 3) = max7301[3];	//chip2 12-19
  max7301[0] = 0xd800;		//chip1 24-31
  max7301[1] = 0xd400;		//chip2 20-27
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_READ_MAX7301, max7301);
  //printf("B- [Chip1 P24-31] and Chip2 20-27] Returns %04x %04x\n",max7301[2],max7301[3]);
  *(return_vec + 4) = max7301[2];	//chip2 24-31
  *(return_vec + 5) = max7301[3];	//chip2 20-27


  max7301[0] = 0xd800;		//chip1 24-31
  max7301[1] = 0xdc00;		//chip1 28-31
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_READ_MAX7301, max7301);
  //printf("C- [Chip1 P24-31] and Chip2 28-31] Returns %04x %04x\n",max7301[2],max7301[3]);
  *(return_vec + 6) = max7301[3];	//chip2 28-31
  //printf("Status Query : \nChip1 12-19 %04x\nChip1 20-27 %04x\nChip1 P24-31 %04x\nChip2 P12-19 %04x\nChip2 P20-27 %04x\nChip2 P28-31 %04x\n",*(return_vec),*(return_vec+2),*(return_vec+4),*(return_vec+3),*(return_vec+5),*(return_vec+6));

}

unsigned int
contactors (unsigned int command)
{
  unsigned int max7301[5];
  unsigned int max7301RX[5];

//      Contactors occupy Max7301 Chip 1 20,21,22,23 to engage- i.e turn 20,21,22,23 hi to engage the contactors
//      The feedback from the contactors is in Chip1 pin 26,30 and Chip2 pin 14 and 18- If the contactor is engaged the pins are pulled low and if the contactor is disengaged the pins are pulled high. So when contactors are engaged we expect Chip 1 24-31 to be 00000000 (0x0000) and Chip 2 12-19 to be 00000000 (0x0000) and with them disengaged we expect Chip1 24-31 to be 00100010 (0x44) and Chip 2 12-19 to be 00100010 (0x44)
//SEE THE MAX7301 DATASHEET FOR INFORMATION ABOUT HOW TO READ AND WRITE TO THE MAX7301

  printf
    ("CONTACTORS OFF Return Should be \"CONTACTORS END 1 Returns 0x0044 0x0044\n");
  printf
    ("CONTACTORS ON Return Should be \"CONTACTORS END 1 0x0000 0x0000\n");
  max7301[0] = 0xd800;		//chip1 24-31
  max7301[1] = 0xcc00;		//chip1 12-19
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_READ_MAX7301, max7301);
  printf
    ("CONTACTORS START 1 [Chip1 P24-31 and Chip2 P12-19] Returns %04x %04x\n",
     max7301[2], max7301[3]);

  max7301[0] = 0xd800;		//chip1 24-31
  max7301[1] = 0xd400;		//chip1 20-27
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_READ_MAX7301, max7301);
  printf
    ("CONTACTORS START 2 [Chip1 P24-31] and Chip2 20-27] Returns %04x %04x\n",
     max7301[2], max7301[3]);

  max7301[0] = 0xd800;		//chip1 24-31
  max7301[1] = 0xdc00;		//chip1 28-31
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_READ_MAX7301, max7301);
  printf
    ("CONTACTORS START 3 [Chip1 P24-31] and Chip2 28-31] Returns %04x %04x\n",
     max7301[2], max7301[3]);

  //sleep(1);
  switch (command)
    {


    case 0:
      printf ("Contactors Off\n");
      max7301[0] = 0x3400;
      max7301[1] = 0;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      max7301[0] = 0x3500;
      max7301[1] = 0;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      max7301[0] = 0x3600;
      max7301[1] = 0;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      max7301[0] = 0x3700;
      max7301[1] = 0;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);

      break;

    case 1:
      printf ("Contactors On\n");
      max7301[0] = 0x34ff;
      max7301[1] = 0;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      max7301[0] = 0x35ff;
      max7301[1] = 0;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      max7301[0] = 0x36ff;
      max7301[1] = 0;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      max7301[0] = 0x37ff;
      max7301[1] = 0;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      printf ("Contactors are Engaged- Wait ~15 seconds for Warning Siren\n");
      //sleep(10);

      printf ("Warning Siren completed\n");
      break;

    case 2:			//just the azimuth off
      max7301[0] = 0x3400;
      max7301[1] = 0;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      max7301[0] = 0x3500;
      max7301[1] = 0;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
	break;

    case 3:			//just the azimuth on
      max7301[0] = 0x34ff;
      max7301[1] = 0;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      max7301[0] = 0x35ff;
      max7301[1] = 0;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      break;

    case 4:			//just the elevation off
      max7301[0] = 0x3600;
      max7301[1] = 0;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      max7301[0] = 0x3700;
      max7301[1] = 0;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      break;

    case 5:			//just the elevation on
      max7301[0] = 0x36ff;
      max7301[1] = 0;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      max7301[0] = 0x37ff;
      max7301[1] = 0;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      break;

    }
  //sleep(1);

  max7301[0] = 0xd800;		//chip1 24-31
  max7301[1] = 0xcc00;		//chip2 12-19
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_READ_MAX7301, max7301);
  printf
    ("CONTACTORS END 1 [Chip1 P24-31 and Chip2 P12-19] Returns  %04x %04x\n",
     max7301[2], max7301[3]);

  max7301[0] = 0xd800;		//chip1 24-31
  max7301[1] = 0xd400;		//chip2 20-27
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_READ_MAX7301, max7301);
  printf
    ("CONTACTORS END 2 [Chip1 P24-31 and Chip2 P20-27] Returns  %04x %04x\n",
     max7301[2], max7301[3]);

  max7301[0] = 0xd800;		//chip1 24-31
  max7301[1] = 0xdc00;		//chip2 28-31
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_READ_MAX7301, max7301);
  printf
    ("CONTACTORS END 3 [Chip1 P24-31 and Chip2 P28-31] Returns  %04x %04x\n",
     max7301[2], max7301[3]);

}


unsigned int
clutchbrake (unsigned int command, unsigned int *return_sts)
{
  unsigned int max7301[5];
  unsigned int max7301RX[5];
  unsigned int STS_VEC[STS_VEC_SIZE];

  //printf("clutchbrake");
  //rintf("CONTACTORS ON Return Should be \"CONTACTORS END 1 0x0000 0x0000\n");
  max7301[0] = 0xd800;		//chip1 24-31
  max7301[1] = 0xcc00;		//chip1 12-19
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_READ_MAX7301, max7301);
  memcpy (&STS_VEC[0], &max7301[2], sizeof (max7301[2]));
  memcpy (&STS_VEC[1], &max7301[3], sizeof (max7301[3]));
  //STS_VEC[0] = max7301[2];

  //STS_VEC[1]=max7301[3];
  //        printf("CONTACTORS START 1 [Chip1 P24-31 and Chip2 P12-19] Returns %04x %04x\n",max7301[2],max7301[3]);
  max7301[0] = 0xd800;		//chip1 24-31
  max7301[1] = 0xd400;		//chip1 20-27
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_READ_MAX7301, max7301);
  //S_VEC[2] = max7301[2];
  memcpy (&STS_VEC[2], &max7301[3], sizeof (max7301[3]));
  //STS_VEC[2]=max7301[3];
  // printf("CONTACTORS START 2 [Chip1 P24-31] and Chip2 20-27] Returns %04x %04x\n",max7301[2],max7301[3]);

  max7301[0] = 0xd800;		//chip1 24-31
  max7301[1] = 0xdc00;		//chip1 28-31
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_READ_MAX7301, max7301);
  memcpy (&STS_VEC[3], &max7301[3], sizeof (max7301[3]));
  //STS_VEC[3]=max7301[3];
  //printf("CONTACTORS START 2 [Chip1 P24-31] and Chip2 20-27] Returns %04x %04x %04x %04x %04x %04x\n",max7301[2],max7301[3],STS_VEC[0],STS_VEC[1],STS_VEC[2],STS_VEC[3]);
  memcpy (return_sts, STS_VEC, sizeof (STS_VEC));


  switch (command)
    {
    case 0:
      printf ("Clutch Brake off\n");
      max7301[0] = 0x4c00;
      max7301[1] = 0x0000;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);

//              max7301[0] = 0x2c00;
//              max7301[1] = 0x0000;
//              max7301[2]=2;
//              ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
//              max7301[0] = 0x2d00;
//              max7301[1] = 0x0000;
//              max7301[2]=2;
//              ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
//              max7301[0] = 0x2e00;
//              max7301[1] = 0x0000;
//              max7301[2]=2;
//              ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
//              max7301[0] = 0x2f00;
//              max7301[1] = 0x0000;
//              max7301[2]=2;
//              ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
//              max7301[0] = 0x3000;
//              max7301[1] = 0x0000;
//              max7301[2]=2;
//              ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
//              max7301[0] = 0x3100;
//              max7301[1] = 0x0000;
//              max7301[2]=2;
//              ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
      break;

    case 1:
      printf ("Clutch Brake on\n");
      max7301[0] = 0x4cff;
      max7301[1] = 0x0000;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);


//              max7301[0] = 0x2cff;
//              max7301[1] = 0x0000;
//              max7301[2]=2;
//              ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301); //BRAKE AZimuth
//              max7301[0] = 0x2dff;
//              max7301[1] = 0x0000;
//              max7301[2]=2;
//              ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
//              max7301[0] = 0x2eff;
//              max7301[1] = 0x0000;
//              max7301[2]=2;
//              ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
//              max7301[0] = 0x2fff;
//              max7301[1] = 0x0000;
//              max7301[2]=2;
//              ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
//              max7301[0] = 0x30ff;
//              max7301[1] = 0x0000;
//              max7301[2]=2;
//              ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
//              max7301[0] = 0x31ff;
//              max7301[1] = 0x0000;
//              max7301[2]=2;
//              ioctl(fd,DEV_IOCTL_WRITE_MAX7301,max7301);
      break;
    case 2:			//az brake off
      //      printf("Clutch Brake off\n");
      max7301[0] = 0x2c00;
      max7301[1] = 0x0000;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      break;
    case 3:			//az brake on
      //      printf("Clutch Brake off\n");
      max7301[0] = 0x2cff;
      max7301[1] = 0x0000;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      break;
    case 4:			//el brake off
      //      printf("Clutch Brake off\n");
      max7301[0] = 0x2f00;
      max7301[1] = 0x0000;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      break;
    case 5:			//el brake on
      //      printf("Clutch Brake off\n");
      max7301[0] = 0x2fff;
      max7301[1] = 0x0000;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      break;
    case 6:			//az cluthc off
      //      printf("Clutch Brake off\n");
      max7301[0] = 0x2d00;
      max7301[1] = 0x0000;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      max7301[0] = 0x2e00;
      max7301[1] = 0x0000;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      break;
    case 7:			//az clutch on
      //      printf("Clutch Brake off\n");
      max7301[0] = 0x2dff;
      max7301[1] = 0x0000;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      max7301[0] = 0x2eff;
      max7301[1] = 0x0000;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      break;
    case 8:			//el clutch off
      //      printf("Clutch Brake off\n");
      max7301[0] = 0x3000;
      max7301[1] = 0x0000;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      max7301[0] = 0x3100;
      max7301[1] = 0x0000;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      break;
    case 9:			//el clutch on
      //      printf("Clutch Brake off\n");
      max7301[0] = 0x30ff;
      max7301[1] = 0x0000;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      max7301[0] = 0x31ff;
      max7301[1] = 0x0000;
      max7301[2] = 2;
      ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
      break;
    case 10:			//engage servo voltage commands
      ioctl (fd, DEV_IOCTL_ENABLE_DAC, max7301);
      break;
    case 11:			//disable servo voltage commands
      ioctl (fd, DEV_IOCTL_DISABLE_DAC, max7301);
      break;
    default:
      memcpy (return_sts, STS_VEC, sizeof (STS_VEC));

    }

  //printf("utchbrake\n");                        
  //printf("CONTACTORS ON Return Should be \"CONTACTORS END 1 0x0000 0x0000\n");
  //max7301[0]=0xd800; //chip1 24-31
  //max7301[1]=0xcc00;//chip1 12-19
  //max7301[2]=2;
  //ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
  //printf("CONTACTORS START 1 [Chip1 P24-31 and Chip2 P12-19] Returns %04x %04x\n",max7301[2],max7301[3]);

  //max7301[0]=0xd800; //chip1 24-31
  //max7301[1]=0xd400;//chip1 20-27
  //max7301[2]=2;
  //ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
  //printf("CONTACTORS START 2 [Chip1 P24-31] and Chip2 20-27] Returns %04x %04x\n",max7301[2],max7301[3]);

  //max7301[0]=0xd800;//chip1 24-31
  //max7301[1]=0xdc00;//chip1 28-31
  //max7301[2]=2;
  //ioctl(fd,DEV_IOCTL_READ_MAX7301,max7301);
  //printf("CONTACTORS START 3 [Chip1 P24-31] and Chip2 28-31] Returns %04x %04x\n",max7301[2],max7301[3]);




}

unsigned int
max7301_init (void)
{
  unsigned int max7301[5];
  unsigned int max7301RX[5];
  max7301[0] = 0x0401;
  max7301[1] = 0x0401;
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);

  max7301[0] = 0x0955;
  max7301[1] = 0x0955;
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
  max7301[0] = 0x0a55;
  max7301[1] = 0x0a55;
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);

  //set Chip 1 port 12-15 as output Chip 2 port 12-15 as Schmitt logic input with pullup 
  max7301[0] = 0x0b55;
  max7301[1] = 0x0bff;
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);

  //set Chip 1 port 16-19 as output Chip 2 port 16-19 as Schmitt logic input with pullup 
  max7301[0] = 0x0c55;
  max7301[1] = 0x0cff;
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
  //set Chip 1 port 20-23 as output Chip 2 port 20-23 as Schmitt logic input with pullup 
  max7301[0] = 0x0d55;
  max7301[1] = 0x0dff;
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
  //set Chip 1 port 24-27 as as Schmitt logic input with pullup  Chip 2 port 24-27 as Schmitt logic input with pullup 
  max7301[0] = 0x0eff;
  max7301[1] = 0x0eff;
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);
  //set Chip 1 port 28-31 as as Schmitt logic input with pullup Chip 2 port 28-31 as as Schmitt logic input with pullup 
  max7301[0] = 0x0fff;
  max7301[1] = 0x0fff;
  max7301[2] = 2;
  ioctl (fd, DEV_IOCTL_WRITE_MAX7301, max7301);

}

void
sort (double *unsorted, double *sorted, int length)
{


  double tmp, tmp2;
  double working[10], sorting[10];
  int i, j;

  //i=length;
  for (i = 0; i < length; i++)
    {
      *(working + i) = *(unsorted + i);
      *(sorting + i) = *(unsorted + i);
      //printf("Starting Unsorted %d %f Sorted %f \n",i,*(working+i),*(sorting+i));
      //printf("AZ TEST SORTING %d\n",az_test2[i]);

    }
  // printf("Sorting Vals\n");
  for (i = 0; i < length; i++)
    {
      //printf("Unsorted %f Sorted %f \n",*(working+i),*(sorting+i));
      for (j = 0; j < length - i; j++)
	if (*(sorting + j + 1) < *(sorting + j))
	  {			/* compare the two neighbors */
	    tmp = *(sorting + j);	/* swap a[j] and a[j+1]      */
	    tmp2 = *(working + j);
	    *(sorting + j) = *(sorting + j + 1);
	    *(working + j) = *(working + j + 1);
	    *(sorting + j + 1) = tmp;
	    *(working + j + 1) = tmp2;
	  }
      //printf("22 Unsorted %f Sorted %f \n",*(unsorted+i),*(sorted+i));      
    }

  for (i = 0; i < length; i++)
    {
      //printf("Ending %d Unsorted %f Sorted %f \n",i,*(working+i),*(sorting+i));
      *(sorted + i) = *(sorting + i);
    }

}
