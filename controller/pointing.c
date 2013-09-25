#include <stdio.h> 
#include <stdlib.h> 
#include <sys/types.h> 
#include <sys/stat.h> 
#include <sys/time.h>
#include "pointing.h"
#include "slalib.h"
#include "telescope_constants.h"
#define PI 3.141592653589793238462643
#define R2D (180.0/PI) /* radians to degrees */

//this file contains routines for conversion between the raw encoder values, antenna azimuth/elevation coordinates (no pointing correction this is assumed to be done by the control PC)- it also contains the software limit routines to prevent
//the antenna driving into limits

unsigned int azimuth_angle_no_sort(double azimuth_command,unsigned int azimuth_encoder_long,unsigned int P0,unsigned int az_min,unsigned int az_max,double *azimuth_command_out)
{
	double current_azimuth;
	double p0d,p1d;//declare variables to store the angle encoder values;
	double az_mind,az_maxd;
	double az_com_temp;
	
	
	p0d = (double)P0;
	p1d = (double)azimuth_encoder_long;
	current_azimuth = (p1d-p0d)*360.;
	current_azimuth = current_azimuth/65535.;
	
	
	az_mind = (double)az_min;
	az_mind-=p0d;
	az_mind = az_mind*360.;
	az_mind = az_mind/65535.;
	
	az_maxd = (double)az_max;
	az_maxd -=p0d;
	az_maxd = az_maxd*360.;
	az_maxd = az_maxd/65535.;
	
	//printf("%f %f %f %f\n",current_azimuth,az_mind,az_maxd,azimuth_command);
	az_com_temp = azimuth_command;
	//printf("Min %f Max %f\n",az_mind,az_maxd);
	//sort_azimuth_double_new(current_azimuth,azimuth_command,az_maxd,az_mind,&az_com_temp);
	
	if(az_com_temp<=az_mind){
	   az_com_temp=az_mind;
	}
	
	if(az_com_temp>=az_maxd){
	 az_com_temp=az_maxd;
	}
	//printf("Current az %f\n",current_azimuth);
	*azimuth_command_out = az_com_temp;
	
  
}

unsigned int azimuth_angle(double azimuth_command,unsigned int azimuth_encoder_long,unsigned int P0,unsigned int az_min,unsigned int az_max,double *azimuth_command_out){
	double current_azimuth;
	double p0d,p1d;//declare variables to store the angle encoder values;
	double az_mind,az_maxd;
	double az_com_temp;
	
	
	p0d = (double)P0;
	p1d = (double)azimuth_encoder_long;
	current_azimuth = (p1d-p0d)*360.;
	current_azimuth = current_azimuth/65535.;
	
	az_mind = (double)az_min;
	az_mind-=(double)P0;
	az_mind = az_mind*360.;
	az_mind = az_mind/65535.;
	
	az_maxd = (double)az_max;
	az_maxd -=(double)P0;
	az_maxd = az_maxd*360.;
	az_maxd = az_maxd/65535.;
	
	az_com_temp = azimuth_command;
	
	sort_azimuth_double_new(current_azimuth,azimuth_command,az_maxd,az_mind,&az_com_temp);
	
	if(az_com_temp<=az_mind){
	    az_com_temp+=360.;
	}
	
	if(az_com_temp>=az_maxd){
	  az_com_temp-=360.;
	}
	
	*azimuth_command_out = az_com_temp;
	
  
}

unsigned int pointing(double AZ,double ALT,double *delta_AZ_Out, double *delta_ALT_Out){


	float P1,P2,P3,P4,P5,P6,P7,P8,P9,P10,P11,P12,P13,P14,P15,P16;
	double deltaX,deltaY,X,Y;
	int j;
	char testbuff[100000];
	char commands[100][100];
	FILE *pointing_file;
	pointing_file = fopen("pointing_coeffs.txt","r");
		if (pointing_file == NULL) {
		  printf("I couldn't open pointing_coeffs.txt for writing.\n");
		  exit(0);
		}
		j=0;
		while(fgets(testbuff, 400, pointing_file) != NULL){
		//char_ptr =fgets(testbuff,50,pid_file);
		//printf("pointing.c testbuff %s \n",testbuff);
		sscanf(testbuff,"%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^',']\n",&commands[0],&commands[1],&commands[2],&commands[3],&commands[4],&commands[5],&commands[6],&commands[7],&commands[8],&commands[9],&commands[10],&commands[11],&commands[12],&commands[13],&commands[14],&commands[15]);
		
		
		P1 = atof(commands[0]);
		P2 = atof(commands[1]);
		P3 = atof(commands[2]);
		P4 = atof(commands[3]);
		P5 = atof(commands[4]);
		P6 = atof(commands[5]);
		P7 = atof(commands[6]);
		P8= atof(commands[7]);
		P9 = atof(commands[8]);
		P10= atof(commands[9]);
		P11= atof(commands[10]);
		P12= atof(commands[11]);
		P13= atof(commands[12]);
		P14= atof(commands[13]);
		P15= atof(commands[14]);
		P16= atof(commands[15]);
		
		
		
		//printf("Pointing Coeffs %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",P1,P2,P3,P4,P5,P6,P7,P8,P9,P10,P11,P12,P13,P14,P15,P16);
		}
	
	X= AZ/R2D;
	Y= ALT/R2D;
	X= slaDranrm(X);
	Y = slaDranrm(Y);
	
	//printf("Pointing Coeffs %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",P1,P2,P3,P4,P5,P6,P7,P8,P9,P10,P11,P12,P13,P14,P15,P16);
	//P1 = +2; //azimuth blind offset
	//P2 = -0.00;
	//P3 = -0.00;
	//P4 = -0.00;
	//P5 = 0.0;
	//P6 = -0.0;
	//P7 = 1.5; //elevation blind offset
	//P8 = -0.0;
	//P9 = -0.00;
	//P10 = -0.000;
	//P11 = 0.0;
	//P12 = 0.0;
	//P13 = 0.00;
	//P14 = -0.;
	//P15 = -0.000;
	//P16 = 0.00;
// 	P1 = 0.;
// 	P2 = 0.;
// 	P3 = 0.;
// 	P4 = 0.;
// 	P5 = 0.;
// 	P6 = 0.;
// 	P7 = 0.;
// 	P8 = 0.;
// 	P9 = 0.;
// 	P10 = 0.;
// 	P11 = 0.;
// 	P12 = 0.;
// 	P13 = 0.;
// 	P14 = 0.;
// 	P15 = 0.;
// 	P16 = 0.;

	deltaX = 0.;
	deltaY = 0.;
	deltaX = P1 - P2*sin(X)/cos(Y) + P3*tan(Y) - P4/cos(Y) + P5*sin(Y)*tan(Y) - P6*cos(X)*tan(Y) + P12*X + P13*cos(X) + P14*sin(X)+P15*cos(2*X) + P16*sin(2*X);
	deltaY = P5*cos(X) + P6*sin(X) + P7 - P8*(cos(X)*sin(Y) - cos(Y)) + P9*Y + P10*cos(Y) + P11*sin(Y);
	

	*delta_AZ_Out =deltaX;
	*delta_ALT_Out = deltaY;
	//printf("pointing model %f %f %f %f %f %f\n",AZ,AZ+deltaX,deltaX,ALT,ALT+deltaY,deltaY);


	return 1;
}


unsigned int pointing2(double AZ,double ALT,double *delta_AZ_Out, double *delta_ALT_Out){


	double P1,P2,P3,P4,P5,P6,P7,P8,P9,P10,P11,P12,P13,P14,P15,P16;
	double deltaX,deltaY,X,Y;
	X= AZ/R2D;
	Y= ALT/R2D;
	X= slaDranrm(X);
	Y = slaDranrm(Y);
	P1 = -0.104;
	P2 = -0.000;
	P3 = -0.00;
	P4 = -0.00;
	P5 = 0.0;
	P6 = -0.0;
	P7 = 0.55;
	P8 = -0.00;
	P9 = -0.00;
	P10 = -0.000;
	P11 = 0.00;
	P12 = 0.00;
	P13 = 0.000;
	P14 = -0.0;
	P15 = -0.0000;
	P16 = 0.000;
// 	P1 = 0.;
// 	P2 = 0.;
// 	P3 = 0.;
// 	P4 = 0.;
// 	P5 = 0.;
// 	P6 = 0.;
// 	P7 = 0.;
// 	P8 = 0.;
// 	P9 = 0.;
// 	P10 = 0.;
// 	P11 = 0.;
// 	P12 = 0.;
// 	P13 = 0.;
// 	P14 = 0.;
// 	P15 = 0.;
// 	P16 = 0.;

	deltaX = 0.;
	deltaY = 0.;
	deltaX = P1 - P2*sin(X)/cos(Y) + P3*tan(Y) - P4/cos(Y) + P5*sin(Y)*tan(Y) - P6*cos(X)*tan(Y) + P12*X + P13*cos(X) + P14*sin(X)+P15*cos(2*X) + P16*sin(2*X);
	deltaY = P5*cos(X) + P6*sin(X) + P7 - P8*(cos(X)*sin(Y) - cos(Y)) + P9*Y + P10*cos(Y) + P11*sin(Y);
	

	*delta_AZ_Out =deltaX;
	*delta_ALT_Out = deltaY;
	//printf("pointing model %f %f %f %f %f %f\n",AZ,AZ+deltaX,deltaX,ALT,ALT+deltaY,deltaY);


	return 1;
}


unsigned int pointing2AZ(double AZ,double ALT,double *delta_AZ_Out, double *delta_ALT_Out){


	double P1,P2,P3,P4,P5,P6,P7,P8,P9,P10,P11,P12,P13,P14,P15,P16;
	double deltaX,deltaY,X,Y;
	X= AZ/R2D;
	Y= ALT/R2D;
	X= slaDranrm(X);
	Y = slaDranrm(Y);
	P1 = -0.0104;
	P2 = -0.006;
	P3 = -0.005;
	P4 = -0.002;
	P5 = 0.05;
	P6 = -0.02;
	P7 = -0.05;
	P8 = -0.0245;
	P9 = -0.001;
	P10 = -0.0002;
	P11 = 0.0175;
	P12 = 0.0496;
	P13 = 0.004;
	P14 = -0.103;
	P15 = -0.00085;
	P16 = 0.00893;
// 	P1 = 0.;
// 	P2 = 0.;
// 	P3 = 0.;
// 	P4 = 0.;
// 	P5 = 0.;
// 	P6 = 0.;
// 	P7 = 0.;
// 	P8 = 0.;
// 	P9 = 0.;
// 	P10 = 0.;
// 	P11 = 0.;
// 	P12 = 0.;
// 	P13 = 0.;
// 	P14 = 0.;
// 	P15 = 0.;
// 	P16 = 0.;

	deltaX = 0.;
	//deltaY = 0.;
	deltaX = P1 - P2*sin(X)/cos(Y) + P3*tan(Y) - P4/cos(Y) + P5*sin(Y)*tan(Y) - P6*cos(X)*tan(Y) + P12*X + P13*cos(X) + P14*sin(X)+P15*cos(2*X) + P16*sin(2*X);
	//deltaY = P5*cos(X) + P6*sin(X) + P7 - P8*(cos(X)*sin(Y) - cos(Y)) + P9*Y + P10*cos(Y) + P11*sin(Y);
	

	*delta_AZ_Out =deltaX;
	//*delta_ALT_Out = deltaY;
	//printf("pointing model %f %f %f %f %f %f\n",AZ,AZ+deltaX,deltaX,ALT,ALT+deltaY,deltaY);
	//printf("AZ pointing\n");

	return 1;
}


unsigned int pointing2ALT(double AZ,double ALT,double *delta_AZ_Out, double *delta_ALT_Out){


	double P1,P2,P3,P4,P5,P6,P7,P8,P9,P10,P11,P12,P13,P14,P15,P16;
	double deltaX,deltaY,X,Y;
	X= AZ/R2D;
	Y= ALT/R2D;
	X= slaDranrm(X);
	Y = slaDranrm(Y);
	P1 = -0.0104;
	P2 = -0.006;
	P3 = -0.005;
	P4 = -0.002;
	P5 = 0.05;
	P6 = -0.02;
	P7 = -0.05;
	P8 = -0.0245;
	P9 = -0.001;
	P10 = -0.0002;
	P11 = 0.0175;
	P12 = 0.0496;
	P13 = 0.004;
	P14 = -0.103;
	P15 = -0.00085;
	P16 = 0.00893;
// 	P1 = 0.;
// 	P2 = 0.;
// 	P3 = 0.;
// 	P4 = 0.;
// 	P5 = 0.;
// 	P6 = 0.;
// 	P7 = 0.;
// 	P8 = 0.;
// 	P9 = 0.;
// 	P10 = 0.;
// 	P11 = 0.;
// 	P12 = 0.;
// 	P13 = 0.;
// 	P14 = 0.;
// 	P15 = 0.;
// 	P16 = 0.;

	//deltaX = 0.;
	deltaY = 0.;
	//deltaX = P1 - P2*sin(X)/cos(Y) + P3*tan(Y) - P4/cos(Y) + P5*sin(Y)*tan(Y) - P6*cos(X)*tan(Y) + P12*X + P13*cos(X) + P14*sin(X)+P15*cos(2*X) + P16*sin(2*X);
	deltaY = P5*cos(X) + P6*sin(X) + P7 - P8*(cos(X)*sin(Y) - cos(Y)) + P9*Y + P10*cos(Y) + P11*sin(Y);
	

	//*delta_AZ_Out =deltaX;
	*delta_ALT_Out = deltaY;
	//printf("ALT pointing\n");


	return 1;
}



void sort_azimuth_double_new(double azimuth_encoder,double azimuth_command,double azimuth_max,double azimuth_min,double *correct_azimuth){
		double az_test[5];
		double az_temp;
		double az_test2[5];
		double tmp,tmp2;
		//double azimuth_encoder;
		int AZ;
		int i,j;


		
		

		az_test[0]=azimuth_command - 360.*2.;
		az_test[1]=azimuth_command - 360.;
		az_test[2]=azimuth_command;
		az_test[3]=azimuth_command + 360.;
		az_test[4]=azimuth_command + 360.*2.;
		for(i=0;i<=4;i++){
			az_test2[i] = abs(azimuth_encoder - az_test[i]);
			//printf("AZ TEST SORTING %f\n",az_test2[i]);
			
		}
		for (i=0; i<4-1; i++) {
  			for (j=0; j<4-1-i; j++)
   				 if (az_test2[j+1] < az_test2[j]) {  /* compare the two neighbors */
    				  	tmp = az_test2[j];         /* swap a[j] and a[j+1]      */
					tmp2 = az_test[j];
     				 	az_test2[j] = az_test2[j+1];
					az_test[j] = az_test[j+1];
     				 	az_test2[j+1] = tmp;
					az_test[j+1]= tmp2;
  				}
			}
		az_temp = az_test[0];
		if(az_temp > azimuth_max){
			az_temp = az_test[1];
		}

		if(az_temp < azimuth_min){
			az_temp = az_test[1];
		}
	
		*(correct_azimuth) = az_temp;


}

void sort_azimuth_double(unsigned int azimuth_encoder_long,double azimuth_command,double azimuth_max,double azimuth_min,double *correct_azimuth){
		double az_test[5];
		double az_temp;
		double az_test2[5];
		double tmp,tmp2;
		double azimuth_encoder;
		int AZ;
		int i,j;


		AZ = (int)azimuth_encoder_long - AZIMUTH_ZERO;
		azimuth_encoder = ((double)AZ*360.)/65535.;

		az_test[0]=azimuth_command - 360.*2.;
		az_test[1]=azimuth_command - 360.;
		az_test[2]=azimuth_command;
		az_test[3]=azimuth_command + 360.;
		az_test[4]=azimuth_command + 360.*2.;
		for(i=0;i<=4;i++){
			az_test2[i] = abs(azimuth_encoder - az_test[i]);
			//printf("AZ TEST SORTING %f\n",az_test2[i]);
			
		}
		for (i=0; i<4-1; i++) {
  			for (j=0; j<4-1-i; j++)
   				 if (az_test2[j+1] < az_test2[j]) {  /* compare the two neighbors */
    				  	tmp = az_test2[j];         /* swap a[j] and a[j+1]      */
					tmp2 = az_test[j];
     				 	az_test2[j] = az_test2[j+1];
					az_test[j] = az_test[j+1];
     				 	az_test2[j+1] = tmp;
					az_test[j+1]= tmp2;
  				}
			}
		az_temp = az_test[0];
		if(az_temp > azimuth_max){
			az_temp = az_test[1];
		}

		if(az_temp < azimuth_min){
			az_temp = az_test[1];
		}
	
		*(correct_azimuth) = az_temp;


}


void sort_azimuth(unsigned int azimuth_encoder,unsigned int azimuth_command,unsigned int azimuth_max,unsigned int azimuth_min,volatile unsigned int *correct_azimuth){
		unsigned int az_test[5];
		unsigned int az_temp;
		long az_test2[5];
		long tmp,tmp2;
		int i,j;

		az_test[0]=azimuth_command - 65535*2;
		az_test[1]=azimuth_command - 65535;
		az_test[2]=azimuth_command;
		az_test[3]=azimuth_command + 65535;
		az_test[4]=azimuth_command + 65535*2;
		for(i=0;i<=4;i++){
			az_test2[i] = abs((long)azimuth_encoder - (long)az_test[i]);
			//printf("AZ TEST SORTING %d\n",az_test2[i]);
			
		}
		for (i=0; i<4-1; i++) {
  			for (j=0; j<4-1-i; j++)
   				 if (az_test2[j+1] < az_test2[j]) {  /* compare the two neighbors */
    				  	tmp = az_test2[j];         /* swap a[j] and a[j+1]      */
					tmp2 = az_test[j];
     				 	az_test2[j] = az_test2[j+1];
					az_test[j] = az_test[j+1];
     				 	az_test2[j+1] = tmp;
					az_test[j+1]= tmp2;
  				}
			}
		az_temp = az_test[0];
		if(az_temp > azimuth_max){
			az_temp = az_test[1];
		}

		if(az_temp < azimuth_min){
			az_temp = az_test[1];
		}
	
// this is the sorted valueCommented out to experiment with not having sorting		*(correct_azimuth) = az_temp;
			*(correct_azimuth) = azimuth_command;


}


unsigned int azalt2encoderearly(double AZ, double ALT,unsigned int *AZ_Encoder, unsigned int *ALT_Encoder){
	unsigned int AZ_command,ALT_command;
	//AZ +=dAZ;
	//ALT+=dALT;
	AZ_command = (unsigned int)(65535.*(AZ/360.));
	ALT_command = (unsigned int)(65535.*(ALT/360.));
	//define in telescope_constants.h
	AZ_command+=AZIMUTH_ZERO;
	//define in telescope_constants.h
	ALT_command+=ELEVATION_ZERO;
	*ALT_Encoder = ALT_command;
	*AZ_Encoder = AZ_command;
	
	

}

unsigned int azalt2encoder(double AZ, double dAZ, double ALT, double dALT,volatile unsigned int *AZ_Encoder,volatile unsigned int *ALT_Encoder){
	unsigned int AZ_command,ALT_command;
	AZ +=dAZ;
	ALT+=dALT;
	AZ_command = (unsigned int)(65535.*(AZ/360.));
	ALT_command = (unsigned int)(65535.*(ALT/360.));
	//define in telescope_constants.h
	AZ_command+=AZIMUTH_ZERO;
	//define in telescope_constants.h
	ALT_command+=ELEVATION_ZERO;
	*ALT_Encoder = ALT_command;
	*AZ_Encoder = AZ_command;
	
	

}


unsigned int encoder2azalt(unsigned int AZ_Encoder, double delta_AZ, unsigned int ALT_Encoder,double delta_ALT,double *AZ, double *ALT){
	int AZprime,ALTprime;
	double AZrad,ALTrad;
	AZprime=(int)AZ_Encoder - AZIMUTH_ZERO;
	ALTprime = (int)ALT_Encoder - ELEVATION_ZERO;
	*AZ = ((double)AZprime*360.)/65535.;	
	*ALT = ((double)ALTprime*360.)/65535.;

	*AZ -=delta_AZ;
	*ALT -=delta_ALT;
	AZrad = *AZ/R2D;
	ALTrad =*ALT/R2D;
	//AZrad= slaDranrm(AZrad);
	
	ALTrad = slaDranrm(ALTrad);
	*AZ =AZrad*R2D;
	//if(*AZ<0){
	//  *AZ+=360.;
	//}
	*ALT =ALTrad*R2D;	

	
	
	

}
unsigned int encoder2azaltprime(unsigned int AZ_Encoder, double delta_AZ, unsigned int ALT_Encoder,double delta_ALT,volatile double *AZ,volatile double *ALT){
	int AZprime,ALTprime;
	double AZrad,ALTrad;

	AZprime=(int)AZ_Encoder - AZIMUTH_ZERO;
	ALTprime = (int)ALT_Encoder - ELEVATION_ZERO;
	*AZ = ((double)AZprime*360.)/65535.;
	*ALT = ((double)ALTprime*360.)/65535.;
	AZrad = *AZ/R2D;
	ALTrad =*ALT/R2D;
	//AZrad= slaDranrm(AZrad);
	ALTrad = slaDranrm(ALTrad);
	*AZ =AZrad*R2D;
	*ALT =ALTrad*R2D;	

	
	
	

}


unsigned int azaltprime2azalt(double AZprime, double delta_AZ, double ALTprime,double delta_ALT,double *AZ, double *ALT){
	double AZout,ALTout,dAZ,dALT,AZrad,ALTrad;
	double errAZ,errALT;
	unsigned int count =0;
	AZrad =AZprime/R2D;
	ALTrad = ALTprime/R2D;
	AZrad = slaDranrm(AZrad);
	ALTrad = slaDranrm(ALTrad);
	AZprime = AZrad*R2D;
	ALTprime = ALTrad*R2D;
	AZout = AZrad * R2D;
	ALTout = ALTrad*R2D;

	AZout = AZout - delta_AZ;
	ALTout = ALTout - delta_ALT;
	dAZ = delta_AZ;
	dALT = delta_ALT;
//	printf("AZprime %f AZout %f dAZ %f \n",AZprime,AZout,delta_AZ);
//	pointing2(AZout,ALTout,&dAZ,&dALT);
	pointing2AZ(AZout,ALTout,&dAZ,&dALT);
	errAZ = (AZout + dAZ) - AZprime;
	pointing2ALT(AZout,ALTout,&dAZ,&dALT);
	errALT = (ALTout + dALT) - ALTprime;
	while((abs(errAZ) >0.004) && count <100){
		if(errAZ <0.){
			AZout = AZout + 0.002;
		}
		else if (errAZ >0.){
			AZout = AZout - 0.002;
		}
		pointing2AZ(AZout,ALTout,&dAZ,&dALT);
		errAZ = (AZout + dAZ) - AZprime;
		if(count >80){
			printf("Error removing Azimuth pointing correction\n");
		}
		count++;
		
	}
	
	count =0;
	while((abs(errALT) >0.004)&& count <100){
		if(errALT <0.){
			ALTout = ALTout + 0.002;
		}
		else if (errAZ >0.){
			ALTout = ALTout - 0.002;
		}
		pointing2ALT(AZout,ALTout,&dAZ,&dALT);
		errALT = (ALTout + dALT) - ALTprime;
		if(count >80){
			printf("Error removing ALTitude pointing correction\n");
		}
		count++;
	}
	
	
	//*AZ= slaDranrm(*AZout);
	//*ALT = slaDranrm(*ALTout);
	AZrad = AZout/R2D;
	ALTrad =ALTout/R2D;
	//AZrad= slaDranrm(AZrad);
	ALTrad = slaDranrm(ALTrad);
	*AZ =AZrad*R2D;
	*ALT =ALTrad*R2D;	


	//*AZ = AZout;
	//*ALT = ALTout;

}


unsigned int azaltprime2azaltshort(double AZprime, double delta_AZ, double ALTprime,double delta_ALT,double *AZ, double *ALT){
	double AZout,ALTout,dAZ,dALT,AZrad,ALTrad;
	double errAZ,errALT;
	unsigned int count =0;
	AZrad =AZprime/R2D;
	ALTrad = ALTprime/R2D;
	AZrad = slaDranrm(AZrad);
	ALTrad = slaDranrm(ALTrad);
	AZprime = AZrad*R2D;
	ALTprime = ALTrad*R2D;
	AZout = AZrad * R2D;
	ALTout = ALTrad*R2D;

	//AZout = AZout - delta_AZ;
	//ALTout = ALTout - delta_ALT;
	//dAZ = delta_AZ;
	//dALT = delta_ALT;
//	printf("AZprime %f AZout %f dAZ %f \n",AZprime,AZout,delta_AZ);
//	pointing2(AZout,ALTout,&dAZ,&dALT);
	pointing2AZ(AZout,ALTout,&dAZ,&dALT);
	delta_AZ = dAZ;
	delta_ALT = dALT;
	AZout = AZout - delta_AZ;
	ALTout = ALTout - delta_ALT;
	errAZ = (AZout + dAZ) - AZprime;
	pointing2ALT(AZout,ALTout,&dAZ,&dALT);
	errALT = (ALTout + dALT) - ALTprime;
	while((abs(errAZ) >0.004) && count <100){
		if(errAZ <0.){
			AZout = AZout + 0.002;
		}
		else if (errAZ >0.){
			AZout = AZout - 0.002;
		}
		pointing2AZ(AZout,ALTout,&dAZ,&dALT);
		errAZ = (AZout + dAZ) - AZprime;
		if(count >80){
			printf("Error removing Azimuth pointing correction\n");
		}
		count++;
		
	}
	
	count =0;
	while((abs(errALT) >0.004)&& count <100){
		if(errALT <0.){
			ALTout = ALTout + 0.002;
		}
		else if (errAZ >0.){
			ALTout = ALTout - 0.002;
		}
		pointing2ALT(AZout,ALTout,&dAZ,&dALT);
		errALT = (ALTout + dALT) - ALTprime;
		if(count >80){
			printf("Error removing ALTitude pointing correction\n");
		}
		count++;
	}
	
	
	//*AZ= slaDranrm(*AZout);
	//*ALT = slaDranrm(*ALTout);
	AZrad = AZout/R2D;
	ALTrad =ALTout/R2D;
	//AZrad= slaDranrm(AZrad);
	ALTrad = slaDranrm(ALTrad);
	*AZ =AZrad*R2D;
	*ALT =ALTrad*R2D;	


	//*AZ = AZout;
	//*ALT = ALTout;

}
