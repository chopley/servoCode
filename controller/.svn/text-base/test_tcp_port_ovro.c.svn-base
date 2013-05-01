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
int equatorial2_withpoint(int RAh,int RAm, double RAs,int DEC_deg,int DEC_min,double DECs,time_t *timeout,int *lengthout,double *Yretalt,double *Yretaz);
int polynomial_withpoint(double AZStart, double AZEnd,double ALTStart,double ALTEnd,time_t *timeout,int length,double *Yretalt,double *Yretaz);

struct new_message_parsing_struct tcp_message;

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
	char udpbuf[BUFSIZE];
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
	int j,ii;
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
    ii=0;
    while(1){	
  	printf("Number of loops %d \n",ii);
	ii++;
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
	remoteServAddr.sin_port = htons(LOCAL_SERVER_PORT_OVRO);
	
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
	j=0;
	while(j<5){
	    bzero(buf, BUFSIZE);
	    //sprintf(buf, "AEL");
	    //sprintf(buf,"GAE,%d",j);
	    // printf("Count %d\n",j);
	    //sprintf(buf,"STS");
	    sprintf(buf,"EBN");
	    j++;
	    //fgets(buf, BUFSIZE, stdin);




	    /* send the message line to the server */
	    n = send(sd, buf, BUFSIZE,0);
	    if (n < 0) 
	      error("ERROR writing to socket");

	    /* print the server's reply */
	    bzero(udpbuf, BUFSIZE);
	    //printf("About to receive\n");
	    n = recv(sd, udpbuf, BUFSIZE,0);
	    if (n < 0) 
	      error("ERROR reading from socket");
	    
	    printf("Echo from server: %s Size %d\n",udpbuf,n);
	    usleep(1000000);
	}
	j=0;
	while(j<5){
	    bzero(buf, BUFSIZE);
	    //sprintf(buf, "AEL");
	    //sprintf(buf,"GAE,%d",j);
	    // printf("Count %d\n",j);
	    sprintf(buf,"STS");
	    //sprintf(buf,"EBN");
	    j++;
	    //fgets(buf, BUFSIZE, stdin);
	    /* send the message line to the server */
	    n = send(sd, buf, BUFSIZE,0);
	    if (n < 0) 
	      error("ERROR writing to socket");
	    /* print the server's reply */
	    bzero(udpbuf, BUFSIZE);
	    //printf("About to receive\n");
	    n = recv(sd, udpbuf, BUFSIZE,0);
	    if (n < 0) 
	      error("ERROR reading from socket");
	    
	    printf("Echo from server: %s Size %d\n",udpbuf,n);
	    usleep(1000000);
	}
	j=0;
	while(j<5){
	    bzero(buf, BUFSIZE);
	    //sprintf(buf, "AEL");
	    //sprintf(buf,"GAE,%d",j);
	    // printf("Count %d\n",j);
	    //sprintf(buf,"STS");
	    sprintf(buf,"EBF");
	    j++;
	    //fgets(buf, BUFSIZE, stdin);




	    /* send the message line to the server */
	    n = send(sd, buf, BUFSIZE,0);
	    if (n < 0) 
	      error("ERROR writing to socket");

	    /* print the server's reply */
	    bzero(udpbuf, BUFSIZE);
	    //printf("About to receive\n");
	    n = recv(sd, udpbuf, BUFSIZE,0);
	    if (n < 0) 
	      error("ERROR reading from socket");
	    
	    printf("Echo from server: %s Size %d\n",udpbuf,n);
	    usleep(1000000);
	}
	j=0;
	while(j<5){
	    bzero(buf, BUFSIZE);
	    //sprintf(buf, "AEL");
	    //sprintf(buf,"GAE,%d",j);
	    // printf("Count %d\n",j);
	    sprintf(buf,"STS");
	    //sprintf(buf,"EBN");
	    j++;
	    //fgets(buf, BUFSIZE, stdin);
	    /* send the message line to the server */
	    n = send(sd, buf, BUFSIZE,0);
	    if (n < 0) 
	      error("ERROR writing to socket");
	    /* print the server's reply */
	    bzero(udpbuf, BUFSIZE);
	    //printf("About to receive\n");
	    n = recv(sd, udpbuf, BUFSIZE,0);
	    if (n < 0) 
	      error("ERROR reading from socket");
	    
	    printf("Echo from server: %s Size %d\n",udpbuf,n);
	    usleep(1000000);
	}
    //sprintf(buf,"AEL,%d",j);
    	usleep(100000);
	//printf("Count %d\n",j);
    	printf("Echo from server: %s \n",udpbuf);
	
    //sprintf(buf,"AEL,%d",j);
    	usleep(100000);
	//printf("Count %d\n",j);
    	printf("Echo from server: %s \n",udpbuf);
	printf("Closing connection to '%s' (IP : %s) : \n Count is %d \n", argv[0], h->h_name,inet_ntoa(*(struct in_addr *)h->h_addr_list[0]),j);
    	usleep(100000);
	close(sd);
    
    }
	
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
	printf("Here\n");
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

	printf("Here\n");
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
	
	printf("Here3\n");
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
	printf("Here\n");
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


int equatorial2_withpoint(int RAh,int RAm, double RAs,int DEC_deg,int DEC_min,double DECs,time_t *timeout,int *lengthout,double *Yretalt,double *Yretaz ){
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
	int length=100;
	int length2;

	
	struct timezone tzp;
	time_t *timev;
	time_t *fittimev;
	double *timevd;
	struct timeval time_struct;
	struct tm *time_ptr;
	time_t current_time;
	
	length2=length+10;
	printf("Here\n");
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

	printf("Here\n");
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
	
	printf("Here3\n");
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
	printf("Here\n");
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
	}
	quad(timevd,aob1,Yretaz,length);
	quad(timevd,elob,Yretalt,length);

	for(i=0;i<length-10;i++){
		ax=*(timevd+i);
		a_az = *(Yretaz) + ax*(*(Yretaz+1)) + ax*(*(Yretaz+2));
		//ay = *(Yret) + ax*(*(Yret+1)) + ax*(*(Yret+2) + ax*(*(Yret+3)))
		a_alt = *(Yretalt) + ax*(*(Yretalt+1)) + ax*(*(Yretalt+2));
		printf("i = %d time %+16.9f az %+16.9f %+16.9f %+16.9f alt %+16.9f %+16.9f %+16.9f \n",i,*(timevd+i),*(aob1+i),a_az,*(aob1+i)-a_az,*(elob+i),a_alt,*(elob+i)-a_alt);	
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
		printf("%f \n",Y[i]);
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
	printf("Here\n");
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


