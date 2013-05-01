/* 
 * tcpserver.c - A simple TCP echo server 
 * usage: tcpserver <port>
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <errno.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "pid.h"

struct pid_structure tcp_control_structs;




#define BUFSIZE 4000

#if 0
/* 
 * Structs exported from in.h
 */

/* Internet address */
struct in_addr {
  unsigned int s_addr; 
};

/* Internet style socket address */
struct sockaddr_in  {
  unsigned short int sin_family; /* Address family */
  unsigned short int sin_port;   /* Port number */
  struct in_addr sin_addr;	 /* IP address */
  unsigned char sin_zero[...];   /* Pad to size of 'struct sockaddr' */
};

/*
 * Struct exported from netdb.h
 */

/* Domain name service (DNS) host entry */
struct hostent {
  char    *h_name;        /* official name of host */
  char    **h_aliases;    /* alias list */
  int     h_addrtype;     /* host address type */
  int     h_length;       /* length of address */
  char    **h_addr_list;  /* list of addresses */
}
#endif

/*
 * error - wrapper for perror
 */
void error(char *msg) {
  perror(msg);
  exit(1);
}

int main(int argc, char **argv) {
 char *string = NULL;
    int fdpipe, ret_valpipe, countpipe, numread;

	char *temp=NULL;
	double az_pointing_removed,alt_pointing_removed;
	double az_pointing_removed_command,alt_pointing_removed_command;
	char *stringin=NULL;
	char *strings[BUFSIZE];
	char delims[] = ",";
	unsigned int i,length;
  int parentfd; /* parent socket */
  int childfd; /* child socket */
  int portno; /* port to listen on */
  int clientlen; /* byte size of client's address */
  struct sockaddr_in serveraddr; /* server's addr */
  struct sockaddr_in clientaddr; /* client addr */
  struct hostent *hostp; /* client host info */
  char buf[BUFSIZE]; /* message buffer */
  char *hostaddrp; /* dotted decimal host addr string */
  int optval; /* flag value for setsockopt */
  int n; /* message byte size */
	int FLAGS;
  /* 
   * check command line arguments 
   */
  if (argc != 2) {
    fprintf(stderr, "usage: %s <port>\n", argv[0]);
    exit(1);
  }
  portno = atoi(argv[1]);

  ret_valpipe = mkfifo("cbass", 0666);

    if ((ret_valpipe == -1) && (errno != EEXIST)) {
        perror("Error creating the named pipe");
        exit (1);
    }
     /* Open the pipe for reading */
   

  

  /* 
   * socket: create the parent socket 
   */
  parentfd = socket(AF_INET, SOCK_STREAM, 0);
  if (parentfd < 0) 
    error("ERROR opening socket");

  /* setsockopt: Handy debugging trick that lets 
   * us rerun the server immediately after we kill it; 
   * otherwise we have to wait about 20 secs. 
   * Eliminates "ERROR on binding: Address already in use" error. 
   */
  optval = 1;
  setsockopt(parentfd, SOL_SOCKET, SO_REUSEADDR, 
	     (const void *)&optval , sizeof(int));

  /*
   * build the server's Internet address
   */
  bzero((char *) &serveraddr, sizeof(serveraddr));

  /* this is an Internet address */
  serveraddr.sin_family = AF_INET;

  /* let the system figure out our IP address */
  serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);

  /* this is the port we will listen on */
  serveraddr.sin_port = htons((unsigned short)portno);

  /* 
   * bind: associate the parent socket with a port 
   */
  if (bind(parentfd, (struct sockaddr *) &serveraddr, 
	   sizeof(serveraddr)) < 0) 
    error("ERROR on binding");

  /* 
   * listen: make this socket ready to accept connection requests 
   */
	while(1){
  if (listen(parentfd, 5) < 0) /* allow 5 requests to queue up */ 
    error("ERROR on listen");

  /* 
   * main loop: wait for a connection request, echo input line, 
   * then close connection.
   */
  clientlen = sizeof(clientaddr);
 

    /* 
     * accept: wait for a connection request 
     */
    childfd = accept(parentfd, (struct sockaddr *) &clientaddr, &clientlen);
   printf("Here we are\n");
   if (childfd < 0) 
      error("ERROR on accept");
    
    /* 
     * gethostbyaddr: determine who sent the message 
     */
    hostp = gethostbyaddr((const char *)&clientaddr.sin_addr.s_addr, 
			  sizeof(clientaddr.sin_addr.s_addr), AF_INET);
  printf("Here we are2\n");
    if (hostp == NULL)
      error("ERROR on gethostbyaddr");
    hostaddrp = inet_ntoa(clientaddr.sin_addr);
    if (hostaddrp == NULL)
      error("ERROR on inet_ntoa\n");
    printf("server established connection with %s (%s)\n", 
	   hostp->h_name, hostaddrp);
    printf("Here we are 3\n");
    /* 
     * read: read input string from the client
     */
     // fdpipe = open("cbass", O_WRONLY);
	 while (1) {
    bzero(buf, BUFSIZE);
	
    n = recv(childfd, &tcp_control_structs,sizeof(tcp_control_structs),0);
	//printf("%s\n",buf);
	i=0;
	//printf("start");
// 	string = &buf[0];
// 	string = "";
// 	
// 	string = strtok( buf, delims );
// 	strings[i]="";
// 	strings[i] = string;
	//printf(cnt,"CONTROL_STATUS,%10.3f,%10.3f,%8u,%8u,%8.3f,%8.3f,%6u,%6u,%2i,%6i,%6i,%6i,%6i,%6i,%6i,%6i,%6i,%6i, b %10lu,b %10lu,%6i,%6i,%6i,%6i,%6i,r %8ld,s %8ld,t %7ld,u %7ld,v %5u,w %5ld,counter %5ld ",loop.azimuth_encoder_double,loop.azimuth_command_double,user.az_encoder_long,loop.az_command_long,loop.altitude_encoder_double,loop.altitude_command_double,user.alt_encoder_long,loop.alt_command_long,user.azimuth_zone,test_dac,user.tacho1,user.tacho2,user.tacho3,user.tacho4,pid_return_new[0],pid_return_new[1],pid_return_new[2],pid_return_new[3],loop.alt1_dac_control,loop.alt2_dac_control,user.encoder_error,err_count_alt,err_count_az,loop.az_command_long,loop.alt_command_long,user.time_struct.tv_sec,user.time_struct.tv_usec,user.time,time_diff,user.counter3,user.counter2);
	printf("tcp_control_structs %10.3f %10.3f %d\n",tcp_control_structs.azimuth_encoder_double,tcp_control_structs.azimuth_command_double,tcp_control_structs.alt_encoder_long);
	printf("\n");
	//printf("end\n\n %s %s %s %s end of split \n\n",strings[0],strings[1],strings[2],strings[3]);

    if (n <=0) 
      error("ERROR reading from socket");

    //printf("%s",buf);
    
    /* 
     * write: echo the input string back to the client 
     */
//    n = write(childfd, buf, strlen(buf));
 //   if (n < 0) 
  //    error("ERROR writing to socket");

  //  close(childfd);
  }
	}
}
