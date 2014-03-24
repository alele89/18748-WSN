#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdlib.h>


#define ASCII 1
#define SLIP  0
#define LAST_CONNECTION	0
#define STATIC_CLIENT 1 

#define MAX_SLIP_BUF	1024

// SLIP control sequences
#define ESC	219
#define END	192
#define START	193

void server_tx (uint8_t * buf, uint8_t size);
void print_usage ();
void server_open (int port);
void server_non_blocking_rx (int fd);
void slip_rx (int fd);
void slip_tx (int fd, uint8_t * buf, int size);

struct hostent *hp;
int sock, length, fromlen,replylen, n,reply_sock;
struct sockaddr_in server;
struct sockaddr_in from;
struct sockaddr_in client;
uint8_t buf[1024];
int got_connection;
int debug;
int reply_mode;
char reply_address[80];

//Temporary
FILE *fDataIn;
time_t PCtime;
long int seq=0;

main (int Parm_Count, char *Parms[])
{

  long BAUD, DATABITS, STOPBITS, PARITYON, PARITY;
  char devicename[80];
  char debug_flag[80];
  int fd, tty, slipin, slipout, res, i, error;
  int received = 0, port_num,tmp_pcount;
  uint8_t c;
  FILE *np;
  int mode, val;
  //place for old and new port settings for serial port
  struct termios oldtio, newtio;
  int start_flag;

  fDataIn = fopen("fDataIn.txt","wb");

  got_connection = 0;
  reply_mode = LAST_CONNECTION;

  mode = ASCII;

  BAUD = B115200;
  //BAUD = B2340;
  
  //BAUD = B500000; - this should be the final one
  DATABITS = CS8;
  STOPBITS = 0;
  //STOPBITS = CSTOPB;
  PARITYON = 0;
  PARITY = 0;
  //PARITYON = PARENB;
  //PARITY = PARODD;
  if (Parm_Count < 3 || Parm_Count > 6)
    print_usage ();
  strcpy (buf, Parms[1]);
  i = sscanf (buf, "%s", devicename);
  if (i != 1)
    print_usage ();
  //open the device(com port) to be non-blocking (read will return immediately)
  fd = open (devicename, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    perror (devicename);
    exit (-1);
  }
  strcpy (buf, Parms[2]);
  i = sscanf (buf, "%d", &port_num);
  if (i != 1)
    print_usage ();
  debug = 0;
  tmp_pcount=3;
  //if (Parm_Count > 3) {
  while(tmp_pcount<Parm_Count)
  {
    strcpy (buf, Parms[tmp_pcount]);
    i = sscanf (buf, "%s", debug_flag);
    if (i != 1)
      print_usage ();
    if (strcmp (debug_flag, "-d") == 0)
      debug = 1;
    else if (strcmp (debug_flag, "-s") == 0)
      debug = 2;
    else if (strcmp (debug_flag, "-a") == 0)
	{
	reply_mode=STATIC_CLIENT;
	tmp_pcount++;
	if(tmp_pcount>=Parm_Count) print_usage();
    	strcpy (reply_address, Parms[tmp_pcount]);
	}
    else 
    	print_usage ();
    tmp_pcount++;
  }


  if(debug!=2) printf ("opened: %s\n", devicename);
  tcgetattr (fd, &oldtio);      // save current port settings 
  // set new port settings for canonical input processing 
  newtio.c_cflag =
    BAUD | DATABITS | STOPBITS | PARITYON | PARITY | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;           //ICANON;
  newtio.c_cc[VMIN] = 1;
  newtio.c_cc[VTIME] = 0;
  tcflush (fd, TCIFLUSH);
  tcsetattr (fd, TCSANOW, &newtio);

  server_open (port_num);
  start_flag = 0;
  while (1) {
    server_non_blocking_rx (fd);
    res = read (fd, &c, 1);
    if (res > 0) {

      if (c == START)
        mode = SLIP;
      if (mode == ASCII)
        if(debug!=2) printf ("%c", c);
      if (mode == SLIP) 
		{
			time(&PCtime);
			fprintf(fDataIn,"\n%ld,%ld",PCtime,seq);seq++;
			fflush(fDataIn);
        slip_rx (fd);
        mode = ASCII;
      } 

    } //else usleep(10000);
  }
}                               //end of main

void slip_rx (int fd)
{
  char slip_buf[MAX_SLIP_BUF];
  uint8_t c;
  int res, i, received;
  int mode;

  received = 0;
  mode = SLIP;
  while (mode == SLIP) {
    // get a character to process
    res = read (fd, &c, 1);
    if (res > 0) {
      // handle bytestuffing if necessary
      switch (c) {

        // if it's an END character then we're done with
        // the packet
      case END:
        // a minor optimization: if there is no
        // data in the packet, ignore it. This is
        // meant to avoid bothering IP with all
        // the empty packets generated by the
        // duplicate END characters which are in
        // turn sent to try to detect line noise.
        if (received) {
          uint8_t checksum;
          uint8_t size;

          size = slip_buf[0];
          if (received - 2 != size) {
            if(debug!=2) printf ("\n*** SLIP rx size mismatch %d vs %d\n", received - 2,
                    size);
            mode = ASCII;
            break;
          }
          checksum = 0;
          for (i = 1; i < received - 1; i++) {
            checksum += slip_buf[i];
          }
          checksum &= 0x7F;
          if (checksum != slip_buf[received - 1]) {
            if(debug!=2) printf ("\n*** SLIP rx checksum error %d != %d...\n", checksum,
                    slip_buf[received - 1]);
            mode = ASCII;
            break;
          }
          server_tx (&slip_buf[1], size);
        }

        mode = ASCII;
        break;

        // if it's the same code as an ESC character, wait
        // and get another character and then figure out
        // what to store in the packet based on that.
      case ESC:
        do {
          res = read (fd, &c, 1);
        } while (res < 1);
        // if "c" is not one of these two, then we
        // have a protocol violation.  The best bet
        // seems to be to leave the byte alone and
        // just stuff it into the packet
        switch (c) {
        case END:
          c = END;
          break;
        case ESC:
          c = ESC;
          break;
        }

        // here we fall into the default handler and let
        // it store the character for us
      default:
        if (received < MAX_SLIP_BUF)
          slip_buf[received++] = c;
      }
    }

  }


}

void server_open (int port)
{
  got_connection = 0;
  sock = socket (AF_INET, SOCK_DGRAM, 0);
  // Non-blocking socket
  fcntl (sock, F_SETFL, O_NONBLOCK);
  if (sock < 0)
    perror ("Opening socket");
  length = sizeof (server);
  bzero (&server, length);
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = INADDR_ANY;
  server.sin_port = htons (port);
  if (bind (sock, (struct sockaddr *) &server, length) < 0)
    perror ("binding");
  fromlen = sizeof (struct sockaddr_in);

  replylen = sizeof (struct sockaddr_in);

if(reply_mode==STATIC_CLIENT)
  {
	if(debug==1) printf( "Setting up static client\r\n" );

  	client.sin_family = AF_INET;
  	hp = gethostbyname (reply_address);
  	if (hp == 0)
          {
                perror ("Unknown client host");
                return;
          }

  	bcopy ((char *) hp->h_addr, (char *) &client.sin_addr, hp->h_length);
  	client.sin_port = htons (port);

  }


}

void server_non_blocking_rx (int fd)
{
  int i;

  n = recvfrom (sock, buf, 1024, 0, (struct sockaddr *) &from, &fromlen);

  if (n > 0) {
  if(reply_mode==STATIC_CLIENT)
	{
	if(from.sin_addr.s_addr != client.sin_addr.s_addr)
		{
		printf( "Reject packet\r\n" );
		return;
		}
	}
    bcopy(&from,&client,sizeof(struct sockaddr));
    got_connection = 1;
    if (debug==1) {
      printf ("\nRX: %d [",n);
      for (i = 0; i < n; i++)
        printf ("%x ", buf[i]);
      printf ("]\n");
    }
    slip_tx (fd, buf, n);
  }
}

void slip_tx (int fd, uint8_t * buf, int size)
{
  uint8_t i;
  int8_t v;
  uint8_t checksum;
  int8_t c;

// Make sure size is less than 128 so it doesn't act as a control
// message
  if (size > 128)
    return;

  checksum = 0;
// Send the start byte
  c = START;
  while(write (fd, &c, 1)<0);
  c = size;
  while(write (fd, &c, 1)<0);
// Send payload and stuff bytes as needed
  for (i = 0; i < size; i++) {
    if (buf[i] == END || buf[i] == ESC) {
      c = ESC;
      write (fd, &c, 1);
    }
    c = buf[i];
    while(write (fd, &c, 1)<0);
    checksum += buf[i];
  }

// Make sure checksum is less than 128 so it doesn't act as a control
// message
  checksum &= 0x7f;
  // Send the end byte
  c = checksum;
  while(write (fd, &c, 1)<0);
  c = END;
  while(write (fd, &c, 1)<0);
}

void server_tx (uint8_t * buf, uint8_t size)
{
  int i;

  if (debug==1) {
    printf ("\nTX: %d [",size);
    for (i = 0; i < size; i++) {
      printf ("%x ", buf[i]);
    }
    printf ("]\n");
  }

  if (got_connection != 0) {
      //if(reply_mode==LAST_CONNECTION)
//	n = sendto (sock, buf, size, 0, (struct sockaddr *) &from, fromlen);
	n = sendto (sock, buf, size, 0, (struct sockaddr *) &client, replylen);

      //if(reply_mode==STATIC_CLIENT)
//	n = sendto (reply_sock, buf, size, 0, (struct sockaddr *) &client, replylen);

	if (n < 0)
      		perror ("sendto");
   }




}


void print_usage ()
{
  printf ("Usage: SLIPstream com-port port <-d or -s> <-a client-address>\n");
  printf ("  Ex: SLIPstream /dev/ttyUSB0 4000\n");
  printf ("  This sets up a UDP server on port 4000\n\n");
  printf ("  -d    Turns on debugging that shows SLIP packets and incomming datagrams\n");
  printf ("  -s    Run in silent mode which stops all printing output\n");
  printf ("  -a    Only reply to client with the following address\n");
  exit (-1);

}
