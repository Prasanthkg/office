#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <errno.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include<math.h>
#include <time.h>
int Nibpfd,TtyFd;
int ByteBits = CS8;
int DeviceSpeed = B115200;
struct termios TermAttr, BackupTermAttr, TtyEcgAttr;
struct termios spo2term;
struct sigaction saio, sbio;
int main(int argc, char *argv[])
{

/*=============================Terminal Settings =============================================================================*/
	              TtyFd = open("/dev/tty", O_RDWR |O_NDELAY, 0);///done here
	              if (TtyFd < 0)
	              printf("Unable to open tty");


	                   memset(&TermAttr, 0, sizeof(struct termios));
	                       TermAttr.c_iflag = IGNPAR;
	                       TermAttr.c_cflag = DeviceSpeed | HUPCL | ByteBits | CREAD | CLOCAL;
	                       TermAttr.c_cc[VMIN] = 1;

	              //Get the attributes of the terminal
	              if (tcgetattr(TtyFd, &BackupTermAttr) < 0)
	              printf("Unable to get tty");

	              //set the above set attributes to terminal
	              if (tcsetattr(TtyFd, TCSANOW, &TermAttr) < 0)
	              printf("Unable to set tty");
	              else
	              printf("tty configured successfully");
                      printf("\n first stage complete");
                     // write(TtyFd,"Prasanth", 10);
                      //close(TtyFd);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


			Nibpfd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY,0);
	                   if (Nibpfd == -1)
	                   {
	                      perror("open_port: Unable to open SAC2\n");
	                      exit(1);
	                   }

	                   /*saio.sa_sigaction = &ecgsignal_handler_IO;
	                   saio.sa_flags = SA_SIGINFO;//=0
	                   saio.sa_restorer = NULL;
	                   sigaction(SIGIO,&saio,NULL);

	                   tcflush(Nibpfd,TCIFLUSH); // Flush input buffer
	                   tcflush(Nibpfd,TCOFLUSH);	// Flush output buffer*/

	                   /*==================ECG Tty Attributes ============*/
	                   //baudRate = B115200;          /* Not needed */
	                   cfsetispeed(&TtyEcgAttr,B4800);
	                   cfsetospeed(&TtyEcgAttr,B4800);
	                   TtyEcgAttr.c_cflag &= ~PARENB;
	                   TtyEcgAttr.c_cflag &= ~CSTOPB;
	                   TtyEcgAttr.c_cflag &= ~CSIZE;
	                   TtyEcgAttr.c_cflag |= CS8;
	                   TtyEcgAttr.c_cflag |= (CLOCAL | CREAD);
	                   TtyEcgAttr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	                   TtyEcgAttr.c_iflag &= ~(IXON | IXOFF | IXANY);
	                   TtyEcgAttr.c_oflag &= ~OPOST;
	                   tcsetattr(Nibpfd,TCSANOW,&TtyEcgAttr);

	                  // fcntl(Nibpfd, F_SETFL, FNDELAY);
	                   //fcntl(Nibpfd, F_SETOWN, getpid());
	                   //fcntl(Nibpfd, F_SETFL,  O_ASYNC ); /**<<<<<<------This line made it work.**/
while(1){
                  //printf("\nUART1 configured....\n");
write(Nibpfd,"Prasanth", 10);
}
close(Nibpfd);
}
