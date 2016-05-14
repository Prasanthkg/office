/*
 ============================================================================
 Name        : Nibp_amigo.c
 Author      : Ajmal
 Version     :
 Copyright   : Your copyright notice
 Description : SUPER NIBP 300A Interface with the mother board.
 ============================================================================
 */

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


/******************************************************************************************************************************************/
/******************************************************************************************************************************************/

#define true 1
#define false 0

/******************************************************************************************************************************************/
/******************************************************************************************************************************************/
typedef int booll;
int Nibpfd,TtyFd;
int ByteBits = CS8;
int DeviceSpeed = B115200;
struct termios TermAttr, BackupTermAttr, TtyEcgAttr;
struct termios spo2term;
struct sigaction saio, sbio;
sigval_t dddd;
unsigned char Char = 'g';
struct termios TtyAttr;
struct termios BackupTtyAttr;
	 static int EscKeyCountmine = 0;


	 unsigned char new, nibp_buf, res_nibp;
	 //unsigned char nibp_data[24]={0};
	 //unsigned char amigo_nibp_start=0,nibp_terminate=0,nibp_reset=0;
	 //unsigned char amigo_nibp_fill=0;
	 //unsigned char amigo_nibp_read[50]={0};
	 //static int buf_size=0,amigo_buf_elt=0;
        char f_nibp=0;
	/* char nibp_mes_start=0,nibp_mes_fin=0,nibp_busy=0,nibp_data_ret=0,nibp_data_ret_fin=0;
		static int nibp_val_cnt=0,cuff_meassure=0,cuff_command=0;
		static int initial_inflate=0,inflate_ok=0,nibp_command=0,nibp_command_send=0;
		static unsigned int sys_lsb=0, sys_msb=0, dia_lsb=0, dia_msb=0, hr_lsb=0, hr_msb=0, mean_lsb=0, mean_msb=0;
		static unsigned int sys_bp=0, dia_bp=0, mn_bp=0, bp_status=0, nipls_rate=0, Err_code=0, cuff_pressure=0;

		static unsigned int amigo_cuff_pressure=0,amigo_cuff_err=0,amigo_opr_type=0;
		static unsigned int amigo_sys_pr=0,amigo_dia_pr=0, amigo_mean_pr=0,amigo_time=0,amigo_pls_rate=0;
		static unsigned char amigo_nibp_messtatus=0, amigo_nibp_sysstatus=0,amigo_patmode=0,amigo_mesmode=0;
		static unsigned char amigo_err=0,st_cmd[10]={0},amigo_nibp_stat=0;*/

	 /***************************/
	 //int second_for_bp=0,minute_for_bp=0;
	 //char ecghchr[5]={0},repeat_time_bp[5]={0};


	 /*Sub Function declarations*/
static void ecgsignal_handler_IO (int sig, siginfo_t *siginfo, void *context);
//void amigo_nibp_res();
//void amigo_nibp_cmd();
/* *************************************************************************************************************************************** *
 * *************************************************************************************************************************************** *
 * *************************************************************************************************************************************** */
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


/*=============================ECG TTy Setup==================================================================================*/


	                 Nibpfd = open("/dev/ttyS3", O_RDWR | O_NOCTTY | O_NDELAY,0);
	                   if (Nibpfd == -1)
	                   {
	                      perror("open_port: Unable to open SAC2\n");
	                      exit(1);
	                   }

	                   saio.sa_sigaction = &ecgsignal_handler_IO;
	                   saio.sa_flags = SA_SIGINFO;//=0
	                   saio.sa_restorer = NULL;
	                   sigaction(SIGIO,&saio,NULL);

	                   tcflush(Nibpfd,TCIFLUSH); // Flush input buffer
	                   tcflush(Nibpfd,TCOFLUSH);	// Flush output buffer

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

	                   fcntl(Nibpfd, F_SETFL, FNDELAY);
	                   fcntl(Nibpfd, F_SETOWN, getpid());
	                   fcntl(Nibpfd, F_SETFL,  O_ASYNC ); /**<<<<<<------This line made it work.**/

                       printf("\nUART1 configured....\n");



/*===================================================================================*/


/*---------------------------------------fbdata-----------------------------------------------------------------*/
     	 //	 x = 1;  y = 0x226;
	 while(1)
     {
     read (TtyFd, &Char, 1);
     printf("data:%c",Char);

	    //if (Char == '\x1b')
        if (Char == 'a')
	    {
        	EscKeyCountmine ++;
	        if (EscKeyCountmine >= 1)
	            {
	            	Char = 0x00;
                    	printf("\n HAI");
	            	goto ExitLabel;
	            	EscKeyCountmine=0;
	            }
	    }
	    else
	    {
		continue;
	    }

     }

     ExitLabel:
	 close(Nibpfd);
     if (tcsetattr(TtyFd, TCSANOW, &BackupTermAttr) < 0)
    	printf("Unable to set tty");
     return 0;
}


/******************************************************************************************************************************************/
/******************************************************************************************************************************************/
/******************************************************************************************************************************************/

static void ecgsignal_handler_IO (int sig, siginfo_t *siginfo, void *context)
{
		static int amigo_nibp_bufcnt;
		ioctl(Nibpfd,TIOCINQ,&nibp_buf);
	    	f_nibp=nibp_buf;
	    	printf("Hello Amigo");
	        for(amigo_nibp_bufcnt=0;amigo_nibp_bufcnt<f_nibp;amigo_nibp_bufcnt++)
	        {
	       	res_nibp = read(Nibpfd,&nibp_buf,1);
	        }
		//amigo_nibp_fill=1;

}


