#include <stdio.h>
#include<stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
int main(void)
{

//const char *device = "/dev/ttyAMA0";
//int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
int fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY,0);
if(fd == -1) {
  printf( "failed to open port\n" );
}

struct termios serial;
    serial.c_iflag = 0;
    serial.c_oflag = 0;
    serial.c_lflag = 0;
    serial.c_cflag = 0;

    serial.c_cc[VMIN] = 0;
    serial.c_cc[VTIME] = 0;

    serial.c_cflag = B115200 | CS8 | CREAD;
//while(1)
//{
    write(fd,"Prasanth", 10);
//}
/*
int wcount = write(fd,"Prasanth", 10);
if (wcount < 0)
{
perror("Write");
return -1;
}
else
{
printf("Sent %d characters\n", wcount);
}
/*char buffer[30];
int rcount = read(fd, buffer, sizeof(buffer));
if (rcount < 0) {
perror("Read");
return -1;
}
else
{
printf("Received %d characters\n", rcount);
}
printf("%c\n",buffer[0]);
*/

close(fd);
}
