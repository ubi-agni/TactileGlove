/*

Test program for gereon handschuh
print all sensor values linear

*/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <iostream>


#include <ncurses.h>  //for text display
#include <SDL/SDL.h> //for FPS timecounting

//serialport stuff
#define BAUDRATE B115200
#define MODEMDEVICE "/dev/ttyACM0"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1
#define MAX_BUFF 1024  //max input buffer size in bytes
#define PACKET_SIZE_BYTES 1

volatile int STOP=FALSE;

void print_SensData(); //output sensor data
void initNcurses(); //setup text terminal display

//store our pressure data
unsigned short ldata[54];
unsigned int frames=0; //framecounter
unsigned int fps=0;
unsigned int lastticks=0;

int main(int argc, char **argv)
{
   int fd; //device handle
   int res; //storesamount of received bytes
   struct termios oldtio,newtio;  //for serial port settings
   unsigned char buf[255];	//data recevie buffer
  

//try to open serial port
printf("Please connect Captest\n");

//serial port settings
   if (argc > 1)
      fd = open(argv[1], O_RDWR | O_NOCTTY );
   else
      fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY );
   if (fd <0) {perror(MODEMDEVICE); exit(-1); }

   tcgetattr(fd,&oldtio); /* save current port settings */

   bzero(&newtio, sizeof(newtio));
 
   newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
   newtio.c_cc[VMIN]     = PACKET_SIZE_BYTES;   /* blocking read until 56 chars received */

   tcflush(fd, TCIFLUSH);
   tcsetattr(fd,TCSANOW,&newtio);
//end serial port settings


  initNcurses(); //for output

//init array
	for(unsigned char x=0;x<54;x++){
		 ldata[x]=0;
	}

unsigned char index;

   while (STOP==FALSE) {       /* loop for input */
	
      res = read(fd,buf,5);   // read a maximum of 255 into buf (actual read count is in res)
//parsing:
//we go through the res bytes in buf[] and look for
//character 1 determines sensornumber, after that a byte 1, then follows the sensor value, thenn NULL byte
//3C 01 0F FF 00
//sensor number range from 0x3C - 0x7B

	index=0;
//printf("%d \n",res );
//printf("%x %x %X %X %x \n",buf[0] , buf[1] ,buf[2] ,buf[3], buf[4] );
	if(res==5){  //5 bytes read
//printf("%x %x %X %X %x \n",buf[0] , buf[1] ,buf[2] ,buf[3], buf[4] );
		if((buf[0]>=0x3C) && (buf[0]<=0x7B) )
		
		{
		//we have a full packet
			unsigned short value = ((0x0F & buf[2])<<8) | buf[3]; //get pressure value
			index = buf[0] -0x3C;  //get index
			//printf("%X \n",  buf[0]);
			ldata[index] = 4095-value;

			if(index==53){ 
				frames++; //framecounter

			}
		}


 	   } //end for 

	//packet parsed and data can be printed
	print_SensData();

   }
   tcsetattr(fd,TCSANOW,&oldtio);//restore serial settings
}



//Init ncurses terminal display
void initNcurses(){
	initscr();	//Ncurses init function
	noecho();
	cbreak();
	timeout(0);
	clear();	//clear terminal
	atexit( (void(*)())endwin );	//Ncurses cleanup function
}

//print the Data good looking
void print_SensData(){	

	//clear terminal
	move(0, 0);
	clrtobot();

	//print fps & title
	mvprintw(0, 0, "---> Handschuh Test <---");

//print data
	for(unsigned char x=0;x<54;x++){
		mvprintw(x+2, 0, "%d:", x+1 );
		mvprintw(x+2, 4, "%d", ldata[x] );

	}
	

	refresh(); 

}





