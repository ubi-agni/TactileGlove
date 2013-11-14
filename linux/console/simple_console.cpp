// Test program for left tactile dataglove v1
// Outputs all sensor values on console
// Linear, unmodified output
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ncurses.h>  // For text display
#include <SDL/SDL.h> // For FPS timecounting

// Serialport settings
#define BAUDRATE B115200
#define MODEMDEVICE "/dev/ttyACM0"
#define _POSIX_SOURCE 1 // POSIX compliant source
#define FALSE 0
#define TRUE 1
#define MAX_BUFF 1024  // Max input buffer size in bytes
#define PACKET_SIZE_BYTES 1

void print_SensData(); // Output sensor data
void initNcurses(); // Setup text terminal display

// Store our pressure data
unsigned short ldata[154];

	unsigned int fps=0;
	static unsigned int frames=0;
	static unsigned long int lastticks=0;
	static unsigned long int lastticks_frame=0;


int main(int argc, char **argv)
{
   int fd; // Serial port device handle
   int res; // Stores amount of received bytes
   struct termios oldtio,newtio;  // For serial port settings
   unsigned char buf[255];	// Data receive buffer
  

// Try to open serial port
printf("Please connect dataglove via USB!\n");

// Serial port settings
   if (argc > 1)
      fd = open(argv[1], O_RDWR | O_NOCTTY );
   else
      fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY );
   if (fd <0) {perror(MODEMDEVICE); exit(-1); }

   tcgetattr(fd,&oldtio); // Save current port settings

   bzero(&newtio, sizeof(newtio));
 
   newtio.c_cc[VTIME]    = 0;   // Inter-character timer unused
   newtio.c_cc[VMIN]     = PACKET_SIZE_BYTES;   // Blocking read until PAKET_SIZE_BYTES chars received

   tcflush(fd, TCIFLUSH);
   tcsetattr(fd,TCSANOW,&newtio);
// End serial port settings


 // initNcurses(); // For output

// Init array
	for(unsigned char x=0;x<54;x++){
		 ldata[x]=0;
	}

unsigned char index;

   while (true)		  // Loop for input
   {
      res = read(fd,buf,5);   // Eead a maximum of 5 bytes into buf (actual read count is in res)
// Parsing:
// we go through the res bytes in buf[] and look for
// * first byte determines taxel number, allowed range is from 0x3C to 0x7B;
// * constant byte 0x01
// * 2 byte sensor value, with first 4 bits as internal AD channel number (ignore)
// * NULL byte (0x00)
// Example: 3C 01 0F FF 00

	index=0;
// printf("%d \n",res );
// printf("%x %x %X %X %x \n",buf[0] , buf[1] ,buf[2] ,buf[3], buf[4] );
	if(res==5){  // If read 5 bytes
// printf("%x %x %X %X %x \n",buf[0] , buf[1] ,buf[2] ,buf[3], buf[4] );
		if((buf[0]>=0x3C) && (buf[0]<=0x7B))
		{
		// We have a full packet
			unsigned short value = ((0x0F & buf[2])<<8) | buf[3]; // Get pressure value
			index = buf[0] -0x3C;  // Get taxel number
			//printf("%X \n",  buf[0]);
			ldata[index] = 4095-value;

			if(index==53){ 
			//	frames++; // FPS counter
			}
		}
			print_SensData();
 	   }

	// Packet parsed and data can be printed

   }
   tcsetattr(fd,TCSANOW,&oldtio); // Restore serial settings
}

// Init ncurses terminal display
void initNcurses(){
	initscr();	// Ncurses init function
	noecho();
	cbreak();
	timeout(0);
	clear();	// Clear terminal
	atexit( (void(*)())endwin ); // Ncurses cleanup function
}

// Pretty print the data
void print_SensData(){	



		frames++;
		if ( (SDL_GetTicks() - lastticks) > 1000 ){		//1 Tick equals 1 ms
			fps = frames/64;
			frames = 0;
			lastticks = SDL_GetTicks();
			printf(" %d \n",fps);
//mvprintw(4, 0, "Receive Time Difference: (ms)");
		}



	// Clear terminal
	//move(0, 0);
	//clrtobot();

	// Print FPS & title
	//mvprintw(0, 0, "---> Tactile Dataglove Test <---");
	//mvprintw(1, 0, "FPS: %u", fps);
//mvprintw(2, 0, "Time: %d", SDL_GetTicks()); 

// Print data
	for(unsigned char x=0;x<54;x++){
	//	mvprintw(x+2, 0, "%d:", x+1 );
	//	mvprintw(x+2, 4, "%d", ldata[x] );
	}

//mvprintw(2, 0, "Receive Time Difference: %d (ms)",SDL_GetTicks() -lastticks_frame );

	lastticks_frame=SDL_GetTicks();
	refresh(); 
}
