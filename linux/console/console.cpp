// Test program for left tactile dataglove v1
// Outputs all sensor values on console
// Linear, unmodified output
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <ncurses.h>  // For text display
#include <boost/thread/thread_time.hpp>

// Serialport settings
#define BAUDRATE B115200
#define MODEMDEVICE "/dev/ttyACM0"
#define PACKET_SIZE_BYTES 1
#define TAXELNB           64
void print_SensData(); // Output sensor data
void initNcurses(); // Setup text terminal display

// Store our pressure data
unsigned short ldata[154];

unsigned int fps=0;
static unsigned int frames=0;
static boost::system_time tLast = boost::get_system_time();


int main(int argc, char **argv)
{
	int fd; // Serial port device handle
	int res; // Stores amount of received bytes
	struct termios oldtio,newtio;  // For serial port settings
	unsigned char buf[255];        // Data receive buffer
  

	// Serial port settings
	if (argc > 1)
		fd = open(argv[1], O_RDWR | O_NOCTTY );
	else
		fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY );
	if (fd <0) {
		perror(MODEMDEVICE); 
		exit(-1); 
	}

	tcgetattr(fd,&oldtio); // Save current port settings

	bzero(&newtio, sizeof(newtio));
 
	newtio.c_cc[VTIME]    = 0;   // Inter-character timer unused
	newtio.c_cc[VMIN]     = PACKET_SIZE_BYTES;   // Blocking read until PAKET_SIZE_BYTES chars received

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd,TCSANOW,&newtio); // set new port settings

	initNcurses();

	// Init array
	for(unsigned char x=0;x<TAXELNB;x++){
		 ldata[x]=0;
	}

	unsigned char index, ch=0;

	while (ch != 'q') // Loop for input
	{
		res = read(fd,buf,5);   // read a maximum of 5 bytes into buf (actual read count is in res)
/* Parsing:
   we go through the res bytes in buf[] and look for
   * first byte determines taxel number, allowed range is from 0x3C to 0x7B;
   * constant byte 0x01
   * 2 byte sensor value, with first 4 bits as internal AD channel number (ignore)
   * NULL byte (0x00)
   Example: 3C 01 0F FF 00
*/

		index=0;
		if(res==5){  // If read 5 bytes
			if((buf[0]>=0x3C) && (buf[0]<=0x7B))
			{
				// We have a full packet
				unsigned short value = ((0x0F & buf[2])<<8) | buf[3]; // Get pressure value
				index = buf[0] -0x3C;  // Get taxel number
				ldata[index] = 4095-value;

				// got full frame
				if(index==TAXELNB-1){ 
					print_SensData();
				}
			}
		}
		ch = getch();
	}
	tcsetattr(fd,TCSANOW,&oldtio); // Restore serial settings
	close(fd);
}

// Init ncurses terminal display
void initNcurses(){
	initscr(); // Ncurses init function
	noecho();
	cbreak();
	timeout(0);
	clear();   // Clear terminal
	atexit( (void(*)())endwin ); // Ncurses cleanup function
}

// Pretty print the data
void print_SensData(){	
	frames++;
	boost::system_time tNow = boost::get_system_time();
	if ( (tNow - tLast).total_milliseconds() > 1000 ){
		fps = frames;
		frames = 0;
		tLast = tNow;
	}

	// Print FPS & title
	mvprintw(0, 0, "---> Tactile Dataglove Test <---");
	mvprintw(1, 0, "FPS: %u", fps); clrtoeol(); printw("\n");
	clrtobot();

	// Print data
	for(unsigned char x=0;x<TAXELNB;x++){
		if (getcurx(stdscr) + 10 >= getmaxx(stdscr)) {
			clrtoeol();
			printw("\n");
		}
		printw("%2d: %4d    ", x+1, ldata[x]);
	}

	refresh(); 
}
