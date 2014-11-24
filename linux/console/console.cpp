// Test program for left tactile dataglove v1
// Outputs all sensor values on console
// Linear, unmodified output
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <boost/thread/thread_time.hpp>
#include <boost/program_options.hpp>

#if HAVE_CURSES
#include <ncurses.h>  // For text display
#endif
#if HAVE_ROS
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#endif

// Serialport settings
#define BAUDRATE          B115200
#define PACKET_SIZE_BYTES 5
#define NO_TAXELS         64
#define OUTPUT_CURSES     (1 << 0)
#define OUTPUT_ROS        (1 << 1)

void initCurses();  // Setup text terminal display
void printCurses(const unsigned short data[NO_TAXELS]);
void publishToROS(const unsigned short data[NO_TAXELS]);

using namespace std;
namespace po=boost::program_options;
std::string sTopic;
uint        nErrors=0;

po::options_description options("options");
void usage(char* argv[]) {
	cout << "usage: " << argv[0] << " [options]" << endl
	     << options << endl;
}

bool handleCommandline(uint &outflags,
                       std::string &device, std::string &sTopic,
                       int argc, char *argv[]) {
	// define processed options
	options.add_options()
		("help,h", "Display this help message.")
		("serial,s", po::value<string>(&device)->default_value("/dev/ttyACM0"), "serial input device")
#if HAVE_CURSES
		("console,c", "enable console output (default)")
#endif
#if HAVE_ROS
		("ros,r", po::value<string>(&sTopic)->implicit_value("TactileGlove"),
		 "enable ros output to specified topic")
#endif
	;

	po::variables_map map;
	po::store(po::command_line_parser(argc, argv)
	          .options(options)
	          .run(), map);

	if (map.count("help")) return true;
	po::notify(map);

	outflags = 0;
	if (map.count("console")) outflags |= OUTPUT_CURSES;
	if (map.count("ros")) outflags |= OUTPUT_ROS;
#if HAVE_CURSES
	if (!outflags) outflags |= OUTPUT_CURSES;
#endif
	if (!outflags) {
		cout << "no output method specified" << endl;
		return true;
	}

	return false;
}

bool bRun=true;
void mySigIntHandler(int sig) {
	bRun = false;
}

int main(int argc, char **argv)
{
	uint outflags;
	std::string sDevice;
	try {
		if (handleCommandline(outflags, sDevice, sTopic, argc, argv)) {
			usage(argv);
			return EXIT_SUCCESS;
		}
	} catch (const exception& e) {
		cerr << e.what() << endl;
		usage(argv);
		return EXIT_FAILURE;
	}

#if HAVE_ROS
	if (outflags & OUTPUT_ROS)
		ros::init (argc, argv, "tactile_glove_server");
#endif
	signal(SIGINT, mySigIntHandler);

	// open and configure serial port
	int fd; // Serial port device handle
	struct termios oldtio,newtio;  // serial port settings

	if ((fd = open(sDevice.c_str(), O_RDONLY | O_NOCTTY )) < 0) {
		perror(sDevice.c_str());
		exit(-1); 
	}

	tcgetattr(fd,&oldtio); // Save current port settings

	bzero(&newtio, sizeof(newtio));
	newtio.c_cc[VTIME]    = 0;   // Inter-character timer unused
	newtio.c_cc[VMIN]     = PACKET_SIZE_BYTES;   // Blocking read until PAKET_SIZE_BYTES chars received

	tcflush(fd, TCIFLUSH);
	tcsetattr(fd,TCSANOW,&newtio); // set new port settings

	if (outflags & OUTPUT_CURSES) initCurses();

	unsigned char buf[PACKET_SIZE_BYTES]; // receive buffer
	int           res;         // stores amount of received bytes
	unsigned char index, ch=0;
	unsigned short data[NO_TAXELS];
	assert(sizeof(data) == sizeof(unsigned short)*NO_TAXELS);
	bzero(data, sizeof(data));

	while (bRun && ch != 'q') // Loop for input
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
			if((buf[0]>=0x3C) && ((index = buf[0] - 0x3C) < NO_TAXELS) &&
			      buf[1] == 0x01 && buf[4] == 0x00)
			{
				// We have a full packet
				unsigned short value = ((0x0F & buf[2])<<8) | buf[3]; // Get pressure value
				data[index] = 4095-value;

				// got full frame
				if(index==NO_TAXELS-1){
					if (outflags & OUTPUT_CURSES) printCurses(data);
					if (outflags & OUTPUT_ROS) publishToROS(data);
				}
			} else ++nErrors;
		}
		ch = getch();
	}
	tcsetattr(fd,TCSANOW,&oldtio); // Restore serial settings
	close(fd);
}

// Init ncurses terminal display
void initCurses()
{
#if HAVE_CURSES
	initscr(); // Ncurses init function
	noecho();
	cbreak();
	timeout(0);
	clear();   // Clear terminal
	atexit( (void(*)())endwin ); // Ncurses cleanup function
#endif
}

// Pretty print the data
void printCurses(const unsigned short data[])
{
#if HAVE_CURSES
	static unsigned int frames=0, fps;
	static boost::system_time tLast = boost::get_system_time();

	frames++;
	boost::system_time tNow = boost::get_system_time();
	if ( (tNow - tLast).total_milliseconds() > 1000 ){
		fps = frames;
		frames = 0;
		tLast = tNow;
	}

	// Print FPS & title
	mvprintw(0, 0, "---> Tactile Dataglove Test <---");
	mvprintw(1, 0, "FPS: %u  errors: %u", fps, nErrors); clrtoeol(); printw("\n");
	clrtobot();

	// Print data
	for(unsigned char x=0;x<NO_TAXELS;x++){
		if (getcurx(stdscr) + 10 >= getmaxx(stdscr)) {
			clrtoeol();
			printw("\n");
		}
		printw("%2d: %4d    ", x+1, data[x]);
	}

	refresh(); 
#endif
}

void publishToROS(const unsigned short data[]) {
#if HAVE_ROS
	static bool bInitialized = false;
	static ros::NodeHandle   rosNodeHandle; //< node handle
	static ros::Publisher    rosPublisher;  //< joint publisher
	static std_msgs::UInt16MultiArray msg;

	if (!bInitialized) {
		rosPublisher = rosNodeHandle.advertise<std_msgs::UInt16MultiArray>(sTopic, 1);
		msg.layout.dim.resize(1);
		msg.layout.dim[0].label = "tactile data";
		msg.layout.dim[0].size  = NO_TAXELS;
		msg.data.resize(NO_TAXELS);
	}

	std::copy(data, data + NO_TAXELS, msg.data.begin());
	rosPublisher.publish(msg);
	ros::spinOnce();
#endif
}
