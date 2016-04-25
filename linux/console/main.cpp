// Test program for left tactile dataglove v1
// Outputs all sensor values on console
// Linear, unmodified output
#include <signal.h>
#include <string.h>
#include <boost/thread/thread_time.hpp>
#include <boost/program_options.hpp>
#include <iostream>
#include <memory>
#include <functional>

#if HAVE_CURSES
#include <ncurses.h>  // For text display
#endif
#if HAVE_ROS
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <tactile_msgs/TactileState.h>
#endif

#include "../lib/RandomInput.h"
#include "../lib/SerialInput.h"
#include "../lib/FileInput.h"
#include <tactile_filters/PieceWiseLinearCalib.h>

#define NO_TAXELS         64
#define OUTPUT_CURSES     (1 << 0)
#define OUTPUT_ROS        (1 << 1)

using namespace std;
using namespace tactile;
namespace po=boost::program_options;

void initCurses();  // Setup text terminal display
void printCurses(const tactile::InputInterface::data_vector &data, const PieceWiseLinearCalib *calib);
void publishToROS(const tactile::InputInterface::data_vector &data);
void publishToROS(const tactile::InputInterface::data_vector &data, const PieceWiseLinearCalib *calib);

std::string sTopic;
uint        nErrors=0;

po::options_description options("options");
void usage(char* argv[]) {
	cout << "usage: " << argv[0] << " [options]" << endl
	     << options << endl;
}

bool handleCommandline(uint &outflags,
                       string &device, string &filename,
                       bool &bLoop, float &fSpeedFactor,
                       string &sTopic, string &sCalib,
                       int argc, char *argv[]) {
	// default input device
	device = "/dev/ttyACM0";

	// define processed options
	po::options_description inputs("input options");
	inputs.add_options()
		("serial,s", po::value<string>(&device)->implicit_value(device), "serial input device")
		("file,f", po::value<string>(&filename)->implicit_value(filename), "csv file input filename")
		("dummy,d", "use random dummy input");
	po::options_description outputs("output options");
	outputs.add_options()
#if HAVE_CURSES
		("console,c", "enable console output (default)")
#endif
#if HAVE_ROS
		("ros,r", po::value<string>(&sTopic)->implicit_value("TactileGlove"),
		 "enable ros output to specified topic")
#endif
		;
	options.add_options()
		("help,h", "Display this help message.")
		("calib,c", po::value<string>(&sCalib), "calibration map")
		("loop,l", "Loop at the end of the file")
		("multiplier,m", po::value<float>(&fSpeedFactor), "File read speed multiplier (>1 is faster)");
	options.add(inputs).add(outputs);

	po::variables_map map;
	po::store(po::command_line_parser(argc, argv)
	          .options(options)
	          .run(), map);

	if (map.count("help")) return true;
	po::notify(map);

	if (map.count("serial") + map.count("dummy") + map.count("file") > 1)
		throw std::logic_error("multiple input methods specified");
	if (map.count("dummy") || map.count("file")) device = "";

	outflags = 0;
	if (map.count("console")) outflags |= OUTPUT_CURSES;
	if (map.count("ros")) outflags |= OUTPUT_ROS;
 
	if (map.count("loop")) bLoop = true; else bLoop = false;
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
	std::string sFilename;
	std::string sCalib;
	float fSpeedFactor = 1.0;
	bool bLoop = false;
	std::auto_ptr<tactile::InputInterface> input;
	PieceWiseLinearCalib *calib=0;

	try {
		if (handleCommandline(outflags, sDevice, sFilename, bLoop, fSpeedFactor, sTopic, sCalib, argc, argv)) {
			usage(argv);
			return EXIT_SUCCESS;
		}
		if (sDevice == "") {
			if (sFilename == "") {
				input.reset(new tactile::RandomInput(NO_TAXELS));
				input->connect(sDevice);
			}	
			else {
				input.reset(new tactile::FileInput(NO_TAXELS, fSpeedFactor, bLoop));
				input->connect(sFilename);
			}
		}
		else
		{
			input.reset(new tactile::SerialInput(NO_TAXELS));
			input->connect(sDevice);
		}

		if (!sCalib.empty()) {
			calib = new PieceWiseLinearCalib(PieceWiseLinearCalib::load(sCalib));
		}
	} catch (const exception& e) {
		cerr << e.what() << endl;
		usage(argv);
		return EXIT_FAILURE;
	}

	// initialize ouput
#if HAVE_ROS
	if (outflags & OUTPUT_ROS)
		ros::init (argc, argv, "tactile_glove", ros::init_options::NoSigintHandler);
	ros::Time::init();
	ros::Rate r(1000); // desired rate [Hz]
#endif
	if (outflags & OUTPUT_CURSES) initCurses();

	// register Ctrl-C handler
	signal(SIGINT, mySigIntHandler);

	string sErr;
	unsigned char ch=0;
	while (bRun && ch != 'q') // loop until Ctrl-C
	{
		try {
			const tactile::InputInterface::data_vector &frame = input->readFrame();
			if (outflags & OUTPUT_CURSES) printCurses(frame, calib);
			if (outflags & OUTPUT_ROS) {
				publishToROS(frame);
				if (calib) publishToROS(frame, calib);
			}
		} catch (const std::exception &e) {
			if (bRun) sErr = e.what(); // not Ctrl-C stopped
			break;
		}

		ch = getch();
#if HAVE_ROS
		r.sleep();
#endif
	}

#if HAVE_CURSES
	endwin(); // ncurses cleanup
#endif

	if (!sErr.empty()) {
		cerr << sErr << endl;
		return (EXIT_FAILURE);
	}
	if (!calib) delete calib;
	return EXIT_SUCCESS;
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
//	atexit( (void(*)())endwin ); // Ncurses cleanup function
#endif
}

// Pretty print the data
void printCurses(const tactile::InputInterface::data_vector &data,
                 const PieceWiseLinearCalib *calib)
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
	for(size_t x=0, end=data.size(); x < end; ++x){
		if (getcurx(stdscr) + 10 >= getmaxx(stdscr)) {
			clrtoeol();
			printw("\n");
		}
		if (calib)
			printw("%2d: %.4f    ", x+1, calib->map(data[x]));
		else
			printw("%2d: %4d    ", x+1, data[x]);
	}

	refresh(); 
#endif
}

void publishToROS(const tactile::InputInterface::data_vector &data) {
#if HAVE_ROS
	static bool bInitialized = false;
	static ros::NodeHandle   rosNodeHandle; //< node handle
	static ros::Publisher    rosPublisher;  //< publisher
	static std_msgs::UInt16MultiArray msg;

	if (!bInitialized) {
		rosPublisher = rosNodeHandle.advertise<std_msgs::UInt16MultiArray>(sTopic, 1);
		msg.layout.dim.resize(1);
		msg.layout.dim[0].label = "tactile data";
		msg.layout.dim[0].size  = data.size();
		msg.data.resize(data.size());
		bInitialized = true;
	}

	msg.data = data;
	rosPublisher.publish(msg);
	ros::spinOnce();
#endif
}

void publishToROS(const tactile::InputInterface::data_vector &data,
                  const PieceWiseLinearCalib *calib) {
#if HAVE_ROS
	static bool bInitialized = false;
	static ros::NodeHandle   rosNodeHandle; //< node handle
	static ros::Publisher    rosPublisher;  //< publisher
	static tactile_msgs::TactileState msg;

	if (!bInitialized) {
		rosPublisher = rosNodeHandle.advertise<tactile_msgs::TactileState>(sTopic + "/calibrated", 1);
		::sensor_msgs::ChannelFloat32 channel;
		channel.name = "tactile";
		channel.values.resize(data.size());
		msg.sensors.push_back(channel);
		bInitialized = true;
	}

	std::transform(data.begin(), data.end(), msg.sensors[0].values.begin(),
	               std::bind(&PieceWiseLinearCalib::map, calib, std::placeholders::_1));

	msg.header.stamp = ros::Time::now();
	++msg.header.seq;
	rosPublisher.publish(msg);
	ros::spinOnce();
#endif
}
