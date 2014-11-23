#include "MainWindow.h"

#include <QApplication>
#include <boost/program_options.hpp>
#if HAVE_ROS
#include <ros/ros.h>
#endif

using namespace std;
namespace po=boost::program_options;
po::options_description options("options");

void usage(char* argv[]) {
	cout << "usage: " << argv[0] << " [options]" << endl
	     << options << endl;
}

#define INPUT_SERIAL     1
#define INPUT_ROS        2
#define INPUT_RANDOM     3
bool handleCommandline(uint &inpflags, int argc, char *argv[]) {
	// define processed options
	options.add_options()
		("help,h", "Display this help message.")
		("serial,s", "use serial input")
#if HAVE_ROS
		("ros,r", "use ros input")
#endif
		("dummy,d", "use random dummy input");

	po::variables_map map;
	po::store(po::command_line_parser(argc, argv)
	          .options(options)
	          .run(), map);

	if (map.count("help")) return true;
	po::notify(map);

	if (map.count("ros") && map.count("serial")) {
		cout << "multiple input methods specified" << endl;
		return true;
	}

	inpflags = INPUT_SERIAL;
	if (map.count("dummy")) inpflags = INPUT_RANDOM;
	if (map.count("ros")) inpflags = INPUT_ROS;

	return false;
}

int main(int argc, char *argv[])
{
	uint inpflags;
	try {
		if (handleCommandline(inpflags, argc, argv)) {
			usage(argv);
			return EXIT_SUCCESS;
		}
	} catch (const exception& e) {
		cerr << e.what() << endl;
		usage(argv);
		return EXIT_FAILURE;
	}

#if HAVE_ROS
	ros::init (argc, argv, "tactile_glove_viz");
#endif

	QApplication app(argc, argv);
	MainWindow w;

	switch (inpflags) {
	case INPUT_SERIAL: w.configSerial(); break;
	case INPUT_ROS: w.configROS(); break;
	case INPUT_RANDOM: w.configRandom(); break;
	}

	w.show();
	return app.exec();
}
