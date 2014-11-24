#include "MainWindow.h"

#include <QApplication>
#include <boost/program_options.hpp>
#include <signal.h>
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
bool handleCommandline(uint &inputMethod, std::string &sInput, int argc, char *argv[]) {
	// define processed options
	options.add_options()
		("help,h", "Display this help message.")
		("serial,s", po::value<string>(&sInput)->implicit_value("/dev/ttyACM0"),
		 "use serial input (default)")
#if HAVE_ROS
		("ros,r", po::value<string>(&sInput)->implicit_value("TactileGlove"),
		 "use ros input")
#endif
		("dummy,d", "use random dummy input");

	po::variables_map map;
	po::store(po::command_line_parser(argc, argv)
	          .options(options)
	          .run(), map);

	if (map.count("help")) return true;

	if (map.count("ros") && map.count("serial") && map.count("dummy")) {
		cout << "multiple input methods specified" << endl;
		return true;
	}

	po::notify(map);
	inputMethod = 0;
	if (map.count("serial")) inputMethod = INPUT_SERIAL;
	if (map.count("dummy")) inputMethod = INPUT_RANDOM;
	if (map.count("ros")) inputMethod = INPUT_ROS;
	if (!inputMethod) { // default
		inputMethod = INPUT_SERIAL;
		sInput = "/dev/ttyACM0";
	}

	return false;
}

QApplication *pApp;
void mySigIntHandler(int sig) {
	if (pApp) pApp->quit();
}

int main(int argc, char *argv[])
{
	uint inputMethod;
	std::string sInput;
	try {
		if (handleCommandline(inputMethod, sInput, argc, argv)) {
			usage(argv);
			return EXIT_SUCCESS;
		}
	} catch (const exception& e) {
		cerr << e.what() << endl;
		usage(argv);
		return EXIT_FAILURE;
	}

	QApplication app(argc, argv);
	MainWindow w;

	QString qsInput = QString::fromStdString(sInput);
	switch (inputMethod) {
	case INPUT_SERIAL: w.configSerial(qsInput); break;
#if HAVE_ROS
	case INPUT_ROS:
		ros::init (argc, argv, "tactile_glove_viz");
		if (!ros::master::check()) {
			cerr << "Failed to contact ROS master." << endl;
			return EXIT_FAILURE;
		}
		w.configROS(qsInput);
		break;
#endif
	case INPUT_RANDOM: w.configRandom(); break;
	}

	// install signal handler
	pApp = &app; signal(SIGINT, mySigIntHandler);

	w.show();
	return app.exec();
}
