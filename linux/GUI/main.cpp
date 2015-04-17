#include "MainWindow.h"

#include <iostream>
#include <fstream>
#include <QApplication>
#include <boost/program_options.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <signal.h>
#if HAVE_ROS
#include <ros/ros.h>
#endif

using namespace std;
namespace po=boost::program_options;
po::options_description cmd; // cmdline options

void usage(char* argv[]) {
	cout << "usage: " << argv[0] << " [options]" << endl
	     << cmd << endl;
}

#define INPUT_SERIAL     1
#define INPUT_ROS        2
#define INPUT_RANDOM     3

bool handleCommandline(uint &inputMethod, std::string &sInput,
                       std::string &sLayout, TaxelMapping &mapping,
                       int argc, char *argv[]) {
	// default input method
	inputMethod = INPUT_SERIAL;
	sInput = "/dev/ttyACM0";
	std::string sConfigFile;

	po::options_description options("options");
	po::options_description config; // config options
	config.add_options()
		("layout,l", po::value<string>(&sLayout)->default_value("P2"),
		 "glove layout")
		("serial,s", po::value<string>(&sInput)->implicit_value(sInput),
		 "use serial input (default)")
#if HAVE_ROS
		("ros,r", po::value<string>(&sInput)->implicit_value("TactileGlove"),
		 "use ros input")
#endif
		("dummy,d", "use random dummy input");

	po::options_description hidden; // hidden positional options
	hidden.add_options()
		("mapping", po::value<vector<string> >()->composing(), "taxel mappings");

	cmd.add_options()
		("help,h", "Display this help message.")
		("config,c", po::value<string>(&sConfigFile), "config file");
	cmd.add(config);

	po::positional_options_description pos;
	pos.add("mapping", -1);

	po::variables_map map;
	po::store(po::command_line_parser(argc, argv)
	          .options(po::options_description().add(cmd).add(hidden))
	          .positional(pos)
	          .run(), map);
	po::notify(map); // assigns variables

	if (map.count("help")) return true;

	TaxelMapping configFileMapping;
	if (map.count("config")) {
		ifstream ifs(sConfigFile.c_str());
		if (!ifs) throw std::runtime_error("cannot open config file " + sConfigFile);

		po::parsed_options parsed = po::parse_config_file(ifs, config, true);
		po::store(parsed, map);
		po::notify(map);
		configFileMapping.merge(TaxelMapping(parsed.options), sLayout);
	}

	if (map.count("ros") + map.count("serial") + map.count("dummy") > 1)
		throw std::logic_error("multiple input methods specified");

	// merge taxel mapping options
	mapping = TaxelMapping(sLayout);
	mapping.merge(configFileMapping);
	if (map.count("mapping")) {
		TaxelMapping cmdlineMapping;
		boost::regex r("([._A-Za-z0-9]*)=(\\d+)");
		boost::smatch match;
		BOOST_FOREACH(const std::string &s, map["mapping"].as< vector<string> >()) {
			if (boost::regex_match(s, match, r))
				cmdlineMapping[match[1]] = boost::lexical_cast<TaxelMapping::mapped_type>(match[2]);
			else throw std::runtime_error("invalid taxel mapping: " + s);
		}
		mapping.merge(cmdlineMapping);
	}

	if (map.count("serial")) inputMethod = INPUT_SERIAL;
	if (map.count("dummy")) inputMethod = INPUT_RANDOM;
	if (map.count("ros")) inputMethod = INPUT_ROS;

	return false;
}

QApplication *pApp;
void mySigIntHandler(int sig) {
	if (pApp) pApp->quit();
}

int main(int argc, char *argv[])
{
	uint inputMethod;
	std::string sInput, sLayout;
	TaxelMapping mapping;
	try {
		if (handleCommandline(inputMethod, sInput, sLayout, mapping, argc, argv)) {
			usage(argv);
			return EXIT_SUCCESS;
		}
	} catch (const exception& e) {
		cerr << e.what() << endl;
		usage(argv);
		return EXIT_FAILURE;
	}

	QApplication app(argc, argv);
	MainWindow w(64);
	try {
		w.initGloveWidget(QString::fromStdString(sLayout), mapping);
	} catch (const std::exception &e) {
		cerr << e.what() << endl;
		return EXIT_FAILURE;
	}

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
