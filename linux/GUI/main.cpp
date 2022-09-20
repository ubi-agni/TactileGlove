#include "MainWindow.h"
#include "TaxelMappingTools.h"

#include <QApplication>
#include <fstream>
#include <boost/regex.hpp>
#include <boost/foreach.hpp>
#include <signal.h>
#if HAVE_ROS
#include <ros/ros.h>
#endif

using namespace std;
namespace po = boost::program_options;
po::options_description cmd;  // cmdline options

void usage(char *argv[])
{
	cout << "usage: " << argv[0] << " [options]" << endl << cmd << endl;
}

#define INPUT_SERIAL 1
#define INPUT_ROS 2
#define INPUT_RANDOM 3

void handleCommandline(uint &inputMethod, std::string &sInput, std::string &sLayout, bool &bMirror, std::string &sCalib,
                       TaxelMapping &mapping, int argc, char *argv[])
{
	// default input method
	inputMethod = INPUT_SERIAL;
	sInput = "/dev/ttyACM0";
	string sMapping;
	string sHandedness;

	po::options_description options("options");
	po::options_description config;  // config options feasible for both cmdline + file
	config.add_options()("mapping,m", po::value<string>(&sMapping),
	                     "name of taxel mapping from config\ne.g. P1, P2, use --list for full list")(
	    "layout,l", po::value<string>(&sLayout)->required(), "glove layout SVG\ne.g. L1, L2, ..., or file name")(
	    "handedness", po::value<string>(&sHandedness)->default_value("right"),
	    "right | left")("calib,c", po::value<string>(&sCalib), "calibration map");

	po::options_description hidden;  // hidden positional options
	hidden.add_options()("taxelmap", po::value<vector<string> >()->composing(), "taxel mappings");

	po::options_description input("input options");
	input.add_options()("serial,s", po::value<string>(&sInput)->implicit_value(sInput), "use serial input (default)")
#if HAVE_ROS
	    ("ros,r", po::value<string>(&sInput)->implicit_value("TactileGlove"), "use ros input")
#endif
	        ("dummy,d", "use random dummy input");

	cmd.add_options()("help,h", "Display this help message.")("list", "List available mappings")(
	    "file,f", po::value<string>(), "config file: specify options and mapping(s)");
	cmd.add(config);
	cmd.add(input);

	po::positional_options_description pos;
	pos.add("taxelmap", -1);

	po::variables_map map;
	po::options_description cmdl_opts;
	cmdl_opts.add(cmd).add(hidden);
	po::store(po::command_line_parser(argc, argv).options(cmdl_opts).positional(pos).run(), map);

	if (map.count("help")) {
		usage(argv);
		exit(EXIT_SUCCESS);
	}

	if (map.count("list")) {
		auto mappings = getAvailableMappings(map.count("file") ? map["file"].as<string>() : string());
		if (!mappings.empty())
			std::cout << "available mappings: " << std::endl;
		else
			std::cout << "no mappings available" << std::endl;

		for (const auto &mapping : mappings)
			std::cout << mapping << std::endl;
		exit(EXIT_SUCCESS);
	}

	TaxelMapping configFileMapping;
	if (map.count("file")) {
		// we need to access config and mapping values before we can do notify() below
		string sConfigFile = map["file"].as<string>();
		if (map.count("mapping"))
			sMapping = map["mapping"].as<string>();

		ifstream ifs(sConfigFile.c_str());
		if (!ifs)
			throw std::runtime_error("cannot open config file " + sConfigFile);
		po::options_description fileOpts;
		fileOpts.add(config).add(input);
		configFileMapping = getMapping(ifs, sMapping, map, fileOpts);
	}

	if (map.count("ros") + map.count("serial") + map.count("dummy") > 1)
		throw std::logic_error("multiple input methods specified");

	// *** merge taxel mapping options ***
	if (map.count("mapping"))
		sMapping = map["mapping"].as<string>();
	mapping = getMapping(sMapping, map);  // initialize from defaults

	// if mapping was provided, but layout is still undefined -> complain
	if (!sMapping.empty() && map.count("layout") == 0)
		throw runtime_error("unknown mapping " + sMapping);
	if (map.count("layout") == 0)
		throw runtime_error("please provide a taxel mapping (-m) or at least a layout (-l)");

	// fill variables and issue exceptions on errors
	// this is only possible here, after having processed configFile and defaultMapping
	po::notify(map);
	sLayout = map["layout"].as<string>();

	// merge stuff from explicit config file
	mapping.merge(configFileMapping);
	// merge cmdline mapping
	if (map.count("taxelmap")) {
		TaxelMapping cmdlineMapping;
		boost::regex r("([._A-Za-z0-9]*)=(\\d*)");
		boost::smatch match;
		BOOST_FOREACH (const std::string &s, map["taxelmap"].as<vector<string> >()) {
			if (boost::regex_match(s, match, r))
				cmdlineMapping[match[1]] = TaxelMapping::parseChannel(match[2]);
			else
				throw std::runtime_error("invalid taxel mapping: " + s);
		}
		mapping.merge(cmdlineMapping);
	}
	if (mapping.empty())
		cerr << "warning: no mapping specified" << endl;

	if (map.count("serial"))
		inputMethod = INPUT_SERIAL;
	if (map.count("dummy"))
		inputMethod = INPUT_RANDOM;
	if (map.count("ros"))
		inputMethod = INPUT_ROS;
	bMirror = (map["handedness"].as<string>() == string("left"));
}

QApplication *pApp;
void mySigIntHandler(int /*signal*/)
{
	if (pApp)
		pApp->quit();
}

int main(int argc, char *argv[])
{
	uint inputMethod;
	std::string sInput, sLayout, sCalib;
	bool bMirror;
	TaxelMapping mapping;
	try {
		handleCommandline(inputMethod, sInput, sLayout, bMirror, sCalib, mapping, argc, argv);
	} catch (const exception &e) {
		cerr << e.what() << endl;
		usage(argv);
		return EXIT_FAILURE;
	}

	QApplication app(argc, argv);
	MainWindow w(64);
	try {
		w.initGloveWidget(QString::fromStdString(sLayout), bMirror, mapping);
		if (!sCalib.empty())
			w.setCalibration(sCalib);
	} catch (const std::exception &e) {
		cerr << e.what() << endl;
		return EXIT_FAILURE;
	}

	QString qsInput = QString::fromStdString(sInput);
	switch (inputMethod) {
	case INPUT_SERIAL:
		w.configSerial(qsInput);
		break;
#if HAVE_ROS
	case INPUT_ROS:
		ros::init(argc, argv, "tactile_glove_viz");
		if (!ros::master::check()) {
			cerr << "Failed to contact ROS master." << endl;
			return EXIT_FAILURE;
		}
		w.configROS(qsInput);
		break;
#endif
	case INPUT_RANDOM:
		w.configRandom();
		break;
	}

	// install signal handler
	pApp = &app;
	signal(SIGINT, mySigIntHandler);

	w.show();
	return app.exec();
}
