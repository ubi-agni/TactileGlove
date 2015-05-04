#include "MainWindow.h"

#include <iostream>
#include <fstream>
#include <cstdio>
#include <QResource>
#include <QApplication>
#include <boost/program_options.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/regex.hpp>
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

// remove "prefix.layout" options, enabling the one with prefix=sMappingFilter
void filterLayoutOptions(const string &sMappingFilter,
                         std::vector<po::basic_option<char> > &options)
{
	const string sLayoutKey ("layout");
	const string sMatchKey = sMappingFilter + "." + sLayoutKey;

	for (std::vector<po::basic_option<char> >::iterator
	     it=options.begin(), end=options.end(); it!=end;) {
		if (it->unregistered && boost::ends_with(it->string_key, sLayoutKey)) {
			if (it->string_key == sMatchKey) {
				// turn option into recognized one
				it->unregistered = false;
				it->string_key = sLayoutKey;
			} else {
				// remove this entry
				it = options.erase(it); // returns next iterator
				continue; // do not ++it again
			}
		}
		++it;
	}
}

// load mapping for sMappingFilter from is and store option "layout" in map
// also consider declard options opts
TaxelMapping mappingFromStream(istream &is,
                               const string &sMappingFilter,
                               const po::options_description &opts,
                               po::variables_map &optsMap)
{
	po::parsed_options parsed = po::parse_config_file(is, opts, true);
	filterLayoutOptions(sMappingFilter, parsed.options);
	po::store(parsed, optsMap); // store known options

	TaxelMapping all(parsed.options); // mapping from all unknown options
	return TaxelMapping().merge(all, sMappingFilter); // filter by sMappingFilter
}

// load default mapping for sMappingFilter and store option "layout" in map
TaxelMapping getDefaultMapping(const std::string &sMappingFilter,
                               const po::options_description &opts,
                               po::variables_map &optsMap)
{
	if (sMappingFilter.empty()) return TaxelMapping();

	QResource res(":taxel.cfg");
	QByteArray data = res.isCompressed() ? qUncompress(res.data(), res.size())
	                                     : QByteArray((const char*) res.data(), res.size());
	istringstream iss(data.constData());
	return mappingFromStream(iss, sMappingFilter, opts, optsMap);
}


bool handleCommandline(uint &inputMethod, std::string &sInput,
                       std::string &sLayout, TaxelMapping &mapping,
                       int argc, char *argv[])
{
	// default input method
	inputMethod = INPUT_SERIAL;
	sInput = "/dev/ttyACM0";
	string sMapping;

	po::options_description options("options");
	po::options_description config; // config options feasible for both cmdline + file
	config.add_options()
		("mapping,m", po::value<string>(&sMapping),
		 "name of taxel mapping from config\ne.g. P1, P2, P3l, P3r")
		("layout,l", po::value<string>(&sLayout)->default_value("P3")->required(),
		 "glove layout SVG\ne.g. P1, P3, or file name")
		;

	po::options_description hidden; // hidden positional options
	hidden.add_options()
		("taxelmap", po::value<vector<string> >()->composing(), "taxel mappings");

	po::options_description input("input options");
	input.add_options()
		("serial,s", po::value<string>(&sInput)->implicit_value(sInput),
		 "use serial input (default)")
#if HAVE_ROS
		("ros,r", po::value<string>(&sInput)->implicit_value("TactileGlove"),
		 "use ros input")
		#endif
		("dummy,d", "use random dummy input");

	cmd.add_options()
		("help,h", "Display this help message.")
		("config,c", po::value<string>(), "config file");
	cmd.add(config);
	cmd.add(input);

	po::positional_options_description pos;
	pos.add("taxelmap", -1);

	po::variables_map map;
	po::store(po::command_line_parser(argc, argv)
	          .options(po::options_description().add(cmd).add(hidden))
	          .positional(pos)
	          .run(), map);

	if (map.count("help")) return true;

	TaxelMapping configFileMapping;
	if (map.count("config")) {
		// we need to access config and mapping values before we can do notify() below
		string sConfigFile = map["config"].as<string>();
		if (map.count("mapping")) sMapping = map["mapping"].as<string>();

		ifstream ifs(sConfigFile.c_str());
		if (!ifs) throw std::runtime_error("cannot open config file " + sConfigFile);
		po::options_description fileOpts; fileOpts.add(config).add(input);
		configFileMapping = mappingFromStream(ifs, sMapping, fileOpts, map);
	}

	if (map.count("ros") + map.count("serial") + map.count("dummy") > 1)
		throw std::logic_error("multiple input methods specified");

	// *** merge taxel mapping options ***
	if (map.count("mapping")) sMapping = map["mapping"].as<string>();
	mapping = getDefaultMapping(sMapping, config, map); // initialize from defaults

	// if mapping was provided, but layout stays
	if (!sMapping.empty() && map.count("layout") == 0)
		throw runtime_error("unknown mapping " + sMapping);

	// fill variables and issue exceptions on errors
	// this is only possible here, after having processed configFile and defaultMapping
	po::notify(map);

	// merge stuff from explicit config file
	mapping.merge(configFileMapping);
	// merge cmdline mapping
	if (map.count("taxelmap")) {
		TaxelMapping cmdlineMapping;
		boost::regex r("([._A-Za-z0-9]*)=(\\d*)");
		boost::smatch match;
		BOOST_FOREACH(const std::string &s, map["taxelmap"].as< vector<string> >()) {
			if (boost::regex_match(s, match, r))
				cmdlineMapping[match[1]] = TaxelMapping::parseChannel(match[2]);
			else throw std::runtime_error("invalid taxel mapping: " + s);
		}
		mapping.merge(cmdlineMapping);
	}
	if (!map.count("layout"))


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
