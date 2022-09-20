#include "TaxelMappingTools.h"
#include <iostream>
#include <fstream>
#include <cstdio>
#include <QFile>
#include <boost/assign/list_of.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <algorithm>

namespace po = boost::program_options;
using namespace std;

po::options_description getConfigOptions()
{
	po::options_description opts;
	opts.add_options()("layout", po::value<string>()->required(),
	                   "SVG layout name")("handedness", po::value<string>()->default_value("right"));
	return opts;
}

// remove options of form "prefix.layout" or "prefix.handedness"
// enabling those with prefix=sMappingFilter
static void filterOptions(const string &sMappingFilter, std::vector<po::basic_option<char> > &options)
{
	for (std::vector<po::basic_option<char> >::iterator it = options.begin(); it != options.end();) {
		if (it->unregistered) {
			const static std::vector<std::string> vFilters = boost::assign::list_of("layout")("handedness");
			std::vector<std::string>::const_iterator sFilterKey = vFilters.end();
			for (auto fit = vFilters.begin(), end = vFilters.end(); fit != end; ++fit) {
				if (boost::ends_with(it->string_key, *fit)) {
					sFilterKey = fit;
					break;
				}
			}
			if (sFilterKey != vFilters.end()) {
				if (it->string_key == sMappingFilter + "." + *sFilterKey) {
					// turn option into recognized one
					it->unregistered = false;
					it->string_key = *sFilterKey;
				} else {
					// remove this entry
					it = options.erase(it);  // returns next iterator
					continue;  // do not ++it again
				}
			}
		}
		++it;
	}
}

// load mapping for sMappingFilter from is and store option "layout" in map
// also consider declard options opts
TaxelMapping getMapping(istream &is, const string &sMappingFilter, po::variables_map &optsMap,
                        const po::options_description &opts)
{
	po::parsed_options parsed = po::parse_config_file(is, opts, true);
	filterOptions(sMappingFilter, parsed.options);
	po::store(parsed, optsMap);  // store known options

	TaxelMapping all(parsed.options);  // mapping from all unknown options
	return TaxelMapping().merge(all, sMappingFilter);  // filter by sMappingFilter
}

// load default mapping for sMappingFilter and store option "layout" in map
TaxelMapping getMapping(const std::string &sMappingFilter, po::variables_map &optsMap)
{
	if (sMappingFilter.empty())
		return TaxelMapping();

	QFile cfg(":taxel.cfg");
	cfg.open(QIODevice::ReadOnly);
	istringstream iss(cfg.readAll().constData());
	return getMapping(iss, sMappingFilter, optsMap);
}

std::vector<std::string> getAvailableMappings(istream &is)
{
	std::vector<std::string> result;
	if (!is)
		return result;

	po::parsed_options parsed = po::parse_config_file(is, getConfigOptions(), true);

	std::map<std::string, unsigned int> groups;
	for (const auto &option : parsed.options) {
		string::size_type dot_pos = option.string_key.find('.');
		if (option.unregistered && dot_pos != string::npos) {
			groups[option.string_key.substr(0, dot_pos)]++;
		}
	}

	for (const auto &group : groups) {
		// only list groups with more than 10 mappings
		if (group.second >= 10)
			result.push_back(group.first);
	}

	return result;
}

std::vector<std::string> getAvailableMappings(const std::string &sConfigFile)
{
	std::vector<std::string> groups;
	// find groups in sConfigFile
	ifstream ifs(sConfigFile.c_str());
	if (ifs)
		groups = getAvailableMappings(ifs);

	// find groups in builtin taxel.cfg
	QFile cfg(":taxel.cfg");
	cfg.open(QIODevice::ReadOnly);
	istringstream iss(cfg.readAll().constData());
	const std::vector<std::string> &builtin = getAvailableMappings(iss);

	// merge
	for (const auto &name : builtin) {
		if (std::find(groups.begin(), groups.end(), name) == groups.end())
			groups.push_back(name);
	}
	return groups;
}
