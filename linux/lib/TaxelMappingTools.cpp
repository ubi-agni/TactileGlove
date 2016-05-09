#include "TaxelMappingTools.h"
#include <iostream>
#include <fstream>
#include <cstdio>
#include <QResource>
#include <boost/assign/list_of.hpp>
#include <boost/algorithm/string/predicate.hpp>

namespace po=boost::program_options;
using namespace std;

po::options_description getConfigOptions() {
	po::options_description opts;
	opts.add_options()
		("layout", po::value<string>()->required(), "SVG layout name")
		("handedness", po::value<string>()->default_value("right"))
		;
	return opts;
}

// remove options of form "prefix.layout" or "prefix.handedness"
// enabling those with prefix=sMappingFilter
static void filterOptions(const string &sMappingFilter,
                          std::vector<po::basic_option<char> > &options)
{
	for (std::vector<po::basic_option<char> >::iterator
		     it=options.begin(); it!=options.end();) {
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
				if(it->string_key == sMappingFilter + "." + *sFilterKey) {
					// turn option into recognized one
					it->unregistered = false;
					it->string_key = *sFilterKey;
				} else {
					// remove this entry
					it = options.erase(it); // returns next iterator
					continue; // do not ++it again
				}
			}
		}
		++it;
	}
}

// load mapping for sMappingFilter from is and store option "layout" in map
// also consider declard options opts
TaxelMapping getMapping(istream &is,
                        const string &sMappingFilter,
                        po::variables_map &optsMap,
                        const po::options_description &opts)
{
	po::parsed_options parsed = po::parse_config_file(is, opts, true);
	filterOptions(sMappingFilter, parsed.options);
	po::store(parsed, optsMap); // store known options

	TaxelMapping all(parsed.options); // mapping from all unknown options
	return TaxelMapping().merge(all, sMappingFilter); // filter by sMappingFilter
}

// load default mapping for sMappingFilter and store option "layout" in map
TaxelMapping getMapping(const std::string &sMappingFilter,
                        po::variables_map &optsMap,
                        const po::options_description &opts)
{
	if (sMappingFilter.empty()) return TaxelMapping();

	QResource res(":taxel.cfg");
	QByteArray data = res.isCompressed() ? qUncompress(res.data(), res.size())
		: QByteArray((const char*) res.data(), res.size());
	istringstream iss(data.constData());
	return getMapping(iss, sMappingFilter, optsMap);
}


std::vector<std::string> getAvailableMappings (istream &is)
{
	std::vector<std::string> result;
	if (!is) return result;

	po::parsed_options parsed = po::parse_config_file(is, getConfigOptions(), true);

	std::map<std::string, unsigned int> groups;
	for (auto it = parsed.options.begin(), end = parsed.options.end(); it != end; ++it) {
		string::size_type dot_pos = it->string_key.find(".");
		if (it->unregistered && dot_pos != string::npos) {
			groups[it->string_key.substr(0, dot_pos)]++;
		}
	}

	for (auto it = groups.begin(), end = groups.end(); it != end; ++it) {
		// only list groups with more than 10 mappings
		if (it->second >= 10) result.push_back(it->first);
	}

	return result;
}

std::vector<std::string>
getAvailableMappings(const std::string &sConfigFile)
{
	std::vector<std::string> groups;
	// find groups in sConfigFile
	ifstream ifs(sConfigFile.c_str());
	if (ifs) groups = getAvailableMappings(ifs);

	// find groups in builtin taxel.cfg
	QResource res(":taxel.cfg");
	QByteArray data = res.isCompressed() ? qUncompress(res.data(), res.size())
		: QByteArray((const char*) res.data(), res.size());
	istringstream iss(data.constData());
	const std::vector<std::string> &builtin = getAvailableMappings(iss);

	// merge
	std::copy(builtin.begin(), builtin.end(), std::back_inserter(groups));
	return groups;
}
