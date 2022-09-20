#include "TaxelMapping.h"
#include <iostream>

#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

using namespace std;
namespace po = boost::program_options;

TaxelMapping::TaxelMapping() {}

TaxelMapping::TaxelMapping(const std::vector<po::basic_option<char> > &options)
{
	for (const auto &option : options) {
		if (!option.unregistered)
			continue;
		(*this)[option.string_key] = parseChannel(option.value.back());
	}
}

TaxelMapping &TaxelMapping::merge(const TaxelMapping &other, const string &sMapping)
{
	for (const auto &it : other) {
		string key = it.first;
		size_t dot = key.rfind('.');
		if (dot != string::npos) {
			// ignore entries for other mappings than sMapping
			if (sMapping.empty() || key.substr(0, dot) != sMapping)
				continue;
			key = key.substr(dot + 1);
		}
		(*this)[key] = it.second;
	}
	return *this;
}

int TaxelMapping::parseChannel(const string &sValue)
{
	if (sValue.empty())
		return -1;
	try {
		return boost::lexical_cast<mapped_type>(sValue);
	} catch (const boost::bad_lexical_cast &e) {
		// rethrow with different message
		throw runtime_error((boost::format("'%s' cannot be interpreted as channel index") % sValue).str());
	}
}
