#include "TaxelMapping.h"
#include <iostream>
#include <fstream>
#include <cstdio>
#include <QResource>

#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

using namespace std;
namespace po=boost::program_options;

const TaxelMapping& getDefaults() {
	static TaxelMapping defaults;
	if (!defaults.empty()) return defaults;

	QResource res(":taxel.cfg");
	QByteArray data = res.isCompressed() ? qUncompress(res.data(), res.size())
	                                     : QByteArray((const char*) res.data(), res.size());
	istringstream iss(data.constData());
	defaults = TaxelMapping(po::parse_config_file(iss, po::options_description(), true).options);
	return defaults;
}

TaxelMapping::TaxelMapping()
{
}

TaxelMapping::TaxelMapping(const std::vector<po::basic_option<char> > options) {
	for (std::vector<po::basic_option<char> >::const_iterator
	     it=options.begin(), end=options.end(); it!=end; ++it) {
		if (!it->unregistered) continue;
		(*this)[it->string_key] = parseChannel(it->value.back());
		cout << it->string_key << ":" << parseChannel(it->value.back()) << endl;
	}
}

TaxelMapping::TaxelMapping(const string& sLayout)
{
	merge(getDefaults(), sLayout);
}

TaxelMapping &TaxelMapping::merge(const TaxelMapping &other, const string &sLayout)
{
	for (const_iterator it=other.begin(), e=other.end(); it!=e; ++it) {
		string key=it->first;
		size_t dot = key.rfind('.');
		if (dot != string::npos) {
			// ignore entries for other layouts than sLayout
			if (sLayout.empty() || key.substr(0,dot) != sLayout) continue;
			key = key.substr(dot+1);
		}
		(*this)[key] = it->second;
	}
	return *this;
}

int TaxelMapping::parseChannel(const string &sValue)
{
	if (sValue.empty()) return -1;
	try {
		return boost::lexical_cast<mapped_type>(sValue);
	} catch (const boost::bad_lexical_cast &e) {
		// rethrow with different message
		throw runtime_error((boost::format("'%s' cannot be interpreted as channel index")
		                    % sValue).str());
	}
}
