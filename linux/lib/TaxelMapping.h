#pragma once

#include <map>
#include <string>
#include <boost/program_options.hpp>

class TaxelMapping : public std::map<std::string, unsigned int>
{
public:
	/// construct empty mapping
	TaxelMapping();
	/// construct mapping from unprocessed options
	TaxelMapping(const std::vector<boost::program_options::basic_option<char> > &options);

	/// merge mapping from other, overwriting existing associations
	/// if keys are given in form prefix.name, prefix needs to match (non-empty) sMapping
	TaxelMapping &merge(const TaxelMapping &other, const std::string &sMapping = "");

	static int parseChannel(const std::string &sValue);
};
