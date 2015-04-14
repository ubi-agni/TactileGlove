#pragma once

#include <map>
#include <string>
#include <boost/program_options.hpp>

class TaxelMapping : public std::map<std::string, unsigned int>
{
public:
	TaxelMapping();
	TaxelMapping(const std::string &sLayout);
	TaxelMapping(const std::vector<boost::program_options::basic_option<char> > options);

	TaxelMapping& merge(const TaxelMapping& other,
	                    const std::string &sLayout="");
};
