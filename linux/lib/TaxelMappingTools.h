#pragma once

#include "TaxelMapping.h"
#include <istream>
#include <vector>

/// return minimal set of config options (layout, handedness)
boost::program_options::options_description getConfigOptions();

/// retrieve taxel mapping for given sMappingFilter from is
TaxelMapping getMapping(std::istream &is, const std::string &sMappingFilter,
                        boost::program_options::variables_map &optsMap,
                        const boost::program_options::options_description &opts = getConfigOptions());

/// retrieve taxel mapping for given sMappingFilter from compile-in taxel.cfg
TaxelMapping getMapping(const std::string &sMappingFilter, boost::program_options::variables_map &optsMap);

/// get list of taxel mappings / glove version ids provided in is
std::vector<std::string> getAvailableMappings(std::istream &is);
/// get list of all (from file and compiled-in) glove version ids
std::vector<std::string> getAvailableMappings(const std::string &sConfigFile = "");
