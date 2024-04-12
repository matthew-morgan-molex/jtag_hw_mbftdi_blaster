#pragma once

#include <fstream>
#include <string>
#include <list>
#include <map>

#include "debug.h"

class CConfig
{
public:
    CConfig();
    ~CConfig();
    std::list<std::string> getIpServers();
    unsigned int set_value(std::string key, unsigned int value);
    unsigned int get_value(std::string key, unsigned int* value);
private:
    std::map<std::string, unsigned int> m_parameter_value;
};

extern CConfig g_cfg;
