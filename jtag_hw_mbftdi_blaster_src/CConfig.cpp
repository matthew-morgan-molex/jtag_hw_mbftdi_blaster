#ifdef _WINDOWS
#include <windows.h>
#else
#endif
#include "CConfig.h"
#include "debug.h"
#include <cstring>

CConfig::CConfig()
{
    // setup defaults
    m_parameter_value["JtagClock"] = 10000000;
    m_parameter_value["SerialNumber"] = 0;
    //m_parameter_value["JtagClockAutoAdjust"] = 0;
}

CConfig::~CConfig()
{
}

unsigned int CConfig::set_value(std::string key, unsigned int value)
{
    if (m_parameter_value.find(key) == m_parameter_value.end())
    {
        return 1;
    }
    m_parameter_value[key] = value;
    return 0;
}

unsigned int CConfig::get_value(std::string key, unsigned int* value)
{
    printd("get_value %s\n", key.c_str());
    if (m_parameter_value.find(key) == m_parameter_value.end())
    {
        printd("*** key not found\n");
        return 1;
    }
    *value = m_parameter_value[key];
    printd("  value = %d\n", *value);
    return 0;
}

std::list<std::string> CConfig::getIpServers()
{
    std::list<std::string> L;
    return L;
}
