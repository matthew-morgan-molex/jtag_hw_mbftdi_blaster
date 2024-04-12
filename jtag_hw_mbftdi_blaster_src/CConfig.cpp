#ifdef _WINDOWS
#include <windows.h>
#else
#endif
#include "CConfig.h"

CConfig g_cfg;

CConfig::CConfig()
{
    // setup defaults
    m_parameter_value["JtagClock"] = 10000000;
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
    if (m_parameter_value.find(key) == m_parameter_value.end())
    {
        return 1;
    }
    *value = m_parameter_value[key];
    return 0;
}

std::list<std::string> CConfig::getIpServers()
{
    std::list<std::string> L;
    return L;
}
