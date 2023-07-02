#pragma once

#include <iostream>
#include <iostream>

#define MI_DEBUG_TOOLS

#ifdef MI_DEBUG_TOOLS
    #define PAUSE std::cin.get();
    #define DEBUG(msg) std::cout << "\033[1;36m > ("<< __func__ << "): \033[0;37m" << msg << "\033[0m \n"
#else
    #define PAUSE 
    #define DEBUG(msg) 
#endif

namespace TaskIt {

static std::string getParamName(const std::string& param_name, std::string ns = std::string()) {
    if (ns.empty()) return "/" + param_name;
    if (ns.front() != '/') ns = "/" + ns;
    if (ns.back() != '/') ns.push_back('/');
    return ns + param_name;
}

}