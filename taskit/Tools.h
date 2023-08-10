#pragma once

#include <iostream>
#include <iostream>
#include <geometry_msgs/Pose.h>

#define MI_DEBUG_TOOLS

#ifdef MI_DEBUG_TOOLS
    #define PAUSE std::cin.get();
    #define DEBUG(msg) std::cout << "\033[1;36m > ("<< __func__ << "): \033[0;37m" << msg << "\033[0m \n"
    #define DEBUG_VEC(msg, vec) std::cout << "\033[1;36m > ("<< __func__ << "): \033[0;37m" << msg << "\033[0m " << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")" << '\n'
                
#else
    #define PAUSE 
    #define DEBUG(msg) 
    #define DEBUG_VEC(msg, vec)
#endif

namespace TaskIt {

static std::string getParamName(const std::string& param_name, std::string ns = std::string()) {
    if (ns.empty()) return "/" + param_name;
    if (ns.front() != '/') ns = "/" + ns;
    if (ns.back() != '/') ns.push_back('/');
    return ns + param_name;
}

static geometry_msgs::Pose neutralPose() {
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    return pose;
}

}