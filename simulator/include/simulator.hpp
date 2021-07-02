#pragma once
#include <string>

#include <robot.hpp>

class Simulator
{
private:
    Robot robot;
    
public:
    Simulator(std::string robot_file, std::string track_file);
    ~Simulator();
};
