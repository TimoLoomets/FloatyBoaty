#pragma once
#include <string>
#include <iostream>

#include <include/visualizer.hpp>

#include <robot.hpp>

class Simulator
{
private:
    Robot robot;
    visualizer::Visualizer visualizer;
    
public:
    Simulator(std::string robot_file, std::string track_file);
    ~Simulator();
};
