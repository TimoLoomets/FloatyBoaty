#pragma once
#include <string>

#include <yaml-cpp/yaml.h>

class Robot
{
private:
  struct Motor
  {
    Motor()
    {
    }

    Motor(std::pair<double, double> location, double heading) : location(location), heading(heading)
    {
    }

    std::pair<double, double> location;
    double heading;
  };

  struct FrictionPoint
  {
    FrictionPoint(std::pair<double, double> location, std::pair<double, double> coefficients)
      : location(location), coefficients(coefficients)
    {
    }

    std::pair<double, double> location;
    std::pair<double, double> coefficients;
  };

  struct CenterOfGravity
  {
    std::pair<double, double> location;
    double mass;
    double moment_of_inertia;
  };

  struct State
  {
    std::pair<double, double> linear;
    double angular;
  };

public:
  Robot(std::string robot_file);
  ~Robot();

  CenterOfGravity center_of_gravity;
  std::map<std::string, Motor> motors;
  std::map<std::string, FrictionPoint> friction_points;
  State position;
  State velocity;
  State acceleration;

  void apply_motor_force(std::string motor, double force);
  void apply_force(std::pair<double, double> location, std::pair<double, double> force);
};
