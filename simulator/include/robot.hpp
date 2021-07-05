#pragma once
#include <string>

#include <yaml-cpp/yaml.h>

#define G 9.81
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

  struct CenterOfGravity
  {
    std::pair<double, double> location;
    double mass;
    double moment_of_inertia;
  };

  struct AxisComponents
  {
    std::pair<double, double> linear;
    double angular;
  };

  int get_sign(double number)
  {
    if (number == 0)
    {
      return 0;
    }
    else if (number > 0)
    {
      return 1;
    }
    else
    {
      return -1;
    }
  }

  double get_change(double time, double& velocity, const double& acceleration, const double& deccelarition);

public:
  Robot(std::string robot_file);
  ~Robot();

  CenterOfGravity center_of_gravity;
  std::map<std::string, Motor> motors;
  AxisComponents friction_coefficient;
  AxisComponents position;
  AxisComponents velocity;
  AxisComponents acceleration;

  void apply_motor_force(std::string motor, double force);
  void apply_force(std::pair<double, double> location, std::pair<double, double> force);
  void step(double time);
};
