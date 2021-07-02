#include <robot.hpp>

Robot::Robot(std::string robot_file)
{
  YAML::Node robot = YAML::LoadFile(robot_file);

  center_of_gravity.location = std::make_pair(robot["center_of_gravity"]["location"][0].as<double>(),
                                              robot["center_of_gravity"]["location"][1].as<double>());
  center_of_gravity.mass = robot["center_of_gravity"]["mass"].as<double>();
  center_of_gravity.moment_of_inertia = robot["center_of_gravity"]["moment_of_inertia"].as<double>();

  if (robot["motors"])
  {
    for (YAML::const_iterator it = robot["motors"].begin(); it != robot["motors"].end(); ++it)
    {
      Motor motor(std::make_pair(it->second["location"][0].as<double>() - center_of_gravity.location.first,
                                 it->second["location"][1].as<double>() - center_of_gravity.location.second),
                  it->second["heading"].as<double>());
      motors.insert({ it->first.as<std::string>(), motor });
    }
  }

  if (robot["friction_points"])
  {
    for (YAML::const_iterator it = robot["friction_points"].begin(); it != robot["friction_points"].end(); ++it)
    {
      FrictionPoint friction_point(
          std::make_pair(it->second["location"][0].as<double>() - center_of_gravity.location.first,
                         it->second["location"][1].as<double>() - center_of_gravity.location.second),
          std::make_pair(it->second["coefficients"][0].as<double>(), it->second["coefficients"][1].as<double>()),
          9.81 / robot["friction_points"].size());
      friction_points.insert({ it->first.as<std::string>(), friction_point });
    }
  }

  center_of_gravity.location = std::make_pair(0, 0);
}

Robot::~Robot()
{
}

void Robot::apply_motor_force(std::string motor, double force)
{
  apply_force(motors[motor].location,
              std::make_pair(force * cos(motors[motor].heading), force * sin(motors[motor].heading)));
}

void Robot::apply_force(std::pair<double, double> location, std::pair<double, double> force)
{
  double location_len = sqrt(location.first * location.first + location.second * location.second);
  double force_len = sqrt(force.first * force.first + force.second * force.second);

  double angle_cos = std::min(
      std::max((location.first * force.first + location.second * force.second) / location_len / force_len, -1.0), 1.0);
  double angle_sin = sin(atan2(force.second, force.first) - atan2(location.second, location.first));

  acceleration.linear.first -= location.first / location_len * angle_cos * force_len / center_of_gravity.mass;
  acceleration.linear.second -= location.second / location_len * angle_cos * force_len / center_of_gravity.mass;
  acceleration.angular += angle_sin * force_len * location_len / center_of_gravity.moment_of_inertia;
}

void Robot::step(double time)
{
  
}
