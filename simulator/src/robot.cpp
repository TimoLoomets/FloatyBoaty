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

  friction_coefficient.linear = std::make_pair(robot["friction_coefficients"]["linear"][0].as<double>(),
                                               robot["friction_coefficients"]["linear"][1].as<double>());
  friction_coefficient.angular = robot["friction_coefficients"]["angular"].as<double>();

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

double Robot::get_change(double time, double& velocity, const double& acceleration, const double& deccelarition)
{
  double distance_traveled = 0;
  double velocity_change_rate;
  if (velocity != 0)
  {
    velocity_change_rate = acceleration + get_sign(velocity) * (-1) * deccelarition;
    if (get_sign(velocity) * get_sign(velocity_change_rate) == -1 && velocity_change_rate != 0)
    {
      const double time_to_zero = velocity / velocity_change_rate;
      if (time_to_zero <= time)
      {
        distance_traveled += velocity_change_rate * time * time / 2 + velocity * time;
        time -= time_to_zero;
        velocity = 0;
      }
    }
  }

  if (velocity == 0)
  {
    velocity_change_rate = acceleration - get_sign(acceleration) * deccelarition;
  }
  distance_traveled += velocity_change_rate * time * time / 2 + velocity * time;
  velocity += velocity_change_rate * time;
  
  return distance_traveled;
}

void Robot::step(double time)
{
  const double x_coefficient = friction_coefficient.linear.first * cos(position.angular) -
                               friction_coefficient.linear.second * sin(position.angular);
  const double y_coefficient = friction_coefficient.linear.first * sin(position.angular) +
                               friction_coefficient.linear.second * cos(position.angular);
  position.linear.first +=
      get_change(time, velocity.linear.first, acceleration.linear.first, x_coefficient * center_of_gravity.mass * G);
  position.linear.second +=
      get_change(time, velocity.linear.second, acceleration.linear.second, y_coefficient * center_of_gravity.mass * G);
  position.angular += get_change(time, velocity.angular, acceleration.angular,
                                 friction_coefficient.angular * center_of_gravity.moment_of_inertia);
}
