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

  if (robot["sensors"])
  {
    for (YAML::const_iterator it = robot["sensors"].begin(); it != robot["sensors"].end(); ++it)
    {
      if (it->second["type"].as<std::string>().compare("distance") == 0)
      {
        DistanceSensor sensor(
            std::make_pair(it->second["location"][0].as<double>() - center_of_gravity.location.first,
                           it->second["location"][1].as<double>() - center_of_gravity.location.second),
            it->second["type"].as<std::string>(), it->second["heading"].as<double>(),
            it->second["detection_angle"].as<double>());
        sensors.insert({ it->first.as<std::string>(), sensor });
      }
    }
  }

  friction_coefficient.linear = std::make_pair(robot["friction_coefficients"]["linear"][0].as<double>(),
                                               robot["friction_coefficients"]["linear"][1].as<double>());
  friction_coefficient.angular = robot["friction_coefficients"]["angular"].as<double>();

  if (robot["boundary"])
  {
    for (std::size_t i = 0; i < robot["boundary"].size(); i++)
    {
      boundary_points.push_back(
          std::make_pair(robot["boundary"][i][0].as<double>() - center_of_gravity.location.first,
                         robot["boundary"][i][1].as<double>() - center_of_gravity.location.second));
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

std::pair<double, double> Robot::local_to_global(std::pair<double, double> point) const
{
  point = std::make_pair(point.first * cos(position.angular) - point.second * sin(position.angular),
                         point.first * sin(position.angular) + point.second * cos(position.angular));

  point.first = point.first + position.linear.first;
  point.second = point.second + position.linear.second;

  return point;
}

void Robot::update_visualisation()
{
  visualisation.boundary->color = cv::Scalar(255, 0, 255);
  visualisation.boundary->points = {};
  for (auto const& point : boundary_points)
  {
    visualisation.boundary->points.push_back(local_to_global(point));
  }

  for (auto const& motor : motors)
  {
    if (visualisation.motors.find(motor.first) == visualisation.motors.end())
    {
      visualisation.motors[motor.first] =
          std::make_shared<visualizer::Point>(local_to_global(motor.second.location), cv::Scalar(0, 155, 0), 2);
    }
    visualisation.motors[motor.first]->location = local_to_global(motor.second.location);
  }

  for (auto& sensor : sensors)
  {
    sensor.second.update_visualization(this);
  }

  visualisation.cog->color = cv::Scalar(0, 255, 255);
  visualisation.cog->radius = 2;
  visualisation.cog->location = local_to_global(center_of_gravity.location);
}

void Robot::add_to_visualizer(visualizer::Visualizer& visualizer)
{
  visualizer.points.push_back(visualisation.cog);

  for (auto const& motor : visualisation.motors)
  {
    visualizer.points.push_back(motor.second);
  }

  for (auto const& sensor : sensors)
  {
    sensor.second.add_to_visualizer(visualizer);
  }

  visualizer.polygons.push_back(visualisation.boundary);
}
