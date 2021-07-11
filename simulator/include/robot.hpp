#pragma once
#include <string>

#include <yaml-cpp/yaml.h>

#include <include/visualizer.hpp>

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

  struct Sensor
  {
    Sensor()
    {
    }

    Sensor(std::pair<double, double> location, std::string type) : location(location), type(type)
    {
    }

    std::pair<double, double> location;
    std::string type;
  };

  struct DistanceSensor : Sensor
  {
    DistanceSensor()
    {
    }

    DistanceSensor(std::pair<double, double> location, std::string type, double heading, double detection_angle)
      : Sensor(location, type), heading(heading), detection_angle(detection_angle)
    {
    }

    double heading;
    double detection_angle;
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

  struct VisualisationComponents{
    std::shared_ptr<visualizer::Polygon> boundary = std::make_shared<visualizer::Polygon>();
    std::shared_ptr<visualizer::Point> cog = std::make_shared<visualizer::Point>();
    std::map<std::string, std::shared_ptr<visualizer::Point>> motors;
    std::map<std::string, std::tuple<std::shared_ptr<visualizer::Point>, std::shared_ptr<visualizer::Line>,
        std::shared_ptr<visualizer::Line>>> sensors;
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
  std::pair<double, double> local_to_global(std::pair<double, double> point);

public:
  Robot(std::string robot_file);
  ~Robot();

  CenterOfGravity center_of_gravity;
  std::map<std::string, Motor> motors;
  std::map<std::string, DistanceSensor> sensors; // TODO: Add some inheritance and stuff to support multiple types of sensors.
  std::vector<std::pair<double, double>> boundary_points;
  AxisComponents friction_coefficient;
  AxisComponents position;
  AxisComponents velocity;
  AxisComponents acceleration;
  VisualisationComponents visualisation;

  void update_visualisation();
  void add_to_visualizer(visualizer::Visualizer& visualizer);
  void apply_motor_force(std::string motor, double force);
  void apply_force(std::pair<double, double> location, std::pair<double, double> force);
  void step(double time);
};
