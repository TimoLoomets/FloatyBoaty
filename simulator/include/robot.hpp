#pragma once
#include <string>

#include <yaml-cpp/yaml.h>

#include <include/visualizer.hpp>

#define G 9.81
class Robot
{
private:
  struct AxisComponents
  {
    std::pair<double, double> linear;
    double angular;
  };
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

    void update_visualization(const Robot* robot)
    {
      if (visualisation_point)
      {
        visualisation_point->location = robot->local_to_global(location);
      }
      else
      {
        visualisation_point =
            std::make_shared<visualizer::Point>(robot->local_to_global(location), cv::Scalar(155, 155, 0), 2);
      }
    }

    void add_to_visualizer(visualizer::Visualizer& visualizer) const
    {
      visualizer.points.push_back(visualisation_point);
    }

    std::shared_ptr<visualizer::Point> visualisation_point;
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

    void update_visualization(const Robot* robot)
    {
      Sensor::update_visualization(robot);
      if (visualisation_left_edge)
      {
        visualisation_left_edge->location = robot->local_to_global(location);
        visualisation_left_edge->heading = heading + detection_angle / 2 + robot->position.angular;
      }
      else
      {
        visualisation_left_edge =
            std::make_shared<visualizer::Line>(robot->local_to_global(location), cv::Scalar(155, 155, 0), 1.5,
                                               heading + detection_angle / 2 + robot->position.angular);
      }
      if (visualisation_right_edge)
      {
        visualisation_left_edge->location = robot->local_to_global(location);
        visualisation_left_edge->heading = heading - detection_angle / 2 + robot->position.angular;
      }
      else
      {
        visualisation_right_edge =
            std::make_shared<visualizer::Line>(robot->local_to_global(location), cv::Scalar(155, 155, 0), 1.5,
                                               heading - detection_angle / 2 + robot->position.angular);
      }
    }

    void add_to_visualizer(visualizer::Visualizer& visualizer) const
    {
      Sensor::add_to_visualizer(visualizer);
      visualizer.lines.push_back(visualisation_left_edge);
      visualizer.lines.push_back(visualisation_right_edge);
    }

    std::shared_ptr<visualizer::Line> visualisation_left_edge;
    std::shared_ptr<visualizer::Line> visualisation_right_edge;

    double heading;
    double detection_angle;
  };

  struct CenterOfGravity
  {
    std::pair<double, double> location;
    double mass;
    double moment_of_inertia;
  };

  struct VisualisationComponents
  {
    std::shared_ptr<visualizer::Polygon> boundary = std::make_shared<visualizer::Polygon>();
    std::shared_ptr<visualizer::Point> cog = std::make_shared<visualizer::Point>();
    std::map<std::string, std::shared_ptr<visualizer::Point>> motors;
    std::map<std::string, std::tuple<std::shared_ptr<visualizer::Point>, std::shared_ptr<visualizer::Line>,
                                     std::shared_ptr<visualizer::Line>>>
        sensors;
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
  std::pair<double, double> local_to_global(std::pair<double, double> point) const;

public:
  Robot(std::string robot_file);
  ~Robot();

  CenterOfGravity center_of_gravity;
  std::map<std::string, Motor> motors;
  std::map<std::string, DistanceSensor> sensors;  // TODO: Add some inheritance and stuff to support multiple types of
                                                  // sensors.
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
  std::optional<double> measure_edge_with_sensor(std::string sensor, std::pair<std::pair<double, double>,std::pair<double, double>> edge);
};
