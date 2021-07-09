#pragma once

#include <string>
#include <iostream>
#include <limits>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <yaml-cpp/yaml.h>

namespace visualizer
{
  struct Point
  {
    std::pair<double, double> location;
    cv::Scalar color;
    int radius;

    Point()
    {
    }

    Point(std::pair<double, double> location, cv::Scalar color, int radius)
      : location(location), color(color), radius(radius)
    {
    }
  };

  struct Polygon
  {
    std::vector<std::pair<double, double>> points;
    cv::Scalar color;

    Polygon()
    {
    }

    Polygon(std::vector<std::pair<double, double>> points, cv::Scalar color) : points(points), color(color)
    {
    }
  };

  class Visualizer
  {
  private:
    typedef std::pair<std::pair<double, double>, std::pair<double, double>> Edge;

    int pixels_per_meter;
    std::string window_name;
    YAML::Node track;
    std::vector<Edge> track_edges;

    cv::Mat image;

    void draw();
    void draw_track();
    void draw_points();
    void draw_polygons();

  public:
    Visualizer(std::string window_name, int pixels_per_meter = 50);
    ~Visualizer();

    std::vector<std::shared_ptr<Point>> points;
    std::vector<std::shared_ptr<Polygon>> polygons;
    std::pair<double, double> track_size = { 14, 14 };
    std::pair<double, double> zero_offset = { 7, 7 };

    void display();
    void load_track(std::string track_file);
    void autoscale_to_track();
    void add_mouse_callback(cv::MouseCallback callback, void* user_data);
    std::pair<double, double> image_to_world(cv::Point point);
    cv::Point world_to_image(std::pair<double, double> point);
  };
}  // namespace visualizer
