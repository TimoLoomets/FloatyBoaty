#pragma once

#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <yaml-cpp/yaml.h>

namespace visualizer
{
  class Visualizer
  {
  private:
    typedef std::pair<std::pair<double, double>, std::pair<double, double>> edge;

    int pixels_per_meter;
    std::string window_name;
    YAML::Node track;
    std::vector<edge> track_edges;

    cv::Mat image;

    void draw();
    void draw_track();

    cv::Point world_to_image(std::pair<double, double> point);

  public:
    Visualizer(std::string window_name, int pixels_per_meter = 50);
    ~Visualizer();

    std::pair<double, double> track_size = { 14, 14 };
    std::pair<double, double> zero_offset = { 7, 7 };

    void display();
    void load_track(std::string track_file);
  };
}  // namespace visualizer
