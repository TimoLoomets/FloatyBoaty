#pragma once
#include <fstream>
#include <iostream>
#include <algorithm>

#include <include/visualizer.hpp>
#include <include/util.hpp>

namespace map_editor
{
  class Editor
  {
  private:
    enum MouseMode
    {
      ADD,
      DELETE,
      ROBOT_POSITION
    };
    MouseMode mouse_mode = MouseMode::ADD;

    double grid_size = 0.1;
    std::string track_file;
    std::unique_ptr<visualizer::Visualizer> visualizer;
    std::shared_ptr<visualizer::Point> hover_point = std::make_shared<visualizer::Point>();
    std::shared_ptr<visualizer::Point> first_edge_point;

    void add_edge(std::pair<double, double> start_point, std::pair<double, double> end_point);
    void cut_edges(std::pair<double, double> start_point, std::pair<double, double> end_point);

  public:
    Editor(std::string track_file);
    ~Editor();

    void hover_at(cv::Point location);
    void left_click_at(cv::Point location);
    void right_button_down();
    void right_button_up();
  };

  void ensure_file_exists(const std::string name);
  void mouse_callback(int event, int x, int y, int flags, void* userdata);
}  // namespace map_editor
