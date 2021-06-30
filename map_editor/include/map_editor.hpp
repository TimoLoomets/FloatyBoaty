#pragma once
#include <fstream>
#include <iostream>

#include <include/visualizer.hpp>

namespace map_editor
{
  class Editor
  {
  private:
    double grid_size = 0.1;
    std::unique_ptr<visualizer::Visualizer> visualizer;
    std::shared_ptr<visualizer::point> hover_point = std::make_shared<visualizer::point>();

  public:
    Editor(std::string track_file);
    ~Editor();

    void hover_at(cv::Point location);
  };

  void ensure_file_exists(const std::string name);
  void mouse_callback(int event, int x, int y, int flags, void* userdata);
}  // namespace map_editor
