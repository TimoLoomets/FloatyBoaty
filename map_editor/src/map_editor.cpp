#include <map_editor.hpp>

int main(int argc, char** argv)
{
  std::string track_file = "track.yaml";

  std::vector<std::string> all_args;
  if (argc > 1)
  {
    all_args.assign(argv + 1, argv + argc);
  }
  for (int i = 0; i < all_args.size(); i++)
  {
    if (all_args[i] == "--track-file")
    {
      if (i + 1 >= all_args.size())
        return -1;
      track_file = all_args[i + 1];
      ++i;
    }
  }

  map_editor::Editor editor(track_file);
  return 0;
}

namespace map_editor
{
  Editor::Editor(std::string track_file) : track_file(track_file)
  {
    ensure_file_exists(track_file);

    visualizer = std::make_unique<visualizer::Visualizer>(track_file);
    visualizer->add_mouse_callback(mouse_callback, this);
    visualizer->load_track(track_file);
    visualizer->display();

    hover_point->color = cv::Scalar(125, 125, 125);
    hover_point->radius = 3;
    visualizer->points.push_back(hover_point);

    cv::waitKey();
  }

  Editor::~Editor()
  {
  }

  void Editor::hover_at(cv::Point location)
  {
    std::pair<double, double> world_loc = visualizer->image_to_world(location);
    world_loc.first = round(world_loc.first / grid_size) * grid_size;
    world_loc.second = round(world_loc.second / grid_size) * grid_size;
    hover_point->location = world_loc;
    visualizer->display();
  }

  void Editor::left_click_at(cv::Point location)
  {
    if (first_edge_point)
    {
      add_edge(first_edge_point->location, hover_point->location);
      visualizer->points.erase(std::remove(visualizer->points.begin(), visualizer->points.end(), first_edge_point),
                               visualizer->points.end());
      first_edge_point.reset();
      visualizer->load_track(track_file);
      visualizer->display();
    }
    else
    {
      first_edge_point = std::make_shared<visualizer::Point>(hover_point->location, cv::Scalar(255, 0, 0), 3);
      visualizer->points.push_back(first_edge_point);
    }
  }

  void Editor::add_edge(std::pair<double, double> start_point, std::pair<double, double> end_point)
  {
    std::vector<std::vector<double>> edge = { { start_point.first, start_point.second },
                                              { end_point.first, end_point.second } };
    YAML::Node track = YAML::LoadFile(track_file);
    track.push_back(edge);
    std::ofstream fout(track_file);
    fout << track;
  }

  void mouse_callback(int event, int x, int y, int flags, void* userdata)
  {
    Editor* editor = static_cast<Editor*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN)
    {
      editor->left_click_at(cv::Point(x, y));
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
      std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if (event == cv::EVENT_MBUTTONDOWN)
    {
      std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if (event == cv::EVENT_MOUSEMOVE)
    {
      editor->hover_at(cv::Point(x, y));
    }
  }

  void ensure_file_exists(const std::string name)
  {
    std::ifstream f(name);

    if (!f.good())
    {
      f.close();
      std::ofstream file(name);
    }
  }
}  // namespace map_editor