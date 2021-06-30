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
    if (all_args[i] == "--track_file")
    {
      if (i + 1 >= all_args.size())
        return -1;
      track_file = all_args[i + 1];
      ++i;
    }
  }

  map_editor::Editor editor(track_file);
}

namespace map_editor
{
  Editor::Editor(std::string track_file)
  {
    ensure_file_exists(track_file);
    visualizer = std::make_unique<visualizer::Visualizer>(track_file);
    visualizer->add_mouse_callback(mouse_callback, this);
    visualizer->load_track(track_file);
    visualizer->display();
    hover_point->color = cv::Scalar(125, 125, 125);
    hover_point->radius = 3;
    visualizer->points.push_back(hover_point);
    std::cout << "points: " << visualizer->points.size() << "\n";
    
    cv::waitKey();
  }

  Editor::~Editor()
  {
  }

  void Editor::hover_at(cv::Point location)
  {
    std::cout << "Hover loc: " << location << "\n";
    std::pair<double, double> world_loc = visualizer->image_to_world(location);
    std::cout << "World loc: " << world_loc.first << " " << world_loc.second << "\n";
    world_loc.first = round(world_loc.first / grid_size) * grid_size;
    world_loc.second = round(world_loc.second / grid_size) * grid_size;
    std::cout << "Rounded world loc: " << world_loc.first << " " << world_loc.second << "\n";
    hover_point->location = visualizer->world_to_image(world_loc);
    std::cout << "Image loc: " << hover_point->location << "\n";
    visualizer->display();
    std::cout << "\n";
  }

  void mouse_callback(int event, int x, int y, int flags, void* userdata)
  {
    Editor* editor = static_cast<Editor*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN)
    {
      std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
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
    std::ofstream file(name);
  }
}  // namespace map_editor