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
    visualizer.get()->add_mouse_callback(mouse_callback, this);
    visualizer.get()->load_track(track_file);
    visualizer.get()->display();
    cv::waitKey();
  }

  Editor::~Editor()
  {
  }

  void mouse_callback(int event, int x, int y, int flags, void* userdata)
  {
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
      std::cout << "Mouse move over the window - position (" << x << ", " << y << ")" << std::endl;
    }
    else if (event == cv::EVENT_MOUSEMOVE)
    {
      std::cout << "Mouse move over the window - position (" << x << ", " << y << ")" << std::endl;
    }
  }

  void ensure_file_exists(const std::string name)
  {
    std::ofstream file(name);
  }
}  // namespace map_editor