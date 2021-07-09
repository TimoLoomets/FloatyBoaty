#include <simulator.hpp>

Simulator::Simulator(std::string robot_file, std::string track_file) : robot{ robot_file }, visualizer{ "Simulator" }
{
  visualizer.load_track(track_file);
  visualizer.autoscale_to_track();

  robot.update_visualisation();
  robot.add_to_visualizer(visualizer);

  visualizer.display();

  cv::waitKey();
}

Simulator::~Simulator()
{
}