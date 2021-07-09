#include <simulator.hpp>

Simulator::Simulator(std::string robot_file, std::string track_file) : robot{ robot_file }, visualizer{ "Simulator" }
{
  visualizer.load_track(track_file);
  visualizer.autoscale_to_track();

  YAML::Node track = YAML::LoadFile(track_file);
  if (track["robot_start_position"])
  {
    robot.position.linear = std::make_pair(track["robot_start_position"]["location"][0].as<double>(),
                                           track["robot_start_position"]["location"][1].as<double>());
    robot.position.angular = track["robot_start_position"]["heading"].as<double>();
  }

  robot.update_visualisation();
  robot.add_to_visualizer(visualizer);

  visualizer.display();

  cv::waitKey();
}

Simulator::~Simulator()
{
}