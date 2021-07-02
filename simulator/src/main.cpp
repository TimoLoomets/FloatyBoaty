#include <simulator.hpp>

int main(int argc, char const* argv[])
{
  std::string track_file = "track.yaml";
  std::string robot_file = "robot.yaml";

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
    else if (all_args[i] == "--robot-file")
    {
      if (i + 1 >= all_args.size())
        return -1;
      robot_file = all_args[i + 1];
      ++i;
    }
  }
  Simulator simulator(robot_file, track_file);
  return 0;
}