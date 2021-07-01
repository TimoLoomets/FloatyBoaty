#include <visualizer.hpp>

namespace visualizer
{
  Visualizer::Visualizer(std::string window_name, int pixels_per_meter)
    : window_name(window_name), pixels_per_meter(pixels_per_meter)
  {
    cv::namedWindow(window_name, 1);
  }

  Visualizer::~Visualizer()
  {
    cv::destroyWindow(window_name);
  }

  void Visualizer::display()
  {
    cv::namedWindow(window_name);
    draw();
    cv::imshow(window_name, image);
  }

  void Visualizer::load_track(std::string track_file)
  {
    track = YAML::LoadFile(track_file);
    for (std::size_t i = 0; i < track.size(); i++)
    {
      track_edges.push_back(std::make_pair(std::make_pair(track[i][0][0].as<double>(), track[i][0][1].as<double>()),
                                           std::make_pair(track[i][1][0].as<double>(), track[i][1][1].as<double>())));
    }
  }

  void Visualizer::draw()
  {
    image = cv::Mat(cv::Size(static_cast<int>(track_size.first * pixels_per_meter),
                             static_cast<int>(track_size.second * pixels_per_meter)),
                    CV_8UC3, cv::Scalar(0, 0, 0));
    draw_track();
    draw_points();
  }

  void Visualizer::draw_track()
  {
    for (edge track_edge : track_edges)
    {
      cv::line(image, world_to_image(track_edge.first), world_to_image(track_edge.second), cv::Scalar(255, 0, 0));
    }
  }

  void Visualizer::draw_points()
  {
    for (auto point : points)
    {
      cv::circle(image, world_to_image(point->location), point->radius, point->color, -1);
    }
  }

  cv::Point Visualizer::world_to_image(std::pair<double, double> point)
  {
    return { static_cast<int>((point.first + zero_offset.first) * pixels_per_meter),
             static_cast<int>((track_size.second - point.second - zero_offset.second) * pixels_per_meter) };
  }

  std::pair<double, double> Visualizer::image_to_world(cv::Point point)
  {
    return std::make_pair(point.x / static_cast<double>(pixels_per_meter) - zero_offset.first,
                          track_size.second - point.y / static_cast<double>(pixels_per_meter) - zero_offset.second);
  }

  void Visualizer::add_mouse_callback(cv::MouseCallback callback, void* user_data)
  {
    cv::setMouseCallback(window_name, callback, user_data);
  }

}  // namespace visualizer
