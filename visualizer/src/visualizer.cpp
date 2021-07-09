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
    track_edges.clear();
    for (std::size_t i = 0; i < track["edges"].size(); i++)
    {
      track_edges.push_back(std::make_pair(std::make_pair(track["edges"][i][0][0].as<double>(), track["edges"][i][0][1].as<double>()),
                                           std::make_pair(track["edges"][i][1][0].as<double>(), track["edges"][i][1][1].as<double>())));
    }
  }

  void Visualizer::draw()
  {
    image = cv::Mat(cv::Size(static_cast<int>(track_size.first * pixels_per_meter),
                             static_cast<int>(track_size.second * pixels_per_meter)),
                    CV_8UC3, cv::Scalar(0, 0, 0));
    draw_track();
    draw_robot_start_position();
    draw_polygons();
    draw_points();
  }

  void Visualizer::draw_track()
  {
    for (Edge track_edge : track_edges)
    {
      cv::line(image, world_to_image(track_edge.first), world_to_image(track_edge.second), cv::Scalar(255, 0, 0));
    }
  }

  void Visualizer::draw_robot_start_position()
  {
    if (track["robot_start_position"])
    {
      cv::Point robot_pos_on_image =
          world_to_image(std::make_pair(track["robot_start_position"]["location"][0].as<double>(),
                                        track["robot_start_position"]["location"][1].as<double>()));
      double robot_heading = track["robot_start_position"]["heading"].as<double>();
      cv::Point heading_arrow_end(static_cast<int>(robot_pos_on_image.x + 10 * cos(-robot_heading)),
                                  static_cast<int>(robot_pos_on_image.y + 10 * sin(-robot_heading)));

      cv::circle(image,
                 robot_pos_on_image,
                 3, cv::Scalar(255, 255, 0), -1);
      cv::arrowedLine(image, robot_pos_on_image, heading_arrow_end, cv::Scalar(255, 255, 0), 1, 8, 0, 0.3);
    }
  }

  void Visualizer::draw_points()
  {
    for (auto point : points)
    {
      cv::circle(image, world_to_image(point->location), point->radius, point->color, -1);
    }
  }

  void Visualizer::draw_polygons()
  {
    for (auto polygon : polygons)
    {
      std::vector<cv::Point> polygon_points;
      for (auto point : polygon->points)
      {
        polygon_points.push_back(world_to_image(point));
      }
      std::vector<std::vector<cv::Point>> polygons;
      polygons.push_back(polygon_points);
      cv::fillPoly(image, polygons, polygon->color);
    }
  }

  void Visualizer::autoscale_to_track()
  {
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();

    for (Edge track_edge : track_edges)
    {
      min_x = std::min({ min_x, track_edge.first.first, track_edge.second.first });
      min_y = std::min({ min_y, track_edge.first.second, track_edge.second.second });

      max_x = std::max({ max_x, track_edge.first.first, track_edge.second.first });
      max_y = std::max({ max_y, track_edge.first.second, track_edge.second.second });
    }

    std::pair<double, double> new_track_size{ max_x - min_x + 1, max_y - min_y + 1 };
    pixels_per_meter = std::ceil(std::max(track_size.first / new_track_size.first * pixels_per_meter,
                                          track_size.second / new_track_size.second * pixels_per_meter));

    zero_offset = std::make_pair(new_track_size.first / 2, new_track_size.second / 2); 

    track_size = new_track_size;
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
