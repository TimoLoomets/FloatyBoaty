#include <visualizer.hpp>

int main(int argc, char** argv)
{
  visualizer::Visualizer vis("myVis");
  vis.display();
  cv::waitKey();
}