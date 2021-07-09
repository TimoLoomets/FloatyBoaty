#pragma once
#include <utility>
#include <algorithm>

namespace Util
{
  bool on_segment(const std::pair<double, double> p, const std::pair<double, double> q, const std::pair<double, double> r)
  {
    if (q.first <= std::max(p.first, r.first) && q.first >= std::min(p.first, r.first) && q.second <= std::max(p.second, r.second) &&
        q.second >= std::min(p.second, r.second))
      return true;
    return false;
  }

  // To find orientation of ordered triplet (p, q, r).
  // The function returns following values
  // 0 --> p, q and r are colinear
  // 1 --> Clockwise
  // 2 --> Counterclockwise
  int orientation(const std::pair<double, double> p, const std::pair<double, double> q, const std::pair<double, double> r)
  {
    double val = (q.second - p.second) * (r.first - q.first) - (q.first - p.first) * (r.second - q.second);

    if (val == 0)
      return 0;                // colinear
    return (val > 0) ? 1 : 2;  // clock or counterclock wise
  }

  // The function that returns true if line segment 'p1q1'
  // and 'p2q2' intersect.
  bool do_intersect(const std::pair<double, double> p1, const std::pair<double, double> q1, const std::pair<double, double> p2,
                   const std::pair<double, double> q2)
  {
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
      return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && on_segment(p1, p2, q1))
      return true;

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && on_segment(p1, q2, q1))
      return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && on_segment(p2, p1, q2))
      return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && on_segment(p2, q1, q2))
      return true;

    return false;  // Doesn't fall in any of the above cases
  }
}  // namespace Util
