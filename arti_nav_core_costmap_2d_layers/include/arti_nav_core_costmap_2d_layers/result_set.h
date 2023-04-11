/// \file
/// \author Alexander Buchegger
/// \date 2021-09-07
/// \copyright ARTI - Autonomous Robot Technology GmbH. All rights reserved.
#ifndef ARTI_NAV_CORE_COSTMAP_2D_LAYERS_RESULT_SET_H
#define ARTI_NAV_CORE_COSTMAP_2D_LAYERS_RESULT_SET_H

#include <cstddef>

namespace arti_nav_core_costmap_2d_layers
{

///
/// Result Set for k-d tree.
///
class ResultSet
{
public:
  enum class Result
  {
    OUTSIDE,
    INSIDE_PADDING,
    INSIDE_CORRIDOR,
  };

  ResultSet(double deviation_distance, double deviation_with_padding_distance);

  Result getResult() const;

  inline double worstDist() const
  {
    return deviation_with_padding_distance_sqr_;
  }

  inline bool full() const
  {
    return result_ == Result::INSIDE_CORRIDOR;
  }

  ///
  /// Called during search to add an element matching the criteria.
  /// @return true if the search should be continued, false if the results are sufficient
  ///
  inline bool addPoint(const double dist, size_t /*index*/)
  {
    if (dist <= deviation_distance_sqr_)
    {
      result_ = Result::INSIDE_CORRIDOR;
    }
    else if (result_ == Result::OUTSIDE && dist <= deviation_with_padding_distance_sqr_)
    {
      result_ = Result::INSIDE_PADDING;
    }
    return result_ != Result::INSIDE_CORRIDOR;
  }

protected:
  double deviation_distance_sqr_;
  double deviation_with_padding_distance_sqr_;
  Result result_{Result::OUTSIDE};
};

}

#endif // ARTI_NAV_CORE_COSTMAP_2D_LAYERS_RESULT_SET_H
