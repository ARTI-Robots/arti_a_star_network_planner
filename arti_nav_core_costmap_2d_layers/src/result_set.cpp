/// \file
/// \author Alexander Buchegger
/// \date 2021-09-07
/// \copyright ARTI - Autonomous Robot Technology GmbH. All rights reserved.
#include <arti_nav_core_costmap_2d_layers/result_set.h>

namespace arti_nav_core_costmap_2d_layers
{

ResultSet::ResultSet(double deviation_distance, double deviation_with_padding_distance)
  : deviation_distance_sqr_(deviation_distance * deviation_distance),
    deviation_with_padding_distance_sqr_(deviation_with_padding_distance * deviation_with_padding_distance)
{
}

ResultSet::Result ResultSet::getResult() const
{
  return result_;
}

}
