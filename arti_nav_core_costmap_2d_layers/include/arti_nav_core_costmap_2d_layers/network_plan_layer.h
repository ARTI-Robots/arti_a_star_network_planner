/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Clemens Mühlbacher,
 *                      ARTI-Robots GesmbH.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Graz University of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#ifndef ARTI_NETWORK_PLAN_COSTMAP_2D_LAYER_NETWORK_PLAN_LAYER_H_
#define ARTI_NETWORK_PLAN_COSTMAP_2D_LAYER_NETWORK_PLAN_LAYER_H_

#include <arti_nav_core_costmap_2d_layers/NetworkPlanLayerConfig.h>
#include <arti_nav_core_costmap_2d_layers/point_set.h>
#include <arti_nav_core_msgs/Movement2DGoalWithConstraints.h>
#include <boost/optional.hpp>
#include <boost/thread/mutex.hpp>
#include <costmap_2d/costmap_2d.h>
#include <dynamic_reconfigure/server.h>
#include <string>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <arti_nav_core_costmap_2d_layers/nanoflann.hpp>
#include <costmap_2d/layer.h>

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace arti_nav_core_costmap_2d_layers
{

class NetworkPlanLayer : public costmap_2d::Layer
{
public:
  NetworkPlanLayer();

  ~NetworkPlanLayer() override;

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
    double* max_y) override;

  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

  void matchSize() override;

  void reset() override;

protected:
  using KDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointSet>, PointSet, 2>;
  using Lock = boost::lock_guard<boost::recursive_mutex>;

  class ResultSet
  {
  public:
    explicit ResultSet(double max_distance_)
      : max_distance(max_distance_)
    {
    }

    inline double worstDist() const
    {
      return max_distance;
    }

    inline bool full() const
    {
      return found_point;
    }

    /**
     * Called during search to add an element matching the criteria.
     * @return true if the search should be continued, false if the results are sufficient
     */
    inline bool addPoint(const double dist, size_t /*index*/)
    {
      if (dist <= max_distance)
      {
        found_point = true;
      }
      return !found_point;
    }

    double max_distance;
    bool found_point{false};
  };

  void resetBounds();
  bool transformBounds(geometry_msgs::Point& transformed_ll, geometry_msgs::Point& transformed_ur);
  void resetBuffer();
  void reconfigure(const NetworkPlanLayerConfig& config);
  void processNetworkPlan(const arti_nav_core_msgs::Movement2DGoalWithConstraintsConstPtr& network_plan);
  unsigned char calculateCostsFor(unsigned int x, unsigned int y, const geometry_msgs::TransformStamped& transform);

  boost::recursive_mutex mutex_;
  std::string path_frame_;

  costmap_2d::Costmap2D buffer_;
  PointSet current_point_set_;
  geometry_msgs::Point lower_left_bound_;
  geometry_msgs::Point upper_right_bound_;

  boost::optional<dynamic_reconfigure::Server<NetworkPlanLayerConfig>> config_server_;
  NetworkPlanLayerConfig config_;
  ros::Subscriber network_plan_subscriber_;
  boost::optional<KDTree> kd_tree_;
};

}

#endif // ARTI_NETWORK_PLAN_COSTMAP_2D_LAYER_NETWORK_PLAN_LAYER_H_

