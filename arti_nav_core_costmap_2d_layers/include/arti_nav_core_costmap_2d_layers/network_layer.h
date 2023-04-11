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

#ifndef ARTI_NETWORK_PLAN_COSTMAP_2D_LAYER_NETWORK_LAYER_H_
#define ARTI_NETWORK_PLAN_COSTMAP_2D_LAYER_NETWORK_LAYER_H_

#include <arti_nav_core_costmap_2d_layers/NetworkLayerConfig.h>
#include <arti_nav_core_costmap_2d_layers/point_set.h>
#include <arti_nav_core_costmap_2d_layers/result_set.h>
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

class NetworkLayer : public costmap_2d::Layer
{
public:
  NetworkLayer();

  ~NetworkLayer() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
    double* max_y) override;

  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_x, int min_y, int max_x, int max_y) override;

  void reset() override;

  void matchSize() override;

protected:
  using KDTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointSet>, PointSet, 2>;
  using Lock = boost::lock_guard<boost::recursive_mutex>;

  void onInitialize() override;

  //bool transformBounds(geometry_msgs::Point& transformed_ll, geometry_msgs::Point& transformed_ur);
  void resetBuffer();
  void reconfigure(const NetworkLayerConfig& config);
  void processPlan(const arti_nav_core_msgs::Movement2DGoalWithConstraintsConstPtr& network_plan);
  ResultSet::Result getDistanceToNetwork(
    const KDTree& kd_tree, unsigned int x, unsigned int y, const geometry_msgs::TransformStamped& transform,
    double max_distance, double padding);
  void loadNetwork();

  boost::recursive_mutex mutex_;
  ros::NodeHandle node_handle_;

  costmap_2d::Costmap2D buffer_;
  bool update_pending_{false};

  boost::optional<dynamic_reconfigure::Server<NetworkLayerConfig>> config_server_;
  NetworkLayerConfig config_;

  ros::Subscriber plan_subscriber_;
  boost::optional<KDTree> network_kd_tree_;
  PointSet network_point_set_;
  std::string network_frame_;
  geometry_msgs::Point network_lower_left_bound_;
  geometry_msgs::Point network_upper_right_bound_;

  boost::optional<KDTree> plan_kd_tree_;
  PointSet plan_point_set_;
  std::string plan_frame_;
  geometry_msgs::Point plan_lower_left_bound_;
  geometry_msgs::Point plan_upper_right_bound_;
};

}

#endif // ARTI_NETWORK_PLAN_COSTMAP_2D_LAYER_NETWORK_LAYER_H_
