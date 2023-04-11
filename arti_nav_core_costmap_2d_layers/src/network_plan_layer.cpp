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

#include <arti_nav_core_costmap_2d_layers/network_plan_layer.h>
#include <arti_nav_core_utils/transformer.h>
#include <functional>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(arti_nav_core_costmap_2d_layers::NetworkPlanLayer, costmap_2d::Layer)

namespace arti_nav_core_costmap_2d_layers
{

NetworkPlanLayer::NetworkPlanLayer()
  : buffer_(0, 0, 1, 0, 0, costmap_2d::NO_INFORMATION)
{
}

NetworkPlanLayer::~NetworkPlanLayer() = default;

void NetworkPlanLayer::onInitialize()
{
  ros::NodeHandle nh;
  ros::NodeHandle layer_nh("~/" + name_);
  current_ = true;
  enabled_ = true;

  config_server_.emplace(mutex_, layer_nh);
  config_server_->setCallback(std::bind(&NetworkPlanLayer::reconfigure, this, std::placeholders::_1));
  network_plan_subscriber_ = layer_nh.subscribe("network_plan", 1, &NetworkPlanLayer::processNetworkPlan, this);
}

void NetworkPlanLayer::matchSize()
{
  Lock lock{mutex_};
  const costmap_2d::Costmap2D* const costmap = layered_costmap_->getCostmap();
  buffer_.resizeMap(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), costmap->getResolution(),
                    costmap->getOriginX(), costmap->getOriginY());
  //  ROS_INFO_STREAM("NetworkPlanLayer::matchSize: sizeInCellsX: " << costmap->getSizeInCellsX() << ", sizeInCellsY: "
  //                                                                << costmap->getSizeInCellsY());
}

void NetworkPlanLayer::reconfigure(const NetworkPlanLayerConfig& config)
{
  Lock lock{mutex_};
  if (!enabled_ && config.enabled)
  {
    matchSize();
  }
  enabled_ = config.enabled;
  config_ = config;
}

void NetworkPlanLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double* /*min_x*/, double* /*min_y*/,
  double* /*max_x*/, double* /*max_y*/)
{
  Lock lock{mutex_};
  if (enabled_ && kd_tree_)
  {
    if (layered_costmap_->isRolling())
    {
      //      ROS_INFO_STREAM("NetworkPlanLayer::updateBounds: originX: " << layered_costmap_->getCostmap()->getOriginX()
      //                                                                  << ", originY: "
      //                                                                  << layered_costmap_->getCostmap()->getOriginY());
      buffer_.updateOrigin(layered_costmap_->getCostmap()->getOriginX(), layered_costmap_->getCostmap()->getOriginY());
    }

    //    geometry_msgs::Point lower_left;
    //    geometry_msgs::Point upper_right;
    //    if (transformBounds(lower_left, upper_right))
    //    {
    //      *min_x = std::min(*min_x, lower_left.x - 1.2 * path_deviation_distance_);
    //      *min_y = std::min(*min_y, lower_left.y - 1.2 * path_deviation_distance_);
    //      *max_x = std::max(*max_x, upper_right.x + 1.2 * path_deviation_distance_);
    //      *max_y = std::max(*max_y, upper_right.y + 1.2 * path_deviation_distance_);
    //    }

    //    ROS_INFO_STREAM(
    //      "NetworkPlanLayer::updateBounds: lower_left.x: " << lower_left.x << ", lower_left.y: " << lower_left.y
    //                                                       << ", upper_right.x: " << upper_right.x << ", upper_right.y: "
    //                                                       << upper_right.y);
    //resetBounds();  // Only update min_* / max_* once after the path has changed to avoid unnecessary computations
  }
}

void NetworkPlanLayer::resetBounds()
{
  Lock lock{mutex_};
  lower_left_bound_.x = std::numeric_limits<double>::max();
  lower_left_bound_.y = std::numeric_limits<double>::max();
  upper_right_bound_.x = -std::numeric_limits<double>::max();
  upper_right_bound_.y = -std::numeric_limits<double>::max();
}

//bool NetworkPlanLayer::transformBounds(geometry_msgs::Point& transformed_ll, geometry_msgs::Point& transformed_ur)
//{
//  const std::string target_frame = layered_costmap_->getGlobalFrameID();
//  if (!tf_->waitForTransform(target_frame, path_frame_, ros::Time(), ros::Duration(1.0)))
//  {
//    ROS_ERROR_STREAM("transform failed: " << path_frame_);
//    return false;
//  }
//
//  geometry_msgs::PointStamped ll;
//  geometry_msgs::PointStamped lr;
//  geometry_msgs::PointStamped ul;
//  geometry_msgs::PointStamped ur;
//
//  geometry_msgs::PointStamped current_point;
//  current_point.header.frame_id = path_frame_;
//  current_point.point = lower_left_bound_;
//  tf_->transformPoint(target_frame, current_point, ll);
//  current_point.point.y = upper_right_bound_.y;
//  tf_->transformPoint(target_frame, current_point, ul);
//  current_point.point.x = upper_right_bound_.x;
//  tf_->transformPoint(target_frame, current_point, ur);
//  current_point.point.y = lower_left_bound_.y;
//  tf_->transformPoint(target_frame, current_point, lr);
//
//  transformed_ll.x = std::min({ll.point.x, lr.point.x, ul.point.x, ur.point.x});
//  transformed_ll.y = std::min({ll.point.y, lr.point.y, ul.point.y, ur.point.y});
//  transformed_ur.x = std::max({ll.point.x, lr.point.x, ul.point.x, ur.point.x});
//  transformed_ur.y = std::max({ll.point.y, lr.point.y, ul.point.y, ur.point.y});
//  return true;
//}

void NetworkPlanLayer::resetBuffer()
{
  buffer_.resetMap(0, 0, buffer_.getSizeInCellsX(), buffer_.getSizeInCellsY());
}

void NetworkPlanLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  Lock lock{mutex_};
  if (enabled_ && kd_tree_)
  {
    //    ROS_INFO_STREAM("NetworkPlanLayer::updateCosts: min_i: " << min_i << ", min_j: " << min_j << ", max_i: " << max_i
    //                                                             << ", max_j: " << max_j);
    std::string tf_error_message;
    boost::optional<geometry_msgs::TransformStamped> transform;
    bool tried_to_get_transform{false};
    for (unsigned j = static_cast<unsigned>(min_j); j < static_cast<unsigned>(max_j); ++j)
    {
      for (unsigned i = static_cast<unsigned>(min_i); i < static_cast<unsigned>(max_i); ++i)
      {
        if (master_grid.getCost(i, j) != costmap_2d::NO_INFORMATION)
        {
          unsigned char current_costs = buffer_.getCost(i, j);
          if (current_costs == costmap_2d::NO_INFORMATION)
          {
            // If costs haven't been calculated for this cell yet, do so now:
            if (!transform && !tried_to_get_transform)
            {
              transform = arti_nav_core_utils::tryToLookupTransform(*tf_, path_frame_,
                                                                    layered_costmap_->getGlobalFrameID(), ros::Time(),
                                                                    ros::Duration(1.0), &tf_error_message);
              tried_to_get_transform = true;
              if (!transform)
              {
                ROS_ERROR_STREAM(
                  "failed to get transform from '" << layered_costmap_->getGlobalFrameID() << "' to '" << path_frame_
                                                   << "'");
              }
            }

            if (transform)
            {
              current_costs = calculateCostsFor(i, j, *transform);
              buffer_.setCost(i, j, current_costs);
            }
          }

          if (current_costs == costmap_2d::LETHAL_OBSTACLE)
          {
            master_grid.setCost(i, j, current_costs);
          }
        }
      }
    }
  }
}

void NetworkPlanLayer::reset()
{
  Lock lock{mutex_};
  resetBuffer();
}

void NetworkPlanLayer::processNetworkPlan(const arti_nav_core_msgs::Movement2DGoalWithConstraintsConstPtr& network_plan)
{
  Lock lock{mutex_};
  kd_tree_.reset();
  current_point_set_.clear();
  resetBuffer();
  resetBounds();

  for (size_t i = 0; i < network_plan->path_limits.poses.size(); ++i)
  {
    const arti_nav_core_msgs::Point2DWithLimits& p0 = network_plan->path_limits.poses[i].point;
    lower_left_bound_.x = std::min(lower_left_bound_.x, p0.x.value);
    lower_left_bound_.y = std::min(lower_left_bound_.y, p0.y.value);
    upper_right_bound_.x = std::max(upper_right_bound_.x, p0.x.value);
    upper_right_bound_.y = std::max(upper_right_bound_.y, p0.y.value);

    if ((i + 1) < network_plan->path_limits.poses.size())
    {
      // Interpolate between adjacent points:
      const arti_nav_core_msgs::Point2DWithLimits& p1 = network_plan->path_limits.poses[i + 1].point;
      const double dx = p1.x.value - p0.x.value;
      const double dy = p1.y.value - p0.y.value;
      const double point_distance = std::hypot(dx, dy);
      const double cosp = dx / point_distance;
      const double sinp = dy / point_distance;

      for (double d = 0; d < point_distance; d += buffer_.getResolution() / 2.0)
      {
        geometry_msgs::Point new_point;
        new_point.x = p0.x.value + cosp * d;
        new_point.y = p0.y.value + sinp * d;
        current_point_set_.addPoint(new_point);
      }
    }
    else
    {
      geometry_msgs::Point new_point;
      new_point.x = p0.x.value;
      new_point.y = p0.y.value;
      current_point_set_.addPoint(new_point);
    }
  }

  kd_tree_.emplace(2, current_point_set_, nanoflann::KDTreeSingleIndexAdaptorParams());
  kd_tree_->buildIndex();

  path_frame_ = network_plan->path_limits.header.frame_id;
}

unsigned char NetworkPlanLayer::calculateCostsFor(
  unsigned int x, unsigned int y, const geometry_msgs::TransformStamped& transform)
{
  geometry_msgs::Point current_point, transformed_point;
  buffer_.mapToWorld(x, y, current_point.x, current_point.y);
  tf2::doTransform(current_point, transformed_point, transform);

  const double query_point[2] = {transformed_point.x, transformed_point.y};
  ResultSet result{config_.deviation_distance * config_.deviation_distance};
  kd_tree_->findNeighbors(result, query_point, nanoflann::SearchParams());

  return result.found_point ? costmap_2d::FREE_SPACE : costmap_2d::LETHAL_OBSTACLE;
}

}
