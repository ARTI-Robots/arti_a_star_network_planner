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

#include <arti_nav_core_costmap_2d_layers/network_layer.h>
#include <arti_graph_processing/edge.h>
#include <arti_graph_processing/graph.h>
#include <arti_graph_processing/graph_loader.h>
#include <arti_graph_processing/vertex.h>
#include <arti_nav_core_utils/transformer.h>
#include <functional>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(arti_nav_core_costmap_2d_layers::NetworkLayer, costmap_2d::Layer)

namespace arti_nav_core_costmap_2d_layers
{

NetworkLayer::NetworkLayer()
  : buffer_(0, 0, 1, 0, 0, costmap_2d::NO_INFORMATION)
{
}

NetworkLayer::~NetworkLayer() = default;

void NetworkLayer::onInitialize()
{
  node_handle_ = ros::NodeHandle("~/" + name_);
  current_ = true;

  config_server_.emplace(mutex_, node_handle_);
  config_server_->setCallback(std::bind(&NetworkLayer::reconfigure, this, std::placeholders::_1));
  plan_subscriber_ = node_handle_.subscribe("plan", 1, &NetworkLayer::processPlan, this);
}

void NetworkLayer::matchSize()
{
  Lock lock{mutex_};
  if (layered_costmap_)
  {
    const costmap_2d::Costmap2D* const costmap = layered_costmap_->getCostmap();
    if (costmap && costmap->getSizeInCellsX() != 0 && costmap->getSizeInCellsY() != 0)
    {
      buffer_.resizeMap(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), costmap->getResolution(),
                        costmap->getOriginX(), costmap->getOriginY());
    }
  }
  update_pending_ = true;
}

void NetworkLayer::reconfigure(const NetworkLayerConfig& config)
{
  if (!enabled_ && config.enabled)
  {
    matchSize();
  }

  const bool load_network = !network_kd_tree_ || config_.network_param_path != config.network_param_path;
  if (config_.network_deviation_distance != config.network_deviation_distance
      || config_.plan_deviation_distance != config.plan_deviation_distance)
  {
    resetBuffer();
    update_pending_ = true;
  }

  enabled_ = config.enabled;
  config_ = config;

  if (load_network)
  {
    loadNetwork();
  }
}

void NetworkLayer::loadNetwork()
{
  network_kd_tree_.reset();
  network_point_set_.clear();
  resetBuffer();
  network_lower_left_bound_.x = std::numeric_limits<double>::max();
  network_lower_left_bound_.y = std::numeric_limits<double>::max();
  network_upper_right_bound_.x = -std::numeric_limits<double>::max();
  network_upper_right_bound_.y = -std::numeric_limits<double>::max();

  // Load networks parameter; name resolving is necessary in case parameter name starts with "~":
  const XmlRpc::XmlRpcValue networks
    = node_handle_.param<XmlRpc::XmlRpcValue>(ros::names::resolve(config_.network_param_path), {});
  if (!(networks.getType() == XmlRpc::XmlRpcValue::TypeArray && networks.size() >= 1))
  {
    ROS_ERROR_STREAM(
      "failed to load graph from parameter '" << config_.network_param_path << "' because it has wrong type or size");
    return;
  }

  arti_graph_processing::GraphLoader graph_loader;
  const arti_graph_processing::GraphPtr network = graph_loader.loadGraph(networks[0]);
  network_frame_ = network->getFrameName();

  for (const arti_graph_processing::EdgePtr& edge : network->getEdges())
  {
    // Interpolate along edges:
    const geometry_msgs::Point& p0 = edge->getSource()->getPose().pose.position;
    const geometry_msgs::Point& p1 = edge->getDestination()->getPose().pose.position;

    network_lower_left_bound_.x = std::min(network_lower_left_bound_.x, p0.x);
    network_lower_left_bound_.y = std::min(network_lower_left_bound_.y, p0.y);
    network_upper_right_bound_.x = std::max(network_upper_right_bound_.x, p0.x);
    network_upper_right_bound_.y = std::max(network_upper_right_bound_.y, p0.y);

    network_lower_left_bound_.x = std::min(network_lower_left_bound_.x, p1.x);
    network_lower_left_bound_.y = std::min(network_lower_left_bound_.y, p1.y);
    network_upper_right_bound_.x = std::max(network_upper_right_bound_.x, p1.x);
    network_upper_right_bound_.y = std::max(network_upper_right_bound_.y, p1.y);

    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    const double point_distance = std::hypot(dx, dy);
    const double cosp = dx / point_distance;
    const double sinp = dy / point_distance;

    for (double d = 0; d < point_distance; d += buffer_.getResolution() / 2.0)
    {
      geometry_msgs::Point new_point;
      new_point.x = p0.x + cosp * d;
      new_point.y = p0.y + sinp * d;
      network_point_set_.addPoint(new_point);
    }
  }

  network_kd_tree_.emplace(2, network_point_set_, nanoflann::KDTreeSingleIndexAdaptorParams());
  network_kd_tree_->buildIndex();

  update_pending_ = true;
}

void NetworkLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double* min_x, double* min_y, double* max_x,
  double* max_y)
{
  Lock lock{mutex_};
  if (enabled_)
  {
    if (layered_costmap_->isRolling() && (buffer_.getOriginX() != layered_costmap_->getCostmap()->getOriginX()
                                          || buffer_.getOriginY() != layered_costmap_->getCostmap()->getOriginY()))
    {
      buffer_.updateOrigin(layered_costmap_->getCostmap()->getOriginX(), layered_costmap_->getCostmap()->getOriginY());
      update_pending_ = true;
    }

    if (update_pending_)
    {
      *min_x = std::min(*min_x, buffer_.getOriginX());
      *min_y = std::min(*min_y, buffer_.getOriginY());
      *max_x = std::max(*max_x, buffer_.getOriginX() + buffer_.getSizeInMetersX());
      *max_y = std::max(*max_y, buffer_.getOriginY() + buffer_.getSizeInMetersY());
    }
  }
}

//bool NetworkLayer::transformBounds(geometry_msgs::Point& transformed_ll, geometry_msgs::Point& transformed_ur)
//{
//  const std::string target_frame = layered_costmap_->getGlobalFrameID();
//  if (!tf_->waitForTransform(target_frame, network_frame_, ros::Time(), ros::Duration(1.0)))
//  {
//    ROS_ERROR_STREAM("transform failed: " << network_frame_);
//    return false;
//  }
//
//  geometry_msgs::PointStamped ll;
//  geometry_msgs::PointStamped lr;
//  geometry_msgs::PointStamped ul;
//  geometry_msgs::PointStamped ur;
//
//  geometry_msgs::PointStamped current_point;
//  current_point.header.frame_id = network_frame_;
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

void NetworkLayer::resetBuffer()
{
  buffer_.resetMap(0, 0, buffer_.getSizeInCellsX(), buffer_.getSizeInCellsY());
}

void NetworkLayer::updateCosts(
  costmap_2d::Costmap2D& master_grid, int /*min_x*/, int /*min_y*/, int /*max_x*/, int /*max_y*/)
{
  Lock lock{mutex_};
  if (enabled_ /*&& update_pending_*/)
  {
    //    ROS_INFO_STREAM("NetworkLayer::updateCosts: min_i: " << min_i << ", min_j: " << min_j << ", max_i: " << max_i
    //                                                             << ", max_j: " << max_j);
    std::string tf_error_message;
    boost::optional<geometry_msgs::TransformStamped> network_transform;
    boost::optional<geometry_msgs::TransformStamped> plan_transform;
    bool tried_to_get_network_transform{false};
    bool tried_to_get_plan_transform{false};
    for (unsigned int y = 0; y < buffer_.getSizeInCellsY(); ++y)
    {
      for (unsigned int x = 0; x < buffer_.getSizeInCellsX(); ++x)
      {
        //if (master_grid.getCost(x, y) != costmap_2d::NO_INFORMATION)
        {
          unsigned char current_costs = buffer_.getCost(x, y);
          if (current_costs == costmap_2d::NO_INFORMATION)
          {
            // If costs haven't been calculated for this cell yet, do so now:
            ResultSet::Result distance_to_network = ResultSet::Result::OUTSIDE;

            if (network_kd_tree_)
            {
              if (!network_transform && !tried_to_get_network_transform)
              {
                network_transform = arti_nav_core_utils::tryToLookupTransform(
                  *tf_, network_frame_, layered_costmap_->getGlobalFrameID(), ros::Time(), ros::Duration(1.0),
                  &tf_error_message);
                tried_to_get_network_transform = true;
                if (!network_transform)
                {
                  ROS_ERROR_STREAM(
                    "failed to get transform from '" << layered_costmap_->getGlobalFrameID() << "' to '"
                                                     << network_frame_ << "'");
                }
              }

              if (network_transform)
              {
                distance_to_network = getDistanceToNetwork(network_kd_tree_.value(), x, y, *network_transform,
                                                           config_.network_deviation_distance,
                                                           2.0 * buffer_.getResolution());
              }
            }

            if (distance_to_network != ResultSet::Result::INSIDE_CORRIDOR && plan_kd_tree_)
            {
              if (!plan_transform && !tried_to_get_plan_transform)
              {
                plan_transform = arti_nav_core_utils::tryToLookupTransform(
                  *tf_, plan_frame_, layered_costmap_->getGlobalFrameID(), ros::Time(), ros::Duration(1.0),
                  &tf_error_message);
                tried_to_get_plan_transform = true;
                if (!plan_transform)
                {
                  ROS_ERROR_STREAM(
                    "failed to get transform from '" << layered_costmap_->getGlobalFrameID() << "' to '"
                                                     << plan_frame_ << "'");
                }
              }

              if (plan_transform)
              {
                const ResultSet::Result distance_to_plan
                  = getDistanceToNetwork(plan_kd_tree_.value(), x, y, *plan_transform, config_.plan_deviation_distance,
                                         2.0 * buffer_.getResolution());
                if (distance_to_plan != ResultSet::Result::OUTSIDE && distance_to_network == ResultSet::Result::OUTSIDE)
                {
                  distance_to_network = distance_to_plan;
                }
              }
            }

            current_costs = (distance_to_network == ResultSet::Result::INSIDE_PADDING) ? costmap_2d::LETHAL_OBSTACLE
                                                                                       : costmap_2d::FREE_SPACE;
            buffer_.setCost(x, y, current_costs);
          }

          if (current_costs == costmap_2d::LETHAL_OBSTACLE)
          {
            master_grid.setCost(x, y, current_costs);
          }
        }
      }
    }

    update_pending_ = false;
  }
}

void NetworkLayer::reset()
{
  Lock lock{mutex_};
  resetBuffer();
  update_pending_ = true;
}

void NetworkLayer::processPlan(const arti_nav_core_msgs::Movement2DGoalWithConstraintsConstPtr& network_plan)
{
  Lock lock{mutex_};
  plan_kd_tree_.reset();
  plan_point_set_.clear();
  resetBuffer();
  plan_lower_left_bound_.x = std::numeric_limits<double>::max();
  plan_lower_left_bound_.y = std::numeric_limits<double>::max();
  plan_upper_right_bound_.x = -std::numeric_limits<double>::max();
  plan_upper_right_bound_.y = -std::numeric_limits<double>::max();

  for (size_t i = 0; i < network_plan->path_limits.poses.size(); ++i)
  {
    const arti_nav_core_msgs::Point2DWithLimits& p0 = network_plan->path_limits.poses[i].point;
    plan_lower_left_bound_.x = std::min(plan_lower_left_bound_.x, p0.x.value);
    plan_lower_left_bound_.y = std::min(plan_lower_left_bound_.y, p0.y.value);
    plan_upper_right_bound_.x = std::max(plan_upper_right_bound_.x, p0.x.value);
    plan_upper_right_bound_.y = std::max(plan_upper_right_bound_.y, p0.y.value);

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
        plan_point_set_.addPoint(new_point);
      }
    }
    else
    {
      geometry_msgs::Point new_point;
      new_point.x = p0.x.value;
      new_point.y = p0.y.value;
      plan_point_set_.addPoint(new_point);
    }
  }

  plan_kd_tree_.emplace(2, plan_point_set_, nanoflann::KDTreeSingleIndexAdaptorParams());
  plan_kd_tree_->buildIndex();

  plan_frame_ = network_plan->path_limits.header.frame_id;
  update_pending_ = true;
}

ResultSet::Result NetworkLayer::getDistanceToNetwork(
  const KDTree& kd_tree, unsigned int x, unsigned int y, const geometry_msgs::TransformStamped& transform,
  double max_distance, double padding)
{
  geometry_msgs::Point current_point, transformed_point;
  buffer_.mapToWorld(x, y, current_point.x, current_point.y);
  tf2::doTransform(current_point, transformed_point, transform);

  const double query_point[2] = {transformed_point.x, transformed_point.y};
  ResultSet result{max_distance, max_distance + padding};
  kd_tree.findNeighbors(result, query_point, nanoflann::SearchParams());
  return result.getResult();
}

}
