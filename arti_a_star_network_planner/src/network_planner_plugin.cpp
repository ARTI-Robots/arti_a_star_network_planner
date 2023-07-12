/*
Created by clemens on 30.03.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_a_star_network_planner/network_planner_plugin.h>
#include <arti_a_star_network_planner/edge.h>
#include <arti_graph_processing/a_star_algorithm.h>
#include <arti_graph_processing/floyd_warshall_algorithm.h>
#include <arti_graph_processing/edge.h>
#include <arti_graph_processing/graph.h>
#include <arti_graph_processing/vertex.h>
#include <arti_nav_core_utils/conversions.h>
#include <arti_nav_core_utils/transformations.h>
#include <arti_nav_core_utils/transformer.h>
#include <arti_ros_param/arti_ros_param.h>
#include <arti_ros_param/param.h>
#include <arti_ros_param/yaml.h>
#include <boost/filesystem.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>
#include <pluginlib/class_list_macros.h>
#include <stdexcept>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <utility>
#include <visualization_msgs/MarkerArray.h>

namespace arti_a_star_network_planner
{

void NetworkPlannerPlugin::initialize(std::string name, arti_nav_core::Transformer* transformer)
{
  nh_ = ros::NodeHandle("~/" + name);
  nh_processing_ = ros::NodeHandle("~/" + name+"_processing");

  graphs_file_path_ = nh_.param<std::string>("graphs_file_path", {});
  //graphs_file_path_ = nh_processing_.param<std::string>("graphs_file_path", {});
  cfg_server_.reset(
    new dynamic_reconfigure::Server<arti_a_star_network_planner::AStarNetworkPlannerConfig>(nh_));
  cfg_server_->setCallback(std::bind(&NetworkPlannerPlugin::reconfigure, this, std::placeholders::_1));

  graph_search_publisher_ = nh_processing_.advertise<visualization_msgs::MarkerArray>("navigation_network_search", 1, true);

  robot_information_ = std::make_shared<RobotInformation>(nh_);

  transformer_ = transformer;

  change_region_service_ = nh_.advertiseService("change_region_service", &NetworkPlannerPlugin::changeRegionCB, this);
  reload_networks_service_ = nh_.advertiseService("reload_networks", &NetworkPlannerPlugin::reloadNetworksCB, this);

  get_plan_service = nh_.advertiseService("get_network_plan", &NetworkPlannerPlugin::getPlanServiceCB, this);
}

void NetworkPlannerPlugin::reconfigure(const arti_a_star_network_planner::AStarNetworkPlannerConfig& new_config)
{
  cfg_ = new_config;

  graph_publisher_.emplace(nh_processing_, "navigation_network", cfg_.increase_factor, cfg_.max_number_increases);

  loadGraphs();

  edge_correction_ = std::make_shared<EdgeCorrection>(cfg_.max_number_increases, cfg_.increase_factor,
                                                      cfg_.edge_cost_validity_period_s);

  if (periodic_check_.isValid())
  {
    periodic_check_.setPeriod(ros::Duration(cfg_.edge_cost_reset_check_period_s));
  }
  else
  {
    periodic_check_ = nh_.createTimer(ros::Duration(cfg_.edge_cost_reset_check_period_s),
                                    &NetworkPlannerPlugin::checkResetEdgeCosts, this, false);
  }
}

void NetworkPlannerPlugin::checkResetEdgeCosts(const ros::TimerEvent& /*e*/)
{
  if (!graphs_file_path_.empty())
  {
    const std::time_t last_modification_time = getLastModificationTime(graphs_file_path_);
    if (graphs_file_last_modification_time_ < last_modification_time)
    {
      graphs_file_last_modification_time_ = last_modification_time;
      loadGraphs();
    }
  }

  if (current_graph_)
  {
    ROS_DEBUG("check current graph for edge_costs that need to be reset");
    const auto& edges_of_graph = current_graph_->getEdges();
    ros::Time current_time = ros::Time::now();

    for (const auto& edge : edges_of_graph)
    {
      edge_correction_->resetEdgeCostIfExpired(*std::dynamic_pointer_cast<arti_a_star_network_planner::Edge>(edge), current_time);
    }
    ROS_DEBUG("Done checking edges, publish graph");
    graph_publisher_->publish(*current_graph_unprocessed_);
    graph_publisher_->publish(*current_graph_, true);
  }
}

bool NetworkPlannerPlugin::setGoal(const arti_nav_core_msgs::Pose2DStampedWithLimits& goal)
{
  current_goal_ = transformPose(goal);
  return current_goal_.is_initialized();
}

arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum NetworkPlannerPlugin::makePlan(
  arti_nav_core_msgs::Movement2DGoalWithConstraints& plan)
{
  if (!current_graph_)
  {
    ROS_ERROR_STREAM("tried to make plan without a network");
    return arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::NO_PATH_POSSIBLE;
  }

  if (!current_goal_)
  {
    ROS_ERROR_STREAM("tried to make plan without a goal");
    return arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::NO_PATH_POSSIBLE;
  }

  const boost::optional<arti_nav_core_msgs::Pose2DStampedWithLimits> current_pose = getCurrentPose();
  if (!current_pose)
  {
    return arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::NO_PATH_POSSIBLE;
  }

  // take the closest vertex as the path gets later anyway optimized in optimizePath()
  const arti_graph_processing::VertexPtr start_vertex = getClosestVertex(*current_pose);

  const arti_graph_processing::VertexPtr goal_vertex = getClosestVertex(*current_goal_);

  const GraphPlan planner_path = arti_graph_processing::AStarAlgorithm::computePath(start_vertex, goal_vertex);

  // check if plan was empty
  if (planner_path.empty())
  {
    ROS_DEBUG_STREAM("Network planner output nothing...");
    return arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::NO_PATH_POSSIBLE;
  }

  ROS_DEBUG_STREAM("calling compute result in plan of size: " << planner_path.size());

  // optimize start and end node
  const auto optimized_planner_path = optimizePath(planner_path, *current_pose, *current_goal_);

  plan.path_limits.header.stamp = current_pose->header.stamp;
  plan.path_limits.header.frame_id = current_graph_->getFrameName();

  plan.path_limits.poses.reserve(optimized_planner_path.size() + 2);

  convertPath(optimized_planner_path, current_pose->pose, current_goal_->pose, plan.path_limits.poses);

  ROS_DEBUG_STREAM("path size: " << plan.path_limits.poses.size());

  /*
   * Interpolation of poses
   */
  if(cfg_.interpolate_resulting_path)
  {
    interpolatePath(plan.path_limits.poses);
  }

  ROS_DEBUG_STREAM("network planer result in plan: " << plan.path_limits);

  // Update goal because the goal position and orientation might have changed depending
  // on the configuration (e.g. when bidirectional_drive=true))
  plan.goal.pose.header = plan.path_limits.header;
  plan.goal.pose.pose = plan.path_limits.poses.back();

  publishSearchGraph(planner_path);

  return arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::PLAN_FOUND;
}

void NetworkPlannerPlugin::handlePlannerError(
  const arti_nav_core_msgs::Pose2DWithLimits& error_pose_a, const arti_nav_core_msgs::Pose2DWithLimits& error_pose_b)
{
  if (current_graph_ && edge_correction_)
  {
  /*
    const arti_graph_processing::VertexPtr start_vertex = getClosestVertex(current_graph_->getFrameName(),
                                                                           error_pose_a.point);
    const arti_graph_processing::VertexPtr goal_vertex = getClosestVertex(current_graph_->getFrameName(),
                                                                          error_pose_b.point);
*/
    arti_nav_core_msgs::Pose2DStampedWithLimits pose_a;
    pose_a.pose = error_pose_a;
    pose_a.header.frame_id = "map";
    pose_a.header.stamp = ros::Time::now();
    arti_nav_core_msgs::Pose2DStampedWithLimits pose_b;
    pose_b.pose = error_pose_b;
    pose_b.header.frame_id = "map";
    pose_b.header.stamp = ros::Time::now();
    auto pose_a_trans = transformPose(pose_a);
    auto pose_b_trans = transformPose(pose_b);
    const arti_graph_processing::VertexPtr start_vertex = getClosestVertex(current_graph_->getFrameName(),
                                                                           pose_a_trans->pose.point);
    const arti_graph_processing::VertexPtr goal_vertex = getClosestVertex(current_graph_->getFrameName(),
                                                                          pose_b_trans->pose.point);
                                                                          
    const GraphPlan planner_path = arti_graph_processing::AStarAlgorithm::computePath(start_vertex, goal_vertex);

    for (const auto& path_segment : planner_path)
    {
      if (path_segment.second)  // Last segment has no edge, only goal vertex
      {
        const auto edge = std::dynamic_pointer_cast<Edge>(path_segment.second);
        if (edge)
        {
          ros::Time current_time = ros::Time::now();
          const double cost_before = edge->getCosts();
          edge_correction_->increaseEdgeCosts(*edge, current_time);
          ROS_WARN_STREAM(
            "increaseEdgeCosts: edge: " << edge->getSource()->getName() << ", cost_before = " << cost_before
                                        << ", cost_after = " << edge->getCosts());

          // also increase cost of edge in opposite direction
          if (!edge->getDestination())
          {
            continue;
          }

          const auto& edges_from_dest = edge->getDestination()->getOutgoingEdges();

          for (const auto& dest_edge : edges_from_dest)
          {
            const arti_graph_processing::VertexPtr dest_edge_destination = dest_edge->getDestination();

            if (dest_edge_destination == edge->getSource())
            {
              const auto my_edge = std::dynamic_pointer_cast<Edge>(dest_edge);
              const double my_cost_before = my_edge->getCosts();
              edge_correction_->increaseEdgeCosts(*my_edge, current_time);
              ROS_WARN_STREAM(
                "increaseEdgeCosts , edge " << my_edge->getSource()->getName() << " cost_before = " << my_cost_before
                                            << ", cost_after = " << my_edge->getCosts());
            }
          }
        }
        else
        {
          ROS_FATAL_STREAM("edge has wrong type, this should never happen");
        }
      }
    }

    graph_publisher_->publish(*current_graph_unprocessed_);
    graph_publisher_->publish(*current_graph_, true);
  }
}

void NetworkPlannerPlugin::loadGraphs()
{
  graph_mappings_.clear();

  GraphLoader graph_loader;

  if (!graphs_file_path_.empty())
  {
    ROS_INFO_STREAM("loading graphs from file");
    if (graphs_file_last_modification_time_ == 0)
    {
      graphs_file_last_modification_time_ = getLastModificationTime(graphs_file_path_);
    }

    const arti_ros_param::RootParam root_param = arti_ros_param::loadYaml(graphs_file_path_.string());
    if (root_param.exists())
    {
      const arti_ros_param::Param graphs_param = root_param["graphs"];
      graph_mappings_ = graph_loader.loadGraphMap(graphs_param.exists() ? graphs_param : root_param);
    }
    else
    {
      ROS_ERROR_STREAM("failed to load graphs file '" << graphs_file_path_ << "'");
    }
  }
  else
  {
    ROS_INFO_STREAM("loading graphs from parameter server");
    graph_mappings_ = graph_loader.loadGraphMap(arti_ros_param::RootParam{ros::NodeHandle{nh_, "graphs"}});
  }

  if (graph_mappings_.empty())
  {
    ROS_ERROR_STREAM("started network planner without a network");
    current_graph_.reset();
    current_graph_unprocessed_.reset();
    graph_publisher_->publish(arti_graph_processing::Graph{{}, "map"});
    graph_publisher_->publish(arti_graph_processing::Graph{{}, "map"}, true);
  }
  else
  {
    // If a current graph was set before, try to choose the graph of the same name:
    if (current_graph_unprocessed_)
    {
      current_graph_unprocessed_ = getGraph(current_graph_unprocessed_->getName());
    }

    // If no current graph was set before, or there was no graph of the same name, select one of the loaded graphs:
    if (!current_graph_unprocessed_)
    {
      current_graph_unprocessed_ = graph_mappings_.begin()->second;
    }

    if(cfg_.interpolate_graph && current_graph_unprocessed_)
    {
      current_graph_ = graph_loader.interpolateGraph(current_graph_unprocessed_, cfg_
      .max_edge_distance, arti_ros_param::RootParam{ros::NodeHandle{nh_ , "graphs"}});

      ROS_INFO("publish interpolated graph now");
      graph_publisher_->publish(*current_graph_, true);
    }
    else
    {
      current_graph_ = current_graph_unprocessed_;
    }
    //arti_graph_processing::FloydWarshallAlgorithm::computeDistanceValues(*current_graph_);


    graph_publisher_->publish(*current_graph_unprocessed_);
    ROS_DEBUG("graph publishing done!");
  }
}

arti_graph_processing::GraphPtr NetworkPlannerPlugin::getGraph(const std::string& name) const
{
  const auto graph_mapping_it = graph_mappings_.find(name);
  if (graph_mapping_it != graph_mappings_.end())
  {
    return graph_mapping_it->second;
  }
  return nullptr;
}

NetworkPlannerPlugin::GraphPlan NetworkPlannerPlugin::optimizePath(
  const GraphPlan planner_path, const arti_nav_core_msgs::Pose2DStampedWithLimits& start,
  const arti_nav_core_msgs::Pose2DStampedWithLimits& goal) const
{
  if (planner_path.empty())
  {
    return planner_path;
  }

  if (!cfg_.skip_network_if_close ||
        cfg_.max_nodes_to_skip <= 0 ||
        cfg_.skip_network_path_length <= 0.0)
  {
    return planner_path;
  }

  // handle trivial case where you directly go from the start to the goal pose
  const double distance_from_start_to_goal_via_path = calculateDistance(planner_path.begin()->first->getPose().pose, start.pose) + 
        calculateDistanceAlongPath(planner_path) + 
        calculateDistance(planner_path.rbegin()->first->getPose().pose, goal.pose);
  
  if (planner_path.size() <= static_cast<size_t>(cfg_.max_nodes_to_skip) &&
      distance_from_start_to_goal_via_path <= cfg_.skip_network_path_length)
  {
    ROS_INFO_STREAM("Directly going from start to end is shorter and allowed: direct distance from start to goal: " << 
      calculateDistance(start.pose, goal.pose) << "; distance from start to goal via path: " << distance_from_start_to_goal_via_path);
    return GraphPlan();
  }

  auto optimized_path = planner_path;

  // optimize start point on graph
  double distance_from_start_optimized = calculateDistance(optimized_path.begin()->first->getPose().pose, start.pose);
  double distance_from_start_unoptimized = distance_from_start_optimized;
  int skipped_nodes_start = 0;

  for (auto if_skip_node = optimized_path.begin();
      if_skip_node != optimized_path.end() && if_skip_node->second;
      ++if_skip_node)
  {
    const double distance_to_next_node = if_skip_node->first->calculateEuclideanDistanceTo(*(if_skip_node->second->getDestination()));
    distance_from_start_unoptimized += distance_to_next_node;

    if (distance_from_start_unoptimized > cfg_.skip_network_path_length)
    {
      break;
    }

    const double distance_if_skipping_this_node = calculateDistance(if_skip_node->second->getDestination()->getPose().pose, start.pose);
    const double distance_without_skipping_this_node = distance_from_start_optimized + distance_to_next_node;

    if (distance_if_skipping_this_node < distance_without_skipping_this_node)
    {
      distance_from_start_optimized = distance_if_skipping_this_node;
      skipped_nodes_start++;

      ROS_INFO_STREAM("Erasing element " << skipped_nodes_start <<
                      " from start because shorter going directly to the next node: distance from start optimized: " <<
                      distance_from_start_optimized <<
                      "; distance from start without skipping this node: " << distance_without_skipping_this_node);

      if (skipped_nodes_start >= cfg_.max_nodes_to_skip)
      {
        break;
      }
    }
    else
    {
      distance_from_start_optimized += distance_to_next_node;
    }
  }

  optimized_path.erase(optimized_path.begin(), optimized_path.begin() + skipped_nodes_start);


  // optimize end point on graph
  double distance_from_goal_optimized = calculateDistance(optimized_path.rbegin()->first->getPose().pose, goal.pose);
  double distance_from_goal_unoptimized = distance_from_goal_optimized;
  int skipped_nodes_goal = 0;

  for (auto if_skip_node = optimized_path.rbegin(), if_skip_node_prev = std::next(optimized_path.rbegin());
      if_skip_node != optimized_path.rend() && if_skip_node_prev != optimized_path.rend();
      ++if_skip_node, ++if_skip_node_prev)
  {
    const double distance_to_previous_node = if_skip_node->first->calculateEuclideanDistanceTo(*(if_skip_node_prev->first));
    distance_from_goal_unoptimized += distance_to_previous_node;

    if (distance_from_goal_unoptimized > cfg_.skip_network_path_length)
    {
      break;
    }

    const double distance_if_skipping_this_node = calculateDistance(if_skip_node_prev->first->getPose().pose, goal.pose);
    const double distance_without_skipping_this_node = distance_from_goal_optimized + distance_to_previous_node;

    if (distance_if_skipping_this_node < distance_without_skipping_this_node)
    {
      distance_from_goal_optimized = distance_if_skipping_this_node;
      skipped_nodes_goal++;

      ROS_INFO_STREAM("Erasing element " << skipped_nodes_goal <<
                      " from end because shorter going directly to the previous node: distance from goal optimized: " <<
                      distance_from_goal_optimized <<
                      "; distance from goal without skipping this node: " << distance_without_skipping_this_node);

      if (skipped_nodes_goal >= cfg_.max_nodes_to_skip)
      {
        break;
      }
    }
    else
    {
      distance_from_goal_optimized += distance_to_previous_node;
    }
  }

  optimized_path.erase(optimized_path.begin() + (optimized_path.size() - skipped_nodes_goal), optimized_path.end());

  ROS_INFO_STREAM("Nodes in path after optimization (excluding start and end pose): " << optimized_path.size());

  return optimized_path;
}

boost::optional<arti_nav_core_msgs::Pose2DStampedWithLimits> NetworkPlannerPlugin::transformPose(
  const arti_nav_core_msgs::Pose2DStampedWithLimits& pose) const
{
  if (!transformer_ || !current_graph_)
  {
    ROS_ERROR_STREAM("planner not initialized");
    return boost::none;
  }

  std::string tf_error_message;
  const auto transform = arti_nav_core_utils::tryToLookupTransform(*transformer_, current_graph_->getFrameName(),
                                                                   pose.header.frame_id, pose.header.stamp,
                                                                   ros::Duration(1.0), &tf_error_message);
  if (!transform)
  {
    ROS_ERROR_STREAM("failed to transform pose: " << tf_error_message);
    return boost::none;
  }

  return arti_nav_core_utils::transformPose(pose, *transform);
}

boost::optional<arti_nav_core_msgs::Pose2DStampedWithLimits> NetworkPlannerPlugin::getCurrentPose() const
{
  if (!robot_information_ || !transformer_ || !current_graph_)
  {
    ROS_ERROR_STREAM("planner not initialized");
    return boost::none;
  }

  std::lock_guard<RobotInformation> lock(*robot_information_);
  const geometry_msgs::PoseStamped robot_pose = robot_information_->getRobotPose();

  std::string tf_error_message;
  const auto transform = arti_nav_core_utils::tryToLookupTransform(*transformer_, current_graph_->getFrameName(),
                                                                   robot_pose.header.frame_id, robot_pose.header.stamp,
                                                                   ros::Duration(5.), &tf_error_message);
  if (!transform)
  {
    ROS_ERROR_STREAM("failed to transform robot pose: " << tf_error_message);
    return boost::none;
  }

  geometry_msgs::PoseStamped pose_out;
  tf2::doTransform(robot_pose, pose_out, *transform);
  return convertPose(pose_out, false);
}



double NetworkPlannerPlugin::calculateDistanceAlongPath(const GraphPlan path)
{
  double distance = 0;
  for (auto &&path_node : path)
  {
    if (path_node.first && path_node.second && path_node.second->getDestination())
    {
      distance += calculateDistance(path_node.first->getPose().pose,path_node.second->getDestination()->getPose().pose);
    }
  }

  return distance;
}

double NetworkPlannerPlugin::calculateDistance(
  const geometry_msgs::Pose& a_pose, const geometry_msgs::Pose& b_pose)
{
  return std::hypot(a_pose.position.x - b_pose.position.x, a_pose.position.y - b_pose.position.y);
}

double NetworkPlannerPlugin::calculateDistance(
  const geometry_msgs::Pose& a_pose, const arti_nav_core_msgs::Pose2DWithLimits& b_pose)
{
  return std::hypot(a_pose.position.x - b_pose.point.x.value, a_pose.position.y - b_pose.point.y.value);
}

double NetworkPlannerPlugin::calculateDistance(
  const arti_nav_core_msgs::Pose2DWithLimits& a_pose, const arti_nav_core_msgs::Pose2DWithLimits& b_pose)
{
  return std::hypot(a_pose.point.x.value - b_pose.point.x.value, a_pose.point.y.value - b_pose.point.y.value);
}

arti_graph_processing::VertexPtr NetworkPlannerPlugin::getClosestVertex(
  const arti_nav_core_msgs::Pose2DStampedWithLimits& pose) const
{
  if (!current_graph_)
  {
    return nullptr;
  }

  return current_graph_->getClosestVertex(arti_nav_core_utils::convertToPointStamped(pose.header, pose.pose.point));
}

arti_graph_processing::VertexPtr NetworkPlannerPlugin::getClosestVertex(
  const std::string& frame_id, const arti_nav_core_msgs::Point2DWithLimits& point) const
{
  if (!current_graph_)
  {
    return nullptr;
  }

  std_msgs::Header header;
  header.frame_id = frame_id;
  return current_graph_->getClosestVertex(arti_nav_core_utils::convertToPointStamped(header, point));
}

void NetworkPlannerPlugin::convertPath(
    const GraphPlan& planner_path,
    const arti_nav_core_msgs::Pose2DWithLimits& start_pose,
    const arti_nav_core_msgs::Pose2DWithLimits& goal_pose,
    std::vector<arti_nav_core_msgs::Pose2DWithLimits>& path) const
{
  // if there is no planned path, then just add the start and goal pose
  if (planner_path.empty())
  {
    path.push_back(start_pose);
    path.push_back(goal_pose);

    if (cfg_.bidirectional_drive)
    {
      // if delta between start and end pose is bigger than 90 degree, drive backwards
      double delta_theta = tfNormalizeAngle(goal_pose.theta.value - start_pose.theta.value);
      if (std::abs(delta_theta) > M_PI_2)
      {
        ROS_INFO_STREAM("Engage reverse Network drive!");
        path.back().theta.value = tfNormalizeAngle(goal_pose.theta.value + M_PI);
      }

    }

    return;
  }

  bool reverse = false;
  // check for first element, then just keep driving direction
  if (cfg_.bidirectional_drive)
  {

    arti_nav_core_msgs::Pose2DWithLimits p1 = convertPose(arti_nav_core_utils::convertToPose(start_pose),
                                                          planner_path.front().first->getPose().pose,
                                                          false);
    // if delta between start and end pose is bigger than 90 degree, drive backwards
    double delta_theta = tfNormalizeAngle(p1.theta.value - start_pose.theta.value);
    if (std::abs(delta_theta) > M_PI_2)
    {
      ROS_INFO_STREAM("Engage reverse Network drive!");
      reverse = true;
    }
  }

  path.push_back(start_pose);

  for (const auto& path_segment : planner_path)
  {
    //ROS_DEBUG_STREAM("path_segment.first->getName(): " << path_segment.first->getName());
    if (path_segment.second)
    {
      //ROS_DEBUG_STREAM("path_segment.second->getDestination().getName(): " << path_segment.second->getDestination()->getName());

      path.push_back(convertPose(path_segment.first->getPose().pose,
                                 path_segment.second->getDestination()->getPose().pose, reverse));
    }
    else
    {
      auto pose = path_segment.first->getPose().pose;

      // If no edge is existing (i.e. the last element), then take the orientation from the edge of the previous node to this node
      // Note that you also cannot just take the orientation from the previous node because
      // this might be the starting node with whatever orientation the current pose has.

      double dx = pose.position.x - path.back().point.x.value;
      double dy = pose.position.y - path.back().point.y.value;

      double theta = std::atan2(dy, dx);

      pose.orientation = tf::createQuaternionMsgFromYaw(theta);

      path.push_back(convertPose(pose, reverse));
    }

  }

  // use current goal orientation for last pose
  // add the additional goal pose only if the goal position differs from the last network pose, otherwise just directly use the goal pose
  if (calculateDistance(goal_pose, path.back()) > 0.)
  {
    path.push_back(goal_pose);

    updateOrientationBetweenLastPoses(path, reverse);
    ROS_DEBUG_STREAM("Debug theta second to last: " << path.end()[-2].theta << " |  last:"
                                                    << path.end()[-1].theta);
  }
  else
  {
    path.back() = current_goal_->pose;
  }
}

void NetworkPlannerPlugin::interpolatePath(
  std::vector<arti_nav_core_msgs::Pose2DWithLimits>& path) const
{

  for(ssize_t i = 0; i+1 < path.size(); i++)
  {
    auto start = path.at(i);
    auto stop = path.at(i+1);

    double dx = stop.point.x.value - start.point.x.value;
    double dy = (stop.point.y.value - start.point.y.value);
    double distance = std::hypot(dx , dy);

    if(distance > cfg_.max_path_distance)
    {
      int div = std::ceil(distance/cfg_.max_path_distance);
      ROS_DEBUG("Start Interpolation at i: %zu, with [x/y]: [%f/%f], distance: %f", i, start.point.x.value, start.point
      .y.value, distance);
      for(int j=1; j < div; j++)
      {
        arti_nav_core_msgs::Pose2DWithLimits pose;
        pose.point.x.lower_limit = start.point.x.lower_limit;
        pose.point.x.upper_limit = start.point.x.upper_limit;
        pose.point.x.has_limits = start.point.x.has_limits;
        pose.point.y.lower_limit = start.point.y.lower_limit;
        pose.point.y.upper_limit = start.point.y.upper_limit;
        pose.point.y.has_limits = start.point.y.has_limits;
        pose.theta.value = start.theta.value;
        pose.theta.has_limits = start.theta.has_limits;
        pose.theta.lower_limit = start.theta.lower_limit;
        pose.theta.upper_limit = start.theta.upper_limit;

        pose.point.x.value = start.point.x.value + (float(j)*dx/float(div));
        pose.point.y.value = start.point.y.value + (float(j)*dy/float(div));
        auto it = path.begin();
        path.insert(std::next(it,(i+j)),pose);
      }
      i += (div-1);
      for(int j=1; j<= div; j++)
      {
        ROS_INFO("Interpolation add point at [x/y] : [%f/%f]", path.at(i-div+j).point.x.value, path.at(i+j-div).point.y
        .value);
      }
      ROS_DEBUG("Path value after inserting [%d] poses, i: %d", div-1, i);
    }
    ROS_DEBUG("new path pose size: %zu", path.size());
  }
  ROS_DEBUG("Finished path interpolation");

}


double NetworkPlannerPlugin::calculatePathLength(
  const GraphPlan& path,
  const arti_nav_core_msgs::Pose2DStampedWithLimits& start,
  const arti_nav_core_msgs::Pose2DStampedWithLimits& goal)
{
  double result = 0.;
  bool first_segment = true;
  for (const auto& path_segment : path)
  {
    if (path_segment.second && path_segment.second->getSource() && path_segment.second->getDestination())
    {
      //segments within the path used to define the traversed edges along the network
      const auto source = path_segment.second->getSource();

      const auto destination = path_segment.second->getDestination();

      result += source->calculateEuclideanDistanceTo(*destination);
      ROS_DEBUG_STREAM("distance of segment " << source->calculateEuclideanDistanceTo(*destination));
    }
    else
    {
      //we are at the end add distance to goal vertex
      result += calculateDistance(path_segment.first->getPose().pose, goal.pose);
      ROS_DEBUG_STREAM("distance to goal vertex " << calculateDistance(path_segment.first->getPose().pose, goal.pose));
    }

    if (first_segment)
    {
      //first segment add distance to start
      result += calculateDistance(path_segment.first->getPose().pose, start.pose);
      ROS_DEBUG_STREAM(
        "distance to start vertex " << calculateDistance(path_segment.first->getPose().pose, start.pose));
      first_segment = false;
    }
  }

  return result;
}

arti_nav_core_msgs::Pose2DStampedWithLimits NetworkPlannerPlugin::convertPose(
  const geometry_msgs::PoseStamped& pose, const geometry_msgs::PoseStamped& destination_pose, const bool reverse) const
{
  arti_nav_core_msgs::Pose2DStampedWithLimits result;
  result.pose = convertPose(pose.pose, destination_pose.pose, reverse);
  result.header = pose.header;

  return result;
}

arti_nav_core_msgs::Pose2DWithLimits NetworkPlannerPlugin::convertPose(
  const geometry_msgs::Pose& pose, const geometry_msgs::Pose& destination_pose, bool reverse) const
{
  arti_nav_core_msgs::Pose2DWithLimits result;
  result.point.x.value = pose.position.x;
  result.point.x.has_limits = true;
  result.point.x.lower_limit = -cfg_.corridor_width;
  result.point.x.upper_limit = cfg_.corridor_width;

  result.point.y.value = pose.position.y;
  result.point.y.has_limits = true;
  result.point.y.lower_limit = -cfg_.corridor_width;
  result.point.y.upper_limit = cfg_.corridor_width;

  // use direction from pose to destination pose to get yaw/theta
  double dx = destination_pose.position.x - pose.position.x;
  double dy = destination_pose.position.y - pose.position.y;
  double theta = std::atan2(dy, dx);
  if (reverse)
  {
    theta = tfNormalizeAngle(theta + M_PI);
  }
  result.theta.value = theta;
  // Orientation is preferred but not enforced:
  result.theta.has_limits = true;
  result.theta.lower_limit = -cfg_.theta_limit;
  result.theta.upper_limit = +cfg_.theta_limit;

  return result;
}

void NetworkPlannerPlugin::updateOrientationBetweenLastPoses(std::vector<arti_nav_core_msgs::Pose2DWithLimits>& poses, const bool reverse) const
{
  // don't modify orientation if only start and end position (no in-between poses)
  if (poses.size() <= 2)
  {
    return;
  }

  double x1 = poses.end()[-2].point.x.value;
  double y1 = poses.end()[-2].point.y.value;

  double x2 = poses.end()[-1].point.x.value;
  double y2 = poses.end()[-1].point.y.value;

  // use direction from pose to destination pose to get yaw/theta
  double dx = x2 - x1;
  double dy = y2 - y1;
  double theta_to_goal = std::atan2(dy, dx);

  double t2 = poses.end()[-1].theta.value;

  if (cfg_.bidirectional_drive)
  {
    // if we're driving in backwards direction, we remain in that direction.
    if (reverse)
    {
      theta_to_goal = tfNormalizeAngle(theta_to_goal + M_PI);
    }

    // if delta between the new second last and last orientation is bigger than 90 degree,
    // we're driving in a different direction than the end pose and we should rotate the end pose
    if (std::abs(tfNormalizeAngle(theta_to_goal - t2)) > M_PI_2)
    {
      t2 = tfNormalizeAngle(t2 + M_PI);
      ROS_INFO_STREAM("rotate end pose 180 Degree!");
      ROS_DEBUG_STREAM(" t1: \n [ " << poses.end()[-1].theta << " ] \n theta new: " << t2);
      poses.end()[-1].theta.value = t2;
    }
  }

  poses.end()[-1].theta.lower_limit = -cfg_.theta_limit;
  poses.end()[-1].theta.upper_limit = +cfg_.theta_limit;
  poses.end()[-2].theta.value = theta_to_goal;
}

arti_nav_core_msgs::Pose2DStampedWithLimits NetworkPlannerPlugin::convertPose(
  const geometry_msgs::PoseStamped& pose, const bool reverse) const
{
  arti_nav_core_msgs::Pose2DStampedWithLimits result;
  result.pose = convertPose(pose.pose, reverse);
  result.header = pose.header;

  return result;
}

arti_nav_core_msgs::Pose2DWithLimits NetworkPlannerPlugin::convertPose(
  const geometry_msgs::Pose& pose, const bool reverse) const
{
  arti_nav_core_msgs::Pose2DWithLimits result;
  result.point.x.value = pose.position.x;
  result.point.x.has_limits = true;
  result.point.x.lower_limit = -cfg_.corridor_width;
  result.point.x.upper_limit = cfg_.corridor_width;

  result.point.y.value = pose.position.y;
  result.point.y.has_limits = true;
  result.point.y.lower_limit = -cfg_.corridor_width;
  result.point.y.upper_limit = cfg_.corridor_width;

  result.theta.value = tf::getYaw(pose.orientation);
  if (reverse)
  {
    result.theta.value = tfNormalizeAngle(result.theta.value + M_PI);
  }

  result.theta.has_limits = true;
  result.theta.lower_limit = -cfg_.theta_limit;
  result.theta.upper_limit = cfg_.theta_limit;

  return result;
}

bool NetworkPlannerPlugin::changeRegionCB(
  arti_move_base_msgs::ChangeRegion::Request& request, arti_move_base_msgs::ChangeRegion::Response& /*response*/)
{
  const auto new_graph = getGraph(request.region_name);
  if (new_graph)
  {

    if(cfg_.interpolate_graph)
    {
      const arti_ros_param::RootParam root_param = arti_ros_param::loadYaml(graphs_file_path_.string());
      if (root_param.exists()) {
        GraphLoader graph_loader;
        current_graph_unprocessed_ = new_graph;
        current_graph_ = graph_loader.interpolateGraph(current_graph_unprocessed_, cfg_.max_edge_distance, root_param);
      }
      else{
        ROS_ERROR("loading interpolated path fail because of missing root param");
      }
    }
    else
    {
      current_graph_ = new_graph;
    }
    graph_publisher_->publish(*current_graph_unprocessed_);
    graph_publisher_->publish(*current_graph_, true);

    return true;
  }
  else
  {
    ROS_ERROR_STREAM("cannot change region to '" << request.region_name << "'");
    return false;
  }
}

bool NetworkPlannerPlugin::reloadNetworksCB(
  std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response& /*response*/)
{
  loadGraphs();
  return true;
}

bool NetworkPlannerPlugin::getPlanServiceCB(arti_move_base_msgs::GetNetworkPlan::Request& request, arti_move_base_msgs::GetNetworkPlan::Response& response)
{
  if(current_graph_)
  {

    //arti_nav_core_utils::convert

    arti_nav_core_msgs::Pose2DStampedWithLimits start_2d = convertPose(request.start, false);
    arti_nav_core_msgs::Pose2DStampedWithLimits target_2d = convertPose(request.target, false);
    // Transform to path frame

    auto temp = transformPose(start_2d);
    auto temp2 = transformPose(target_2d);
    if( temp && temp2)
    {
      start_2d = *temp;
      target_2d = *temp2;
    }
    else {
      ROS_ERROR("no transform for start/target Pose");
      return false;
    }

    // take the closest vertex as the path gets later anyway optimized in optimizePath()
    const arti_graph_processing::VertexPtr start_vertex = getClosestVertex(start_2d);
    const arti_graph_processing::VertexPtr goal_vertex = getClosestVertex(target_2d);


    if(start_vertex && goal_vertex)
    {
      const GraphPlan planner_path = arti_graph_processing::AStarAlgorithm::computePath(start_vertex, goal_vertex);
      if (planner_path.empty())
      {
        ROS_DEBUG_STREAM("Network planner output empty!!!");
//        return arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::NO_PATH_POSSIBLE;
        return false;
      }
//      const auto optimized_planner_path = optimizePath(planner_path, *current_pose, *current_goal_);

      arti_nav_core_msgs::Movement2DGoalWithConstraints plan;
      plan.path_limits.header.stamp = start_2d.header.stamp;
      plan.path_limits.header.frame_id = current_graph_->getFrameName();

      plan.path_limits.poses.reserve(planner_path.size() + 2);

//      arti_nav_core_utils::convertToPose2D()
      convertPath(planner_path, start_2d.pose, target_2d.pose, plan.path_limits.poses);
      // TODO add publisher to seperated topic?
//      plan.path_limits.poses
      nav_msgs::Path out = arti_nav_core_utils::convertToPath(plan.path_limits);
      response.path = out;
      return true;
    }
    else
    {
      ROS_ERROR("target or goal couldn't get a nearest node");
      return false;
    }
  }
  else
  {
    ROS_ERROR("No Graph loaded!!!");
    return false;
  }
}

void NetworkPlannerPlugin::publishSearchGraph(
  const std::vector<std::pair<arti_graph_processing::VertexPtr,
    arti_graph_processing::EdgePtr>>& planner_path)
{
  if (current_graph_)
  {
    visualization_msgs::MarkerArray graph_markers;
    visualization_msgs::Marker edges_marker;
    edges_marker.ns = "edges";
    edges_marker.id = 0;
    edges_marker.action = visualization_msgs::Marker::ADD;
//    edges_marker.type = visualization_msgs::Marker::LINE_LIST;
    //for arrow x is diameter y head diameter  If scale.z is not zero, it specifies the head length
    //for line_list only x is used, controls the width of the line
    edges_marker.type = visualization_msgs::Marker::ARROW;
    edges_marker.header.frame_id = current_graph_->getFrameName();
    edges_marker.header.stamp = ros::Time::now();
    edges_marker.color.r = 1;
    edges_marker.color.g = 0.0;
    edges_marker.color.b = 0.0;
    edges_marker.color.a = 1.;
    edges_marker.pose.orientation.w = 1.;
    edges_marker.scale.x = 0.1;
    edges_marker.scale.y = 0.1;
    edges_marker.scale.z = 0.001;

    edges_marker.points.reserve(planner_path.size() * 2);


    for (const auto& path_segment : planner_path)
    {
      if (!path_segment.second)
      {
        break;
      }

      const arti_graph_processing::VertexPtr source = path_segment.second->getSource();
      const arti_graph_processing::VertexPtr destination = path_segment.second->getDestination();

      if (source && destination)
      {
        edges_marker.points.push_back(source->getPose().pose.position);
        edges_marker.points.push_back(destination->getPose().pose.position);
        graph_markers.markers.push_back(edges_marker);
        edges_marker.id++;
        edges_marker.points.clear();
      }
    }


    graph_search_publisher_.publish(graph_markers);
  }
}

geometry_msgs::PoseStamped NetworkPlannerPlugin::getVertexPose(const arti_graph_processing::VertexPtr& vertex)
{
  geometry_msgs::PoseStamped pose = vertex->getPose();
  pose.header.stamp = ros::Time::now();
  return pose;
}

std::time_t NetworkPlannerPlugin::getLastModificationTime(const boost::filesystem::path& path)
{
  boost::system::error_code error_code;
  const std::time_t last_modification_time = boost::filesystem::last_write_time(path, error_code);
  if (error_code)
  {
    ROS_ERROR_STREAM(
      "could not get last modification time of '" << path << "': " << error_code << ": " << error_code.message());
    return 0;
  }
  return last_modification_time;
}


arti_graph_processing::EdgePtr NetworkPlannerPlugin::GraphLoader::loadEdge(
  const arti_graph_processing::Graph& /*graph*/, const arti_graph_processing::VertexPtr& source,
  const arti_graph_processing::VertexPtr& destination, double costs, const arti_ros_param::Param& /*root_param*/)
{
  return std::make_shared<Edge>(source, destination, costs);
}

}

PLUGINLIB_EXPORT_CLASS(arti_a_star_network_planner::NetworkPlannerPlugin, arti_nav_core::BaseNetworkPlanner)
