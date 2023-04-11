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

  graphs_file_path_ = nh_.param<std::string>("graphs_file_path", {});
  corridor_width_ = nh_.param<double>("corridor_width", 1.);
  const auto increase_factor = nh_.param<double>("increase_factor", 2.);
  const auto edge_cost_validity_period_s = nh_.param<double>("edge_cost_validity_period_s", 100.);
  const auto edge_cost_reset_check_period_s = nh_.param<double>("edge_cost_reset_check_period_s", 1.);
  const auto max_number_increases = nh_.param<int>("max_number_increases", 100.);
  bidirectional_drive_ = nh_.param<bool>("bidirectional_drive", false);
  skip_network_if_close_ = nh_.param<bool>("skip_network_if_close", false);
  skip_network_path_length_ = nh_.param<double>("skip_network_path_length", 1.);
  max_nodes_to_skip_ = static_cast<uint64_t>(nh_.param<int>("max_nodes_to_skip", 1));

  graph_publisher_.emplace(nh_, "navigation_network", increase_factor, max_number_increases);

  loadGraphs();

  graph_search_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("navigation_network_search", 1, true);

  start_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("start", 1, true);
  start_vertex_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("start_vertex", 1, true);
  goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal", 1, true);
  goal_vertex_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal_vertex", 1, true);

  robot_information_ = std::make_shared<RobotInformation>(nh_);

  transformer_ = transformer;

  edge_correction_ = std::make_shared<EdgeCorrection>(max_number_increases, increase_factor,
                                                      edge_cost_validity_period_s);

  change_region_service_ = nh_.advertiseService("change_region_service", &NetworkPlannerPlugin::changeRegionCB, this);
  reload_networks_service_ = nh_.advertiseService("reload_networks", &NetworkPlannerPlugin::reloadNetworksCB, this);

  periodic_check_ = nh_.createTimer(ros::Duration(edge_cost_reset_check_period_s),
                                    &NetworkPlannerPlugin::checkResetEdgeCosts, this, false);
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
    const auto& edges_of_graph = current_graph_->getEdges();
    ros::Time current_time = ros::Time::now();

    ROS_DEBUG_STREAM("check ResetEdgeCosts, edges_of_graph.size() = " << edges_of_graph.size());

    for (const auto& edge : edges_of_graph)
    {
      edge_correction_->resetEdgeCostIfExpired(*std::dynamic_pointer_cast<Edge>(edge), current_time);
    }
    graph_publisher_->publish(*current_graph_);
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

  start_pub_.publish(
    arti_nav_core_utils::convertToPoseStamped(*current_pose, arti_nav_core_utils::non_finite_values::REPLACE_BY_0));
  //goal_pub_.publish(
  //  arti_nav_core_utils::convertToPoseStamped(*current_goal_, arti_nav_core_utils::non_finite_values::REPLACE_BY_0));

  //TODO CHECK THE NEXT N CLOSEST TO PREVENT RETURNS
  const arti_graph_processing::VertexPtr start_vertex = getClosestVertex(*current_pose);
  start_vertex_pub_.publish(getVertexPose(start_vertex));

  const arti_graph_processing::VertexPtr goal_vertex = getClosestVertex(*current_goal_);
  //goal_vertex_pub_.publish(getVertexPose(goal_vertex));

  GraphPlan planner_path = arti_graph_processing::AStarAlgorithm::computePath(start_vertex, goal_vertex);

  // check if plan was empty
  if (planner_path.empty())
  {
    ROS_DEBUG_STREAM("Network planner output nothing...");
    return arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::NO_PATH_POSSIBLE;
  }

  // optimize start and end node
  planner_path = optimizePath(planner_path, *current_pose, *current_goal_);

  // check if plan was empty
  if (planner_path.empty())
  {
    ROS_DEBUG_STREAM("Network planner output nothing...");
    return arti_nav_core::BaseNetworkPlanner::BaseNetworkPlannerErrorEnum::NO_PATH_POSSIBLE;
  }

  ROS_DEBUG_STREAM("calling compute result in plan of size: " << planner_path.size());

  bool ignore_planned_path = false;
  if (skip_network_if_close_ && !planner_path.empty() && (planner_path.size() <= max_nodes_to_skip_))
  {
    ROS_DEBUG_STREAM("check to skip network if close");
    const double path_length = calculatePathLength(planner_path, *current_pose, *current_goal_);
    ROS_DEBUG_STREAM("path_length: " << path_length);

    const double start_to_goal_distance = calculateDistance(current_pose->pose, current_goal_->pose);
    ROS_DEBUG_STREAM("start_to_goal_distance: " << start_to_goal_distance);

    if ((path_length < skip_network_path_length_) && (start_to_goal_distance < path_length))
    {
      ROS_DEBUG_STREAM("ignore planned path");
      ignore_planned_path = true;
    }
  }

  plan.path_limits.header.stamp = current_pose->header.stamp;
  plan.path_limits.header.frame_id = current_graph_->getFrameName();
  if (!ignore_planned_path)
  {
    plan.path_limits.poses.reserve(planner_path.size() + 2);
  }
  plan.path_limits.poses.push_back(current_pose->pose);
  if (!ignore_planned_path)
  {
    convertPath(planner_path, plan.path_limits.poses);
  }
  // use current goal orientation for last position orientation
  plan.path_limits.poses.push_back(current_goal_->pose);
  ROS_DEBUG_STREAM("path size: " << plan.path_limits.poses.size());

  updateOrientationBetweenLastPoses(plan.path_limits);
  ROS_DEBUG_STREAM("Debug theta second to last: " << plan.path_limits.poses.end()[-2].theta << " |  last:"
                                                  << plan.path_limits.poses.end()[-1].theta);


  ROS_DEBUG_STREAM("network planer result in plan: " << plan.path_limits);
  // test goal conversion
  geometry_msgs::PoseStamped pose_goal;
  pose_goal.pose.position.x = plan.path_limits.poses.end()[-1].point.x.value;
  pose_goal.pose.position.y = plan.path_limits.poses.end()[-1].point.y.value;
  pose_goal.pose.orientation = tf::createQuaternionMsgFromYaw(plan.path_limits.poses.end()[-1].theta.value);
  pose_goal.header.frame_id = plan.path_limits.header.frame_id;
  pose_goal.header.stamp = ros::Time::now();
  goal_pub_.publish(pose_goal);
  //planner_path.back().second->getDestination()->getPose().pose.
  auto goal_vert = getVertexPose(goal_vertex);
  goal_vert.pose.orientation = pose_goal.pose.orientation;
  goal_vertex_pub_.publish(goal_vert);
  // End Test

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
    const arti_graph_processing::VertexPtr start_vertex = getClosestVertex(current_graph_->getFrameName(),
                                                                           error_pose_a.point);
    const arti_graph_processing::VertexPtr goal_vertex = getClosestVertex(current_graph_->getFrameName(),
                                                                          error_pose_b.point);

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

          // also increaese cost of edge in opposite direction
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

    graph_publisher_->publish(*current_graph_);
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
    graph_publisher_->publish(arti_graph_processing::Graph{{}, "map"});
  }
  else
  {
    // If a current graph was set before, try to choose the graph of the same name:
    if (current_graph_)
    {
      current_graph_ = getGraph(current_graph_->getName());
    }

    // If no current graph was set before, or there was no graph of the same name, select one of the loaded graphs:
    if (!current_graph_)
    {
      current_graph_ = graph_mappings_.begin()->second;
    }

    //arti_graph_processing::FloydWarshallAlgorithm::computeDistanceValues(*current_graph_);

    graph_publisher_->publish(*current_graph_);
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
  GraphPlan planner_path, const arti_nav_core_msgs::Pose2DStampedWithLimits& start,
  const arti_nav_core_msgs::Pose2DStampedWithLimits& goal)
{
  if (planner_path.empty())
  {
    return planner_path;
  }

  bool start_optimized = false;
  bool prev_set = false;
  arti_graph_processing::VertexPtr prev_node_0;
  arti_graph_processing::VertexPtr prev_node_1;

  for (auto it = planner_path.begin(); it != planner_path.end(); it++)
  {
    // reached end of path
    if (!it->second)
    {
      if (!prev_set)
      {
        break;
      }

      // optimize end and stop
      const double distance_nodes = prev_node_0->calculateEuclideanDistanceTo(*prev_node_1);
      const double distance_node_0 = calculateDistance(prev_node_0->getPose().pose, goal.pose);
      const double distance_node_1 = calculateDistance(prev_node_1->getPose().pose, goal.pose);

      if ((distance_node_1 - distance_node_0) < distance_nodes * 1.5 ||
          distance_node_1 < distance_nodes / 2.0)
      {
        //ROS_ERROR("Network Planner we should remove the end due to backturn path");
        it = planner_path.erase(it);
        // have to remove twice to fix it actually
        //it = planner_path.erase(it);
      }
      else if (distance_node_1 < distance_nodes / 2.0)
      {
        //ROS_ERROR("Network Planner we should remove the end due to close distance of the goal and the last node...");
        it = planner_path.erase(it);
        // have to remove twice to fix it actually
        //it = planner_path.erase(it);
      }

      break;
    }
    else
    {
      prev_node_0 = it->second->getSource();
      prev_node_1 = it->second->getDestination();
      prev_set = true;
      //ROS_DEBUG_STREAM("prev_node_0: " << prev_node_0->getName());
      //ROS_DEBUG_STREAM("prev_node_1: " << prev_node_1->getName());
    }

    if (!start_optimized)
    {
      const arti_graph_processing::VertexPtr node_0 = it->second->getSource();
      const arti_graph_processing::VertexPtr node_1 = it->second->getDestination();

      const double distance_node_0 = calculateDistance(node_0->getPose().pose, start.pose);
      const double distance_node_1 = calculateDistance(node_1->getPose().pose, start.pose);
      const double distance_nodes = node_0->calculateEuclideanDistanceTo(*node_1);

      if ((distance_node_1 - distance_node_0) < distance_nodes * 1.5)
      {
        // remove start - we are closer if we go directly to node 1
        //ROS_ERROR("Network Planner we should remove the start...");
        it = planner_path.erase(it);
      }
      start_optimized = true;
    }
  }

  return planner_path;
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
  const GraphPlan& planner_path, std::vector<arti_nav_core_msgs::Pose2DWithLimits>& path) const
{
  bool reverse = false;
  // check for first element, then just keep driving direction
  if (planner_path.front().second && bidirectional_drive_)
  {
    arti_nav_core_msgs::Pose2DWithLimits p1 = convertPose(planner_path.front().first->getPose().pose,
                                                          planner_path.front().second->getDestination()->getPose().pose,
                                                          false);
    // if delta between start and end pose is bigger than 90 degree, drive backwards
    double delta_theta = tfNormalizeAngle(p1.theta.value - getCurrentPose()->pose.theta.value);
    if (std::abs(delta_theta) > M_PI_2)
    {
      ROS_WARN_STREAM("Engage reverse Network drive!");
      reverse = true;
    }
  }

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
      path.push_back(convertPose(path_segment.first->getPose().pose, reverse));
    }

  }
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
      //first segement add distance to start
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
  result.point.x.lower_limit = -corridor_width_;
  result.point.x.upper_limit = corridor_width_;

  result.point.y.value = pose.position.y;
  result.point.y.has_limits = true;
  result.point.y.lower_limit = -corridor_width_;
  result.point.y.upper_limit = corridor_width_;

  // use direction from pose to destination pose to get yaw/theta
  double dx = destination_pose.position.x - pose.position.x;
  double dy = destination_pose.position.y - pose.position.y;
  double theta = std::atan2(dy, dx);
  if (reverse)
  {
    theta = tfNormalizeAngle(theta + M_PI);
  }
  result.theta.value = theta; // tf::getYaw(pose.orientation);
  // Orientation is preferred but not enforced:
  result.theta.has_limits = true;
  result.theta.lower_limit = -M_PI_4;
  result.theta.upper_limit = +M_PI_4;

  return result;
}

void NetworkPlannerPlugin::updateOrientationBetweenLastPoses(arti_nav_core_msgs::Path2DWithLimits& path) const
{
  // dont modify orientation if only start and end position (no inbetween poses)
  if (path.poses.size() <= 2)
  {
    return;
  }

  double x1 = path.poses.end()[-2].point.x.value;
  double y1 = path.poses.end()[-2].point.y.value;

  double x2 = path.poses.end()[-1].point.x.value;
  double y2 = path.poses.end()[-1].point.y.value;

  // use direction from pose to destination pose to get yaw/theta
  double dx = x2 - x1;
  double dy = y2 - y1;
  std::atan2(dy, dx);
  double theta = std::atan2(dy, dx); //tf::getYaw(.orientation);
  double t1 = path.poses.end()[-2].theta.value;
  double t2 = path.poses.end()[-1].theta.value;
  if (bidirectional_drive_)
  {

    // if delta between start and end pose is bigger than 90 degree, drive backwards
    if (std::abs(tfNormalizeAngle(t1 - t2)) > M_PI_2)
    {
      t2 = tfNormalizeAngle(t2 + M_PI);
      ROS_WARN_STREAM("rotate endpose 180 Degree!");
      ROS_DEBUG_STREAM(" t1: \n [ " << path.poses.end()[-1].theta << " ] \n theta new: " << t2);
      path.poses.end()[-1].theta.value = t2; //fNormalizeAngle(t2 + M_PI);

    }
    if (std::abs(tfNormalizeAngle(theta - t2)) > M_PI_2)
    {
      theta = tfNormalizeAngle(theta + M_PI);
    }
    //ROS_WARN_STREAM(" theta [-1]: \n [ " << path.poses.end()[-1].theta.value << " ] \n  [-2]: " <<t1 << " \n [-3]: "
    //                  << path.poses.end()[-3].theta.value << " \n atan2(dy,dx): " << theta);

  }
  path.poses.end()[-1].theta.lower_limit = -M_PI_4;
  path.poses.end()[-1].theta.upper_limit = +M_PI_4;
  path.poses.end()[-2].theta.value = theta;
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
  result.point.x.lower_limit = -corridor_width_;
  result.point.x.upper_limit = corridor_width_;

  result.point.y.value = pose.position.y;
  result.point.y.has_limits = true;
  result.point.y.lower_limit = -corridor_width_;
  result.point.y.upper_limit = corridor_width_;

  result.theta.value = tf::getYaw(pose.orientation);
  if (reverse)
  {
    result.theta.value = tfNormalizeAngle(result.theta.value + M_PI);
  }
  // Orientation is preferred but not enforced:
  result.theta.has_limits = true;
  result.theta.lower_limit = -M_PI_4;
  result.theta.upper_limit = M_PI_4;

  return result;
}

bool NetworkPlannerPlugin::changeRegionCB(
  arti_move_base_msgs::ChangeRegion::Request& request, arti_move_base_msgs::ChangeRegion::Response& /*response*/)
{
  const auto new_graph = getGraph(request.region_name);
  if (new_graph)
  {
    current_graph_ = new_graph;
    graph_publisher_->publish(*current_graph_);
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
    edges_marker.type = visualization_msgs::Marker::LINE_LIST;
    edges_marker.header.frame_id = current_graph_->getFrameName();
    edges_marker.header.stamp = ros::Time::now();
    edges_marker.color.r = 1;
    edges_marker.color.g = 0.0;
    edges_marker.color.b = 0.0;
    edges_marker.color.a = 1.;
    edges_marker.pose.orientation.w = 1.;
    edges_marker.scale.x = 0.1;
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
      }
    }
    graph_markers.markers.push_back(edges_marker);

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
