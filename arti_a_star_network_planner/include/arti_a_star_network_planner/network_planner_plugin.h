/*
Created by clemens on 30.03.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_A_STAR_NETWORK_PLANNER_NETWORK_PLANNER_PLUGIN_H
#define ARTI_A_STAR_NETWORK_PLANNER_NETWORK_PLANNER_PLUGIN_H

#include <arti_nav_core/base_network_planner.h>
#include <arti_nav_core_msgs/Movement2DGoalWithConstraints.h>
#include <arti_nav_core_msgs/Point2DWithLimits.h>
#include <arti_nav_core_msgs/Pose2DStampedWithLimits.h>
#include <arti_a_star_network_planner/robot_information.h>
#include <arti_a_star_network_planner/edge_correction.h>
#include <arti_graph_processing/graph_loader.h>
#include <arti_graph_processing/graph_visualization_publisher.h>
#include <arti_graph_processing/types.h>
#include <arti_move_base_msgs/ChangeRegion.h>
#include <boost/filesystem/path.hpp>
#include <boost/optional.hpp>
#include <ctime>
#include <std_srvs/Empty.h>
#include <string>

namespace arti_a_star_network_planner
{

class NetworkPlannerPlugin : public arti_nav_core::BaseNetworkPlanner
{
public:
  NetworkPlannerPlugin() = default;

  void initialize(std::string name, arti_nav_core::Transformer* transformer) override;

  bool setGoal(const arti_nav_core_msgs::Pose2DStampedWithLimits& goal) override;

  BaseNetworkPlannerErrorEnum makePlan(arti_nav_core_msgs::Movement2DGoalWithConstraints& plan) override;

  void handlePlannerError(
    const arti_nav_core_msgs::Pose2DWithLimits& error_pose_a,
    const arti_nav_core_msgs::Pose2DWithLimits& error_pose_b) override;

private:
  using GraphPlan = std::vector<std::pair<arti_graph_processing::VertexPtr, arti_graph_processing::EdgePtr>>;

  struct GraphLoader : public arti_graph_processing::GraphLoader
  {
    arti_graph_processing::EdgePtr loadEdge(
      const arti_graph_processing::Graph& graph, const arti_graph_processing::VertexPtr& source,
      const arti_graph_processing::VertexPtr& destination, double costs,
      const arti_ros_param::Param& root_param) override;
  };

  void loadGraphs();

  arti_graph_processing::GraphPtr getGraph(const std::string& name) const;

  void checkResetEdgeCosts(const ros::TimerEvent& e);

  boost::optional<arti_nav_core_msgs::Pose2DStampedWithLimits> transformPose(
    const arti_nav_core_msgs::Pose2DStampedWithLimits& pose) const;

  boost::optional<arti_nav_core_msgs::Pose2DStampedWithLimits> getCurrentPose() const;

  static double calculateDistance(
    const geometry_msgs::Pose& a_pose, const geometry_msgs::Pose& b_pose);

  static double calculateDistance(
    const geometry_msgs::Pose& a_pose, const arti_nav_core_msgs::Pose2DWithLimits& b_pose);

  static double calculateDistance(
    const arti_nav_core_msgs::Pose2DWithLimits& a_pose, const arti_nav_core_msgs::Pose2DWithLimits& b_pose);

  arti_graph_processing::VertexPtr getClosestVertex(
    const arti_nav_core_msgs::Pose2DStampedWithLimits& pose) const;

  arti_graph_processing::VertexPtr getClosestVertex(
    const std::string& frame_id, const arti_nav_core_msgs::Point2DWithLimits& point) const;

  static GraphPlan optimizePath(
    GraphPlan planer_path, const arti_nav_core_msgs::Pose2DStampedWithLimits& start,
    const arti_nav_core_msgs::Pose2DStampedWithLimits& goal);

  void convertPath(const GraphPlan& planner_path, std::vector<arti_nav_core_msgs::Pose2DWithLimits>& path) const;

  static double calculatePathLength(
    const GraphPlan& path, const arti_nav_core_msgs::Pose2DStampedWithLimits& start,
    const arti_nav_core_msgs::Pose2DStampedWithLimits& goal);

  arti_nav_core_msgs::Pose2DStampedWithLimits convertPose(
    const geometry_msgs::PoseStamped& pose,
    const geometry_msgs::PoseStamped& destination_pose,
    const bool reverse) const;

  arti_nav_core_msgs::Pose2DWithLimits convertPose(
    const geometry_msgs::Pose& pose,
    const geometry_msgs::Pose& destination_pose,
    const bool reverse) const;

  arti_nav_core_msgs::Pose2DStampedWithLimits convertPose(
    const geometry_msgs::PoseStamped& pose, const bool reverse) const;

  arti_nav_core_msgs::Pose2DWithLimits convertPose(const geometry_msgs::Pose& pose, const bool reverse) const;

  void updateOrientationBetweenLastPoses(arti_nav_core_msgs::Path2DWithLimits& path) const;

  bool changeRegionCB(
    arti_move_base_msgs::ChangeRegion::Request& request, arti_move_base_msgs::ChangeRegion::Response& response);
  bool reloadNetworksCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  void publishSearchGraph(
    const std::vector<std::pair<arti_graph_processing::VertexPtr,
      arti_graph_processing::EdgePtr>>& planer_path);

  static geometry_msgs::PoseStamped getVertexPose(const arti_graph_processing::VertexPtr& vertex);
  static std::time_t getLastModificationTime(const boost::filesystem::path& path);

  ros::NodeHandle nh_;
  boost::filesystem::path graphs_file_path_;
  std::time_t graphs_file_last_modification_time_{0};
  std::map<std::string, arti_graph_processing::GraphPtr> graph_mappings_;
  arti_graph_processing::GraphPtr current_graph_;
  boost::optional<arti_nav_core_msgs::Pose2DStampedWithLimits> current_goal_;
  double corridor_width_{1.};
  bool bidirectional_drive_{false};
  ros::Timer periodic_check_;
  bool skip_network_if_close_;
  double skip_network_path_length_;
  uint64_t max_nodes_to_skip_;

  std::shared_ptr<RobotInformation> robot_information_;

  arti_nav_core::Transformer* transformer_{nullptr};

  std::shared_ptr<EdgeCorrection> edge_correction_;

  ros::ServiceServer change_region_service_;
  ros::ServiceServer reload_networks_service_;

  boost::optional<arti_graph_processing::GraphVisualizationPublisher> graph_publisher_;
  ros::Publisher graph_search_publisher_;
  ros::Publisher start_pub_;
  ros::Publisher start_vertex_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher goal_vertex_pub_;
};
}

#endif //ARTI_A_STAR_NETWORK_PLANNER_NETWORK_PLANNER_PLUGIN_H
