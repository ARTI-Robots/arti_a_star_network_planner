/*
Created by clemens on 29.04.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_A_STAR_NETWORK_PLANNER_CONVERT_PATH_H
#define ARTI_A_STAR_NETWORK_PLANNER_CONVERT_PATH_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace arti_a_star_network_planner
{
class ConvertPath
{
public:
  explicit ConvertPath(const ros::NodeHandle& nh);

  void convertPath();

private:
  struct Pose
  {
    double x;
    double y;
    double yaw;

    double distance(const Pose& other) const;
    double angularDistance(const Pose& other) const;
  };

  struct RawEdge;

  struct RawVertex
  {
    Pose pose;

    std::shared_ptr<RawEdge> outgoing_edge;
    std::shared_ptr<RawEdge> incoming_edge;
  };

  struct RawEdge
  {
    std::weak_ptr<RawVertex> source;
    std::weak_ptr<RawVertex> sink;
  };

  struct Path
  {
    std::vector<std::shared_ptr<RawVertex>> vertices;
  };

  struct CombinedVertex
  {
    std::set<std::shared_ptr<RawVertex>> vertex_set;
    Pose seed_pose;
  };

  struct Edge;

  struct Vertex
  {
    Pose pose;
    std::string name_;

    std::vector<std::shared_ptr<Edge>> outgoing_edges;
    std::vector<std::shared_ptr<Edge>> incoming_edges;
  };

  struct Edge
  {
    std::weak_ptr<Vertex> source;
    std::weak_ptr<Vertex> sink;
  };

  struct Graph
  {
    std::set<std::shared_ptr<Vertex>> vertex_set;
  };

  Path loadPath();
  std::vector<std::shared_ptr<CombinedVertex>> combineVertices(const Path& path);
  Graph createGraph(const std::vector<std::shared_ptr<CombinedVertex>>& vertex_set);
  void expand(
    const std::shared_ptr<Vertex>& vertex, std::set<std::shared_ptr<Vertex>>& vertex_set,
    const std::set<std::shared_ptr<Vertex>>& checked_vertices, bool incoming, bool outgoing);
  Graph reduceGraph(const Graph& full_graph);
  Graph connectGraph(const Graph& reduced_graph);
  std::set<std::shared_ptr<Edge>> getEdges(const Graph& graph);

  void writeGraph(const Graph& graph);

  void publishPath(const Path& path);
  void publishGraph(const Graph& graph, ros::Publisher& graph_pub);

  visualization_msgs::Marker visualizePoses(const std::vector<Pose>& poses);
  geometry_msgs::Pose convertToPose(const Pose& pose);

  ros::NodeHandle nh_;

  std::string frame_id_;
  std::string name_;
  std::string path_file_name_;
  std::string output_graph_file_name_;
  double same_vertex_distance_;
  double min_distance_;
  double max_rotation_difference_;
  double connection_distance_;

  ros::Publisher raw_path_pub_;
  ros::Publisher full_graph_pub_;
  ros::Publisher reduce_graph_pub_;
  ros::Publisher connected_graph_pub_;
};
}

#endif //ARTI_A_STAR_NETWORK_PLANNER_CONVERT_PATH_H
