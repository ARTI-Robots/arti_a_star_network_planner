/*
Created by clemens on 29.04.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_a_star_network_planner/convert_path.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>
#include <fstream>

namespace arti_a_star_network_planner
{
ConvertPath::ConvertPath(const ros::NodeHandle& nh)
  : nh_(nh)
{
  frame_id_ = nh_.param<std::string>("frame_id", "map");
  name_ = nh_.param<std::string>("name", "nav_network");
  path_file_name_ = nh_.param<std::string>("path_file_name", "path.yaml");
  output_graph_file_name_ = nh_.param<std::string>("output_graph_file_name", "graph.yaml");

  same_vertex_distance_ = nh_.param<double>("same_vertex_distance", 0.1);
  min_distance_ = nh_.param<double>("min_distance", 2.);
  max_rotation_difference_ = nh_.param<double>("max_rotation_difference", M_PI / 4.);
  connection_distance_ = nh_.param<double>("min_distance", connection_distance_ + 0.2);

  raw_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("raw_path", 1, true);
  full_graph_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("full_graph", 1, true);
  reduce_graph_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("reduce_graph", 1, true);
  connected_graph_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("connected_graph", 1, true);
}

void ConvertPath::convertPath()
{
  Path path = loadPath();
  publishPath(path);

  std::vector<std::shared_ptr<ConvertPath::CombinedVertex>> combined_vertices = combineVertices(path);

  Graph full_graph = createGraph(combined_vertices);
  publishGraph(full_graph, full_graph_pub_);

  Graph reduced_graph = reduceGraph(full_graph);
  publishGraph(reduced_graph, reduce_graph_pub_);

  Graph graph = connectGraph(reduced_graph);
  publishGraph(graph, connected_graph_pub_);

  writeGraph(graph);
}

ConvertPath::Path ConvertPath::loadPath()
{
  Path result;

  try
  {
    YAML::Node yaml = YAML::LoadFile(path_file_name_);

    for (const YAML::Node& vertex : yaml["path"])
    {
      std::shared_ptr<RawVertex> source_vertex(new RawVertex);
      source_vertex->pose.x = vertex["source"]["x"].as<double>();
      source_vertex->pose.y = vertex["source"]["y"].as<double>();
      source_vertex->pose.yaw = vertex["source"]["yaw"].as<double>();
      result.vertices.push_back(source_vertex);

      std::shared_ptr<RawVertex> sink_vertex(new RawVertex);
      sink_vertex->pose.x = vertex["sink"]["x"].as<double>();
      sink_vertex->pose.y = vertex["sink"]["y"].as<double>();
      sink_vertex->pose.yaw = vertex["sink"]["yaw"].as<double>();
      result.vertices.push_back(sink_vertex);

      std::shared_ptr<RawEdge> new_edge(new RawEdge);
      new_edge->source = source_vertex;
      new_edge->sink = sink_vertex;

      source_vertex->outgoing_edge = new_edge;
      sink_vertex->incoming_edge = new_edge;
    }
  }
  catch (YAML::BadFile& ex)
  {
    ROS_ERROR_STREAM("ConvertPath path file '" << path_file_name_ << "' not found");
    throw ex;
  }
  catch (YAML::Exception& ex)
  {
    ROS_ERROR_STREAM("ConvertPath path data is invalid: " << ex.what());
    throw ex;
  }

  return result;
}

std::vector<std::shared_ptr<ConvertPath::CombinedVertex>> ConvertPath::combineVertices(const Path& path)
{
  std::vector<std::shared_ptr<CombinedVertex>> result;

  for (const auto& vertex : path.vertices)
  {
    bool is_contained = false;

    for (auto& combined_vertex : result)
    {
      if (vertex->pose.distance(combined_vertex->seed_pose) < same_vertex_distance_)
      {
        combined_vertex->vertex_set.insert(vertex);
        is_contained = true;
        break;
      }
    }

    if (!is_contained)
    {
      std::shared_ptr<CombinedVertex> new_vertex(new CombinedVertex);
      new_vertex->seed_pose = vertex->pose;
      new_vertex->vertex_set.insert(vertex);

      result.push_back(new_vertex);
    }
  }

  return result;
}

ConvertPath::Graph ConvertPath::createGraph(const std::vector<std::shared_ptr<CombinedVertex>>& vertex_set)
{
  Graph result;

  // store information how the raw vertices where combined into a full used vertex
  std::map<std::shared_ptr<RawVertex>, std::shared_ptr<CombinedVertex>> raw_to_combined_mapping;
  std::map<std::shared_ptr<CombinedVertex>, std::shared_ptr<Vertex>> combined_to_normal_mapping;

  // first create vertices
  for (const auto& vertex : vertex_set)
  {
    std::shared_ptr<Vertex> new_vertex(new Vertex);
    new_vertex->pose = vertex->seed_pose;
    std::stringstream name_stream;
    name_stream << "V_" << result.vertex_set.size();
    new_vertex->name_ = name_stream.str();

    result.vertex_set.insert(new_vertex);

    combined_to_normal_mapping.insert(std::make_pair(vertex, new_vertex));

    for (const auto& raw_vertex : vertex->vertex_set)
    {
      raw_to_combined_mapping.insert(std::make_pair(raw_vertex, vertex));
    }
  }

  // second create the edges
  for (const auto& vertex : vertex_set)
  {
    std::shared_ptr<Vertex> source_vertex = combined_to_normal_mapping[vertex];
    for (const auto& raw_vertex : vertex->vertex_set)
    {
      if (raw_vertex->outgoing_edge)
      {
        std::shared_ptr<RawVertex> raw_sink = raw_vertex->outgoing_edge->sink.lock();
        std::shared_ptr<CombinedVertex> combined_sink = raw_to_combined_mapping[raw_sink];
        std::shared_ptr<Vertex> sink_vertex = combined_to_normal_mapping[combined_sink];

        std::shared_ptr<Edge> new_edge(new Edge);
        new_edge->source = source_vertex;
        new_edge->sink = sink_vertex;

        source_vertex->outgoing_edges.push_back(new_edge);
        sink_vertex->incoming_edges.push_back(new_edge);
      }
    }
  }

  return result;
}

void ConvertPath::expand(
  const std::shared_ptr<Vertex>& vertex, std::set<std::shared_ptr<Vertex>>& vertex_set,
  const std::set<std::shared_ptr<Vertex>>& checked_vertices, bool incoming, bool outgoing)
{
  if (incoming)
  {
    for (const auto& edge : vertex->incoming_edges)
    {
      std::shared_ptr<Vertex> vertex_to_add = edge->source.lock();

      if (checked_vertices.count(vertex_to_add) == 0)
      {
        vertex_set.insert(vertex_to_add);
      }
    }
  }

  if (outgoing)
  {
    for (const auto& edge : vertex->outgoing_edges)
    {
      std::shared_ptr<Vertex> vertex_to_add = edge->sink.lock();

      if (checked_vertices.count(vertex_to_add) == 0)
      {
        vertex_set.insert(vertex_to_add);
      }
    }
  }
}

ConvertPath::Graph ConvertPath::reduceGraph(const Graph& full_graph)
{
  Graph result;

  // first only add vertices if they are far enough away
  for (const auto& vertex : full_graph.vertex_set)
  {
    bool should_be_added = true;
    std::set<std::shared_ptr<Vertex>> vertices_to_check;
    std::set<std::shared_ptr<Vertex>> checked_vertices;
    expand(vertex, vertices_to_check, checked_vertices, true, true);

    while (!vertices_to_check.empty())
    {
      const auto it = vertices_to_check.begin();
      vertices_to_check.erase(it);

      std::shared_ptr<Vertex> other_vertex_to_check = *it;
      checked_vertices.insert(other_vertex_to_check);

      if (result.vertex_set.count(other_vertex_to_check) != 0)
      {
        if ((vertex->pose.distance(other_vertex_to_check->pose) < min_distance_)
            && (vertex->pose.angularDistance(other_vertex_to_check->pose) < max_rotation_difference_))
        {
          should_be_added = false;
          break;
        }
      }

      if (vertex->pose.distance(other_vertex_to_check->pose) < min_distance_)
      {
        expand(other_vertex_to_check, vertices_to_check, checked_vertices, true, true);
      }
    }

    if (should_be_added)
    {
      result.vertex_set.insert(vertex);
    }
  }

  // second compute outgoing edges between vertices which are part of the result
  for (const auto& vertex : result.vertex_set)
  {
    std::set<std::shared_ptr<Vertex>> vertices_to_check;
    std::set<std::shared_ptr<Vertex>> checked_vertices;
    expand(vertex, vertices_to_check, checked_vertices, false, true);

    while (!vertices_to_check.empty())
    {
      const auto it = vertices_to_check.begin();
      vertices_to_check.erase(it);

      std::shared_ptr<Vertex> other_vertex_to_check = *it;
      checked_vertices.insert(other_vertex_to_check);

      if (result.vertex_set.count(other_vertex_to_check) != 0)
      {
        std::shared_ptr<Edge> new_edge(new Edge);
        new_edge->source = vertex;
        new_edge->sink = other_vertex_to_check;
        vertex->outgoing_edges.push_back(new_edge);
        other_vertex_to_check->incoming_edges.push_back(new_edge);
      }
      else
      {
        expand(other_vertex_to_check, vertices_to_check, checked_vertices, false, true);
      }
    }
  }

  // third compute incoming edges between vertices which are part of the result
  for (const auto& vertex : result.vertex_set)
  {
    std::set<std::shared_ptr<Vertex>> vertices_to_check;
    std::set<std::shared_ptr<Vertex>> checked_vertices;
    expand(vertex, vertices_to_check, checked_vertices, true, false);

    while (!vertices_to_check.empty())
    {
      const auto it = vertices_to_check.begin();
      vertices_to_check.erase(it);

      std::shared_ptr<Vertex> other_vertex_to_check = *it;
      checked_vertices.insert(other_vertex_to_check);

      if (result.vertex_set.count(other_vertex_to_check) != 0)
      {
        std::shared_ptr<Edge> new_edge(new Edge);
        new_edge->source = other_vertex_to_check;
        new_edge->sink = vertex;
        vertex->incoming_edges.push_back(new_edge);
        other_vertex_to_check->outgoing_edges.push_back(new_edge);
      }
      else
      {
        expand(other_vertex_to_check, vertices_to_check, checked_vertices, true, false);
      }
    }
  }

  // forth remove outgoing edges which connect to a vertex not part of the result
  for (auto& vertex : result.vertex_set)
  {
    std::vector<std::shared_ptr<Edge>> new_outgoing_edges;
    for (const auto& edge : vertex->outgoing_edges)
    {
      if (result.vertex_set.count(edge->sink.lock()) != 0)
      {
        new_outgoing_edges.push_back(edge);
      }
    }

    vertex->outgoing_edges = new_outgoing_edges;
  }

  // fifth remove incoming edges which connect to a vertex not part of the result
  for (auto& vertex : result.vertex_set)
  {
    std::vector<std::shared_ptr<Edge>> new_incoming_edges;
    for (const auto& edge : vertex->incoming_edges)
    {
      if (result.vertex_set.count(edge->source.lock()) != 0)
      {
        new_incoming_edges.push_back(edge);
      }
    }

    vertex->incoming_edges = new_incoming_edges;
  }

  return result;
}

ConvertPath::Graph ConvertPath::connectGraph(const Graph& reduced_graph)
{
  Graph result;
  // connect vertices which are close to each other
  for (auto& vertex1 : reduced_graph.vertex_set)
  {
    for (auto& vertex2 : reduced_graph.vertex_set)
    {
      if (vertex1 == vertex2)
      {
        continue;
      }

      if (vertex1->pose.distance(vertex2->pose) > connection_distance_)
      {
        continue;
      }

      bool already_connected = false;
      if (!already_connected)
      {
        for (const auto& edge : vertex1->incoming_edges)
        {
          if (edge->source.lock() == vertex2)
          {
            already_connected = true;
            break;
          }
        }
      }

      if (!already_connected)
      {
        for (const auto& edge : vertex1->outgoing_edges)
        {
          if (edge->sink.lock() == vertex2)
          {
            already_connected = true;
            break;
          }
        }
      }

      if (already_connected)
      {
        continue;
      }

      std::shared_ptr<Edge> first_edge(new Edge);
      first_edge->source = vertex1;
      first_edge->sink = vertex2;
      vertex1->outgoing_edges.push_back(first_edge);
      vertex2->incoming_edges.push_back(first_edge);

      std::shared_ptr<Edge> second_edge(new Edge);
      second_edge->source = vertex2;
      second_edge->sink = vertex1;
      vertex1->incoming_edges.push_back(second_edge);
      vertex2->outgoing_edges.push_back(second_edge);
    }

    result.vertex_set.insert(vertex1);
  }

  // allow movement in both directions
  std::set<std::shared_ptr<Edge>> edges = getEdges(result);
  for (const auto& edge: edges)
  {
    bool already_connected = false;
    for (const auto& incomming_edges : edge->source.lock()->incoming_edges)
    {
      if (incomming_edges->source.lock() == edge->sink.lock())
      {
        already_connected = true;
        break;
      }
    }

    if (!already_connected)
    {
      std::shared_ptr<Edge> first_edge(new Edge);
      first_edge->source = edge->sink;
      first_edge->sink = edge->source;
      first_edge->source.lock()->outgoing_edges.push_back(first_edge);
      first_edge->sink.lock()->incoming_edges.push_back(first_edge);
    }
  }
  return result;
}

std::set<std::shared_ptr<ConvertPath::Edge>> ConvertPath::getEdges(const Graph& graph)
{
  std::set<std::shared_ptr<Edge>> result;

  for (const auto& vertex: graph.vertex_set)
  {
    for (const auto& edge : vertex->incoming_edges)
    {
      result.insert(edge);
    }

    for (const auto& edge : vertex->outgoing_edges)
    {
      result.insert(edge);
    }
  }

  return result;
}

void ConvertPath::writeGraph(const Graph& graph)
{
  YAML::Node graph_data;
  graph_data["name"] = name_;
  graph_data["frame_id"] = frame_id_;

  YAML::Node vertex_params;
  for (const auto& vertex: graph.vertex_set)
  {
    YAML::Node pose;
    pose["x"] = vertex->pose.x;
    pose["y"] = vertex->pose.y;
    pose["z"] = "0.000001"; // 0 causes an error when writing->reading as yaml
    pose["yaw"] = vertex->pose.yaw;
    pose["name"] = vertex->name_;

    vertex_params.push_back(pose);
  }
  graph_data["vertices"] = vertex_params;

  std::set<std::shared_ptr<Edge>> edges = getEdges(graph);
  YAML::Node edge_params;
  for (const auto& edge: edges)
  {
    std::shared_ptr<Vertex> source = edge->source.lock();
    std::shared_ptr<Vertex> sink = edge->sink.lock();

    YAML::Node edge_param;
    if (source && sink)
    {
      edge_param["source"] = source->name_;
      edge_param["sink"] = sink->name_;
      edge_param["costs"] = source->pose.distance(sink->pose);
      if (edge_param["costs"].as<std::string>() == "0")
      {
        edge_param["costs"] = "0.000001"; // 0 causes an error when writing->reading as yaml
      }
    }
    edge_params.push_back(edge_param);
  }
  graph_data["edges"] = edge_params;

  YAML::Node complete_graph_data;
  complete_graph_data.push_back(graph_data);

  YAML::Node final_data;
  final_data["graphs"] = complete_graph_data;

  std::ofstream file(output_graph_file_name_);
  file << final_data;
  file.flush();
}

void ConvertPath::publishPath(const Path& path)
{
  visualization_msgs::MarkerArray graph_msg;
  visualization_msgs::Marker line_list;
  line_list.ns = "edges";
  line_list.id = 0;
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.header.frame_id = frame_id_;
  line_list.header.stamp = ros::Time::now();
  line_list.color.r = 0.;
  line_list.color.g = 196. / 255.;
  line_list.color.b = 36. / 255.;
  line_list.color.a = 1.;
  line_list.pose.orientation.w = 1.;
  line_list.scale.x = 0.1;
  line_list.scale.y = 0.1;
  line_list.scale.z = 0.1;

  std::vector<Pose> poses(path.vertices.size());
  for (const auto& vertex: path.vertices)
  {
    if (vertex->outgoing_edge)
    {
      line_list.points.push_back(convertToPose(vertex->outgoing_edge->source.lock()->pose).position);
      line_list.points.push_back(convertToPose(vertex->outgoing_edge->sink.lock()->pose).position);
    }
    poses.push_back(vertex->pose);
  }
  graph_msg.markers.push_back(line_list);
  graph_msg.markers.push_back(visualizePoses(poses));

  raw_path_pub_.publish(graph_msg);
}

void ConvertPath::publishGraph(const Graph& graph, ros::Publisher& graph_pub)
{
  visualization_msgs::MarkerArray graph_msg;
  visualization_msgs::Marker line_list;
  line_list.ns = "edges";
  line_list.id = 0;
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.header.frame_id = frame_id_;
  line_list.header.stamp = ros::Time::now();
  line_list.color.r = 0.;
  line_list.color.g = 196. / 255.;
  line_list.color.b = 36. / 255.;
  line_list.color.a = 1.;
  line_list.pose.orientation.w = 1.;
  line_list.scale.x = 0.1;
  line_list.scale.y = 0.1;
  line_list.scale.z = 0.1;

  std::set<std::shared_ptr<Edge>> edges = getEdges(graph);
  for (const auto& edge: edges)
  {
    std::shared_ptr<Vertex> source = edge->source.lock();
    std::shared_ptr<Vertex> sink = edge->sink.lock();

    if (source && sink)
    {
      line_list.points.push_back(convertToPose(source->pose).position);
      line_list.points.push_back(convertToPose(sink->pose).position);
    }
  }
  graph_msg.markers.push_back(line_list);

  std::vector<Pose> poses(graph.vertex_set.size());
  for (const auto& vertex: graph.vertex_set)
  {
    poses.push_back(vertex->pose);
  }

  graph_msg.markers.push_back(visualizePoses(poses));

  graph_pub.publish(graph_msg);
}

visualization_msgs::Marker ConvertPath::visualizePoses(const std::vector<Pose>& poses)
{
  visualization_msgs::Marker sphere_list;
  sphere_list.ns = "vertex_poses";
  sphere_list.id = 0;
  sphere_list.action = visualization_msgs::Marker::ADD;
  sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
  sphere_list.header.frame_id = frame_id_;
  sphere_list.header.stamp = ros::Time::now();
  sphere_list.color.r = 235. / 255.;
  sphere_list.color.g = 235. / 255.;
  sphere_list.color.b = 52. / 255.;
  sphere_list.color.a = 1.;
  sphere_list.pose.orientation.w = 1.;
  sphere_list.scale.x = 0.2;
  sphere_list.scale.y = 0.2;
  sphere_list.scale.z = 0.2;
  sphere_list.points.reserve(poses.size());

  for (const auto& pose : poses)
  {
    sphere_list.points.push_back(convertToPose(pose).position);
  }

  return sphere_list;
}

geometry_msgs::Pose ConvertPath::convertToPose(const Pose& pose)
{
  geometry_msgs::Pose result;
  result.position.x = pose.x;
  result.position.y = pose.y;
  result.orientation = tf::createQuaternionMsgFromYaw(pose.yaw);

  return result;
}

double ConvertPath::Pose::distance(const Pose& other) const
{
  return std::hypot(x - other.x, y - other.y);
}

double ConvertPath::Pose::angularDistance(const Pose& other) const
{
  return std::min(std::abs(angles::shortest_angular_distance(yaw, other.yaw)),
                  std::abs(angles::shortest_angular_distance(other.yaw, yaw)));
}

}

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "convert_path_node");
    ros::NodeHandle nh("~");

    arti_a_star_network_planner::ConvertPath node(nh);
    ros::spinOnce();

    node.convertPath();
    ROS_INFO_STREAM("creating graph displaying graph");
    ros::spin();
  }
  catch (std::exception& ex)
  {
    std::cerr << "Unhandled exception: " << ex.what() << std::endl;
    return 1;
  }

  return 0;
}
