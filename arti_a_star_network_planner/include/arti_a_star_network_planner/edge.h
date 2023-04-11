/*
Created by abuchegger on 2020-11-27.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ARTI_A_STAR_NETWORK_PLANNER_EDGE_H
#define ARTI_A_STAR_NETWORK_PLANNER_EDGE_H

#include <arti_graph_processing/edge.h>
#include <ros/time.h>

namespace arti_a_star_network_planner
{

class Edge : public arti_graph_processing::Edge
{
public:
  Edge(
    const arti_graph_processing::VertexPtr& source, const arti_graph_processing::VertexPtr& destination, double costs);
  Edge(Edge&&) = default;

  size_t getIncreaseCount() const;

  void setCost(double cost, ros::Time expiration_time);
  void resetCostIfExpired(ros::Time current_time);

protected:
  size_t increase_count_{0};
  double initial_cost_{0.};
  ros::Time expiration_time_{0};
};

}

#endif //ARTI_A_STAR_NETWORK_PLANNER_EDGE_H
