/*
Created by abuchegger on 2020-11-27.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <arti_a_star_network_planner/edge.h>
#include <utility>

namespace arti_a_star_network_planner
{
Edge::Edge(
  const arti_graph_processing::VertexPtr& source, const arti_graph_processing::VertexPtr& destination, double costs)
  : arti_graph_processing::Edge(source, destination, costs), initial_cost_(costs)
{
}

size_t Edge::getIncreaseCount() const
{
  return increase_count_;
}

void Edge::resetCostIfExpired(ros::Time current_time)
{
  if (costs_ == initial_cost_)
  {
    return;
  }

  if (current_time > expiration_time_)
  {
    increase_count_ = 0;
    costs_ = initial_cost_;
    expiration_time_ = ros::Time(0);
  }
}


void Edge::setCost(double cost, ros::Time expiration_time)
{
  if (cost > costs_)
  {
    increase_count_++;
  }
  expiration_time_ = expiration_time;
  costs_ = cost;
}

}
