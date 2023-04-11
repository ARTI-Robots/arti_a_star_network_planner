/*
Created by clemens on 31.03.20.
This file is part of the software provided by ARTI
Copyright (c) 2020, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_a_star_network_planner/edge_correction.h>
#include <limits>

namespace arti_a_star_network_planner
{
EdgeCorrection::EdgeCorrection(size_t max_number_increases, double increase_factor, double edge_cost_validity_period_s)
  : max_number_increases_(max_number_increases), increase_factor_(increase_factor)
{
  validity_period_ = ros::Duration(edge_cost_validity_period_s);
}

void EdgeCorrection::increaseEdgeCosts(Edge& edge, ros::Time current_time)
{
  ros::Time expiration_time = current_time + validity_period_;

  if (edge.getIncreaseCount() >= max_number_increases_)
  {
    edge.setCost(std::numeric_limits<double>::infinity(), expiration_time);
  }
  else
  {
    edge.setCost(edge.getCosts() * increase_factor_, expiration_time);
  }
}

void EdgeCorrection::resetEdgeCostIfExpired(Edge& edge, ros::Time current_time)
{
  edge.resetCostIfExpired(current_time);
}


}
