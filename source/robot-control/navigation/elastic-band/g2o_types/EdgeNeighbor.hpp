/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
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
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann
 * Adaptation: Mickaël Salamin, Fall 2019
 * Implementation: ROS package teb_local_planner
 * Source: https://github.com/rst-tu-dortmund/teb_local_planner/
 *********************************************************************/

#ifndef _EDGE_NEIGHBOR_HPP_
#define _EDGE_NEIGHBOR_HPP_

#include <elastic-band/g2o_types/BaseTebEdges.hpp>
#include <elastic-band/g2o_types/VertexPose.hpp>
#include <elastic-band/g2o_types/Penalties.hpp>
#include <elastic-band/TebConfig.hpp>
#include <elastic-band/RobotFootprintModel.hpp>

namespace elastic_band
{

/**
 * @class EdgeNeighbor
 * @brief Edge defining the cost function for keeping a minimum distance from neighbors.
 *
 * The edge depends on two vertices \f$ \mathbf{s}_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyBelow}( dist2neighbor, min_neighbor_dist ) \cdot weight_neighbor \f$. \n
 * \e dist2neighbor denotes the distance between two robots at their respective pose \n
 * \e penaltyBelow denotes the penalty function, see penaltyBoundFromBelow() \n
 * @see TebPlanner::AddEdgesNeighbors
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeNeighbor : public BaseTebBinaryEdge<1, double, VertexPose, VertexPose>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeNeighbor()
  {
    _measurement = 0;
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && robot_model_, "You must call setTebConfig() and setRobotModel() on EdgeNeighbor()");
    const VertexPose* bandpt1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* bandpt2 = static_cast<const VertexPose*>(_vertices[1]);

    const double dist = robot_model_->calculateDistance(bandpt1->pose(), bandpt2->pose());

    _error[0] = penaltyBoundFromBelow(dist, cfg_->neighbors.min_neighbor_dist, cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeNeighbor::computeError() _error[0]=%f\n", _error[0]);
  }

  /**
   * @brief Set pointer to the robot model
   * @param robot_model Robot model required for distance calculation
   */
  void setRobotModel(const BaseRobotFootprintModel* robot_model)
  {
    robot_model_ = robot_model;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   */
  void setParameters(const TebConfig& cfg, const BaseRobotFootprintModel* robot_model)
  {
    cfg_ = &cfg;
    robot_model_ = robot_model;
  }

protected:

  const BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // namespace elastic_band

#endif /* _EDGE_NEIGHBOR_HPP_ */
