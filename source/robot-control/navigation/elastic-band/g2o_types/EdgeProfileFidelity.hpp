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

#ifndef _EDGE_PROFILE_FIDELITY_HPP_
#define _EDGE_PROFILE_FIDELITY_HPP_

#include <elastic-band/g2o_types/BaseTebEdges.hpp>
#include <elastic-band/g2o_types/VertexPose.hpp>
#include <elastic-band/g2o_types/VertexTimeDiff.hpp>
#include <elastic-band/TebConfig.hpp>

#include <iostream>

namespace elastic_band
{

/**
 * @class EdgeProfileFidelity
 * @brief Edge defining the cost function for minimizing the difference in terms of translational and rotational velocity
 * w.r.t. the reference velocity profile of the trajectory
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$ and minimizes: \n
 * \f$ \min [(v-v_{ref})^2,(omega-omega_{ref})^2]^T \cdot weight \f$. \n
 * \e v is calculated using the difference quotient and the position parts of both poses. \n
 * \e omega is calculated using the difference quotient of both yaw angles followed by a normalization to [-pi, pi]. \n
 * \e v_{ref} is the reference translational velocity between both poses. \n
 * \e omega_{ref} is the reference rotational velocity between both poses. \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 2: the first component represents the translational velocity and
 * the second one the rotational velocity.
 * @see TebPlanner::AddEdgesProfileFidelity
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeProfileFidelity : public BaseTebMultiEdge<2, const Velocity*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeProfileFidelity()
  {
    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
    _measurement = NULL;
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeProfileFidelity()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);

    const Eigen::Vector2d deltaS = conf2->estimate().position() - conf1->estimate().position();

    double dist = deltaS.norm();
    const double angle_diff = g2o::normalize_theta(conf2->theta() - conf1->theta());
    if (cfg_->trajectory.exact_arc_length && angle_diff != 0)
    {
        double radius =  dist/(2*sin(angle_diff/2));
        dist = fabs( angle_diff * radius ); // actual arg length!
    }
    double vel = dist / deltaT->estimate();

    // vel *= g2o::sign(deltaS[0]*cos(conf1->theta()) + deltaS[1]*sin(conf1->theta())); // consider direction
    vel *= fast_sigmoid( 100 * (deltaS.x()*cos(conf1->theta()) + deltaS.y()*sin(conf1->theta())) ); // consider direction

    const double omega = angle_diff / deltaT->estimate();

    _error[0] = std::abs(vel   - _measurement->translation());
    _error[1] = std::abs(omega - _measurement->rotation());

    ROS_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]), "EdgeProfileFidelity::computeError() _error[0]=%f _error[1]=%f\n",_error[0],_error[1]);
  }

  /**
   * @brief Set pointer to associated reference velocity for the underlying cost function
   * @param velocity Velocity instance containing the reference translational and rotational velocity components
   */
  void setVelocity(const Velocity* velocity)
  {
    _measurement = velocity;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param velocity Velocity instance containing the reference translational and rotational velocity components
   */
  void setParameters(const TebConfig& cfg, const Velocity* velocity)
  {
    cfg_ = &cfg;
    _measurement = velocity;
  }

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace elastic_band

#endif /* _EDGE_PROFILE_FIDELITY_HPP_ */
