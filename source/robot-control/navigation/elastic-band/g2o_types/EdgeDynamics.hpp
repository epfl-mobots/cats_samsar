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

#ifndef _EDGE_DYNAMICS_HPP_
#define _EDGE_DYNAMICS_HPP_

#include <elastic-band/g2o_types/BaseTebEdges.hpp>
#include <elastic-band/g2o_types/VertexPose.hpp>
#include <elastic-band/g2o_types/VertexTimeDiff.hpp>
#include <elastic-band/g2o_types/Penalties.hpp>
#include <elastic-band/kinematics/Timestamp.hpp>
#include <elastic-band/kinematics/Velocity.hpp>
#include <elastic-band/TebConfig.hpp>

namespace elastic_band
{

/**
 * @class EdgeDynamics
 * @brief Edge defining the cost function for limiting the dynamical constraints and minimizing the dynamical objectives.
 * 
 * The edge concatenates the following three edges: EdgeVelocity, EdgeAcceleration, EdgeProfileFidelity
 * @see TebPlanner::AddEdgesDynamics
 * @see EdgeVelocity
 * @see EdgeAcceleration
 * @see EdgeProfileFidelity
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeDynamics : public BaseTebMultiEdge<12, std::pair<std::pair<Velocity, Velocity>, std::pair<Timestep, Timestep>>>
{
public:

  /**
   * @brief Construct edge
   */
  EdgeDynamics()
  {
    _measurement = std::pair<std::pair<Velocity, Velocity>, std::pair<Timestep, Timestep>>();
    this->resize(5);
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeDynamics()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    const Eigen::Vector2d diff2 = pose3->position() - pose2->position();

    double dist1 = diff1.norm();
    double dist2 = diff2.norm();
    const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
    const double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());

    if (cfg_->trajectory.exact_arc_length) // use exact arc length instead of Euclidean approximation
    {
      if (angle_diff1 != 0)
      {
        const double radius = dist1 / (2 * std::sin(angle_diff1 / 2));
        dist1 = std::abs(angle_diff1 * radius);
      }
      if (angle_diff2 != 0)
      {
        const double radius = dist2 / (2 * std::sin(angle_diff2 / 2));
        dist2 = std::abs(angle_diff2 * radius);
      }
    }

    const double delta_t1 = dt1->dt();
    const double delta_t2 = dt2->dt();
    const double dt_sum = delta_t1 + delta_t2;

    double vel_lin1 = dist1 / delta_t1;
    double vel_lin2 = dist2 / delta_t2;

    vel_lin1 *= g2o::sign(diff1.x() * std::cos(pose1->theta()) + diff1.y() * std::sin(pose1->theta()));
    vel_lin2 *= g2o::sign(diff2.x() * std::cos(pose2->theta()) + diff2.y() * std::sin(pose2->theta()));
//    vel_lin1 *= fast_sigmoid(100 * (diff1.x() * std::cos(pose1->theta()) + diff1.y() * std::sin(pose1->theta())));
//    vel_lin2 *= fast_sigmoid(100 * (diff2.x() * std::cos(pose2->theta()) + diff2.y() * std::sin(pose2->theta())));

    const double vel_rot1 = angle_diff1 / delta_t1;
    const double vel_rot2 = angle_diff2 / delta_t2;

    const double acc_lin = 2 * (vel_lin2 - vel_lin1) / dt_sum;
    const double acc_rot = 2 * (vel_rot2 - vel_rot1) / dt_sum;

    _error[ 0] = penaltyBoundToInterval(acc_lin,                                    cfg_->robot.acc_lim_x,     cfg_->optim.penalty_epsilon);
    _error[ 1] = penaltyBoundToInterval(acc_rot,                                    cfg_->robot.acc_lim_theta, cfg_->optim.penalty_epsilon);

    _error[ 2] = penaltyBoundToInterval(vel_lin1, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x,     cfg_->optim.penalty_epsilon);
    _error[ 3] = penaltyBoundToInterval(vel_rot1,                                   cfg_->robot.max_vel_theta, cfg_->optim.penalty_epsilon);

    _error[ 4] = penaltyBoundToInterval(vel_lin2, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x,     cfg_->optim.penalty_epsilon);
    _error[ 5] = penaltyBoundToInterval(vel_rot2,                                   cfg_->robot.max_vel_theta, cfg_->optim.penalty_epsilon);

    _error[ 6] = std::abs(vel_lin1 - _measurement.first  .first.translation());
    _error[ 7] = std::abs(vel_rot1 - _measurement.first  .first.rotation());
    _error[ 8] = std::abs(delta_t1 - _measurement.second .first.count());

    _error[ 9] = std::abs(vel_lin2 - _measurement.first .second.translation());
    _error[10] = std::abs(vel_rot2 - _measurement.first .second.rotation());
    _error[11] = std::abs(delta_t2 - _measurement.second.second.count());

    ROS_ASSERT_MSG(std::isfinite(_error[ 0]), "EdgeDynamics::computeError()      acc_lin : _error[ 0]=%f\n", _error[ 0]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 1]), "EdgeDynamics::computeError()      acc_rot : _error[ 1]=%f\n", _error[ 1]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 2]), "EdgeDynamics::computeError()      vel_lin1: _error[ 2]=%f\n", _error[ 2]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 3]), "EdgeDynamics::computeError()      vel_rot1: _error[ 3]=%f\n", _error[ 3]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 4]), "EdgeDynamics::computeError()      vel_lin2: _error[ 4]=%f\n", _error[ 4]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 5]), "EdgeDynamics::computeError()      vel_rot2: _error[ 5]=%f\n", _error[ 5]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 6]), "EdgeDynamics::computeError() diff_vel_lin1: _error[ 6]=%f\n", _error[ 6]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 7]), "EdgeDynamics::computeError() diff_vel_rot1: _error[ 7]=%f\n", _error[ 7]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 8]), "EdgeDynamics::computeError() diff_delta_t1: _error[ 8]=%f\n", _error[ 8]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 9]), "EdgeDynamics::computeError() diff_vel_lin2: _error[ 9]=%f\n", _error[ 9]);
    ROS_ASSERT_MSG(std::isfinite(_error[10]), "EdgeDynamics::computeError() diff_vel_rot2: _error[10]=%f\n", _error[10]);
    ROS_ASSERT_MSG(std::isfinite(_error[11]), "EdgeDynamics::computeError() diff_delta_t2: _error[11]=%f\n", _error[11]);
  }

  /**
   * @brief Set associated reference velocities for the underlying profile fidelity cost function
   * @param velocities Velocity instances containing the reference translational and rotational velocities components
   */
  void setVelocities(std::pair<Velocity, Velocity> velocities)
  {
    _measurement.first = velocities;
  }

  /**
   * @brief Set associated reference timesteps for the underlying profile fidelity cost function
   * @param timesteps Timestep instances containing the reference time differences between both pairs of vertex poses
   */
  void setTimesteps(std::pair<Timestep, Timestep> timesteps)
  {
    _measurement.second = timesteps;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig instance storing configuration parameters
   * @param velocities Velocity instances containing the reference translational and rotational velocities components
   * @param timesteps  Timestep instances containing the reference time differences between both pairs of vertex poses
   */
  void setParameters(const TebConfig& cfg, std::pair<Velocity, Velocity> velocities, std::pair<Timestep, Timestep> timesteps)
  {
    cfg_ = &cfg;
    _measurement = std::pair<std::pair<Velocity, Velocity>, std::pair<Timestep, Timestep>>(velocities, timesteps);
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/**
 * @class EdgeDynamicsHolonomic
 * @brief Edge defining the cost function for limiting the holonomic dynamical constraints and minimizing the holonomic dynamical objectives.
 * 
 * The edge concatenates the following three edges: EdgeVelocityHolonomic, EdgeAccelerationHolonomic, EdgeProfileFidelity
 * @see TebPlanner::AddEdgesDynamics
 * @see EdgeVelocityHolonomic
 * @see EdgeAccelerationHolonomic
 * @see EdgeProfileFidelity
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeDynamicsHolonomic : public BaseTebMultiEdge<17, std::pair<std::pair<Velocity, Velocity>, std::pair<Timestep, Timestep>>>
{
public:

  /**
   * @brief Construct edge
   */
  EdgeDynamicsHolonomic()
  {
    _measurement = std::pair<std::pair<Velocity, Velocity>, std::pair<Timestep, Timestep>>();
    this->resize(5);
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeDynamics()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    const Eigen::Vector2d diff2 = pose3->position() - pose2->position();

    const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
    const double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());

    const double cos_theta1 = std::cos(pose1->theta());
    const double sin_theta1 = std::sin(pose1->theta());
    const double cos_theta2 = std::cos(pose2->theta());
    const double sin_theta2 = std::sin(pose2->theta());

    const double p1_dx =  cos_theta1 * diff1.x() + sin_theta1 * diff1.y();
    const double p1_dy = -sin_theta1 * diff1.x() + cos_theta1 * diff1.y();
    const double p2_dx =  cos_theta2 * diff2.x() + sin_theta2 * diff2.y();
    const double p2_dy = -sin_theta2 * diff2.x() + cos_theta2 * diff2.y();

    const double delta_t1 = dt1->dt();
    const double delta_t2 = dt2->dt();
    const double dt_sum = delta_t1 + delta_t2;

    const double vel_lin1 = p1_dx / delta_t1;
    const double vel_lin2 = p2_dx / delta_t2;
    
    const double vel_str1 = p1_dy / delta_t1;
    const double vel_str2 = p2_dy / delta_t2;

    const double vel_rot1 = angle_diff1 / delta_t1;
    const double vel_rot2 = angle_diff2 / delta_t2;

    const double acc_lin = 2 * (vel_lin2 - vel_lin1) / dt_sum;
    const double acc_str = 2 * (vel_str2 - vel_str1) / dt_sum;
    const double acc_rot = 2 * (vel_rot2 - vel_rot1) / dt_sum;

    _error[ 0] = penaltyBoundToInterval(acc_lin,                                    cfg_->robot.acc_lim_x,     cfg_->optim.penalty_epsilon);
    _error[ 1] = penaltyBoundToInterval(acc_str,                                    cfg_->robot.acc_lim_y,     cfg_->optim.penalty_epsilon);
    _error[ 2] = penaltyBoundToInterval(acc_rot,                                    cfg_->robot.acc_lim_theta, cfg_->optim.penalty_epsilon);

    _error[ 3] = penaltyBoundToInterval(vel_lin1, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x,     cfg_->optim.penalty_epsilon);
    _error[ 4] = penaltyBoundToInterval(vel_str1,                                   cfg_->robot.max_vel_y,     0.0);
    _error[ 5] = penaltyBoundToInterval(vel_rot1,                                   cfg_->robot.max_vel_theta, cfg_->optim.penalty_epsilon);

    _error[ 6] = penaltyBoundToInterval(vel_lin2, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x,     cfg_->optim.penalty_epsilon);
    _error[ 7] = penaltyBoundToInterval(vel_str2,                                   cfg_->robot.max_vel_y,     0.0);
    _error[ 8] = penaltyBoundToInterval(vel_rot2,                                   cfg_->robot.max_vel_theta, cfg_->optim.penalty_epsilon);

    _error[ 9] = std::abs(vel_lin1 - _measurement.first  .first.translation());
    _error[10] = std::abs(vel_str1 - _measurement.first  .first.strafing());
    _error[11] = std::abs(vel_rot1 - _measurement.first  .first.rotation());
    _error[12] = std::abs(delta_t1 - _measurement.second .first.count());

    _error[13] = std::abs(vel_lin2 - _measurement.first .second.translation());
    _error[14] = std::abs(vel_str2 - _measurement.first .second.strafing());
    _error[15] = std::abs(vel_rot2 - _measurement.first .second.rotation());
    _error[16] = std::abs(delta_t2 - _measurement.second.second.count());

    ROS_ASSERT_MSG(std::isfinite(_error[ 0]), "EdgeDynamics::computeError()      acc_lin : _error[ 0]=%f\n", _error[ 0]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 1]), "EdgeDynamics::computeError()      acc_str : _error[ 1]=%f\n", _error[ 1]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 2]), "EdgeDynamics::computeError()      acc_rot : _error[ 2]=%f\n", _error[ 2]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 3]), "EdgeDynamics::computeError()      vel_lin1: _error[ 3]=%f\n", _error[ 3]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 4]), "EdgeDynamics::computeError()      vel_str1: _error[ 4]=%f\n", _error[ 4]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 5]), "EdgeDynamics::computeError()      vel_rot1: _error[ 5]=%f\n", _error[ 5]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 6]), "EdgeDynamics::computeError()      vel_lin2: _error[ 6]=%f\n", _error[ 6]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 7]), "EdgeDynamics::computeError()      vel_str2: _error[ 7]=%f\n", _error[ 7]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 8]), "EdgeDynamics::computeError()      vel_rot2: _error[ 8]=%f\n", _error[ 8]);
    ROS_ASSERT_MSG(std::isfinite(_error[ 9]), "EdgeDynamics::computeError() diff_vel_lin1: _error[ 9]=%f\n", _error[ 9]);
    ROS_ASSERT_MSG(std::isfinite(_error[10]), "EdgeDynamics::computeError() diff_vel_str1: _error[10]=%f\n", _error[10]);
    ROS_ASSERT_MSG(std::isfinite(_error[11]), "EdgeDynamics::computeError() diff_vel_rot1: _error[11]=%f\n", _error[11]);
    ROS_ASSERT_MSG(std::isfinite(_error[12]), "EdgeDynamics::computeError() diff_delta_t1: _error[12]=%f\n", _error[12]);
    ROS_ASSERT_MSG(std::isfinite(_error[13]), "EdgeDynamics::computeError() diff_vel_lin2: _error[13]=%f\n", _error[13]);
    ROS_ASSERT_MSG(std::isfinite(_error[14]), "EdgeDynamics::computeError() diff_vel_str2: _error[14]=%f\n", _error[14]);
    ROS_ASSERT_MSG(std::isfinite(_error[15]), "EdgeDynamics::computeError() diff_vel_rot2: _error[15]=%f\n", _error[15]);
    ROS_ASSERT_MSG(std::isfinite(_error[16]), "EdgeDynamics::computeError() diff_delta_t2: _error[16]=%f\n", _error[16]);
  }

  /**
   * @brief Set associated reference velocities for the underlying profile fidelity cost function
   * @param velocities Velocity instances containing the reference translational and rotational velocities components
   */
  void setVelocities(std::pair<Velocity, Velocity> velocities)
  {
    _measurement.first = velocities;
  }

  /**
   * @brief Set associated reference timesteps for the underlying profile fidelity cost function
   * @param timesteps Timestep instances containing the reference time differences between both pairs of vertex poses
   */
  void setTimesteps(std::pair<Timestep, Timestep> timesteps)
  {
    _measurement.second = timesteps;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig instance storing configuration parameters
   * @param velocities Velocity instances containing the reference translational and rotational velocities components
   * @param timesteps  Timestep instances containing the reference time differences between both pairs of vertex poses
   */
  void setParameters(const TebConfig& cfg, std::pair<Velocity, Velocity> velocities, std::pair<Timestep, Timestep> timesteps)
  {
    cfg_ = &cfg;
    _measurement = std::pair<std::pair<Velocity, Velocity>, std::pair<Timestep, Timestep>>(velocities, timesteps);
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace elastic_band

#endif /* _EDGE_DYNAMICS_HPP_ */
