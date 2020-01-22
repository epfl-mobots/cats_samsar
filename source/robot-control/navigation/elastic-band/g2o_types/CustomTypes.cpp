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

#include <elastic-band/g2o_types/EdgeAcceleration.hpp>
#include <elastic-band/g2o_types/EdgeDynamicObstacle.hpp>
#include <elastic-band/g2o_types/EdgeDynamics.hpp>
#include <elastic-band/g2o_types/EdgeKinematics.hpp>
#include <elastic-band/g2o_types/EdgeNeighbor.hpp>
#include <elastic-band/g2o_types/EdgeObstacle.hpp>
#include <elastic-band/g2o_types/EdgePreferRotDir.hpp>
#include <elastic-band/g2o_types/EdgeProfileFidelity.hpp>
#include <elastic-band/g2o_types/EdgeShortestPath.hpp>
#include <elastic-band/g2o_types/EdgeTimeOptimal.hpp>
#include <elastic-band/g2o_types/EdgeVelocity.hpp>
#include <elastic-band/g2o_types/EdgeViaPoint.hpp>
#include <elastic-band/g2o_types/VertexPose.hpp>
#include <elastic-band/g2o_types/VertexTimeDiff.hpp>

#include <g2o/core/factory.h>

namespace elastic_band
{

G2O_REGISTER_TYPE(VERTEX_POSE, VertexPose);
G2O_REGISTER_TYPE(VERTEX_TIMEDIFF, VertexTimeDiff);
G2O_REGISTER_TYPE(EDGE_TIME_OPTIMAL, EdgeTimeOptimal);
G2O_REGISTER_TYPE(EDGE_SHORTEST_PATH, EdgeShortestPath);
G2O_REGISTER_TYPE(EDGE_PROFILE_FIDELITY, EdgeProfileFidelity);
G2O_REGISTER_TYPE(EDGE_VELOCITY, EdgeVelocity);
G2O_REGISTER_TYPE(EDGE_VELOCITY_HOLONOMIC, EdgeVelocityHolonomic);
G2O_REGISTER_TYPE(EDGE_ACCELERATION, EdgeAcceleration);
G2O_REGISTER_TYPE(EDGE_ACCELERATION_START, EdgeAccelerationStart);
G2O_REGISTER_TYPE(EDGE_ACCELERATION_GOAL, EdgeAccelerationGoal);
G2O_REGISTER_TYPE(EDGE_ACCELERATION_HOLONOMIC, EdgeAccelerationHolonomic);
G2O_REGISTER_TYPE(EDGE_ACCELERATION_HOLONOMIC_START, EdgeAccelerationHolonomicStart);
G2O_REGISTER_TYPE(EDGE_ACCELERATION_HOLONOMIC_GOAL, EdgeAccelerationHolonomicGoal);
G2O_REGISTER_TYPE(EDGE_DYNAMICS, EdgeDynamics);
G2O_REGISTER_TYPE(EDGE_DYNAMICS_HOLONOMIC, EdgeDynamicsHolonomic);
G2O_REGISTER_TYPE(EDGE_KINEMATICS_DIFF_DRIVE, EdgeKinematicsDiffDrive);
G2O_REGISTER_TYPE(EDGE_KINEMATICS_CARLIKE, EdgeKinematicsCarlike);
G2O_REGISTER_TYPE(EDGE_NEIGHBOR, EdgeNeighbor);
G2O_REGISTER_TYPE(EDGE_OBSTACLE, EdgeObstacle);
G2O_REGISTER_TYPE(EDGE_INFLATED_OBSTACLE, EdgeInflatedObstacle);
G2O_REGISTER_TYPE(EDGE_INFLUENTIAL_OBSTACLE, EdgeInfluentialObstacle);
G2O_REGISTER_TYPE(EDGE_DYNAMIC_OBSTACLE, EdgeDynamicObstacle);
G2O_REGISTER_TYPE(EDGE_VIA_POINT, EdgeViaPoint);
G2O_REGISTER_TYPE(EDGE_PREFER_ROTDIR, EdgePreferRotDir);

} // namespace elastic_band
