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

#ifndef _ORIENTATE_ACTION_HPP_
#define _ORIENTATE_ACTION_HPP_

#include <g2o/core/g2o_core_api.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/hyper_graph_action.h>

#include <elastic-band/g2o_types/EdgeKinematics.hpp>
#include <elastic-band/g2o_types/VertexPose.hpp>

namespace elastic_band
{

using namespace g2o;

/**
 * \brief Set orientation of consecutive path configurations
 *        with respect to the heading of their prior neighbor
 *        and to the local curvature constraint.
 */
class OrientateAction;

class OrientateElementAction : public HyperGraphElementAction
{
public:

    OrientateElementAction(const std::string& typeName = "") : HyperGraphElementAction(typeName)
    {
    }

    virtual HyperGraphElementAction* operator()(const HyperGraph::HyperGraphElement* element, Parameters* parameters)
    {
        assert(dynamic_cast<const HyperGraph::HyperGraphElement*>(element) && "element is not instantiated");

        if (typeid(*element).name() != _typeName) {
            return nullptr;
        }

        const EdgeKinematicsDiffDrive* kinematics = static_cast<const EdgeKinematicsDiffDrive*>(element);
        HyperGraphElementAction::Parameters* params = static_cast<HyperGraphElementAction::Parameters*>(parameters);
        (void) params;

        // Get edge's vertices, i.e. two consecutive poses for kinematic constraint
        HyperGraph::VertexContainer vertices = kinematics->vertices();
        assert(vertices.size() == 2 && "wrong number of vertices for the kinematics edge");
        VertexPose* conf1 = static_cast<VertexPose*>(vertices.at(0));
        VertexPose* conf2 = static_cast<VertexPose*>(vertices.at(1));

        // Impose nonholonomicity constraint for differential drive robot,
        // which is equivalent to follow a path of constant local curvature
        Eigen::Vector2d deltaS = conf2->position() - conf1->position();
        const double alpha = std::atan2(deltaS.y(), deltaS.x());;
        conf2->theta() = 2 * alpha - conf1->theta();

        return this;
    }

    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, Parameters* parameters)
    {
        return (*this)(const_cast<const HyperGraph::HyperGraphElement*>(element), parameters);
    }
};

class OrientateAction : public HyperGraphAction
{
public:

    OrientateAction() :
        HyperGraphAction(),
        _orientateElementAction(nullptr),
        _orientateEdgeName("")
    {
    }

    virtual HyperGraphAction* operator()(const HyperGraph* graph, Parameters* parameters = 0)
    {
        assert(dynamic_cast<const SparseOptimizer*>(graph) && "graph is not a SparseOptimizer");
        assert(dynamic_cast<HyperGraphAction::ParametersIteration*>(parameters) && "error casting parameters");

        const SparseOptimizer* optimizer = static_cast<const SparseOptimizer*>(graph);
        HyperGraphAction::ParametersIteration* params = static_cast<HyperGraphAction::ParametersIteration*>(parameters);
        (void) optimizer;

        if (params->iteration > 0) {
            applyAction(const_cast<HyperGraph*>(graph), _orientateElementAction, nullptr, _orientateEdgeName);
        }

        return this;
    }

    OrientateElementAction* orientateElementAction() const {return _orientateElementAction;}
    void setOrientateElementAction(OrientateElementAction* orientateElementAction) {_orientateElementAction = orientateElementAction;}

    std::string orientateEdgeName() const {return _orientateEdgeName;}
    void setOrientateEdgeName(std::string orientateEdgeName) {_orientateEdgeName = orientateEdgeName;}

protected:

    OrientateElementAction* _orientateElementAction;
    std::string _orientateEdgeName;
};

} // namespace elastic_band

#endif /* _ORIENTATE_ACTION_HPP_ */
