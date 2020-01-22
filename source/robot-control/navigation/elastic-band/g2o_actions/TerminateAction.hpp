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

#ifndef _TERMINATE_ACTION_HPP_
#define _TERMINATE_ACTION_HPP_

#include <g2o/core/g2o_core_api.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/hyper_graph_action.h>

#include <limits>
#include <chrono>

namespace elastic_band
{

using namespace g2o;

/**
 * \brief Stop iterating based on the improvement which is (oldChi - currentChi) / oldChi.
 *
 * Adapted from g2o::SparseOptimizerTerminateAction where the gain (oldChi - currentChi) / currentChi
 * is replaced by the (bounded and normalized) percent improvement (oldChi - currentChi) / oldChi
 * and the end conditions additionally include a minimum significant error
 * as well as a timeout with the resolution of a microsecond.
 * If the improvement  is larger than zero and below the associated threshold, or
 * if the graph error  is larger than zero and below the associated threshold, or
 * if the elapsed time is larger than zero and above the associated threshold, or
 * if the maximum number of iterations is reached, then the optimizer is stopped.
 * Typical usage of this action includes adding it as a postIteration action,
 * by calling addPostIterationAction on a sparse optimizer.
 */
class TerminateAction : public HyperGraphAction
{
public:

    TerminateAction() :
        HyperGraphAction(),
        _improvementThreshold(cst(1e-6)),
        _timeoutThreshold(0),
        _chiThreshold(0),
        _lastChi(0),
        _auxTerminateFlag(false),
        _maxIterations(std::numeric_limits<int>::max()),
        _startingTime(std::chrono::system_clock::now())
    {
    }

    virtual HyperGraphAction* operator()(const HyperGraph* graph, Parameters* parameters = 0)
    {
        assert(dynamic_cast<const SparseOptimizer*>(graph) && "graph is not a SparseOptimizer");
        assert(dynamic_cast<HyperGraphAction::ParametersIteration*>(parameters) && "error casting parameters");

        const SparseOptimizer* optimizer = static_cast<const SparseOptimizer*>(graph);
        HyperGraphAction::ParametersIteration* params = static_cast<HyperGraphAction::ParametersIteration*>(parameters);

        const_cast<SparseOptimizer*>(optimizer)->computeActiveErrors();
        if (params->iteration < 0) {
            // Let the optimizer run for at least one iteration,
            // hence we reset the stop flag
            setOptimizerStopFlag(optimizer, false);
        } else if (params->iteration == 0) {
            // First iteration, we store the chi2 value
            _lastChi = optimizer->activeRobustChi2();
            // Check whether it is worthwhile
            // to continue the optimization
            if (_lastChi <= _chiThreshold) { // Tell the optimizer to stop
                setOptimizerStopFlag(optimizer, true);
            }
        } else {
            // Compute the improvement and stop the optimizer
            // in case the improvement is below the threshold
            // or the allocated task budget has been exhausted
            // or the hypergraph error can fairly be neglected
            // or we reached the maximum number of iterations
            if (isOptimizerStoppable(optimizer, params->iteration)) { // Tell the optimizer to stop
                setOptimizerStopFlag(optimizer, true);
            }
        }
        return this;
    }

    bool isOptimizerStoppable(const SparseOptimizer* optimizer, const int iteration)
    {
        bool stopOptimizer = false;
        const number_t previousChi = _lastChi;
        const number_t currentChi = optimizer->activeRobustChi2();
        _lastChi = currentChi;
        if (iteration < _maxIterations && currentChi > _chiThreshold) {
            const number_t improvement = (previousChi - currentChi) / previousChi;
            if (improvement >= 0 && improvement < _improvementThreshold) {
                stopOptimizer = true;
            } else if (_timeoutThreshold > 0) {
                const std::chrono::time_point<std::chrono::system_clock> currentTime = std::chrono::system_clock::now();
                const number_t elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - _startingTime).count();
                if (elapsedTime >= 0 && elapsedTime > _timeoutThreshold) {
                    stopOptimizer = true;
                }
            }
        } else {
            stopOptimizer = true;
        }
        return stopOptimizer;
    }

    void setOptimizerStopFlag(const SparseOptimizer* optimizer, const bool stop)
    {
        if (optimizer->forceStopFlag()) {
            *(optimizer->forceStopFlag()) = stop;
        } else {
            _auxTerminateFlag = stop;
            const_cast<SparseOptimizer*>(optimizer)->setForceStopFlag(&_auxTerminateFlag);
        }
    }

    number_t improvementThreshold() const {return _improvementThreshold;}
    void setImprovementThreshold(number_t improvementThreshold) {_improvementThreshold = improvementThreshold;}

    number_t timeoutThreshold() const {return _timeoutThreshold;}
    void setTimeoutThreshold(number_t timeoutThreshold) {_timeoutThreshold = timeoutThreshold;}

    number_t chiThreshold() const {return _chiThreshold;}
    void setChiThreshold(number_t chiThreshold) {_chiThreshold = std::abs(chiThreshold);}

    int maxIterations() const {return _maxIterations;}
    void setMaxIterations(int maxIterations) {_maxIterations = maxIterations;}

    void resetTimer() {_startingTime = std::chrono::system_clock::now();}

protected:

    number_t _improvementThreshold;
    number_t _timeoutThreshold;
    number_t _chiThreshold;
    number_t _lastChi;
    bool _auxTerminateFlag;
    int _maxIterations;
    std::chrono::time_point<std::chrono::system_clock> _startingTime;
};

} // namespace elastic_band

#endif /* _TERMINATE_ACTION_HPP_ */
