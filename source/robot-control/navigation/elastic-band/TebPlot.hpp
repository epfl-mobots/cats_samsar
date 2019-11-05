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
 * Author: Christoph Rösmann
 * Adaptation: Mickaël Salamin, Fall 2019
 * Implementation: ROS package teb_local_planner
 * Source: https://github.com/rst-tu-dortmund/teb_local_planner/
 *********************************************************************/

#ifndef _TEB_PLOT_HPP_
#define _TEB_PLOT_HPP_

#include <elastic-band/kinematics/Trajectory.hpp>

#include <QtWidgets/QMainWindow>

namespace elastic_band
{

/**
 * @class TebPlot
 * @brief Plot trajectory profiles
 */
class TebPlot
{
private:

    /**
     * @brief Default constructor (non-instantiable class)
     */
    TebPlot() {}

public:

    /** @name Utility functions */
    //@{

    /**
     * @brief Close the specified window and delete its content
     * @param window Parent widget to be closed
     * @return Window pointing to nullptr
     */
    static QMainWindow* closeWindow(QMainWindow* window);

    //@}

    /** @name Generate geometric plots */
    //@{

    /**
     * @brief Plot the path of the specified trajectory
     * @param trajectory Trajectory from which to extract the path
     * @param window Widget in which to plot the path
     * @param title Title to name the plot
     * @param range Range of the plot
     * @param size Size of the plot
     * @return Window containing the chart view
     */
    static QMainWindow* plotPath(const Trajectory* trajectory, QMainWindow* window = nullptr, const QString title = QString("Path"),
                                 const QPair<double, double> range = QPair<double, double>(-1, +1), const QPair<int, int> size = QPair<int, int>(800, 800));

    //@}

    /** @name Generate profile plots */
    //@{

    /**
     * @brief Plot the profile of the specified trajectory data
     * @param trajectory Trajectory from which to extract the profile
     * @param window Widget in which to plot the profile
     * @param title Title to name the plot
     * @param data Data to include in the plot
     * @param legends Legends of the chart series
     * @param limited Whether the chart series are limits
     * @param limits Limit values of the chart series that are limited
     * @param links Links between non-limit chart series and their limits
     * @param axisY Vertical axis label
     * @param axisX Horizontal axis label
     * @param size Size of the window
     * @return Window containing the chart view
     */
    static QMainWindow* plotProfile(const Trajectory* trajectory, QMainWindow* window = nullptr, const QString title = QString("Profile"),
                                    const QList<QVector<double>> data = QList<QVector<double>>(), const QList<QString> legends = QList<QString>(),
                                    const QList<bool> limited = QList<bool>(), const QList<double> limits = QList<double>(), const QList<size_t> links = QList<size_t>(),
                                    const QString axisY = QString("Data"), const QString axisX = QString("Time [ms]"), const QPair<int, int> size = QPair<int, int>(800, 600));

    /**
     * @brief Plot the pose profile of the specified trajectory
     * @param trajectory Trajectory from which to extract the profile
     * @param window Widget in which to plot the profile
     * @param title Title to name the plot
     * @return Window containing the chart view
     */
    static QMainWindow* plotProfilePose(const Trajectory* trajectory, QMainWindow* window = nullptr, const QString title = QString("Pose profile"));

    /**
     * @brief Plot the speed profile of the specified trajectory
     * @param trajectory Trajectory from which to extract the profile
     * @param window Widget in which to plot the profile
     * @param title Title to name the plot
     * @return Window containing the chart view
     */
    static QMainWindow* plotProfileSpeed(const Trajectory* trajectory, QMainWindow* window = nullptr, const QString title = QString("Speed profile"));

    /**
     * @brief Plot the velocity profile of the specified trajectory
     * @param trajectory Trajectory from which to extract the profile
     * @param window Widget in which to plot the profile
     * @param title Title to name the plot
     * @return Window containing the chart view
     */
    static QMainWindow* plotProfileVelocity(const Trajectory* trajectory, QMainWindow* window = nullptr, const QString title = QString("Velocity profile"));

    /**
     * @brief Plot the acceleration profile of the specified trajectory
     * @param trajectory Trajectory from which to extract the profile
     * @param window Widget in which to plot the profile
     * @param title Title to name the plot
     * @return Window containing the chart view
     */
    static QMainWindow* plotProfileAcceleration(const Trajectory* trajectory, QMainWindow* window = nullptr, const QString title = QString("Acceleration profile"));

    //@}

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared instances of TebPlot
typedef boost::shared_ptr<TebPlot> TebPlotPtr;
//! Abbrev. for shared instances of TebPlot (read-only)
typedef boost::shared_ptr<const TebPlot> TebPlotConstPtr;

} // namespace elastic_band

#endif /* _TEB_PLOT_HPP_ */
