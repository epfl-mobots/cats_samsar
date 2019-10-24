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

#ifndef _TRAJECTORY_HPP_
#define _TRAJECTORY_HPP_

#include <elastic-band/kinematics/PoseSE2.hpp>
#include <elastic-band/kinematics/Velocity.hpp>
#include <elastic-band/kinematics/Acceleration.hpp>
#include <elastic-band/kinematics/Timestamp.hpp>

#include <Eigen/Core>

namespace elastic_band
{

/**
  * @class Point
  * @brief This class implements a point.
  * The point consists of a pose, a velocity, an acceleration and a timestamp.
  */
class Point
{
public:

    /** @name Construct Point instances */
    ///@{

    /**
    * @brief Default constructor
    */
    Point()
    {
        setZero();
    }

    /**
    * @brief Overloaded constructor
    * @param pose pose of the point
    * @param velocity velocity of the point
    * @param acceleration acceleration of the point
    * @param timestamp timestamp of the point
    */
    Point(const PoseSE2 pose, const Velocity velocity, const Acceleration acceleration, const Timestamp timestamp)
    {
        _pose         = pose;
        _velocity     = velocity;
        _acceleration = acceleration;
        _timestamp    = timestamp;
    }

    /**
    * @brief Copy constructor
    * @param point Point instance
    */
    Point(const Point& point)
    {
        _pose         = point._pose;
        _velocity     = point._velocity;
        _acceleration = point._acceleration;
        _timestamp    = point._timestamp;
    }

    ///@}


    /**
    * @brief Destruct Point
    */ 
    ~Point() {}


    /** @name Access and modify values */
    ///@{

    /**
    * @brief Access the pose of the point
    * @return reference to the pose point
    */
    PoseSE2& pose() {return _pose;}

    /**
    * @brief Access the pose of the point (read-only)
    * @return const reference to the pose of the point
    */
    const PoseSE2& pose() const {return _pose;}

    /**
    * @brief Access the velocity of the point
    * @return reference to the velocity point
    */
    Velocity& velocity() {return _velocity;}

    /**
    * @brief Access the velocity of the point (read-only)
    * @return const reference to the velocity of the point
    */
    const Velocity& velocity() const {return _velocity;}

    /**
    * @brief Access the acceleration of the point
    * @return reference to the acceleration point
    */
    Acceleration& acceleration() {return _acceleration;}

    /**
    * @brief Access the acceleration of the point (read-only)
    * @return const reference to the acceleration of the point
    */
    const Acceleration& acceleration() const {return _acceleration;}

    /**
    * @brief Access the timestamp of the point
    * @return reference to the timestamp point
    */
    Timestamp& timestamp() {return _timestamp;}

    /**
    * @brief Access the timestamp of the point (read-only)
    * @return const reference to the timestamp of the point
    */
    const Timestamp& timestamp() const {return _timestamp;}

    /**
    * @brief Set point to zero
    */
    void setZero()
    {
        _pose.setZero();
        _velocity.setZero();
        _acceleration.setZero();
        _timestamp.setZero();
    }

    ///@}


    /** @name Operator overloads / Allow some arithmetic operations */
    ///@{

    /**
    * @brief Assignment operator
    * @param rhs Point instance
    * @todo exception safe version of the assignment operator
    */
    Point& operator=(const Point& rhs)
    {
        if (&rhs != this)
        {
            _pose         = rhs._pose;
            _velocity     = rhs._velocity;
            _acceleration = rhs._acceleration;
            _timestamp    = rhs._timestamp;
        }
        return *this;
    }

    /**
     * @brief Output stream operator
     * @param stream output stream
     * @param point to be used
     */
    friend std::ostream& operator<<(std::ostream& stream, const Point& point)
    {
        stream << "Point = ";
        stream << "pose: (" << point._pose << "), ";
        stream << "velocity: (" << point._velocity << "), ";
        stream << "acceleration: (" << point._acceleration << "), ";
        stream << "timestamp: (" << point._timestamp << ")";
        return stream;
    }

    ///@}

private:

    PoseSE2      _pose;
    Velocity     _velocity;
    Acceleration _acceleration;
    Timestamp    _timestamp;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared instances of Point
typedef boost::shared_ptr<Point> PointPtr;
//! Abbrev. for shared const Point pointers
typedef boost::shared_ptr<const Point> PointConstPtr;
//! Abbrev. for containers storing multiple Point pointers
typedef std::vector<PointPtr> PointContainer;


/**
  * @class Trajectory
  * @brief This class implements a trajectory.
  * The trajectory consists of a series of points.
  */
class Trajectory
{
public:

    /**
     * @struct RobotParameters
     * @brief This structure implements robot parameters necessary for effective trajectory definition.
     */
    typedef struct RobotParameters {
        double wheel_radius;   //!< Wheel radius
        double wheel_distance; //!< Inter-wheel distance
    } RobotParameters;

public:

    /** @name Construct Trajectory instances */
    ///@{

    /**
    * @brief Default constructor
    */
    Trajectory() {}

    /**
    * @brief Overloaded constructor
    * @param trajectory vector of Point instances constituting the trajectory
    * @param robot_parameters instance of struct RobotParameters containing the robot parameters
    */
    Trajectory(const PointContainer trajectory, const RobotParameters robot_parameters)
    {
        _trajectory       = trajectory;
        _robot_parameters = robot_parameters;
    }

    /**
    * @brief Copy constructor
    * @param trajectory Trajectory instance
    */
    Trajectory(const Trajectory& trajectory)
    {
        _trajectory       = trajectory._trajectory;
        _robot_parameters = trajectory._robot_parameters;
    }

    ///@}


    /**
    * @brief Destruct Trajectory
    */ 
    ~Trajectory() {}


    /** @name Access and modify values */
    ///@{

    /**
    * @brief Access the trajectory
    * @return reference to the trajectory
    */
    PointContainer& trajectory() {return _trajectory;}

    /**
    * @brief Access the trajectory (read-only)
    * @return const reference to the trajectory
    */
    const PointContainer& trajectory() const {return _trajectory;}

    /**
    * @brief Access the robot parameters
    * @return reference to the robot parameters
    */
    RobotParameters& robotParameters() {return _robot_parameters;}

    /**
    * @brief Access the robot parameters (read-only)
    * @return const reference to the robot parameters
    */
    const RobotParameters& robotParameters() const {return _robot_parameters;}

    /**
    * @brief Access the pose profile
    * @return vector of PoseSE2 instances constituting the pose profile
    */
    PoseSE2Container getProfilePose() const
    {
        PoseSE2Container profile(_trajectory.size());
        for (unsigned int i = 0; i < profile.size(); i++) {
            *profile.at(i) = _trajectory.at(i)->pose();
        }
        return profile;
    }

    /**
    * @brief Access the velocity profile
    * @return vector of Velocity instances constituting the velocity profile
    */
    VelocityContainer getProfileVelocity() const
    {
        VelocityContainer profile(_trajectory.size() - 1);
        for (unsigned int i = 0; i < profile.size(); i++) {
            *profile.at(i) = _trajectory.at(i)->velocity();
        }
        return profile;
    }

    /**
    * @brief Access the acceleration profile
    * @return vector of Acceleration instances constituting the acceleration profile
    */
    AccelerationContainer getProfileAcceleration() const
    {
        AccelerationContainer profile(_trajectory.size() - 2);
        for (unsigned int i = 0; i < profile.size(); i++) {
            *profile.at(i) = _trajectory.at(i)->acceleration();
        }
        return profile;
    }

    /**
    * @brief Access the timestamp profile
    * @return vector of Timestamp instances constituting the timestamp profile
    */
    TimestampContainer getProfileTimestamp() const
    {
        TimestampContainer profile(_trajectory.size());
        for (unsigned int i = 0; i < profile.size(); i++) {
            *profile.at(i) = _trajectory.at(i)->timestamp();
        }
        return profile;
    }

    /**
    * @brief Access the timestep profile
    * @return vector of Timestep instances constituting the timestep profile
    */
    TimestepContainer getProfileTimestep() const
    {
        TimestepContainer profile(_trajectory.size() - 1);
        for (unsigned int i = 0; i < profile.size(); i++) {
            *profile.at(i) = _trajectory.at(i+1)->timestamp() - _trajectory.at(i)->timestamp();
        }
        return profile;
    }

    /**
    * @brief Modify the pose profile
    * @param profile vector of PoseSE2 instances constituting the pose profile
    * @param update whether to update the other trajectory profiles
    */
    void setProfilePose(const PoseSE2Container profile, const bool update = true)
    {
        _trajectory.resize(profile.size());
        for (unsigned int i = 0; i < profile.size(); i++) {
            _trajectory.at(i)->pose() = *profile.at(i);
        }
        if (update) {
            derivePose();
            deriveVelocity();
        }
    }

    /**
    * @brief Modify the velocity profile
    * @param profile vector of Velocity instances constituting the velocity profile
    * @param update whether to update the other trajectory profiles
    */
    void setProfileVelocity(const VelocityContainer profile, const bool update = true, const PoseSE2 initPose = PoseSE2())
    {
        _trajectory.resize(profile.size() + 1);
        for (unsigned int i = 0; i < profile.size(); i++) {
            _trajectory.at(i)->velocity() = *profile.at(i);
        }
        if (update) {
            deriveVelocity();
            integrateVelocity(initPose);
        }
    }

    /**
    * @brief Modify the acceleration profile
    * @param profile vector of Acceleration instances constituting the acceleration profile
    * @param update whether to update the other trajectory profiles
    */
    void setProfileAcceleration(const AccelerationContainer profile, const bool update = true, const PoseSE2 initPose = PoseSE2(), const Velocity initVelocity = Velocity())
    {
        _trajectory.resize(profile.size() + 2);
        for (unsigned int i = 0; i < profile.size(); i++) {
            _trajectory.at(i)->acceleration() = *profile.at(i);
        }
        if (update) {
            integrateAcceleration(initVelocity);
            integrateVelocity(initPose);
        }
    }

    /**
    * @brief Modify the timestamp profile
    * @param profile vector of milliseconds constituting the timestamp profile
    * @param update whether to update the other trajectory profiles
    */
    void setProfileTimestamp(const TimestampContainer profile, const bool update = true)
    {
        _trajectory.resize(profile.size());
        for (unsigned int i = 0; i < profile.size(); i++) {
            _trajectory.at(i)->timestamp() = *profile.at(i);
        }
        if (update) {
            derivePose();
            deriveVelocity();
        }
    }

    /**
    * @brief Modify the timestep profile
    * @param profile vector of milliseconds constituting the timestep profile
    * @param update whether to update the other trajectory profiles
    */
    void setProfileTimestep(const TimestepContainer profile, const bool update = true)
    {
        _trajectory.resize(profile.size() + 1);
        _trajectory.front()->timestamp() = Timestamp::zero();
        for (unsigned int i = 0; i < profile.size(); i++) {
            _trajectory.at(i+1)->timestamp() = *profile.at(i) + _trajectory.at(i)->timestamp();
        }
        if (update) {
            derivePose();
            deriveVelocity();
        }
    }

    ///@}


    /** @name Arithmetic operations for which operators are not always reasonable */
    ///@{

    /**
    * @brief Compute the velocity profile as the derivative of the pose profile
    */
    void derivePose()
    {
        double r, d, v, w, o;
        Eigen::Vector2d p;
        double a;
        double sx, sy, vx, vy;
        TimestepContainer timesteps = getProfileTimestep();
        for (unsigned int i = 0; i < _trajectory.size() - 1; i++) {
            r = _robot_parameters.wheel_radius;
            d = _robot_parameters.wheel_distance;
            p = _trajectory.at(i+1)->pose().position() - _trajectory.at(i)->pose().position();
            a = _trajectory.at(i+1)->pose().theta()    - _trajectory.at(i)->pose().theta();
            sx = p.x() * std::cos(_trajectory.at(i)->pose().theta());
            sy = p.y() * std::sin(_trajectory.at(i)->pose().theta());
            vx = p.x() / timesteps.at(i)->count();
            vy = p.y() / timesteps.at(i)->count();
            v = g2o::sign(sx + sy) * p.norm() / timesteps.at(i)->count();
            w = g2o::normalize_theta(a)       / timesteps.at(i)->count();
            o = g2o::normalize_theta(std::atan2(vx, vy));
            _trajectory.at(i)->velocity() = Velocity(r, d, v, w, o);
        }
    }

    /**
    * @brief Compute the acceleration profile as the derivative of the velocity profile
    */
    void deriveVelocity()
    {
        double vdot, wdot;
        TimestepContainer timesteps = getProfileTimestep();
        for (unsigned int i = 0; i < _trajectory.size() - 2; i++) {
            vdot = 2 * (_trajectory.at(i+1)->velocity().translation() - _trajectory.at(i)->velocity().translation()) / (timesteps.at(i+1)->count() + timesteps.at(i)->count());
            wdot = 2 * (_trajectory.at(i+1)->velocity().rotation()    - _trajectory.at(i)->velocity().rotation()   ) / (timesteps.at(i+1)->count() + timesteps.at(i)->count());
            _trajectory.at(i)->acceleration() = Acceleration(vdot, wdot);
        }
    }

    /**
    * @brief Compute the pose profile as the integral of the velocity profile
    */
    void integrateVelocity(const PoseSE2 initPose)
    {
        double x, y, t;
        TimestepContainer timesteps = getProfileTimestep();
        _trajectory.front()->pose() = initPose;
        for (unsigned int i = 0; i < _trajectory.size() - 1; i++) {
            t = _trajectory.at(i)->velocity().rotation()                  * timesteps.at(i)->count() + _trajectory.at(i)->pose().theta();
            x = _trajectory.at(i)->velocity().translation() * std::cos(t) * timesteps.at(i)->count() + _trajectory.at(i)->pose().x();
            y = _trajectory.at(i)->velocity().translation() * std::sin(t) * timesteps.at(i)->count() + _trajectory.at(i)->pose().y();
            t = g2o::normalize_theta(t);
            _trajectory.at(i+1)->pose() = PoseSE2(x, y, t);
        }
    }

    /**
    * @brief Compute the velocity profile as the integral of the acceleration profile
    */
    void integrateAcceleration(const Velocity initVelocity)
    {
        double r, d, v, w, o;
        TimestepContainer timesteps = getProfileTimestep();
        _trajectory.front()->velocity() = initVelocity;
        for (unsigned int i = 0; i < _trajectory.size() - 2; i++) {
            r = _robot_parameters.wheel_radius;
            d = _robot_parameters.wheel_distance;
            v = _trajectory.at(i)->acceleration().translation() * (timesteps.at(i+1)->count() + timesteps.at(i)->count()) / 2 + _trajectory.at(i)->velocity().translation();
            w = _trajectory.at(i)->acceleration().rotation()    * (timesteps.at(i+1)->count() + timesteps.at(i)->count()) / 2 + _trajectory.at(i)->velocity().rotation();
            o = g2o::normalize_theta(w                          * (                             timesteps.at(i)->count())     + _trajectory.at(i)->velocity().orientation());
            _trajectory.at(i+1)->velocity() = Velocity(r, d, v, w, o);
        }
    }

    ///@}


    /** @name Operator overloads / Allow some arithmetic operations */
    ///@{

    /**
    * @brief Assignment operator
    * @param rhs Trajectory instance
    * @todo exception safe version of the assignment operator
    */
    Trajectory& operator=(const Trajectory& rhs)
    {
        if (&rhs != this)
        {
            _trajectory = rhs._trajectory;
        }
        return *this;
    }

    /**
     * @brief Output stream operator
     * @param stream output stream
     * @param trajectory to be used
     */
    friend std::ostream& operator<<(std::ostream& stream, const Trajectory& trajectory)
    {
        stream << "Trajectory = [";
        for (PointContainer::const_iterator point = trajectory._trajectory.begin(); point != trajectory._trajectory.end(); point++) {
            stream << *point << std::endl;
        }
        stream << "]";
        return stream;
    }

    ///@}

private:

    PointContainer  _trajectory;
    RobotParameters _robot_parameters;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared instances of Trajectory
typedef boost::shared_ptr<Trajectory> TrajectoryPtr;
//! Abbrev. for shared const Trajectory pointers
typedef boost::shared_ptr<const Trajectory> TrajectoryConstPtr;
//! Abbrev. for containers storing multiple Trajectory pointers
typedef std::vector<TrajectoryPtr> TrajectoryContainer;

} // namespace elastic_band

#endif /* _TRAJECTORY_HPP_ */
