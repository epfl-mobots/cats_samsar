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

#ifndef _VELOCITY_HPP_
#define _VELOCITY_HPP_

#include <g2o/stuff/misc.h>

#include <Eigen/Core>

#include <cassert>

namespace elastic_band
{

/**
  * @class Velocity
  * @brief This class implements a velocity in the domain \f$\mathbb{R}^2\f$
  * The velocity consists of a translational part in x and y and a rotational part in z.
  * The velocity can also be expressed in terms of the robot's left and right wheels speed.
  */
class Velocity
{
public:

    /** @name Construct Velocity instances */
    ///@{

    /**
    * @brief Default constructor
    */
    Velocity()
    {
        setZero();
        _wheel_radius    = 1;
        _wheel_distance  = 1;
    }

    /**
    * @brief Overloaded constructor with typical parameters
    * @param r wheel radius
    * @param d inter-wheel distance
    * @param v translational velocity
    * @param w rotational velocity in rad/s
    * @param o orientation in rad
    */
    Velocity(const double r, const double d, const double v, const double w, const double o)
    {
        VelocityRobot(r, d, v, w, o);
    }

    /**
    * @brief Construct Velocity using single components r, d, x, y, z
    * @param r wheel radius
    * @param d inter-wheel distance
    * @param x x-axis translational velocity
    * @param y y-axis translational velocity
    * @param z z-axis rotational velocity in rad/s
    */
    void VelocityAxes(const double r, const double d, const double x, const double y, const double z)
    {
        assert(r > 0 && d > 0);
        _wheel_radius    = r;
        _wheel_distance  = d;
        _vel_orientation = atan2(x, y);
        _vel_translation = (std::cos(_vel_orientation) != 0) ? x / std::cos(_vel_orientation) : y / std::sin(_vel_orientation);
        _vel_rotation    = z;
        _vel_robot = Eigen::Vector3d(x, y, z);
        _vel_wheel = Eigen::Vector2d(_vel_translation/r - _vel_rotation*d/(2*r), _vel_translation/r + _vel_rotation*d/(2*r));
    }

    /**
    * @brief Construct Velocity using single components r, d, v, w, o
    * @param r wheel radius
    * @param d inter-wheel distance
    * @param v translational velocity
    * @param w rotational velocity in rad/s
    * @param o orientation in rad
    */
    void VelocityRobot(const double r, const double d, const double v, const double w, const double o)
    {
        assert(r > 0 && d > 0);
        _wheel_radius    = r;
        _wheel_distance  = d;
        _vel_orientation = o;
        _vel_translation = v;
        _vel_rotation    = w;
        _vel_robot = Eigen::Vector3d(v*std::cos(o), v*std::sin(o), w);
        _vel_wheel = Eigen::Vector2d(v/r - w*d/(2*r), v/r + w*d/(2*r));
    }

    /**
    * @brief Construct Velocity using single components r, d, wl, wr, o
    * @param r wheel radius
    * @param d inter-wheel distance
    * @param wl  left wheel rotational velocity in rad/s
    * @param wr right wheel rotational velocity in rad/s
    * @param o orientation in rad
    */
    void VelocityWheel(const double r, const double d, const double wl, const double wr, const double o)
    {
        assert(r > 0 && d > 0);
        _wheel_radius    = r;
        _wheel_distance  = d;
        _vel_orientation = o;
        _vel_translation = r*(wr+wl)/2;
        _vel_rotation    = r*(wr-wl)/d;
        _vel_robot = Eigen::Vector3d(_vel_translation*std::cos(o), _vel_translation*std::sin(o), _vel_rotation);
        _vel_wheel = Eigen::Vector2d(wl, wr);
    }

    /**
    * @brief Copy constructor
    * @param velocity Velocity instance
    */
    Velocity(const Velocity& velocity)
    {
        _vel_robot       = velocity._vel_robot;
        _vel_wheel       = velocity._vel_wheel;
        _vel_orientation = velocity._vel_orientation;
        _vel_translation = velocity._vel_translation;
        _vel_rotation    = velocity._vel_rotation;
        _wheel_radius    = velocity._wheel_radius;
        _wheel_distance  = velocity._wheel_distance;
    }

    ///@}


    /**
    * @brief Destruct Velocity
    */ 
    ~Velocity() {}


    /** @name Access and modify values */
    ///@{

    /**
    * @brief Access the 2D velocity (translation and rotation) (read-only)
    * @return const 2D velocity (translation and rotation)
    */
    const Eigen::Vector2d velocity2d() const {return Eigen::Vector2d(_vel_translation, _vel_rotation);}

    /**
    * @brief Access the 3D robot viewpoint of the velocity
    * @return reference to the 3D robot velocity
    */
    Eigen::Vector3d& robot() {return _vel_robot;}

    /**
    * @brief Access the 3D robot viewpoint of the velocity (read-only)
    * @return const reference to the 3D robot velocity
    */
    const Eigen::Vector3d& robot() const {return _vel_robot;}

    /**
    * @brief Access the 2D wheel viewpoint of the velocity
    * @return reference to the 2D wheel velocity
    */
    Eigen::Vector2d& wheel() {return _vel_wheel;}

    /**
    * @brief Access the 2D wheel viewpoint of the velocity (read-only)
    * @return const reference to the 2D wheel velocity
    */
    const Eigen::Vector2d& wheel() const {return _vel_wheel;}

    /**
    * @brief Access the orientation of the velocity
    * @return reference to the orientation
    */
    double& orientation() {return _vel_orientation;}

    /**
    * @brief Access the orientation of the velocity (read-only)
    * @return const reference to the orientation
    */
    const double& orientation() const {return _vel_orientation;}

    /**
    * @brief Access the translation part of the velocity
    * @return reference to the translational velocity
    */
    double& translation() {return _vel_translation;}

    /**
    * @brief Access the translation part of the velocity (read-only)
    * @return const reference to the translational velocity
    */
    const double& translation() const {return _vel_translation;}

    /**
    * @brief Access the rotation part of the velocity
    * @return reference to the rotational velocity
    */
    double& rotation() {return _vel_rotation;}

    /**
    * @brief Access the rotation part of the velocity (read-only)
    * @return const reference to the rotational velocity
    */
    const double& rotation() const {return _vel_rotation;}

    /**
    * @brief Access the x-axis translation part of the velocity
    * @return reference to the x-axis translational velocity
    */
    double& x() {return _vel_robot.coeffRef(0);}

    /**
    * @brief Access the x-axis translation part of the velocity (read-only)
    * @return const reference to the x-axis translational velocity
    */
    const double& x() const {return _vel_robot.coeffRef(0);}

    /**
    * @brief Access the y-axis translation part of the velocity
    * @return reference to the y-axis translational velocity
    */
    double& y() {return _vel_robot.coeffRef(1);}

    /**
    * @brief Access the y-axis translation part of the velocity (read-only)
    * @return const reference to the y-axis translational velocity
    */
    const double& y() const {return _vel_robot.coeffRef(1);}

    /**
    * @brief Access the z-axis rotation part of the velocity
    * @return reference to the z-axis rotational velocity
    */
    double& z() {return _vel_robot.coeffRef(2);}

    /**
    * @brief Access the z-axis rotation part of the velocity (read-only)
    * @return const reference to the z-axis rotational velocity
    */
    const double& z() const {return _vel_robot.coeffRef(2);}

    /**
    * @brief Access the orientation of the velocity
    * @return reference to the orientation
    */
    double& o() {return orientation();}

    /**
    * @brief Access the orientation of the velocity (read-only)
    * @return const reference to the orientation
    */
    const double& o() const {return orientation();}

    /**
    * @brief Access the translation part of the velocity
    * @return reference to the ranslational velocity
    */
    double& v() {return translation();}

    /**
    * @brief Access the translation part of the velocity (read-only)
    * @return const reference to the translational velocity
    */
    const double& v() const {return translation();}

    /**
    * @brief Access the rotation part of the velocity
    * @return reference to the rotational velocity
    */
    double& w() {return rotation();}

    /**
    * @brief Access the rotation part of the velocity (read-only)
    * @return const reference to the rotational velocity
    */
    const double& w() const {return rotation();}

    /**
    * @brief Access the left wheel rotation part of the velocity
    * @return reference to the left wheel rotational velocity
    */
    double& left() {return _vel_wheel.coeffRef(0);}

    /**
    * @brief Access the left wheel rotation part of the velocity (read-only)
    * @return const reference to the left wheel rotational velocity
    */
    const double& left() const {return _vel_wheel.coeffRef(0);}

    /**
    * @brief Access the right wheel rotation part of the velocity
    * @return reference to the right wheel rotational velocity
    */
    double& right() {return _vel_wheel.coeffRef(1);}

    /**
    * @brief Access the right wheel rotation part of the velocity (read-only)
    * @return const reference to the right wheel rotational velocity
    */
    const double& right() const {return _vel_wheel.coeffRef(1);}

    /**
    * @brief Access the left wheel rotation part of the velocity
    * @return reference to the left wheel rotational velocity
    */
    double& l() {return left();}

    /**
    * @brief Access the left wheel rotation part of the velocity (read-only)
    * @return const reference to the left wheel rotational velocity
    */
    const double& l() const {return left();}

    /**
    * @brief Access the right wheel rotation part of the velocity
    * @return reference to the right wheel rotational velocity
    */
    double& r() {return right();}

    /**
    * @brief Access the right wheel rotation part of the velocity (read-only)
    * @return const reference to the right wheel rotational velocity
    */
    const double& r() const {return right();}

    /**
    * @brief Set velocity to zero
    */
    void setZero()
    {
        _vel_robot.setZero();
        _vel_wheel.setZero();
        _vel_orientation = 0;
        _vel_translation = 0;
        _vel_rotation    = 0;
    }

    /**
    * @brief Set wheel radius
    */
    void setRadius(const double radius)
    {
        _wheel_radius = radius;
        _vel_wheel.coeffRef(0) = _vel_translation/_wheel_radius - _vel_rotation*_wheel_distance/(2*_wheel_radius);
        _vel_wheel.coeffRef(1) = _vel_translation/_wheel_radius + _vel_rotation*_wheel_distance/(2*_wheel_radius);
    }

    /**
    * @brief Set inter-wheel distance
    */
    void setDistance(const double distance)
    {
        _wheel_distance = distance;
        _vel_wheel.coeffRef(0) = _vel_translation/_wheel_radius - _vel_rotation*_wheel_distance/(2*_wheel_radius);
        _vel_wheel.coeffRef(1) = _vel_translation/_wheel_radius + _vel_rotation*_wheel_distance/(2*_wheel_radius);
    }

    /**
    * @brief Get wheel radius
    * @return wheel radius
    */
    double getRadius() const {return _wheel_radius;}

    /**
    * @brief Get inter-wheel distance
    * @return inter-wheel distance
    */
    double getDistance() const {return _wheel_distance;}

    /**
     * @brief Return the unit vector of the current orientation
     * @returns [cos(theta), sin(theta))]^T
     */
    Eigen::Vector2d orientationUnitVec() const {return Eigen::Vector2d(std::cos(_vel_orientation), std::sin(_vel_orientation));}

    ///@}


    /** @name Arithmetic operations for which operators are not always reasonable */
    ///@{

    /**
    * @brief Update robot and wheel velocity components
    */
    void update()
    {
        _vel_robot.coeffRef(0) = _vel_translation * std::cos(_vel_orientation);
        _vel_robot.coeffRef(1) = _vel_translation * std::sin(_vel_orientation);
        _vel_robot.coeffRef(2) = _vel_rotation;
        _vel_wheel.coeffRef(0) = _vel_translation/_wheel_radius - _vel_rotation*_wheel_distance/(2*_wheel_radius);
        _vel_wheel.coeffRef(1) = _vel_translation/_wheel_radius + _vel_rotation*_wheel_distance/(2*_wheel_radius);
    }

    /**
    * @brief Scale all velocity components and normalize orientation afterwards to [-pi, pi]
    * @param factor scale factor
    */
    void scale(const double factor)
    {
        _vel_robot       *= factor;
        _vel_wheel       *= factor;
        _vel_rotation    *= factor;
        _vel_translation *= factor;
        _vel_orientation  = g2o::normalize_theta(std::atan2(_vel_robot.coeffRef(1), _vel_robot.coeffRef(0)));
    }

    /**
    * @brief Increment the velocity by adding a double[3] array and normalize orientation afterwards to [-pi, pi]
    * @param velocity_as_array 3D double array [x, y, z]
    */
    void plus(const double* velocity_as_array)
    {
        _vel_robot.coeffRef(0) += velocity_as_array[0];
        _vel_robot.coeffRef(1) += velocity_as_array[1];
        _vel_robot.coeffRef(2) += velocity_as_array[2];
        _vel_orientation = g2o::normalize_theta(std::atan2(_vel_robot.coeffRef(1), _vel_robot.coeffRef(0)));
        _vel_translation = g2o::sign(_vel_orientation) * Eigen::Vector2d(_vel_robot.coeffRef(0), _vel_robot.coeffRef(1)).norm();
        _vel_rotation    = _vel_robot.coeffRef(2);
        _vel_wheel.coeffRef(0) = _vel_translation/_wheel_radius - _vel_rotation*_wheel_distance/(2*_wheel_radius);
        _vel_wheel.coeffRef(1) = _vel_translation/_wheel_radius + _vel_rotation*_wheel_distance/(2*_wheel_radius);
    }

    /**
    * @brief Get the mean / average of two velocities and store it in the caller class
    * For the wheels: keep caller class' wheel radius and inter-wheel distance
    * For the velocity: (v1+v2)/2
    * For the orientation: take the angle of the mean direction vector
    * @param velocity1 first velocity to consider
    * @param velocity2 second velocity to consider
    */
    void averageInPlace(const Velocity& velocity1, const Velocity& velocity2)
    {
        _vel_robot.coeffRef(0) = (velocity1._vel_robot.coeffRef(0) + velocity2._vel_robot.coeffRef(0)) / 2;
        _vel_robot.coeffRef(1) = (velocity1._vel_robot.coeffRef(1) + velocity2._vel_robot.coeffRef(1)) / 2;
        _vel_robot.coeffRef(2) = (velocity1._vel_robot.coeffRef(2) + velocity2._vel_robot.coeffRef(2)) / 2;
        _vel_orientation = g2o::average_angle(velocity1._vel_orientation, velocity2._vel_orientation);
        _vel_translation = g2o::sign(_vel_orientation) * Eigen::Vector2d(_vel_robot.coeffRef(0), _vel_robot.coeffRef(1)).norm();
        _vel_rotation    = _vel_robot.coeffRef(2);
        _vel_wheel.coeffRef(0) = _vel_translation/_wheel_radius - _vel_rotation*_wheel_distance/(2*_wheel_radius);
        _vel_wheel.coeffRef(1) = _vel_translation/_wheel_radius + _vel_rotation*_wheel_distance/(2*_wheel_radius);
    }

    /**
    * @brief Get the mean / average of two velocities and return the result (static)
    * For the wheels: take mean wheel radius and mean inter-wheel distance
    * For the velocity: (v1+v2)/2
    * For the orientation: take the angle of the mean direction vector
    * @param velocity1 first velocity to consider
    * @param velocity2 second velocity to consider
    * @return mean / average of \c velocity1 and \c velocity2
    */
    static Velocity average(const Velocity& velocity1, const Velocity& velocity2)
    {
        return Velocity((velocity1._wheel_radius    + velocity2._wheel_radius   ) / 2,
                        (velocity1._wheel_distance  + velocity2._wheel_distance ) / 2,
                        (velocity1._vel_translation + velocity2._vel_translation) / 2,
                        (velocity1._vel_rotation    + velocity2._vel_rotation   ) / 2,
                        g2o::average_angle(velocity1._vel_orientation, velocity2._vel_orientation));
    }

    /**
    * @brief Rotate velocity globally
    * 
    * Compute [velocity_x, velocity_y] = Rot(\c angle) * [velocity_x, velocity_y].
    * If \c adjust_theta, theta is also rotated by \c angle
    * @param angle the angle defining the 2D rotation
    * @param adjust_theta if \c true, the orientation theta is also rotated
    */
    void rotateGlobal(const double angle, const bool adjust_theta = true)
    {
        double new_x = std::cos(angle)*_vel_robot.x() - std::sin(angle)*_vel_robot.y();
        double new_y = std::sin(angle)*_vel_robot.x() + std::cos(angle)*_vel_robot.y();
        _vel_robot.x() = new_x;
        _vel_robot.y() = new_y;
        if (adjust_theta)
            _vel_orientation = g2o::normalize_theta(_vel_orientation + angle);
    }

    ///@}


    /** @name Operator overloads / Allow some arithmetic operations */
    ///@{

    /**
    * @brief Assignment operator
    * @param rhs Velocity instance
    * @todo exception safe version of the assignment operator
    */
    Velocity& operator=(const Velocity& rhs)
    {
        if (&rhs != this)
        {
            _vel_robot       = rhs._vel_robot;
            _vel_wheel       = rhs._vel_wheel;
            _vel_orientation = rhs._vel_orientation;
            _vel_translation = rhs._vel_translation;
            _vel_rotation    = rhs._vel_rotation;
        }
        return *this;
    }

    /**
    * @brief Compound assignment operator (addition)
    * @param rhs addend
    */
    Velocity& operator+=(const Velocity& rhs)
    {
        _vel_robot      += rhs._vel_robot;
        _vel_wheel      += rhs._vel_wheel;
        _vel_rotation   += rhs._vel_rotation;
        _vel_orientation = g2o::normalize_theta(_vel_orientation + rhs._vel_orientation);
        _vel_translation = g2o::sign(_vel_orientation) * Eigen::Vector2d(_vel_robot.coeffRef(0), _vel_robot.coeffRef(1)).norm();
        return *this;
    }

    /**
     * @brief Arithmetic operator overload for additions
     * @param lhs First addend
     * @param rhs Second addend
     */
    friend Velocity operator+(Velocity lhs, const Velocity& rhs) 
    {
        return lhs += rhs;
    }

    /**
    * @brief Compound assignment operator (subtraction)
    * @param rhs value to subtract
    */
    Velocity& operator-=(const Velocity& rhs)
    {
        _vel_robot      -= rhs._vel_robot;
        _vel_wheel      -= rhs._vel_wheel;
        _vel_rotation   -= rhs._vel_rotation;
        _vel_orientation = g2o::normalize_theta(_vel_orientation - rhs._vel_orientation);
        _vel_translation = g2o::sign(_vel_orientation) * Eigen::Vector2d(_vel_robot.coeffRef(0), _vel_robot.coeffRef(1)).norm();
        return *this;
    }

    /**
     * @brief Arithmetic operator overload for subtractions
     * @param lhs First term
     * @param rhs Second term
     */
    friend Velocity operator-(Velocity lhs, const Velocity& rhs) 
    {
        return lhs -= rhs;
    }

    /**
    * @brief Multiply velocity with scalar and return copy without additional calculations about data consistency
    * @param velocity velocity to scale
    * @param scalar factor to multiply with
    * @warning scalar is not checked to be non-zero before multiplying
    */
    friend Velocity operator*(Velocity velocity, const double scalar) 
    {
        velocity._vel_robot       *= scalar;
        velocity._vel_wheel       *= scalar;
        velocity._vel_orientation *= scalar;
        velocity._vel_translation *= scalar;
        velocity._vel_rotation    *= scalar;
        return velocity;
    }

    /**
    * @brief Multiply velocity with scalar and return copy without additional calculations about data consistency
    * @param scalar factor to multiply with
    * @param velocity velocity to scale
    * @warning scalar is not checked to be non-zero before multiplying
    */
    friend Velocity operator*(const double scalar, Velocity velocity) 
    {
        velocity._vel_robot       *= scalar;
        velocity._vel_wheel       *= scalar;
        velocity._vel_orientation *= scalar;
        velocity._vel_translation *= scalar;
        velocity._vel_rotation    *= scalar;
        return velocity;
    }

    /**
    * @brief Divide velocity with scalar and return copy without additional calculations about data consistency
    * @param velocity velocity to scale
    * @param scalar factor to divide with
    * @warning scalar is not checked to be non-zero before dividing
    */
    friend Velocity operator/(Velocity velocity, const double scalar) 
    {
        velocity._vel_robot       /= scalar;
        velocity._vel_wheel       /= scalar;
        velocity._vel_orientation /= scalar;
        velocity._vel_translation /= scalar;
        velocity._vel_rotation    /= scalar;
        return velocity;
    }

    /**
    * @brief Divide velocity with scalar and return copy without additional calculations about data consistency
    * @param scalar factor to divide with
    * @param velocity velocity to scale
    * @warning scalar is not checked to be non-zero before dividing
    */
    friend Velocity operator/(const double scalar, Velocity velocity) 
    {
        velocity._vel_robot       /= scalar;
        velocity._vel_wheel       /= scalar;
        velocity._vel_orientation /= scalar;
        velocity._vel_translation /= scalar;
        velocity._vel_rotation    /= scalar;
        return velocity;
    }

    /**
     * @brief Output stream operator
     * @param stream output stream
     * @param velocity to be used
     */
    friend std::ostream& operator<<(std::ostream& stream, const Velocity& velocity)
    {
        stream << "Velocity = ";
        stream << "x: " << velocity._vel_robot[0]    << ", y: " << velocity._vel_robot[1] << ", z: " << velocity._vel_robot[2]    << ", ";
        stream << "v: " << velocity._vel_translation << ", w: " << velocity._vel_rotation << ", o: " << velocity._vel_orientation << ", ";
        stream << "l: " << velocity._vel_wheel[0]    << ", r: " << velocity._vel_wheel[1];
        return stream;
    }

    ///@}

private:

    Eigen::Vector3d _vel_robot; // x,y,z (axis components)
    Eigen::Vector2d _vel_wheel; // l,r (left,right speeds)
    double _vel_orientation; // o (heading direction theta)
    double _vel_translation; // v (linear velocity)
    double _vel_rotation;    // w (angular velocity)
    double _wheel_radius;   // r (wheel radius)
    double _wheel_distance; // d (inter-wheel distance)

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared instances of Velocity
typedef boost::shared_ptr<Velocity> VelocityPtr;
//! Abbrev. for shared const Velocity pointers
typedef boost::shared_ptr<const Velocity> VelocityConstPtr;
//! Abbrev. for containers storing multiple Velocity pointers
typedef std::vector<VelocityPtr> VelocityContainer;

} // namespace elastic_band

#endif /* _VELOCITY_HPP_ */
