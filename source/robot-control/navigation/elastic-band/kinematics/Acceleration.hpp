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

#ifndef _ACCELERATION_HPP_
#define _ACCELERATION_HPP_

#include <Eigen/Core>

namespace elastic_band
{

/**
  * @class Acceleration
  * @brief This class implements an acceleration in the domain \f$\mathbb{R}^2\f$
  * The acceleration consists of a translational part and a rotational part.
  */
class Acceleration
{
public:

    /** @name Construct Acceleration instances */
    ///@{

    /**
    * @brief Default constructor
    */
    Acceleration()
    {
        setZero();
    }

    /**
    * @brief Overloaded constructor
    * @param vdot translational acceleration
    * @param wdot rotational acceleration in rad/s^2
    */
    Acceleration(const double vdot, const double wdot)
    {
        _acc_translation = vdot;
        _acc_rotation    = wdot;
    }

    /**
    * @brief Copy constructor
    * @param acceleration Acceleration instance
    */
    Acceleration(const Acceleration& acceleration)
    {
        _acc_translation = acceleration._acc_translation;
        _acc_rotation    = acceleration._acc_rotation;
    }

    ///@}


    /**
    * @brief Destruct Acceleration
    */ 
    ~Acceleration() {}


    /** @name Access and modify values */
    ///@{

    /**
    * @brief Access the 2D acceleration (translation and rotation) (read-only)
    * @return const 2D acceleration (translation and rotation)
    */
    const Eigen::Vector2d acceleration2d() const {return Eigen::Vector2d(_acc_translation, _acc_rotation);}

    /**
    * @brief Access the translation part of the acceleration
    * @return reference to the translational acceleration
    */
    double& translation() {return _acc_translation;}

    /**
    * @brief Access the translation part of the acceleration (read-only)
    * @return const reference to the translational acceleration
    */
    const double& translation() const {return _acc_translation;}

    /**
    * @brief Access the rotation part of the acceleration
    * @return reference to the rotational acceleration
    */
    double& rotation() {return _acc_rotation;}

    /**
    * @brief Access the rotation part of the acceleration (read-only)
    * @return const reference to the rotational acceleration
    */
    const double& rotation() const {return _acc_rotation;}

    /**
    * @brief Access the translation part of the acceleration
    * @return reference to the ranslational acceleration
    */
    double& vdot() {return translation();}

    /**
    * @brief Access the translation part of the acceleration (read-only)
    * @return const reference to the translational acceleration
    */
    const double& vdot() const {return translation();}

    /**
    * @brief Access the rotation part of the acceleration
    * @return reference to the rotational acceleration
    */
    double& wdot() {return rotation();}

    /**
    * @brief Access the rotation part of the acceleration (read-only)
    * @return const reference to the rotational acceleration
    */
    const double& wdot() const {return rotation();}

    /**
    * @brief Set acceleration to zero
    */
    void setZero()
    {
        _acc_translation = 0;
        _acc_rotation    = 0;
    }

    ///@}


    /** @name Arithmetic operations for which operators are not always reasonable */
    ///@{

    /**
    * @brief Scale all acceleration components
    * @param factor scale factor
    */
    void scale(const double factor)
    {
        _acc_translation *= factor;
        _acc_rotation    *= factor;
    }

    /**
    * @brief Increment the acceleration by adding a double[2] array
    * @param acceleration_as_array 2D double array [vdot, wdot]
    */
    void plus(const double* acceleration_as_array)
    {
        _acc_translation += acceleration_as_array[0];
        _acc_rotation    += acceleration_as_array[1];
    }

    /**
    * @brief Get the mean / average of two accelerations and store it in the caller class
    * @param acceleration1 first acceleration to consider
    * @param acceleration2 second acceleration to consider
    */
    void averageInPlace(const Acceleration& acceleration1, const Acceleration& acceleration2)
    {
        _acc_translation = (acceleration1._acc_translation + acceleration2._acc_translation) / 2;
        _acc_rotation    = (acceleration1._acc_rotation    + acceleration2._acc_rotation   ) / 2;
    }

    /**
    * @brief Get the mean / average of two accelerations and return the result (static)
    * @param acceleration1 first acceleration to consider
    * @param acceleration2 second acceleration to consider
    * @return mean / average of \c acceleration1 and \c acceleration2
    */
    static Acceleration average(const Acceleration& acceleration1, const Acceleration& acceleration2)
    {
        return Acceleration((acceleration1._acc_translation + acceleration2._acc_translation) / 2,
                            (acceleration1._acc_rotation    + acceleration2._acc_rotation   ) / 2);
    }

    ///@}


    /** @name Operator overloads / Allow some arithmetic operations */
    ///@{

    /**
    * @brief Assignment operator
    * @param rhs Acceleration instance
    * @todo exception safe version of the assignment operator
    */
    Acceleration& operator=(const Acceleration& rhs)
    {
        if (&rhs != this)
        {
            _acc_translation = rhs._acc_translation;
            _acc_rotation    = rhs._acc_rotation;
        }
        return *this;
    }

    /**
    * @brief Compound assignment operator (addition)
    * @param rhs addend
    */
    Acceleration& operator+=(const Acceleration& rhs)
    {
        _acc_translation += rhs._acc_translation;
        _acc_rotation    += rhs._acc_rotation;
        return *this;
    }

    /**
     * @brief Arithmetic operator overload for additions
     * @param lhs First addend
     * @param rhs Second addend
     */
    friend Acceleration operator+(Acceleration lhs, const Acceleration& rhs) 
    {
        return lhs += rhs;
    }

    /**
    * @brief Compound assignment operator (subtraction)
    * @param rhs value to subtract
    */
    Acceleration& operator-=(const Acceleration& rhs)
    {
        _acc_translation -= rhs._acc_translation;
        _acc_rotation    -= rhs._acc_rotation;
        return *this;
    }

    /**
     * @brief Arithmetic operator overload for subtractions
     * @param lhs First term
     * @param rhs Second term
     */
    friend Acceleration operator-(Acceleration lhs, const Acceleration& rhs) 
    {
        return lhs -= rhs;
    }

    /**
    * @brief Multiply acceleration with scalar and return copy without additional calculations about data consistency
    * @param acceleration acceleration to scale
    * @param scalar factor to multiply with
    * @warning scalar is not checked to be non-zero before multiplying
    */
    friend Acceleration operator*(Acceleration acceleration, const double scalar) 
    {
        acceleration._acc_translation *= scalar;
        acceleration._acc_rotation    *= scalar;
        return acceleration;
    }

    /**
    * @brief Multiply acceleration with scalar and return copy without additional calculations about data consistency
    * @param scalar factor to multiply with
    * @param acceleration acceleration to scale
    * @warning scalar is not checked to be non-zero before multiplying
    */
    friend Acceleration operator*(const double scalar, Acceleration acceleration) 
    {
        acceleration._acc_translation *= scalar;
        acceleration._acc_rotation    *= scalar;
        return acceleration;
    }

    /**
    * @brief Divide acceleration with scalar and return copy without additional calculations about data consistency
    * @param acceleration acceleration to scale
    * @param scalar factor to divide with
    * @warning scalar is not checked to be non-zero before dividing
    */
    friend Acceleration operator/(Acceleration acceleration, const double scalar) 
    {
        acceleration._acc_translation /= scalar;
        acceleration._acc_rotation    /= scalar;
        return acceleration;
    }

    /**
    * @brief Divide acceleration with scalar and return copy without additional calculations about data consistency
    * @param scalar factor to divide with
    * @param acceleration acceleration to scale
    * @warning scalar is not checked to be non-zero before dividing
    */
    friend Acceleration operator/(const double scalar, Acceleration acceleration) 
    {
        acceleration._acc_translation /= scalar;
        acceleration._acc_rotation    /= scalar;
        return acceleration;
    }

    /**
     * @brief Output stream operator
     * @param stream output stream
     * @param acceleration to be used
     */
    friend std::ostream& operator<<(std::ostream& stream, const Acceleration& acceleration)
    {
        stream << "Acceleration = ";
        stream << "vdot: " << acceleration._acc_translation << ", wdot: " << acceleration._acc_rotation;
        return stream;
    }

    ///@}

private:

    double _acc_translation; // vdot
    double _acc_rotation;    // wdot

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared instances of Acceleration
typedef boost::shared_ptr<Acceleration> AccelerationPtr;
//! Abbrev. for shared const Acceleration pointers
typedef boost::shared_ptr<const Acceleration> AccelerationConstPtr;
//! Abbrev. for containers storing multiple Acceleration pointers
typedef std::vector<AccelerationPtr> AccelerationContainer;

} // namespace elastic_band

#endif /* _ACCELERATION_HPP_ */
