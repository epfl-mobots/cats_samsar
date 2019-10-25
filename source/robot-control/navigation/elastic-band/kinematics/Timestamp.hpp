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

#ifndef _TIMESTAMP_HPP_
#define _TIMESTAMP_HPP_

#include <Eigen/Core>

#include <chrono>

namespace elastic_band
{

typedef std::chrono::duration<double, std::ratio<1>> timestamp_t; //!< Type for a floating-point second

/**
  * @class Timestamp
  * @brief This class implements a timestamp.
  * The timestamp consists of a duration in unit timestamp_t.
  */
class Timestamp : public timestamp_t
{
public:

    /** @name Construct Timestamp instances */
    ///@{

    /**
    * @brief Default constructor
    */
    Timestamp() : timestamp_t()
    {
        setZero();
    }

    /**
    * @brief Generic constructor
    * @param timestamp Timestamp instance
    */
    template<typename Rep, typename Period>
    Timestamp(const std::chrono::duration<Rep, Period>& timestamp) : timestamp_t(std::chrono::duration_cast<timestamp_t>(timestamp))
    {
    }

    /**
    * @brief Derived constructor
    * @param timestamp Timestamp instance
    */
    Timestamp(const timestamp_t& timestamp) : timestamp_t(timestamp)
    {
    }

    /**
    * @brief Copy constructor
    * @param timestamp Timestamp instance
    */
    Timestamp(const Timestamp& timestamp) : timestamp_t(timestamp)
    {
    }

    ///@}


    /**
    * @brief Destruct Timestamp
    */ 
    ~Timestamp() {}


    /** @name Access and modify values */
    ///@{

    /**
    * @brief Set timestamp to zero
    */
    void setZero()
    {
        *this = timestamp_t::zero();
    }

    ///@}


    /** @name Operator overloads / Allow some arithmetic operations */
    ///@{

    /**
     * @brief Output stream operator
     * @param stream output stream
     * @param timestamp to be used
     */
    friend std::ostream& operator<<(std::ostream& stream, const Timestamp& timestamp)
    {
        stream << timestamp.count();
        return stream;
    }

    ///@}

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for shared instances of Timestamp
typedef boost::shared_ptr<Timestamp> TimestampPtr;
//! Abbrev. for shared const Timestamp pointers
typedef boost::shared_ptr<const Timestamp> TimestampConstPtr;
//! Abbrev. for containers storing multiple Timestamp pointers
typedef std::vector<TimestampPtr> TimestampContainer;
//! Abbrev. for containers storing multiple Timestamp instances
typedef std::vector<Timestamp> TimestampVector;

//! Abbrev. for instances of Timestep
typedef Timestamp Timestep;
//! Abbrev. for shared instances of Timestep
typedef boost::shared_ptr<Timestep> TimestepPtr;
//! Abbrev. for shared const Timestep pointers
typedef boost::shared_ptr<const Timestep> TimestepConstPtr;
//! Abbrev. for containers storing multiple Timestep pointers
typedef std::vector<TimestepPtr> TimestepContainer;
//! Abbrev. for containers storing multiple Timestep instances
typedef std::vector<Timestep> TimestepVector;

} // namespace elastic_band

#endif /* _TIMESTAMP_HPP_ */
