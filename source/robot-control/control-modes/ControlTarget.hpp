#ifndef CATS2_CONTROL_TARGET_HPP
#define CATS2_CONTROL_TARGET_HPP

#include "interfaces/Values.hpp"

#include <AgentState.hpp>
#include <QtCore/QSharedPointer>

/*!
 * \brief The type of the target.
 */
enum class ControlTargetType
{
    SPEED,
    SPEEDS,
    POSITION
};

/*!
 * \brief The generic control target class.
 */
class ControlTarget
{
public:
    //! Constructor.
    explicit ControlTarget(ControlTargetType type) : m_type(type) {}
    //! Destructor.
    virtual ~ControlTarget() = default;

public:
    //! Return the control target type.
    ControlTargetType type() const { return m_type; }

private:
    //! The type of control target.
    ControlTargetType m_type;
};

/*!
 * \brief Defines the target speed for the robot.
 */
class TargetSpeed : public ControlTarget
{
public:
    //! Constructor.
    explicit TargetSpeed(float leftSpeed = 0.0, float rightSpeed = 0.0) :
        ControlTarget(ControlTargetType::SPEED),
        m_leftSpeed(leftSpeed),
        m_rightSpeed(rightSpeed)
    {
    }

    //! Return the left motor speed.
    float leftSpeed() const { return m_leftSpeed; }
    //! Return the right motor speed.
    float rightSpeed() const { return m_rightSpeed; }

private:
    // TODO : define units
    //! Left motor target speed.
    float m_leftSpeed;
    //! Right motor target speed.
    float m_rightSpeed;
};

/*!
 * \brief Defines the target speeds for the robot.
 */
class TargetSpeeds : public ControlTarget
{
public:
    //! Constructor.
    explicit TargetSpeeds(Values leftSpeeds = Values(), Values rightSpeeds = Values(), int startingIndex = 0) :
        ControlTarget(ControlTargetType::SPEEDS),
        m_leftSpeeds(leftSpeeds),
        m_rightSpeeds(rightSpeeds),
        m_startingIndex(startingIndex)
    {
    }

    //! Return the left motor speeds.
    Values leftSpeeds() const { return m_leftSpeeds; }
    //! Return the right motor speeds.
    Values rightSpeeds() const { return m_rightSpeeds; }
    //! Return the starting list index.
    int startingIndex() const { return m_startingIndex; }

private:
    //! Left motor target speeds.
    Values m_leftSpeeds;
    //! Right motor target speeds.
    Values m_rightSpeeds;
    //! Starting index from previous command list.
    int m_startingIndex;
};

/*!
 * \brief Stores the target position for the robot.
 */
class TargetPosition : public ControlTarget
{
public:
    //! Constructor.
    explicit TargetPosition(PositionMeters position) :
        ControlTarget(ControlTargetType::POSITION),
        m_position(position)
    {
    }

    //! Return the target position.
    PositionMeters position() const { return m_position; }

private:
    //! The target position.
    PositionMeters m_position;
};

#endif // CATS2_CONTROL_TARGET_HPP

