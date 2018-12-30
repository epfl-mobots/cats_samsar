#include "PositionSubscriber.hpp"
#include "FishBot.hpp"

#include <QtCore/QDebug>

/*!
 * Constructor.
 */
PositionSubscriber::PositionSubscriber(FishBot* robot) : ControlMode(robot, ControlModeType::POSITION_SUBSCRIBER)
{
}

/*!
 * Destructor.
 */
PositionSubscriber::~PositionSubscriber()
{
    qDebug() << "Destroying the object";
}

/*!
 * The step of the control mode, generates the target based on the predefined
 * trajectory.
 */
ControlTargetPtr PositionSubscriber::step()
{
    // if port is open and subscriber is active

    return ControlTargetPtr(new TargetSpeed(0, 0));
}

/*!
 * Informs on what kind of control taropengets this control mode generates.
 */
QList<ControlTargetType> PositionSubscriber::supportedTargets()
{
    return QList<ControlTargetType>({ControlTargetType::SPEED,
        ControlTargetType::POSITION});
}

/*!
 * Called when the control mode is activated.
 */
void PositionSubscriber::start()
{
}

/*!
 * Called when the control mode is disactivated.
 */
void PositionSubscriber::finish()
{
}
