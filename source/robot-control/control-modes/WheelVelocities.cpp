#include "WheelVelocities.hpp"
#include "FishBot.hpp"

#include <QtCore/QDebug>

/*!
 * Constructor.
 */
WheelVelocities::WheelVelocities(FishBot* robot) : ControlMode(robot, ControlModeType::WHEEL_VELOCITIES),
                                                   m_wheelVelocities(RobotControlSettings::get().wheelVelocities()),
                                                   m_currentIndex(0),
                                                   m_loopWheelVelocities(RobotControlSettings::get().loopWheelVelocities()),
                                                   m_providePointsOnTimer(RobotControlSettings::get().provideWheelPointsOnTimer())
{
    connect(&m_updateTimer, &QTimer::timeout, this, &WheelVelocities::updateCurrentIndex);
}

/*!
 * Destructor.
 */
WheelVelocities::~WheelVelocities()
{
    qDebug() << "Destroying the object";
}

/*!
 * The step of the control mode, generates the target based on the predefined
 * trajectory.
 */
ControlTargetPtr WheelVelocities::step()
{
    if (!m_wheelVelocities.isEmpty()) {
        // TODO: implement method to update the velocities once the current velocity is close to the last
        // requested velocity
        //     // if the points are updated upon arrival
        //     if (!m_providePointsOnTimer && m_robot->state().position().isValid() && m_robot->state().position().closeTo(m_trajectory.at(m_currentIndex))) {
        //         // the robot approaches the current waypoint, hence it needs to be
        //         // updated
        updateCurrentIndex();
        //     }

        // returns the current position
        if ((m_currentIndex >= 0) && (m_currentIndex < m_wheelVelocities.size())) {
            return ControlTargetPtr(new TargetSpeed(
                m_wheelVelocities.at(m_currentIndex).x(),
                m_wheelVelocities.at(m_currentIndex).y()));
        }
    }
    else {
        std::cout << "file empty" << std::endl;
    }
    // // if the trajectory is not defined then don't move
    return ControlTargetPtr(new TargetSpeed(0, 0));
}

/*!
 * Switches to the next waypoint.
 */
void WheelVelocities::updateCurrentIndex()
{
    if (m_wheelVelocities.size() > 0) {
        if ((m_currentIndex != m_wheelVelocities.size()) || m_loopWheelVelocities)
            m_currentIndex = (m_currentIndex + 1) % m_wheelVelocities.size();
    }
    else {
        m_currentIndex = 0;
    }
}

/*!
 * Informs on what kind of control targets this control mode generates.
 */
QList<ControlTargetType> WheelVelocities::supportedTargets()
{
    return QList<ControlTargetType>({ControlTargetType::SPEED,
        ControlTargetType::POSITION});
}

/*!
 * Called when the control mode is activated.
 */
void WheelVelocities::start()
{
    // if (m_providePointsOnTimer) {
    //     double intervalMs = 1000. / RobotControlSettings::get().controlFrequencyHz();
    //     m_updateTimer.start(intervalMs);
    // }
}

/*!
 * Called when the control mode is disactivated.
 */
void WheelVelocities::finish()
{
    // if (m_providePointsOnTimer)
    //     m_updateTimer.stop();

    // m_currentIndex = 0;
}
