﻿#ifndef CATS2_WHEEL_VELOCITIES_HPP
#define CATS2_WHEEL_VELOCITIES_HPP

#include "ControlMode.hpp"

#include <QtCore/QTimer>

/*!
 * Makes the robots to follow the predefined trajectory. When it arrives to the
 * last point it restarts it from the first point.
 */
// TODO : to show the trajectory on the viewer
class WheelVelocities : public ControlMode {
    Q_OBJECT
public:
    //! Constructor.
    explicit WheelVelocities(FishBot* robot);
    //! Destructor.
    virtual ~WheelVelocities() override;

    //! Called when the control mode is activated.
    virtual void start() override;
    //! The step of the control mode.
    virtual ControlTargetPtr step() override;
    //! Called when the control mode is disactivated.
    virtual void finish() override;

    //! Informs on what kind of control targets this control mode generates.
    virtual QList<ControlTargetType> supportedTargets() override;

private slots:
    //! Switches to the next waypoint.
    void updateCurrentIndex();

private:
    //! The trajectory to follow.
    QList<PositionMeters> m_wheelVelocities;
    //! The current point index.
    int m_currentIndex;
    //! Defines if the trajectory should be restarted once the last point is
    //! reached, read from settings.
    bool m_loopWheelVelocities;
    //! Specifies if the next point of the trajectory is to be provided on
    //! timer or once the previous is reached, read from settings.
    bool m_providePointsOnTimer;
    //! Updates the waypoints at given frequency.
    QTimer m_updateTimer;
};

#endif // CATS2_WHEEL_VELOCITIES_HPP
