#ifndef CATS2_POSITION_SUBSCRIBER_HPP
#define CATS2_POSITION_SUBSCRIBER_HPP

#include "ControlMode.hpp"

#include <QtCore/QTimer>
#include <QTcpSocket>

/*!
 * Makes the robots to follow the predefined trajectory. When it arrives to the
 * last point it restarts it from the first point.
 */
// TODO : to show the trajectory on the viewer
class PositionSubscriber : public ControlMode {
    Q_OBJECT
public:
    //! Constructor.
    explicit PositionSubscriber(FishBot* robot);
    //! Destructor.
    virtual ~PositionSubscriber() override;

    //! Called when the control mode is activated.
    virtual void start() override;
    //! The step of the control mode.
    virtual ControlTargetPtr step() override;
    //! Called when the control mode is disactivated.
    virtual void finish() override;

    virtual QList<ControlTargetType> supportedTargets() override;

private:
    QTcpSocket _sub;
};

#endif // CATS2_POSITION_SUBSCRIBER_HPP
