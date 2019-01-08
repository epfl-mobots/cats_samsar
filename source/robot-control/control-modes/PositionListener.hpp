#ifndef CATS2_POSITION_LISTENER_HPP
#define CATS2_POSITION_LISTENER_HPP

#include "ControlMode.hpp"

#include <QtCore/QTimer>
#include <QtNetwork>
#include <mutex>

/*!
 * Makes the robots to follow the predefined trajectory. When it arrives to the
 * last point it restarts it from the first point.
 */
// TODO : to show the trajectory on the viewer
class PositionListener : public ControlMode {
    Q_OBJECT
public:
    //! Constructor.
    explicit PositionListener(FishBot* robot);
    //! Destructor.
    virtual ~PositionListener() override;

    //! Called when the control mode is activated.
    virtual void start() override;
    //! The step of the control mode.
    virtual ControlTargetPtr step() override;
    //! Called when the control mode is disactivated.
    virtual void finish() override;

    virtual QList<ControlTargetType> supportedTargets() override;

    float ArrayToFloat(QByteArray source);

signals:
    void dataReceived(QByteArray);

private slots:
    void newConnection();
    void disconnected();
    void readyRead();

private:
    QTcpServer* server_;
    QHash<QTcpSocket*, QByteArray*> buffers_;
    QHash<QTcpSocket*, float*> sizes_;
    std::vector<float> target_values_;
    std::mutex lock_;
};

#endif // CATS2_POSITION_LISTENER_HPP
