#include "PositionListener.hpp"
#include "FishBot.hpp"

#include <QtCore/QDebug>
#include <algorithm>

/*!
 * Constructor.
 */
PositionListener::PositionListener(FishBot* robot) : ControlMode(robot, ControlModeType::POSITION_LISTENER)
{
    server_ = new QTcpServer(this);
    connect(server_, SIGNAL(newConnection()), SLOT(newConnection()));
}

void PositionListener::newConnection()
{
    while (server_->hasPendingConnections()) {
        QTcpSocket* socket = server_->nextPendingConnection();
        connect(socket, SIGNAL(readyRead()), SLOT(readyRead()));
        connect(socket, SIGNAL(disconnected()), SLOT(disconnected()));
        QByteArray* buffer = new QByteArray();
        float* s = new float(0);
        buffers_.insert(socket, buffer);
        sizes_.insert(socket, s);
    }
}

void PositionListener::disconnected()
{
    QTcpSocket* socket = static_cast<QTcpSocket*>(sender());
    QByteArray* buffer = buffers_.value(socket);
    float* s = sizes_.value(socket);
    socket->deleteLater();
    delete buffer;
    delete s;
}

void PositionListener::readyRead()
{
    QTcpSocket* socket = static_cast<QTcpSocket*>(sender());
    QByteArray* buffer = buffers_.value(socket);

    uint pack_size = 16;
    uint bytes = 4;
    uint num_numbers = pack_size / bytes;

    std::vector<float> values;
    while (socket->bytesAvailable() > 0) {
        if (socket->bytesAvailable() < pack_size)
            continue;
        buffer->append(socket->read(pack_size));
        for (uint i = 0; i < num_numbers; ++i) {
            values.push_back(ArrayToFloat(buffer->mid(0, bytes)));
            buffer->remove(0, bytes);
        }
    }

    std::lock_guard<std::mutex> lg(lock_);
    target_values_ = values;
}

float PositionListener::ArrayToFloat(QByteArray source)
{
    QByteArray reverse;
    reverse.resize(source.size());
    std::reverse_copy(source.constBegin(), source.constEnd(), reverse.begin());

    float temp;
    QDataStream data(&reverse, QIODevice::ReadWrite);
    data.setFloatingPointPrecision(QDataStream::SinglePrecision);
    data >> temp;
    return temp;
}

/*!
 * Destructor.
 */
PositionListener::~PositionListener()
{
    qDebug() << "Destroying the object";
}

/*!
 * The step of the control mode, generates the target based on the predefined
 * trajectory.
 */
ControlTargetPtr PositionListener::step()
{
    // if port is open and subscriber is active
    {
        std::lock_guard<std::mutex> lg(lock_);
        if (target_values_.size() > 0) {
            qDebug() << "Setting target position to: " << target_values_[0] << " " << target_values_[1];
            return ControlTargetPtr(new TargetPosition(PositionMeters(target_values_[0], target_values_[1])));
        }
        else {
            qDebug() << "Velocities set to zero";
            return ControlTargetPtr(new TargetSpeed(0, 0));
        }
    }
}

/*!
 * Informs on what kind of control taropengets this control mode generates.
 */
QList<ControlTargetType> PositionListener::supportedTargets()
{
    return QList<ControlTargetType>({ControlTargetType::SPEED,
        ControlTargetType::POSITION});
}

/*!
 * Called when the control mode is activated.
 */
void PositionListener::start()
{
    qDebug() << "Listening: " << server_->listen(QHostAddress::Any, 5623);
}

/*!
 * Called when the control mode is disactivated.
 */
void PositionListener::finish()
{
    server_->close();
    qDebug() << "Disconnecting listener";
}
