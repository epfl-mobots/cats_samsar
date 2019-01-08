#include "PositionListener.hpp"
#include "FishBot.hpp"

#include <QtCore/QDebug>
#include <algorithm>

/*!
 * Constructor.
 */
PositionListener::PositionListener(FishBot* robot) : ControlMode(robot, ControlModeType::POSITION_LISTENER),
                                                     m_port(RobotControlSettings::get().port())
{
    m_server = new QTcpServer(this);
    connect(m_server, SIGNAL(newConnection()), SLOT(newConnection()));
}

void PositionListener::newConnection()
{
    while (m_server->hasPendingConnections()) {
        QTcpSocket* socket = m_server->nextPendingConnection();
        connect(socket, SIGNAL(readyRead()), SLOT(readyRead()));
        connect(socket, SIGNAL(disconnected()), SLOT(disconnected()));
        QByteArray* buffer = new QByteArray();
        float* s = new float(0);
        m_buffers.insert(socket, buffer);
        m_sizes.insert(socket, s);
    }
}

void PositionListener::disconnected()
{
    QTcpSocket* socket = static_cast<QTcpSocket*>(sender());
    QByteArray* buffer = m_buffers.value(socket);
    float* s = m_sizes.value(socket);
    socket->deleteLater();
    delete buffer;
    delete s;
}

void PositionListener::readyRead()
{
    QTcpSocket* socket = static_cast<QTcpSocket*>(sender());
    QByteArray* buffer = m_buffers.value(socket);

    uint pack_size = 16; // strictly four numbers for now
    uint bytes = 4; // strictly 4 bytes for now
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

    std::lock_guard<std::mutex> lg(m_lock);
    m_target_values = values;
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
        std::lock_guard<std::mutex> lg(m_lock);
        if (m_target_values.size() > 0) {
            qDebug() << "Setting target position to: " << m_target_values[0] << " " << m_target_values[1];
            return ControlTargetPtr(new TargetPosition(PositionMeters(m_target_values[0], m_target_values[1])));
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
    qDebug() << "Listening: " << m_server->listen(QHostAddress::Any, m_port) << " in port " << m_port;
}

/*!
 * Called when the control mode is disactivated.
 */
void PositionListener::finish()
{
    m_server->close();
    qDebug() << "Disconnecting listener";
}
