#include "ToulouseControlMode.hpp"

#include "FishBot.hpp"
#include "model/epfl_factory.hpp"
#include "model/model.hpp"
#include "settings/RobotControlSettings.hpp"

#include <QtCore/QDebug>
#include <QtCore/QtMath>

ToulouseControlMode::ToulouseControlMode(FishBot* robot, QList<FishBot*> robots)
    : FishModelBase(robot, ControlModeType::TOULOUSE_MODE), m_robots(robots)
{
    for (auto& rbt : m_robots) {
        if (rbt->getSimulation() != nullptr) {
            m_sim = rbt->getSimulation();
            break;
        }
    }

    if (m_sim != nullptr) {
        m_robot->setSimulation(m_sim); // FIXME: assignment is not working, patched in FishBot
    } else {
        resetModel();
    }
}

void ToulouseControlMode::updateModelParameters()
{
    const FishModelSettings& fishModelSettings = RobotControlSettings::get().fishModelSettings();
    m_sim->dt = fishModelSettings.agentParameters.dt;

    int id_count = 0;
    for (auto& a : m_sim->agents) {
        a.first->length = fishModelSettings.agentParameters.length;
        a.first->width = fishModelSettings.agentParameters.width;
        a.first->height = fishModelSettings.agentParameters.height;
        a.first->fov = fishModelSettings.agentParameters.fov;
        a.first->meanSpeed = fishModelSettings.agentParameters.meanSpeed;
        a.first->varSpeed = fishModelSettings.agentParameters.varSpeed;
        a.first->maxTurningRate = M_PI_2;

        Fishmodel::ToulouseModel* tm = reinterpret_cast<Fishmodel::ToulouseModel*>(a.second.get());

        tm->radius = fishModelSettings.rummyFishModelSettings.radius;

        tm->perceived_agents = fishModelSettings.rummyFishModelSettings.perceived_agents;
        tm->gamma_rand = fishModelSettings.rummyFishModelSettings.gamma_rand;
        tm->gamma_wall = fishModelSettings.rummyFishModelSettings.gamma_wall;
        tm->wall_interaction_range = fishModelSettings.rummyFishModelSettings.wall_interaction_range;
        tm->body_length = fishModelSettings.rummyFishModelSettings.body_length;

        tm->alpha = fishModelSettings.rummyFishModelSettings.alpha;
        tm->tau0 = fishModelSettings.rummyFishModelSettings.tau0;
        tm->velocity_coef = fishModelSettings.rummyFishModelSettings.velocity_coef;
        tm->length_coef = fishModelSettings.rummyFishModelSettings.length_coef;
        tm->time_coef = fishModelSettings.rummyFishModelSettings.time_coef;

        tm->id() = id_count++;

        tm->reinit();
    }
}

void ToulouseControlMode::resetModel()
{
    if (!m_currentGrid.empty()) {
        // size of the area covered by the matrix
        Fishmodel::Coord_t size = {m_currentGrid.cols * m_gridSizeMeters, m_currentGrid.rows * m_gridSizeMeters};
        // create the arena
        m_arena.reset(new Fishmodel::Arena(m_currentGrid, size));
        Fishmodel::EpflSimulationFactory factory(*m_arena);
        factory.nbFishes = static_cast<size_t>(RobotControlSettings::get().numberOfAnimals());
        factory.nbRobots = static_cast<size_t>(RobotControlSettings::get().numberOfRobots()); // we generate one simulator for every robot
        factory.nbVirtuals = 0;
        factory.behaviorFishes = "TM";
        factory.behaviorRobots = "TM";
        factory.behaviorVirtuals = "TM";
        m_sim = factory.createAndShare();
        m_robot->setSimulation(m_sim); // FIXME: assignment is not working, patched in FishBot
        updateModelParameters();
        //        cv::imshow( "ModelGrid", m_currentGrid);
    } else {
        for (auto& a : m_sim->agents) {
            Fishmodel::ToulouseModel* tm = reinterpret_cast<Fishmodel::ToulouseModel*>(a.second.get());
            tm->reinit();
        }
    }
}

/*!
 * Called when the control mode is activated. Used to reset mode's parameters.
 */
void ToulouseControlMode::start()
{
    Fishmodel::ToulouseModel* robot = reinterpret_cast<Fishmodel::ToulouseModel*>(m_sim->robots[m_robot->firmwareId()].second);
    if (robot->robot() == nullptr) {
        robot->robot() = m_robot;
    }
    FishModelBase::start();
}

/*!
 * The step of the control mode.
 */
ControlTargetPtr ToulouseControlMode::step()
{
    ControlTargetPtr target = FishModelBase::step();
    Fishmodel::ToulouseModel* robot = reinterpret_cast<Fishmodel::ToulouseModel*>(m_sim->robots[m_robot->firmwareId()].second);
    if (!target.isNull() && isTargetValid()) {
        Values speedsL, speedsR;
        int index;
        QList<double> speeds;
        std::tie(index, speeds) = robot->getSpeedCommands();
        for (int i = 0; i < speeds.size(); i++) {
            const qint16 speed = static_cast<qint16>(std::ceil(speeds.at(i)));
            if (i % 2 == 0) {
                speedsL.append(speed);
            } else {
                speedsR.append(speed);
            }
        }
        target.reset(new TargetSpeeds(speedsL, speedsR, index));
    }
    robot->is_kicking() = false;
    robot->has_stepped() = false;
    robot->to_be_optimized() = true;
    return target;
}

/*!
 * Informs on what kind of control targets this control mode generates.
 */
QList<ControlTargetType> ToulouseControlMode::supportedTargets()
{
    return QList<ControlTargetType>({ControlTargetType::SPEED, ControlTargetType::SPEEDS, ControlTargetType::POSITION});
}
