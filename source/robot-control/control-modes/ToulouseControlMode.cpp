#include "ToulouseControlMode.hpp"

#include "FishBot.hpp"
#include "model/epfl_factory.hpp"
#include "model/model.hpp"
#include "settings/RobotControlSettings.hpp"

#include <QtCore/QDebug>
#include <QtCore/QtMath>

ToulouseControlMode::ToulouseControlMode(FishBot* robot)
    : FishModelBase(robot, ControlModeType::TOULOUSE_MODE)
{
    resetModel();
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
        tm->wall_interaction_range
            = fishModelSettings.rummyFishModelSettings.wall_interaction_range;
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
        Fishmodel::Coord_t size
            = {m_currentGrid.cols * m_gridSizeMeters, m_currentGrid.rows * m_gridSizeMeters};
        // create the arena
        m_arena.reset(new Fishmodel::Arena(m_currentGrid, size));
        Fishmodel::EpflSimulationFactory factory(*m_arena);
        factory.nbFishes = static_cast<size_t>(RobotControlSettings::get().numberOfAnimals());
        factory.nbRobots = static_cast<size_t>(
            RobotControlSettings::get()
                .numberOfRobots()); // we generate one simulator for every robot
        factory.nbVirtuals = 0;
        factory.behaviorFishes = "TM";
        factory.behaviorRobots = "TM";
        factory.behaviorVirtuals = "TM";
        m_sim = factory.create();
        updateModelParameters();
        //        cv::imshow( "ModelGrid", m_currentGrid);
    }

    for (auto& a : m_sim->agents) {
        Fishmodel::ToulouseModel* tm = reinterpret_cast<Fishmodel::ToulouseModel*>(a.second.get());
        tm->reinit();
    }
}
