#include "DensestPoint.hpp"

#include "FishBot.hpp"
#include "model/densestPointModel.hpp"
#include "model/epfl_factory.hpp"
#include "model/model.hpp"
#include "settings/RobotControlSettings.hpp"

DensestPoint::DensestPoint(FishBot* robot) : GenericFishModel(robot, ControlModeType::DENSEST_POINT)
{
    resetModel();
}

void DensestPoint::updateModelParameters()
{
    const FishModelSettings& fishModelSettings = RobotControlSettings::get().fishModelSettings();
    m_sim->dt = fishModelSettings.agentParameters.dt;

    for (auto& a : m_sim->agents) {
        a.first->length = fishModelSettings.agentParameters.length;
        a.first->width = fishModelSettings.agentParameters.width;
        a.first->height = fishModelSettings.agentParameters.height;
        a.first->fov = fishModelSettings.agentParameters.fov;
        a.first->meanSpeed = fishModelSettings.agentParameters.meanSpeed;
        a.first->varSpeed = fishModelSettings.agentParameters.varSpeed;
        a.first->maxTurningRate = M_PI_2;

        Fishmodel::DensestPointModel* dpt
            = reinterpret_cast<Fishmodel::DensestPointModel*>(a.second.get());
        dpt->reinit();
    }
}

void DensestPoint::resetModel()
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
        factory.behaviorFishes = "DPT";
        factory.behaviorRobots = "DPT";
        factory.behaviorVirtuals = "DPT";
        m_sim = factory.create();
        updateModelParameters();
    }
}
