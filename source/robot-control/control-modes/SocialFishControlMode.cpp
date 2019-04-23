#include "SocialFishControlMode.hpp"

#include "FishBot.hpp"
#include "model/epfl_factory.hpp"
#include "model/model.hpp"
#include "model/socialFishModel.hpp"
#include "settings/RobotControlSettings.hpp"

#include <QtCore/QDebug>
#include <QtCore/QtMath>

SocialFishControlMode::SocialFishControlMode(FishBot* robot)
    : GenericFishModel(robot, ControlModeType::SOCIAL_FISH_MODEL)
{
    resetModel();
}

void SocialFishControlMode::updateModelParameters()
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

        Fishmodel::SocialFishModel* sfm
            = reinterpret_cast<Fishmodel::SocialFishModel*>(a.second.get());

        sfm->_num_cells = fishModelSettings.socialFishModelSettings.numCells;
        sfm->_group_threshold = fishModelSettings.socialFishModelSettings.groupThreshold;
        sfm->_cells_forward = fishModelSettings.socialFishModelSettings.cellsForward;
        sfm->_cells_backward = fishModelSettings.socialFishModelSettings.cellsBackward;
        sfm->_min_speed = fishModelSettings.socialFishModelSettings.minSpeed;
        sfm->_max_speed = fishModelSettings.socialFishModelSettings.maxSpeed;
        sfm->_prob_obey = fishModelSettings.socialFishModelSettings.probObey;
        sfm->_prob_move = fishModelSettings.socialFishModelSettings.probMove;
        sfm->_prob_change_speed = fishModelSettings.socialFishModelSettings.probChangeSpeed;
        sfm->_heading_change_duration
            = fishModelSettings.socialFishModelSettings.headingChangeDuration;
        sfm->_sum_weight = fishModelSettings.socialFishModelSettings.sumWeight;
        sfm->_influence_alpha = fishModelSettings.socialFishModelSettings.influence_alpha;
        sfm->_heading_bias
            = simu::types::to_heading(fishModelSettings.socialFishModelSettings.heading_bias);
        qDebug() << "Heading bias: " << sfm->_heading_bias;

        sfm->_target_reset_threshold
            = fishModelSettings.socialFishModelSettings.targetResetThreshold;
        sfm->_history_reset = fishModelSettings.socialFishModelSettings.historyReset;

        sfm->reinit();
    }
}

void SocialFishControlMode::resetModel()
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
        factory.behaviorFishes = "SFM";
        factory.behaviorRobots = "SFM";
        factory.behaviorVirtuals = "SFM";
        m_sim = factory.create();
        updateModelParameters();
        //        cv::imshow( "ModelGrid", m_currentGrid);
    }
}
