/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 * Adaptation: Mickaël Salamin, Fall 2019
 * Implementation: ROS package teb_local_planner
 * Source: https://github.com/rst-tu-dortmund/teb_local_planner/
 *********************************************************************/

#include <elastic-band/TebPlanner.hpp>

#include <map>
#include <limits>

namespace elastic_band
{

// ============== Implementation ===================

TebPlanner::TebPlanner() : cfg_(NULL), obstacles_(NULL), via_points_(NULL), trajectory_ref_(NULL), cost_(HUGE_VAL),
                           prefer_rotdir_(RotType::none), robot_model_(new PointRobotFootprint()), initialized_(false), optimized_(false)
{
}

TebPlanner::TebPlanner(const TebConfig& cfg, ObstacleContainer* obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{
  initialize(cfg, obstacles, robot_model, visual, via_points);
}

TebPlanner::~TebPlanner()
{
  clearGraph();
  // free dynamically allocated memory
  //if (optimizer_)
  //  g2o::Factory::destroy();
  //g2o::OptimizationAlgorithmFactory::destroy();
  //g2o::HyperGraphActionLibrary::destroy();
}

void TebPlanner::initialize(const TebConfig& cfg, ObstacleContainer* obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{
  // init optimizer (set solver and block ordering settings)
  optimizer_ = initOptimizer();

  cfg_ = &cfg;
  obstacles_ = obstacles;
  robot_model_ = robot_model;
  via_points_ = via_points;
  trajectory_ref_ = NULL;
  cost_ = HUGE_VAL;
  prefer_rotdir_ = RotType::none;
  setVisualization(visual);

  vel_start_.first = true;
  vel_start_.second.linear.x = 0;
  vel_start_.second.linear.y = 0;
  vel_start_.second.angular.z = 0;

  vel_goal_.first = true;
  vel_goal_.second.linear.x = 0;
  vel_goal_.second.linear.y = 0;
  vel_goal_.second.angular.z = 0;

  initialized_ = true;
  optimized_ = false;
}


void TebPlanner::setVisualization(TebVisualizationPtr visualization)
{
  visualization_ = visualization;
}

void TebPlanner::visualize()
{
  if (!visualization_)
    return;

  visualization_->publishLocalPlanAndPoses(teb_);

  if (teb_.sizePoses() > 0)
    visualization_->publishRobotFootprintModel(teb_.Pose(0), *robot_model_);

  if (cfg_->trajectory.publish_feedback)
    visualization_->publishFeedbackMessage(*this, *obstacles_);

}


/*
 * registers custom vertices and edges in g2o framework
 */
void TebPlanner::registerG2OTypes()
{
  g2o::Factory::destroy();
  g2o::Factory* factory = g2o::Factory::instance();
  factory->registerType("VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>);
  factory->registerType("VERTEX_TIMEDIFF", new g2o::HyperGraphElementCreator<VertexTimeDiff>);
  factory->registerType("EDGE_TIME_OPTIMAL", new g2o::HyperGraphElementCreator<EdgeTimeOptimal>);
  factory->registerType("EDGE_SHORTEST_PATH", new g2o::HyperGraphElementCreator<EdgeShortestPath>);
  factory->registerType("EDGE_PROFILE_FIDELITY", new g2o::HyperGraphElementCreator<EdgeProfileFidelity>);
  factory->registerType("EDGE_VELOCITY", new g2o::HyperGraphElementCreator<EdgeVelocity>);
  factory->registerType("EDGE_VELOCITY_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>);
  factory->registerType("EDGE_ACCELERATION", new g2o::HyperGraphElementCreator<EdgeAcceleration>);
  factory->registerType("EDGE_ACCELERATION_START", new g2o::HyperGraphElementCreator<EdgeAccelerationStart>);
  factory->registerType("EDGE_ACCELERATION_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationGoal>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>);
  factory->registerType("EDGE_DYNAMICS", new g2o::HyperGraphElementCreator<EdgeDynamics>);
  factory->registerType("EDGE_DYNAMICS_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeDynamicsHolonomic>);
  factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", new g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>);
  factory->registerType("EDGE_KINEMATICS_CARLIKE", new g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>);
  factory->registerType("EDGE_NEIGHBOR", new g2o::HyperGraphElementCreator<EdgeNeighbor>);
  factory->registerType("EDGE_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeObstacle>);
  factory->registerType("EDGE_INFLATED_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeInflatedObstacle>);
  factory->registerType("EDGE_INFLUENTIAL_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeInfluentialObstacle>);
  factory->registerType("EDGE_DYNAMIC_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeDynamicObstacle>);
  factory->registerType("EDGE_VIA_POINT", new g2o::HyperGraphElementCreator<EdgeViaPoint>);
  factory->registerType("EDGE_PREFER_ROTDIR", new g2o::HyperGraphElementCreator<EdgePreferRotDir>);
  return;
}

/*
 * initialize g2o optimizer. Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 */
boost::shared_ptr<g2o::SparseOptimizer> TebPlanner::initOptimizer()
{
  // Call register_g2o_types once, even for multiple TebPlanner instances (thread-safe)
  static boost::once_flag flag = BOOST_ONCE_INIT;
  boost::call_once(&registerG2OTypes, flag);

  // allocating the optimizer
  boost::shared_ptr<g2o::SparseOptimizer> optimizer = boost::make_shared<g2o::SparseOptimizer>();
  TEBLinearSolver* linearSolver = new TEBLinearSolver(); // see typedef in optimization.h
  linearSolver->setBlockOrdering(true);
  // TEBBlockSolver* blockSolver = new TEBBlockSolver(linearSolver);
  // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
  TEBBlockSolver* blockSolver = new TEBBlockSolver(std::unique_ptr<TEBBlockSolver::LinearSolverType>(linearSolver));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<TEBBlockSolver>(blockSolver));

  optimizer->setAlgorithm(solver);

  optimizer->initMultiThreading(); // required for >Eigen 3.1

  return optimizer;
}

namespace {
  void removeOptimizerOrientateAction(g2o::SparseOptimizer* optimizer, OrientateAction* orientateAction)
  {
    delete orientateAction->orientateElementAction();
    optimizer->removePreIterationAction(orientateAction);
  }

  void removeOptimizerTerminateAction(g2o::SparseOptimizer* optimizer, TerminateAction* terminateAction)
  {
    terminateAction->setOptimizerStopFlag(optimizer, false);
    optimizer->removePostIterationAction(terminateAction);
  }

  void removeActions(g2o::SparseOptimizer* optimizer, std::vector<g2o::HyperGraphAction*> actions)
  {
    for (size_t i = 0; i < actions.size(); i++)
    {
      g2o::HyperGraphAction* action = actions.at(i);
      if (action == nullptr)
        continue;
      if (dynamic_cast<OrientateAction*>(action))
        removeOptimizerOrientateAction(optimizer, dynamic_cast<OrientateAction*>(action));
      if (dynamic_cast<TerminateAction*>(action))
        removeOptimizerTerminateAction(optimizer, dynamic_cast<TerminateAction*>(action));
    }
  }
}
#define NO_OPTIMIZATION true
bool TebPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop,
                             double end_condition_improvement, double end_condition_timeout, double end_condition_error,
                             double obst_cost_scale, double viapoint_cost_scale, bool compute_cost_afterwards, bool alternative_time_cost)
{
#if NO_OPTIMIZATION
  optimized_ = true;
  return true;
#else
  if (!optimizer_ || !cfg_->optim.optimization_activate)
    return false;

  bool success = false;
  optimized_ = false;

  double weight_multiplier = 1.0;

  // TODO(roesmann): we introduced the non-fast mode with the support of dynamic obstacles
  //                (which leads to better results in terms of x-y-t homotopy planning).
  //                 however, we have not tested this mode intensively yet, so we keep
  //                 the legacy fast mode as default until we finish our tests.
  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;

  // Convert thresholds to actual values
  const double threshold_improvement = std::max(end_condition_improvement, 0.) / 100.;
  const double threshold_timeout = std::max(end_condition_timeout, 0.);
  const double threshold_chi = std::max(end_condition_error, 0.);

  // If no resizing is needed, optimize the graph all at once
  if (!cfg_->trajectory.teb_autosize && cfg_->optim.weight_adapt_factor == 1.)
  {
    iterations_innerloop = iterations_outerloop * iterations_innerloop;
    iterations_outerloop = 1;
  }

  // Set the custom orientate action for differential drive kinematics
  OrientateAction* orientateAction = nullptr;
  if (cfg_->optim.set_orientate_action == true &&
    !(cfg_->optim.weight_kinematics_nh == 0 && cfg_->optim.weight_kinematics_forward_drive  == 0) &&
     (cfg_->robot.min_turning_radius   == 0 || cfg_->optim.weight_kinematics_turning_radius == 0))
  {
    const std::string orientateEdgeName = typeid(EdgeKinematicsDiffDrive).name();
    OrientateElementAction* orientateElementAction = new OrientateElementAction(orientateEdgeName);
    orientateAction = new OrientateAction();
    orientateAction->setOrientateElementAction(orientateElementAction);
    orientateAction->setOrientateEdgeName(orientateEdgeName);
    optimizer_->addPreIterationAction(orientateAction);
  }

  // Set the custom terminate action with improvement, timeout and chi2 as end conditions
  TerminateAction* resizingAction = nullptr;
  TerminateAction* terminateAction = new TerminateAction();
  terminateAction->setImprovementThreshold(threshold_improvement);
  terminateAction->setTimeoutThreshold(threshold_timeout);
  terminateAction->setChiThreshold(threshold_chi);
  terminateAction->setMaxIterations(iterations_innerloop);
  terminateAction->resetTimer();
  optimizer_->addPostIterationAction(terminateAction);
  if (iterations_outerloop > 1)
  {
    resizingAction = new TerminateAction();
    resizingAction->setImprovementThreshold(threshold_improvement);
    resizingAction->setTimeoutThreshold(threshold_timeout);
    resizingAction->setChiThreshold(threshold_chi);
    resizingAction->setMaxIterations(iterations_outerloop);
    resizingAction->resetTimer();
  }

  // Construct the list of custom actions
  std::vector<g2o::HyperGraphAction*> actions;
  actions.push_back(dynamic_cast<g2o::HyperGraphAction*>(orientateAction));
  actions.push_back(dynamic_cast<g2o::HyperGraphAction*>(terminateAction));
  actions.push_back(dynamic_cast<g2o::HyperGraphAction*>(resizingAction));

  for (int i = 0; i < iterations_outerloop; ++i)
  {
    if (cfg_->trajectory.teb_autosize)
      teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);

    success = buildGraph(weight_multiplier);
    if (!success)
    {
      clearGraph();
      removeActions(&(*optimizer_), actions);
      return false;
    }
    success = optimizeGraph(iterations_innerloop, false);
    if (!success)
    {
      clearGraph();
      removeActions(&(*optimizer_), actions);
      return false;
    }
    optimized_ = true;

    if (i < iterations_outerloop-1)
    {
      if (resizingAction->isOptimizerStoppable(&(*optimizer_), i))
        i = iterations_outerloop-1;
    }

    if (i == iterations_outerloop-1)
    {
      if (cfg_->optim.save_optimized_graph) // save hypergraph only in the last iteration
        optimizer_->save(cfg_->optim.file_optimized_graph.c_str());

      if (compute_cost_afterwards) // compute cost vec only in the last iteration
        computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
    }

    clearGraph();

    weight_multiplier *= cfg_->optim.weight_adapt_factor;
  }

  removeActions(&(*optimizer_), actions);

  return true;
#endif
}

bool TebPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards,
                             double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{
  if (cfg_->optim.optimization_activate==false)
    return false;

  bool success = false;
  optimized_ = false;

  double weight_multiplier = 1.0;

  // TODO(roesmann): we introduced the non-fast mode with the support of dynamic obstacles
  //                (which leads to better results in terms of x-y-t homotopy planning).
  //                 however, we have not tested this mode intensively yet, so we keep
  //                 the legacy fast mode as default until we finish our tests.
  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;

  for(int i=0; i<iterations_outerloop; ++i)
  {
    if (cfg_->trajectory.teb_autosize)
    {
      //teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples);
      teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);
    }

    success = buildGraph(weight_multiplier);
    if (!success)
    {
        clearGraph();
        return false;
    }
    success = optimizeGraph(iterations_innerloop, false);
    if (!success)
    {
        clearGraph();
        return false;
    }
    optimized_ = true;

    if (compute_cost_afterwards && i==iterations_outerloop-1) // compute cost vec only in the last iteration
      computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);

    clearGraph();

    weight_multiplier *= cfg_->optim.weight_adapt_factor;
  }

  return true;
}

void TebPlanner::setVelocityStart(const Velocity& vel_start, const bool to_be_considered)
{
  vel_start_.first = to_be_considered;
  vel_start_.second.linear.x = vel_start.translation();
  vel_start_.second.linear.y = 0;
  vel_start_.second.angular.z = vel_start.rotation();
}

void TebPlanner::setVelocityStart(const geometry_msgs::Twist& vel_start)
{
  vel_start_.first = true;
  vel_start_.second.linear.x = vel_start.linear.x;
  vel_start_.second.linear.y = vel_start.linear.y;
  vel_start_.second.angular.z = vel_start.angular.z;
}

void TebPlanner::setVelocityGoal(const Velocity& vel_goal, const bool to_be_considered)
{
  vel_goal_.first = to_be_considered;
  vel_goal_.second.linear.x = vel_goal.translation();
  vel_goal_.second.linear.y = 0;
  vel_goal_.second.angular.z = vel_goal.rotation();
}

void TebPlanner::setVelocityGoal(const geometry_msgs::Twist& vel_goal)
{
  vel_goal_.first = true;
  vel_goal_.second = vel_goal;
}

bool TebPlanner::plan(const std::vector<TrajectoryPtr>& initial_plan, const std::vector<std::shared_ptr<std::vector<size_t>>> fix_pose_indices, const bool fix_timediff_vertices, const bool fix_pose_vertices, const bool fix_goal_pose_vertex)
{
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (initial_plan.empty())
    return false;
  trajectory_ref_ = nullptr;
  trajectories_ref_ = &initial_plan;

  // init trajectory
  teb_.clearTimedElasticBand();
  teb_.initTrajectoryToGoal(initial_plan, fix_pose_indices, fix_timediff_vertices, fix_pose_vertices, fix_goal_pose_vertex, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);

  // now optimize
  bool success = optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations, cfg_->optim.stop_below_percentage_improvement, cfg_->optim.stop_after_elapsed_time_microsecs, cfg_->optim.stop_below_significant_error_chi2);
  trajectories_ref_ = nullptr;
  return success;
}

bool TebPlanner::plan(const Trajectory& initial_plan, const std::shared_ptr<std::vector<size_t>> fix_pose_indices, const bool fix_timediff_vertices, const bool fix_pose_vertices, const bool fix_goal_pose_vertex, const bool free_goal_vel, const Velocity* start_vel)
{
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (initial_plan.trajectory().empty())
    return false;
  trajectories_ref_ = nullptr;
  trajectory_ref_ = &initial_plan;
  if (!teb_.isInit())
  {
    // init trajectory
    teb_.initTrajectoryToGoal(initial_plan, fix_pose_indices, fix_timediff_vertices, fix_pose_vertices, fix_goal_pose_vertex, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
  }
  else // warm start
  {
    PoseSE2 start(initial_plan.trajectory().front()->pose());
    PoseSE2 goal(initial_plan.trajectory().back()->pose());
    if (teb_.sizePoses() > 0 && (goal.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist) // actual warm start!
      teb_.updateAndPruneTEB(start, goal, cfg_->trajectory.min_samples); // update TEB
    else // goal too far away -> reinit
    {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(initial_plan, fix_pose_indices, fix_timediff_vertices, fix_pose_vertices, fix_goal_pose_vertex, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
    }
  }
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)

  // now optimize
  bool success = optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations, cfg_->optim.stop_below_percentage_improvement, cfg_->optim.stop_after_elapsed_time_microsecs, cfg_->optim.stop_below_significant_error_chi2);
  trajectory_ref_ = nullptr;
  return success;
}

bool TebPlanner::plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (!teb_.isInit())
  {
    // init trajectory
    teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
  }
  else // warm start
  {
    PoseSE2 start_(initial_plan.front().pose);
    PoseSE2 goal_(initial_plan.back().pose);
    if (teb_.sizePoses()>0 && (goal_.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist) // actual warm start!
      teb_.updateAndPruneTEB(start_, goal_, cfg_->trajectory.min_samples); // update TEB
    else // goal too far away -> reinit
    {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
    }
  }
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)

  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
}

bool TebPlanner::plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  PoseSE2 start_(start);
  PoseSE2 goal_(goal);
  return plan(start_, goal_, start_vel);
}

bool TebPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (!teb_.isInit())
  {
    // init trajectory
    teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion); // 0 intermediate samples, but dt=1 -> autoResize will add more samples before calling first optimization
  }
  else // warm start
  {
    if (teb_.sizePoses()>0 && (goal.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist) // actual warm start!
      teb_.updateAndPruneTEB(start, goal, cfg_->trajectory.min_samples);
    else // goal too far away -> reinit
    {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
    }
  }
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)

  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
}


bool TebPlanner::buildGraph(double weight_multiplier)
{
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    ROS_WARN("Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }

  offsets_.clear();
  offsets_.push_back(0);
  if (trajectories_ref_ != nullptr)
  {
    for (size_t r = 0; r < trajectories_ref_->size(); ++r)
      offsets_.push_back(offsets_.at(r) + trajectories_ref_->at(r).get()->trajectory().size());
  }
  else
    offsets_.push_back(teb_.sizePoses());

  // add TEB vertices
  AddTEBVertices();

  // add Edges (local cost functions)
  for (size_t r = 0; r < offsets_.size() - 1; ++r)
  {
    const size_t offset = offsets_.at(r);
    const size_t length = offsets_.at(r + 1) - offsets_.at(r);
    const size_t shift = r;

    if (trajectories_ref_ != nullptr)
      trajectory_ref_ = trajectories_ref_->at(r).get();

    if (cfg_->obstacles.legacy_obstacle_association)
      AddEdgesObstaclesLegacy(length, offset, weight_multiplier);
    else
      AddEdgesObstacles(length, offset, weight_multiplier);

    if (cfg_->obstacles.include_dynamic_obstacles)
      AddEdgesDynamicObstacles(length, offset);

    if (cfg_->optim.single_dynamics_edge)
      AddEdgesDynamics(length, offset, shift);
    else
    {
      AddEdgesVelocity(length, offset, shift);

      AddEdgesAcceleration(length, offset, shift);

      AddEdgesProfileFidelity(length, offset, shift);
    }

    AddEdgesShortestPath(length, offset);

    AddEdgesTimeOptimal(length, offset, shift);

    if (cfg_->robot.min_turning_radius == 0 || cfg_->optim.weight_kinematics_turning_radius == 0)
      AddEdgesKinematicsDiffDrive(length, offset); // we have a differential drive robot
    else
      AddEdgesKinematicsCarlike(length, offset); // we have a carlike robot since the turning radius is bounded from below

    AddEdgesPreferRotDir(length, offset);

    if (trajectories_ref_ != nullptr)
      trajectory_ref_ = nullptr;
  }

  AddEdgesViaPoints();

  AddEdgesNeighbors();

  return true;
}

bool TebPlanner::optimizeGraph(int no_iterations,bool clear_after)
{
  if (cfg_->robot.max_vel_x<0.01)
  {
    ROS_WARN("optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. Optimizing aborted...");
    if (clear_after) clearGraph();
    return false;
  }

  if (!teb_.isInit() || teb_.sizePoses() < cfg_->trajectory.min_samples)
  {
    ROS_WARN("optimizeGraph(): TEB is empty or has too less elements. Skipping optimization.");
    if (clear_after) clearGraph();
    return false;
  }

  optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  optimizer_->initializeOptimization();
  optimizer_->computeInitialGuess();
  optimizer_->computeActiveErrors();

  int iter = optimizer_->optimize(no_iterations);

  // Save Hessian for visualization
  //  g2o::OptimizationAlgorithmLevenberg* lm = dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
  //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");
  // // g2o::OptimizationAlgorithmLevenberg* lm = dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
  // // lm->solver().saveHessian("~/Documents/CATS2/logs/Hessian.txt");

  if(!iter)
  {
    ROS_ERROR("optimizeGraph(): Optimization failed! iter=%i", iter);
    return false;
  }

  if (clear_after) clearGraph();

  return true;
}

void TebPlanner::clearGraph()
{
  // clear optimizer states
  if (optimizer_)
  {
    //optimizer.edges().clear(); // optimizer.clear deletes edges!!! Therefore do not run optimizer.edges().clear()
    optimizer_->vertices().clear();  // neccessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
    optimizer_->clear();
  }
}



void TebPlanner::AddTEBVertices()
{
  // add vertices to graph
  ROS_DEBUG_COND(cfg_->optim.optimization_verbose, "Adding TEB vertices ...");
  unsigned int id_counter = 0; // used for vertices ids
  for (size_t r = 0; r < offsets_.size()-1; ++r)
  {
    for (size_t i = offsets_.at(r); i < offsets_.at(r+1); ++i)
    {
      teb_.PoseVertex(i)->setId(id_counter++);
      optimizer_->addVertex(teb_.PoseVertex(i));
      if (teb_.sizeTimeDiffs() != 0 && i-r < teb_.sizeTimeDiffs() && i < offsets_.at(r+1)-1)
      {
        teb_.TimeDiffVertex(i-r)->setId(id_counter++);
        optimizer_->addVertex(teb_.TimeDiffVertex(i-r));
      }
    }
  }
}


void TebPlanner::AddEdgesNeighbors()
{
  if (cfg_->optim.weight_neighbor == 0 || trajectories_ref_ == nullptr || trajectories_ref_->size() < 2)
    return; // if weight equals zero or single robot skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_neighbor);

  size_t offset1 = 0;
  size_t offset2 = 0;

  for (size_t r = 0; r < trajectories_ref_->size() - 1; ++r)
  {
    for (size_t s = r + 1; s < trajectories_ref_->size(); ++s)
    {
      offset2 += trajectories_ref_->at(s-1).get()->trajectory().size();

      const size_t length = std::min(trajectories_ref_->at(r).get()->trajectory().size(),
                                     trajectories_ref_->at(s).get()->trajectory().size());

      for (size_t i = 0; i < length; ++i)
      {
        if (teb_.PoseVertex(offset1+i)->fixed() &&
            teb_.PoseVertex(offset2+i)->fixed())
          continue;

        const double dist = robot_model_->calculateDistance(teb_.Pose(offset1+i),
                                                            teb_.Pose(offset2+i));

        if (dist < cfg_->neighbors.association_dist)
        {
          EdgeNeighbor* neighbor_edge = new EdgeNeighbor;
          neighbor_edge->setVertex(0,teb_.PoseVertex(offset1+i));
          neighbor_edge->setVertex(1,teb_.PoseVertex(offset2+i));
          neighbor_edge->setInformation(information);
          neighbor_edge->setParameters(*cfg_, robot_model_.get());
          optimizer_->addEdge(neighbor_edge);
        }
      }
    }

    offset1 += trajectories_ref_->at(r).get()->trajectory().size();
    offset2 = offset1;
  }
}


void TebPlanner::AddEdgesObstacles(size_t length, size_t offset, double weight_multiplier)
{
  if (obstacles_==nullptr || (cfg_->optim.weight_obstacle_influence==0 && (cfg_->optim.weight_obstacle==0 || weight_multiplier==0)))
    return; // if weight equals zero skip adding edges!

  const bool inflated    = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist && cfg_->optim.weight_obstacle_inflation > 0;
  const bool influential = cfg_->obstacles.influence_dist > cfg_->obstacles.min_obstacle_dist && cfg_->optim.weight_obstacle_influence > 0;

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);

  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1,1) = cfg_->optim.weight_obstacle_inflation;
  information_inflated(0,1) = information_inflated(1,0) = 0;

  Eigen::Matrix<double,3,3> information_influential;
  information_influential.fill(0);
  information_influential(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_influential(1,1) = cfg_->optim.weight_obstacle_inflation;
  information_influential(2,2) = cfg_->optim.weight_obstacle_influence;

  // define teb points limits (skip first and last if fixed)
  int begin = 0;
  int end = length - 1;
  if (teb_.PoseVertex(offset+begin)->fixed())
    ++begin;
  if (end > 0 && teb_.PoseVertex(offset+end)->fixed())
    --end;

  // iterate all teb points inside limits
  for (int i = begin; i <= end; ++i)
  {
    if (teb_.PoseVertex(offset+i)->fixed())
      continue; // skip pose if fixed

    double left_min_dist = std::numeric_limits<double>::max();
    double right_min_dist = std::numeric_limits<double>::max();
    Obstacle* left_obstacle = nullptr;
    Obstacle* right_obstacle = nullptr;

    std::vector<Obstacle*> relevant_obstacles;

    const Eigen::Vector2d pose_orient = teb_.Pose(offset+i).orientationUnitVec();

    // iterate obstacles
    for (const ObstaclePtr& obst : *obstacles_)
    {
      // we handle dynamic obstacles differently below
      if(cfg_->obstacles.include_dynamic_obstacles && obst->isDynamic())
        continue;

      // calculate distance to robot model
      double dist = robot_model_->calculateDistance(teb_.Pose(offset+i), obst.get());

      // force considering obstacle if really close to the current pose
      if (dist < cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_force_inclusion_factor)
      {
        relevant_obstacles.push_back(obst.get());
        continue;
      }
      // cut-off distance
      if (dist > cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_cutoff_factor)
        continue;

      // determine side (left or right) and assign obstacle if closer than the previous one
      if (cross2d(pose_orient, obst->getCentroid()) > 0) // left
      {
        if (dist < left_min_dist)
        {
          left_min_dist = dist;
          left_obstacle = obst.get();
        }
      }
      else
      {
        if (dist < right_min_dist)
        {
          right_min_dist = dist;
          right_obstacle = obst.get();
        }
      }
    }

    // consider left and right obstacle if they exist
    if (left_obstacle)
      relevant_obstacles.push_back(left_obstacle);
    if (right_obstacle)
      relevant_obstacles.push_back(right_obstacle);

    // create obstacle edges
    for (const Obstacle* obst : relevant_obstacles)
    {
      if (influential)
      {
        EdgeInfluentialObstacle* dist_bandpt_obst = new EdgeInfluentialObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(offset+i));
        dist_bandpt_obst->setInformation(information_influential);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst);
        optimizer_->addEdge(dist_bandpt_obst);
      }
      else if (inflated)
      {
        EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(offset+i));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst);
        optimizer_->addEdge(dist_bandpt_obst);
      }
      else
      {
        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(offset+i));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst);
        optimizer_->addEdge(dist_bandpt_obst);
      }
    }
  }
}


void TebPlanner::AddEdgesObstaclesLegacy(size_t length, size_t offset, double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr)
    return; // if weight equals zero skip adding edges!

  const bool inflated    = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist && cfg_->optim.weight_obstacle_inflation > 0;
  const bool influential = cfg_->obstacles.influence_dist > cfg_->obstacles.min_obstacle_dist && cfg_->optim.weight_obstacle_influence > 0;

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);

  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1,1) = cfg_->optim.weight_obstacle_inflation;
  information_inflated(0,1) = information_inflated(1,0) = 0;

  Eigen::Matrix<double,3,3> information_influential;
  information_influential.fill(0);
  information_influential(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_influential(1,1) = cfg_->optim.weight_obstacle_inflation;
  information_influential(2,2) = cfg_->optim.weight_obstacle_influence;

  // define teb points limits (skip first and last if fixed)
  int begin = 0;
  int end = length - 1;
  if (teb_.PoseVertex(offset+begin)->fixed())
    ++begin;
  if (end > 0 && teb_.PoseVertex(offset+end)->fixed())
    --end;

  for (ObstacleContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (cfg_->obstacles.include_dynamic_obstacles && (*obst)->isDynamic()) // we handle dynamic obstacles differently below
      continue;

    int index;
    if (cfg_->obstacles.obstacle_poses_affected >= length)
      index = offset + length / 2;
    else
      index = teb_.findClosestTrajectoryPose(*(obst->get()));

    // check if obstacle is outside limits
    if (index < offset+begin || index > offset+end)
      continue;

    std::vector<int> indices;
    indices.push_back(index);

    for (int neighbourIdx = 1; neighbourIdx < floor(cfg_->obstacles.obstacle_poses_affected/2); neighbourIdx++)
    {
      if (index + neighbourIdx <= offset+end)
        indices.push_back(index + neighbourIdx);
      if (index - neighbourIdx >= offset+begin) // needs to be casted to int to allow negative values
        indices.push_back(index - neighbourIdx);
    }

    // iterate all neighboring teb points
    for (int i = 0; i < indices.size(); ++i)
    {
      if (teb_.PoseVertex(indices.at(i))->fixed())
        continue; // skip pose if fixed

      if (influential)
      {
        EdgeInfluentialObstacle* dist_bandpt_obst = new EdgeInfluentialObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(indices.at(i)));
        dist_bandpt_obst->setInformation(information_influential);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
      }
      else if (inflated)
      {
        EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(indices.at(i)));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
      }
      else
      {
        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(indices.at(i)));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
      }
    }
  }
}


void TebPlanner::AddEdgesDynamicObstacles(size_t length, size_t offset, double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==NULL)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_dynamic_obstacle * weight_multiplier;
  information(1,1) = cfg_->optim.weight_dynamic_obstacle_inflation;
  information(0,1) = information(1,0) = 0;

  // define teb points limits (skip first and last if fixed)
  int begin = 0;
  int end = length - 1;
  if (teb_.PoseVertex(offset+begin)->fixed())
    ++begin;
  if (end > 0 && teb_.PoseVertex(offset+end)->fixed())
    --end;

  for (ObstacleContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (!(*obst)->isDynamic())
      continue;

    double time = 0;
    for (int i = 0; i < begin; ++i)
      time += teb_.TimeDiff(offset+i);

    // iterate all teb points inside limits
    for (int i = begin; i <= end; ++i)
    {
      if (teb_.PoseVertex(offset+i)->fixed())
        continue; // skip pose if fixed

      EdgeDynamicObstacle* dynobst_edge = new EdgeDynamicObstacle(time);
      dynobst_edge->setVertex(0,teb_.PoseVertex(offset+i));
      dynobst_edge->setInformation(information);
      dynobst_edge->setParameters(*cfg_, robot_model_.get(), obst->get());
      optimizer_->addEdge(dynobst_edge);

      if (i < length - 1)
        time += teb_.TimeDiff(offset+i);
    }
  }
}


void TebPlanner::AddEdgesViaPoints()
{
  if (cfg_->optim.weight_viapoint==0 || via_points_==NULL || via_points_->empty() )
    return; // if weight equals zero skip adding edges!

  int start_pose_idx = 0;

  int n = teb_.sizePoses();
  if (n<3) // we do not have any degrees of freedom for reaching via-points
    return;

  for (ViaPointContainer::const_iterator vp_it = via_points_->begin(); vp_it != via_points_->end(); ++vp_it)
  {

    int index = teb_.findClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
    if (cfg_->trajectory.via_points_ordered)
      start_pose_idx = index+2; // skip a point to have a DOF inbetween for further via-points

    // check if point conicides with goal or is located behind it
    if ( index > n-2 )
      index = n-2; // set to a pose before the goal, since we can move it away!
    // check if point coincides with start or is located before it
    if ( index < 1)
    {
      if (cfg_->trajectory.via_points_ordered)
      {
        index = 1; // try to connect the via point with the second (and non-fixed) pose. It is likely that autoresize adds new poses inbetween later.
      }
      else
      {
        ROS_DEBUG("TebPlanner::AddEdgesViaPoints(): skipping a via-point that is close or behind the current robot pose.");
        continue; // skip via points really close or behind the current robot pose
      }
    }
    Eigen::Matrix<double,1,1> information;
    information.fill(cfg_->optim.weight_viapoint);

    EdgeViaPoint* edge_viapoint = new EdgeViaPoint;
    edge_viapoint->setVertex(0,teb_.PoseVertex(index));
    edge_viapoint->setInformation(information);
    edge_viapoint->setParameters(*cfg_, &(*vp_it));
    optimizer_->addEdge(edge_viapoint);
  }
}


void TebPlanner::AddEdgesDynamics(size_t length, size_t offset, size_t shift)
{
  if (cfg_->robot.max_vel_y == 0 || cfg_->robot.acc_lim_y == 0) // non-holonomic robot
  {
    if ((cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_theta==0) &&
        (cfg_->optim.weight_acc_lim_x==0 && cfg_->optim.weight_acc_lim_theta==0) &&
       ((cfg_->optim.weight_profile_fidelity_v==0
      && cfg_->optim.weight_profile_fidelity_w==0
      && cfg_->optim.weight_profile_fidelity_t==0)
      || cfg_->trajectory.teb_autosize==true
      || trajectory_ref_==nullptr))
      return; // if velocity weights equal zero and profile weights equal zero or resizing enabled or no reference trajectory skip adding edges!

    const int n = length;

    Eigen::Matrix<double,12,12> information;
    information.fill(0);
    information( 0, 0) = cfg_->optim.weight_acc_lim_x;
    information( 1, 1) = cfg_->optim.weight_acc_lim_theta;
    information( 2, 2) = cfg_->optim.weight_max_vel_x;
    information( 3, 3) = cfg_->optim.weight_max_vel_theta;
    information( 4, 4) = cfg_->optim.weight_max_vel_x;
    information( 5, 5) = cfg_->optim.weight_max_vel_theta;
    information( 6, 6) = cfg_->optim.weight_profile_fidelity_v;
    information( 7, 7) = cfg_->optim.weight_profile_fidelity_w;
    information( 8, 8) = cfg_->optim.weight_profile_fidelity_t;
    information( 9, 9) = cfg_->optim.weight_profile_fidelity_v;
    information(10,10) = cfg_->optim.weight_profile_fidelity_w;
    information(11,11) = cfg_->optim.weight_profile_fidelity_t;

    Eigen::Matrix<double,2,2> information_acc;
    information_acc.fill(0);
    information_acc(0,0) = cfg_->optim.weight_acc_lim_x;
    information_acc(1,1) = cfg_->optim.weight_acc_lim_theta;

    // check if an initial velocity should be taken into account
    if (vel_start_.first)
    {
      EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(offset+0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(offset+1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(offset-shift+0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information_acc);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // add the dynamics edge for each tuple of three teb poses
    for (int i = 0; i < n - 2; ++i)
    {
      EdgeDynamics* dynamics_edge = new EdgeDynamics;
      dynamics_edge->setVertex(0,teb_.PoseVertex(offset+i+0));
      dynamics_edge->setVertex(1,teb_.PoseVertex(offset+i+1));
      dynamics_edge->setVertex(2,teb_.PoseVertex(offset+i+2));
      dynamics_edge->setVertex(3,teb_.TimeDiffVertex(offset-shift+i+0));
      dynamics_edge->setVertex(4,teb_.TimeDiffVertex(offset-shift+i+1));
      dynamics_edge->setInformation(information);
      dynamics_edge->setVelocities(
        trajectory_ref_ == nullptr
        ? std::pair<Velocity, Velocity>()
        : std::pair<Velocity, Velocity>(trajectory_ref_->trajectory().at(i+0)->velocity(),
                                        trajectory_ref_->trajectory().at(i+1)->velocity()));
      dynamics_edge->setTimesteps(
        trajectory_ref_ == nullptr
        ? std::pair<Timestep, Timestep>()
        : std::pair<Timestep, Timestep>(trajectory_ref_->trajectory().at(i+1)->timestamp() - trajectory_ref_->trajectory().at(i+0)->timestamp(),
                                        trajectory_ref_->trajectory().at(i+2)->timestamp() - trajectory_ref_->trajectory().at(i+1)->timestamp()));
      dynamics_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(dynamics_edge);
    }

    // check if a goal velocity should be taken into account
    if (vel_goal_.first)
    {
      EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(offset+n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(offset+n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(offset-shift+n-2));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information_acc);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
  }
  else // holonomic-robot
  {
    if ((cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_y==0 && cfg_->optim.weight_max_vel_theta==0) &&
        (cfg_->optim.weight_acc_lim_x==0 && cfg_->optim.weight_acc_lim_y==0 && cfg_->optim.weight_acc_lim_theta==0) &&
       ((cfg_->optim.weight_profile_fidelity_v==0
      && cfg_->optim.weight_profile_fidelity_w==0
      && cfg_->optim.weight_profile_fidelity_t==0)
      || cfg_->trajectory.teb_autosize==true
      || trajectory_ref_==nullptr))
      return; // if velocity weights equal zero and profile weights equal zero or resizing enabled or no reference trajectory skip adding edges!

    const int n = length;

    Eigen::Matrix<double,17,17> information;
    information.fill(0);
    information( 0, 0) = cfg_->optim.weight_acc_lim_x;
    information( 1, 1) = cfg_->optim.weight_acc_lim_y;
    information( 2, 2) = cfg_->optim.weight_acc_lim_theta;
    information( 3, 3) = cfg_->optim.weight_max_vel_x;
    information( 4, 4) = cfg_->optim.weight_max_vel_y;
    information( 5, 5) = cfg_->optim.weight_max_vel_theta;
    information( 6, 6) = cfg_->optim.weight_max_vel_x;
    information( 7, 7) = cfg_->optim.weight_max_vel_y;
    information( 8, 8) = cfg_->optim.weight_max_vel_theta;
    information( 9, 9) = cfg_->optim.weight_profile_fidelity_v;
    information(10,10) = cfg_->optim.weight_profile_fidelity_v;
    information(11,11) = cfg_->optim.weight_profile_fidelity_w;
    information(12,12) = cfg_->optim.weight_profile_fidelity_t;
    information(13,13) = cfg_->optim.weight_profile_fidelity_v;
    information(14,14) = cfg_->optim.weight_profile_fidelity_v;
    information(15,15) = cfg_->optim.weight_profile_fidelity_w;
    information(16,16) = cfg_->optim.weight_profile_fidelity_t;

    Eigen::Matrix<double,3,3> information_acc;
    information_acc.fill(0);
    information_acc(0,0) = cfg_->optim.weight_acc_lim_x;
    information_acc(1,1) = cfg_->optim.weight_acc_lim_y;
    information_acc(2,2) = cfg_->optim.weight_acc_lim_theta;

    // check if an initial velocity should be taken into account
    if (vel_start_.first)
    {
      EdgeAccelerationHolonomicStart* acceleration_edge = new EdgeAccelerationHolonomicStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(offset+0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(offset+1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(offset-shift+0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information_acc);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // add the dynamics edge for each tuple of three teb poses
    for (int i = 0; i < n - 2; ++i)
    {
      EdgeDynamicsHolonomic* dynamics_edge = new EdgeDynamicsHolonomic;
      dynamics_edge->setVertex(0,teb_.PoseVertex(offset+i+0));
      dynamics_edge->setVertex(1,teb_.PoseVertex(offset+i+1));
      dynamics_edge->setVertex(2,teb_.PoseVertex(offset+i+2));
      dynamics_edge->setVertex(3,teb_.TimeDiffVertex(offset-shift+i+0));
      dynamics_edge->setVertex(4,teb_.TimeDiffVertex(offset-shift+i+1));
      dynamics_edge->setInformation(information);
      dynamics_edge->setVelocities(
        trajectory_ref_ == nullptr
        ? std::pair<Velocity, Velocity>()
        : std::pair<Velocity, Velocity>(trajectory_ref_->trajectory().at(i+0)->velocity(),
                                        trajectory_ref_->trajectory().at(i+1)->velocity()));
      dynamics_edge->setTimesteps(
        trajectory_ref_ == nullptr
        ? std::pair<Timestep, Timestep>()
        : std::pair<Timestep, Timestep>(trajectory_ref_->trajectory().at(i+1)->timestamp() - trajectory_ref_->trajectory().at(i+0)->timestamp(),
                                        trajectory_ref_->trajectory().at(i+2)->timestamp() - trajectory_ref_->trajectory().at(i+1)->timestamp()));
      dynamics_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(dynamics_edge);
    }

    // check if a goal velocity should be taken into account
    if (vel_goal_.first)
    {
      EdgeAccelerationHolonomicGoal* acceleration_edge = new EdgeAccelerationHolonomicGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(offset+n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(offset+n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(offset-shift+n-2));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information_acc);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
  }
}


void TebPlanner::AddEdgesVelocity(size_t length, size_t offset, size_t shift)
{
  if (cfg_->robot.max_vel_y == 0) // non-holonomic robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!

    int n = length;
    Eigen::Matrix<double,2,2> information;
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_theta;
    information(0,1) = 0.0;
    information(1,0) = 0.0;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocity* velocity_edge = new EdgeVelocity;
      velocity_edge->setVertex(0,teb_.PoseVertex(offset+i));
      velocity_edge->setVertex(1,teb_.PoseVertex(offset+i+1));
      velocity_edge->setVertex(2,teb_.TimeDiffVertex(offset-shift+i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    }
  }
  else // holonomic-robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_y==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!

    int n = length;
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_y;
    information(2,2) = cfg_->optim.weight_max_vel_theta;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocityHolonomic* velocity_edge = new EdgeVelocityHolonomic;
      velocity_edge->setVertex(0,teb_.PoseVertex(offset+i));
      velocity_edge->setVertex(1,teb_.PoseVertex(offset+i+1));
      velocity_edge->setVertex(2,teb_.TimeDiffVertex(offset-shift+i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    }

  }
}

void TebPlanner::AddEdgesAcceleration(size_t length, size_t offset, size_t shift)
{
  if (cfg_->optim.weight_acc_lim_x==0  && cfg_->optim.weight_acc_lim_theta==0)
    return; // if weight equals zero skip adding edges!

  int n = length;

  if (cfg_->robot.max_vel_y == 0 || cfg_->robot.acc_lim_y == 0) // non-holonomic robot
  {
    Eigen::Matrix<double,2,2> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_theta;

    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(offset+0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(offset+1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(offset-shift+0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
      acceleration_edge->setVertex(0,teb_.PoseVertex(offset+i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(offset+i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(offset+i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(offset-shift+i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(offset-shift+i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(offset+n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(offset+n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(offset-shift+n-2));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
  }
  else // holonomic robot
  {
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_y;
    information(2,2) = cfg_->optim.weight_acc_lim_theta;

    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationHolonomicStart* acceleration_edge = new EdgeAccelerationHolonomicStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(offset+0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(offset+1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(offset-shift+0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAccelerationHolonomic* acceleration_edge = new EdgeAccelerationHolonomic;
      acceleration_edge->setVertex(0,teb_.PoseVertex(offset+i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(offset+i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(offset+i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(offset-shift+i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(offset-shift+i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationHolonomicGoal* acceleration_edge = new EdgeAccelerationHolonomicGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(offset+n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(offset+n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(offset-shift+n-2));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
  }
}


void TebPlanner::AddEdgesProfileFidelity(size_t length, size_t offset, size_t shift)
{
  if ((cfg_->optim.weight_profile_fidelity_v==0
    && cfg_->optim.weight_profile_fidelity_w==0
    && cfg_->optim.weight_profile_fidelity_t==0)
    || cfg_->trajectory.teb_autosize==true
    || trajectory_ref_==nullptr)
    return; // if weights equal zero or resizing enabled or no reference trajectory skip adding edges!

  Eigen::Matrix<double,3,3> information;
  information.fill(0);
  information(0,0) = cfg_->optim.weight_profile_fidelity_v;
  information(1,1) = cfg_->optim.weight_profile_fidelity_w;
  information(2,2) = cfg_->optim.weight_profile_fidelity_t;

  for (int i=0; i < length-1; ++i)
  {
    EdgeProfileFidelity* profile_fidelity_edge = new EdgeProfileFidelity;
    profile_fidelity_edge->setVertex(0,teb_.PoseVertex(offset+i));
    profile_fidelity_edge->setVertex(1,teb_.PoseVertex(offset+i+1));
    profile_fidelity_edge->setVertex(2,teb_.TimeDiffVertex(offset-shift+i));
    profile_fidelity_edge->setInformation(information);
    profile_fidelity_edge->setVelocity(trajectory_ref_->trajectory().at(i)->velocity());
    profile_fidelity_edge->setTimestep(trajectory_ref_->trajectory().at(i+1)->timestamp() - trajectory_ref_->trajectory().at(i)->timestamp());
    profile_fidelity_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(profile_fidelity_edge);
  }
}

void TebPlanner::AddEdgesShortestPath(size_t length, size_t offset)
{
  if (cfg_->optim.weight_shortest_path==0)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_shortest_path);

  for (int i=0; i < length-1; ++i)
  {
    EdgeShortestPath* shortest_path_edge = new EdgeShortestPath;
    shortest_path_edge->setVertex(0,teb_.PoseVertex(offset+i));
    shortest_path_edge->setVertex(1,teb_.PoseVertex(offset+i+1));
    shortest_path_edge->setInformation(information);
    shortest_path_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(shortest_path_edge);
  }
}

void TebPlanner::AddEdgesTimeOptimal(size_t length, size_t offset, size_t shift)
{
  if (cfg_->optim.weight_optimaltime==0)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_optimaltime);

  for (int i=0; i < length-1; ++i)
  {
    EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
    timeoptimal_edge->setVertex(0,teb_.TimeDiffVertex(offset-shift+i));
    timeoptimal_edge->setInformation(information);
    timeoptimal_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(timeoptimal_edge);
  }
}


void TebPlanner::AddEdgesKinematicsDiffDrive(size_t length, size_t offset)
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_forward_drive==0)
    return; // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;

  for (int i=0; i < length-1; i++) // ignore twiced start only
  {
    if (teb_.PoseVertex(offset+i)->fixed() && teb_.PoseVertex(offset+i+1)->fixed())
      continue;

    EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
    kinematics_edge->setVertex(0,teb_.PoseVertex(offset+i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(offset+i+1));
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }
}

void TebPlanner::AddEdgesKinematicsCarlike(size_t length, size_t offset)
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_turning_radius==0)
    return; // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_turning_radius;

  for (int i=0; i < length-1; i++) // ignore twiced start only
  {
    if (teb_.PoseVertex(offset+i)->fixed() && teb_.PoseVertex(offset+i+1)->fixed())
      continue;

    EdgeKinematicsCarlike* kinematics_edge = new EdgeKinematicsCarlike;
    kinematics_edge->setVertex(0,teb_.PoseVertex(offset+i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(offset+i+1));
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }
}


void TebPlanner::AddEdgesPreferRotDir(size_t length, size_t offset)
{
  //TODO(roesmann): Note, these edges can result in odd predictions, in particular
  //                we can observe a substantional mismatch between open- and closed-loop planning
  //                leading to a poor control performance.
  //                At the moment, we keep these functionality for oscillation recovery:
  //                Activating the edge for a short time period might not be crucial and
  //                could move the robot to a new oscillation-free state.
  //                This needs to be analyzed in more detail!
  if (prefer_rotdir_ == RotType::none || cfg_->optim.weight_prefer_rotdir==0)
    return; // if weight equals zero skip adding edges!

  if (prefer_rotdir_ != RotType::right && prefer_rotdir_ != RotType::left)
  {
    ROS_WARN("TebPlanner::AddEdgesPreferRotDir(): unsupported RotType selected. Skipping edge creation.");
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,1,1> information_rotdir;
  information_rotdir.fill(cfg_->optim.weight_prefer_rotdir);

  for (int i=0; i < length-1 && i < 3; ++i) // currently: apply to first 3 rotations
  {
    EdgePreferRotDir* rotdir_edge = new EdgePreferRotDir;
    rotdir_edge->setVertex(0,teb_.PoseVertex(offset+i));
    rotdir_edge->setVertex(1,teb_.PoseVertex(offset+i+1));
    rotdir_edge->setInformation(information_rotdir);

    if (prefer_rotdir_ == RotType::left)
        rotdir_edge->preferLeft();
    else if (prefer_rotdir_ == RotType::right)
        rotdir_edge->preferRight();

    optimizer_->addEdge(rotdir_edge);
  }
}

void TebPlanner::computeCurrentCost(const Trajectory* initial_plan, const double obst_cost_scale, const double viapoint_cost_scale, const bool alternative_time_cost)
{
  trajectory_ref_ = initial_plan;
  computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
  trajectory_ref_ = nullptr;
}

void TebPlanner::computeCurrentCost(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{
  // check if graph is empty/exist  -> important if function is called between buildGraph and optimizeGraph/clearGraph
  bool graph_exist_flag(false);
  if (optimizer_->edges().empty() && optimizer_->vertices().empty())
  {
    // here the graph is build again, for time efficiency make sure to call this function
    // between buildGraph and Optimize (deleted), but it depends on the application
    buildGraph();
    optimizer_->initializeOptimization();
  }
  else
  {
    graph_exist_flag = true;
  }

  optimizer_->computeInitialGuess();

  cost_ = 0;

  if (alternative_time_cost)
  {
    cost_ += teb_.getSumOfAllTimeDiffs();
    // TEST we use SumOfAllTimeDiffs() here, because edge cost depends on number of samples, which is not always the same for similar TEBs,
    // since we are using an AutoResize Function with hysteresis.
  }

  // now we need pointers to all edges -> calculate error for each edge-type
  // since we aren't storing edge pointers, we need to check every edge
  for (std::vector<g2o::OptimizableGraph::Edge*>::const_iterator it = optimizer_->activeEdges().begin(); it!= optimizer_->activeEdges().end(); it++)
  {
    double cur_cost = (*it)->chi2();

    if (dynamic_cast<EdgeObstacle*>(*it) != nullptr
        || dynamic_cast<EdgeInflatedObstacle*>(*it) != nullptr
        || dynamic_cast<EdgeDynamicObstacle*>(*it) != nullptr)
    {
      cur_cost *= obst_cost_scale;
    }
    else if (dynamic_cast<EdgeViaPoint*>(*it) != nullptr)
    {
      cur_cost *= viapoint_cost_scale;
    }
    else if (dynamic_cast<EdgeTimeOptimal*>(*it) != nullptr && alternative_time_cost)
    {
      continue; // skip these edges if alternative_time_cost is active
    }
    cost_ += cur_cost;
  }

  // delete temporary created graph
  if (!graph_exist_flag)
    clearGraph();
}


void TebPlanner::extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt, double& vx, double& vy, double& omega) const
{
  if (dt == 0)
  {
    vx = 0;
    vy = 0;
    omega = 0;
    return;
  }

  Eigen::Vector2d deltaS = pose2.position() - pose1.position();

  if (cfg_->robot.max_vel_y == 0) // nonholonomic robot
  {
    Eigen::Vector2d conf1dir( cos(pose1.theta()), sin(pose1.theta()) );
    // translational velocity
    double dir = deltaS.dot(conf1dir);
    vx = (double) g2o::sign(dir) * deltaS.norm()/dt;
    vy = 0;
  }
  else // holonomic robot
  {
    // transform pose 2 into the current robot frame (pose1)
    // for velocities only the rotation of the direction vector is necessary.
    // (map->pose1-frame: inverse 2d rotation matrix)
    double cos_theta1 = std::cos(pose1.theta());
    double sin_theta1 = std::sin(pose1.theta());
    double p1_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
    double p1_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();
    vx = p1_dx / dt;
    vy = p1_dy / dt;
  }

  // rotational velocity
  double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
  omega = orientdiff/dt;
}

bool TebPlanner::getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses) const
{
  if (teb_.sizePoses()<2)
  {
    ROS_ERROR("TebPlanner::getVelocityCommand(): The trajectory contains less than 2 poses. Make sure to init and optimize/plan the trajectory fist.");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
  look_ahead_poses = std::max(1, std::min(look_ahead_poses, teb_.sizePoses() - 1));
  double dt = 0.0;
  for(int counter = 0; counter < look_ahead_poses; ++counter)
    dt += teb_.TimeDiff(counter);
  if (dt<=0)
  {
    ROS_ERROR("TebPlanner::getVelocityCommand() - timediff<=0 is invalid!");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }

  // Get velocity from the first two configurations
  extractVelocity(teb_.Pose(0), teb_.Pose(look_ahead_poses), dt, vx, vy, omega);
  return true;
}

void TebPlanner::getVelocityProfile(VelocityContainer& velocity_profile) const
{
  const size_t n = teb_.sizePoses();
  velocity_profile.resize(n + 1);

  double strafing; // useless variable

  // initialize velocities
  for (size_t i = 0; i < velocity_profile.size(); ++i)
  {
    if (velocity_profile.at(i) == nullptr)
      velocity_profile.at(i) = VelocityPtr(new Velocity());
  }

  // start velocity
  velocity_profile.front()->orientation() = teb_.Pose(0).theta();
  velocity_profile.front()->translation() = vel_start_.second.linear.x;
  velocity_profile.front()->rotation()    = vel_start_.second.angular.z;

  // intermediate velocities
  for (size_t i = 1; i < n; ++i)
  {
    velocity_profile.at(i)->orientation() = teb_.Pose(i-1).theta();
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), velocity_profile.at(i)->translation(), strafing, velocity_profile.at(i)->rotation());
  }

  // goal velocity
  velocity_profile.back()->orientation() = teb_.BackPose().theta();
  velocity_profile.back()->translation() = vel_goal_.second.linear.x;
  velocity_profile.back()->rotation()    = vel_goal_.second.angular.z;

  // update velocities
  for (size_t i = 0; i < velocity_profile.size(); ++i)
  {
    velocity_profile.at(i)->update();
  }
}

void TebPlanner::getVelocityProfile(std::vector<geometry_msgs::Twist>& velocity_profile) const
{
  int n = teb_.sizePoses();
  velocity_profile.resize( n+1 );

  // start velocity
  velocity_profile.front().linear.z = 0;
  velocity_profile.front().angular.x = velocity_profile.front().angular.y = 0;
  velocity_profile.front().linear.x = vel_start_.second.linear.x;
  velocity_profile.front().linear.y = vel_start_.second.linear.y;
  velocity_profile.front().angular.z = vel_start_.second.angular.z;

  for (int i=1; i<n; ++i)
  {
    velocity_profile[i].linear.z = 0;
    velocity_profile[i].angular.x = velocity_profile[i].angular.y = 0;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), velocity_profile[i].linear.x, velocity_profile[i].linear.y, velocity_profile[i].angular.z);
  }

  // goal velocity
  velocity_profile.back().linear.z = 0;
  velocity_profile.back().angular.x = velocity_profile.back().angular.y = 0;
  velocity_profile.back().linear.x = vel_goal_.second.linear.x;
  velocity_profile.back().linear.y = vel_goal_.second.linear.y;
  velocity_profile.back().angular.z = vel_goal_.second.angular.z;
}

void TebPlanner::getFullTrajectory(std::vector<TrajectoryPtr>& trajectories, std::vector<Timestamp> timestamps) const
{
  for (size_t r = 0; r < std::min(trajectories.size(), offsets_.size() - 1); ++r)
  {
    const size_t offset = offsets_.at(r);
    const size_t n = offsets_.at(r + 1) - offsets_.at(r);

    trajectories.at(r)->resize(n);

    if (n == 0)
      continue;

    TimestepContainer timestep_profile(n - 1);
    PoseSE2Container pose_profile(n);

    // timestep profile
    for (size_t i = 0; i < timestep_profile.size(); ++i)
    {
      timestep_profile.at(i) = TimestepPtr(new Timestep(timestep_t(teb_.TimeDiff(offset-r+i))));
    }
    // pose profile
    for (size_t i = 0; i < pose_profile.size(); ++i)
    {
      pose_profile.at(i) = PoseSE2Ptr(new PoseSE2(teb_.Pose(offset+i).x(), teb_.Pose(offset+i).y(), teb_.Pose(offset+i).theta()));
    }

    // insert timestep and pose profiles (velocity and acceleration profiles are computed automatically)
    trajectories.at(r)->setProfileTimestep(timestep_profile, timestamps.at(r), false);
    trajectories.at(r)->setProfilePose(pose_profile);
  }
}

void TebPlanner::getFullTrajectory(Trajectory& trajectory, Timestamp timestamp) const
{
  const size_t n = teb_.sizePoses();

  trajectory.resize(n);

  if (n == 0)
    return;

  TimestepContainer timestep_profile(n - 1);
  PoseSE2Container pose_profile(n);

  // timestep profile
  for (size_t i = 0; i < timestep_profile.size(); ++i)
  {
    timestep_profile.at(i) = TimestepPtr(new Timestep(timestep_t(teb_.TimeDiff(i))));
  }
  // pose profile
  for (size_t i = 0; i < pose_profile.size(); ++i)
  {
    pose_profile.at(i) = PoseSE2Ptr(new PoseSE2(teb_.Pose(i).x(), teb_.Pose(i).y(), teb_.Pose(i).theta()));
  }

  // insert timestep and pose profiles (velocity and acceleration profiles are computed automatically)
  trajectory.setProfileTimestep(timestep_profile, timestamp, false);
  trajectory.setProfilePose(pose_profile);
}

void TebPlanner::getFullTrajectory(std::vector<teb_local_planner::TrajectoryPointMsg>& trajectory) const
{
  int n = teb_.sizePoses();

  trajectory.resize(n);

  if (n == 0)
    return;

  double curr_time = 0;

  // start
  teb_local_planner::TrajectoryPointMsg& start = trajectory.front();
  teb_.Pose(0).toPoseMsg(start.pose);
  start.velocity.linear.z = 0;
  start.velocity.angular.x = start.velocity.angular.y = 0;
  start.velocity.linear.x = vel_start_.second.linear.x;
  start.velocity.linear.y = vel_start_.second.linear.y;
  start.velocity.angular.z = vel_start_.second.angular.z;
  start.time_from_start.fromSec(curr_time);

  curr_time += teb_.TimeDiff(0);

  // intermediate points
  for (int i=1; i < n-1; ++i)
  {
    teb_local_planner::TrajectoryPointMsg& point = trajectory[i];
    teb_.Pose(i).toPoseMsg(point.pose);
    point.velocity.linear.z = 0;
    point.velocity.angular.x = point.velocity.angular.y = 0;
    double vel1_x, vel1_y, vel2_x, vel2_y, omega1, omega2;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), vel1_x, vel1_y, omega1);
    extractVelocity(teb_.Pose(i), teb_.Pose(i+1), teb_.TimeDiff(i), vel2_x, vel2_y, omega2);
    point.velocity.linear.x = 0.5*(vel1_x+vel2_x);
    point.velocity.linear.y = 0.5*(vel1_y+vel2_y);
    point.velocity.angular.z = 0.5*(omega1+omega2);
    point.time_from_start.fromSec(curr_time);

    curr_time += teb_.TimeDiff(i);
  }

  // goal
  teb_local_planner::TrajectoryPointMsg& goal = trajectory.back();
  teb_.BackPose().toPoseMsg(goal.pose);
  goal.velocity.linear.z = 0;
  goal.velocity.angular.x = goal.velocity.angular.y = 0;
  goal.velocity.linear.x = vel_goal_.second.linear.x;
  goal.velocity.linear.y = vel_goal_.second.linear.y;
  goal.velocity.angular.z = vel_goal_.second.angular.z;
  goal.time_from_start.fromSec(curr_time);
}


bool TebPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                             double inscribed_radius, double circumscribed_radius, int look_ahead_idx)
{
  if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
    look_ahead_idx = teb().sizePoses() - 1;

  for (int i=0; i <= look_ahead_idx; ++i)
  {
    if ( costmap_model->footprintCost(teb().Pose(i).x(), teb().Pose(i).y(), teb().Pose(i).theta(), footprint_spec, inscribed_radius, circumscribed_radius) < 0 )
    {
      if (visualization_)
      {
        visualization_->publishInfeasibleRobotPose(teb().Pose(i), *robot_model_);
      }
      return false;
    }
    // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold
    // and interpolates in that case.
    // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!
    if (i<look_ahead_idx)
    {
      double delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().Pose(i+1).theta()) -
                                              g2o::normalize_theta(teb().Pose(i).theta()));
      Eigen::Vector2d delta_dist = teb().Pose(i+1).position()-teb().Pose(i).position();
      if(fabs(delta_rot) > cfg_->trajectory.min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
      {
        int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / cfg_->trajectory.min_resolution_collision_check_angular),
                                            std::ceil(delta_dist.norm() / inscribed_radius)) - 1;
        PoseSE2 intermediate_pose = teb().Pose(i);
        for(int step = 0; step < n_additional_samples; ++step)
        {
          intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
          intermediate_pose.theta() = g2o::normalize_theta(intermediate_pose.theta() +
                                                           delta_rot / (n_additional_samples + 1.0));
          if ( costmap_model->footprintCost(intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta(),
            footprint_spec, inscribed_radius, circumscribed_radius) == -1 )
          {
            if (visualization_)
            {
              visualization_->publishInfeasibleRobotPose(intermediate_pose, *robot_model_);
            }
            return false;
          }
        }
      }
    }
  }
  return true;
}


bool TebPlanner::isHorizonReductionAppropriate(const std::vector<geometry_msgs::PoseStamped>& initial_plan) const
{
  if (teb_.sizePoses() < int( 1.5*double(cfg_->trajectory.min_samples) ) ) // trajectory is short already
    return false;

  // check if distance is at least 2m long // hardcoded for now
  double dist = 0;
  for (int i=1; i < teb_.sizePoses(); ++i)
  {
    dist += ( teb_.Pose(i).position() - teb_.Pose(i-1).position() ).norm();
    if (dist > 2)
      break;
  }
  if (dist <= 2)
    return false;

  // check if goal orientation is differing with more than 90° and the horizon is still long enough to exclude parking maneuvers.
  // use case: Sometimes the robot accomplish the following navigation task:
  // 1. wall following 2. 180° curve 3. following along the other side of the wall.
  // If the trajectory is too long, the trajectory might intersect with the obstace and the optimizer does
  // push the trajectory to the correct side.
  if ( std::abs( g2o::normalize_theta( teb_.Pose(0).theta() - teb_.BackPose().theta() ) ) > M_PI/2)
  {
    ROS_DEBUG("TebPlanner::isHorizonReductionAppropriate(): Goal orientation - start orientation > 90° ");
    return true;
  }

  // check if goal heading deviates more than 90° w.r.t. start orienation
  if (teb_.Pose(0).orientationUnitVec().dot(teb_.BackPose().position() - teb_.Pose(0).position()) < 0)
  {
    ROS_DEBUG("TebPlanner::isHorizonReductionAppropriate(): Goal heading - start orientation > 90° ");
    return true;
  }

  // check ratio: distance along the inital plan and distance of the trajectory (maybe too much is cut off)
  int idx=0; // first get point close to the robot (should be fast if the global path is already pruned!)
  for (; idx < (int)initial_plan.size(); ++idx)
  {
    if ( std::sqrt(std::pow(initial_plan[idx].pose.position.x-teb_.Pose(0).x(), 2) + std::pow(initial_plan[idx].pose.position.y-teb_.Pose(0).y(), 2)) )
      break;
  }
  // now calculate length
  double ref_path_length = 0;
  for (; idx < int(initial_plan.size())-1; ++idx)
  {
    ref_path_length += std::sqrt(std::pow(initial_plan[idx+1].pose.position.x-initial_plan[idx].pose.position.x, 2)
                     + std::pow(initial_plan[idx+1].pose.position.y-initial_plan[idx].pose.position.y, 2) );
  }

  // check distances along the teb trajectory (by the way, we also check if the distance between two poses is > obst_dist)
  double teb_length = 0;
  for (int i = 1; i < teb_.sizePoses(); ++i )
  {
    double dist = (teb_.Pose(i).position() - teb_.Pose(i-1).position()).norm();
    if (dist > 0.95*cfg_->obstacles.min_obstacle_dist)
    {
      ROS_DEBUG("TebPlanner::isHorizonReductionAppropriate(): Distance between consecutive poses > 0.9*min_obstacle_dist");
      return true;
    }
    ref_path_length += dist;
  }
  if (ref_path_length>0 && teb_length/ref_path_length < 0.7) // now check ratio
  {
    ROS_DEBUG("TebPlanner::isHorizonReductionAppropriate(): Planned trajectory is at least 30° shorter than the initial plan");
    return true;
  }


  // otherwise we do not suggest shrinking the horizon:
  return false;
}

} // namespace elastic_band
