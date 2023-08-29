//
// Created by yaozhuo on 2021/11/28.
//

#include "optimal_planner.h"

// g2o custom edges and vertices for the TEB planner
#include "teb_edges.h"
#include "teb_vertex.h"

#include <memory>
#include <limits>

#include <boost/thread/once.hpp>

// ============== Implementation ===================

    TebOptimalPlanner::TebOptimalPlanner() : cfg_(NULL), via_points_(NULL), cost_(HUGE_VAL), prefer_rotdir_(RotType::none),
                                             robot_model_(new PointRobotFootprint()), initialized_(false), optimized_(false)
    {
    }

    TebOptimalPlanner::TebOptimalPlanner(const TebConfig& cfg, 
                                         const DIST_TO_OBSTACLE_FUNC& dist_func, 
                                         RobotFootprintModelPtr robot_model, const ViaPointContainer* via_points)
    {
        initialize(cfg, dist_func, robot_model, via_points);
    }

    TebOptimalPlanner::~TebOptimalPlanner()
    {
        clearGraph();
        // free dynamically allocated memory
        //if (optimizer_)
        //  g2o::Factory::destroy();
        //g2o::OptimizationAlgorithmFactory::destroy();
        //g2o::HyperGraphActionLibrary::destroy();
    }

    void TebOptimalPlanner::updateRobotModel(RobotFootprintModelPtr robot_model)
    {
        robot_model_ = robot_model;
    }

    void TebOptimalPlanner::initialize(const TebConfig& cfg, 
                                       const DIST_TO_OBSTACLE_FUNC& dist_func, 
                                       const RobotFootprintModelPtr& robot_model, 
                                       const ViaPointContainer* via_points)
    {
        // init optimizer (set solver and block ordering settings)
        optimizer_ = initOptimizer();

        cfg_ = &cfg;
        robot_model_ = robot_model;
        via_points_ = via_points;
        cost_ = HUGE_VAL;
        prefer_rotdir_ = RotType::none;

        vel_start_.first = true;
        vel_start_.second.x() = 0;
        vel_start_.second.y() = 0;
        vel_start_.second.z() = 0;

        vel_goal_.first = true;
        vel_goal_.second.x() = 0;
        vel_goal_.second.y() = 0;
        vel_goal_.second.z() = 0;
        initialized_ = true;

        dist_func_ = dist_func;
    }

/*
 * registers custom vertices and edges in g2o framework
 */
    void TebOptimalPlanner::registerG2OTypes()
    {
        g2o::Factory* factory = g2o::Factory::instance();
        factory->registerType("VERTEX_POSE",             std::make_shared<g2o::HyperGraphElementCreator<VertexPose>>());
        factory->registerType("VERTEX_TIMEDIFF",         std::make_shared<g2o::HyperGraphElementCreator<VertexTimeDiff>>());

        factory->registerType("EDGE_TIME_OPTIMAL",       std::make_shared<g2o::HyperGraphElementCreator<EdgeTimeOptimal>>());
        factory->registerType("EDGE_SHORTEST_PATH",      std::make_shared<g2o::HyperGraphElementCreator<EdgeShortestPath>>());
        factory->registerType("EDGE_VELOCITY",           std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocity>>());
        factory->registerType("EDGE_VELOCITY_HOLONOMIC", std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>>());
        factory->registerType("EDGE_VIA_POINT",          std::make_shared<g2o::HyperGraphElementCreator<EdgeViaPoint>>());
        factory->registerType("EDGE_ACCELERATION",       std::make_shared<g2o::HyperGraphElementCreator<EdgeAcceleration>>());
        factory->registerType("EDGE_ACCELERATION_START",     std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationStart>>());
        factory->registerType("EDGE_ACCELERATION_GOAL",      std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationGoal>>());
        factory->registerType("EDGE_ACCELERATION_HOLONOMIC", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>>());
        factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>>());
        factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>>());
        return;
    }

    /*
     * initialize g2o optimizer. Set solver settings here.
     * Return: pointer to new SparseOptimizer Object.
     */
    boost::shared_ptr<g2o::SparseOptimizer> TebOptimalPlanner::initOptimizer()
    {
        // Call register_g2o_types once, even for multiple TebOptimalPlanner instances (thread-safe)
        static boost::once_flag flag = BOOST_ONCE_INIT;
        //boost::call_once(&registerG2OTypes, flag);

        // allocating the optimizer
        boost::shared_ptr<g2o::SparseOptimizer> optimizer = boost::make_shared<g2o::SparseOptimizer>();
        std::unique_ptr<TEBLinearSolver> linear_solver(new TEBLinearSolver()); // see typedef in optimization.h
        //linear_solver->setBlockOrdering(true);
        std::unique_ptr<TEBBlockSolver> block_solver(new TEBBlockSolver(std::move(linear_solver)));
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

        optimizer->setAlgorithm(solver);

        //optimizer->initMultiThreading(); // required for >Eigen 3.1

        return optimizer;
    }


    bool TebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards,
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

    void TebOptimalPlanner::setVelocityStart(const Eigen::Vector3d& vel_start)
    {
        vel_start_.first = true;
        vel_start_.second = vel_start;
    }

    void TebOptimalPlanner::setVelocityGoal(const Eigen::Vector3d& vel_goal)
    {
        vel_goal_.first = true;
        vel_goal_.second = vel_goal;
    }

    bool TebOptimalPlanner::plan(const std::vector<PoseSE2>& initial_plan, const Eigen::Vector3d start_vel, bool free_goal_vel)
    {
        if(!initialized_)
            std::cerr << "Call initialize() first." << std::endl;
        if (!teb_.isInit())
        {
            teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation,
                                      cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
        }
        else // warm start
        {
            PoseSE2 start_(initial_plan.front());
            PoseSE2 goal_(initial_plan.back());
            if (teb_.sizePoses()>0
                && (goal_.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist
                && fabs(g2o::normalize_theta(goal_.theta() - teb_.BackPose().theta())) < cfg_->trajectory.force_reinit_new_goal_angular) // actual warm start!
                teb_.updateAndPruneTEB(start_, goal_, cfg_->trajectory.min_samples); // update TEB
            else // goal too far away -> reinit
            {
                printf("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
                teb_.clearTimedElasticBand();
                teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation,
                                          cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
            }
        }
        setVelocityStart(start_vel);
        if (free_goal_vel)
            setVelocityGoalFree();
        else
            vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)

        // now optimize
        return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
    }

    bool TebOptimalPlanner::buildGraph(double weight_multiplier)
    {
        if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
        {
            printf("Cannot build graph, because it is not empty. Call graphClear()!");
            return false;
        }

        optimizer_->setComputeBatchStatistics(cfg_->recovery.divergence_detection_enable);

        // add TEB vertices, including PoseSE2 and time interval
        AddTEBVertices();

        /* push the result trajectory to close to a series of coordinate */
        AddEdgesViaPoints();

        /* try to limit the velocity to a specific window */
        AddEdgesVelocity();

        /* push the trajectory to cost less time */
        AddEdgesTimeOptimal();

        /* push the trajectory to shorter */
        AddEdgesShortestPath();

        /* try to limit the acceleration to a specific window */
        AddEdgesAcceleration();

        /* yz: add strafing constraint, limit robot from move in y direction when change direction */
        AddEdgesKinematicsDiffDrive();

        /* yz: avoid collide with obstaccle in grid map */
        //AddEdgesObstaclesGridMap(cfg_->optim.weight_adapt_factor);

        return true;
    }


    void TebOptimalPlanner::AddEdgesViaPoints()
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

            // check if point coincides with goal or is located behind it
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
                    printf("TebOptimalPlanner::AddEdgesViaPoints(): skipping a via-point that is close or behind the current robot pose.");
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

    void TebOptimalPlanner::AddEdgesVelocity()
    {
        if (cfg_->robot.max_vel_y == 0) // non-holonomic robot
        {
            if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_theta==0)
                return; // if weight equals zero skip adding edges!

            int n = teb_.sizePoses();
            Eigen::Matrix<double,2,2> information;
            information(0,0) = cfg_->optim.weight_max_vel_x;
            information(1,1) = cfg_->optim.weight_max_vel_theta;
            information(0,1) = 0.0;
            information(1,0) = 0.0;

            for (int i=0; i < n - 1; ++i)
            {
                EdgeVelocity* velocity_edge = new EdgeVelocity;
                velocity_edge->setVertex(0,teb_.PoseVertex(i));
                velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
                velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
                velocity_edge->setInformation(information);
                velocity_edge->setTebConfig(*cfg_);
                optimizer_->addEdge(velocity_edge);
            }
        }
        else // holonomic-robot
        {
            if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_y==0 && cfg_->optim.weight_max_vel_theta==0)
                return; // if weight equals zero skip adding edges!

            int n = teb_.sizePoses();
            Eigen::Matrix<double,3,3> information;
            information.fill(0);
            information(0,0) = cfg_->optim.weight_max_vel_x;
            information(1,1) = cfg_->optim.weight_max_vel_y;
            information(2,2) = cfg_->optim.weight_max_vel_theta;

            for (int i=0; i < n - 1; ++i)
            {
                EdgeVelocityHolonomic* velocity_edge = new EdgeVelocityHolonomic;
                velocity_edge->setVertex(0,teb_.PoseVertex(i));
                velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
                velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
                velocity_edge->setInformation(information);
                velocity_edge->setTebConfig(*cfg_);
                optimizer_->addEdge(velocity_edge);
            }

        }
    }


    bool TebOptimalPlanner::optimizeGraph(int no_iterations,bool clear_after)
    {
        if (cfg_->robot.max_vel_x<0.01)
        {
            printf("optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. Optimizing aborted...");
            if (clear_after) clearGraph();
            return false;
        }

        if (!teb_.isInit() || teb_.sizePoses() < cfg_->trajectory.min_samples)
        {
            printf("optimizeGraph(): TEB is empty or has too less elements. Skipping optimization.");
            if (clear_after) clearGraph();
            return false;
        }

        optimizer_->setVerbose(cfg_->optim.optimization_verbose);
        optimizer_->initializeOptimization();

        int iter = optimizer_->optimize(no_iterations);

        // Save Hessian for visualization
        //  g2o::OptimizationAlgorithmLevenberg* lm = dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
        //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");

        if(!iter)
        {
            std::cerr << "optimizeGraph(): Optimization failed! iter=" << iter << std::endl;
            return false;
        }

        if (clear_after) clearGraph();

        return true;
    }

    void TebOptimalPlanner::clearGraph()
    {
        // clear optimizer states
        if (optimizer_)
        {
            // we will delete all edges but keep the vertices.
            // before doing so, we will delete the link from the vertices to the edges.
            auto& vertices = optimizer_->vertices();
            for(auto& v : vertices)
                v.second->edges().clear();

            optimizer_->vertices().clear();  // necessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
            optimizer_->clear();
        }
    }



    void TebOptimalPlanner::AddTEBVertices()
    {
        // add vertices to graph
        //std::cout << "Adding TEB vertices ..." << std::endl;
        unsigned int id_counter = 0; // used for vertices ids
        for (int i=0; i<teb_.sizePoses(); ++i)
        {
            teb_.PoseVertex(i)->setId(id_counter++);
            optimizer_->addVertex(teb_.PoseVertex(i));
            if (teb_.sizeTimeDiffs()!=0 && i<teb_.sizeTimeDiffs())
            {
                teb_.TimeDiffVertex(i)->setId(id_counter++);
                optimizer_->addVertex(teb_.TimeDiffVertex(i));
            }
        }
    }


    void TebOptimalPlanner::AddEdgesTimeOptimal()
    {
        if (cfg_->optim.weight_optimaltime==0)
            return; // if weight equals zero skip adding edges!

        Eigen::Matrix<double,1,1> information;
        information.fill(cfg_->optim.weight_optimaltime);

        for (int i=0; i < teb_.sizeTimeDiffs(); ++i)
        {
            EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
            timeoptimal_edge->setVertex(0,teb_.TimeDiffVertex(i));
            timeoptimal_edge->setInformation(information);
            timeoptimal_edge->setTebConfig(*cfg_);
            optimizer_->addEdge(timeoptimal_edge);
        }
    }

    void TebOptimalPlanner::AddEdgesShortestPath()
    {
        if (cfg_->optim.weight_shortest_path==0)
            return; // if weight equals zero skip adding edges!

        Eigen::Matrix<double,1,1> information;
        information.fill(cfg_->optim.weight_shortest_path);

        for (int i=0; i < teb_.sizePoses()-1; ++i)
        {
            EdgeShortestPath* shortest_path_edge = new EdgeShortestPath;
            shortest_path_edge->setVertex(0,teb_.PoseVertex(i));
            shortest_path_edge->setVertex(1,teb_.PoseVertex(i+1));
            shortest_path_edge->setInformation(information);
            shortest_path_edge->setTebConfig(*cfg_);
            optimizer_->addEdge(shortest_path_edge);
        }
    }

    bool TebOptimalPlanner::hasDiverged() const
    {
        // Early returns if divergence detection is not active
        if (!cfg_->recovery.divergence_detection_enable)
            return false;

        auto stats_vector = optimizer_->batchStatistics();

        // No statistics yet
        if (stats_vector.empty())
            return false;

        // Grab the statistics of the final iteration
        const auto last_iter_stats = stats_vector.back();

        return last_iter_stats.chi2 > cfg_->recovery.divergence_detection_max_chi_squared;
    }

    void TebOptimalPlanner::computeCurrentCost(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
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

            if (dynamic_cast<EdgeViaPoint*>(*it) != nullptr)
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


    void TebOptimalPlanner::extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt, double& vx, double& vy, double& omega) const
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

    bool TebOptimalPlanner::getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses) const
    {
        if (teb_.sizePoses()<2)
        {
            std::cerr << "TebOptimalPlanner::getVelocityCommand(): The trajectory contains less than 2 poses. Make sure to init and optimize/plan the trajectory fist." << std::endl;
            vx = 0;
            vy = 0;
            omega = 0;
            return false;
        }
        look_ahead_poses = std::max(1, std::min(look_ahead_poses, teb_.sizePoses() - 1));
        double dt = 0.0;
        for(int counter = 0; counter < look_ahead_poses; ++counter)
        {
            dt += teb_.TimeDiff(counter);
            if(dt >= cfg_->trajectory.dt_ref * look_ahead_poses)  // TODO: change to look-ahead time? Refine trajectory?
            {
                look_ahead_poses = counter + 1;
                break;
            }
        }
        if (dt<=0)
        {
            std::cerr << "TebOptimalPlanner::getVelocityCommand() - timediff<=0 is invalid!" << std::endl;
            vx = 0;
            vy = 0;
            omega = 0;
            return false;
        }

        // Get velocity from the first two configurations
        extractVelocity(teb_.Pose(0), teb_.Pose(look_ahead_poses), dt, vx, vy, omega);
        return true;
    }

    void TebOptimalPlanner::getVelocityProfile(std::vector<Eigen::Vector3d>& velocity_profile) const
    {
        int n = teb_.sizePoses();
        velocity_profile.resize( n+1 );

        // start velocity
        velocity_profile.front().x() = vel_start_.second.x();
        velocity_profile.front().y() = vel_start_.second.y();
        velocity_profile.front().z() = vel_start_.second.z();

        for (int i=1; i<n; ++i)
        {
            extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), velocity_profile[i].x(), velocity_profile[i].y(), velocity_profile[i].z());
        }

        // goal velocity
        velocity_profile.back().x() = vel_goal_.second.x();
        velocity_profile.back().y() = vel_goal_.second.y();
        velocity_profile.back().z() = vel_goal_.second.z();

    }

    void TebOptimalPlanner::getFullTrajectory(std::vector<PoseSE2>& trajectory, std::vector<double>& time_diffs) const
    {
        int n = teb_.sizePoses();

        trajectory.resize(n);
        time_diffs.resize(n);
        if (n == 0)
            return;

        double curr_time = 0;

        // start
        trajectory.front() = teb_.Pose(0);
        time_diffs.front() = teb_.TimeDiff(0);
        // intermediate points
        for (int i=1; i < n-1; ++i)
        {
            trajectory[i] = teb_.Pose(i);
            time_diffs[i] = teb_.TimeDiff(i);
        }
        std::cout << std::endl;
        // goal
        trajectory.back() = teb_.BackPose();
        time_diffs.back() = teb_.BackTimeDiff();
    }


    void TebOptimalPlanner::AddEdgesAcceleration()
    {
        if (cfg_->optim.weight_acc_lim_x==0  && cfg_->optim.weight_acc_lim_theta==0)
            return; // if weight equals zero skip adding edges!

        int n = teb_.sizePoses();

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
                acceleration_edge->setVertex(0,teb_.PoseVertex(0));
                acceleration_edge->setVertex(1,teb_.PoseVertex(1));
                acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
                acceleration_edge->setInitialVelocity(vel_start_.second);
                acceleration_edge->setInformation(information);
                acceleration_edge->setTebConfig(*cfg_);
                optimizer_->addEdge(acceleration_edge);
            }

            // now add the usual acceleration edge for each tuple of three teb poses
            for (int i=0; i < n - 2; ++i)
            {
                EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
                acceleration_edge->setVertex(0,teb_.PoseVertex(i));
                acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
                acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
                acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
                acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
                acceleration_edge->setInformation(information);
                acceleration_edge->setTebConfig(*cfg_);
                optimizer_->addEdge(acceleration_edge);
            }

            // check if a goal velocity should be taken into accound
            if (vel_goal_.first)
            {
                EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
                acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
                acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
                acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
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
                acceleration_edge->setVertex(0,teb_.PoseVertex(0));
                acceleration_edge->setVertex(1,teb_.PoseVertex(1));
                acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
                acceleration_edge->setInitialVelocity(vel_start_.second);
                acceleration_edge->setInformation(information);
                acceleration_edge->setTebConfig(*cfg_);
                optimizer_->addEdge(acceleration_edge);
            }

            // now add the usual acceleration edge for each tuple of three teb poses
            for (int i=0; i < n - 2; ++i)
            {
                EdgeAccelerationHolonomic* acceleration_edge = new EdgeAccelerationHolonomic;
                acceleration_edge->setVertex(0,teb_.PoseVertex(i));
                acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
                acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
                acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
                acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
                acceleration_edge->setInformation(information);
                acceleration_edge->setTebConfig(*cfg_);
                optimizer_->addEdge(acceleration_edge);
            }

            // check if a goal velocity should be taken into accound
            if (vel_goal_.first)
            {
                EdgeAccelerationHolonomicGoal* acceleration_edge = new EdgeAccelerationHolonomicGoal;
                acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
                acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
                acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
                acceleration_edge->setGoalVelocity(vel_goal_.second);
                acceleration_edge->setInformation(information);
                acceleration_edge->setTebConfig(*cfg_);
                optimizer_->addEdge(acceleration_edge);
            }
        }
    }


    void TebOptimalPlanner::AddEdgesKinematicsDiffDrive()
    {
        if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_forward_drive==0)
            return; // if weight equals zero skip adding edges!

        // create edge for satisfiying kinematic constraints
        Eigen::Matrix<double,2,2> information_kinematics;
        information_kinematics.fill(0.0);
        information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
        information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;

        for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
        {
            EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
            kinematics_edge->setVertex(0,teb_.PoseVertex(i));
            kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));
            kinematics_edge->setInformation(information_kinematics);
            kinematics_edge->setTebConfig(*cfg_);
            optimizer_->addEdge(kinematics_edge);
        }
    }


    void TebOptimalPlanner::AddEdgesObstaclesGridMap(double weight_multiplier)
    {
        if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0)// || obstacles_==nullptr )
            return; // if weight equals zero skip adding edges!

        bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;

        Eigen::Matrix<double,1,1> information;
        information.fill(cfg_->optim.weight_obstacle * weight_multiplier);

        auto create_edge = [ &information, this] (int index, int next_index, const DIST_TO_OBSTACLE_FUNC& dist_func) {

            EdgeObstacleGridMap* dist_obst = new EdgeObstacleGridMap;
            dist_obst->setVertex(0,teb_.PoseVertex(index));
            dist_obst->setVertex(1,teb_.PoseVertex(next_index));
            dist_obst->setInformation(information);
            dist_obst->setParameters(*cfg_, dist_func);
            optimizer_->addEdge(dist_obst);

        };

        // iterate all teb points, skipping the last and, if the EdgeVelocityObstacleRatio edges should not be created, the first one too
        const int first_vertex = cfg_->optim.weight_velocity_obstacle_ratio == 0 ? 1 : 0;
        for (int i = 1; i < teb_.sizePoses() - 1; ++i)
        {
            create_edge(i, i + 1 , dist_func_);
        }
    }
