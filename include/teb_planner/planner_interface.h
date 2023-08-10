//
// Created by yaozhuo on 2021/11/28.
//

#ifndef FREENAV_PLANNER_INTERFACE_H
#define FREENAV_PLANNER_INTERFACE_H

// boost
#include <boost/shared_ptr.hpp>

// this package
#include "pose_se2.h"
#include "robot_footprint_model.h"

/**
 * @class PlannerInterface
 * @brief This abstract class defines an interface for local planners
 */
class PlannerInterface
{
public:

    /**
     * @brief Default constructor
     */
    PlannerInterface()
    {
    }
    /**
     * @brief Virtual destructor.
     */
    virtual ~PlannerInterface()
    {
    }


    /** @name Plan a trajectory */
    //@{

    /**
     * @brief Plan a trajectory based on an initial reference plan.
     *
     * Provide this method to create and optimize a trajectory that is initialized
     * according to an initial reference plan (given as a container of poses).
     * @param initial_plan vector of geometry_msgs::PoseStamped
     * @param start_vel Current start velocity (e.g. the velocity of the robot, only linear.x and angular.z are used)
     * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed,
     *        otherwise the final velocity will be zero (default: false)
     * @return \c true if planning was successful, \c false otherwise
     */
    virtual bool plan(const std::vector<PoseSE2>& initial_plan, Eigen::Vector3d start_vel = {0, 0, 0}, bool free_goal_vel=false) = 0;

    /**
     * @brief Get the velocity command from a previously optimized plan to control the robot at the current sampling interval.
     * @warning Call plan() first and check if the generated plan is feasible.
     * @param[out] vx translational velocity [m/s]
     * @param[out] vy strafing velocity which can be nonzero for holonomic robots [m/s]
     * @param[out] omega rotational velocity [rad/s]
     * @param[in] look_ahead_poses index of the final pose used to compute the velocity command.
     * @return \c true if command is valid, \c false otherwise
     */
    virtual bool getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses) const = 0;

    //@}


    /**
     * @brief Reset the planner.
     */
    virtual void clearPlanner() = 0;

    /**
     * @brief Prefer a desired initial turning direction (by penalizing the opposing one)
     *
     * A desired (initial) turning direction might be specified in case the planned trajectory oscillates between two
     * solutions (in the same equivalence class!) with similar cost. Check the parameters in order to adjust the weight of the penalty.
     * Initial means that the penalty is applied only to the first few poses of the trajectory.
     * @param dir This parameter might be RotType::left (prefer left), RotType::right (prefer right) or RotType::none (prefer none)
     */
    virtual void setPreferredTurningDir(RotType dir) {printf("setPreferredTurningDir() not implemented for this planner.");}

    /**
     * @brief Visualize planner specific stuff.
     * Overwrite this method to provide an interface to perform all planner related visualizations at once.
     */
    virtual void visualize()
    {
    }

    virtual void updateRobotModel(RobotFootprintModelPtr robot_model)
    {
    }

    /**
     * Compute and return the cost of the current optimization graph (supports multiple trajectories)
     * @param[out] cost current cost value for each trajectory
     *                  [for a planner with just a single trajectory: size=1, vector will not be cleared]
     * @param obst_cost_scale Specify extra scaling for obstacle costs
     * @param alternative_time_cost Replace the cost for the time optimal objective by the actual (weighted) transition time
     */
    virtual void computeCurrentCost(std::vector<double>& cost, double obst_cost_scale=1.0, bool alternative_time_cost=false)
    {
    }

    /**
     * @brief Returns true if the planner has diverged.
     */
    virtual bool hasDiverged() const = 0;

};

//! Abbrev. for shared instances of PlannerInterface or it's subclasses
typedef boost::shared_ptr<PlannerInterface> PlannerInterfacePtr;



#endif //FREENAV_PLANNER_INTERFACE_H
