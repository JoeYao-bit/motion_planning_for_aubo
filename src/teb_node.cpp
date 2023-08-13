#include "common.h"

using std::placeholders::_1;

using namespace freeNav::RimJump;

bool new_path = false;

nav_msgs::msg::Odometry odom_msg;
nav_msgs::msg::Path path_msg;
std::vector<PoseSE2> global_path;

class TEBSubscriber : public rclcpp::Node
{
public:
  TEBSubscriber() : Node("teb_subscriber")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Path>("/global_path", 10, std::bind(&TEBSubscriber::topic_callback, this, _1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  }

private:

  void topic_callback(const nav_msgs::msg::Path::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Receive global path with %u way points", msg->poses.size());
    path_msg = *msg;
    new_path = true;

    global_path.clear();
    for(const auto& way_point : path_msg.poses) {
      PoseSE2 pt;
      pt.x() = way_point.pose.position.x;
      pt.y() = way_point.pose.position.y;
      pt.theta() = tf2::getYaw(way_point.pose.orientation);
      global_path.push_back(pt);
    }

    auto discrete_global_path = pathDiscretize(global_path, 1.);
    // set final orientation
    //auto offset = global_path.back() - global_path[global_path.size()-2];
    //double yaw = getAngle(offset) * (offset[1] > 0 ? 1 : -1);
    //discrete_global_path.back().theta() = yaw;
  }


  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
};


bool pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& global_pose, std::vector<PoseSE2>& global_plan, 
                     double dist_behind_robot, double max_global_path_ahead_dist)
{
  if (global_plan.empty())
    return true;
  
  try
  {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    //std::cout << "first frame id = " << global_plan.front().header.frame_id << " / second frame id = " << global_pose.header.frame_id << std::endl;
    // in tf2 frame_ids cannot start with a '/'
    //geometry_msgs::msg::TransformStamped global_to_plan_transform;// = tf_buffer_->lookupTransform("odom", "map", this->now(), rclcpp::Duration(2));
    //geometry_msgs::msg::PoseStamped robot;
    //tf2::doTransform(global_pose, robot, global_to_plan_transform);
    
    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
    
    // iterate plan until a pose close the robot is found
    std::vector<PoseSE2>::iterator it = global_plan.begin();
    std::vector<PoseSE2>::iterator erase_end = it;
    while (it != global_plan.end())
    {
      double dx = global_pose.pose.position.x - it->x();
      double dy = global_pose.pose.position.y - it->y();
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
        erase_end = it;
        break;
      }
      ++it;
    }
    if (erase_end == global_plan.end())
      return false;
    
    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);

  }
  catch (const tf2::TransformException& ex)
  {
    // error here
    RCLCPP_INFO(rclcpp::get_logger("newNode"), "Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  // cut to the neartes dist 
  double current_dist = 0;
  std::vector<PoseSE2> retv = {global_plan.front()};
  for(int i=0; i<global_plan.size()-1; i++) {
    // get current line
    Eigen::Vector2d  line = (global_plan[i+1]-global_plan[i]).position();
    double temp = current_dist + line.norm();
    if(temp < max_global_path_ahead_dist) {
      current_dist = temp;
      retv.push_back(global_plan[i+1]);
    } else if (temp > max_global_path_ahead_dist) {
      double forward_dist = max_global_path_ahead_dist - current_dist;
      // move forward
      line = line/line.norm();
      Eigen::Vector2d forward_pt = global_plan[i].position() + line * forward_dist;
      retv.push_back(PoseSE2(forward_pt, 0));
      break;
    } else {
      retv.push_back(global_plan[i+1]);
      break;
    }
  }
  global_plan = retv;
  return true;
}

/*
bool TebLocalPlannerROS::transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                  const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
                  std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx, geometry_msgs::TransformStamped* tf_plan_to_global) const
{
  // this method is a slightly modified version of base_local_planner/goal_functions.h

  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  try 
  {
    if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp,
                                                                                  plan_pose.header.frame_id, ros::Duration(cfg_.robot.transform_tolerance));

    //let's get the pose of the robot in the frame of the plan
    geometry_msgs::PoseStamped robot_pose;
    tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

    //we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                     costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                           // located on the border of the local costmap
    

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;
    
    //we need to loop to a point on the plan that is within a certain distance of the robot
    bool robot_reached = false;
    for(int j=0; j < (int)global_plan.size(); ++j)
    {
      double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;

      if (robot_reached && new_sq_dist > sq_dist)
        break;

      if (new_sq_dist < sq_dist) // find closest distance
      {
        sq_dist = new_sq_dist;
        i = j;
        if (sq_dist < 0.05)      // 2.5 cm to the robot; take the immediate local minima; if it's not the global
          robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this
      }
    }
    
    geometry_msgs::PoseStamped newer_pose;
    
    double plan_length = 0; // check cumulative Euclidean distance along the plan
    
    //now we'll transform until points are outside of our distance threshold
    while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length<=0 || plan_length <= max_plan_length))
    {
      const geometry_msgs::PoseStamped& pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);

      double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      
      // caclulate distance to previous pose
      if (i>0 && max_plan_length>0)
        plan_length += distance_points2d(global_plan[i-1].pose.position, global_plan[i].pose.position);

      ++i;
    }
        
    // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
    // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan.empty())
    {
      tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);
      
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
    }
    else
    {
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
    }
    
    // Return the transformation from the global plan to the global planning frame if desired
    if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
  }
  catch(tf::LookupException& ex)
  {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ConnectivityException& ex) 
  {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ExtrapolationException& ex) 
  {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

    return false;
  }

  return true;
}
*/

class CmdVelPublisher : public rclcpp::Node
{
  public:
    CmdVelPublisher()
    : Node("CmdVelPublisher")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

 
    void publishCmdVel(double vx, double vy, double w)
    {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = vx;
      message.linear.y = vy;
      message.angular.z = w;
      publisher_->publish(message);
    }

  private:

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

};

class TEBPathPublisher : public rclcpp::Node
{
  public:
    TEBPathPublisher()
    : Node("TebPathPublisher")
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Path>("/teb_path", 10);
    }
 
    void publishTraj(const std::vector<PoseSE2>& traj)
    {
      if(traj.size() < 2) {
        RCLCPP_INFO(this->get_logger(), "TEB traj size %i way points, < 2, do not publish ", traj.size());
        return;
      }
      auto message = nav_msgs::msg::Path();

      message.header.stamp = this->get_clock()->now();
      message.header.frame_id = "/map";
      std::cout << " teb traj: ";
      for(const auto& pt : traj) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "/map";
        pose.pose.position.x = pt.x();
        pose.pose.position.y = pt.y();
        tf2::Quaternion quat; quat.setRPY(0, 0, pt.theta());
        pose.pose.orientation = tf2::toMsg(quat);
        message.poses.push_back(pose);
        std::cout << "(" << pt.x() << ", " << pt.y() << ", " << pt.theta() << ")";
      }
      std::cout << std::endl;
      RCLCPP_INFO(this->get_logger(), "Publis TEB traj with %i way points", traj.size());
      publisher_->publish(message);
    }

  private:

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;

};


class TEBInputPathPublisher : public rclcpp::Node
{
  public:
    TEBInputPathPublisher()
    : Node("TebInputPathPublisher")
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Path>("/teb_input_path", 10);
    }
 
    void publishTraj(const std::vector<PoseSE2>& traj)
    {
      if(traj.size() < 2) {
        RCLCPP_INFO(this->get_logger(), "TEB traj size %i way points, < 2, do not publish ", traj.size());
        return;
      }
      auto message = nav_msgs::msg::Path();

      message.header.stamp = this->get_clock()->now();
      message.header.frame_id = "/map";
      std::cout << " teb traj: ";
      for(const auto& pt : traj) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "/map";
        pose.pose.position.x = pt.x();
        pose.pose.position.y = pt.y();
        tf2::Quaternion quat; quat.setRPY(0, 0, pt.theta());
        pose.pose.orientation = tf2::toMsg(quat);
        message.poses.push_back(pose);
        std::cout << "(" << pt.x() << ", " << pt.y() << ", " << pt.theta() << ")";
      }
      std::cout << std::endl;
      RCLCPP_INFO(this->get_logger(), "Publis TEB traj with %i way points", traj.size());
      publisher_->publish(message);
    }

  private:

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;

};


class OdometrySubscriber : public rclcpp::Node
{
public:
  OdometrySubscriber()
  : Node("odometry_subscriber")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&OdometrySubscriber::topic_callback, this, _1));
  }


private:

  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
    double yaw = tf2::getYaw(msg->pose.pose.orientation);
    RCLCPP_INFO(this->get_logger(), "TEB Receive Odom %f %f %f", msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    odom_msg = nav_msgs::msg::Odometry(*msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

};

TebOptimalPlanner teb_planner;
double vx = 0, vy = 0, w = 0;

TebConfig config;
RobotFootprintModelPtr robot_model = nullptr;
ViaPointContainer via_points;

std::vector<PoseSE2> result_traj; // poses of trajectory
std::vector<double> time_diffs; // time diff between poses



int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    // subscribe glboal path 
    auto teb_sub = std::make_shared<TEBSubscriber>();

    // subscribe odometry 
    auto odometry_sub = std::make_shared<OdometrySubscriber>();

    // publish cmd vel 
    CmdVelPublisher cmd_vel_pub;

    // set the frequency to run TEB
    rclcpp::WallRate loop_rate(1/control_frequency);

    /* set robot shape */
    Point2dContainer robot_shape;
    robot_shape = {{.1, .05}, {.1, -.05}, {-.05, -.05}, {-.05, .05}};
    robot_model = boost::make_shared<PolygonRobotFootprint>(robot_shape);

    TEBPathPublisher teb_traj_pub;

    TEBInputPathPublisher teb_input_pub;

    while (rclcpp::ok())
    {
        
        RCLCPP_INFO(rclcpp::get_logger("newNode"), "-------timer callback!-----------");
        vx = 0, vy = 0, w = 0;
        if(new_path) {

          geometry_msgs::msg::PoseStamped robot_pose;
          robot_pose.header = odom_msg.header;
          robot_pose.pose = odom_msg.pose.pose;
          // std::vector<geometry_msgs::msg::PoseStamped> global_plan = path_msg.poses;
          std::cout << " robot pose: " << robot_pose.pose.position.x << ", " << robot_pose.pose.position.y << ", " << tf2::getYaw(robot_pose.pose.orientation) << std::endl;
          // before prune
          std::cout << " before prune: ";
          for(const auto& dgp : global_path) {
            std::cout << "(" << dgp.x() << ", " << dgp.y() << ")->";
          }
          std::cout << " final yaw = " << tf2::getYaw(path_msg.poses.back().pose.orientation) << std::endl;
          auto pruned_path = global_path;
          if(pruneGlobalPlan(robot_pose, pruned_path, config.trajectory.global_plan_prune_distance, config.trajectory.max_global_plan_lookahead_dist)) {
            // after prune
            std::cout << " after prune: ";
            for(const auto& gp : pruned_path) {
              std::cout << "(" << gp.x() << ", " << gp.y() << ")->";
            }

            Eigen::Vector2d yaw_vec = (pruned_path.back() - pruned_path[pruned_path.size() - 2]).position();
            yaw_vec = yaw_vec / yaw_vec.norm();
            std::cout << " yaw_vec " << yaw_vec.x() << ", " << yaw_vec.y() << std::endl;
            pruned_path.back().theta() = acos(yaw_vec.x()) * (yaw_vec.y() > 0 ? 1 : -1);
            std::cout << " final yaw = " <<pruned_path.back().theta() << std::endl;
            //std::cout << " pathd.size() = " << global_path.size() << " / path.size() = " << path.size() << std::endl;
                      // check whether reach target
            double delta_orient = g2o::normalize_theta(pruned_path.back().theta() - tf2::getYaw(robot_pose.pose.orientation));
            double dx = pruned_path.back().x() - robot_pose.pose.position.x;
            double dy = pruned_path.back().y() - robot_pose.pose.position.y;
            if(fabs(std::sqrt(dx*dx+dy*dy)) < config.goal_tolerance.xy_goal_tolerance
              && fabs(delta_orient) < config.goal_tolerance.yaw_goal_tolerance
              //&& (!config.goal_tolerance.complete_global_plan || via_points_.size() == 0)
              //&& (base_local_planner::stopped(base_odom, config.goal_tolerance.theta_stopped_vel, config.goal_tolerance.trans_stopped_vel) || config.goal_tolerance.free_goal_vel)
              )
            {
              RCLCPP_INFO(rclcpp::get_logger("newNode"), "reach target");
              new_path = false;
              cmd_vel_pub.publishCmdVel(0, 0, 0);
            } else {
              // if not, start TEB 
              std::cout << "-- start TEB " << std::endl;
              
              if(!teb_planner.isInitialize()) {
                teb_planner.initialize(config, robot_model, &via_points);
              }
              double c_vx = odom_msg.twist.twist.linear.x, c_vy = odom_msg.twist.twist.linear.y, c_w = odom_msg.twist.twist.angular.z;
              teb_planner.setVelocityStart(Eigen::Vector3d(c_vx, c_vy, c_w));
              teb_planner.setVelocityGoal(Eigen::Vector3d(0, 0, 0));
              if(pruned_path.size() >= 2) {
                pruned_path.front().x() = robot_pose.pose.position.x;
                pruned_path.front().y() = robot_pose.pose.position.y;
                pruned_path.front().theta() = tf2::getYaw(robot_pose.pose.orientation);
              } else {
                PoseSE2 temp;
                temp.x() = robot_pose.pose.position.x;
                temp.y() = robot_pose.pose.position.y;
                temp.theta() = tf2::getYaw(robot_pose.pose.orientation);
                std::reverse(pruned_path.begin(), pruned_path.end());
                pruned_path.push_back(temp);
                std::reverse(pruned_path.begin(), pruned_path.end());
              }
              auto pruned_path_discrete = pathDiscretize(pruned_path,.3);
              pruned_path_discrete.front().theta() = tf2::getYaw(robot_pose.pose.orientation);
              teb_input_pub.publishTraj(pruned_path_discrete);
              //std::cout << "-- teb initialized " << std::endl;
              if(teb_planner.plan(pruned_path_discrete, Eigen::Vector3d(c_vx, c_vy, c_w), false)) {
                  //std::cout << "-- teb success" << std::endl;
                  teb_planner.getFullTrajectory(result_traj, time_diffs);
                  teb_planner.getVelocityCommand(vx, vy, w, 1);
                  teb_traj_pub.publishTraj(result_traj);
                  //std::cout << "velocity command = " << vx << ", " << vy << ", " << w << std::endl;
                  RCLCPP_INFO(rclcpp::get_logger("newNode"), "-- velocity command (vx, vy, w) = %f %f %f", vx, vy, w);
                  teb_planner.clearPlanner();
                  cmd_vel_pub.publishCmdVel(vx, vy, w);
                  // TODO：visualize teb's path during planning， add look ahead dist
              } else {
                  RCLCPP_INFO(rclcpp::get_logger("newNode"), "-- teb failed");
                  teb_planner.clearPlanner();
                  cmd_vel_pub.publishCmdVel(0, 0, 0);
              }
            }
          } else {
            // after prune failed
            std::cout << " after prune failed: ";
            for(const auto& gp : pruned_path) {
              std::cout << "(" << gp.x() << ", " << gp.y() << ")->";
            }
            std::cout << std::endl;
            new_path = false;
            cmd_vel_pub.publishCmdVel(0, 0, 0);
          }
        }
        //cmd_vel_pub.publishCmdVel(0.5, 0, 0);
        rclcpp::spin_some(teb_sub);
        rclcpp::spin_some(odometry_sub);
        loop_rate.sleep();
    }


    return 0;
}

