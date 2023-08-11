#include "common.h"

using std::placeholders::_1;

using namespace freeNav::RimJump;

bool new_path = false;

freeNav::Path<double, 2> global_path = {};

nav_msgs::msg::Odometry odom_msg;


std::vector<PoseSE2> discrete_path;

class GlobalPathSubscriber : public rclcpp::Node
{
public:
  GlobalPathSubscriber()
  : Node("global_path_subscriber")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Path>("/global_path", 10, std::bind(&GlobalPathSubscriber::topic_callback, this, _1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  }

private:

  void topic_callback(const nav_msgs::msg::Path::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Receive global path with %u way points", msg->poses.size());
    global_path.clear();
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header = odom_msg.header;
    robot_pose.pose = odom_msg.pose.pose;
    std::vector<geometry_msgs::msg::PoseStamped> global_plan = msg->poses;
    if(pruneGlobalPlan(robot_pose, global_plan, 1.)) {
      for(const auto& way_point : msg->poses) {
        freeNav::Pointd<2> pt;
        pt[0] = way_point.pose.position.x;
        pt[1] = way_point.pose.position.y;
        global_path.push_back(pt);
      }
      new_path = true;
      discrete_path = pathDiscretize(global_path, .2);
      //std::cout << " pathd.size() = " << global_path.size() << " / path.size() = " << path.size() << std::endl;
    }
  }

  bool pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& global_pose, std::vector<geometry_msgs::msg::PoseStamped>& global_plan, double dist_behind_robot) const
  {
    if (global_plan.empty())
      return true;
    
    try
    {
      // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
      geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_buffer_->lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, this->now());
      geometry_msgs::msg::PoseStamped robot;
      tf2::doTransform(global_pose, robot, global_to_plan_transform);
      
      double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
      
      // iterate plan until a pose close the robot is found
      std::vector<geometry_msgs::msg::PoseStamped>::iterator it = global_plan.begin();
      std::vector<geometry_msgs::msg::PoseStamped>::iterator erase_end = it;
      while (it != global_plan.end())
      {
        double dx = robot.pose.position.x - it->pose.position.x;
        double dy = robot.pose.position.y - it->pose.position.y;
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
      RCLCPP_INFO(rclcpp::get_logger("newNode"), "Cannot prune path since no transform is available: %s\n", ex.what());
      return false;
    }
    return true;
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
};

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
    auto global_path_sub = std::make_shared<GlobalPathSubscriber>();

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

    while (rclcpp::ok())
    {
        
        RCLCPP_INFO(rclcpp::get_logger("newNode"), "-------timer callback!-----------");
        vx = 0, vy = 0, w = 0;
        // if(new_path) {
        //   // check whether reach target
        //   geometry_msgs::msg::Point robot_pose = odom_msg.pose.pose.position;
        //   double dx = global_path.back()[0] - robot_pose.x;
        //   double dy = global_path.back()[1] - robot_pose.y;
        //   //double delta_orient = g2o::normalize_theta( tf2::getYaw(global_goal.pose.orientation) - robot_pose_.theta() );
        //   if(fabs(std::sqrt(dx*dx+dy*dy)) < config.goal_tolerance.xy_goal_tolerance
        //     //&& fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance
        //     //&& (!config.goal_tolerance.complete_global_plan || via_points_.size() == 0)
        //     //&& (base_local_planner::stopped(base_odom, config.goal_tolerance.theta_stopped_vel, config.goal_tolerance.trans_stopped_vel) || config.goal_tolerance.free_goal_vel)
        //     )
        //   {
        //     new_path = false;
        //   } else {
        //     // if not, start TEB 
        //     std::cout << "-- start TEB " << std::endl;
        //     if(!teb_planner.isInitialize()) {
        //         teb_planner.initialize(config, robot_model, &via_points);
        //     }
        //     teb_planner.setVelocityStart(Eigen::Vector3d(0, 0, 0));
        //     teb_planner.setVelocityGoal(Eigen::Vector3d(0, 0, 0));
        //     //std::cout << "-- teb initialized " << std::endl;
        //     if(teb_planner.plan(discrete_path, Eigen::Vector3d(), false)) {
        //         //std::cout << "-- teb success" << std::endl;
        //         teb_planner.getFullTrajectory(result_traj, time_diffs);
        //         teb_planner.getVelocityCommand(vx, vy, w, 4);
        //         //std::cout << "velocity command = " << vx << ", " << vy << ", " << w << std::endl;
        //         RCLCPP_INFO(rclcpp::get_logger("newNode"), "-- velocity command (vx, vy, w) = %f %f %f", vx, vy, w);
        //     } else {
        //         RCLCPP_INFO(rclcpp::get_logger("newNode"), "-- teb failed");
        //     }
        //     teb_planner.clearPlanner();
        //   }
        // }
        cmd_vel_pub.publishCmdVel(vx, vy, w);
        rclcpp::spin_some(global_path_sub);
        rclcpp::spin_some(odometry_sub);
        loop_rate.sleep();
    }


    return 0;
}

