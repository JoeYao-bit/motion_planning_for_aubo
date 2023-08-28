#include "common.h"
#include <limits.h>
using std::placeholders::_1;

using namespace freeNav::RimJump;

bool new_path = false;
bool first_new_path = false;

nav_msgs::msg::Odometry odom_msg;
nav_msgs::msg::OccupancyGrid map_msg;
nav_msgs::msg::Path path_msg;
std::vector<PoseSE2> global_path;

TebOptimalPlanner teb_planner;
double vx = 0, vy = 0, w = 0;

TebConfig config;
RobotFootprintModelPtr robot_model = nullptr;
ViaPointContainer via_points;

std::vector<PoseSE2> result_traj; // poses of trajectory
std::vector<double> time_diffs; // time diff between poses

Point2dContainer robot_shape;

DistanceMapUpdaterPtr<2> distance_map;

MapConverter map_converter;

auto is_occupied = [](const Pointi<2> & pt) -> bool { 
  if(isOutOfBoundary(pt, map_converter.dim_)) { return true; }
  Id id = PointiToId(pt, map_converter.dim_);
  if(id >= map_converter.occupancy_grid.size()) { return true; }
  return map_converter.occupancy_grid[id]; 
};

IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

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
    first_new_path = true;
    global_path.clear();
    via_points.clear();
    for(const auto& way_point : path_msg.poses) {
      PoseSE2 pt;
      pt.x() = way_point.pose.position.x;
      pt.y() = way_point.pose.position.y;
      pt.theta() = tf2::getYaw(way_point.pose.orientation);
      global_path.push_back(pt);
      via_points.push_back(pt.position());
    }

    //auto discrete_global_path = pathDiscretize(global_path, 1.);
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
  if (global_plan.size() < 2)
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
    // while (it != global_plan.end())
    // {
    //   double dx = global_pose.pose.position.x - it->x();
    //   double dy = global_pose.pose.position.y - it->y();
    //   double dist_sq = dx * dx + dy * dy;
    //   if (dist_sq < dist_thresh_sq)
    //   {
    //     erase_end = it;
    //     break;
    //   }
    //   ++it;
    // }
    Eigen::Vector2d glboal_pt;
    Eigen::Vector2d closest_pt;
    glboal_pt.x() = global_pose.pose.position.x, glboal_pt.y() = global_pose.pose.position.y;
    double min_dist = std::numeric_limits<double>::max();
    for(;it != global_plan.end()-1; it++) {
      auto& line1 = it->position(), line2 = (it+1)->position();
      Eigen::Vector2d temp_pt;
      double temp_dist = pointDistToLine(glboal_pt, line1, line2, temp_pt);
      // use = to prefer segment close to target 
      if(temp_dist <= min_dist) {
        min_dist = temp_dist;
        erase_end = it;
        closest_pt = temp_pt;
      }
    }
    // stop when far way to the path
    if(min_dist > dist_behind_robot) { 
      std::cout << " prune failed because current pose " << " too far to path " << std::endl;
      return false; 
      }

    if (erase_end == global_plan.end())
      return false;
    
    //if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end + 1);

    // TODO: update to closest point on line, rather than the global pose 
    PoseSE2 global_se2(glboal_pt, tf2::getYaw(global_pose.pose.orientation));  

    std::reverse(global_plan.begin(), global_plan.end());
    // ignore pendicular point if too close
    // if((global_plan.back().position() - closest_pt).norm() > 0.1) {
    //   global_plan.push_back(PoseSE2(closest_pt, 0));//global_se2
    // }
    //global_plan.push_back(PoseSE2(closest_pt, 0));
    global_plan.push_back(global_se2);
    std::reverse(global_plan.begin(), global_plan.end());
  }
  catch (const tf2::TransformException& ex)
  {
    // error here
    RCLCPP_INFO(rclcpp::get_logger("newNode"), "Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  // cut to the neartes dist 
  global_plan.front().x() = global_pose.pose.position.x;
  global_plan.front().y() = global_pose.pose.position.y;
  global_plan.front().theta() = tf2::getYaw(global_pose.pose.orientation);
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
    // = 10 cause delay in msg receive
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 1, std::bind(&OdometrySubscriber::topic_callback, this, _1));
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

class GridMapSubscriber : public rclcpp::Node
{
public:
  GridMapSubscriber()
  : Node("grid_map_subscriber")
  {
    // = 10 cause delay in msg receive
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 1, std::bind(&GridMapSubscriber::topic_callback, this, _1));
  }


private:

  void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) const
  {
    map_msg = nav_msgs::msg::OccupancyGrid(*msg);
    // update map
    RCLCPP_INFO(this->get_logger(), "receive occupancy grid map msg with size %i %i", map_msg.info.width, map_msg.info.height);
    map_converter.setWorldMap(map_msg);
    distance_map = std::make_shared<DistanceMapUpdater<2> >(is_occupied, map_converter.dim_);
    RCLCPP_INFO(this->get_logger(), "finish set occupancy grid map");
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;

};

DIST_TO_OBSTACLE_FUNC dist_func = [](const PoseSE2& pose, const PoseSE2& pose2) -> double {

    Pointd<2> ptd1({pose.x(), pose.y()}), ptd2({pose2.x(), pose2.y()});
    Pointi<2> pti1 = map_converter.transformToPixel(ptd1), pti2 = map_converter.transformToPixel(ptd2), closest_pt;
    PathLen min_dist = MAX<PathLen>;
    bool pass_occupied = false;
    if(pti1 != pti2) {
        //std::cout << " line pt " << pti1 << " / " << pti2 << std::endl;
        Line<2> line(pti1, pti2);
        for (int i = 0; i < line.step; i++) {
            Pointi<2> temp_pt = line.GetPoint(i);
            pass_occupied = false;
            if(is_occupied(temp_pt)) {
                //std::cout << " current polygon is occupied" << std::endl;
                pass_occupied = true;
            }
            PathLen dist;
            if (distance_map->getClosestDistance(temp_pt, dist, closest_pt)) {
                Pointd<2> ptd = map_converter.transformToWorld(closest_pt);
                // get point to line distance, in metric distance to allow sightly offset in global pose will cause offset in
                //std::cout << " ptd, ptd1, ptd2 " << ptd << ptd1 << ptd2 << std::endl;
                PathLen temp_dist = pointDistToLine(ptd, ptd1, ptd2) * (pass_occupied ? -1 : 1);
                //std::cout << " temp dist " << temp_dist << std::endl;
                //closest_ptds.push_back(ptd);
                // update minimal dist
                if (temp_dist < min_dist) { min_dist = temp_dist; }
            } else {
                std::cout << " out of boundary " << std::endl;
                return 0;
            }
        }
    } else
        {
        // project into one grid
        Pointi<2> temp_pt = pti1;
        pass_occupied = false;
        if(is_occupied(temp_pt)) {
            //std::cout << " current polygon is occupied" << std::endl;
            pass_occupied = true;
        }
        //std::cout << " single pt " << pti1 << std::endl;
        PathLen dist;
        if (distance_map->getClosestDistance(temp_pt, dist, closest_pt)) {
            Pointd<2> ptd = map_converter.transformToWorld(closest_pt);
            // get point to line distance, in metric distance to allow sightly offset in global pose will cause offset in
            //std::cout << " ptd, ptd1, ptd2 " << ptd << ptd1 << ptd2 << std::endl;
            PathLen temp_dist = pointDistToLine(ptd, ptd1, ptd2) * (pass_occupied ? -1 : 1);
            //std::cout << " temp dist " << temp_dist << std::endl;
            //closest_ptds.push_back(ptd);
            // update minimal dist
            if (temp_dist < min_dist) { min_dist = temp_dist; }
        } else {
            std::cout << " out of boundary " << std::endl;
            return 0;
        }
    }
    return min_dist;// * (pass_occupied ? -1 : 1);
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    // subscribe glboal path 
    auto teb_sub = std::make_shared<TEBSubscriber>();

    // subscribe odometry 
    auto odometry_sub = std::make_shared<OdometrySubscriber>();

    auto grid_map_sub = std::make_shared<GridMapSubscriber>();

    // publish cmd vel 
    CmdVelPublisher cmd_vel_pub;

    // set the frequency to run TEB
    rclcpp::WallRate loop_rate(1/control_frequency);

    /* set robot shape */
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
            Eigen::Vector2d yaw_vec;
            if(pruned_path.back().position() == global_path.back().position()) {
              yaw_vec = global_path.back().position() - global_path[global_path.size() - 2].position();
            } else {
              yaw_vec = pruned_path.back().position() - pruned_path[pruned_path.size() - 2].position();
            }
            yaw_vec = yaw_vec / yaw_vec.norm();
            std::cout << " yaw_vec " << yaw_vec.x() << ", " << yaw_vec.y() << std::endl;
            pruned_path.back().theta() = acos(yaw_vec.x()) * (yaw_vec.y() > 0 ? 1 : -1);
            std::cout << " final yaw = " <<pruned_path.back().theta();
            //std::cout << " pathd.size() = " << global_path.size() << " / path.size() = " << path.size() << std::endl;
                      // check whether reach target
            double delta_orient = g2o::normalize_theta(pruned_path.back().theta() - tf2::getYaw(robot_pose.pose.orientation));
            std::cout << " and angle offset " << delta_orient << std::endl;
            double dx = pruned_path.back().x() - robot_pose.pose.position.x;
            double dy = pruned_path.back().y() - robot_pose.pose.position.y;
            RCLCPP_INFO(rclcpp::get_logger("newNode"), "complete global path state %i", config.goal_tolerance.complete_global_plan);
            if(fabs(std::sqrt(dx*dx+dy*dy)) < config.goal_tolerance.xy_goal_tolerance
              //&& fabs(delta_orient) < config.goal_tolerance.yaw_goal_tolerance
              //&& (!config.goal_tolerance.complete_global_plan || via_points.size() == 0)
              //&& (base_local_planner::stopped(base_odom, config.goal_tolerance.theta_stopped_vel, config.goal_tolerance.trans_stopped_vel) || config.goal_tolerance.free_goal_vel)
              )
            {
              RCLCPP_INFO(rclcpp::get_logger("newNode"), "reach target");
              new_path = false;
              cmd_vel_pub.publishCmdVel(0, 0, 0);
            } else {
              // if not, start TEB 
              std::cout << "-- start TEB " << std::endl;
              
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
              pruned_path_discrete.back().theta() = pruned_path.back().theta();
              teb_input_pub.publishTraj(pruned_path_discrete);

              //if(first_new_path) 
              {
                via_points.clear();
                for(const auto& ppd  : pruned_path_discrete) {
                  via_points.push_back(ppd.position());
                }
                teb_planner.initialize(config, dist_func, robot_model, &via_points);
                first_new_path = false;
              }

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
        rclcpp::spin_some(grid_map_sub);
        loop_rate.sleep();
    }
    return 0;
}

