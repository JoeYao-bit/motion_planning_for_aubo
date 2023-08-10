//
// Created by yaozhuo on 2021/9/22.
//

#include "path_smooth.h"
#include "gtest/gtest.h"
#include "canvas.h"
#include "thread_pool.h"
#include "optimal_planner.h"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "rclcpp/time.hpp"


using namespace freeNav::RimJump;
using namespace freeNav;
Path<int, 2> path;
Pointds<2> pathd;
Pointi<2> click_point;
std::vector<Eigen::Vector3d> smoothed_path;
ThreadPool planning_thread;

Canvas canvas("Path Smooth", 1000, 700, 200);

std::vector<PoseSE2> waypoints;

std::vector<PoseSE2> result_traj;
std::vector<double> time_diffs;

ViaPointContainer via_points;
TebConfig config;
RobotFootprintModelPtr robot_model = nullptr;
TebOptimalPlanner teb_planner;

double current_time = 0;

class GlobalPathPublisher : public rclcpp::Node
{
  public:
    GlobalPathPublisher()
    : Node("GlobalPathPublisher")
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);
    }

 
    void publishPath(const Pathd<2>& path)
    {
      auto message = nav_msgs::msg::Path();

      message.header.stamp = this->get_clock()->now();
      message.header.frame_id = "map";

      for(const auto& pt : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = pt[0];
        pose.pose.position.x = pt[1];
        message.poses.push_back(pose);
      }

      RCLCPP_INFO(this->get_logger(), "Publis global path with %i way points", path.size());
      std::cout << path << std::endl;
      publisher_->publish(message);

    }

  private:

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;

};

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);

    /* set robot shape */
    Point2dContainer robot_shape;
    robot_shape = {{.1, .05}, {.1, -.05}, {-.05, -.05}, {-.05, .05}};
    robot_model = boost::make_shared<PolygonRobotFootprint>(robot_shape);

    auto callback = [](int event, int x, int y, int flags, void *){
        if(event == CV_EVENT_LBUTTONDOWN) {
            click_point[0] = x;
            click_point[1] = y;
            path.push_back(click_point);
            pathd.push_back(canvas.transformToWorld(path.back()));
        } else if (event == CV_EVENT_RBUTTONDOWN) {
            path.clear();
            pathd.clear();
            result_traj.clear();
            printf("clear all paths");
        }
    };

    canvas.setMouseCallBack(callback);

    GlobalPathPublisher global_path_pub;

    while(rclcpp::ok()) {
        canvas.resetCanvas();
        canvas.drawAxis(8., 6.);

        if(!pathd.empty()) {
            canvas.drawPathf(pathd, 2);
            canvas.drawPointfs(pathd, .03, 2);
        }

        if(!result_traj.empty()) {
            canvas.drawPointfs(result_traj, time_diffs, 0, robot_shape, 2);
        }

        char key = canvas.show(33);
        // 32 = space
        if(key == 32) {
            if(planning_thread.pool_[0].joinable()) {
                planning_thread.Schedule([&] {

                    // publish global path to ros2
                    global_path_pub.publishPath(pathd);
                    // smooth path
                    // std::cout << "-- start" << std::endl;
                    // std::vector<PoseSE2> path = pathDiscretize(pathd, .2);
                    // std::cout << " pathd.size() = " << pathd.size() << " / path.size() = " << path.size() << std::endl;
                    // if(!teb_planner.isInitialize()) {
                    //     teb_planner.initialize(config, robot_model, &via_points);
                    // }
                    // teb_planner.setVelocityStart(Eigen::Vector3d(0, 0, 0));
                    // teb_planner.setVelocityGoal(Eigen::Vector3d(0, 0, 0));
                    // std::cout << "-- teb initialized " << std::endl;
                    // if(teb_planner.plan(path, Eigen::Vector3d(), false)) {
                    //     std::cout << "-- teb success" << std::endl;
                    //     teb_planner.getFullTrajectory(result_traj, time_diffs);
                    // } else {
                    //     std::cout << "-- teb failed" << std::endl;
                    // }
                    // teb_planner.clearPlanner();
                    // std::cout << "-- all planning finished" << std::endl;

                });
            }
        }
    }
    return 0;
}
