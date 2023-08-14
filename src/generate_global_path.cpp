//
// Created by yaozhuo on 2021/9/22.
//

#include "canvas.h"
#include "thread_pool.h"
#include "common.h"

using namespace freeNav;

Pointds<2> pathd;
Pointi<2> click_point;
std::vector<Eigen::Vector3d> smoothed_path;
ThreadPool planning_thread;

Canvas canvas("Set Global Path", 1000, 700, 200);
double current_time = 0;

Pathd<2> error_path = {{-0.995, -0.02}, {-0.515, -0.24}, {0.155, -0.2}, 
                      {0.825, 0.03}, {1.7, 0.375}, {2.235, -0.46}, 
                      {1.905, -1.165}, {1.08, -1.48}, {0.365, -1.485}};
class GlobalPathPublisher : public rclcpp::Node
{
  public:
    GlobalPathPublisher()
    : Node("GlobalPathPublisher")
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);
    }

 
    void publishPath(const Pathd<2>& path)
    {
      if(path.size() < 2) {
        RCLCPP_INFO(this->get_logger(), "Global path size %i way points, < 2, do not publish ", path.size());
        return;
      }
      auto message = nav_msgs::msg::Path();

      message.header.stamp = this->get_clock()->now();
      message.header.frame_id = "/map";

      for(const auto& pt : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "/map";
        pose.pose.position.x = pt[0];
        pose.pose.position.y = pt[1];
        message.poses.push_back(pose);
      }
      RCLCPP_INFO(this->get_logger(), "Publis global path with %i way points", path.size());
      std::cout << path << std::endl;
      publisher_->publish(message);

    }

  private:

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;

};
// 指定编译以及线程数量
// colcon build --packages-select teb_planner --parallel-workers 4

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);
    auto callback = [](int event, int x, int y, int flags, void *){
        if(event == CV_EVENT_LBUTTONDOWN) {
            click_point[0] = x;
            click_point[1] = y;
            pathd.push_back(canvas.transformToWorld(click_point));
        } else if (event == CV_EVENT_RBUTTONDOWN) {
            pathd.clear();
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
        char key = canvas.show(33);
        // 32 = space
        if(key == 32) {
            if(planning_thread.pool_[0].joinable()) {
                planning_thread.Schedule([&] {
                    // publish global path to ros2
                    global_path_pub.publishPath(error_path); // pathd
                });
            }
        }
    }
    rclcpp::shutdown();
    return 0;
}
