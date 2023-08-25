//
// Created by yaozhuo on 2021/9/22.
//

#include "canvas.h"
#include "thread_pool.h"
#include "common.h"
#include "picture_map_loader.h"

using namespace freeNav;

Pointds<2> pathd;
Pointi<2> click_point;
std::vector<Eigen::Vector3d> smoothed_path;
ThreadPool planning_thread;

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
class GridMapPublisher : public rclcpp::Node
{
  public:
    GridMapPublisher()
    : Node("GridMapPublisher")
    {
      publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    }
 
    void publishMap(const IS_OCCUPIED_FUNC<2>& is_occupied, DimensionLength* dim)
    {
      nav_msgs::msg::OccupancyGrid map_msg;
      map_msg.header.frame_id = "/map";
      map_msg.header.stamp = this->get_clock()->now();

      map_msg.info.map_load_time = this->get_clock()->now();
      map_msg.info.width = dim[0];
      map_msg.info.height = dim[1];
      map_msg.info.resolution = 0.05;

      geometry_msgs::msg::Pose origin_pose;
      origin_pose.position.x = -(.05*dim[0])/2.;
      origin_pose.position.y = -(.05*dim[1])/2.;
      tf2::Quaternion quat; quat.setRPY(0, 0, 0);
      origin_pose.orientation = tf2::toMsg(quat);
      map_msg.info.origin = origin_pose;
      
      int total_index = dim[0] * dim[1];
      map_msg.data.resize(total_index, 0);
      for(int x=0; x<dim[0]; x++) {
        for(int y=0; y<dim[1]; y++) {
          Pointi<2> pt({x, y});
          map_msg.data[x + (dim[1] - 1 - y) * dim[0]] = is_occupied(pt) ? 100 : 0; 
        }
      }
      publisher_->publish(map_msg);
    }

  private:

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;

};

// 指定编译以及线程数量
// colcon build --packages-select teb_planner --parallel-workers 4

Canvas* canvas;

auto is_grid_occupied = [](const cv::Vec3b& color) -> bool {
    if (color[0] <= 200 || color[1] <= 200 || color[2] <= 200) return true;
    return false;
};

// load grid map and publish 
std::string map_path = "/home/yaozhuo/code/teb_local_planner/map/indoor_map.png";

PictureLoader loader(map_path, is_grid_occupied);
auto dimension = loader.getDimensionInfo();

auto is_occupied = [](const Pointi<2> & pt) -> bool { return loader.isOccupied(pt); };

auto set_occupied = [](const Pointi<2> & pt) { loader.setOccupied(pt); };

IS_OCCUPIED_FUNC<2> is_occupied_func = is_occupied;

SET_OCCUPIED_FUNC<2> set_occupied_func = set_occupied;

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);

    canvas = new Canvas("Set Global Path", dimension[0], dimension[1], 20, 1); 

    auto callback = [](int event, int x, int y, int flags, void *) {
        if(event == CV_EVENT_LBUTTONDOWN) {
            click_point[0] = x;
            click_point[1] = y;
            pathd.push_back(canvas->transformToWorld(click_point));
        } else if (event == CV_EVENT_RBUTTONDOWN) {
            pathd.clear();
            printf("clear all paths");
        }
    };

    canvas->setMouseCallBack(callback);

    GlobalPathPublisher global_path_pub;
    GridMapPublisher map_pub;
    map_pub.publishMap(is_occupied, dimension);
    int count = 0;
    while(rclcpp::ok()) {
      count ++;
      count = count % 50; // pub map per second
      if(count == 0) {
        map_pub.publishMap(is_occupied, dimension);
      }
      canvas->resetCanvas();
      canvas->drawGridMap(dimension, is_occupied_func);
      canvas->drawAxis(3.5, 3.5);

      if(!pathd.empty()) {
          canvas->drawPathf(pathd, 2);
          canvas->drawPointfs(pathd, .03, 2);
      }
      char key = canvas->show(100);
      // 32 = space
      if(key == 32) {
          if(planning_thread.pool_[0].joinable()) {
              planning_thread.Schedule([&] {
                  // publish global path to ros2
                  //global_path_pub.publishPath(error_path);
                  global_path_pub.publishPath(pathd);
              });
          }
      }
    }
    delete canvas;
    rclcpp::shutdown();
    return 0;
}
