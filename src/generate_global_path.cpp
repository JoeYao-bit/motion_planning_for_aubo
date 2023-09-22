//
// Created by yaozhuo on 2021/9/22.
//

#include "canvas.h"
#include "thread_pool.h"
#include "common.h"
#include "picture_map_loader.h"
#include "distance_map_update.h"

using namespace freeNav;

Pointds<2> pathd;
Pointi<2> click_point;
std::vector<Eigen::Vector3d> smoothed_path;
ThreadPool planning_thread;

double current_time = 0;

// demo path for test
Pathd<2> error_path = {{-1.81, 0.53}, {-0.94, 0.51}, {0.26, -0.38}, {1.33, -0.31}};

agv_path_msgs::msg::AgvPath agv_path;

class GlobalPathPublisher : public rclcpp::Node
{
  public:
    GlobalPathPublisher()
    : Node("GlobalPathPublisher")
    {
      publisher_ = this->create_publisher<agv_path_msgs::msg::AgvPath>("/global_path", 10);
    }

 
    void publishPath(const Pathd<2>& path)
    {
      if(path.size() < 2) {
        RCLCPP_INFO(this->get_logger(), "Global path size %i way points, < 2, do not publish ", path.size());
        return;
      }
      auto message = agv_path_msgs::msg::AgvPath();

      message.header.stamp = this->get_clock()->now();
      message.header.frame_id = "/map";
      message.forward_flag = true;

      message.vel_max = 0.6;
      message.vel_min = 0.0;
      
      message.angular_max = 1.0;
      message.angular_min = 0.0;

      message.acc_theta_max = 1.0;
      message.acc_vel_max = 1.0;

      for(const auto& pt : path) {
        geometry_msgs::msg::Pose pose;
        //pose.header.stamp = this->get_clock()->now();
        //pose.header.frame_id = "/map";
        pose.position.x = pt[0];
        pose.position.y = pt[1];
        message.poses.push_back(pose);
      }
      RCLCPP_INFO(this->get_logger(), "Publis global path with %i way points", path.size());
      std::cout << path << std::endl;
      publisher_->publish(message);

    }

  private:

    rclcpp::Publisher<agv_path_msgs::msg::AgvPath>::SharedPtr publisher_;

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
      map_msg.info.resolution = 1/pixe_to_map_ratio;

      geometry_msgs::msg::Pose origin_pose;
      origin_pose.position.x = -(dim[0]/pixe_to_map_ratio)/2.;
      origin_pose.position.y = -(dim[1]/pixe_to_map_ratio)/2.;
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

    canvas = new Canvas("Set Global Path", dimension[0], dimension[1], pixe_to_map_ratio, 2); 

    auto callback = [](int event, int x, int y, int flags, void *) {
        if(event == CV_EVENT_LBUTTONDOWN) {
            click_point[0] = x;
            click_point[1] = y;
            auto ptd_temp = canvas->transformToWorld(click_point);
            //std::cout << "click_point" << click_point << " ptd_temp " << ptd_temp << std::endl;
            pathd.push_back(ptd_temp);
        } else if (event == CV_EVENT_RBUTTONDOWN) {
            pathd.clear();
            printf("clear all paths");
        }
    };

    canvas->setMouseCallBack(callback);

    DistanceMapUpdaterPtr<2> distance_map = std::make_shared<DistanceMapUpdater<2> >(is_occupied, dimension);

    GlobalPathPublisher global_path_pub;
    GridMapPublisher map_pub;
    map_pub.publishMap(is_occupied, dimension);

    int count = 0;
    bool draw_dist_map = false;
    while(rclcpp::ok()) {
      count ++;
      // repeat publish cause merely one second delay in TEB node, considering only update the map partially
      // pub map after the first 0.5 second
      if(count == .5/control_frequency) {
        map_pub.publishMap(is_occupied, dimension);
      }
      canvas->resetCanvas();
      canvas->drawGridMap(dimension, is_occupied_func);
      canvas->drawAxis(3.5, 3.5);

      if(draw_dist_map) {
            canvas->draw_DistMap(distance_map->dimension_info_,
                                 distance_map->dist_map_,
                                 distance_map->max_dist_,
                                 distance_map->min_dist_);
      }

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
      } else if(key == 'f') {
        draw_dist_map = !draw_dist_map;
      }
    }
    delete canvas;
    rclcpp::shutdown();
    return 0;
}
