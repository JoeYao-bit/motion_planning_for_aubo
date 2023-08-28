#ifndef COMMON_H
#define COMMOM_H

#include <memory> // shared_ptr

#include "point.h"
#include "thread_pool.h"
#include "path_smooth.h"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"


#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

const double control_frequency = .1;
using namespace freeNav;
// converter of map
class MapConverter {
public:
   MapConverter() {
      dim_ = new DimensionLength[3];
   }

   void setWorldMap(const nav_msgs::msg::OccupancyGrid& map_msg) {
      map_msg_ = map_msg;
      dim_[0] = map_msg.info.width;
      dim_[1] = map_msg.info.height;
      occupancy_grid.resize(dim_[0]*dim_[1], true);
      for(int x=0; x<dim_[0]; x++) {
          for(int y=0; y<dim_[1]; y++) {
            // 100 is threshold of whether grid is occupied, update if origin map updated
            if(map_msg.data[x + y * dim_[0]] > 100) {
              occupancy_grid[x + y * dim_[0]] = false;
            } 
          }
      }
    //   std::cout << " pixel (0, 0) to world " << pixelToWorld(Pointi<2>({0, 0})) << std::endl;
    //   std::cout << " world (0, 0) to pixel " << worldToPixel(Pointd<2>({0, 0})) << std::endl;
   }

   // left down corner is map origin, x, y, theta is origin's pose 
  Pointd<2> pixelToWorld(const Pointi<2>& pt) {
    Pointd<2> retv;
    const geometry_msgs::msg::Point& origin = map_msg_.info.origin.position;
    const double& theta = tf2::getYaw(map_msg_.info.origin.orientation);
    const double& resolution = map_msg_.info.resolution;
    retv[0] = origin.x + (pt[0]*cos(theta) - pt[1]*sin(theta))*resolution;
    retv[1] = origin.y + (pt[0]*sin(theta) + pt[1]*cos(theta))*resolution;
    return retv;
  }

  Pointi<2> worldToPixel(const Pointd<2>& ptd) {
    Pointi<2> retv;
    const geometry_msgs::msg::Point& origin = map_msg_.info.origin.position;
    const double& theta = tf2::getYaw(map_msg_.info.origin.orientation);
    const double& resolution = map_msg_.info.resolution;
    Pointd<2> offset;
    offset[0] = ptd[0] - origin.x, offset[1] = ptd[1] - origin.y;
    retv[0] = round((offset[0]*cos(-theta)  - offset[1]*sin(-theta)) / resolution);
    retv[1] = round((offset[0]*sin(-theta)  + offset[1]*cos(-theta)) / resolution);
    return retv;
  }

  DimensionLength* dim_;

  std::vector<bool> occupancy_grid;

  nav_msgs::msg::OccupancyGrid map_msg_;

};

#endif