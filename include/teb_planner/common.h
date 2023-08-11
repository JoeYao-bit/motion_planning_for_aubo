#include <memory>

#include "point.h"
#include "thread_pool.h"
#include "path_smooth.h"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"

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

const double control_frequency = 1;