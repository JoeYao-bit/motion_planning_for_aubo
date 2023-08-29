// establish a fake robot to receive command
#include "common.h"


using std::placeholders::_1;

geometry_msgs::msg::Twist cmd_vel, pre_cmd_vel;
geometry_msgs::msg::PoseWithCovariance pose_with_covariance;

class StaticFramePublisher : public rclcpp::Node
{
public:
  explicit StaticFramePublisher()
  : Node("static_turtle_tf2_broadcaster")
  {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Publish static transforms once at startup
    //this->make_transforms();
  }

  void pub_transforms()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";

    t.transform.translation.x = 0;
    t.transform.translation.y = 0;
    t.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }

private:

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_static_broadcaster_;

};

class FakeRobotSubscriber : public rclcpp::Node
{
public:
  FakeRobotSubscriber()
  : Node("fake_robot_subscriber")
  {

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
       "/initialpose", 10, std::bind(&FakeRobotSubscriber::initial_pose_topic_callback, this, _1));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&FakeRobotSubscriber::cmd_vel_topic_callback, this, _1));
  }

private:

  void initial_pose_topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) const
  {
    geometry_msgs::msg::Point position = msg->pose.pose.position;
    double yaw = tf2::getYaw(msg->pose.pose.orientation);
    RCLCPP_INFO(this->get_logger(), "Receive initial pose(x, y, yaw): %f, %f, %f", position.x, position.y, yaw);
    pose_with_covariance = msg->pose;
    // only for test
    //pose_with_covariance.pose.position.x = -1.81;
    //pose_with_covariance.pose.position.y =  0.53;
  }

  void cmd_vel_topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Receive cmd start");
    geometry_msgs::msg::Vector3 v = msg->linear;
    geometry_msgs::msg::Vector3 w = msg->angular;
    RCLCPP_INFO(this->get_logger(), "Receive cmd vel(vx, w): %f, %f, %f", v.x, v.y, w.z);
    cmd_vel = *msg;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

};

class OdomPublisher : public rclcpp::Node
{
  public:
    OdomPublisher()
    : Node("odom_publisher")
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 1);
      odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    void publish_odometry()
    {
      auto& current_pose = pose_with_covariance.pose;

      double x = current_pose.position.x + control_frequency*(cmd_vel.linear.x + pre_cmd_vel.linear.x ) * cos(tf2::getYaw(current_pose.orientation)) / 2.;
      double y = current_pose.position.y + control_frequency*(cmd_vel.linear.x + pre_cmd_vel.linear.x ) * sin(tf2::getYaw(current_pose.orientation)) / 2.;
      double yaw = tf2::getYaw(current_pose.orientation) + control_frequency*(cmd_vel.angular.z + pre_cmd_vel.angular.z) / 2.;

      std::cout << " pre yaw = " << tf2::getYaw(current_pose.orientation) << " / after yaw = " << yaw <<  std::endl;

      std::cout << "FakeRobot cmd_vel " << cmd_vel.linear.x << ", " << cmd_vel.angular.z << " / pre_cmd_vel " << pre_cmd_vel.linear.x << ", " << pre_cmd_vel.angular.z << std::endl; 
     
      // double x = current_pose.position.x + control_frequency* ( cmd_vel.linear.x + pre_cmd_vel.linear.x) * cos(tf2::getYaw(current_pose.orientation)) / 2.;
      // double y = current_pose.position.y + control_frequency* ( cmd_vel.linear.x + pre_cmd_vel.linear.x) * sin(tf2::getYaw(current_pose.orientation)) / 2.;
      // double yaw = tf2::getYaw(current_pose.orientation) + control_frequency*(cmd_vel.angular.z + pre_cmd_vel.angular.z) / 2.;

      pre_cmd_vel = cmd_vel;

      current_pose.position.x = x;
      current_pose.position.y = y;
      tf2::Quaternion quat; quat.setRPY(0, 0, yaw);
      current_pose.orientation = tf2::toMsg(quat);

      double vx = cmd_vel.linear.x;
      double vy = 0.0;
      double vth = cmd_vel.angular.z;

      rclcpp::Time current_time = this->get_clock()->now();

      //since all odometry is 6DOF we'll need a quaternion created from yaw
      quat.setRPY(0, 0, yaw);
      geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(quat);//tf2_ros::createQuaternionMsgFromRollPitchYaw(1.581071,0,yaw);

      //first, we'll publish the transform over tf
      geometry_msgs::msg::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "/odom";
      odom_trans.child_frame_id = "/base_link";

      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      //send the transform
      odom_broadcaster_->sendTransform(odom_trans);

      //next, we'll publish the odometry message over ROS
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "/odom";

      //set the position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;//odom_quat;

      std::cout << " pub yaw = " << tf2::getYaw(odom_quat) << std::endl;

      //set the velocity
      odom.child_frame_id = "/base_link";
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = vth;

      //publish the message
      publisher_->publish(odom);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

    //std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


};

// example about pub cmd_vel manually: ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

ThreadPool pool;

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    // publish tf from world to map 
    StaticFramePublisher static_pub;

    // subscribe cmd vel
    auto cmd_vel_sub_ptr   = std::make_shared<FakeRobotSubscriber>(); 

    // publish odom
    auto odom_pub_ptr   = std::make_shared<OdomPublisher>(); 

    rclcpp::WallRate loop_rate(1/control_frequency);

    while (rclcpp::ok())
    {

        RCLCPP_INFO(rclcpp::get_logger("newNode"), "-------timer callback!-----------");
        odom_pub_ptr->publish_odometry();
        static_pub.pub_transforms();
        rclcpp::spin_some(cmd_vel_sub_ptr);
        loop_rate.sleep();
    }


    return 0;
}