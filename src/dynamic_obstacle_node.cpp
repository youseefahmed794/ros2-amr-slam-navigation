/**
 * @file dynamic_obstacle_node.cpp
 * @brief Publishes moving obstacles as Gazebo model commands for Nav2 testing.
 *
 * Spawns virtual cylindrical obstacles that move along predefined paths in the
 * warehouse, creating dynamic avoidance scenarios. The obstacles are published
 * as TF frames so Nav2's costmap can detect them via the obstacle layer.
 *
 * Usage:
 *   ros2 run amr_slam_navigation dynamic_obstacle_node
 *   ros2 run amr_slam_navigation dynamic_obstacle_node --ros-args \
 *       -p num_obstacles:=3 -p speed:=0.4
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

struct ObstacleState {
  double x, y;
  double vx, vy;
  double path_x_min, path_x_max;
  double path_y_min, path_y_max;
  std::string frame_id;
};

class DynamicObstacleNode : public rclcpp::Node {
public:
  DynamicObstacleNode() : Node("dynamic_obstacle_node") {
    // Parameters
    this->declare_parameter("num_obstacles", 3);
    this->declare_parameter("speed", 0.3);
    this->declare_parameter("publish_rate", 10.0);

    const int num = this->get_parameter("num_obstacles").as_int();
    const double speed = this->get_parameter("speed").as_double();
    const double rate = this->get_parameter("publish_rate").as_double();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize obstacles with predefined patrol paths
    // These positions are chosen to create interesting navigation challenges
    struct PathDef {
      double x0, y0, x_min, x_max, y_min, y_max, vx, vy;
    };

    std::vector<PathDef> paths = {
      // Obstacle patrolling the corridor horizontally
      {0.0, 4.5, -3.0, 8.0, 4.5, 4.5, speed, 0.0},
      // Obstacle moving vertically between shelves
      {-7.0, -3.0, -7.0, -7.0, -6.0, 3.0, 0.0, speed},
      // Obstacle in the right room
      {7.0, -3.0, 4.0, 10.0, -3.0, -3.0, speed * 0.8, 0.0},
      // Diagonal mover near center
      {2.0, -2.0, 0.0, 6.0, -5.0, 1.0, speed * 0.6, speed * 0.4},
    };

    for (int i = 0; i < std::min(num, static_cast<int>(paths.size())); ++i) {
      auto& p = paths[i];
      obstacles_.push_back({
        p.x0, p.y0, p.vx, p.vy,
        p.x_min, p.x_max, p.y_min, p.y_max,
        "dynamic_obstacle_" + std::to_string(i)
      });
    }

    // Publishers for each obstacle's "scan" (as odometry for visualization)
    for (size_t i = 0; i < obstacles_.size(); ++i) {
      auto pub = this->create_publisher<nav_msgs::msg::Odometry>(
        "/dynamic_obstacles/obstacle_" + std::to_string(i), 10);
      odom_pubs_.push_back(pub);
    }

    // Timer
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&DynamicObstacleNode::update_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "Dynamic obstacle node started with %zu obstacles at %.1f m/s",
      obstacles_.size(), speed);
  }

private:
  void update_callback() {
    auto now = this->get_clock()->now();
    double dt = 1.0 / this->get_parameter("publish_rate").as_double();

    for (size_t i = 0; i < obstacles_.size(); ++i) {
      auto& obs = obstacles_[i];

      // Update position
      obs.x += obs.vx * dt;
      obs.y += obs.vy * dt;

      // Bounce off path boundaries
      if (obs.x <= obs.path_x_min || obs.x >= obs.path_x_max) {
        obs.vx = -obs.vx;
        obs.x = std::clamp(obs.x, obs.path_x_min, obs.path_x_max);
      }
      if (obs.y <= obs.path_y_min || obs.y >= obs.path_y_max) {
        obs.vy = -obs.vy;
        obs.y = std::clamp(obs.y, obs.path_y_min, obs.path_y_max);
      }

      // Publish TF (so costmap obstacle layer can see it)
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = now;
      tf.header.frame_id = "odom";
      tf.child_frame_id = obs.frame_id;
      tf.transform.translation.x = obs.x;
      tf.transform.translation.y = obs.y;
      tf.transform.translation.z = 0.3;
      tf.transform.rotation.w = 1.0;
      tf_broadcaster_->sendTransform(tf);

      // Publish odometry for visualization
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = now;
      odom.header.frame_id = "odom";
      odom.child_frame_id = obs.frame_id;
      odom.pose.pose.position.x = obs.x;
      odom.pose.pose.position.y = obs.y;
      odom.pose.pose.position.z = 0.3;
      odom.pose.pose.orientation.w = 1.0;
      odom.twist.twist.linear.x = obs.vx;
      odom.twist.twist.linear.y = obs.vy;
      odom_pubs_[i]->publish(odom);
    }
  }

  std::vector<ObstacleState> obstacles_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_pubs_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicObstacleNode>());
  rclcpp::shutdown();
  return 0;
}
