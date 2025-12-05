#ifndef INTERACTIVE_MARKER_POSE_SERVER_INTERACTIVE_MARKER_POSE_SERVER_HPP
#define INTERACTIVE_MARKER_POSE_SERVER_INTERACTIVE_MARKER_POSE_SERVER_HPP

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

class InteractiveMarkerPoseControl
{
public:
  explicit InteractiveMarkerPoseControl( const rclcpp::Node::SharedPtr &node );
  ~InteractiveMarkerPoseControl() = default;

private:
  using InteractiveMarker = visualization_msgs::msg::InteractiveMarker;
  using InteractiveMarkerControl = visualization_msgs::msg::InteractiveMarkerControl;
  using InteractiveMarkerFeedback = visualization_msgs::msg::InteractiveMarkerFeedback;
  using Marker = visualization_msgs::msg::Marker;

  // Core helpers
  void createMarkers();
  void resetMarkers();
  void make6DofMarker( const std::string &marker_name, bool fixed );
  Marker makeBox( InteractiveMarker &msg );
  InteractiveMarkerControl &makeBoxControl( InteractiveMarker &msg );

  // Callbacks
  void processFeedback( const InteractiveMarkerFeedback::ConstSharedPtr &feedback );
  void initPoseCB( const geometry_msgs::msg::PoseStamped::SharedPtr pose );
  void resetMarkerCB( const std_msgs::msg::Empty::SharedPtr msg );

  // Node
  rclcpp::Node::SharedPtr node_;

  // Interactive marker server and menu handler
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;

  // TF2
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters / state
  std::string frame_id_;
  std::string marker_name_param_;         // legacy single-marker param
  std::vector<std::string> marker_names_; // possibly multiple markers
  bool multi_marker_mode_{ false };

  Eigen::Isometry3d init_pose_{ Eigen::Isometry3d::Identity() };
  bool init_pose_received_{ false };
  bool initial_pose_logged_{ false };

  // Publishers: one per marker
  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
      pose_publishers_;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr init_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
};

#endif // INTERACTIVE_MARKER_POSE_SERVER_INTERACTIVE_MARKER_POSE_SERVER_HPP
