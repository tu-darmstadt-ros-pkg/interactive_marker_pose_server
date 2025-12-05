#include <Eigen/Geometry>
#include <cmath>
#include <interactive_marker_pose_server/interactive_marker_pose_server.hpp>

InteractiveMarkerPoseControl::InteractiveMarkerPoseControl( const rclcpp::Node::SharedPtr &node )
    : node_( node ), tf_buffer_( node_->get_clock() )
{
  RCLCPP_INFO( node_->get_logger(), "Starting InteractiveMarkerPoseControl node." );

  // --- Parameters -----------------------------------------------------------
  frame_id_ = node_->declare_parameter<std::string>( "frame_id", "base" );

  // Use a consistent, typo-free default name
  marker_name_param_ =
      node_->declare_parameter<std::string>( "marker_name", "interactive_marker_pose_server" );

  // Optional multi-marker mode
  node_->declare_parameter<std::vector<std::string>>( "marker_names", std::vector<std::string>() );
  marker_names_ = node_->get_parameter( "marker_names" ).as_string_array();
  if ( !marker_names_.empty() ) {
    multi_marker_mode_ = true;
    RCLCPP_DEBUG( node_->get_logger(), "Configured for multi-marker mode with %zu markers.",
                  marker_names_.size() );
  } else {
    // Legacy single-marker behavior
    marker_names_.push_back( marker_name_param_ );
    multi_marker_mode_ = false;
    RCLCPP_DEBUG( node_->get_logger(), "Configured for single-marker mode with marker '%s'.",
                  marker_name_param_.c_str() );
  }

  // Initial pose (x, y, z, roll, pitch, yaw)
  std::vector<double> init_pose_vec =
      node_->declare_parameter<std::vector<double>>( "init_pose", std::vector<double>( 6, 0.0 ) );

  if ( init_pose_vec.size() != 6 ) {
    RCLCPP_ERROR( node_->get_logger(), "Init pose must have 6 elements." );
    throw std::runtime_error( "Init pose must have 6 elements" );
  }

  // Construct initial pose as RPY on top of translation
  init_pose_ = Eigen::AngleAxisd( init_pose_vec[5], Eigen::Vector3d::UnitZ() ) *
               Eigen::AngleAxisd( init_pose_vec[4], Eigen::Vector3d::UnitY() ) *
               Eigen::AngleAxisd( init_pose_vec[3], Eigen::Vector3d::UnitX() );
  init_pose_.translation() = Eigen::Vector3d( init_pose_vec[0], init_pose_vec[1], init_pose_vec[2] );

  init_pose_received_ = true;

  RCLCPP_DEBUG( node_->get_logger(),
                "Initial pose: pos=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f), frame_id='%s'",
                init_pose_vec[0], init_pose_vec[1], init_pose_vec[2], init_pose_vec[3],
                init_pose_vec[4], init_pose_vec[5], frame_id_.c_str() );

  // --- Publishers per marker -----------------------------------------------
  for ( const auto &name : marker_names_ ) {
    std::string topic;
    if ( !multi_marker_mode_ ) {
      // Backwards compatible: single marker publishes on "pose"
      topic = "pose";
    } else {
      // Multi-marker: "<marker_name>/pose"
      topic = name + "/pose";
    }

    pose_publishers_[name] = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        topic, rclcpp::QoS( 1 ).reliable() );
    RCLCPP_DEBUG( node_->get_logger(), "Publishing pose for marker '%s' on topic '%s'",
                  name.c_str(), topic.c_str() );
  }

  // --- Interactive Marker Server -------------------------------------------
  // Use the same name/namespace as marker_name_param_, so RViz can point to it.
  server_ =
      std::make_shared<interactive_markers::InteractiveMarkerServer>( marker_name_param_, node_ );

  RCLCPP_INFO( node_->get_logger(),
               "InteractiveMarkerServer created with name '%s'. RViz update topic: '/%s/update'",
               marker_name_param_.c_str(), marker_name_param_.c_str() );

  // --- Init pose source (optional TF-based init) ---------------------------
  std::string init_pose_topic = node_->declare_parameter<std::string>( "init_pose_topic", "" );

  if ( !init_pose_topic.empty() ) {
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>( tf_buffer_ );
    init_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        init_pose_topic, rclcpp::QoS( 1 ).best_effort(),
        std::bind( &InteractiveMarkerPoseControl::initPoseCB, this, std::placeholders::_1 ) );
    RCLCPP_DEBUG(
        node_->get_logger(),
        "Waiting for initial pose on topic '%s'. Markers will be created after first pose.",
        init_pose_topic.c_str() );
    init_pose_received_ = false;
  } else {
    RCLCPP_DEBUG(
        node_->get_logger(),
        "No 'init_pose_topic' set. Creating markers immediately with configured initial pose." );
    createMarkers();
  }

  // --- Reset subscription ---------------------------------------------------
  std::string reset_topic = node_->declare_parameter<std::string>( "reset_topic", "reset_marker" );
  reset_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      reset_topic, rclcpp::QoS( 1 ).reliable(),
      std::bind( &InteractiveMarkerPoseControl::resetMarkerCB, this, std::placeholders::_1 ) );

  RCLCPP_DEBUG( node_->get_logger(), "Subscribed to reset topic '%s'.", reset_topic.c_str() );
}

void InteractiveMarkerPoseControl::createMarkers()
{
  RCLCPP_DEBUG( node_->get_logger(), "Creating interactive markers." );
  server_->clear();

  for ( const auto &name : marker_names_ ) { make6DofMarker( name, false ); }

  server_->applyChanges();
  RCLCPP_DEBUG( node_->get_logger(), "Interactive markers created and applied." );
}

void InteractiveMarkerPoseControl::resetMarkers()
{
  RCLCPP_DEBUG( node_->get_logger(), "Resetting all interactive markers to initial pose." );
  createMarkers();
}

InteractiveMarkerPoseControl::Marker InteractiveMarkerPoseControl::makeBox( InteractiveMarker &msg )
{
  Marker marker;
  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5f;
  marker.color.g = 0.5f;
  marker.color.b = 0.5f;
  marker.color.a = 1.0f;

  return marker;
}

InteractiveMarkerPoseControl::InteractiveMarkerControl &
InteractiveMarkerPoseControl::makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  // Make the cube visible so the marker is easy to spot in RViz.
  control.markers.push_back( makeBox( msg ) );
  msg.controls.push_back( control );
  return msg.controls.back();
}

void InteractiveMarkerPoseControl::make6DofMarker( const std::string &marker_name, bool fixed )
{
  RCLCPP_DEBUG( node_->get_logger(), "Creating 6-DoF marker '%s' (fixed=%s) in frame '%s'.",
                marker_name.c_str(), fixed ? "true" : "false", frame_id_.c_str() );

  InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id_;
  int_marker.header.stamp = node_->now();
  int_marker.scale = 0.2;

  int_marker.name = marker_name;
  int_marker.description = marker_name;

  // Initial pose from init_pose_
  int_marker.pose = tf2::toMsg( init_pose_ );

  // Insert a box control (visible cube)
  makeBoxControl( int_marker );

  InteractiveMarkerControl control;

  if ( fixed ) {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  // X-axis controls
  control.orientation.w = 0.707;
  control.orientation.x = 0.707;
  control.orientation.y = 0.0;
  control.orientation.z = 0.0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back( control );

  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back( control );

  // Z-axis controls
  control.orientation.w = 0.707;
  control.orientation.x = 0.0;
  control.orientation.y = 0.707;
  control.orientation.z = 0.0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back( control );

  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back( control );

  // Y-axis controls
  control.orientation.w = 0.707;
  control.orientation.x = 0.0;
  control.orientation.y = 0.0;
  control.orientation.z = 0.707;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back( control );

  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back( control );

  // Insert into server and bind feedback callback
  server_->insert( int_marker, std::bind( &InteractiveMarkerPoseControl::processFeedback, this,
                                          std::placeholders::_1 ) );

  RCLCPP_DEBUG( node_->get_logger(), "6-DoF marker '%s' inserted into InteractiveMarkerServer.",
                int_marker.name.c_str() );
}

void InteractiveMarkerPoseControl::processFeedback(
    const InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  if ( feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE ) {
    auto it = pose_publishers_.find( feedback->marker_name );
    if ( it != pose_publishers_.end() ) {
      geometry_msgs::msg::PoseStamped msg;
      msg.pose = feedback->pose;
      msg.header = feedback->header;
      it->second->publish( msg );

      RCLCPP_DEBUG( node_->get_logger(), "Published pose update for marker '%s' at time %.3f.",
                    feedback->marker_name.c_str(), rclcpp::Time( feedback->header.stamp ).seconds() );
    } else {
      // Should not happen unless marker names/publishers get out of sync
      RCLCPP_WARN( node_->get_logger(), "Received feedback for unknown marker '%s'",
                   feedback->marker_name.c_str() );
    }
  }

  server_->applyChanges();
}

void InteractiveMarkerPoseControl::initPoseCB( const geometry_msgs::msg::PoseStamped::SharedPtr pose )
{
  if ( !tf_listener_ ) {
    RCLCPP_WARN( node_->get_logger(), "Received init pose, but TF listener not initialized." );
    return;
  }

  geometry_msgs::msg::TransformStamped marker_to_pose_frame;
  try {
    marker_to_pose_frame =
        tf_buffer_.lookupTransform( frame_id_, pose->header.frame_id, pose->header.stamp );
  } catch ( const tf2::TransformException &ex ) {
    RCLCPP_ERROR( node_->get_logger(), "Transformation exception in initPoseCB: %s", ex.what() );
    return;
  }

  // Convert TF transform and pose to Eigen
  Eigen::Isometry3d marker_to_pose_frame_eigen;
  Eigen::Isometry3d pose_tf_eigen;

  marker_to_pose_frame_eigen = tf2::transformToEigen( marker_to_pose_frame );
  tf2::fromMsg( pose->pose, pose_tf_eigen );

  Eigen::Isometry3d marker_pose = marker_to_pose_frame_eigen * pose_tf_eigen;
  init_pose_ = marker_pose;

  if ( !initial_pose_logged_ ) {
    RCLCPP_DEBUG( node_->get_logger(), "Received initial position from topic and TF." );
    initial_pose_logged_ = true;
  }

  if ( !init_pose_received_ ) {
    init_pose_received_ = true;
    RCLCPP_DEBUG( node_->get_logger(),
                  "Initial pose received for the first time. Creating / resetting markers." );
    resetMarkers();
  }
}

void InteractiveMarkerPoseControl::resetMarkerCB( const std_msgs::msg::Empty::SharedPtr /*msg*/ )
{
  RCLCPP_DEBUG( node_->get_logger(), "Reset request received. Resetting interactive markers." );
  resetMarkers();
}
