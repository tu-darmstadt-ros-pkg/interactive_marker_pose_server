#include <interactive_marker_pose_server/interactive_marker_pose_server.hpp>

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );

  auto node = rclcpp::Node::make_shared( "interactive_marker_pose_server" );
  auto controller = std::make_shared<InteractiveMarkerPoseControl>( node );

  rclcpp::spin( node );
  rclcpp::shutdown();
  return 0;
}
