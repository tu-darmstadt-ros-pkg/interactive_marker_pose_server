#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <hector_testing_utils/hector_testing_utils.hpp>
#include <interactive_marker_pose_server/interactive_marker_pose_server.hpp>

#if __has_include( <tf2_ros/static_transform_broadcaster.hpp> )
  #include <tf2_ros/static_transform_broadcaster.hpp>
#else
  #include <tf2_ros/static_transform_broadcaster.h>
#endif

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/interactive_marker_update.hpp>
#include <visualization_msgs/srv/get_interactive_markers.hpp>

using namespace std::chrono_literals;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using visualization_msgs::msg::InteractiveMarker;
using visualization_msgs::msg::InteractiveMarkerFeedback;
using visualization_msgs::msg::InteractiveMarkerUpdate;
using visualization_msgs::srv::GetInteractiveMarkers;

namespace
{
constexpr double kTolerance = 1e-6;

Pose makePose( double x, double y, double z, double roll, double pitch, double yaw )
{
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = ( Eigen::AngleAxisd( yaw, Eigen::Vector3d::UnitZ() ) *
                   Eigen::AngleAxisd( pitch, Eigen::Vector3d::UnitY() ) *
                   Eigen::AngleAxisd( roll, Eigen::Vector3d::UnitX() ) )
                     .toRotationMatrix();
  iso.translation() = Eigen::Vector3d( x, y, z );
  return tf2::toMsg( iso );
}

bool posesEqual( const Pose &a, const Pose &b )
{
  Eigen::Quaterniond qa( a.orientation.w, a.orientation.x, a.orientation.y, a.orientation.z );
  Eigen::Quaterniond qb( b.orientation.w, b.orientation.x, b.orientation.y, b.orientation.z );
  // q and -q describe the same rotation
  return std::abs( a.position.x - b.position.x ) < kTolerance &&
         std::abs( a.position.y - b.position.y ) < kTolerance &&
         std::abs( a.position.z - b.position.z ) < kTolerance &&
         std::abs( std::abs( qa.dot( qb ) ) - 1.0 ) < kTolerance;
}

::testing::AssertionResult assertPoseEq( const Pose &actual, const Pose &expected )
{
  if ( posesEqual( actual, expected ) ) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "pose (" << actual.position.x << ", " << actual.position.y << ", " << actual.position.z
         << " | " << actual.orientation.x << ", " << actual.orientation.y << ", "
         << actual.orientation.z << ", " << actual.orientation.w << ") != expected ("
         << expected.position.x << ", " << expected.position.y << ", " << expected.position.z
         << " | " << expected.orientation.x << ", " << expected.orientation.y << ", "
         << expected.orientation.z << ", " << expected.orientation.w << ")";
}
} // namespace

// Parameterized over the namespace the server node is started in.
class InteractiveMarkerPoseServerTest : public hector_testing_utils::HectorTestFixture,
                                        public ::testing::WithParamInterface<std::string>
{
protected:
  void TearDown() override
  {
    control_.reset();
    server_node_.reset();
    HectorTestFixture::TearDown();
  }

  void startServer( const std::vector<rclcpp::Parameter> &parameters )
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides( parameters );
    server_node_ =
        std::make_shared<rclcpp::Node>( "interactive_marker_pose_server", GetParam(), options );
    control_ = std::make_shared<InteractiveMarkerPoseControl>( server_node_ );
    executor_->add_node( server_node_ );
  }

  // Resolves a name relative to the namespace of the server node.
  std::string resolve( const std::string &name ) const
  {
    const std::string &ns = GetParam();
    return ( ns.empty() || ns == "/" ? "" : ns ) + "/" + name;
  }

  std::vector<InteractiveMarker> getMarkers( const std::string &server_name )
  {
    auto client = tester_node_->create_test_client<GetInteractiveMarkers>(
        resolve( server_name + "/get_interactive_markers" ) );
    auto response = hector_testing_utils::call_service<GetInteractiveMarkers>(
        client->get(), std::make_shared<GetInteractiveMarkers::Request>(), *executor_ );
    if ( !response ) {
      ADD_FAILURE() << "Service call to get_interactive_markers of '" << server_name << "' failed.";
      return {};
    }
    return response->markers;
  }

  const InteractiveMarker *findMarker( const std::vector<InteractiveMarker> &markers,
                                       const std::string &name )
  {
    for ( const auto &marker : markers ) {
      if ( marker.name == name )
        return &marker;
    }
    return nullptr;
  }

  InteractiveMarkerFeedback makeFeedback( const std::string &marker_name, const Pose &pose,
                                          const std::string &frame_id )
  {
    InteractiveMarkerFeedback feedback;
    feedback.header.frame_id = frame_id;
    feedback.header.stamp = tester_node_->now();
    feedback.marker_name = marker_name;
    feedback.control_name = "move_x";
    feedback.event_type = InteractiveMarkerFeedback::POSE_UPDATE;
    feedback.pose = pose;
    return feedback;
  }

  rclcpp::Node::SharedPtr server_node_;
  std::shared_ptr<InteractiveMarkerPoseControl> control_;
};

TEST_P( InteractiveMarkerPoseServerTest, CreatesSingleMarkerAtInitPose )
{
  startServer(
      { rclcpp::Parameter( "marker_name", "pose_marker" ), rclcpp::Parameter( "frame_id", "map" ),
        rclcpp::Parameter( "init_pose", std::vector<double>{ 1.0, 2.0, 3.0, 0.1, 0.2, 0.3 } ) } );

  auto markers = getMarkers( "pose_marker" );
  ASSERT_EQ( markers.size(), 1u );
  EXPECT_EQ( markers[0].name, "pose_marker" );
  EXPECT_EQ( markers[0].header.frame_id, "map" );
  EXPECT_TRUE( assertPoseEq( markers[0].pose, makePose( 1.0, 2.0, 3.0, 0.1, 0.2, 0.3 ) ) );
  // Box control + move and rotate control for each axis
  EXPECT_EQ( markers[0].controls.size(), 7u );
}

TEST_P( InteractiveMarkerPoseServerTest, CreatesAllMarkersInMultiMarkerMode )
{
  startServer(
      { rclcpp::Parameter( "marker_name", "multi_server" ),
        rclcpp::Parameter( "marker_names", std::vector<std::string>{ "left", "right" } ),
        rclcpp::Parameter( "init_pose", std::vector<double>{ 0.5, 0.0, 0.0, 0.0, 0.0, 0.0 } ) } );

  auto markers = getMarkers( "multi_server" );
  ASSERT_EQ( markers.size(), 2u );
  for ( const std::string name : { "left", "right" } ) {
    const InteractiveMarker *marker = findMarker( markers, name );
    ASSERT_NE( marker, nullptr ) << "Marker '" << name << "' missing.";
    EXPECT_TRUE( assertPoseEq( marker->pose, makePose( 0.5, 0.0, 0.0, 0.0, 0.0, 0.0 ) ) );
  }
}

TEST_P( InteractiveMarkerPoseServerTest, FeedbackPublishesPoseInSingleMarkerMode )
{
  startServer( { rclcpp::Parameter( "marker_name", "pose_marker" ),
                 rclcpp::Parameter( "frame_id", "map" ) } );

  auto feedback_pub = tester_node_->create_test_publisher<InteractiveMarkerFeedback>(
      resolve( "pose_marker/feedback" ) );
  auto pose_sub = tester_node_->create_test_subscription<PoseStamped>( resolve( "pose" ) );
  ASSERT_TRUE( tester_node_->wait_for_all_connections( *executor_, 10s ) );

  const Pose target = makePose( 1.5, -0.5, 0.25, 0.0, 0.0, M_PI / 2 );
  feedback_pub->publish( makeFeedback( "pose_marker", target, "map" ) );

  ASSERT_TRUE( pose_sub->wait_for_message( *executor_, 5s ) );
  EXPECT_EQ( pose_sub->last_message()->header.frame_id, "map" );
  EXPECT_TRUE( assertPoseEq( pose_sub->last_message()->pose, target ) );
}

TEST_P( InteractiveMarkerPoseServerTest, FeedbackPublishesOnlyOnTopicOfMovedMarker )
{
  startServer(
      { rclcpp::Parameter( "marker_name", "multi_server" ),
        rclcpp::Parameter( "marker_names", std::vector<std::string>{ "left", "right" } ) } );

  auto feedback_pub = tester_node_->create_test_publisher<InteractiveMarkerFeedback>(
      resolve( "multi_server/feedback" ) );
  auto left_sub = tester_node_->create_test_subscription<PoseStamped>( resolve( "left/pose" ) );
  auto right_sub = tester_node_->create_test_subscription<PoseStamped>( resolve( "right/pose" ) );
  ASSERT_TRUE( tester_node_->wait_for_all_connections( *executor_, 10s ) );

  const Pose target = makePose( 0.0, 1.0, 0.0, 0.0, 0.0, 0.0 );
  feedback_pub->publish( makeFeedback( "right", target, "base" ) );

  ASSERT_TRUE( right_sub->wait_for_message( *executor_, 5s ) );
  EXPECT_TRUE( assertPoseEq( right_sub->last_message()->pose, target ) );
  EXPECT_FALSE( left_sub->wait_for_message( *executor_, 500ms ) );
}

TEST_P( InteractiveMarkerPoseServerTest, ResetRestoresInitPose )
{
  startServer( { rclcpp::Parameter( "marker_name", "pose_marker" ),
                 rclcpp::Parameter( "init_pose", std::vector<double>{ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 } ),
                 rclcpp::Parameter( "reset_topic", "reset_marker" ) } );

  auto feedback_pub = tester_node_->create_test_publisher<InteractiveMarkerFeedback>(
      resolve( "pose_marker/feedback" ) );
  auto reset_pub =
      tester_node_->create_test_publisher<std_msgs::msg::Empty>( resolve( "reset_marker" ) );
  auto pose_sub = tester_node_->create_test_subscription<PoseStamped>( resolve( "pose" ) );
  auto update_sub = tester_node_->create_test_subscription<InteractiveMarkerUpdate>(
      resolve( "pose_marker/update" ), rclcpp::QoS( 100 ) );
  ASSERT_TRUE( tester_node_->wait_for_all_connections( *executor_, 10s ) );

  const Pose moved = makePose( 3.0, 2.0, 1.0, 0.0, 0.0, 1.0 );
  feedback_pub->publish( makeFeedback( "pose_marker", moved, "base" ) );
  ASSERT_TRUE( pose_sub->wait_for_message( *executor_, 5s ) );

  auto markers = getMarkers( "pose_marker" );
  ASSERT_EQ( markers.size(), 1u );
  ASSERT_TRUE( assertPoseEq( markers[0].pose, moved ) );

  // A reset recreates the markers, which publishes an update
  reset_pub->publish( std_msgs::msg::Empty() );
  ASSERT_TRUE(
      update_sub->wait_for_message( *executor_, 5s, []( const InteractiveMarkerUpdate &update ) {
        return update.type == InteractiveMarkerUpdate::UPDATE && !update.markers.empty();
      } ) );

  markers = getMarkers( "pose_marker" );
  ASSERT_EQ( markers.size(), 1u );
  EXPECT_TRUE( assertPoseEq( markers[0].pose, makePose( 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 ) ) );
}

TEST_P( InteractiveMarkerPoseServerTest, InitPoseTopicCreatesMarkerAtTransformedPose )
{
  startServer( { rclcpp::Parameter( "marker_name", "pose_marker" ),
                 rclcpp::Parameter( "frame_id", "map" ),
                 rclcpp::Parameter( "init_pose_topic", "init_pose" ) } );

  // No markers until the first init pose was received
  EXPECT_TRUE( getMarkers( "pose_marker" ).empty() );

  // map -> odom: translated by (1, 0, 0) and rotated by 90 deg around z
  tf2_ros::StaticTransformBroadcaster tf_broadcaster( *tester_node_ );
  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.stamp = tester_node_->now();
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = "odom";
  map_to_odom.transform.translation.x = 1.0;
  map_to_odom.transform.rotation.z = std::sin( M_PI / 4 );
  map_to_odom.transform.rotation.w = std::cos( M_PI / 4 );
  tf_broadcaster.sendTransform( map_to_odom );

  auto init_pose_pub = tester_node_->create_test_publisher<PoseStamped>( resolve( "init_pose" ) );
  auto update_sub = tester_node_->create_test_subscription<InteractiveMarkerUpdate>(
      resolve( "pose_marker/update" ), rclcpp::QoS( 100 ) );
  ASSERT_TRUE( tester_node_->wait_for_all_connections( *executor_, 10s ) );

  PoseStamped init_pose;
  init_pose.header.frame_id = "odom";
  init_pose.pose = makePose( 2.0, 0.0, 0.5, 0.0, 0.0, 0.0 );

  // The TF listener of the server may not have the transform yet, so retry until markers appear
  bool markers_created = false;
  for ( int i = 0; i < 50 && !markers_created; ++i ) {
    init_pose.header.stamp = tester_node_->now();
    init_pose_pub->publish( init_pose );
    markers_created = update_sub->wait_for_message(
        *executor_, 200ms,
        []( const InteractiveMarkerUpdate &update ) { return !update.markers.empty(); } );
  }
  ASSERT_TRUE( markers_created ) << "Markers were not created after publishing an init pose.";

  auto markers = getMarkers( "pose_marker" );
  ASSERT_EQ( markers.size(), 1u );
  EXPECT_EQ( markers[0].header.frame_id, "map" );
  // (2, 0, 0.5) in odom is (1, 2, 0.5) in map, yaw of 90 deg is inherited from the transform
  EXPECT_TRUE( assertPoseEq( markers[0].pose, makePose( 1.0, 2.0, 0.5, 0.0, 0.0, M_PI / 2 ) ) );
}

TEST_P( InteractiveMarkerPoseServerTest, ThrowsOnInvalidInitPose )
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      { rclcpp::Parameter( "init_pose", std::vector<double>{ 1.0, 2.0, 3.0 } ) } );
  auto node = std::make_shared<rclcpp::Node>( "interactive_marker_pose_server", GetParam(), options );
  EXPECT_THROW( InteractiveMarkerPoseControl control( node ), std::runtime_error );
}

INSTANTIATE_TEST_SUITE_P( Namespaces, InteractiveMarkerPoseServerTest,
                          ::testing::Values( "", "/robot" ),
                          []( const ::testing::TestParamInfo<std::string> &info ) {
                            return info.param.empty() ? std::string( "global" )
                                                      : std::string( "namespaced" );
                          } );
