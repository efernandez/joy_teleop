/**
 * Based on the turtlebot_marker_server.cpp
 */

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

#include <string>

class MarkerTeleopServer
{
  public:
    MarkerTeleopServer()
      : _nh()
      , _server("marker_teleop_server")
      , _link_name("base_footprint")
      , _linear_scale(1.0)
      , _angular_scale(1.0)
      , _holonomic(false)
      , _position(false)
    {
      ros::NodeHandle nh_priv("~");

      nh_priv.param("link_name", _link_name, _link_name);
      nh_priv.param("linear_scale", _linear_scale, _linear_scale);
      nh_priv.param("angular_scale", _angular_scale, _angular_scale);
      nh_priv.param("holonomic", _holonomic, _holonomic);
      nh_priv.param("position", _position, _position);

      _pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
      createInteractiveMarkers();
    }

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  private:
    void createInteractiveMarkers();

    ros::NodeHandle _nh;
    ros::Publisher _pub;
    interactive_markers::InteractiveMarkerServer _server;

    std::string _link_name;

    double _linear_scale;
    double _angular_scale;

    bool _holonomic;
    bool _position;
};

void MarkerTeleopServer::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  // Handle angular change (yaw is the only direction in which you can rotate)
  double yaw = tf::getYaw(feedback->pose.orientation);

  if (_position)
  {
    // @todo
  }
  else
  {
    geometry_msgs::Twist vel;
    vel.angular.z = _angular_scale * yaw;
    vel.linear.x = _linear_scale * feedback->pose.position.x;

    _pub.publish(vel);
  }

  // Make the marker snap back to the robot
  _server.setPose("marker_teleop", geometry_msgs::Pose());

  _server.applyChanges();
}

void MarkerTeleopServer::createInteractiveMarkers()
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = _link_name;
  int_marker.name = "marker_teleop";
  int_marker.description = "Move";

  visualization_msgs::InteractiveMarkerControl control;

  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);

  if (_holonomic)
  {
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  _server.insert(int_marker, boost::bind(&MarkerTeleopServer::processFeedback, this, _1));

  _server.applyChanges();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_teleop_server");
  MarkerTeleopServer mts;

  while (ros::ok())
  {
    ros::spin();
  }

  return EXIT_SUCCESS;
}
