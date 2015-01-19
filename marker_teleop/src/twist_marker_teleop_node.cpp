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
      , _srv("marker_teleop_srv")
      , _int_marker_name("marker_teleop")
      , _link_name("base_footprint")
      , _linear_scale(1.0)
      , _angular_scale(1.0)
      , _holonomic(false)
    {
      ros::NodeHandle nh_priv("~");

      nh_priv.param("link_name", _link_name, _link_name);
      nh_priv.param("linear_scale", _linear_scale, _linear_scale);
      nh_priv.param("angular_scale", _angular_scale, _angular_scale);
      nh_priv.param("holonomic", _holonomic, _holonomic);

      _pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

      createInteractiveMarker();
    }

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  private:
    void createInteractiveMarker();

    ros::NodeHandle _nh;
    ros::Publisher _pub;

    interactive_markers::InteractiveMarkerServer _srv;

    visualization_msgs::InteractiveMarker _int_marker;

    std::string _int_marker_name;

    std::string _link_name;

    double _linear_scale;
    double _angular_scale;

    bool _holonomic;
};

void MarkerTeleopServer::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  const double yaw = tf::getYaw(feedback->pose.orientation);

  geometry_msgs::Twist vel;
  vel.angular.z = _angular_scale * yaw;
  vel.linear.x = _linear_scale * feedback->pose.position.x;

  _pub.publish(vel);

  // Make the marker snap back to the robot
  _srv.setPose(_int_marker_name, geometry_msgs::Pose());

  _srv.applyChanges();
}

void MarkerTeleopServer::createInteractiveMarker()
{
  _int_marker.header.frame_id = _link_name;
  _int_marker.name = _int_marker_name;
  _int_marker.description = "Twist";

  visualization_msgs::InteractiveMarkerControl control;

  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  _int_marker.controls.push_back(control);

  /* @todo disable because it hides the MOVE_PLANE/ROTATE
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  _int_marker.controls.push_back(control);
  */

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "move_rotate";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  _int_marker.controls.push_back(control);

  if (_holonomic)
  {
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    _int_marker.controls.push_back(control);
  }

  _srv.insert(_int_marker, boost::bind(&MarkerTeleopServer::processFeedback, this, _1));

  _srv.applyChanges();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist_marker_teleop_node");
  MarkerTeleopServer mts;

  while (ros::ok())
  {
    ros::spin();
  }

  return EXIT_SUCCESS;
}
