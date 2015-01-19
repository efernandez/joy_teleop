
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/PoseStamped.h>
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
    {
      ros::NodeHandle nh_priv("~");

      nh_priv.param("link_name", _link_name, _link_name);

      _pub = _nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

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
};

void MarkerTeleopServer::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  geometry_msgs::PoseStamped pose;
  pose.header = feedback->header;
  pose.pose   = feedback->pose;

  _pub.publish(pose);

  // Make the marker snap back to the robot
  _srv.setPose(_int_marker_name, geometry_msgs::Pose());

  _srv.applyChanges();
}

void MarkerTeleopServer::createInteractiveMarker()
{
  _int_marker.header.frame_id = _link_name;
  _int_marker.name = _int_marker_name;

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

  _int_marker.description = "Pose";

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "move_plane";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  _int_marker.controls.push_back(control);

  _srv.insert(_int_marker, boost::bind(&MarkerTeleopServer::processFeedback, this, _1));

  _srv.applyChanges();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_marker_teleop_node");
  MarkerTeleopServer mts;

  while (ros::ok())
  {
    ros::spin();
  }

  return EXIT_SUCCESS;
}
