/**
 * Based on the turtlebot_marker_server.cpp
 */

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
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
      , _mode(MODE_TWIST)
    {
      ros::NodeHandle nh_priv("~");

      nh_priv.param("link_name", _link_name, _link_name);
      nh_priv.param("linear_scale", _linear_scale, _linear_scale);
      nh_priv.param("angular_scale", _angular_scale, _angular_scale);
      nh_priv.param("holonomic", _holonomic, _holonomic);

      std::string mode_str = modeToString(_mode);
      nh_priv.param("mode", mode_str, mode_str);
      _mode = modeFromString(mode_str);
      if (_mode == MODE_UNKNOWN)
      {
        ROS_WARN_STREAM("Unknown mode '" << mode_str << "'! Using default mode '" << MODE_DEFAULT << "'");
        _mode = MODE_DEFAULT;
      }

      _pub_cmd_vel = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
      _pub_pose    = _nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
      _pub_path    = _nh.advertise<nav_msgs::Path>("path", 1);

      createInteractiveMarker();
      addMenu();
    }

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void modeCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  private:
    enum MenuEntry
    {
      MENU_ENTRY_TWIST = 1,
      MENU_ENTRY_POSE,
      MENU_ENTRY_PATH
    };

    enum Mode
    {
      MODE_TWIST,
      MODE_POSE,
      MODE_PATH,

      MODE_UNKNOWN,
      MODE_DEFAULT = MODE_TWIST
    };

    void createInteractiveMarker();

    void addMenu();

    void updateInteractiveMarkerControl();

    static Mode modeFromString(const std::string& str);
    static std::string modeToString(const Mode& mode);

    ros::NodeHandle _nh;
    ros::Publisher _pub_cmd_vel;
    ros::Publisher _pub_pose;
    ros::Publisher _pub_path;

    interactive_markers::InteractiveMarkerServer _srv;
    interactive_markers::MenuHandler _menu;

    visualization_msgs::InteractiveMarker _int_marker;

    std::string _int_marker_name;

    nav_msgs::Path _path_msg;

    std::string _link_name;

    double _linear_scale;
    double _angular_scale;

    bool _holonomic;

    Mode _mode;
};

void MarkerTeleopServer::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (_mode == MODE_POSE)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = feedback->header;
    pose.pose   = feedback->pose;

    if (_mode == MODE_PATH)
    {
      if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN)
      {
        _path_msg.poses.clear();
      }

      _path_msg.header = feedback->header;
      _path_msg.poses.push_back(pose);

      _pub_path.publish(_path_msg);
    }
    else
    {
      _pub_pose.publish(pose);
    }
  }
  else // if (_mode == MODE_PATH)
  {
    const double yaw = tf::getYaw(feedback->pose.orientation);

    geometry_msgs::Twist vel;
    vel.angular.z = _angular_scale * yaw;
    vel.linear.x = _linear_scale * feedback->pose.position.x;

    _pub_cmd_vel.publish(vel);
  }

  // Make the marker snap back to the robot
  _srv.setPose("marker_teleop", geometry_msgs::Pose());

  _srv.applyChanges();
}

void MarkerTeleopServer::modeCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_ASSERT_MSG(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT, "Event type != MENU_SELECT");

  switch (feedback->menu_entry_id)
  {
    case MENU_ENTRY_TWIST:
      _mode = MODE_TWIST;
      break;

    case MENU_ENTRY_POSE:
      _mode = MODE_POSE;
      break;

    case MENU_ENTRY_PATH:
      _mode = MODE_PATH;
      break;

    default:
      ROS_ERROR("Unknown menu entry!");
      return;
  }

  _srv.erase(_int_marker.name);
  createInteractiveMarker();
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

  if (_mode == MODE_POSE)
  {
    _int_marker.description = _mode == MODE_PATH ? "Path" : "Position";

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "move_plane";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  }
  else // if (_mode == MODE_TWIST)
  {
    _int_marker.description = "Twist";

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "move_rotate";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  }
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

void MarkerTeleopServer::addMenu()
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;

  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  control.name = "menu";
  control.description = "Menu";
  control.always_visible = true;

  marker.header.frame_id = "base_link";
  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.b = 0.0;
  marker.color.a = 0.5;
  marker.mesh_resource = "package://ant_description/meshes/base/base_v2.stl";
  control.markers.push_back(marker);

  marker.header.frame_id = "wheel_left_link";
  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.b = 0.0;
  marker.mesh_resource = "package://ant_description/meshes/wheels/wheel_v2.stl";
  control.markers.push_back(marker);

  marker.header.frame_id = "wheel_right_link";
  control.markers.push_back(marker);

  // @todo add casters

  _int_marker.controls.push_back(control);

  const auto callback = boost::bind(&MarkerTeleopServer::modeCallback, this, _1);
  _menu.insert("twist" , callback);
  _menu.insert("pose" , callback);
  _menu.insert("path" , callback);

  _menu.apply(_srv, _int_marker_name);
  _srv.applyChanges();
}

void MarkerTeleopServer::updateInteractiveMarkerControl()
{
  // @todo simply update the controls (except for the menu one)
  // _int_marker.controls.
}

MarkerTeleopServer::Mode MarkerTeleopServer::modeFromString(const std::string& str)
{
  if      (str == "twist") return MODE_TWIST;
  else if (str == "pose" ) return MODE_POSE;
  else if (str == "path" ) return MODE_PATH;
  else                     return MODE_UNKNOWN;
}

std::string MarkerTeleopServer::modeToString(const Mode& mode)
{
  switch (mode)
  {
    case MODE_TWIST: return "twist";
    case MODE_POSE : return "pose";
    case MODE_PATH : return "path";
    default:         return "";
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_teleop_node");
  MarkerTeleopServer mts;

  while (ros::ok())
  {
    ros::spin();
  }

  return EXIT_SUCCESS;
}
