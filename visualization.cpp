#include "visualization.h"


std::mt19937 MarkerVisualization::mt_random_engine_(std::chrono::system_clock::now().time_since_epoch().count());

MarkerVisualization::MarkerVisualization(ros::NodeHandle& nodeHandle): nh_(nodeHandle)
{

  marker_lifetime_ = ros::Duration(0.0);  // 0 - unlimited

  nh_.param("frame_id", global_frame_, std::string("map"));
  nh_.param("global_scale", global_scale_, 1.0);
  nh_.param("alpha", alpha_, 1.0);
  nh_.param("marker_topic", marker_topic_, std::string("markers"));


  bool latched;
  nh_.param("latched", latched, false);
  
  pub_rviz_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 10, latched);
  
  // Cache the reusable markers
  loadRvizMarkers();
  
}

MarkerVisualization::~MarkerVisualization()
{
}

bool MarkerVisualization::loadRvizMarkers()
{
  // Load reset marker -------------------------------------------------
  reset_marker_.header.frame_id = global_frame_;
  reset_marker_.header.stamp = ros::Time();
  reset_marker_.ns = "deleteAllMarkers";  // helps during debugging
  reset_marker_.action = 3;               // TODO(davetcoleman): In ROS-J set to visualization_msgs::Marker::DELETEALL;
  reset_marker_.pose.orientation.w = 1;

  // Load arrow ----------------------------------------------------

  arrow_marker_.header.frame_id = global_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  arrow_marker_.ns = "Arrow";
  // Set the marker type.
  arrow_marker_.type = visualization_msgs::Marker::ARROW;
  // Set the marker action.  Options are ADD and DELETE
  arrow_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  arrow_marker_.lifetime = marker_lifetime_;
  // Constants
  arrow_marker_.pose = getIdentityPose();

  // Load cuboid ----------------------------------------------------

  cuboid_marker_.header.frame_id = global_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  cuboid_marker_.ns = "Cuboid";
  // Set the marker type.
  cuboid_marker_.type = visualization_msgs::Marker::CUBE;
  // Set the marker action.  Options are ADD and DELETE
  cuboid_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  cuboid_marker_.lifetime = marker_lifetime_;
  // Constants
  cuboid_marker_.pose = getIdentityPose();

  // Load line ----------------------------------------------------

  line_strip_marker_.header.frame_id = global_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  line_strip_marker_.ns = "Line";
  // Set the marker type.
  line_strip_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  // Set the marker action.  Options are ADD and DELETE
  line_strip_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  line_strip_marker_.lifetime = marker_lifetime_;
  // Constants
  line_strip_marker_.pose = getIdentityPose();

  // Load path ----------------------------------------------------

  line_list_marker_.header.frame_id = global_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  line_list_marker_.ns = "Line_List";
  // Set the marker type.
  line_list_marker_.type = visualization_msgs::Marker::LINE_LIST;
  // Set the marker action.  Options are ADD and DELETE
  line_list_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  line_list_marker_.lifetime = marker_lifetime_;
  // Constants
  line_list_marker_.pose = getIdentityPose();

  // Load sphers ----------------------------------------------------

  spheres_marker_.header.frame_id = global_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  spheres_marker_.ns = "Spheres";
  // Set the marker type.
  spheres_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
  // Set the marker action.  Options are ADD and DELETE
  spheres_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  spheres_marker_.lifetime = marker_lifetime_;
  // Constants
  spheres_marker_.pose = getIdentityPose();

  // Load Block ----------------------------------------------------
  block_marker_.header.frame_id = global_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  block_marker_.ns = "Block";
  // Set the marker action.  Options are ADD and DELETE
  block_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  block_marker_.type = visualization_msgs::Marker::CUBE;
  // Lifetime
  block_marker_.lifetime = marker_lifetime_;
  // Constants
  block_marker_.pose = getIdentityPose();

  // Load Cylinder ----------------------------------------------------
  cylinder_marker_.header.frame_id = global_frame_;
  // Set the marker action.  Options are ADD and DELETE
  cylinder_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  cylinder_marker_.type = visualization_msgs::Marker::CYLINDER;
  // Lifetime
  cylinder_marker_.lifetime = marker_lifetime_;
  // Constants
  cylinder_marker_.pose = getIdentityPose();

  // Load Mesh ----------------------------------------------------
  mesh_marker_.header.frame_id = global_frame_;

  // Set the marker action.  Options are ADD and DELETE
  mesh_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  mesh_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  // Lifetime
  mesh_marker_.lifetime = marker_lifetime_;
  // Constants
  mesh_marker_.pose = getIdentityPose();

  // Load Sphere -------------------------------------------------
  sphere_marker_.header.frame_id = global_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  sphere_marker_.ns = "Sphere";
  // Set the marker type.
  sphere_marker_.type = visualization_msgs::Marker::SPHERE;
  // Set the marker action.  Options are ADD and DELETE
  sphere_marker_.action = visualization_msgs::Marker::ADD;
  // Marker group position and orientation
  sphere_marker_.pose.position.x = 0;
  sphere_marker_.pose.position.y = 0;
  sphere_marker_.pose.position.z = 0;
  sphere_marker_.pose.orientation.x = 0.0;
  sphere_marker_.pose.orientation.y = 0.0;
  sphere_marker_.pose.orientation.z = 0.0;
  sphere_marker_.pose.orientation.w = 1.0;
  // Create a sphere point
  // Add the point pair to the line message
  sphere_marker_.points.resize(1);
  sphere_marker_.colors.resize(1);
  // Lifetime
  sphere_marker_.lifetime = marker_lifetime_;
  // Constants
  sphere_marker_.pose = getIdentityPose();

  // Load Text ----------------------------------------------------
  // Set the namespace and id for this marker.  This serves to create a unique
  // ID
  text_marker_.ns = "Text";
  // Set the marker action.  Options are ADD and DELETE
  text_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  // Lifetime
  text_marker_.lifetime = marker_lifetime_;
  // Constants
  text_marker_.pose = getIdentityPose();

  // Load Triangle List -------------------------------------------
  // Set the namespace and id for this marker. This serves to create a unique ID
  triangle_marker_.header.frame_id = global_frame_;
  triangle_marker_.ns = "Triangle";
  // Set the marker action. Options are ADD and DELETE
  triangle_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type
  triangle_marker_.type = visualization_msgs::Marker::TRIANGLE_LIST;
  // Lifetime
  triangle_marker_.lifetime = marker_lifetime_;
  // Constants
  triangle_marker_.pose = getIdentityPose();

  return true;
}

bool MarkerVisualization::deleteAllMarkers()
{
  // Helper for publishing rviz markers
  return publishMarker(reset_marker_);
}

bool MarkerVisualization::trigger()
{
  if (markers_.markers.empty())
  {
    ROS_WARN_STREAM("Batch publishing triggered but queue is empty (unnecessary function call)");
    return false;
  }

  bool result = publishMarkers(markers_);

  markers_.markers.clear();  // remove all cached markers
  return result;
}

geometry_msgs::Pose MarkerVisualization::getIdentityPose()
{
  geometry_msgs::Pose pose;
  // Position
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;

  // Orientation on place
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  return pose;
}

geometry_msgs::Pose MarkerVisualization::getPoseMsg(double x, double y, double theta)
{
  geometry_msgs::Pose pose;
  // Position
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0;

  // Orientation on place
  pose.orientation = tf::createQuaternionMsgFromYaw(theta);
  return pose;
}

geometry_msgs::Pose MarkerVisualization::getPoseMsg(double x, double y, geometry_msgs::Quaternion orientation)
{
  geometry_msgs::Pose pose;
  // Position
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0;

  // Orientation on place
  pose.orientation = orientation;
  return pose;
}

std_msgs::ColorRGBA MarkerVisualization::getColor(colors color) const
{
  std_msgs::ColorRGBA result;

  switch (color)
  {
    case RED:
      result.r = 0.8;
      result.g = 0.1;
      result.b = 0.1;
      result.a = alpha_;
      break;
    case GREEN:
      result.r = 0.1;
      result.g = 0.8;
      result.b = 0.1;
      result.a = alpha_;
      break;
    case GREY:
      result.r = 0.9;
      result.g = 0.9;
      result.b = 0.9;
      result.a = alpha_;
      break;
    case DARK_GREY:
      result.r = 0.6;
      result.g = 0.6;
      result.b = 0.6;
      result.a = alpha_;
      break;
    case WHITE:
      result.r = 1.0;
      result.g = 1.0;
      result.b = 1.0;
      result.a = alpha_;
      break;
    case ORANGE:
      result.r = 1.0;
      result.g = 0.5;
      result.b = 0.0;
      result.a = alpha_;
      break;
    case TRANSLUCENT_LIGHT:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.1;
      result.a = 0.1;
      break;
    case TRANSLUCENT:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.1;
      result.a = 0.25;
      break;
    case TRANSLUCENT_DARK:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.1;
      result.a = 0.5;
      break;
    case BLACK:
      result.r = 0.0;
      result.g = 0.0;
      result.b = 0.0;
      result.a = alpha_;
      break;
    case YELLOW:
      result.r = 1.0;
      result.g = 1.0;
      result.b = 0.0;
      result.a = alpha_;
      break;
    case BROWN:
      result.r = 0.597;
      result.g = 0.296;
      result.b = 0.0;
      result.a = alpha_;
      break;
    case PINK:
      result.r = 1.0;
      result.g = 0.4;
      result.b = 1;
      result.a = alpha_;
      break;
    case LIME_GREEN:
      result.r = 0.6;
      result.g = 1.0;
      result.b = 0.2;
      result.a = alpha_;
      break;
    case CLEAR:
      result.r = 1.0;
      result.g = 1.0;
      result.b = 1.0;
      result.a = 0.0;
      break;
    case PURPLE:
      result.r = 0.597;
      result.g = 0.0;
      result.b = 0.597;
      result.a = alpha_;
      break;
    case CYAN:
      result.r = 0.0;
      result.g = 1.0;
      result.b = 1.0;
      result.a = alpha_;
      break;
    case MAGENTA:
      result.r = 1.0;
      result.g = 0.0;
      result.b = 1.0;
      result.a = alpha_;
      break;
    case RAND:
      result = createRandColor();
      break;
    case DEFAULT:
      ROS_WARN_STREAM("The 'DEFAULT' color should probably not "
                                     "be used with getColor(). Defaulting to "
                                     "blue.");
    case BLUE:
    default:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.8;
      result.a = alpha_;
      break;
  }

  return result;
}

std_msgs::ColorRGBA MarkerVisualization::createRandColor() const
{
  std_msgs::ColorRGBA result;

  const std::size_t max_attempts = 20;  // bound the performance
  std::size_t attempts = 0;

  // Make sure color is not *too* dark
  do
  {
    result.r = fRand(0.0, 1.0);
    result.g = fRand(0.0, 1.0);
    result.b = fRand(0.0, 1.0);
    // ROS_DEBUG_STREAM_NAMED(LOGNAME, "Looking for random color that is not too light, current value is "
    //<< (result.r + result.g + result.b) << " attempt #" << attempts);
    attempts++;
    if (attempts > max_attempts)
    {
      ROS_WARN_STREAM("Unable to find appropriate random color after " << max_attempts << " attempts");
      break;
    }
  } while (result.r + result.g + result.b < 1.5);  // 3 would be white

  // Set alpha value
  result.a = alpha_;

  return result;
}

geometry_msgs::Vector3 MarkerVisualization::getScale(scales scale, double marker_scale) const
{
  geometry_msgs::Vector3 result;
  double val(0.0);
  switch (scale)
  {
    case XXXXSMALL:
      val = 0.001;
      break;
    case XXXSMALL:
      val = 0.0025;
      break;
    case XXSMALL:
      val = 0.005;
      break;
    case XSMALL:
      val = 0.0065;
      break;
    case SMALL:
      val = 0.0075;
      break;
    case MEDIUM:
      val = 0.01;
      break;
    case LARGE:
      val = 0.025;
      break;
    case XLARGE:
      val = 0.05;
      break;
    case XXLARGE:
      val = 0.075;
      break;
    case XXXLARGE:
      val = 0.1;
      break;
    case XXXXLARGE:
      val = 0.5;
      break;
    default:
      ROS_ERROR_STREAM("Not implemented yet");
      break;
  }

  // Allows an individual marker size factor and a size factor for all markers
  result.x = val * marker_scale * global_scale_;
  result.y = val * marker_scale * global_scale_;
  result.z = val * marker_scale * global_scale_;

  return result;
}

void MarkerVisualization::setRandomSeed(unsigned int seed)
{
  mt_random_engine_.seed(seed);
}

double MarkerVisualization::dRand(double min, double max)
{
  return std::uniform_real_distribution<double>(min, max)(mt_random_engine_);
}

float MarkerVisualization::fRand(float min, float max)
{
  return std::uniform_real_distribution<float>(min, max)(mt_random_engine_);
}

int MarkerVisualization::iRand(int min, int max)
{
  return std::uniform_int_distribution<int>(min, max)(mt_random_engine_);
}

bool MarkerVisualization::publishMarker(visualization_msgs::Marker& marker)
{
  // Add single marker to array
  markers_.markers.push_back(marker);

  return true;
}

bool MarkerVisualization::publishMarkers(visualization_msgs::MarkerArray& markers)
{
  if (pub_rviz_markers_ == nullptr)
  {  // always check this before publishing
    return false;
  }

  // Check if any actual markers exist to publish
  if (markers.markers.empty())
  {
    return false;
  }

  for (auto& marker : markers.markers)
  {
    double norm = 0;
    norm += marker.pose.orientation.w * marker.pose.orientation.w;
    norm += marker.pose.orientation.x * marker.pose.orientation.x;
    norm += marker.pose.orientation.y * marker.pose.orientation.y;
    norm += marker.pose.orientation.z * marker.pose.orientation.z;
    norm = std::sqrt(norm);
    if (norm < std::numeric_limits<double>::epsilon())
    {
      marker.pose.orientation.w = 1;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
    }
    else
    {
      marker.pose.orientation.w = marker.pose.orientation.w / norm;
      marker.pose.orientation.x = marker.pose.orientation.x / norm;
      marker.pose.orientation.y = marker.pose.orientation.y / norm;
      marker.pose.orientation.z = marker.pose.orientation.z / norm;
    }
  }
  // Publish
  pub_rviz_markers_.publish(markers);
  return true;
}

bool MarkerVisualization::publishArrow(const geometry_msgs::Pose& pose, std::size_t id){
  colors color = RED;
  double length = 0.1*XLARGE;
  scales scale = XLARGE;
  publishArrow(pose, color, scale, length, id);

  return true;
}

bool MarkerVisualization::publishArrow(const geometry_msgs::Pose& pose, colors color, scales scale, double length,
                                   std::size_t id)
{
  // Set the frame ID and timestamp.
  arrow_marker_.header.stamp = ros::Time::now();
  arrow_marker_.header.frame_id = global_frame_;

  if (id == 0)
  {
    arrow_marker_.id++;
  }
  else
  {
    arrow_marker_.id = id;
  }

  arrow_marker_.pose = pose;
  arrow_marker_.color = getColor(color);
  arrow_marker_.scale = getScale(scale);

  // override previous x scale specified
  if (length == 0)
  {  // auto set the scale
    arrow_marker_.scale.x *= 10.0;
  }
  else
  {
    arrow_marker_.scale.x = length;
  }

  // Helper for publishing rviz markers
  return publishMarker(arrow_marker_);
}

bool MarkerVisualization::publishLine(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2, colors color,
                                  scales scale)
{
  return publishLine(point1, point2, getColor(color), scale);
}

bool MarkerVisualization::publishLine(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2,
                                  const std_msgs::ColorRGBA& color, scales scale)
{
  return publishLine(point1, point2, color, getScale(scale));
}

bool MarkerVisualization::publishLine(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2,
                                  const std_msgs::ColorRGBA& color, const geometry_msgs::Vector3& scale)
{
  // Set the timestamp
  line_strip_marker_.header.stamp = ros::Time::now();

  line_strip_marker_.id++;
  line_strip_marker_.color = color;
  line_strip_marker_.scale = scale;
  line_strip_marker_.scale.y = 0;
  line_strip_marker_.scale.z = 0;

  line_strip_marker_.points.clear();
  line_strip_marker_.points.push_back(point1);
  line_strip_marker_.points.push_back(point2);

  // Helper for publishing rviz markers
  return publishMarker(line_strip_marker_);
}

bool MarkerVisualization::publishText(const geometry_msgs::Pose& pose, const std::string& text, colors color, scales scale,
                                  bool static_id)
{
  return publishText(pose, text, color, getScale(scale), static_id);
}

bool MarkerVisualization::publishText(const geometry_msgs::Pose& pose, const std::string& text, colors color,
                                  const geometry_msgs::Vector3 scale, bool static_id)
{
  // Save the ID if this is a static ID or keep incrementing ID if not static
  double temp_id = text_marker_.id;
  if (static_id)
  {
    text_marker_.id = 0;
  }
  else
  {
    text_marker_.id++;
  }

  text_marker_.header.stamp = ros::Time::now();
  text_marker_.header.frame_id = global_frame_;
  text_marker_.text = text;
  text_marker_.pose = pose;
  text_marker_.color = getColor(color);
  text_marker_.scale = scale;
  text_marker_.scale.x = 0;
  text_marker_.scale.y = 0;
  // Helper for publishing rviz markers
  publishMarker(text_marker_);

  // Restore the ID count if needed
  if (static_id)
  {
    text_marker_.id = temp_id;
  }

  return true;
}

bool MarkerVisualization::publishLines(const std::vector<geometry_msgs::Point>& aPoints,
                                   const std::vector<geometry_msgs::Point>& bPoints,
                                   const std::vector<std_msgs::ColorRGBA>& colors, const geometry_msgs::Vector3& scale)
{
  // Setup marker
  line_list_marker_.header.stamp = ros::Time();
  line_list_marker_.ns = "Line Array";

  // Provide a new id every call to this function
  line_list_marker_.id++;

  line_list_marker_.scale = scale;
  line_list_marker_.scale.z = 0;
  line_list_marker_.scale.y = 0;

  // line_list_marker_.color = getColor(BLUE); // This var is not used

  // Add each point pair to the line message
  line_list_marker_.points.clear();
  line_list_marker_.colors.clear();
  for (std::size_t i = 0; i < aPoints.size(); ++i)
  {
    line_list_marker_.points.push_back(aPoints[i]);
    line_list_marker_.points.push_back(bPoints[i]);
    line_list_marker_.colors.push_back(colors[i]);
    line_list_marker_.colors.push_back(colors[i]);
  }

  // Testing
  BOOST_ASSERT_MSG(line_list_marker_.colors.size() == line_list_marker_.points.size(), "Arrays mismatch in size");
  BOOST_ASSERT_MSG(line_list_marker_.colors.size() == aPoints.size() * 2, "Colors arrays mismatch in size");

  // Helper for publishing rviz markers
  return publishMarker(line_list_marker_);
}

bool MarkerVisualization::publishLineStrip(const std::vector<geometry_msgs::Point>& path, colors color, scales scale,
                                       const std::string& ns)
{
  if (path.size() < 2)
  {
    ROS_WARN_STREAM("Skipping path because " << path.size() << " points passed in.");
    return true;
  }

  line_strip_marker_.header.stamp = ros::Time();
  line_strip_marker_.ns = ns;

  // Provide a new id every call to this function
  line_strip_marker_.id++;

  std_msgs::ColorRGBA this_color = getColor(color);
  line_strip_marker_.scale = getScale(scale);
  line_strip_marker_.scale.z = 0;
  line_strip_marker_.scale.y = 0;
  line_strip_marker_.color = this_color;
  line_strip_marker_.points.clear();
  line_strip_marker_.colors.clear();

  for (std::size_t i = 1; i < path.size(); ++i)
  {
    // Add the point pair to the line message
    line_strip_marker_.points.push_back(path[i - 1]);
    line_strip_marker_.points.push_back(path[i]);
    line_strip_marker_.colors.push_back(this_color);
    line_strip_marker_.colors.push_back(this_color);
  }

  // Helper for publishing rviz markers
  return publishMarker(line_strip_marker_);
}

bool MarkerVisualization::publishPath(const nav_msgs::Path& path, colors color, scales scale,
                                       const uint& ns)
{
  if (path.poses.size() < 2)
  {
    ROS_WARN_STREAM("Skipping path because " << path.poses.size() << " points passed in.");
    return true;
  }

  line_strip_marker_.header.stamp = ros::Time();
  line_strip_marker_.ns = "path" + std::to_string(ns);

  // Provide a new id every call to this function
  line_strip_marker_.id = ns;

  std_msgs::ColorRGBA this_color = getColor(color);
  line_strip_marker_.scale = getScale(scale);
  line_strip_marker_.scale.z = 0;
  line_strip_marker_.scale.y = 0;
  line_strip_marker_.color = this_color;
  line_strip_marker_.points.clear();
  line_strip_marker_.colors.clear();

  for (std::size_t i = 1; i < path.poses.size(); ++i)
  {
    // Add the point pair to the line message
    line_strip_marker_.points.push_back(path.poses[i - 1].pose.position);
    line_strip_marker_.points.push_back(path.poses[i].pose.position);
    line_strip_marker_.colors.push_back(this_color);
    line_strip_marker_.colors.push_back(this_color);
  }

  // Helper for publishing rviz markers
  return publishMarker(line_strip_marker_);
}


