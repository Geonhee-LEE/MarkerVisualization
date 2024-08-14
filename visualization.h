#pragma once


#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

// C++
#include <random>
#include <chrono>


enum colors
{
  BLACK = 0,
  BROWN = 1,
  BLUE = 2,
  CYAN = 3,
  GREY = 4,
  DARK_GREY = 5,
  GREEN = 6,
  LIME_GREEN = 7,
  MAGENTA = 8,
  ORANGE = 9,
  PURPLE = 10,
  RED = 11,
  PINK = 12,
  WHITE = 13,
  YELLOW = 14,
  TRANSLUCENT = 15,
  TRANSLUCENT_LIGHT = 16,
  TRANSLUCENT_DARK = 17,
  RAND = 18,
  CLEAR = 19,
  DEFAULT = 20  // i.e. 'do not change default color'
};

enum scales
{
  XXXXSMALL = 1,
  XXXSMALL = 2,
  XXSMALL = 3,
  XSMALL = 4,
  SMALL = 5,
  MEDIUM = 6,  // same as REGULAR
  LARGE = 7,
  XLARGE = 8,
  XXLARGE = 9,
  XXXLARGE = 10,
  XXXXLARGE = 11,
};

/*!
 * Class MarkerVisualization.
 */
class MarkerVisualization
{
 public:
  /*!
   * Constructor.
   */
  MarkerVisualization(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  ~MarkerVisualization();

  bool trigger();
  bool deleteAllMarkers();
  
  /**
   * \brief Pre-load rviz markers for better efficiency
   * \return converted pose   * \return true on sucess
   */
  bool loadRvizMarkers();

  /**
   * \brief Display a visualization_msgs Marker of a custom type. Allows reuse of the ros publisher
   * \param marker - a pre-made marker ready to be published
   * \return true on success
   */
  bool publishMarker(visualization_msgs::Marker& marker);
  /**
   * \brief Display an array of markers, allows reuse of the ROS publisher
   * \param markers
   * \return true on success
   */
  bool publishMarkers(visualization_msgs::MarkerArray& markers);
  bool publishArrow(const geometry_msgs::Pose& pose, std::size_t id);
  bool publishArrow(const geometry_msgs::Pose& pose, colors color, scales scale, double length,
                                   std::size_t id);
  bool publishLine(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2,
                                  const std_msgs::ColorRGBA& color, const geometry_msgs::Vector3& scale);
  bool publishLine(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2, colors color,
                                  scales scale);
  bool publishLine(const geometry_msgs::Point& point1, const geometry_msgs::Point& point2,
                                  const std_msgs::ColorRGBA& color, scales scale);                                  
  bool publishText(const geometry_msgs::Pose& pose, const std::string& text, colors color = WHITE,
                   scales scale = MEDIUM, bool static_id = true);
  bool publishText(const geometry_msgs::Pose& pose, const std::string& text, colors color,
                   const geometry_msgs::Vector3 scale, bool static_id = true);

  bool publishLines(const std::vector<geometry_msgs::Point>& aPoints,
                    const std::vector<geometry_msgs::Point>& bPoints,
                    const std::vector<std_msgs::ColorRGBA>& colors, const geometry_msgs::Vector3& scale);
  bool publishLineStrip(const std::vector<geometry_msgs::Point>& path, colors color, scales scale,
                                       const std::string& ns);
  bool publishPath(const nav_msgs::Path& path, colors color, scales scale,
                                       const uint& ns);

  geometry_msgs::Pose getPoseMsg(double x, double y, double theta);
  geometry_msgs::Pose getPoseMsg(double x, double y, geometry_msgs::Quaternion orientation);

  /**
   * \brief Create a pose of position (0,0,0) and quaternion (0,0,0,1)
   * \param Pose to fill in
   */
  static geometry_msgs::Pose getIdentityPose();


  visualization_msgs::MarkerArray getMarkerArray() const{
    return marker_arrays_;
  }

  std_msgs::ColorRGBA getColor(colors color) const;
  geometry_msgs::Vector3 getScale(scales scale, double marker_scale = 1.0) const;

  double getGlobalScale() const
  {
    return global_scale_;
  }
  /**
   * \brief Setter for the global scale used for changing size of all markers
   */
  void setGlobalScale(double global_scale)
  {
    global_scale_ = global_scale;
  }

  /**
   * \brief Create a random color that is not too light
   * \return the RGB message of a random color
   */
  std_msgs::ColorRGBA createRandColor() const;

  /**
   * \brief Set seed for random methods above
   *
   * Random sampling uses std::mt19937 internally
   */
  static void setRandomSeed(unsigned int seed);
  /**
   * \brief Get random between min and max
   */
  static std::mt19937 mt_random_engine_;
  static double dRand(double min, double max);
  static float fRand(float min, float max);
  static int iRand(int min, int max);

 private:
  ros::NodeHandle nh_;
  std::string global_frame_;
  std::string marker_topic_;  // topic to publish to rviz
  ros::Publisher pub_rviz_markers_;  // for rviz visualization markers

  // Cached Rviz Marker Array
  visualization_msgs::MarkerArray markers_;
  // Cached Rviz markers
  visualization_msgs::Marker arrow_marker_;
  visualization_msgs::Marker sphere_marker_;
  visualization_msgs::Marker block_marker_;
  visualization_msgs::Marker cylinder_marker_;
  visualization_msgs::Marker mesh_marker_;
  visualization_msgs::Marker text_marker_;
  visualization_msgs::Marker cuboid_marker_;
  visualization_msgs::Marker line_strip_marker_;
  visualization_msgs::Marker line_list_marker_;
  visualization_msgs::Marker spheres_marker_;
  visualization_msgs::Marker reset_marker_;
  visualization_msgs::Marker triangle_marker_;

  double alpha_;         // opacity of all markers
  double global_scale_;  // allow all markers to be increased by a constanct factor
  visualization_msgs::MarkerArray marker_arrays_;

  // Duration to have Rviz markers persist, 0 for infinity
  ros::Duration marker_lifetime_;

};
