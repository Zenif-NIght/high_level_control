#ifndef __TOPOLOGY_AGENT__
#define __TOPOLOGY_AGENT__

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "go2goal/topology_graph/topology_graph.h"
#include "nav_msgs/Odometry.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace go2goal
{
class TopologyAgent
{
public:
  TopologyAgent(const std::string& odom_topic, const std::string& goal_topic, TopologyGraph& graph);
  TopologyAgent(const TopologyAgent& old);

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void generateNewGoal(bool force = false);
  void publishGoal();

private:
  // Topology graph variables
  TopologyGraph& graph;
  double dist_to_change;  // Once vehicle is within this distance then the goal will be updated

  // Agent state variables
  geometry_msgs::Pose2D current_goal;  // 2D goal
  geometry_msgs::PoseStamped goal;     // Goal message to be published
  uint8_t goal_index;
  nav_msgs::Odometry::ConstPtr odom;
  bool goal_calculated;

  // ROS variables
  ros::NodeHandle n;
  ros::Publisher pub_goal;
  ros::Subscriber sub_odom;
  tf::TransformListener tf_listener;

  // Topic variables
  const std::string goal_topic;
  const std::string odom_topic;

  void generateInitialGoal();
  bool get2DTransformedPose(geometry_msgs::Pose2D& pose);
  void generateGoalMessage();
};
}
#endif  // __TOPOLOGY_AGENT__
