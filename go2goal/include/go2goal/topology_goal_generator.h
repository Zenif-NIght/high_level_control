#ifndef __TOPOLOGY_GOAL_GENERATOR__
#define __TOPOLOGY_GOAL_GENERATOR__

#include "ros/ros.h"
#include "go2goal/topology_graph/topology_graph.h"
#include "go2goal/topology_graph/topology_agent.h"

#include <set>
#include <vector>

namespace go2goal
{
class TopologyGoalGenerator
{
public:
  TopologyGoalGenerator(const std::string& frame_id, const std::string& odom_base, const std::string& goal_base,
                        const std::string& topic_base, double dist_to_change);

  void processTopics();
  void processAgentGoals();

  const std::string frame_id;
  const std::string odom_base;
  const std::string topic_base;
  const std::string goal_base;

private:
  // Topology graph variables
  TopologyGraph graph;
  double dist_to_change;  // Once vehicle is within this distance then the goal will be updated

  // ROS variables
  ros::NodeHandle n;

  // Namespace variables
  std::set<std::string> namespace_set;
  std::vector<TopologyAgent> agent_controllers;

  /**
   * @brief extractTopicNamespace
   * @param topic_input any ros topic
   * @param namespace_out the namespace of the topic which begins with topic_base
   * @return true if topic_base was found within the input topic
   */
  bool extractTopicNamespace(const std::string& topic_input, std::string& namespace_out);
};
}
#endif  // __TOPOLOGY_GOAL_GENERATOR__
