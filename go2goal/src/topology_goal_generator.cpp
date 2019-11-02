#include "go2goal/topology_goal_generator.h"
#include "go2goal/topology_graph/topology_agent.h"
#include "go2goal/topology_graph/topology_graph.h"

using namespace go2goal;

TopologyGoalGenerator::TopologyGoalGenerator(const std::string& frame_id, const std::string& odom_base,
                                             const std::string& goal_base, const std::string& topic_base,
                                             double dist_to_change)
  : frame_id(frame_id)
  , odom_base(odom_base)
  , topic_base(topic_base)
  , goal_base(goal_base)
  , graph(frame_id)
  , dist_to_change(dist_to_change)
{
}

void TopologyGoalGenerator::processTopics()
{
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  std::string output = "\n";
  for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); ++it)
  {
    const ros::master::TopicInfo& info = *it;
    std::string namespace_in = "";

    if (extractTopicNamespace(info.name, namespace_in))
    {
      // Check to see if the namespace set already has a given namespace
      if (namespace_set.find(namespace_in) == namespace_set.end())
      {
        // Insert the namespace into the namespace set
        namespace_set.insert(namespace_in);
        output += "topic: " + info.name + ", valid namespace = " + namespace_in + "\n";

        // Create variables needed to track the given agent
        std::string odom_topic = "/" + namespace_in + odom_base;
        std::string goal_topic = "/" + namespace_in + goal_base;
        agent_controllers.emplace_back(odom_topic, goal_topic, graph);
      }
    }
    else
    {
      output += "topic: " + info.name + "\n";
    }
  }
  // ROS_INFO_STREAM("TopologyGoalGenerator::processTopics() topics: \n" << output);
}

bool TopologyGoalGenerator::extractTopicNamespace(const std::string& topic_input, std::string& namespace_out)
{
  // Find first and second forward slashes
  size_t first_slash = topic_input.find('/');
  if (first_slash == std::string::npos)
    return false;
  size_t second_slash = topic_input.find('/', first_slash + 1);
  if (second_slash == std::string::npos)
    return false;

  // Find the topic base
  size_t topic_base_pos = topic_input.find(topic_base, first_slash + 1);

  // Check validity of the topic base position
  if (topic_base_pos == std::string::npos || topic_base_pos > second_slash)
    return false;

  // Extract the full namespace
  namespace_out = topic_input.substr(topic_base_pos, second_slash - topic_base_pos + 1);
  return true;
}

void TopologyGoalGenerator::processAgentGoals()
{
  // Loop through each agent controller
  for (std::vector<TopologyAgent>::iterator iter = agent_controllers.begin(); iter != agent_controllers.end(); ++iter)
  {
    // Create new goals
    iter->generateNewGoal();

    // Publish commands
    iter->publishGoal();
  }

  // Publish visualization
  graph.publishGraph();
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "topology_goal_generator");
  ros::NodeHandle nh;

  // Create inputs for goal generator
  std::string frame_id = "/map";
  std::string odom_base = "odom";
  std::string goal_base = "goal";
  std::string topic_base = "robot";
  double dist_to_change = 0.25;

  // Create goal generator
  TopologyGoalGenerator generator(frame_id, odom_base, goal_base, topic_base, dist_to_change);

  ros::Rate loop_rate(1);

  // Publish goal
  while (ros::ok())
  {
    generator.processTopics();
    generator.processAgentGoals();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
