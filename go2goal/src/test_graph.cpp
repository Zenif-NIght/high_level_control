#include "ros/ros.h"

#include "go2goal/topology_graph/topology_graph.h"
#include "geometry_msgs/Pose2D.h"

#include <string>
#include <sstream>

std::string outputAssignemnts(go2goal::TopologyGraph& graph)
{
  std::vector<uint8_t> values;
  graph.getAssignedIndices(values);
  std::stringstream assignemnts;
  for (std::vector<uint8_t>::iterator iter = values.begin(); iter != values.end(); ++iter)
  {
    assignemnts << static_cast<int>(*iter) << ",";
  }
  return assignemnts.str();
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "rand_goal_generator");
  ros::NodeHandle nh;

  // Create a graph
  go2goal::TopologyGraph graph("map");
  uint8_t prev_index = 0;

  ros::Rate loop_rate(1);
  ROS_INFO_STREAM("Index = " << static_cast<int>(prev_index));

  // Publish goal
  while (ros::ok())
  {
    uint8_t index = graph.getRandomNeighbor(prev_index);
    geometry_msgs::Pose2D point;
    graph.getIndexPoint(index, point);
    ROS_INFO_STREAM("Transition: " << static_cast<int>(prev_index) << "-->" << static_cast<int>(index)
                                   << " Position = (" << point.x << ", " << point.y
                                   << "). Assigned: " << outputAssignemnts(graph));
    prev_index = index;

    // Pause
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
