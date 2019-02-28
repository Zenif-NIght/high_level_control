#ifndef __TOPOLOGY_GRAPH__
#define __TOPOLOGY_GRAPH__

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

#include <random>
#include <map>
#include <vector>



namespace go2goal
{

class TopologyGraph
{
public:
    TopologyGraph(const std::string & frame_id);

public:

    /**
     * @brief getRandomNeighbor
     * @param start_index current index in the graph
     * @return An index in the graph with an edge connected to the start index
     */
    uint8_t getRandomNeighbor(uint8_t start_index);

    /**
     * @brief getIndexPoint
     * @param index The index of the vertex in question
     * @param vertex_point The 2D point corresponding to the input index
     * @return true if successful, false otherwise
     */
    bool getIndexPoint(uint8_t index, geometry_msgs::Pose2D & vertex_point);

    /**
     * @brief frame_id ID for the frame that the points are represented in
     */
    const std::string frame_id;

    /**
     * @brief publishGraph Publishes the visualization of the graph
     */
    void publishGraph();


private:
    // ROS variables
    ros::NodeHandle n;
    ros::Publisher pub_visualization;

    // Graph variables
    std::map<uint8_t, std::vector<uint8_t>> index_to_neighbors;
    std::map<uint8_t, geometry_msgs::Pose2D> index_to_point;

    // Random number generation
    std::default_random_engine generator;
    uint8_t calculateRandomNumber(uint8_t min_val, uint8_t max_val);
    bool isValidIndex(uint8_t index);
};

}
#endif // __TOPOLOGY_GRAPH__
