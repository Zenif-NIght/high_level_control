#ifndef __TOPOLOGY_GRAPH__
#define __TOPOLOGY_GRAPH__

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

#include <random>
#include <map>
#include <vector>
#include <set>

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
    uint8_t getRandomNeighbor(uint8_t start_index, bool adjust_assignments = true);

    /**
     * @brief getClosestVertex
     * @param position The position to compare against (assumed to be in frame_id frame)
     * @return index of the closes vertex
     */
    uint8_t getClosestVertex(const geometry_msgs::Pose2D & position, bool adjust_assignments = true);

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

    /**
     * @brief getAssignedIndices
     * @param indices Vector which will contain the assigned indices. Contents will be cleared prior to adding data
     */
    void getAssignedIndices(std::vector<uint8_t> & indices);


private:
    // ROS variables
    ros::NodeHandle n;
    ros::Publisher pub_visualization;

    // Graph variables
    std::map<uint8_t, std::vector<uint8_t>> index_to_neighbors;
    std::map<uint8_t, geometry_msgs::Pose2D> index_to_point;

    // Assignment variables
    std::set<uint8_t> assigned_verticies;

    // Random number generation
    std::default_random_engine generator;
    uint8_t calculateRandomNumber(uint8_t min_val, uint8_t max_val);

    // Index functions
    bool isValidIndex(uint8_t index);
    void addVertexPoint(uint8_t index, double x, double y);
    void addNeighborSet(uint8_t index, std::vector<uint8_t> & neighbors);
    bool verifyIndexCorrelation();
};

}
#endif // __TOPOLOGY_GRAPH__
