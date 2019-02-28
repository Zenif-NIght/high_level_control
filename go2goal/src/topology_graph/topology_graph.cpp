#include "go2goal/topology_graph/topology_graph.h"
#include <stdexcept>

using namespace go2goal;

TopologyGraph::TopologyGraph(const std::string & frame_id): frame_id(frame_id){
    // *******  Create Hard coded
}

uint8_t TopologyGraph::getRandomNeighbor(uint8_t start_index){
    // Should only be called for valid indices
    if(!isValidIndex(start_index))
        throw std::invalid_argument("TopologyGraph::getRandomNeighbor() Received index out of bounds");
    
    // Get an iterator to the starting index
    std::map<uint8_t, std::vector<uint8_t>>::iterator element = index_to_point.find(start_index);
    
    // Get a new index
    u_int size = element->second.size();
    uint8_t new_index = start_index; // Default is to just return the start index if another index cannot be found
    if(size > 0) {
        new_index = calculateRandomNumber(0, static_cast<u_int8_t>(size-1));
    }

    // Return the new index
    return new_index;
}

bool TopologyGraph::getIndexPoint(uint8_t index, geometry_msgs::Pose2D & vertex_point){
    if(!isValidIndex(index))
        return false;

    vertex_point = index_to_point.find(index)->second;
    return true;
}

void TopologyGraph::publishGraph(){

}

uint8_t TopologyGraph::calculateRandomNumber(uint8_t min_val, uint8_t max_val){
    std::uniform_int_distribution<uint8_t> distribution(min_val,max_val);
    return static_cast<uint8_t>(distribution(generator));
}

bool TopologyGraph::isValidIndex(uint8_t index){
    if(index_to_point.find(index) == index_to_point.end())
        return false;
    return true;
}
