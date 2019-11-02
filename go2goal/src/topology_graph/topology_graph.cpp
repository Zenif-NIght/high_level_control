#include "go2goal/topology_graph/topology_graph.h"
#include <stdexcept>
#include <float.h>
#include <math.h>

#include <tf/tf.h>

using namespace go2goal;

TopologyGraph::TopologyGraph(const std::string& frame_id) : frame_id(frame_id)
{
  /// TODO: Read topology graph in from file
  // *******  Create Hard coded pose information *************** //
  addVertexPoint(0, -5, 5);   // Vertex 0 // Top Row
  addVertexPoint(1, 0, 5);    // Vertex 1
  addVertexPoint(2, 5, 5);    // Vertex 2
  addVertexPoint(3, -5, 0);   // Vertex 3 // Middle Row
  addVertexPoint(4, 0, 0);    // Vertex 4
  addVertexPoint(5, 5, 0);    // Vertex 5
  addVertexPoint(6, -5, -5);  // Vertex 6 // Bottom Row
  addVertexPoint(7, 0, -5);   // Vertex 7
  addVertexPoint(8, 5, -5);   // Vertex 8

  // Create neighborhoods
  index_to_neighbors.emplace(std::pair<uint8_t, std::vector<uint8_t>>(0, { 1, 3 }));        // Vertex 0 -- Top Row
  index_to_neighbors.emplace(std::pair<uint8_t, std::vector<uint8_t>>(1, { 0, 2, 4 }));     // Vertex 1
  index_to_neighbors.emplace(std::pair<uint8_t, std::vector<uint8_t>>(2, { 1, 5 }));        // Vertex 2
  index_to_neighbors.emplace(std::pair<uint8_t, std::vector<uint8_t>>(3, { 0, 4, 6 }));     // Vertex 3 -- Middle Row
  index_to_neighbors.emplace(std::pair<uint8_t, std::vector<uint8_t>>(4, { 1, 3, 5, 7 }));  // Vertex 4
  index_to_neighbors.emplace(std::pair<uint8_t, std::vector<uint8_t>>(5, { 2, 4, 8 }));     // Vertex 5
  index_to_neighbors.emplace(std::pair<uint8_t, std::vector<uint8_t>>(6, { 3, 7 }));        // Vertex 6 -- Bottom Row
  index_to_neighbors.emplace(std::pair<uint8_t, std::vector<uint8_t>>(7, { 4, 6, 8 }));     // Vertex 7
  index_to_neighbors.emplace(std::pair<uint8_t, std::vector<uint8_t>>(8, { 5, 7 }));        // Vertex 8

  // Check to ensure graph formed properly
  if (!verifyIndexCorrelation())
  {
    throw std::invalid_argument("TopologyGraph::TopologyGraph() Graph not initialized properly");
  }

  // ************ Create visualization **************** //
  pub_visualization = n.advertise<visualization_msgs::MarkerArray>("topology_graph", 1);
  createInitialVisualizationMessages();
}

void TopologyGraph::createInitialVisualizationMessages()
{
  // ********* Visualize the vertices ******** //
  // Create a visualization marker
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = frame_id;
  marker.ns = "topology_graph";
  marker.type = marker.CYLINDER;
  marker.action = marker.ADD;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.25;
  marker.color.r = marker.color.g = 0.0;
  marker.color.b = 255;
  marker.color.a = 1;
  marker.lifetime = ros::Duration(0);
  marker.frame_locked = true;

  // Initialize the marker pose
  marker.pose.position.z = -.25;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // Loop through vertices and add the marker (update id, pose)
  int marker_id = 0;
  for (std::map<uint8_t, geometry_msgs::Pose2D>::iterator iter = index_to_point.begin(); iter != index_to_point.end();
       ++iter, ++marker_id)
  {
    // Update the pose
    marker.pose.position.x = iter->second.x;
    marker.pose.position.y = iter->second.y;

    // Update and store the id
    marker.id = marker_id;
    index_to_marker_id[iter->first] = marker_id;

    // Add the marker to the marker array
    markers.markers.push_back(marker);
  }

  // ********* Visualize the Lines ******** //
  // Initialize the marker for drawing lines
  marker.scale.x = 0.1;
  marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.0;
  marker.type = marker.LINE_STRIP;

  // Loop through all points
  for (std::map<uint8_t, geometry_msgs::Pose2D>::iterator iter = index_to_point.begin(); iter != index_to_point.end();
       ++iter)
  {
    // Create the first point
    geometry_msgs::Point pnt1;
    pnt1.x = iter->second.x;
    pnt1.y = iter->second.y;
    pnt1.z = -.25;

    // Loop through all neighbors to get the other points
    for (std::vector<uint8_t>::iterator it_pnts = index_to_neighbors[iter->first].begin();
         it_pnts != index_to_neighbors[iter->first].end(); ++it_pnts, ++marker_id)
    {
      // Get the vector pointed to by the iterator
      geometry_msgs::Point pnt2;
      pnt2.x = index_to_point[*it_pnts].x;
      pnt2.y = index_to_point[*it_pnts].y;
      pnt2.z = -0.25;

      // Add Points to the marker
      marker.points.clear();
      marker.points.push_back(pnt1);
      marker.points.push_back(pnt2);

      // Add the marker
      marker.id = marker_id;
      markers.markers.push_back(marker);
    }
  }
}

uint8_t TopologyGraph::getRandomNeighbor(uint8_t start_index, bool adjust_assignments)
{
  // Should only be called for valid indices
  if (!isValidIndex(start_index))
    throw std::invalid_argument("TopologyGraph::getRandomNeighbor() Received index out of bounds");

  // Get an iterator to the starting index
  std::map<uint8_t, std::vector<uint8_t>>::iterator element = index_to_neighbors.find(start_index);

  // Get a new index
  size_t size = element->second.size();
  uint8_t new_index = start_index;  // Default is to just return the start index if another index cannot be found
  if (size > 0)
  {
    new_index = calculateRandomNumber(0, static_cast<u_int8_t>(size - 1));
    new_index = element->second[new_index];
  }

  // Update the assignments
  if (adjust_assignments)
  {
    std::set<uint8_t>::iterator start_iter = assigned_verticies.find(start_index);  // Remove the start_index element
    if (start_iter != assigned_verticies.end())
    {
      assigned_verticies.erase(start_iter);
    }
    assigned_verticies.insert(new_index);  // Add the new index to the assigned vertex list
  }

  // Return the new index
  return new_index;
}

uint8_t TopologyGraph::getClosestVertex(const geometry_msgs::Pose2D& position, bool adjust_assignments)
{
  double dist = std::numeric_limits<double>::max();
  uint8_t index_closest = 0;

  // Loop through the possible vertex points to get the closest points
  for (std::map<uint8_t, geometry_msgs::Pose2D>::iterator iter = index_to_point.begin(); iter != index_to_point.end();
       ++iter)
  {
    double del_x = position.x - iter->second.x;
    double del_y = position.y - iter->second.y;
    double dist_tmp = std::sqrt(del_x * del_x + del_y * del_y);

    if (dist_tmp < dist)
    {
      dist = dist_tmp;
      index_closest = iter->first;
    }
  }

  // Update the assignment
  if (adjust_assignments)
    assigned_verticies.insert(index_closest);

  return index_closest;
}

bool TopologyGraph::getIndexPoint(uint8_t index, geometry_msgs::Pose2D& vertex_point)
{
  if (!isValidIndex(index))
    return false;

  vertex_point = index_to_point.find(index)->second;
  return true;
}

void TopologyGraph::publishGraph()
{
  // Update colors on the vertex markers
  for (std::map<uint8_t, geometry_msgs::Pose2D>::iterator iter = index_to_point.begin(); iter != index_to_point.end();
       ++iter)
  {
    // Get the marker id index
    size_t id = static_cast<size_t>(index_to_marker_id[iter->first]);

    // Check to see if the index is in the assigned verties
    if (assigned_verticies.find(iter->first) == assigned_verticies.end())
    {  // Not found
      markers.markers[id].color.b = 255;
      markers.markers[id].color.r = 0.0;
    }
    else
    {  // vertex assigned
      markers.markers[id].color.b = 0.0;
      markers.markers[id].color.r = 255;
    }
  }

  pub_visualization.publish(markers);
}

uint8_t TopologyGraph::calculateRandomNumber(uint8_t min_val, uint8_t max_val)
{
  std::uniform_int_distribution<uint8_t> distribution(min_val, max_val);
  return static_cast<uint8_t>(distribution(generator));
}

bool TopologyGraph::isValidIndex(uint8_t index)
{
  if (index_to_point.find(index) == index_to_point.end())
    return false;
  return true;
}

void TopologyGraph::addVertexPoint(uint8_t index, double x, double y)
{
  geometry_msgs::Pose2D loc;
  loc.theta = 0;
  loc.x = x;
  loc.y = y;
  index_to_point[index] = loc;
}

bool TopologyGraph::verifyIndexCorrelation()
{
  // Check the sizes of the maps
  if (index_to_neighbors.size() != index_to_point.size())
    return false;

  // Loop through each agent to get the neighborhood
  for (std::map<uint8_t, std::vector<uint8_t>>::iterator iter = index_to_neighbors.begin();
       iter != index_to_neighbors.end(); ++iter)
  {
    // Check to see if the key is also in the index_to_point map
    if (!isValidIndex(iter->first))
      return false;

    // Iterate through all indices of the neighbors to check for valid neighbors
    for (std::vector<uint8_t>::iterator it_neigh = iter->second.begin(); it_neigh != iter->second.end(); ++it_neigh)
    {
      // Check to see if the neighbor is a key in index_to_point map
      if (!isValidIndex(*it_neigh))
        return false;
    }
  }
  return true;
}

void TopologyGraph::getAssignedIndices(std::vector<uint8_t>& indices)
{
  indices.clear();
  for (std::set<uint8_t>::iterator iter = assigned_verticies.begin(); iter != assigned_verticies.end(); ++iter)
  {
    indices.push_back(*iter);
  }
}
