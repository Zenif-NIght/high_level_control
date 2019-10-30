#include "go2goal/topology_graph/topology_agent.h"

#include "geometry_msgs/PoseStamped.h"
#include <math.h>

using namespace go2goal;

TopologyAgent::TopologyAgent(const std::string & odom_topic, const std::string & goal_topic, TopologyGraph & graph):
    graph(graph), dist_to_change(0.25), goal_index(0), goal_calculated(false), odom_topic(odom_topic), goal_topic(goal_topic)
{
    // Initialize ROS Variables
    pub_goal = n.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);
    sub_odom = n.subscribe(odom_topic, 1, &TopologyAgent::odomCallback, this);
}

TopologyAgent::TopologyAgent(const TopologyAgent & old):
    graph(old.graph), dist_to_change(old.dist_to_change), goal_index(old.goal_index), goal_calculated(old.goal_calculated),
    current_goal(old.current_goal), goal(old.goal), odom_topic(old.odom_topic), goal_topic(old.goal_topic)
{
    // Initialize ROS Variables
    pub_goal = n.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);
    sub_odom = n.subscribe(odom_topic, 1, &TopologyAgent::odomCallback, this);
}


void TopologyAgent::odomCallback(const nav_msgs::Odometry::ConstPtr & msg){
    odom = msg;
    if(!goal_calculated)
        generateInitialGoal();
}

void TopologyAgent::generateNewGoal(bool force){
    // Only generate a goal if odometry received
    if(!odom || !goal_calculated)
        return;

    // Determine whether or not to get a new goal
    bool getNewGoal = force;
    
    if(!getNewGoal) { // Check distance to see if close enough to goal
        // Get the current position of the vehicle
        geometry_msgs::Pose2D pose;
        if(!get2DTransformedPose(pose)) {
            ROS_ERROR("TopologyAgent::generateNewGoal() Cannot calculate position");
            return;
        }

        // Calculate distance to the goal
        double dist;
        double del_x = pose.x - current_goal.x;
        double del_y = pose.y - current_goal.y;
        dist = std::sqrt(del_x*del_x + del_y*del_y);
        if(dist < dist_to_change)
            getNewGoal = true;
    }

    // Calculate the new goal
    if (getNewGoal) {
        //int goal_index_prev = static_cast<int>(goal_index);
        goal_index = graph.getRandomNeighbor(goal_index);
        graph.getIndexPoint(goal_index, current_goal);
        generateGoalMessage();

        //ROS_INFO_STREAM(odom_topic << "dist: " << dist << " Previous: " << goal_index_prev << " Next: " << static_cast<int>(goal_index));
    }
}

void TopologyAgent::publishGoal(){
    // Don't publish if no goal calculated
    if(!goal_calculated)
        return;

    // Create the goal to publish
    goal.header.stamp = odom->header.stamp;
    pub_goal.publish(goal);
}

void TopologyAgent::generateInitialGoal(){

    // Can only generate goal if odometry has been received
    if(!odom)
        return;

    // Get the closest index
    geometry_msgs::Pose2D pose;
    if(!get2DTransformedPose(pose))
        return;
    goal_index = graph.getClosestVertex(pose);

    // Get the goal position
    if(!graph.getIndexPoint(goal_index, current_goal)) {
        ROS_ERROR("TopologyAgent::generateInitialGoal() valid goal not received with goal index");
        return;
    }
    goal_calculated = true;
    generateGoalMessage();
}

bool TopologyAgent::get2DTransformedPose(geometry_msgs::Pose2D & pose){
    if(!odom)
        return false;

    // Initialize output 3D pose
    geometry_msgs::PoseStamped pose3d;
    pose3d.header.frame_id = graph.frame_id;
    pose3d.header.stamp = odom->header.stamp;

    // Initialize odom pose
    geometry_msgs::PoseStamped pose_odom;
    pose_odom.header.frame_id = odom->header.frame_id;
    pose_odom.header.stamp = odom->header.stamp;
    pose_odom.pose.orientation = odom->pose.pose.orientation;
    pose_odom.pose.position = odom->pose.pose.position;

    // Tranform odom to the correct frame
    try {
        tf_listener.waitForTransform(graph.frame_id,odom->header.frame_id, odom->header.stamp,ros::Duration(0.5));
        tf_listener.transformPose(graph.frame_id, pose_odom, pose3d);
    }
    catch(const tf::TransformException &ex) {
        ROS_ERROR("TopologyAgent::get2DTransformedPose() %s", ex.what());
        return false;
    }

    // Extract 3D pose
    pose.theta = tf::getYaw(pose3d.pose.orientation);
    pose.x = pose3d.pose.position.x;
    pose.y = pose3d.pose.position.y;
    return true;

}

void TopologyAgent::generateGoalMessage(){
    if(!goal_calculated || !odom)
        return;

    goal.header.frame_id = graph.frame_id;
    goal.header.stamp = odom->header.stamp;
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(current_goal.theta);
    goal.pose.position.x = current_goal.x;
    goal.pose.position.y = current_goal.y;
    goal.pose.position.z = 0.0;
}
