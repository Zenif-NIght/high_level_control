#ifndef __RAND_GOAL_GENERATOR__
#define __RAND_GOAL_GENERATOR__

#include "ros/ros.h"
#include <random>

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace go2goal
{

class RandGoalGenerator
{
public:
    RandGoalGenerator();

public:

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr & msg);

    void publishGoal();
    bool calculateGoal();

private:
    // ROS variables
    ros::NodeHandle n;
    ros::Publisher pub_goal;
    ros::Subscriber sub_odom;
    tf::TransformListener tf_listener;

    // Message storage variables
    geometry_msgs::PoseStamped goal;
    nav_msgs::Odometry::ConstPtr odom;

    // Nominal command values
    double x_max;   // x must be in [-x_max x_max]
    double y_max;   // y must be in [-y_max y_max]
    double z_max;   // z must be in [-z_max z_max]
    double dist_to_change; // Once vehicle is within this distance then the goal will be updated

    // Random number generation
    std::default_random_engine generator;
    void generateNewGoal();
    double calculateRandomNumber(double min_val, double max_val);
};

}
#endif // __RAND_GOAL_GENERATOR__
