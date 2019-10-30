#include "go2goal/rand_goal_generator.h"
#include <math.h>

using namespace go2goal;

RandGoalGenerator::RandGoalGenerator() : x_max(5.0), y_max(5.0), z_max(0.0), dist_to_change(0.1)
{
  // Read in parameters for goal generation
  ros::NodeHandle nh("~");
  nh.getParam("x_max", x_max);
  nh.getParam("y_max", y_max);
  nh.getParam("z_max", z_max);
  nh.getParam("dist_to_change", dist_to_change);

  // Adjust input values to be positive
  x_max = std::abs(x_max);
  y_max = std::abs(y_max);
  z_max = std::abs(z_max);
  dist_to_change = std::abs(dist_to_change);

  // Initialize the goal
  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = 0.0;
  goal.pose.position.y = 0.0;
  goal.pose.position.z = 0.0;
  goal.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  generateNewGoal();

  // Initialize ROS variables
  pub_goal = n.advertise<geometry_msgs::PoseStamped>("goal", 1);
  sub_odom = n.subscribe("odom", 1, &RandGoalGenerator::odomCallback, this);
}

void RandGoalGenerator::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom = msg;
}

void RandGoalGenerator::publishGoal()
{
  if (!odom)
    return;

  // Update the goal to have the latest odom header
  goal.header.frame_id = odom->header.frame_id;
  goal.header.stamp = odom->header.stamp;

  // Publish the goal
  pub_goal.publish(goal);
}

/**
 * @brief RandGoalGenerator::calculateGoal
 * @return true if a new goal has been calculated, false otherwise
 */
bool RandGoalGenerator::calculateGoal()
{
  if (!odom)
    return false;

  // Calculate the distance to the goal
  double del_x = goal.pose.position.x - odom->pose.pose.position.x;
  double del_y = goal.pose.position.y - odom->pose.pose.position.y;
  double del_z = goal.pose.position.z - odom->pose.pose.position.z;
  double dist = std::sqrt(del_x * del_x + del_y * del_y + del_z + del_z);

  // Update the goal if the vehicle is close to the goal
  if (dist <= dist_to_change)
  {
    generateNewGoal();
    return true;
  }
  return false;
}

double RandGoalGenerator::calculateRandomNumber(double min_val, double max_val)
{
  std::uniform_real_distribution<double> distribution(min_val, max_val);
  double number = distribution(generator);
  return number;
}

void RandGoalGenerator::generateNewGoal()
{
  goal.pose.position.x = calculateRandomNumber(-x_max, x_max);
  goal.pose.position.y = calculateRandomNumber(-y_max, y_max);
  goal.pose.position.z = calculateRandomNumber(-z_max, z_max);
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "rand_goal_generator");
  ros::NodeHandle nh;

  // Create goal generator
  RandGoalGenerator generator;

  ros::Rate loop_rate(1);

  // Publish goal
  while (ros::ok())
  {
    generator.calculateGoal();
    generator.publishGoal();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
