#include <ros/ros.h>
#include "exploration_hh.h"
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/Led.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <turtlebot_msgs/TakePanorama.h>
#include <amcl/map/map.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration_hh_node");
  Exploration exploration_object;

  ROS_INFO("Finished initialization, now running in the loop");
  exploration_object.run();
  return 0;
}

Exploration::Exploration()
{
    ac_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base",true);
    subExplorationGoals_ = n_.subscribe("/database_binding/exploration_goals",10,&Exploration::explorationGoalCallback, this);
    busyDriving = false;
    //initialize currentGoal
    currentGoal_.target_pose.header.frame_id = "map";
    while(!ac_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
}

void Exploration::explorationGoalCallback(const database_binding::explorationGoal received_goal) {
  ROS_INFO("Received a exploration goal with x = %f and y = %f",received_goal.exploration_x,received_goal.exploration_y);
  newGoals_.push_back(received_goal);
  calcExplorationGoals();
}

bool Exploration::calcExplorationGoals()
{
  if (!newGoals_.empty())
  {
      orderedGoals_.push_back(newGoals_.front());
      //ROS_INFO("The first pushed prob is %f",orderedGoals_.back().exploration_prob);
      newGoals_.erase(newGoals_.begin());
  }
}

void Exploration::setNewGoal()
{
  if (!orderedGoals_.empty())
  {
    currentGoal_.target_pose.header.stamp = ros::Time::now();
    currentGoal_.target_pose.pose.position.x = orderedGoals_.front().exploration_x;
    currentGoal_.target_pose.pose.position.y = orderedGoals_.front().exploration_y;
    currentGoal_.target_pose.pose.orientation.w = 0.70710678;
    currentGoal_.target_pose.pose.orientation.z = -0.70710678;
    ROS_INFO("Sending goal with x = %f and y = %f",currentGoal_.target_pose.pose.position.x, currentGoal_.target_pose.pose.position.y);
    ac_->sendGoal(currentGoal_);
    orderedGoals_.erase(orderedGoals_.begin());
    busyDriving = true;
    ROS_INFO("Sent new goal to the actionserver");
  }
  else
  {
    busyDriving = false;
    ROS_INFO("No new goals to set");
  }
}

int Exploration::run()
{
  ros::Rate r(1);
  while (ros::ok())
  {
    //explorationGoal stuff
    if((!busyDriving && !orderedGoals_.empty()) || (busyDriving && (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)))
      {
        setNewGoal();
      }


    ros::spinOnce();
    r.sleep();
  }
    return 0;
}
