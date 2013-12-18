#ifndef EXPLORATION_HH_H
#define EXPLORATION_HH_H
#include <ros/ros.h>
#include <vector>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/Led.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <database_binding/explorationGoal.h>       //exploration messagetype
#include <move_base_msgs/MoveBaseAction.h>          //to make a move-base-client
#include <actionlib/client/simple_action_client.h>

class Exploration
{
private:
  //ROS-stuff
  ros::NodeHandle n_;
  ros::Subscriber subExplorationGoals_;
  ros::ServiceClient clientMap_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* ac_;

  //own-variables
  std::vector<database_binding::explorationGoal> newGoals_;
  std::vector<database_binding::explorationGoal> orderedGoals_;
  move_base_msgs::MoveBaseGoal currentGoal_;
  bool busyDriving;

  //own-functions
  void explorationGoalCallback(const database_binding::explorationGoal received_goal);
  bool calcExplorationGoals();
  void setNewGoal();
public:
    Exploration();
    int run();


};

#endif // EXPLORATION_HH_H
