#ifndef EXPLORATION_HH_H
#define EXPLORATION_HH_H

#include <ros/ros.h>                                // needed for general ROS-support
#include <vector>                                   // needed to store the goals
#include <kobuki_msgs/ButtonEvent.h>                //TODO depricated?
#include <kobuki_msgs/Led.h>                        //TODO  --
#include <std_srvs/Empty.h>                         //TODO  --
#include <std_msgs/Empty.h>                         //TODO  --
#include <database_binding/explorationGoal.h>       // exploration messagetype
#include <move_base_msgs/MoveBaseAction.h>          // to make a move-base-client
#include <actionlib/client/simple_action_client.h>  //  -- 

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
