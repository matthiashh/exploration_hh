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
#include <person_detector/DetectionObjectArray.h>   // to store the detections



class Exploration
{
private:
  //ROS-stuff
  ros::NodeHandle n_;
  ros::Subscriber subExplorationGoals_;
  ros::ServiceClient clientMap_;
  ros::Subscriber sub_detections_;
  ros::ServiceClient confirmation_client_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* ac_;

  //own-variables
  std::vector<database_binding::explorationGoal> newGoals_;
  std::vector<database_binding::explorationGoal> orderedGoals_;
  move_base_msgs::MoveBaseGoal currentGoal_;
  bool busyDriving;
  person_detector::DetectionObjectArray detections_;

  //own-functions
  void explorationGoalCallback(const database_binding::explorationGoal received_goal);
  void detectionsCallback(const person_detector::DetectionObjectArray rec);
  bool calcExplorationGoals();
  void setNewGoal();
public:
    Exploration();
    int run();

//    // layer-stuff
//    virtual void onInitialize();
//    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
//    bool isDiscretized()
//    {
//      return true;
//    }
//    virtual void matchSize();
};

#endif // EXPLORATION_HH_H
