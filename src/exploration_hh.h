#ifndef EXPLORATION_HH_H
#define EXPLORATION_HH_H

#include <ros/ros.h>                                // needed for general ROS-support
#include <vector>                                   // needed to store the goals
#include <kobuki_msgs/ButtonEvent.h>                //TODO depricated?
#include <kobuki_msgs/Led.h>                        //TODO  --
#include <std_srvs/Empty.h>                         //TODO  --
#include <std_msgs/Empty.h>                         //TODO  --
#include <exploration_hh/ExplorationGoal.h>         // exploration messagetype
#include <move_base_msgs/MoveBaseAction.h>          // to make a move-base-client
#include <actionlib/client/simple_action_client.h>  //  -- 
#include <person_detector/DetectionObjectArray.h>   // to store the detections

namespace exploration_hh
{
  enum state
  {
    IDLE,
    EXPLORATION_GOAL,
    FACE_RECOGNITION,
    CONFIRMATION,
    OBSTACLE_GOAL
  };
}

class Exploration
{
private:
  //ROS-stuff
  ros::NodeHandle n_;
  ros::Subscriber subExplorationGoals_;
  ros::ServiceClient clientMap_;
  ros::Subscriber sub_detections_;
  ros::ServiceClient confirmation_client_;
  ros::Publisher pub_speech_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* ac_;

  //own-variables
  std::vector<exploration_hh::ExplorationGoal> exploration_goals_;
  std::vector<exploration_hh::ExplorationGoal> recognition_goals_;
  std::vector<exploration_hh::ExplorationGoal> obstacle_goals_;
  std::vector<exploration_hh::ExplorationGoal> ordered_goals_;
  move_base_msgs::MoveBaseGoal currentGoal_;
  bool busyDriving;
  person_detector::DetectionObjectArray detections_;
  int last_request_id;
  ros::Time last_request_time;
  std::string name;
  exploration_hh::state node_state;

  //own-functions
  void explorationGoalCallback(const exploration_hh::ExplorationGoal received_goal);
  void detectionsCallback(const person_detector::DetectionObjectArray rec);
  bool calcExplorationGoals();
  bool processDetections();
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
