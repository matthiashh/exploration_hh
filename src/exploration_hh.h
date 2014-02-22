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
#include <visualization_msgs/Marker.h>              // to show state in rviz

namespace exploration_hh
{
  enum state
  {
    IDLE,
    EXPLORATION,
    FACE_RECOGNITION,
    CONFIRMATION,
    OBSTACLE,
    PANORAMA
  };

  enum goal_type
  {
    EXPLORATION_GOAL = 0,
    RECOGNITION_GOAL = 1,
    OBSTACLE_GOAL = 2
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
  ros::Publisher pubConfirmations_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* ac_;
  ros::Publisher pub_point_marker_;
  visualization_msgs::Marker point_marker_;
  ros::Publisher pub_text_marker_;
  visualization_msgs::Marker text_marker_;

  //own-variables
  std::vector<exploration_hh::ExplorationGoal> exploration_goals_;
  std::vector<exploration_hh::ExplorationGoal> recognition_goals_;
  std::vector<exploration_hh::ExplorationGoal> obstacle_goals_;
  std::vector<exploration_hh::ExplorationGoal> ordered_goals_;
  exploration_hh::ExplorationGoal current_goal_;
  move_base_msgs::MoveBaseGoal move_base_goal_;
  bool busyDriving;
  person_detector::DetectionObjectArray detections_;
  int goal_counter;
  std::string name;
  exploration_hh::state node_state;
  int speech_confirmation_id;


  //own-functions
  void explorationGoalCallback(const exploration_hh::ExplorationGoal received_goal);
  void detectionsCallback(const person_detector::DetectionObjectArray rec);
  bool calcGoals();
  void showGoals();
  bool processDetections();
  void setGoal_();
  int confirmation_();
  int recognitionGoal_();
  int explorationGoal_();
  int obstacleGoal_();
  int panorama_();
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
