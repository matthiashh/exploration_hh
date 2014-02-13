#include <ros/ros.h>                          // Needed for general ROS-support
#include "exploration_hh.h"                   // Class header
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/Led.h>
#include <std_srvs/Empty.h>                   // Needed to send calls
#include <std_msgs/Empty.h>                   //  --
#include <turtlebot_msgs/TakePanorama.h>      // TODO Remove?
#include <amcl/map/map.h>
#include <ros/console.h>                      // To perform debug-outputs
#include <human_interface/RecognitionConfirmation.h>  // for confirmation of a person




int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration_hh_node");
  Exploration exploration_object;

  ROS_INFO("Finished initialization, now running in the loop");
  //This loop is supposed to run endless
  exploration_object.run();
  return 0;
}

//! This function initializes the exploration object

Exploration::Exploration()
{
  // Initialization of the ROS-objects
  ac_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base",true);
  subExplorationGoals_ = n_.subscribe("/database_binding/exploration_goals",10,&Exploration::explorationGoalCallback, this);
  // Needed - otherwise we're not going to have the first goal set
  busyDriving = false;
  //initialize currentGoal TODO: Should this move?
  currentGoal_.target_pose.header.frame_id = "map";
  while(!ac_->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //initialize detectionhandling
  sub_detections_ = n_.subscribe("/person_detector/all_recognitions",10,&Exploration::detectionsCallback,this);
  confirmation_client_ = n_.serviceClient<human_interface::RecognitionConfirmation>("human_interface/speech_confirmation");
}

//! This callback is called if a new goal is submitted on the topic

void Exploration::explorationGoalCallback(const database_binding::explorationGoal received_goal) {
  ROS_INFO("Received a exploration goal with x = %f and y = %f",received_goal.exploration_x,received_goal.exploration_y);
  newGoals_.push_back(received_goal);
  calcExplorationGoals();
}

void Exploration::detectionsCallback(const person_detector::DetectionObjectArray rec)
{
  detections_ = rec;
}

//! This orders the goals in the most effective way.

bool Exploration::calcExplorationGoals()
{
  if (!newGoals_.empty())
  {
      orderedGoals_.push_back(newGoals_.front());
      //ROS_INFO("The first pushed prob is %f",orderedGoals_.back().exploration_prob);
      newGoals_.erase(newGoals_.begin());
  }
}

//! This function sets the next goal and sends it to the actionserver

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

//! This function is supposed to run endless

int Exploration::run()
{
  ros::Rate r(1);
  while (ros::ok())
  {
    //explorationGoal stuff
    ROS_DEBUG("Our goals state is: %s", ac_->getState().toString().c_str());
    // checks if we can set a new goal
    if((!busyDriving && !orderedGoals_.empty()) || (busyDriving && (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)))
      {
        setNewGoal();
      }

    //detectionsstuff
    ROS_INFO("Looking for detection");
    if (!detections_.detections.empty() && !detections_.detections.front().recognitions.name_array.empty())
    {
      ROS_INFO("Found a detection and asking");
      human_interface::RecognitionConfirmationRequest req;
      human_interface::RecognitionConfirmationResponse res;
      req.header.stamp = ros::Time::now();
      std::string name = detections_.detections.front().recognitions.name_array.front().label;
      req.name_array.push_back(name);
      req.recognition_id = detections_.detections.front().header.seq;
      confirmation_client_.call(req,res);
    }



    ros::spinOnce();
    r.sleep();
  }
    return 0;
}
