#include <ros/ros.h>                          // Needed for general ROS-support
#include "exploration_hh.h"                   // Class header
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/Led.h>
#include <std_srvs/Empty.h>                   // Needed to send calls
#include <std_msgs/Empty.h>                   //  --
#include <human_interface/SpeechRequest.h>    // To say things
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
  pub_speech_ = n_.advertise<human_interface::SpeechRequest>("/human_interface/speech_request",10);
  // Needed - otherwise we're not going to have the first goal set
  busyDriving = false;
  //initialize currentGoal TODO: Should this move?
  currentGoal_.target_pose.header.frame_id = "map";
  while(!ac_->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //initialize detection handling
  sub_detections_ = n_.subscribe("/person_detector/all_recognitions",10,&Exploration::detectionsCallback,this);
  confirmation_client_ = n_.serviceClient<human_interface::RecognitionConfirmation>("human_interface/speech_confirmation");
  //init name - not permanent
  name = "Matthias";
  node_state = exploration_hh::IDLE;
}

//! This callback is called if a new goal is submitted on the topic

void Exploration::explorationGoalCallback(const exploration_hh::ExplorationGoal received_goal) {
  ROS_INFO("Received a exploration goal with x = %f and y = %f",received_goal.pose.pose.position.x,received_goal.pose.pose.position.y);
  exploration_goals_.push_back(received_goal);
}

void Exploration::detectionsCallback(const person_detector::DetectionObjectArray rec)
{
  detections_ = rec;
}

//! This orders the goals in the most effective way.

bool Exploration::calcExplorationGoals()
{
  while (!exploration_goals_.empty())
  {
      ordered_goals_.push_back(exploration_goals_.front());
      //ROS_INFO("The first pushed prob is %f",orderedGoals_.back().exploration_prob);
      exploration_goals_.erase(exploration_goals_.begin());
  }
}

//! This function sets the next goal and sends it to the actionserver

void Exploration::setNewGoal()
{
  if (!ordered_goals_.empty())
  {
    currentGoal_.target_pose.header.stamp = ros::Time::now();
    currentGoal_.target_pose.pose.position.x = ordered_goals_.front().pose.pose.position.x;
    currentGoal_.target_pose.pose.position.y = ordered_goals_.front().pose.pose.position.y;
    currentGoal_.target_pose.pose.orientation.w = 0.70710678;
    currentGoal_.target_pose.pose.orientation.z = -0.70710678;
    ROS_INFO("Sending goal with x = %f and y = %f",currentGoal_.target_pose.pose.position.x, currentGoal_.target_pose.pose.position.y);
    ac_->sendGoal(currentGoal_);
    busyDriving = true;
    ROS_INFO("Sent new goal to the actionserver");
  }
  else
  {
    busyDriving = false;
    ROS_INFO("No new goals to set");
  }
}

bool Exploration::processDetections()
{
  //skipping if we don't have any detections
  if (detections_.detections.empty()) return true;

  //going through all the detections
  for (unsigned int it = 0; it < detections_.detections.size(); it++)
  {
    if (!detections_.detections[it].confirmation.running &&
        !detections_.detections[it].confirmation.tried &&
        !detections_.detections[it].confirmation.suceeded)
    {
      //check distance with an if

      human_interface::RecognitionConfirmationRequest req;
      human_interface::RecognitionConfirmationResponse res;
      req.header.stamp = ros::Time::now();
      req.recognition_id = detections_.detections[it].header.seq;
      ROS_INFO("Found a new detection with id %i",req.recognition_id);
      //build up names to search for
      if (detections_.detections[it].recognitions.name_array.empty())
      {
          req.name_array.push_back(name);
      }
      else
      {
        //to check if our person is among the names
        bool our_person = false;
        for (unsigned int in = 0; in < detections_.detections[it].recognitions.name_array.size(); in++)
        {
          //add all the names to the array
          req.name_array.push_back(detections_.detections[it].recognitions.name_array[in].label);
          if (detections_.detections[it].recognitions.name_array[in].label == name) our_person = true;
        }
        //if our persons name is not among them, we add that person
        if (!our_person) req.name_array.push_back(name);
      }
      ac_->cancelAllGoals();
      //create new goal
      ROS_INFO("Now driving to the place, where we've seen the detection.");
      currentGoal_.target_pose.header.seq++;
      currentGoal_.target_pose.header.stamp = ros::Time::now();
      currentGoal_.target_pose.pose = detections_.detections[it].last_seen_from.pose.pose;
      ac_->sendGoal(currentGoal_);
      ros::Rate r(5);
      while (ac_->getState() == actionlib::SimpleClientGoalState::PENDING || ac_->getState() == actionlib::SimpleClientGoalState::ACTIVE)
      {
        r.sleep();
      }
      if (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("Asking for confirmation of ID %i",detections_.detections[it].header.seq);
          confirmation_client_.call(req,res);
          if (!res.sucessfull)
            {
              human_interface::SpeechRequest sp_req;
              sp_req.text_to_say = "I didn't receive a proper answer. I'm going on.";
              pub_speech_.publish(sp_req);
            }
          //if it wasn't the right person, keep on going
          if (res.sucessfull && res.label == name)
            {
              ROS_INFO("Found the person - we're done");
              human_interface::SpeechRequest sp_req;
              sp_req.text_to_say = "Hey, I was searching for you. I'm so happy that I found you.";
              ordered_goals_.clear();
              node_state = exploration_hh::IDLE;
            }
          else
            {
              ROS_INFO("That wasn't the right person - going on");
              setNewGoal();
            }

        }
      else
      {
        ROS_INFO("Didn't reach that person, we go on with the normal goals");
        setNewGoal();
      }

      return true;
    }
  }


  return true;
}

//! This function is supposed to run endless

int Exploration::run()
{
  ros::Rate r(10);
  while (ros::ok())
  {
    //explorationGoal stuff

    calcExplorationGoals();
    ROS_DEBUG("Our goals state is: %s", ac_->getState().toString().c_str());
    // checks if we can set a new goal
    if(!busyDriving && !ordered_goals_.empty())
    {
       setNewGoal();
    }


    if (busyDriving && (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
      {
        ordered_goals_.erase(ordered_goals_.begin());
        setNewGoal();
      }

    //detectionsstuff
    processDetections();



    ros::spinOnce();
    r.sleep();
  }
    return 0;
}
