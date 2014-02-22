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
#include <person_detector/SpeechConfirmation.h> // To publish the speech confirmation
#include <geometry_msgs/Point.h>              // for RVIZ




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
  pubConfirmations_ = n_.advertise<person_detector::SpeechConfirmation>("/person_detector/confirmations",10);
  ac_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base",true);
  subExplorationGoals_ = n_.subscribe("/database_binding/exploration_goals",10,&Exploration::explorationGoalCallback, this);
  pub_speech_ = n_.advertise<human_interface::SpeechRequest>("/human_interface/speech_request",10);
  //initialize currentGoal
  move_base_goal_.target_pose.header.frame_id = "map";
  while(!ac_->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //initialize rviz marker
  pub_point_marker_ = n_.advertise<visualization_msgs::Marker>("/exploration_hh/goal_marker",10);
  pub_text_marker_ = n_.advertise<visualization_msgs::Marker>("/exploration_hh/goal_text_marker",10);
  point_marker_.header.frame_id = "/map";
  point_marker_.ns = "exploration_hh/goal_marker";
  point_marker_.id = 0;
  point_marker_.lifetime = ros::Duration(10);
  point_marker_.action = visualization_msgs::Marker::ADD;
  point_marker_.type = visualization_msgs::Marker::POINTS;
  point_marker_.scale.x = 0.2;
  point_marker_.scale.y = 0.2;
  point_marker_.scale.z = 0.2;
  point_marker_.color.g = 1.0f;
  point_marker_.color.a = 1.0;

  text_marker_.header.frame_id = "/map";
  text_marker_.ns = "exploration_hh/text_marker";
  text_marker_.id = 0;
  text_marker_.lifetime = ros::Duration(10);
  text_marker_.action = visualization_msgs::Marker::ADD;
  text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.2;
  text_marker_.color.r = 1.0f;
  text_marker_.color.a = 1.0;

  //initialize detection handling
  sub_detections_ = n_.subscribe("/person_detector/all_recognitions",10,&Exploration::detectionsCallback,this);
  confirmation_client_ = n_.serviceClient<human_interface::RecognitionConfirmation>("human_interface/speech_confirmation");
  //init name - not permanent
  name = "Matthias";
  goal_counter = 0;
  node_state = exploration_hh::IDLE;
  speech_confirmation_id = 0;
}

//! This callback is called if a new goal is submitted on the topic

void Exploration::explorationGoalCallback(const exploration_hh::ExplorationGoal received_goal) {
  ROS_INFO("Received a exploration goal with x = %f and y = %f",received_goal.pose.pose.position.x,received_goal.pose.pose.position.y);
  exploration_hh::ExplorationGoal goal = received_goal;
  goal.header.seq = goal_counter;
  goal.done = false;
  goal_counter++;
  exploration_goals_.push_back(goal);
}

void Exploration::detectionsCallback(const person_detector::DetectionObjectArray rec)
{
  detections_ = rec;
}

//! This orders the goals in the most effective way.

bool Exploration::calcGoals()
{
  //clear the ordered goals and renew them
  ordered_goals_.clear();
  //simple approach - first all recognitions - then all explorations
  for (unsigned int it = 0; it < recognition_goals_.size(); it++)
  {
      if (!recognition_goals_[it].done)
      {
        ordered_goals_.push_back(recognition_goals_[it]);
      }
  }
  for (unsigned int it = 0; it < exploration_goals_.size(); it++)
  {
      if (!exploration_goals_[it].done)
      {
        ordered_goals_.push_back(exploration_goals_[it]);
      }
  }
}

void Exploration::showGoals()
{
  //first publish all goals
  geometry_msgs::Point p;
  for (unsigned int it = 0; it < exploration_goals_.size(); it++)
  {
      p.x = exploration_goals_[it].pose.pose.position.x;
      p.y = exploration_goals_[it].pose.pose.position.y;
      p.z = 0;
      point_marker_.id = exploration_goals_[it].header.seq;
      if (exploration_goals_[it].done)
      {
        point_marker_.color.b = 0;
        point_marker_.color.g = 1;
        point_marker_.color.r = 0;
      }
      else
      {
        point_marker_.color.b = 0;
        point_marker_.color.r = 1;
        point_marker_.color.g = 0;
      }
      point_marker_.points.push_back(p);
      pub_point_marker_.publish(point_marker_);
      point_marker_.points.clear();
  }
  for (unsigned int it = 0; it < obstacle_goals_.size(); it++)
  {
      point_marker_.pose.position.x = obstacle_goals_[it].pose.pose.position.x;
      point_marker_.pose.position.y = obstacle_goals_[it].pose.pose.position.y;
      point_marker_.pose.position.z = 0;
      point_marker_.id = obstacle_goals_[it].header.seq;
      if (obstacle_goals_[it].done)
      {
        point_marker_.color.b = 0;
        point_marker_.color.g = 1;
        point_marker_.color.r = 0;
      }
      else
      {
        point_marker_.color.b = 0;
        point_marker_.color.r = 1;
        point_marker_.color.g = 0;
      }
      pub_point_marker_.publish(point_marker_);
  }
  for (unsigned int it = 0; it < recognition_goals_.size(); it++)
  {
      point_marker_.pose.position.x = recognition_goals_[it].pose.pose.position.x;
      point_marker_.pose.position.y = recognition_goals_[it].pose.pose.position.y;
      point_marker_.pose.position.z = 0;
      point_marker_.id = recognition_goals_[it].header.seq;
      if (recognition_goals_[it].done)
      {
        point_marker_.color.b = 0;
        point_marker_.color.g = 1;
        point_marker_.color.r = 0;
      }
      else
      {
        point_marker_.color.b = 0;
        point_marker_.color.r = 1;
        point_marker_.color.g = 0;
      }
      pub_point_marker_.publish(point_marker_);
  }

  //then publish the text of the ordered goals
  std::string label;
  for (unsigned int it = 0; it < ordered_goals_.size(); it++)
  {
      label = boost::lexical_cast<std::string>(it+1);
      label += ". ";
      switch (ordered_goals_[it].type)
      {
      case exploration_hh::EXPLORATION_GOAL :
          label += "Exploration ";
          break;
        case exploration_hh::OBSTACLE_GOAL :
          label += "Obstacle ";
          break;
        case exploration_hh::RECOGNITION_GOAL :
          label += "Recognition";
      }
      label += boost::lexical_cast<std::string>(ordered_goals_[it].probability);
      text_marker_.pose.position.x = ordered_goals_[it].pose.pose.position.x-0.2;
      text_marker_.pose.position.y = ordered_goals_[it].pose.pose.position.y-0.2;
      text_marker_.pose.position.z = 0.2;
      text_marker_.text = label;
      text_marker_.id = ordered_goals_[it].header.seq;
      pub_text_marker_.publish(text_marker_);
  }
}

//! This function sets the next goal and sends it to the actionserver

void Exploration::setGoal_()
{
  if (!ordered_goals_.empty())
  {
    switch (current_goal_.type)
    {
      case exploration_hh::EXPLORATION_GOAL :
        node_state = exploration_hh::EXPLORATION;
        ROS_INFO("Switched state to EXPLORATION_GOAL");
        break;
      case exploration_hh::OBSTACLE_GOAL :
        node_state = exploration_hh::OBSTACLE;
        ROS_INFO("Switched state to OBSTACLE_GOAL");
        break;
      case exploration_hh::RECOGNITION_GOAL :
        node_state = exploration_hh::FACE_RECOGNITION;
        ROS_INFO("Switched state to RECOGNITION_GOAL");
    }
    current_goal_ = ordered_goals_.front();
    move_base_goal_.target_pose.header.stamp = ros::Time::now();
    move_base_goal_.target_pose.pose.position.x = ordered_goals_.front().pose.pose.position.x;
    move_base_goal_.target_pose.pose.position.y = ordered_goals_.front().pose.pose.position.y;
    move_base_goal_.target_pose.pose.orientation.w = 0.70710678;
    move_base_goal_.target_pose.pose.orientation.z = -0.70710678;
    ROS_INFO("Sending goal with x = %f and y = %f",move_base_goal_.target_pose.pose.position.x, move_base_goal_.target_pose.pose.position.y);
    ac_->sendGoal(move_base_goal_);
    //set status based on the type of the goal
  }
  else
  {
    ROS_INFO_THROTTLE(30,"No new goals to set. State is IDLE");
    node_state = exploration_hh::IDLE;
  }
}

int Exploration::confirmation_()
{
  //search for our detection
  int id = current_goal_.detection_id;
  person_detector::DetectionObject* object;
  for (unsigned int it = 0; it < detections_.detections.size(); it++)
  {
    if (detections_.detections[it].header.seq = id)
    {
        object = &detections_.detections[it];
        break;
    }
  }
  //form service call
  human_interface::RecognitionConfirmationRequest req;
  human_interface::RecognitionConfirmationResponse res;
  req.header.stamp = ros::Time::now();
  req.recognition_id = object->header.seq;
  ROS_INFO("Found a new detection with id %i",req.recognition_id);
  //build up names to search for
  if (object->recognitions.name_array.empty())
  {
      req.name_array.push_back(name);
  }
  else
  {
    //to check if our person is among the names
    bool our_person = false;
    for (unsigned int in = 0; in < object->recognitions.name_array.size(); in++)
    {
      //add all the names to the array
      req.name_array.push_back(object->recognitions.name_array[in].label);
      if (object->recognitions.name_array[in].label == name) our_person = true;
    }
    //if our persons name is not among them, we add that person
    if (!our_person) req.name_array.push_back(name);
  }
  //publish marker for person_detector
  person_detector::SpeechConfirmation query_to_detector;
  query_to_detector.header.frame_id = "/map";
  query_to_detector.header.stamp = ros::Time::now();
  query_to_detector.header.seq = speech_confirmation_id;
  speech_confirmation_id++;
  query_to_detector.id = object->header.seq;
  query_to_detector.label = "";
  query_to_detector.running = true;
  pubConfirmations_.publish(query_to_detector);
  //make the service call
  ROS_INFO("Asking for confirmation of ID %i",object->header.seq);
  confirmation_client_.call(req,res);
  //process result

  //if it was the right person, we're done
  if (res.sucessfull)
    {
      query_to_detector.header.seq = speech_confirmation_id;
      query_to_detector.header.stamp = ros::Time::now();
      speech_confirmation_id++;
      query_to_detector.running = false;
      query_to_detector.tried = true;
      query_to_detector.suceeded = true;
      query_to_detector.label = res.label;
      pubConfirmations_.publish(query_to_detector);
      if (res.label == name)
      {
        ROS_INFO("Found the person - we're done");
        //publish result to person_detector
        human_interface::SpeechRequest sp_req;
        sp_req.text_to_say = "Hey, I was searching for you. I'm so happy that I found you.";
        pub_speech_.publish(sp_req);
        ordered_goals_.clear();
        node_state = exploration_hh::IDLE;
      }
      else //another person
      {
        ROS_INFO("Found another person - going on");
        human_interface::SpeechRequest sp_req;
        sp_req.text_to_say = "You were not the one I was locking for. I'm going on";
        pub_speech_.publish(sp_req);
        setGoal_();
      }
    }
    else if (!res.sucessfull && res.answered)
    {
      human_interface::SpeechRequest sp_req;
      sp_req.text_to_say = "Seems like I had a wrong guess. But I'm going on now";
      pub_speech_.publish(sp_req);
      ROS_INFO("Wrong guess about the person - going on");
      query_to_detector.header.seq = speech_confirmation_id;
      speech_confirmation_id++;
      query_to_detector.running = false;
      query_to_detector.tried = true;
      query_to_detector.suceeded = false;
      pubConfirmations_.publish(query_to_detector);
      //find our goal to delete it
      for (unsigned int it = 0; it < recognition_goals_.size(); it++)
      {
          if (recognition_goals_[it].header.seq = current_goal_.header.seq)
          {
              recognition_goals_[it].done = true;
          }
      }
      ordered_goals_.erase(ordered_goals_.begin());
      setGoal_();
    }
    else
    {
      human_interface::SpeechRequest sp_req;
      sp_req.text_to_say = "You didn't answer, so I'm going to take a picture";
      pub_speech_.publish(sp_req);
      query_to_detector.header.stamp = ros::Time::now();
      query_to_detector.header.seq = speech_confirmation_id;
      speech_confirmation_id++;
      query_to_detector.running = false;
      query_to_detector.suceeded = false;
      query_to_detector.tried = true;
      pubConfirmations_.publish(query_to_detector);
      //start panorama
      node_state = exploration_hh::PANORAMA;
      ROS_INFO("Switched state to PANORAMA");
    }
  return 0;
}

int Exploration::recognitionGoal_()
{
  //check if the person is in our FOV
  //later
  //wait if our goal is running
  if (ac_->getState() == actionlib::SimpleClientGoalState::PENDING || ac_->getState() == actionlib::SimpleClientGoalState::ACTIVE)
  {
    ROS_INFO_THROTTLE(15,"Driving to recognition goal");
    return 0;
  }
  if (ac_->getState() == actionlib::SimpleClientGoalState::ABORTED)
  {
    //find goal
    for (unsigned int it = 0; it < recognition_goals_.size(); it++)
    {
      if (current_goal_.header.seq == recognition_goals_[it].header.seq)
      {
          recognition_goals_[it].done = true;
      }
    }
    ordered_goals_.erase(ordered_goals_.begin());
    human_interface::SpeechRequest sp_req;
    sp_req.text_to_say = "Sorry, I could reach you. I'm going on";
    pub_speech_.publish(sp_req);
    setGoal_();
  }
  if (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    node_state = exploration_hh::CONFIRMATION;
    ROS_INFO("Switched state to CONFIRMATION");
  }

  return 0;
}

int Exploration::explorationGoal_()
{
  //wait if our goal is running
  if (ac_->getState() == actionlib::SimpleClientGoalState::PENDING || ac_->getState() == actionlib::SimpleClientGoalState::ACTIVE)
  {
    ROS_INFO_THROTTLE(15,"Driving to exploration goal");
    return 0;
  }
  if (ac_->getState() == actionlib::SimpleClientGoalState::ABORTED)
  {
    //find goal
    for (unsigned int it = 0; it < exploration_goals_.size(); it++)
    {
      if (current_goal_.header.seq == exploration_goals_[it].header.seq)
      {
          exploration_goals_[it].done = true;
      }
    }
    ordered_goals_.erase(ordered_goals_.begin());
    setGoal_();
  }
  if (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    node_state = exploration_hh::PANORAMA;
    ROS_INFO("Switched state to PANORAMA");
  }
  return 0;
}

int Exploration::obstacleGoal_()
{
  return 0;
}

int Exploration::panorama_()
{
  //crazy stuff


  //clear goal
  //search for the goal
  exploration_hh::ExplorationGoal* found_goal;
  for (unsigned int it = 0; it < exploration_goals_.size(); it++)
  {
    if (exploration_goals_[it].header.seq == current_goal_.header.seq)
    {
        found_goal = &exploration_goals_[it];
        break;
    }
  }
  for (unsigned int it = 0; it < obstacle_goals_.size(); it++)
  {
      if (obstacle_goals_[it].header.seq == current_goal_.header.seq)
        {
            found_goal = &obstacle_goals_[it];
            break;
        }
  }
  for (unsigned int it = 0; it < recognition_goals_.size(); it++)
  {
      if (recognition_goals_[it].header.seq == current_goal_.header.seq)
        {
            found_goal = &recognition_goals_[it];
            break;
        }
  }
  found_goal->done = true;
  ordered_goals_.erase(ordered_goals_.begin());
  setGoal_();
  return 0;
}

bool Exploration::processDetections()
{
  //skipping if we don't have any detections
  if (detections_.detections.empty()) return true;

  //going through all the detections
  for (unsigned int it = 0; it < detections_.detections.size(); it++)
  {
    //requirements of the goal
    if (!detections_.detections[it].confirmation.running &&
        !detections_.detections[it].confirmation.tried &&
        !detections_.detections[it].confirmation.suceeded)
    {
      //check, if we already have this goal
      bool found = false;
      if (recognition_goals_.size() > 0)
      {
        for (unsigned int ig = 0; ig < recognition_goals_.size(); ig++)
        {
          if (recognition_goals_[ig].detection_id == detections_.detections[it].header.seq)
          {
            found = true;
            break;
          }
        }
        //skip this goal, if it's already there
        if (found) continue;
      }
      //otherwise create a new one
      exploration_hh::ExplorationGoal goal;
      goal.header.frame_id = "/map";
      goal.header.seq = goal_counter;
      goal_counter++;
      goal.header.stamp = ros::Time::now();
      goal.pose.header.frame_id = "/map";
      goal.pose.header.seq = 0;
      goal.pose.header.stamp = ros::Time::now();
      goal.pose.pose = detections_.detections[it].last_seen_from.pose.pose;
      goal.type = exploration_hh::RECOGNITION_GOAL;
      goal.done = false;
      // there's some space to improve here
      goal.probability = 100;
      goal.detection_id = detections_.detections[it].header.seq;
      recognition_goals_.push_back(goal);
      ROS_INFO("Added new detection");
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
    calcGoals();
    showGoals();
    //detectionsstuff
    processDetections();
    //check if our current goal is still the prefered goal in the order


    switch (node_state)
    {
      case exploration_hh::IDLE :
        setGoal_();
        break;
      case exploration_hh::CONFIRMATION :
        confirmation_();
        break;
      case exploration_hh::FACE_RECOGNITION :
        recognitionGoal_();
        break;
      case exploration_hh::EXPLORATION :
        explorationGoal_();
        break;
      case exploration_hh::PANORAMA :
        panorama_();
        break;
      case exploration_hh::OBSTACLE :
        obstacleGoal_();
        break;
    }

    ros::spinOnce();
    r.sleep();
  }
    return 0;
}
