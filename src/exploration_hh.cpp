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

#include <human_interface/YesNoQuestion.h>    // To ask yes-no-questions
//#include <human_interface/include/human_interface/enums.h>        // To get enums - doesn't work
#include <person_detector/SpeechConfirmation.h> // To publish the speech confirmation
#include <geometry_msgs/Point.h>              // for RVIZ
#include <sensor_msgs/image_encodings.h>      // to save images as files


//! The main loop initializes the node, build an object and runs it endless

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration_hh_node");
  Exploration exploration_object;

  ROS_INFO("Finished initialization, now running in the loop");
  //This loop is supposed to run endless
  exploration_object.run();
  return 0;
}

/*! The publisher, subscriber. clients are initialized. The constructor waits for the movebase actionserver to come up. */

Exploration::Exploration() :
  imageTransport_(n_)
{
  // Initialization of the ROS-objects
  pubConfirmations_ = n_.advertise<person_detector::SpeechConfirmation>("/person_detector/confirmations",10);
  ac_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base",true);
  sub_exploration_goals_ = n_.subscribe("/database_binding/exploration_goals",10,&Exploration::explorationGoalCallback, this);
  sub_obstacles_ = n_.subscribe("/person_detector/all_obstacles",10,&Exploration::obstacleCallback,this);
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
  yes_no_client_ = n_.serviceClient<human_interface::YesNoQuestion>("human_interface/yes_no_question");
  //init name - not permanent
  name_ = "Matthias";
  goal_counter_ = 0;
  node_state_ = exploration_hh::IDLE;
  speech_confirmation_id_ = 0;
  //default probability threshhold
  accept_threshold_ = 50;
  erase_threshold_ = 40;
  image_counter_ = 0;
  first_goal_set_ = false;

  //initialize image-saving
  sub_img_ = new image_transport::Subscriber (imageTransport_.subscribe("/turtlebot_panorama/panorama",1,&Exploration::imageCallback,this));

}

/*! It receives a goal and creates a new ExplorationGoal and pushes in the Exploration::exploration_goals_ vector. */

void Exploration::explorationGoalCallback(const exploration_hh::ExplorationGoal received_goal) {
  ROS_INFO("Received a exploration goal with x = %f and y = %f",received_goal.pose.pose.position.x,received_goal.pose.pose.position.y);
  exploration_hh::ExplorationGoal goal = received_goal;
  goal.header.seq = goal_counter_;
  goal.done = false;
  goal_counter_++;
  exploration_goals_.push_back(goal);
}

/*! Basically safes the received array into a class variabe */

void Exploration::detectionsCallback(const person_detector::DetectionObjectArray rec)
{
  detections_ = rec;
}

void Exploration::obstacleCallback(const person_detector::ObstacleArray obs)
{
  obstacles_ = obs;
}

/*! It saves the image interally with metainformation of the robot_pose, the seen obstacles and the seen faces.
    It also saves as well as to the disk to /tmp/panorama_+image_counter_ */

void Exploration::imageCallback(const sensor_msgs::Image::ConstPtr &img)
{
  //we receive several images in between, before we get the actual panorama image - trying.
  //but we know, that panorama images are quite big, so we define a treshhold
  if (img.get()->width < 1500) return;

  //first save internal
  exploration_hh::img_meta internal;
  internal.id = image_counter_;
  internal.img = *img.get();
  //more later on
  images_.push_back(internal);

  //then save it to the harddisk
  cv_bridge::CvImagePtr cv_ptr;
  std::string filename = "/tmp/panorama_";
  filename += boost::lexical_cast<std::string>(image_counter_);
  filename += ".png";
  image_counter_++;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& ex)
  {
    ROS_ERROR("cv_bridge exception when converting panorama image: %s",ex.what());
    return;
  }
  //generate filename
  try
  {
    cv::imwrite(filename,cv_ptr->image);
  }
  catch (cv_bridge::Exception& ex)
  {
    ROS_ERROR("imwrite exception when saving the image: %s",ex.what());
  }
  ROS_INFO("Saved an image called %s",filename.c_str());

}

/*! Right now we build the vector with the following rules
    1. all face detection goals
    2. all obstacle goals
    3. all exploration goals
    This makes sure, that every obstacle is checked.*/

bool Exploration::calcGoals()
{
  //clear the ordered goals and renew them
  ordered_goals_.clear();
  //simple approach - first all recognitions - then all obstacles - then all explorations
  for (unsigned int it = 0; it < recognition_goals_.size(); it++)
  {
    if (!recognition_goals_[it].done)
    {
      ordered_goals_.push_back(&recognition_goals_[it]);
    }
  }
  for (unsigned int it = 0; it < obstacle_goals_.size(); it++)
  {
    if (!obstacle_goals_[it].done)
    {
        ordered_goals_.push_back(&obstacle_goals_[it]);
    }
  }
  for (unsigned int it = 0; it < exploration_goals_.size(); it++)
  {
    if (!exploration_goals_[it].done)
    {
      ordered_goals_.push_back(&exploration_goals_[it]);
    }
  }
}

/*! We publish 2 different kind of rviz markers
    1. all goals as cubes
    2. text to all cubes
    All done goals have green cubes, the other have red cubes.*/

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
      switch (ordered_goals_[it]->type)
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
      label += boost::lexical_cast<std::string>(ordered_goals_[it]->probability);
      text_marker_.pose.position.x = ordered_goals_[it]->pose.pose.position.x-0.2;
      text_marker_.pose.position.y = ordered_goals_[it]->pose.pose.position.y-0.2;
      text_marker_.pose.position.z = 0.2;
      text_marker_.text = label;
      text_marker_.id = ordered_goals_[it]->header.seq;
      pub_text_marker_.publish(text_marker_);
  }
}

/*! Right now we ignore the orientation of the goal in order to avoid invalid quaternion configuration. This will change in future
    \todo Implement usage of orientation */

void Exploration::setGoal_()
{
  first_goal_set_ = true;
  if (!ordered_goals_.empty())
  {
    switch (ordered_goals_.front()->type)
    {
      case exploration_hh::EXPLORATION_GOAL :
        node_state_ = exploration_hh::EXPLORATION;
        ROS_INFO("Switched state to EXPLORATION_GOAL");
        break;
      case exploration_hh::OBSTACLE_GOAL :
        node_state_ = exploration_hh::OBSTACLE;
        ROS_INFO("Switched state to OBSTACLE_GOAL");
        break;
      case exploration_hh::RECOGNITION_GOAL :
        node_state_ = exploration_hh::FACE_RECOGNITION;
        ROS_INFO("Switched state to RECOGNITION_GOAL");
    }
    current_goal_ = ordered_goals_.front();
    move_base_goal_.target_pose.header.stamp = ros::Time::now();
    move_base_goal_.target_pose.pose.position.x = (*ordered_goals_.front()).pose.pose.position.x;
    move_base_goal_.target_pose.pose.position.y = (*ordered_goals_.front()).pose.pose.position.y;
    move_base_goal_.target_pose.pose.orientation.w = 0.70710678;
    move_base_goal_.target_pose.pose.orientation.z = -0.70710678;
    ROS_INFO("Sending goal with x = %f and y = %f",move_base_goal_.target_pose.pose.position.x, move_base_goal_.target_pose.pose.position.y);
    ac_->sendGoal(move_base_goal_);
    //set status based on the type of the goal
  }
  else
  {
    ROS_INFO_THROTTLE(30,"No new goals to set. State is IDLE");
    node_state_ = exploration_hh::IDLE;
  }
}

/*! Searches for our goal in the array of detections and informs the person_detector about our intentions.
    Forms confirmation request for human_interface by building and array of possible names. The response is processed and the person_detector is informed about the new state of this detection.
    Then the robot reacts to the result. If no response is received the node goes to the state PANORAMA.
    \todo New state FOUND */

int Exploration::confirmation_face_()
{
  //search for our detection
  int id = current_goal_->detection_id;
  person_detector::DetectionObject* object;
  for (unsigned int it = 0; it < detections_.detections.size(); it++)
  {
    if (detections_.detections[it].header.seq = id)
    {
        object = &detections_.detections[it];
        break;
    }
  }
  //publish marker for person_detector
  person_detector::SpeechConfirmation query_to_detector;
  query_to_detector.header.frame_id = "/map";
  query_to_detector.header.stamp = ros::Time::now();
  query_to_detector.header.seq = speech_confirmation_id_;
  speech_confirmation_id_++;
  query_to_detector.id = object->header.seq;
  query_to_detector.label = "";
  query_to_detector.running = true;
  pubConfirmations_.publish(query_to_detector);

  //form service call
  human_interface::RecognitionConfirmationRequest req;
  human_interface::RecognitionConfirmationResponse res;
  req.header.stamp = ros::Time::now();
  req.recognition_id = object->header.seq;
  ROS_INFO("Found a new detection with id %i",req.recognition_id);
  //build up names to search for
  if (object->recognitions.name_array.empty())
  {
      req.name_array.push_back(name_);
  }
  else
  {
    //to check if our person is among the names
    bool our_person = false;
    for (unsigned int in = 0; in < object->recognitions.name_array.size(); in++)
    {
      //add all the names to the array
      req.name_array.push_back(object->recognitions.name_array[in].label);
      if (object->recognitions.name_array[in].label == name_) our_person = true;
    }
    //if our persons name is not among them, we add that person
    if (!our_person) req.name_array.push_back(name_);
  }
  //make the service call
  ROS_INFO("Asking for confirmation of ID %i",object->header.seq);
  confirmation_client_.call(req,res);
  //process result

  //if it was the right person, we're done
  if (res.sucessfull)
    {
      query_to_detector.header.seq = speech_confirmation_id_;
      query_to_detector.header.stamp = ros::Time::now();
      speech_confirmation_id_++;
      query_to_detector.running = false;
      query_to_detector.tried = true;
      query_to_detector.suceeded = true;
      query_to_detector.label = res.label;
      pubConfirmations_.publish(query_to_detector);
      if (res.label == name_)
      {
        ROS_INFO("Found the person - we're done");
        //publish result to person_detector
        human_interface::SpeechRequest sp_req;
        sp_req.text_to_say = "Hey, I was searching for you. I'm so happy that I found you.";
        pub_speech_.publish(sp_req);
        ordered_goals_.clear();
        node_state_ = exploration_hh::IDLE;
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
      query_to_detector.header.seq = speech_confirmation_id_;
      speech_confirmation_id_++;
      query_to_detector.running = false;
      query_to_detector.tried = true;
      query_to_detector.suceeded = false;
      pubConfirmations_.publish(query_to_detector);
      //find our goal to delete it
      for (unsigned int it = 0; it < recognition_goals_.size(); it++)
      {
          if (recognition_goals_[it].header.seq = current_goal_->header.seq)
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
      query_to_detector.header.seq = speech_confirmation_id_;
      speech_confirmation_id_++;
      query_to_detector.running = false;
      query_to_detector.suceeded = false;
      query_to_detector.tried = true;
      pubConfirmations_.publish(query_to_detector);
      //start panorama
      node_state_ = exploration_hh::PANORAMA;
      ROS_INFO("Switched state to PANORAMA");
    }
  return 0;
}

/*! Informs the person_detector about the intention and asks if the obstace is a human. Processes the result and may ask more questions. If there's no response the node goes into the state PANORAMA */

int Exploration::confirmation_obstacle_()
{
  // inform person_detector about our plans
  person_detector::SpeechConfirmation conf;
  conf.header.seq = speech_confirmation_id_;
  speech_confirmation_id_++;
  conf.header.stamp = ros::Time::now();
  conf.id = current_goal_->detection_id;
  conf.running = true;
  pubConfirmations_.publish(conf);

  //Ask if it's a human person
  human_interface::YesNoQuestionRequest req;
  human_interface::YesNoQuestionResponse res;
  req.question = "Hey, I'm Max and I'm searching for a person. I've seen this spot and wonder if you are human. Are you?";
  req.header.stamp = ros::Time::now();
  req.expires = ros::Time::now() + ros::Duration(15);
  yes_no_client_.call(req,res);

  //process result
  if (res.status == human_interface::ANSWERED)
  {
    if (res.answer == true)
    {
      //Ask if it's the person, we're looking for
        req.question = "Cool. Are you " + name_ + "?";
        req.header.seq++;
        yes_no_client_.call(req,res);
      //Process this result
      if (res.status == human_interface::ANSWERED)
      {
        if (res.answer == true)
        {
          //we found the person
          conf.header.seq = speech_confirmation_id_;
          speech_confirmation_id_++;
          conf.header.stamp = conf.latest_confirmation = ros::Time::now();
          conf.label = name_;
          conf.running = false;
          conf.suceeded = true;
          pubConfirmations_.publish(conf);
          human_interface::SpeechRequest s_req;
          s_req.text_to_say = "Yeah - I've found you.";
          pub_speech_.publish(s_req);
          ordered_goals_.clear();
          node_state_ = exploration_hh::IDLE;
        }
        else
        {
          human_interface::SpeechRequest s_req;
          s_req.text_to_say = "Ok. I'm keeping on searching for " + name_ + ".";
          conf.header.seq = speech_confirmation_id_;
          speech_confirmation_id_++;
          conf.header.stamp = ros::Time::now();
          conf.running = false;
          conf.suceeded = true;
          pub_speech_.publish(s_req);
          //set goal done and set new goal
          current_goal_->done = true;
          ordered_goals_.erase(ordered_goals_.begin());
          setGoal_();
        }
      }
      else if (res.status == human_interface::UNANSWERED || res.status == human_interface::WRONG_ANSWER || res.status == human_interface::BLOCKED_SPEAKER)
      {
        conf.header.seq = speech_confirmation_id_;
        speech_confirmation_id_++;
        conf.header.stamp = ros::Time::now();
        conf.running = false;
        conf.tried = true;
        pubConfirmations_.publish(conf);
        human_interface::SpeechRequest s_req;
        s_req.text_to_say = "Okay, I assume that this is an obstacle and take a picture";
        pub_speech_.publish(s_req);
        node_state_ = exploration_hh::PANORAMA;
        ROS_INFO("Switched state to PANORAMA");
      }
    }
    else // answering no-human
    {
      human_interface::SpeechRequest s_req;
      s_req.text_to_say = "You must be kidding. But I'm taking you serious and treat you as an obstacle.";
      pub_speech_.publish(s_req);
      conf.header.seq = speech_confirmation_id_;
      speech_confirmation_id_++;
      conf.header.stamp = ros::Time::now();
      conf.running = false;
      conf.tried = true;
      pubConfirmations_.publish(conf);
      node_state_ = exploration_hh::PANORAMA;
      ROS_INFO("Switched to state PANORAMA.");
    }
  }
  else if (res.status == human_interface::BLOCKED_SPEAKER || res.status == human_interface::UNANSWERED || res.status == human_interface::WRONG_ANSWER)
  {
    conf.header.seq = speech_confirmation_id_;
    speech_confirmation_id_++;
    conf.header.stamp = ros::Time::now();
    conf.running = false;
    conf.tried = true;
    pubConfirmations_.publish(conf);
    human_interface::SpeechRequest s_req;
    s_req.text_to_say = "I didn't get an proper answer. I'm going to take a picture now";
    pub_speech_.publish(s_req);
    node_state_ = exploration_hh::PANORAMA;
    ROS_INFO("Switched to state PANORAMA.");
  }
}

/*! Waits until the base reaches the goal. If the point can't be reached, the goal is set to done. This will change in the future.
    \todo Implement better handling of unreached goals */

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
    //find goal and mark it as done
    for (unsigned int it = 0; it < recognition_goals_.size(); it++)
    {
      if (current_goal_->header.seq == recognition_goals_[it].header.seq)
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
    node_state_ = exploration_hh::CONFIRMATION;
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
      if (current_goal_->header.seq == exploration_goals_[it].header.seq)
      {
          exploration_goals_[it].done = true;
      }
    }
    ordered_goals_.erase(ordered_goals_.begin());
    setGoal_();
  }
  if (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    node_state_ = exploration_hh::PANORAMA;
    ROS_INFO("Switched state to PANORAMA");
  }
  return 0;
}

int Exploration::obstacleGoal_()
{
  //wait if our goal is running
  if (ac_->getState() == actionlib::SimpleClientGoalState::PENDING || ac_->getState() == actionlib::SimpleClientGoalState::ACTIVE)
  {
    ROS_INFO_THROTTLE(15,"Driving to obstacle goal, %i",current_goal_->header.seq);
    return 0;
  }
  if (ac_->getState() == actionlib::SimpleClientGoalState::ABORTED)
  {
    //find goal
    for (unsigned int it = 0; it < exploration_goals_.size(); it++)
    {
      if (current_goal_->header.seq == obstacle_goals_[it].header.seq)
      {
          obstacle_goals_[it].done = true;
      }
    }
    ordered_goals_.erase(ordered_goals_.begin());
    setGoal_();
  }
  if (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    node_state_ = exploration_hh::CONFIRMATION;
    ROS_INFO("Switched state to CONFIRMATION");
  }
  return 0;
}

/*! Performs a full turn to do a panorama picture. If the picture is received, the goal is set to be done and a new goal will be set.*/

int Exploration::panorama_()
{
  //crazy stuff

  //clear goal
  //search for the goal
  exploration_hh::ExplorationGoal* found_goal;
  for (unsigned int it = 0; it < exploration_goals_.size(); it++)
  {
    if (exploration_goals_[it].header.seq == current_goal_->header.seq)
    {
        found_goal = &exploration_goals_[it];
        break;
    }
  }
  for (unsigned int it = 0; it < obstacle_goals_.size(); it++)
  {
    if (obstacle_goals_[it].header.seq == current_goal_->header.seq)
    {
        found_goal = &obstacle_goals_[it];
        break;
    }
  }
  for (unsigned int it = 0; it < recognition_goals_.size(); it++)
  {
    if (recognition_goals_[it].header.seq == current_goal_->header.seq)
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

/*! If an detection_goal with the same id is found, this goal is going to be updated. If no corresponding goal is found, a new goal will be created.*/

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
      goal.header.seq = goal_counter_;
      goal_counter_++;
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

/*! If an obstace_goal with the same ID is found, it's updated. Otherwise the obstacle is added as a goal if it fulfills the criteria
    - presence -> a goal has to be present to be accepted
    - probability higher than the threshold */

bool Exploration::processObstacles()
{
  //skip if it's empty
  if (obstacles_.obstacles.empty()) return true;
  //update goals
  for (unsigned int it = 0; it < obstacles_.obstacles.size(); it++)
  {
    //check if we already have this goal
    bool found = false;
    if (obstacle_goals_.size() > 0)
    {
      for (unsigned int ig = 0; ig < obstacle_goals_.size(); ig++)
      {
        if (obstacle_goals_[ig].detection_id == obstacles_.obstacles[it].header.seq)
        {
          //update information
          found = true;
          //delete it, if it's unchecked and doesn't exist anymore OR if the probability went lower than 50
          if ((!obstacles_.obstacles[it].present && !obstacles_.obstacles[it].confirmation.suceeded) || obstacles_.obstacles[it].probability < erase_threshold_)
          {
            ROS_INFO("Deleted obstacle with ID %i. It was present: %i. It had probability: %i",obstacle_goals_[ig].detection_id,obstacles_.obstacles[it].present,obstacles_.obstacles[it].probability);
            obstacle_goals_.erase(obstacle_goals_.begin()+ig);
            break;
          }
          if (obstacles_.obstacles[it].confirmation.suceeded)
          {
              obstacle_goals_[ig].done = true;
          }
          obstacle_goals_[ig].pose.pose = obstacles_.obstacles[it].robot_pose.pose.pose;
          obstacle_goals_[ig].probability = obstacles_.obstacles[it].probability;
          obstacle_goals_[ig].header.stamp = ros::Time::now();
          break;
        }
      }
    }
    //if we found that obstacle,we're done with that
    if (found) continue;
    //checks obstacles for the criteria of a threshold and the presence and creates a new goal
    if (obstacles_.obstacles[it].probability > accept_threshold_ || obstacles_.obstacles[it].present)
    {
      exploration_hh::ExplorationGoal goal;
      goal.header.frame_id = "/map";
      goal.header.seq = goal_counter_;
      goal_counter_++;
      goal.header.stamp = ros::Time::now();
      goal.pose.header.frame_id = "/map";
      goal.pose.header.seq = 0;
      goal.pose.header.stamp = ros::Time::now();
      goal.pose.pose = obstacles_.obstacles[it].robot_pose.pose.pose;
      goal.type = exploration_hh::OBSTACLE_GOAL;
      goal.done = false;
      goal.probability = obstacles_.obstacles[it].probability;
      goal.detection_id = obstacles_.obstacles[it].header.seq;
      obstacle_goals_.push_back(goal);
      ROS_INFO("Added new obstacle to the goal list with goal_number %i, goal id %i and probability %i",goal_counter_,obstacle_goals_[it].header.seq,goal.probability);

    }
  }
}

/*! This function is supposed to run endless on a certain frequency. It calls the state functions according to the nodes state.
    \todo Add other frequencies for some functions */

int Exploration::run()
{
  ros::Rate r(10);
  while (ros::ok())
  {
    //care about goals
    calcGoals();
    showGoals();
    //detectionsstuff
    processDetections();
    processObstacles();

    //we just need to continue if we have goals
    if (!ordered_goals_.empty() || first_goal_set_)
      {
        //check if our current goal is still the prefered goal in the order
        if (current_goal_->detection_id != ordered_goals_.front()->detection_id)
        {
          //front goal is different - we have to set a new goal
          if (node_state_ == exploration_hh::IDLE || node_state_ == exploration_hh::EXPLORATION || node_state_ == exploration_hh::OBSTACLE || node_state_ == exploration_hh::CONFIRMATION)
          {
            //in these state we're just driving or waiting - we can safely set a new goal
            setGoal_();
          }
          else if (node_state_ == exploration_hh::PANORAMA)
          {
            // aborting the panorama takes extra effort

            // abort the panorama action
            setGoal_();
          }
        }

        switch (node_state_)
        {
          case exploration_hh::IDLE :
            setGoal_();
            break;
          case exploration_hh::CONFIRMATION :
            if (current_goal_->type == exploration_hh::RECOGNITION_GOAL)
            {
              confirmation_face_();
            }
            else
            {
              confirmation_obstacle_();
            }
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
      }

    ros::spinOnce();
    r.sleep();
  }
    return 0;
}
