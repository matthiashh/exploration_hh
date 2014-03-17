#include <ros/ros.h>                          // Needed for general ROS-support
#include "exploration_hh.h"                   // Class header
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/Led.h>
#include <std_srvs/Empty.h>                   // Needed to send calls
#include <std_msgs/Empty.h>                   //  --
#include <human_interface/SpeechRequest.h>    // To say things
#include <amcl/map/map.h>
#include <ros/console.h>                      // To perform debug-outputs
#include <human_interface/YesNoQuestion.h>    // To ask yes-no-questions
//#include <human_interface/include/human_interface/enums.h>        // To get enums - doesn't work
#include <person_detector/SpeechConfirmation.h> // To publish the speech confirmation
#include <geometry_msgs/Point.h>              // for RVIZ
#include <sensor_msgs/image_encodings.h>      // to save images as files
#include <exploration_hh/returnPlaces.h>      // the struct for the database binding
#include <actionlib_msgs/GoalStatus.h>        // to process the goal status
#include <tf/LinearMath/Quaternion.h>         // to calculate the quaternion
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigen>
#include <navfn/navfn/navfn_ros.h>
#include <nav_core/base_global_planner.h>
#include <navfn/MakeNavPlan.h>

//! The main loop initializes the node, build an object and runs it endless

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration_hh_node");
  Exploration exploration_object("/exploration_hh/task_server","find_person");

  ROS_INFO("Finished initialization, now running in the loop");
  //This loop is supposed to run endless
  exploration_object.run();
  return 0;
}

/*! The publisher, subscriber. clients are initialized. The constructor waits for the movebase actionserver to come up. */

Exploration::Exploration(std::string task_server_name, std::string task_name) : RobotControlSimpleClient(task_server_name,task_name),
  imageTransport_(n_)
{
  // Initialization of the ROS-objects
  pub_confirmations_ = n_.advertise<person_detector::SpeechConfirmation>("/person_detector/confirmations",10);
  ac_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base",true);
  sub_exploration_goals_ = n_.subscribe("/database_binding/exploration_goals",10,&Exploration::explorationGoalCallback, this);
  sub_obstacles_ = n_.subscribe("/person_detector/all_obstacles",10,&Exploration::obstacleCallback,this);
  pub_speech_ = n_.advertise<human_interface::SpeechRequest>("/human_interface/speech_request",10);
  //initialize currentGoal
  move_base_goal_.target_pose.header.frame_id = "map";
//  while(!ac_->waitForServer(ros::Duration(5.0)))
//  {
//    ROS_INFO("Waiting for the move_base action server to come up");
//  }
  //initialize currentgoal
  current_goal_ = new exploration_hh::ExplorationGoal;
  current_goal_->detection_id = 100;
  current_goal_->done = true;

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
  point_marker_.pose.position.z = 0.3;
//  point_marker_.scale.z = 0.2;
//  point_marker_.color.g = 1.0f;
//  point_marker_.color.a = 1.0;

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
  //init name - will be overwritten by the task
  name_ = "Matthias";
  goal_counter_ = 0;
  node_state_ = exploration_hh::IDLE;
  speech_confirmation_id_ = 0;
  //default probability threshhold
  accept_threshold_ = 75;
  erase_threshold_ = 60;
  image_counter_ = 0;
  panorama_taken_ = false;
  panorama_running_ = false;
  image_taken_ = false;
  image_running_ = false;

  //initialize image-saving
  sub_img_ = new image_transport::Subscriber (imageTransport_.subscribe("/camera/rgb/image_color",1,&Exploration::imageCallback,this));
  sub_pano_ = new image_transport::Subscriber (imageTransport_.subscribe("/turtlebot_panorama/panorama",3,&Exploration::panoramaCallback,this));
  pub_pano_start_ = n_.advertise<std_msgs::Empty> ("/turtlebot_panorama/take_pano",1);
  pub_pano_stop_ = n_.advertise<std_msgs::Empty> ("/turtlebot_panorama/stop_pano",1);

  //initialize database connection
  std::string host,port,user,passwd,db;
  if (!n_.getParam("/database/hostname",host) ||
      !n_.getParam("/database/port",port) ||
      !n_.getParam("/database/db_user",user) ||
      !n_.getParam("/database/db_passwd",passwd) ||
      !n_.getParam("/database/db_name",db))
  {
      ROS_ERROR("Couldn't get all parameters for the database connection. Did you start robot_control and got a connection?");
      ROS_ERROR("All database related tasks won't work.");
  }
  else
  {
    ROS_INFO("Trying to connect with host %s, port %s, user %s, passwd %s, db %s",host.c_str(), port.c_str(),user.c_str(),passwd.c_str(),db.c_str());
    this->database_ = new database_interface::PostgresqlDatabase (host,port,user,passwd,db);
  }
}

/*! It receives a goal and creates a new ExplorationGoal and pushes in the Exploration::exploration_goals_ vector. */

void Exploration::explorationGoalCallback(const exploration_hh::ExplorationGoal received_goal) {
  ROS_INFO("Received a exploration goal with x = %f and y = %f",received_goal.pose.pose.position.x,received_goal.pose.pose.position.y);
  exploration_hh::ExplorationGoal goal = received_goal;
  goal.header.seq = goal_counter_;
  goal_counter_++;
  goal.done = false;
  goal.probability = received_goal.probability;
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

void Exploration::panoramaCallback(const sensor_msgs::Image::ConstPtr &img)
{
  //we receive several images in between, before we get the actual panorama image - trying.
  //but we know, that panorama images are quite big, so we define a treshhold
  if (!panorama_running_ || img.get()->width < 1500) return;
  ROS_INFO("Received picture - we're done with the panorama");
  panorama_taken_ = true;

}

/*! It saves an RGB image internally and to file to allow evaluation
    */
void Exploration::imageCallback(const sensor_msgs::Image::ConstPtr &img)
{
  //just quit if we don't have to take an image
  if (image_running_)
    {
      tmp_picture = img;
      image_taken_ = true;
    }
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
  point_marker_.points.clear();
  point_marker_.colors.clear();
  geometry_msgs::Point p;
  p.z = 0;
  std_msgs::ColorRGBA c;
  c.a = 1;
  for (unsigned int it = 0; it < exploration_goals_.size(); it++)
  {
      p.x = exploration_goals_[it].pose.pose.position.x;
      p.y = exploration_goals_[it].pose.pose.position.y;
      if (exploration_goals_[it].done)
      {
        c.b = 0;
        c.g = 1;
        c.r = 0;
      }
      else
      {
        c.b = 0;
        c.r = 1;
        c.g = 0;
      }
      point_marker_.colors.push_back(c);
      point_marker_.points.push_back(p);      
  }
  for (unsigned int it = 0; it < obstacle_goals_.size(); it++)
  {
      p.x = obstacle_goals_[it].pose.pose.position.x;
      p.y = obstacle_goals_[it].pose.pose.position.y;
      if (obstacle_goals_[it].done)
      {
        c.b = 0;
        c.g = 1;
        c.r = 0;
      }
      else
      {
        c.b = 0;
        c.r = 1;
        c.g = 0;
      }
      point_marker_.colors.push_back(c);
      point_marker_.points.push_back(p);
  }
  for (unsigned int it = 0; it < recognition_goals_.size(); it++)
  {
      p.x = recognition_goals_[it].pose.pose.position.x;
      p.y = recognition_goals_[it].pose.pose.position.y;
      if (recognition_goals_[it].done)
      {
        c.b = 0;
        c.g = 1;
        c.r = 0;
      }
      else
      {
        c.b = 0;
        c.r = 1;
        c.g = 0;
      }
      point_marker_.colors.push_back(c);
      point_marker_.points.push_back(p);
  }
  pub_point_marker_.publish(point_marker_);

  //then publish the text of the ordered goals
  std::string label;
  int id;
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
          label += "Obstacle ID: ";
          id = ordered_goals_[it]->detection_id;
          label += boost::lexical_cast<std::string>(id);
          break;
        case exploration_hh::RECOGNITION_GOAL :
          label += "Recognition ID: ";
          id = ordered_goals_[it]->detection_id;
          label += boost::lexical_cast<std::string>(id);
      }
      label += " prob: ";
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

void Exploration::setGoal()
{
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

int Exploration::confirmation_face()
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
  pub_confirmations_.publish(query_to_detector);

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
      pub_confirmations_.publish(query_to_detector);
      if (res.label == name_)
      {
        ROS_INFO("Found the person - we're done");
        //publish result to person_detector
        human_interface::SpeechRequest sp_req;
        sp_req.text_to_say = "Hey, I was searching for you. I'm so happy that I found you.";
        pub_speech_.publish(sp_req);
        node_state_ = exploration_hh::FOUND;
      }
      else //another person
      {
        ROS_INFO("Found another person - going on");
        human_interface::SpeechRequest sp_req;
        sp_req.text_to_say = "You were not the one I was locking for. I'm going on";
        pub_speech_.publish(sp_req);
        setGoal();
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
      pub_confirmations_.publish(query_to_detector);
      //find our goal to delete it
      current_goal_->done = true;
      ordered_goals_.erase(ordered_goals_.begin());
      setGoal();
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
      pub_confirmations_.publish(query_to_detector);
      //start panorama
      node_state_ = exploration_hh::PHOTO;
      ROS_INFO("Switched state to PHOTO");
    }
  return 0;
}

/*! Informs the person_detector about the intention and asks if the obstace is a human. Processes the result and may ask more questions. If there's no response the node goes into the state PANORAMA */

int Exploration::confirmation_obstacle()
{
  // inform person_detector about our plans
  person_detector::SpeechConfirmation conf;
  conf.header.seq = speech_confirmation_id_;
  speech_confirmation_id_++;
  conf.header.stamp = ros::Time::now();
  conf.id = current_goal_->detection_id;
  conf.running = true;
  pub_confirmations_.publish(conf);

  //Ask if it's a human person
  human_interface::YesNoQuestionRequest req;
  human_interface::YesNoQuestionResponse res;
  req.question = "Hey, I'm Max and I'm searching for a person. I've seen this spot and wonder if you are human. Are you?";
  req.header.stamp = ros::Time::now();
  req.expires = ros::Time::now() + ros::Duration(15);
  if (!yes_no_client_.call(req,res))
  {
      ROS_ERROR("Couldn't call for a yes-no-question - is the node 'human_interface' running?");
  }

  //process result
  if (res.status == human_interface::ANSWERED)
  {
    if (res.answer == true)
    {
      //Ask if it's the person, we're looking for
        req.question = "Cool. Are you " + name_ + "?";
        req.header.seq++;
        if (!yes_no_client_.call(req,res))
        {
            ROS_ERROR("Couldn't call for a yes-no-question - is the node 'human_interface' running?");
        }
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
          pub_confirmations_.publish(conf);
          human_interface::SpeechRequest s_req;
          s_req.text_to_say = "Yeah - I've found you.";
          pub_speech_.publish(s_req);
          node_state_ = exploration_hh::FOUND;
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
          pub_confirmations_.publish(conf);
          pub_speech_.publish(s_req);
          //set goal done and set new goal
          current_goal_->done = true;
          ordered_goals_.erase(ordered_goals_.begin());
          setGoal();
        }
      }
      else if (res.status == human_interface::UNANSWERED || res.status == human_interface::WRONG_ANSWER || res.status == human_interface::BLOCKED_SPEAKER)
      {
        conf.header.seq = speech_confirmation_id_;
        speech_confirmation_id_++;
        conf.header.stamp = ros::Time::now();
        conf.running = false;
        conf.tried = true;
        pub_confirmations_.publish(conf);
        human_interface::SpeechRequest s_req;
        s_req.text_to_say = "Okay, I assume that this is an obstacle and take a picture";
        pub_speech_.publish(s_req);
        node_state_ = exploration_hh::PHOTO;
        ROS_INFO("Switched state to PHOTO");
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
      pub_confirmations_.publish(conf);
      node_state_ = exploration_hh::PHOTO;
      ROS_INFO("Switched to state PHOTO.");
    }
  }
  else if (res.status == human_interface::BLOCKED_SPEAKER || res.status == human_interface::UNANSWERED || res.status == human_interface::WRONG_ANSWER)
  {
    conf.header.seq = speech_confirmation_id_;
    speech_confirmation_id_++;
    conf.header.stamp = ros::Time::now();
    conf.running = false;
    conf.tried = true;
    pub_confirmations_.publish(conf);
    human_interface::SpeechRequest s_req;
    s_req.text_to_say = "I didn't get an proper answer. I'm going to take a picture now";
    pub_speech_.publish(s_req);
    node_state_ = exploration_hh::PHOTO;
    ROS_INFO("Switched to state PHOTO.");
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
  else if (ac_->getState() == actionlib::SimpleClientGoalState::ABORTED)
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
    sp_req.text_to_say = "Sorry, I couldn't' reach you. I'm going on";
    pub_speech_.publish(sp_req);
    setGoal();
  }
  else if (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
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
    // this is bad - find a better solution
    current_goal_->done = true;
    ordered_goals_.erase(ordered_goals_.begin());
    setGoal();
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
    ROS_INFO_THROTTLE(15,"Driving to obstacle goal %i for detection ID %i",current_goal_->header.seq,current_goal_->detection_id);
    return 0;
  }
  if (ac_->getState() == actionlib::SimpleClientGoalState::ABORTED)
  {
    //this is a bad solution
      ROS_WARN("Couldn't reach obstagle goal %i for detection ID %i",current_goal_->header.seq,current_goal_->detection_id );
    current_goal_->done = true;
    ordered_goals_.erase(ordered_goals_.begin());
    setGoal();
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
  if (!panorama_taken_ && !panorama_running_)
  {
    ROS_INFO("Starting to turn around");
    std_msgs::Empty msg;
    pub_pano_start_.publish(msg);
    panorama_running_ = true;
  }
  else if (!panorama_taken_ && panorama_running_)
  {
    ROS_INFO_THROTTLE(5,"Turning around...");
    return true;
  }
  else
  {
    ROS_INFO("Finished turn. Marking goal as done.");
    panorama_running_ = false;
    panorama_taken_ = false;
    //clear goal
    //search for the goal
    //exploration_hh::ExplorationGoal* found_goal;
    /*! \todo We have pointers. This can be done easier! */
    for (unsigned int it = 0; it < exploration_goals_.size(); it++)
    {
      if (exploration_goals_[it].header.seq == current_goal_->header.seq)
      {
          //found_goal = &exploration_goals_[it];
          exploration_goals_[it].done = true;
          break;
      }
    }
//    for (unsigned int it = 0; it < obstacle_goals_.size(); it++)
//    {
//      if (obstacle_goals_[it].header.seq == current_goal_->header.seq)
//      {
//          found_goal = &obstacle_goals_[it];
//          break;
//      }
//    }
//    for (unsigned int it = 0; it < recognition_goals_.size(); it++)
//    {
//      if (recognition_goals_[it].header.seq == current_goal_->header.seq)
//      {
//          found_goal = &recognition_goals_[it];
//          break;
//      }
//    }
//    found_goal->done = true;
    ordered_goals_.erase(ordered_goals_.begin());
    setGoal();
  }
  return 0;
}


void Exploration::found()
{
  human_interface::YesNoQuestionRequest req;
  human_interface::YesNoQuestionResponse res;
  req.question = "Are you okay?";
  if (!yes_no_client_.call(req,res))
  {
      ROS_ERROR("Couldn't call for a yes-no-question - is the node 'human_interface' running?");
  }
  human_interface::SpeechRequest s_req;
  if (res.answer)
  {
    s_req.text_to_say = "That's fine. So I'm going to go on";
  }
  else
  {
      s_req.text_to_say = "That's a pity. I'm informing your smart home about that";
  }
  pub_speech_.publish(s_req);
  std::string task_result = "Sucessfully found the person based on face recognition";
  finishTask(true, task_result);
  node_state_ = exploration_hh::IDLE;
  ROS_INFO("Done with the task. Node state is IDLE");
  return;
}

void Exploration::photo()
{
  if (!image_taken_ && !image_running_)
  {
    ROS_INFO("Requested to take a picture");
    image_taken_ = false;
    image_running_ = true;
  }
  else if (image_running_ && !image_taken_)
  {
    ROS_INFO_THROTTLE(2,"Waiting for image");
  }
  else
  {
    //save internal
    exploration_hh::img_meta internal;
    internal.id = image_counter_;
    image_counter_++;
    internal.img = *tmp_picture.get();

    //search for the pose
    if (current_goal_->type == exploration_hh::RECOGNITION_GOAL)
    {
      for (unsigned int it = 0; it < detections_.detections.size(); it++)
        {
          if (current_goal_->detection_id == detections_.detections[it].header.seq)
          {
              internal.face_detections_pose = detections_.detections[it].latest_pose_map.pose;
              internal.face_detection_id = detections_.detections[it].header.seq;
              break;
          }
        }
    }
    else
    {
      for (unsigned int it = 0; it < obstacles_.obstacles.size(); it++)
      {
        if (current_goal_->detection_id == obstacles_.obstacles[it].header.seq)
        {
            internal.obstacle_pose = obstacles_.obstacles[it].center;
            internal.obstacle_id = obstacles_.obstacles[it].header.seq;
            break;
        }
      }
    }
    images_.push_back(internal);

    ROS_INFO("Saved image internally");
    //then save it to the harddisk
    cv_bridge::CvImagePtr cv_ptr;
    std::string filename = "/tmp/picture_";
    filename += boost::lexical_cast<std::string>(image_counter_);
    filename += "_task_";
    filename += boost::lexical_cast<std::string>((*task_goal_.getGoal()).task_id);
    if (current_goal_->type == exploration_hh::OBSTACLE_GOAL)
    {
      filename += "_obs_";
    }
    else
    {
      filename += "_rec_";
    }
    int det_id = current_goal_->detection_id;
    filename += boost::lexical_cast<std::string>(det_id);
    filename += ".png";
    try
    {
      cv_ptr = cv_bridge::toCvCopy(tmp_picture,sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& ex)
    {
      ROS_ERROR("cv_bridge exception when converting image: %s",ex.what());
      return;
    }
    try
    {
      cv::imwrite(filename,cv_ptr->image);
    }
    catch (cv_bridge::Exception& ex)
    {
      ROS_ERROR("imwrite exception when saving the image: %s",ex.what());
    }
    ROS_INFO("Saved an image called %s",filename.c_str());

    // set state
    current_goal_->done = true;
    ordered_goals_.erase(ordered_goals_.begin());
    setGoal();
  }

}

bool Exploration::getPlaces()
{
  std::vector< boost::shared_ptr<returnPlaces> > places;

  database_interface::FunctionCallObj parameter;
  parameter.name = "return_id_x_y_prob";
  if (!database_->callFunction(places,parameter))
  {
    ROS_ERROR("Unable to get goals from the database. Aborting task.");
    return false;
  }
  ROS_INFO("Retrieved %i places(s)",places.size());
  if (places.size() > 0) {
      ROS_INFO("These are:");
      ROS_INFO("key\tpos_x\tpos_y\tprobability");
    }
  exploration_hh::ExplorationGoal newGoal;
  newGoal.header.stamp = ros::Time::now();
  newGoal.header.frame_id = "/map";
  for (size_t i=0; i<places.size(); i++)
    {
      ROS_INFO("%i\t%f\t%f\t%f",places[i]->id_.data(),places[i]->pos_x_.data(),places[i]->pos_y_.data(),places[i]->prob_.data() );
      newGoal.pose.pose.position.x = places[i]->pos_x_.data();
      newGoal.pose.pose.position.y = places[i]->pos_y_.data();
      newGoal.pose.pose.position.z = 0;
      newGoal.probability = places[i]->prob_.data();
      newGoal.header.seq = goal_counter_;
      goal_counter_++;
      newGoal.done = false;
      exploration_goals_.push_back(newGoal);
    }
  return true;
}

bool Exploration::calcGoalPlace(geometry_msgs::Pose *robot_pose, geometry_msgs::Pose *int_place, geometry_msgs::Pose &goal)
{
  ROS_INFO("Robot Pos x: %f, y: %f",robot_pose->position.x,robot_pose->position.y);
  ROS_INFO("Obs Pos x: %f, y: %f",int_place->position.x,int_place->position.y);
  //the pythagorean theorem and the quadratic equation are used to calculate the position
  //the goal is to find a point 1m in front of the interesting place on the line between the robot pose and the place
  double distance = 1;
  //build a line
  double b = 0;
  double m = 0;
  m = (robot_pose->position.y - int_place->position.y) / (robot_pose->position.x - int_place->position.x);
  b = robot_pose->position.y - m*robot_pose->position.x;

//  //we want to know, how much we have to move into x-direction to be <distance> away from the point
//  // a = 1 + m*m
//  // b = -2*int_place_x - 2*m*int_place_y + 2*m*b
//  // c = int_place_x*int_place_x + int_place_y*int_place_y - int_place_x*2*b + b*b - distance*distance
//  double q_a;
//  double q_b;
//  double q_c;
//  q_a = 1+m*m;
//  q_b = -2*int_place->position.x - 2*m*int_place->position.y + 2*m*b;
//  q_c = int_place->position.x*int_place->position.x + int_place->position.y*int_place->position.y - int_place->position.x*2*b + b*b - distance*distance;
//  //check the value in the sqrt
//  double root = (q_b*q_b)-4*q_a*q_c;
//  if (root < 0)
//    {
//      ROS_WARN_THROTTLE(10,"Root was negative - that's a bad sign");
//      return false;
//    }

//  double x_1;
//  double x_2;
//  x_1 = (-q_b + sqrt(root)) / (2*q_a);
//  x_2 = (-q_b - sqrt(root)) / (2*q_a);
//  //check distances to the robot pose and pick the closest one
//  double d1;
//  double d2;
//  d1 = sqrt((robot_pose->position.x - x_1)*(robot_pose->position.x - x_1));
//  d2 = sqrt((robot_pose->position.x - x_2)*(robot_pose->position.x - x_2));
//  if (d1 < d2)
//  {
//    goal.position.x =  x_1;
//    goal.position.y = m*x_1+b;
//  }
//  else
//  {
//    goal.position.x = x_2;
//    goal.position.y = m*x_2+b;
//  }

  //quaternions
  tf::Vector3 axe (0,0,1);
  double angle = atan(m);
  //tf::Vector3 rob (robot_pose->position.x,robot_pose->position.y,robot_pose->position.z);
  //tf::Vector3 obs (int_place->position.x,int_place->position.y,int_place->position.z);
  tf::Quaternion q;
  q.setRotation(axe,angle);
  //Eigen::Quaternion<double> quat;
  //Eigen::QuaternionBase<double>::setFromTwoVectors(rob,obs);
  goal.orientation.w = q.w();
  goal.orientation.x = q.x();
  goal.orientation.y = q.y();
  goal.orientation.z = q.z();

  ROS_INFO("Calc Place Pos x: %f, y: %f. Orient qz: %f qw: %f",goal.position.x,goal.position.y,goal.orientation.z,goal.orientation.w);
  return true;
}

bool Exploration::checkIncomingGoal(robot_control::RobotTaskGoalConstPtr goal, robot_control::RobotTaskResult &res)
{
  //get configuration for that goal

  //get places for that goal
  getPlaces();
  return true;
}

bool Exploration::cleanupCancelledGoal(robot_control::RobotTaskResult &res)
{
  exploration_goals_.clear();
  res.success = false;
  res.end_result = "The goal got cancelled. There were " + boost::lexical_cast<std::string>(ordered_goals_.size()) + " left.";
  return true;
}

void Exploration::finishTask(bool success, std::string res)
{
  result_.end_result = res;
  if (success)
  {
    result_.success = true;
    task_goal_.setSucceeded(result_,"Normally finished");
  }
  else
  {
    result_.success = false;
    task_goal_.setSucceeded(result_,"Didn't find the person");
  }
  exploration_goals_.clear();
  goal_active_ = false;
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
      //goal.pose.pose = detections_.detections[it].last_seen_from.pose.pose;
      goal.type = exploration_hh::RECOGNITION_GOAL;
      goal.done = false;
      // there's some space to improve here
      goal.probability = 100;
      goal.detection_id = detections_.detections[it].header.seq;
      goal.pose.pose = detections_.detections[it].last_seen_from.pose.pose;
      geometry_msgs::PoseWithCovarianceStamped p;
      //ROS_INFO("Calculating place for detection goal with detection ID %i",goal.detection_id);
      calcGoalPlace(&detections_.detections[it].last_seen_from.pose.pose,&detections_.detections[it].latest_pose_map.pose,goal.pose.pose);
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
    if (obstacles_.obstacles[it].probability > accept_threshold_ && obstacles_.obstacles[it].present)
    {
      exploration_hh::ExplorationGoal goal;
      goal.header.frame_id = "/map";
      goal.header.seq = goal_counter_;
      goal_counter_++;
      goal.header.stamp = ros::Time::now();
      goal.pose.header.frame_id = "/map";
      goal.pose.header.seq = 0;
      goal.pose.header.stamp = ros::Time::now();
      goal.type = exploration_hh::OBSTACLE_GOAL;
      goal.done = false;
      goal.probability = obstacles_.obstacles[it].probability;
      goal.detection_id = obstacles_.obstacles[it].header.seq;
      // calculate the place to go - at least the direction
      double m = (obstacles_.obstacles[it].robot_pose.pose.pose.position.y - obstacles_.obstacles[it].center.position.y) /
          (obstacles_.obstacles[it].robot_pose.pose.pose.position.x - obstacles_.obstacles[it].center.position.x);
      double angle = atan(m);
      ROS_INFO("Angle is %f calulated from m %f",angle,m);
      tf::Vector3 axe (0,0,1);
      tf::Quaternion q;
      q.setRotation(axe,angle);
      goal.pose.pose.position = obstacles_.obstacles[it].robot_pose.pose.pose.position;
      ROS_INFO("Quaternion w is %f and z is %f",q.w(),q.z());
      goal.pose.pose.orientation.w = q.w();
      goal.pose.pose.orientation.z = q.z();

      //ROS_INFO("Calculating place for obstacle goal with detection ID %i",goal.detection_id);
//      if(!calcGoalPlace(&obstacles_.obstacles[it].robot_pose.pose.pose,&obstacles_.obstacles[it].center,goal.pose.pose))
//      {
//        continue;
//      }
      //goal.pose.pose = obstacles_.obstacles[it].robot_pose.pose.pose;
      obstacle_goals_.push_back(goal);
      ROS_INFO("Added new obstacle to the goal list with goal_number %i, goal id %i and probability %i",goal_counter_,goal.detection_id,goal.probability);

    }
  }
}

/*! This function is supposed to run endless on a certain frequency. It calls the state functions according to the nodes state.
    \todo Add other frequencies for some functions */

int Exploration::run()
{
  navfn::NavfnROS nav;

  ros::Rate r(10);
  while (ros::ok())
  {
    showGoals();
    //care about goals
    //if (task_goal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
    if (goal_active_)
    {
      calcGoals();
      // finish if there aren't any new goals
      if (ordered_goals_.empty())
      {
        std::string task_result = "Didn't find the person.";
        finishTask(false,task_result);
      }
      // or give feedback
      else
      {
        feedback_.percentage = (exploration_goals_.size()+obstacle_goals_.size()+recognition_goals_.size())/ordered_goals_.size();
        feedback_.intermediate_result = "Still running";
        task_goal_.publishFeedback(feedback_);
      }
      //detectionsstuff
      processDetections();
      processObstacles();

      //check if our current goal is still the prefered goal in the order
      if (current_goal_->detection_id != ordered_goals_.front()->detection_id)
      {
        //front goal is different - we have to set a new goal - in these state we're just driving or waiting - we can safely set a new goal
        if (node_state_ == exploration_hh::IDLE || node_state_ == exploration_hh::EXPLORATION || node_state_ == exploration_hh::OBSTACLE || node_state_ == exploration_hh::FACE_RECOGNITION)
        {
          setGoal();
        }
        else if (node_state_ == exploration_hh::PANORAMA)
        {
          // aborting the panorama takes extra effort
          std_msgs::Empty msg;
          pub_pano_stop_.publish(msg);
          ROS_INFO("Stopped panorama picture");
          panorama_taken_ = false;
          panorama_running_ = false;
          // abort the panorama action
          setGoal();
          }
        }
    }
    switch (node_state_)
    {
      case exploration_hh::IDLE :
        setGoal();
        break;
      case exploration_hh::CONFIRMATION :
        if (current_goal_->type == exploration_hh::RECOGNITION_GOAL)
        {
          confirmation_face();
        }
        else
        {
          confirmation_obstacle();
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
      case exploration_hh::FOUND :
        found();
        break;
      case exploration_hh::PHOTO :
        photo();
        break;
    }


    ros::spinOnce();
    r.sleep();
  }
    return 0;
}
