#ifndef EXPLORATION_HH_H
#define EXPLORATION_HH_H

#include <ros/ros.h>                                // needed for general ROS-support
#include <robot_control/RobotControlSimpleClient.h>  // class inherits from the simpleclient
#include <vector>                                   // needed to store the goals
#include <kobuki_msgs/ButtonEvent.h>                //TODO depricated?
#include <kobuki_msgs/Led.h>                        //TODO  --
#include <std_srvs/Empty.h>                         //TODO  --
#include <std_msgs/Empty.h>                         //TODO  --
#include <exploration_hh/ExplorationGoal.h>         // exploration messagetype
#include <move_base_msgs/MoveBaseAction.h>          // to make a move-base-client
#include <actionlib/client/simple_action_client.h>  //  -- 
#include <person_detector/DetectionObjectArray.h>   // to process and store the detections
#include <person_detector/ObstacleArray.h>          // to process and store the obstacles
#include <visualization_msgs/Marker.h>              // to show state in rviz
#include <sensor_msgs/Image.h>                      // to store the panorama images
#include <image_transport/image_transport.h>        // to subscribe to image topics
#include <cv_bridge/cv_bridge.h>                     // to save pictures
#include <opencv/cv.h>                              // to save pictures
#include <opencv/highgui.h>                         // to save pictures
#include <human_interface/RecognitionConfirmation.h>  // for confirmation of a person
#include <database_interface/postgresql_database.h> // to be able to use the database

//just here for non permanent purposes
namespace human_interface {

  struct speechRec {
    ros::Time time;
    std::string sentence;
  };

  enum yes_no_result {
    ANSWERED = 0,
    UNANSWERED = 1,
    WRONG_ANSWER = 2,
    BLOCKED_SPEAKER = 3
  };
}

namespace exploration_hh
{
  //! Enum to describe the states of the state machine
  /*! This node is a state machine switiching between these states */
  enum state
  {
    IDLE,                                                                 //!< Nothing to do. No more goals
    EXPLORATION,                                                          //!< The current goal is an exploration goal
    FACE_RECOGNITION,                                                     //!< The current goal is a goal for a recognized face
    CONFIRMATION,                                                         //!< The confirmation of an obstacle or a recognized face takes place
    OBSTACLE,                                                             //!< The current goal is an obstacle goal
    PANORAMA                                                              //!< A panorama picture is taken
  };
  //! Enum to distinguish different kind of goals
  /*! Every goal has to be of one kind.*/
  enum goal_type
  {
    EXPLORATION_GOAL = 0, /*!< A goal sent by the database */
    RECOGNITION_GOAL = 1, /*!< A goal for a recognized face */
    OBSTACLE_GOAL = 2     /*!< A goal for a recognized obstacle */
  };
  //! A struct to save an panorama image with metainformation
  /*! This can be used for later purposes. So this may be transfered into a ROS message.*/
  struct img_meta
  {
    int id;                                                              //!< A unique ID
    sensor_msgs::Image img;                                              //!< The picture
    geometry_msgs::Pose robot_pose;                                      //!< The pose of the robot when picture started to be taken
    std::vector<int> obstacles_;                                         //!< The IDs of the visible obstacles
    std::vector<geometry_msgs::Pose> obstacle_poses_;                    //!< The poses of the visible obstacles
    std::vector<int> face_detections_;                                   //!< The IDs of the visible face recognitions
    std::vector<geometry_msgs::Pose> face_detections_poses_;             //!< The poses of the visible face detections
  };
}
/*! The Exploration class is able to coordinate and influence the search for a person */


class Exploration : public RobotControlSimpleClient
{
private:
  //ROS and Markers
  //ros::NodeHandle n_;                                                    //!< Mandatory ROS-Nodehandler
  ros::Subscriber sub_exploration_goals_;                                //!< Subscriber for database-given exploration goals
  ros::Subscriber sub_detections_;                                       //!< Subscriber for person_detector face detections
  ros::Subscriber sub_obstacles_;                                        //!< Subscriber for person_detector obstacles
  ros::ServiceClient confirmation_client_;                               //!< Client for human_interface confirmation requests
  ros::ServiceClient yes_no_client_;                                     //!< Client for human_interface yes-no-questions
  ros::Publisher pub_speech_;                                            //!< Publisher for human_interface text-to-speech requests
  ros::Publisher pub_confirmations_;                                     //!< Publisher for person_detector confirmations
  image_transport::Subscriber *sub_img_;                                 //!< Subscriber for the panorama image topic
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* ac_;    //!< Client for navigation goals
  ros::Publisher pub_point_marker_;                                      //!< Publisher for rviz goal-cubes
  visualization_msgs::Marker point_marker_;                              //!< Internal storage for rviz goal-cubes. This is used to avoid the initialization.
  ros::Publisher pub_text_marker_;                                       //!< Publisher for rviz goal-text
  visualization_msgs::Marker text_marker_;                               //!< Internal storage for rviz goal-text

  //Database
  //! A database object for the connection to the PostgresqlDatabase
  /*! It holds the connection, can report about its state and do queries.*/
  database_interface::PostgresqlDatabase* database_;

  //own-variables
  std::vector<exploration_hh::ExplorationGoal> exploration_goals_;       //!< Internal storage for exploration goals
  std::vector<exploration_hh::ExplorationGoal> recognition_goals_;       //!< Internal storage for face recognition goals
  std::vector<exploration_hh::ExplorationGoal> obstacle_goals_;          //!< Internal storage for obstacle goals

  //! The ordered goals after they have been ordered by calcGoals()
  /*! The first item of this vector is always the next goal. If it changes, the new first item will be set as the next goal and the state changes according of the state of that goal.
  */
  std::vector<exploration_hh::ExplorationGoal*> ordered_goals_;
  exploration_hh::ExplorationGoal* current_goal_;                        //!< Holds a pointer to the current goal
  move_base_msgs::MoveBaseGoal move_base_goal_;                          //!< The navigation goal sent to the movebase

  //! Holds all detections of the person_detector
  /*! This object is frequently replaced by a new array of detections sent from the person_detector. Don't change anything or store any data in there. Use the extracted ExplorationGoal s or send a person_detector::confirmation message to the person detector if you need to update anything.*/
  person_detector::DetectionObjectArray detections_;

  //! Holds all detected obstacles of the person_detector
  /*! This object is frequently replaced by a new array of detections sent from the person_detector. Don't change anything or store any data in there. Use the extracted ExplorationGoal s or send a person_detector::confirmation message to the person detector if you need to update anything.*/
  person_detector::ObstacleArray obstacles_;
  int goal_counter_;                                                     //!< Counter to give every new goal a unique ID

  //! The name of the person, we're searching right now
  /*! If the search doesn't have a name, this string is empty */
  std::string name_;

  //! Saves the current state of the node
  /*! The node has several states defined in the exploration_hh::state enum. Whenever the state is changed, this variable has to be updated.*/
  exploration_hh::state node_state_;
  int speech_confirmation_id_;                                          //!< Counter to give human_interface confirmations unique IDs

  //! The threshold for accepting obstacle goals
  /*! Obstacle goals get on a scale from 0 to 100 points. It is possible to set this threshold to accept just interesting goals. Be careful setting this variable. If it's too low the search process will drown in obstacle goals. It it's too high it will cause false negatives. A reasonable value for quite a lot of goals is 40. A balanced value could be 50. A high value with some false negatives could be 60.
      \sa erase_threshold_ */
  int accept_threshold_;

  //! The threshold for deleting obstacle goals
  /*! If an obstacle turns out to be a wrong detection or if its size shrinks the corresponding obstacle goal should be deleted. It is possible to set a treshold for that, but be carefull setting it. It is recommended to keep a distance (e.g. 10 points) from Exploration::accept_treshold_ in order to avoid goals appearing and disappearing all the time.
      \sa accept_treshold_ */
  int erase_threshold_;

  //! A counter for the panorama images we take
  /*! This counter also affects the filenames */
  int image_counter_;
  std::vector<exploration_hh::img_meta> images_;                        //!< Storage of all images with metainformation
  image_transport::ImageTransport imageTransport_;                      //!< Needed to connect to an sensor_msgs::Image stream
  bool first_goal_set_;                                                 //!< Needed to avoid a segfault if we haven't set a goal before



  //own-functions are documented in the cpp
  //! The callback for a new exploration goal sent by the database
  /*! \param received_goal The new goal received from the database */
  void explorationGoalCallback(const exploration_hh::ExplorationGoal received_goal);

  //! The callback for the person_detector face detections
  /*! \param rec The received array of detections */
  void detectionsCallback(const person_detector::DetectionObjectArray rec);

  //! The callback for the person_detector obstacle detections
  /*! \param obs The received array of obstacles */
  void obstacleCallback(const person_detector::ObstacleArray obs);

  //! The callback for the panorama image
  /*! \param img The received image */
  void imageCallback(const sensor_msgs::Image::ConstPtr& img);

  //! This function orders all goals and creates a new ordered_goals_ vector
  /*! \return suceess or not */
  bool calcGoals();

  //! This functions prepares and publishes the information vor rviz
  void showGoals();

  //! Goes through the latest face detection array and updates and add goals
  /*! \sa recognition_goals_ detections_ */
  bool processDetections();

  //! Goes through the latest obstacle array and updates and adds the goals
  /*! \sa obstacle_goals_ obstacles_ */
  bool processObstacles();

  //! Sets the next goal of the ordered_goals vector and changes the state
  /*! \sa ordered_goals_ current_goal_ */
  void setGoal();

  //! Called function in the state CONFIRMATION with current_goal_ = OBSTACLE_GOAL
  int confirmation_face();

  //! Called function in the state CONFIRMATION with current_goal_ = RECOGNITION_GOAL
  int confirmation_obstacle();

  //! Called function in the state RECOGNITION
  int recognitionGoal_();

  //! Called function in the state EXPLORATION
  int explorationGoal_();

  //! Called function in the state OBSTACLE
  int obstacleGoal_();

  //! Called function in the state PANORAMA
  int panorama_();

  //! Used to get a list of places to a task
  /*! \return true on success */
  bool getPlaces();

  //! Check for incoming goals if database data is available
  /*! \return true if the new goal will be accepted */
  bool checkIncomingGoal(robot_control::RobotTaskGoalConstPtr goal, robot_control::RobotTaskResult &res);

  bool cleanupCancelledGoal(robot_control::RobotTaskResult &res);

  //! Cleanup after finishing the task
  void finishTask(bool success, std::string res);

public:

    //! Constructor - intializes the class
    Exploration(std::string task_server_name, std::string task_name);

    //! Run loop which runs endless
    int run();
};

#endif // EXPLORATION_HH_H
