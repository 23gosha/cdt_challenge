/*
 * NavigationDemo.cpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 *
 */

#include "grid_map_cdt/Challenge.hpp" 
#include <tf_conversions/tf_eigen.h>
#include <grid_map_cv/grid_map_cv.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <cv.h>
#include <highgui.h>
#include <limits>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen_conversions/eigen_msg.h>

using namespace grid_map;
using namespace std::chrono;


namespace grid_map_demos {

NavigationDemo::NavigationDemo(ros::NodeHandle& nodeHandle, bool& success)
    : nodeHandle_(nodeHandle),
      filterChain_("grid_map::GridMap"),
      demoMode_(false)
{
  if (!readParameters()) {
    success = false;
    return;
  }

  subscriber_ = nodeHandle_.subscribe(inputTopic_, 1, &NavigationDemo::callback, this);
  listener_ = new tf::TransformListener();

  outputGridmapPub_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/filtered_map", 1, true);
  footstepPlanRequestPub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/footstep_plan_request", 10);
  actionPub_ = nodeHandle_.advertise<std_msgs::Int16>("/action_cmd", 10);

  // Setup filter chain.
  if (!filterChain_.configure(filterChainParametersName_, nodeHandle)) {
    ROS_ERROR("Could not configure the filter chain!");
    success = false;
    return;
  }
  
  success = true;

  verbose_ = false;
  verboseTimer_ = true;
  plannerEnabled_ = true; // start enabled
}


NavigationDemo::~NavigationDemo()
{
}


bool NavigationDemo::readParameters()
{
  if (!nodeHandle_.getParam("input_topic", inputTopic_)) {
    ROS_ERROR("Could not read parameter `input_topic`.");
    return false;
  }
  nodeHandle_.param("filter_chain_parameter_name", filterChainParametersName_, std::string("grid_map_filters"));
  
  nodeHandle_.param("demo_mode", demoMode_, true);
  if (demoMode_)
    ROS_INFO("In demo mode [%d]. will use a hard coded gridmap bag and robot pose", int(demoMode_) );
  else
    ROS_INFO("In live mode [%d]. will listen for poses continuously", int(demoMode_) );

  return true;
}


void NavigationDemo::tic(){
  lastTime_ = high_resolution_clock::now();
}


std::chrono::duration<double> NavigationDemo::toc(){
  auto nowTime = high_resolution_clock::now();
  duration<double> elapsedTime = duration_cast<milliseconds>(nowTime - lastTime_);
  lastTime_ = nowTime;
  // std::cout << elapsedTime.count() << "ms elapsed" << std::endl;    
  return elapsedTime;
}


void NavigationDemo::callback(const grid_map_msgs::GridMap& message)
{
  if (!plannerEnabled_){
    std::cout << "planner enabled. at the goal? grab a beer!\n";
    return;
  }

  // The all important position goal - get the robot there
  Position pos_goal(14.5, 4.0);

  Eigen::Isometry3d pose_robot = Eigen::Isometry3d::Identity(); // ??
  if(demoMode_){ // demoMode

    Eigen::Vector3d robot_xyz = Eigen::Vector3d(0.0,0.0,0); //rpy
    Eigen::Vector3d robot_rpy = Eigen::Vector3d(0,0,0); //rpy

    pose_robot.setIdentity();
    pose_robot.translation() = robot_xyz;
    Eigen::Quaterniond motion_R = Eigen::AngleAxisd(robot_rpy(2), Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(robot_rpy(1), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(robot_rpy(0), Eigen::Vector3d::UnitX()); // order is ypr

    pose_robot.rotate( motion_R );

  }else{ // online

    tf::StampedTransform transform;
    try {
        listener_->waitForTransform("/odom", "/base", ros::Time(0), ros::Duration(10.0) );
        listener_->lookupTransform("/odom", "/base", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    tf::transformTFToEigen (transform, pose_robot);
    if (verbose_) std::cout << pose_robot.translation().transpose() << " current pose_robot\n";
  }


  Eigen::Isometry3d pose_chosen_carrot = Eigen::Isometry3d::Identity();
  bool sendCommand = planCarrot(message, pose_robot, pos_goal, pose_chosen_carrot);
  //std::cout << "pose_chosen_carrot: \n"<< pose_chosen_carrot.translation() << std::endl;

  if(sendCommand){
    // Send the carrot to the position controller
    geometry_msgs::PoseStamped m;
    m.header = message.info.header;
    tf::poseEigenToMsg (pose_chosen_carrot, m.pose);
    footstepPlanRequestPub_.publish(m);
  }

}

bool NavigationDemo::planCarrot(const grid_map_msgs::GridMap& message,
  Eigen::Isometry3d pose_robot, Position pos_goal,
  Eigen::Isometry3d& pose_chosen_carrot)
{
  std::cout << "Start carrot planner\n";
  tic();

  // Compute distance to the goal:
  Position pos_robot( pose_robot.translation().head(2) ); // ??
  double current_dist_to_goal = (pos_goal - pos_robot).norm();
  std::cout << "current distance to goal: " << current_dist_to_goal << std::endl;

  // If within 1.5m of goal - stop walking
  if (current_dist_to_goal < 1.5){
    // Determine a carrot pose: x and y from the above. z is the robot's height.
    // yaw in the direction of the carrot. roll,pitch zero
    Eigen::Vector4d carrot_relative_pose = pose_robot.matrix().inverse()*Eigen::Vector4d(pos_goal(0), pos_goal(1), 0, 1) ;
    double carrot_relative_theta = atan2(carrot_relative_pose(1),carrot_relative_pose(0));
    if (verbose_) std::cout << carrot_relative_pose.transpose() << " - relative carrot\n";
    if (verbose_) std::cout << carrot_relative_theta << " - relative carrot - theta\n";

    Eigen::Isometry3d pose_chosen_carrot_relative = Eigen::Isometry3d::Identity();
    pose_chosen_carrot_relative.translation() = Eigen::Vector3d( carrot_relative_pose(0),carrot_relative_pose(1),0);
    Eigen::Quaterniond motion_R = Eigen::AngleAxisd(carrot_relative_theta, Eigen::Vector3d::UnitZ()) // yaw
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) // pitch
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()); // roll

    pose_chosen_carrot_relative.rotate( motion_R );
    pose_chosen_carrot = pose_robot * pose_chosen_carrot_relative;
    std::cout << current_dist_to_goal << "m to goal. carrot is goal\n";
    // disable carrot planner
    plannerEnabled_ = false;

    // Send message to position_controller to start free gait action
    std_msgs::Int16 actionMsg;
    actionMsg.data = 1;
    ros::Duration(1.0).sleep(); // ??
    actionPub_.publish(actionMsg);

    return true;
  }


  // Convert message to map.
  GridMap inputMap;
  GridMapRosConverter::fromMessage(message, inputMap);
  // Apply filter chain.
  grid_map::GridMap outputMap;
  if (!filterChain_.update(inputMap, outputMap)) { // ??
    ROS_ERROR("Could not update the grid map filter chain!");
    return false;
  }
  //if (verboseTimer_) std::cout << toc().count() << "ms: filter chain\n";


  ////// Put your code here ////////////////////////////////////


  /*outputMap.add("carrots", Matrix::Zero(outputMap.getSize()(0), outputMap.getSize()(1)));
  Position carrotPositon = Position(0.0,1.0);
  Index carrotIndex;
  if(outputMap.getIndex(carrotPositon, carrotIndex)){
      //ROS_INFO("Carrot index identified");
      outputMap.at("carrots", carrotIndex) = 1.0;
  }
  //pos_goal = carrotPositon;*/


  float traversibility_threshold = 0.8;
  const size_t windowSize = 27; // how far should it keep from untraversible areas
  const double MAX_FLOAT = std::numeric_limits<float>::max();
  const double MAX_DIST = MAX_FLOAT;
  const grid_map::SlidingWindowIterator::EdgeHandling edgeHandling = grid_map::SlidingWindowIterator::EdgeHandling::EMPTY;
  outputMap.add("nan_removed", Matrix::Zero(outputMap.getSize()(0), outputMap.getSize()(1)));

  std::cout << "TIME1: " << toc().count() << std::endl;
  for(grid_map::SlidingWindowIterator iterator(outputMap, "elevation", edgeHandling, windowSize); !iterator.isPastEnd(); ++iterator){
      //++iterator;
      //++iterator;
      outputMap.at("nan_removed", *iterator) = iterator.getData().numberOfFinites() - windowSize*windowSize;
  }
  outputMap.add("non_traversible_removed", Matrix::Zero(outputMap.getSize()(0), outputMap.getSize()(1)));
  for(grid_map::SlidingWindowIterator iterator(outputMap, "traversability", edgeHandling, windowSize); !iterator.isPastEnd(); ++iterator){
      //++iterator;
      //++iterator;
      //++iterator;
      if(iterator.getData().minCoeffOfFinites() < traversibility_threshold)
        outputMap.at("non_traversible_removed", *iterator) = -1;
      else
        outputMap.at("non_traversible_removed", *iterator) = 0;
  }
  outputMap.add("allowed", Matrix::Zero(outputMap.getSize()(0), outputMap.getSize()(1)));
  for(GridMapIterator iterator(outputMap); !iterator.isPastEnd(); ++iterator){
      const Index index(*iterator);
      if(outputMap.at("non_traversible_removed", index) == 0 &&
              outputMap.at("nan_removed", index) == 0)
          outputMap.at("allowed", *iterator) = 1;
      else
          outputMap.at("allowed", *iterator) = 0;
  }
  std::cout << "TIME2: " << toc().count() << std::endl;

  outputMap.add("proximity_heat_map", Matrix::Zero(outputMap.getSize()(0), outputMap.getSize()(1)));
  for(GridMapIterator iterator(outputMap); !iterator.isPastEnd(); ++iterator){
      Index index(*iterator);
      Position pos_cell;
      outputMap.getPosition(index, pos_cell);
      if(outputMap.at("allowed", *iterator))
        outputMap.at("proximity_heat_map", index) = (pos_goal - pos_cell).norm();
      else
        outputMap.at("proximity_heat_map", index) = MAX_DIST; // set to 2 for visualising
  }

  // Carrot following mode
  Position currentPosition = pos_robot;
  Index currentPositionIndex;
  if(outputMap.getIndex(currentPosition, currentPositionIndex)){
      //ROS_INFO("Current position index identified\n");
  }

  // Setting buffer size
  int submapBufferSizeInt = 43;
  Index submapStartIndex = currentPositionIndex - Index((submapBufferSizeInt - 1)/2,(submapBufferSizeInt - 1)/2);
  Index submapBufferSize(submapBufferSizeInt,submapBufferSizeInt);


  Index minValIndex;
  float minVal = MAX_FLOAT;
  for (grid_map::SubmapIterator iterator(outputMap, submapStartIndex, submapBufferSize);
       !iterator.isPastEnd();++iterator){
      //std::cout << outputMap.at("proximity_heat_map", *iterator) << std::endl;
      if(outputMap.at("proximity_heat_map", *iterator) < minVal){
          minVal = outputMap.at("proximity_heat_map", *iterator);
          minValIndex = Index(*iterator);
      }
  }
  Position chosenCarrot;
  //std::cout << "Moving to " << minValIndex << std::endl;
  outputMap.getPosition(minValIndex, chosenCarrot);

  /*
  // Finding how many meters per index (turns out 0.02)
  Index minIndexTest = Index(0,0);
  Index maxIndexTest = Index(0,100);
  Position minPosTest;
  Position maxPosTest;
  outputMap.getPosition(minIndexTest, minPosTest);
  outputMap.getPosition(maxIndexTest, maxPosTest);
  std::cout << "DISTANCE: " << (maxPosTest - minPosTest) << std::endl;*/


  // For how long it goes after choosing direction
  //float stepSize = 3;
  //chosenCarrot = chosenCarrot + stepSize*(chosenCarrot - currentPosition);

  pose_chosen_carrot.translation() = Eigen::Vector3d(chosenCarrot(0),chosenCarrot(1),0);
/*
  // Visualising path mode
  Position currentPosition = Position(1.0,0.0);
  Index currentPositionIndex;
  if(outputMap.getIndex(currentPosition, currentPositionIndex)){
      ROS_INFO("Current position index identified\n");
  }
  outputMap.add("path", Matrix::Zero(outputMap.getSize()(0), outputMap.getSize()(1)));
  const int pathIterationsNumber = 200;
  for(int i=0;i<pathIterationsNumber;++i){

      int submapBufferSizeInt = 7;
      Index submapStartIndex = currentPositionIndex - Index((submapBufferSizeInt - 1)/2,(submapBufferSizeInt - 1)/2);
      Index submapBufferSize(submapBufferSizeInt,submapBufferSizeInt);

      Index minValIndex;
      float minVal = MAX_FLOAT;
      //ROS_INFO("Current position index: %d %d\n", currentPositionIndex(0), currentPositionIndex(1));
      for (grid_map::SubmapIterator iterator(outputMap, submapStartIndex, submapBufferSize);
           !iterator.isPastEnd();++iterator){
          if(outputMap.at("proximity_heat_map", *iterator) < minVal){
              minVal = outputMap.at("proximity_heat_map", *iterator);
              minValIndex = Index(*iterator);
          }
          //float val = outputMap.at("proximity_heat_map", *iterator);
          //ROS_INFO("%d %d %f\n", Index(*iterator)(0), Index(*iterator)(1), val);
      }
      //ROS_INFO("\n\n");
      currentPositionIndex = minValIndex;
      outputMap.at("path", currentPositionIndex) = 2;
  }*/





  ////// Put your code here ////////////////////////////////////


  // Publish filtered output grid map.
  grid_map_msgs::GridMap outputMessage;
  GridMapRosConverter::toMessage(outputMap, outputMessage);
  outputGridmapPub_.publish(outputMessage);
  if (verbose_) std::cout << "finished processing\n";
  if (verboseTimer_) std::cout << toc().count() << "ms: publish output\n";

  //std::cout << "finish - carrot planner\n\n";
  

  // REMOVE THIS WHEN YOUR ARE DEVELOPING ----------------
  // create a fake carrot - replace with a good carrot
  //std::cout << "REPLACE FAKE CARROT!\n";
  //pose_chosen_carrot.translation() = Eigen::Vector3d(2.0,0,0);
  //
  std::cout << "Carrot Pos:\n" << pose_chosen_carrot.translation() << std::endl;
  std::cout << "Robot Pos: \n" << chosenCarrot << std::endl;
  //ROS_INFO("from planCarrot pos_chosen_carrot %d %d %d \n",pose_chosen_carrot.translation());
  // REMOVE THIS -----------------------------------------

  return true;
}

} /* namespace */
