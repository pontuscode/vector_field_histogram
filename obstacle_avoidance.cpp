#include "air_lab2/occ.h"
#include "air_lab2/histogram_grid.h"
#include "air_lab2/histogram_polar.h"
#include "air_lab2/vector_field_histogram.h"

#include <ros/node_handle.h>
#include <ros/service.h>
#include <ros/subscriber.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>


/*
For obstacle avoidance we need to know three central things:
  - Our position
    * Using odometry messages
  - Our surroundings
    * Using laserscan data
  - Our goal
    * Using path planner

We also need to be able to tell the robot to steer away from obstacles.
It seems fitting to use cmd_vel for this, but it is to be determined.
*/

class ObstacleAvoidance{

public:
  ObstacleAvoidance(const ros::NodeHandle& _nodeHandle) :
    m_nodeHandle(_nodeHandle), m_grid_cellsize(0.1),
    m_robot_size(1.0), m_histogram_dimensions(33)
  {
    vfh = new VectorFieldHistogram(100000, m_robot_size, 18, 5);
    lidar_sub = m_nodeHandle.subscribe("scan", 1, &ObstacleAvoidance::laser_callback, this);
    position_sub = m_nodeHandle.subscribe("/husky0/odometry", 1, &ObstacleAvoidance::update_position, this);
    goal_sub = m_nodeHandle.subscribe("/husky0/waypoints", 1, &ObstacleAvoidance::setGoal, this);
    pos_sub = m_nodeHandle.subscribe("/husky0/position_reached", 1, &ObstacleAvoidance::goalReached, this);
    vel_pub = m_nodeHandle.advertise<geometry_msgs::Twist>("/husky0/cmd_vel", 1);
    to_vel_control = m_nodeHandle.advertise<std_msgs::Empty>("/husky0/to_vel_control", 1);
    turn_control = m_nodeHandle.advertise<std_msgs::Float64>("/husky0/obstacle_avoidance_angular", 1);
  }

  void goalReached(const std_msgs::Empty& e){
    // Skip first
    if(this->goalCounter == this->goals.size()){
      this->goals.pop_front();
    }
    this->goalCounter--;
    //ROS_INFO_STREAM("REACHED GOAL");
  }

  void setGoal(const nav_msgs::Path::ConstPtr& _path){
    for(int i = 0; i < _path->poses.size(); i++){
      this->goals.push_back(_path->poses[i]);
    }
    // goalCounter is used to check if we have popped the first position_reached
    // or not. position_reached gets invoked before we start moving towards first goal.
    this->goalCounter = this->goals.size() + 1;
  }

  void update_position(const nav_msgs::Odometry::ConstPtr& _message){
    this->position_x = _message->pose.pose.position.x;
    this->position_y = _message->pose.pose.position.y;
    /* Arbitrary large number to avoid seg fault during start */
    double goal_x = 100.0;
    double goal_y = 100.0;
    if(this->goals.size() > 0){
      double goal_x = this->goals[0].pose.position.x;
      double goal_y = this->goals[0].pose.position.y;
    }

    tf::Quaternion q(_message->pose.pose.orientation.x,
                     _message->pose.pose.orientation.y,
                     _message->pose.pose.orientation.z,
                     _message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    this->driving_direction = yaw;
  }

  void change_direction(double delta_theta){
    double target;
    target = this->driving_direction - delta_theta - 2*PI;
    std_msgs::Float64 msg;
    msg.data = target;
    this->turn_control.publish(msg);
    /*msg.angular.z = target;
    msg.linear.x = 0.7;
    message_number++;
    if(message_number > 5){
      this->turning = false;
      message_number = 0;
    }
    if(!turning){
      this->vel_pub.publish(msg);
      std_msgs::Empty e;
      this->to_vel_control.publish(e);
      this->turning = true;
    }*/
  }

  void laser_callback(const sensor_msgs::LaserScanPtr& _message){
    double angle = _message -> angle_min;
    double distance;
    HistogramGrid histogram_grid{m_histogram_dimensions, m_grid_cellsize};
    HistogramPolar histogram_polar{5};
    for(int i = 0; i < _message -> ranges.size(); i++){
      distance = _message -> ranges[i];
      if(std::isnan(distance)) {
        distance = _message -> range_max;
      }
      // 1. Update the histogram grid
      histogram_grid.update_grid(distance, angle);
      angle += _message -> angle_increment;
    }

    // 2. Update the polar histogram
    histogram_polar.update_polar(&histogram_grid, this->position_x,
                                 this->position_y);
    // Smooth out peaks and valleys
    histogram_polar.smooth_histogram(5);
    double goal_x = this->goals[0].pose.position.x;
    double goal_y = this->goals[0].pose.position.y;
    double target_direction = atan2(goal_y - this->position_y,
                                    goal_x - this->position_x);
    // 3. Calculate the best direction to go based on target and obstacles

    double best_direction = this->vfh->calculate_direction(&histogram_polar,
                                                            target_direction,
                                                            this->driving_direction);
    //ROS_INFO_STREAM(best_direction);
    // If the best direction has changed from previous iteration
    if(this->goals.size() > 0){
      if(best_direction != 0.0){
        change_direction(best_direction);
      }
    }
  }

private:
  ros::Subscriber lidar_sub;
  ros::Subscriber position_sub;
  ros::Subscriber goal_sub;
  ros::Subscriber pos_sub;
  ros::Subscriber tw_sub;
  ros::Publisher vel_pub;
  ros::Publisher to_vel_control;
  ros::Publisher turn_control;
  ros::NodeHandle m_nodeHandle;
  VectorFieldHistogram* vfh;
  tf::TransformListener m_tfListener{};
  std::deque<geometry_msgs::PoseStamped> goals;
  double position_x, position_y = 0.0;
  double driving_direction = 0.0;
  int m_histogram_dimensions = 0;
  double m_robot_size, m_grid_cellsize;
  int goalCounter = 0;
  bool turning = false;
  int message_number = 0;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "ObstacleAvoidance");
  ros::NodeHandle n;

  ObstacleAvoidance ptd(n);

  ros::spin();
  return 0;
}
