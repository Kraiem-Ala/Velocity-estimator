#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include "obstacle_velocity/obstacles_msgs.h"
#include "obstacle_velocity/obstacle.h"
#include "ros/ros.h"
#include <cmath>
#include <fstream>
using namespace message_filters;
ros::Time begin ; 
std::ofstream data;
void callback(const obstacle_velocity::obstacles_msgs& robot1, const obstacle_velocity::obstacles_msgs& robot2)
{
  //std::cout << "robot_1 " << robot1.header.stamp.sec << " " <<robot1.header.stamp.nsec << " robot2 "<< robot2.header.stamp.sec<< " " <<robot2.header.stamp.nsec << std::endl ;
  begin = ros::Time::now();
  data << begin.sec<<"."<<begin.nsec << ";" ;
  data << robot1.robot_odometry.x <<";"<<robot1.robot_odometry.y << ";" << robot1.robot_orientation << ";"  << robot1.velocity << ";" << robot1.acceleration << ";" ;
  data << " ;" ;
  data << robot2.robot_odometry.x <<";"<<robot2.robot_odometry.y << ";" << robot2.robot_orientation << ";" << robot2.velocity << ";" << robot2.acceleration << "; ;" ;
  for (int i = 0 ; i != 4; ++i)
  {
    if ( i < robot1.obstacles.size())
      data << robot1.obstacles[i].obst_pose.x << ";" << robot1.obstacles[i].obst_pose.y << ";" << robot1.obstacles[i].obst_orientation << ";" << robot1.obstacles[i].obst_velocity  << "; ;";
    else
      data << 0 << ";" << 0 << ";" << 0 << ";" << 0  << "; ;";  
  }


  for (int i = 0 ; i != 4; ++i)
  {
    if ( i < robot2.obstacles.size())
      data << robot2.obstacles[i].obst_pose.x << ";" << robot2.obstacles[i].obst_pose.y << ";" << robot2.obstacles[i].obst_orientation << ";" << robot2.obstacles[i].obst_velocity  << "; ;";
    else
      data << 0 << ";" << 0 << ";" << 0 << ";" << 0  << "; ;";  
  }
   data << "\n" ;
  //ROS_INFO("time_now %i %i" , begin.sec , begin.nsec) ;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");
  ros::NodeHandle nh;
  data.open ("/home/kraiem/Desktop/PFE/simulation_workspace/src/obstacle_velocity/csv/data.csv");
  data << "Time ; Robot1_X ; Robot1_Y ; Robot1_Teta ; Robot1_Velocity ; Robot1_acceleration ; ; Robot2_X ; Robot2_Y ; Robot2_Teta ; Robot2_Velocity ; Robot2_acceleration ; ; Robot1_obstacle1 x ;Robot1_obstacle1 y ;Robot1_obstacle1_orientation ;Robot1_obstacle1_velocity ; ; Robot1_obstacle2 x ;Robot1_obstacle2 y ;Robot1_obstacle2_orientation ;Robot1_obstacle2_velocity ; ;Robot1_obstacle3 x ;Robot1_obstacle3 y ;Robot1_obstacle3_orientation ;Robot1_obstacle3_velocity ; ; Robot1_obstacle4 x ;Robot1_obstacle4 y ;Robot1_obstacle4_orientation ;Robot1_obstacle4_velocity ; ;Robot2_obstacle1 x ;Robot2_obstacle1 y ;Robot2_obstacle1_orientation ;Robot2_obstacle1_velocity ; ; Robot2_obstacle2 x ;Robot2_obstacle2 y ;Robot2_obstacle2_orientation ;Robot2_obstacle2_velocity ; ;Robot2_obstacle3 x ;Robot2_obstacle3 y ;Robot2_obstacle3_orientation ;Robot2_obstacle3_velocity ; ; Robot2_obstacle4 x ;Robot2_obstacle4 y ;Robot2_obstacle4_orientation ;Robot2_obstacle4_velocity \n" ;
  message_filters::Subscriber<obstacle_velocity::obstacles_msgs> robot1_sub(nh, "/robot1/speeds", 10);
  message_filters::Subscriber<obstacle_velocity::obstacles_msgs> robot2_sub(nh, "/robot2/speeds", 10);
  typedef sync_policies::ApproximateTime<obstacle_velocity::obstacles_msgs, obstacle_velocity::obstacles_msgs> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(3) ,robot1_sub, robot2_sub);
  sync.registerCallback(callback);

  ros::spin();
  data.close();
  return 0;
}