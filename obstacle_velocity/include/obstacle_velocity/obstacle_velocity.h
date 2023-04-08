#ifndef OBSTACLE_VELOCITY_H_
#define OBSTACLE_VELOCITY_H_

#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/PolygonStamped.h"
#include "visualization_msgs/Marker.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "nav_msgs/GetMap.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Quaternion.h"
#include <vector>
#include <cmath>

struct point
{
	float x ;
	float y ;
};

class obstacle_velocity
{
private:
	ros::NodeHandle n;
	ros::Subscriber Global_Path_Sub;
	ros::Subscriber Odom_Sub;
	ros::Subscriber Costmap_Sub;
	ros::Publisher MarkPub ;
	ros::Publisher costmap_pub ;
	ros::Publisher costmap_update;
	ros::ServiceClient map_client;
public:
	float x_odom , y_odom , w_odom , V_odom , path_yaw , yaw_angle;
	geometry_msgs::PoseStamped global_path;
	void Odom_callback(const nav_msgs::Odometry::Ptr& odom);
	void Global_path_callback(const nav_msgs::Path::Ptr& msg);
	void Global_costmap_callback(const nav_msgs::OccupancyGrid::Ptr& msg);
	geometry_msgs::Point world_to_map (geometry_msgs::Point world_coordinates) ;
	void crop_map( int destination_size_x, int region_size_x,int region_size_y , int orientation);
	void show_cropped_area();
	geometry_msgs::Point world_to_map(float x , float y);
	point map_coo(float x , float y);
	point std_map_coo(float x , float y);
	geometry_msgs::Point map_to_world(int x , int y) ;
	int creat_area();
	bool first_time ;
	std::vector<point> return_points();
	std::vector<point> return_points_area() ;
	std::vector<point> return_vertics(point pt1 , point pt2);
	std::vector<point> return_vertics_std(point pt1 , point pt2);
	void crop_map (point pt ,std::vector<point> A , std::vector<point> B , std::vector<point> C , std::vector<point> D) ;
	void rect_tester() ;
	int getIndex( int mx, int my) ;
	int getIndex_area( int mx, int my) ;
	std::vector<point> fill_area(std::vector<point> A , std::vector<point> B , std::vector<point> C , std::vector<point> D);
	geometry_msgs::Quaternion orientation ;
	bool compare_index(point pt1 , point pt2) ;

	

	obstacle_velocity()
	{
		ros::NodeHandle nh("~");
		std::string robot_name  ;
		nh.getParam("robot_name", robot_name);

		std::string costmap_topic = "/"+robot_name +"/move_base/global_costmap/costmap";
      	n.param("costmap_topic", costmap_topic, costmap_topic);
      	
      	std::string odom_topic = "/"+robot_name +"/Uqtr_Robot/diff_drive_controller/odom";
      	n.param("odom_topic", odom_topic, odom_topic);
      	
      	std::string global_path_topic = "/"+robot_name +"/move_base/TebLocalPlannerROS/global_plan"; 
      	n.param("global_path_topic", global_path_topic, global_path_topic);

      	std::string Marker_topic = "/"+robot_name +"/Obstacles_centroide_points";
      	n.param("Marker_topic", Marker_topic, Marker_topic);
		
		std::string cropped_map_topic = "/"+robot_name +"/cropped_map";
      	n.param("cropped_map_topic", cropped_map_topic, cropped_map_topic);
		
		this->Global_Path_Sub = n.subscribe(global_path_topic, 1, &obstacle_velocity::Global_path_callback,this);
		this->Odom_Sub = n.subscribe(odom_topic, 1 , &obstacle_velocity::Odom_callback , this);
		this->Costmap_Sub = n.subscribe(costmap_topic,1 , &obstacle_velocity::Global_costmap_callback, this);
		this->MarkPub = n.advertise<visualization_msgs::Marker>(Marker_topic, 1);
		this->costmap_pub = n.advertise<nav_msgs::OccupancyGrid>(cropped_map_topic,1) ;
		this->costmap_update = n.advertise<map_msgs::OccupancyGridUpdate>("/"+robot_name + "/cropped_map_update",1) ;
		this->map_client = n.serviceClient<std_srvs::Empty>("/"+robot_name + "/move_base/clear_costmaps");
		this->first_time = true ;
		
	};
protected: 
	nav_msgs::OccupancyGrid cropped_map;
	nav_msgs::OccupancyGrid cropped_map_no_static;
	nav_msgs::OccupancyGrid global_costmap ;
	nav_msgs::OccupancyGrid global_static_map ;
	map_msgs::OccupancyGridUpdate map_update ;
	nav_msgs::GetMap clear_map;
};
#endif

						/* 	^
						   / \
						    |
							|
							|
							|
							|	
							|                                            
							|							(x3 ,y3)				(x2 ,y2)
							|							    ________B___________
							|							   |                    |					
							|               			   |                    |
							|                              |                    |      										x1 =  x_odom + 1.5 * cos( pi/2 - teta)
							|							   |                    |											y1 =  y_odom - 1.5 * sin( pi/2 - teta)
							|				               C                    | 6.0m      
							|							   |                    |											x4 =  x_odom + 1.5 * cos( pi/2 + teta)
							|							   |                    A											y4 =  y_odom + 1.5 * sin( pi/2 + teta)
							|							   |                    |
							|							   |        3.0 m       |											x2/3 = x1/2 + 6 * cos (teta)	
							|                              |____D_______________|										  	y2/3 = y1/2 + 6 * sin (teta)
							|								  	(x_od , y_od)
							|						  (x4 ,y4)                (x1 ,y1)
							|
							|_______________________________________________________________________________________\
																													/
																														*/