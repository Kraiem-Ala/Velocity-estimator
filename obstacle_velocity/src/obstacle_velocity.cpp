#include "obstacle_velocity/obstacle_velocity.h"
#include <tf/tf.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Int8.h"
#include<algorithm> // for copy() and assign()
#include<iterator> // for back_inserter
#include<vector> // for vectors
#include "map_msgs/OccupancyGridUpdate.h"
#include <typeinfo>
const double pi = 3.14159265359;


void obstacle_velocity::Odom_callback(const nav_msgs::Odometry::Ptr& odom)
{
	tf::Pose pose;
	tf::poseMsgToTF(odom->pose.pose, pose);
  	this->yaw_angle = tf::getYaw(pose.getRotation());
  	this->x_odom=odom->pose.pose.position.x;
  	this->y_odom=odom->pose.pose.position.y;
  	this->w_odom=odom->twist.twist.angular.z;
  	this->V_odom=odom->twist.twist.linear.x;
  	this->orientation = odom->pose.pose.orientation ;

}

void obstacle_velocity::Global_path_callback(const nav_msgs::Path::Ptr& msg)
{
	if(msg->poses.size() < 80)
		this->global_path=msg->poses.back() ;
	else
		this->global_path = msg->poses[80];
	tf::Pose pose;
	tf::poseMsgToTF(this->global_path.pose, pose);
  	this->path_yaw = tf::getYaw(pose.getRotation());
}
void obstacle_velocity::Global_costmap_callback(const nav_msgs::OccupancyGrid::Ptr& msg)
{
	
	if (this->first_time)
	{
		this->global_static_map = *msg ;
		this->first_time = false ;
		ROS_INFO("Map service called successfully");
	}

	this->global_costmap = *msg;
	tf2::Quaternion myQuaternion;
	myQuaternion.setRPY( 0, 0, 0 );  // Create this quaternion from roll/pitch/yaw (in radians)
	myQuaternion.normalize(); 
	geometry_msgs::Quaternion geo_quat = tf2::toMsg(myQuaternion);
	this->cropped_map.info.origin.orientation = geo_quat ;

	
	//strat_point.y= this->global_path.pose.position.x ;
	int size_x = static_cast<int>(10/this->global_costmap.info.resolution);
	int size_y = static_cast<int>(10/this->global_costmap.info.resolution);
	this->cropped_map.info.width=size_x;
	this->cropped_map.info.height=size_y;
	this->cropped_map.info.origin.position.z = 0;
	//this->cropped_map.info.origin.orientation = this->orientation ;
	this->cropped_map.info.map_load_time = this->global_costmap.info.map_load_time;
	this->cropped_map.info.resolution = this->global_costmap.info.resolution;
	this->cropped_map.header = this->global_costmap.header;
	//this->cropped_map.info.height= (int)size_y;
	

	this->crop_map(size_x , size_x , size_y , this->creat_area());
	//this->rect_tester() ;
}

void obstacle_velocity::crop_map( int destination_size_x, int region_size_x,int region_size_y , int orientation)
{
  tf2::Quaternion myQuaternion;
  geometry_msgs::Point lower_left_point ;
  int index  , j = 0;
  /*switch(orientation){
    // now, we'll copy the source map into the destination map
  case (0) :
  {*/
  	lower_left_point= this->world_to_map(this->x_odom-5 , this->y_odom -5);
    index = static_cast<int>(this->global_costmap.info.width * (lower_left_point.y-1) + lower_left_point.x) ;
  	auto  start_source = this->global_costmap.data.begin() + index ;
    auto end_source= start_source + region_size_x ;
  	auto static_source = this->global_static_map.data.begin() + index ;
    auto end_static= static_source + region_size_x ;
    auto start_destination = this->cropped_map.data.begin();
    for (int i = 0; i < region_size_y ; ++i)
    {
    	//std::cout << lower_left_point.y + i << std::endl;
    	copy(start_source, end_source , back_inserter(this->cropped_map.data));
    	
    	if (lower_left_point.y + i < 1) 
    		{
    		std::fill(this->cropped_map.data.end()-region_size_y  , this->cropped_map.data.end()  , -1) ;
    		}

    	if (lower_left_point.x + region_size_x > this->global_costmap.info.width)
    	{
    		int cells_exceded = (lower_left_point.x + region_size_x ) - this->global_costmap.info.width  ;
    		std::fill(this->cropped_map.data.end() - cells_exceded  , this->cropped_map.data.end(), -1);
    	}
    	if (lower_left_point.y + i > this->global_costmap.info.height  )
    	{
    		
    		copy(start_source , end_source , back_inserter(this->cropped_map.data));
    		std::fill(start_destination , start_destination + region_size_y , -1);
    		
    	}
    	
    	for (auto t = static_source; t != end_static  ; ++t)
    	{
  			if ((int)this->cropped_map.data[j] == -1 )
  			{
  				this->cropped_map_no_static.data.push_back(-1);
  			}
  			else if (((int)this->cropped_map.data[j] - (int) *t) < 50 )
  			{
  				this->cropped_map_no_static.data.push_back(0) ;
  			}
  			else
  			{
    			this->cropped_map_no_static.data.push_back( (int)this->cropped_map.data[j] - (int) *t) ;
				}
				++j ;
    	}

    	start_source += this->global_costmap.info.width ;
    	end_source = start_source + region_size_x ;
    	static_source += this->global_costmap.info.width ;
    	end_static= static_source + region_size_x ;
    	start_destination += region_size_x ;
    }
    myQuaternion.setRPY( 0, 0, 0 );  // Create this quaternion from roll/pitch/yaw (in radians)
		myQuaternion.normalize(); 
		geometry_msgs::Quaternion geo_quat = tf2::toMsg(myQuaternion);
		this->cropped_map.info.origin.orientation = geo_quat ;
		this->cropped_map.info.origin.position = this->map_to_world(lower_left_point.x , lower_left_point.y);
		this->cropped_map.info.origin.position.y = this->y_odom -5;
		this->cropped_map.info.origin.position.x = this->x_odom -5;
		
  
  /*case(1) :
  {
  	lower_left_point= this->world_to_map(this->x_odom - 5.0  , this->y_odom -5.0);
  	index = static_cast<int>(this->global_costmap.info.width * (lower_left_point.y -1) + lower_left_point.x) ;
  	auto start_source = this->global_costmap.data.begin() + static_cast<int>(this->global_costmap.info.width * (lower_left_point.y -1) + lower_left_point.x ) ;
    auto end_source= start_source + region_size_x ;
    auto start_destination = this->cropped_map.data.begin();
    auto static_source = this->global_static_map.data.begin() + index ;
    auto end_static= static_source + region_size_x ;

    for (int i = 0; i < region_size_y ; ++i)
    {
    	if (lower_left_point.x < 0)
    	{
    		
    		copy(start_source , end_source , back_inserter(this->cropped_map.data));
    		std::fill(start_destination  , start_destination + std::abs(lower_left_point.x) , -1);
    		start_source += this->global_costmap.info.width ;
    		end_source = start_source + region_size_x ;
	    	start_destination += region_size_x ;
    	}
   	else 
    	{
	  		copy(start_source, end_source , back_inserter(this->cropped_map.data));
	    	start_source += this->global_costmap.info.width ;
	    	end_source = start_source + region_size_x ;
	    	start_destination += region_size_x ;
    	}
    	for (auto t = static_source; t != end_static  ; ++t)
    	{
  			if ((int)this->cropped_map.data[j] == -1 )
  			{
  				this->cropped_map_no_static.data.push_back(-1);
  			}
  			else if (((int)this->cropped_map.data[j] - (int) *t) < 20 )
  			{
  				this->cropped_map_no_static.data.push_back(0) ;
  			}
  			else
  			{
    			this->cropped_map_no_static.data.push_back( (int)this->cropped_map.data[j] - (int) *t) ;
				}
				++j ;
    	}
    	static_source += this->global_costmap.info.width ;
    	end_static= static_source + region_size_x ;
  	
  	}
  	myQuaternion.setRPY( 0, 0, 0 );  // Create this quaternion from roll/pitch/yaw (in radians)
		myQuaternion.normalize(); 
		geometry_msgs::Quaternion geo_quat = tf2::toMsg(myQuaternion);
		this->cropped_map.info.origin.orientation = geo_quat ;
		this->cropped_map.info.origin.position.x = this->x_odom -5.0  ;
		this->cropped_map.info.origin.position.y = this->y_odom -5.0;
  	break;
  }
  case(3) :
  {
		lower_left_point= this->world_to_map(this->x_odom  - 5  , this->y_odom -5  );
  	index = static_cast<int>(this->global_costmap.info.width * (lower_left_point.y -1) + lower_left_point.x) ;
  	auto start_source = this->global_costmap.data.begin() + static_cast<int>(this->global_costmap.info.width * (lower_left_point.y -1) + lower_left_point.x ) ;
    auto init_start = start_source ;
    auto end_source= start_source + region_size_y ;
    auto start_destination = this->cropped_map.data.begin();
    auto static_source = this->global_static_map.data.begin() + index ;
    auto end_static= static_source + region_size_y ;
    for (int i = 0; i < region_size_x ; ++i)
    {
    	if (lower_left_point.x < 0)
    	{
    		copy(start_source , end_source , back_inserter(this->cropped_map.data));
    		std::fill(start_destination  , start_destination + std::abs(lower_left_point.x) , -1);
    	}
    	if (lower_left_point.y + i > this->global_costmap.info.height  )
    	{
    		
    		copy(start_source , end_source , back_inserter(this->cropped_map.data));
    		std::fill(start_destination , start_destination + region_size_y , -1);
    		//std::cout <<"eeeeeeeeeeeeeeeeee";
    		
    	}
     	else 
    	{
	  		copy(start_source, end_source , back_inserter(this->cropped_map.data));
	  	}
    	
    	for (auto t = static_source; t != end_static  ; ++t)
    	{
  			if ((int)this->cropped_map.data[j] == -1 )
  			{
  				this->cropped_map_no_static.data.push_back(-1);
  			}
  			else if (((int)this->cropped_map.data[j] - (int) *t) < 20 )
  			{
  				this->cropped_map_no_static.data.push_back(0) ;
  			}
  			else
  			{
    			this->cropped_map_no_static.data.push_back( (int)this->cropped_map.data[j] - (int) *t) ;
				}
				++j ;
    	}
    	start_source += this->global_costmap.info.width ;
  		end_source = start_source + region_size_y ;
    	start_destination += region_size_y ;
    	static_source += this->global_costmap.info.width ;
    	end_static= static_source + region_size_y ;

  	
  	}
  	myQuaternion.setRPY( 0, 0, 0 );  // Create this quaternion from roll/pitch/yaw (in radians)
		myQuaternion.normalize(); 
		geometry_msgs::Quaternion geo_quat = tf2::toMsg(myQuaternion);
		this->cropped_map.info.origin.orientation = geo_quat ;
		this->cropped_map.info.origin.position.x = this->x_odom -5.0   ;
		this->cropped_map.info.origin.position.y = this->y_odom -5.0;
		this->cropped_map.info.width=region_size_y;
		this->cropped_map.info.height=region_size_x;
  	break;
  }
  case(2) :
  {
  	lower_left_point= this->world_to_map(this->x_odom  -5  , this->y_odom  - 5);
  	index = static_cast<int>(this->global_costmap.info.width * (lower_left_point.y -1) + lower_left_point.x) ;
  	auto start_source = this->global_costmap.data.begin() + static_cast<int>(this->global_costmap.info.width * (lower_left_point.y -1) + lower_left_point.x ) ;
    auto end_source= start_source + region_size_y ;
    auto init_start = start_source ;
    auto start_destination = this->cropped_map.data.begin();
    auto static_source = this->global_static_map.data.begin() + index ;
    auto end_static= static_source + region_size_y ;
    for (int i = 0; i < region_size_x ; ++i)
    {
    	if (lower_left_point.y < 0) 
    	{
    		std::fill(start_destination  , start_destination + region_size_y , -1);
    		lower_left_point.y += 1;
    	}
    	else if (lower_left_point.x < 0)
    	{
    		copy(start_source , end_source , back_inserter(this->cropped_map.data));
    		std::fill(start_destination  , start_destination + std::abs(lower_left_point.x) , -1);
    	}
     	else 
    	{
	  		copy(start_source, end_source , back_inserter(this->cropped_map.data));
	  	}
    	for (auto t = static_source; t != end_static  ; ++t)
    	{
  			if ((int)this->cropped_map.data[j] == -1 )
  			{
  				this->cropped_map_no_static.data.push_back(-1);
  			}
  			else if (((int)this->cropped_map.data[j] - (int) *t) < 20 )
  			{
  				this->cropped_map_no_static.data.push_back(0) ;
  			}
  			else if ((int)this->cropped_map.data[j] == 100 )
  			{
  				this->cropped_map_no_static.data.push_back(0) ;
  			}
  			else
  			{
    			this->cropped_map_no_static.data.push_back( (int)this->cropped_map.data[j] - (int) *t) ;
				}
				++j ;
    	}
    	start_source += this->global_costmap.info.width ;
  		end_source = start_source + region_size_y ;
    	start_destination += region_size_y ;
    	static_source += this->global_costmap.info.width ;
    	end_static= static_source + region_size_y ;

  	
  	}

  	myQuaternion.setRPY( 0, 0, 0 );  // Create this quaternion from roll/pitch/yaw (in radians)
		myQuaternion.normalize(); 
		geometry_msgs::Quaternion geo_quat = tf2::toMsg(myQuaternion);
		this->cropped_map.info.origin.orientation = geo_quat ;
		this->cropped_map.info.origin.position.x = this->x_odom -5.0   ;
		this->cropped_map.info.origin.position.y = this->y_odom  - 5.0;
		this->cropped_map.info.width=region_size_y;
		this->cropped_map.info.height=region_size_x;
  	break;
  }
}*/
this->cropped_map.info.map_load_time = this->global_costmap.info.map_load_time;
this->cropped_map.info.resolution = this->global_costmap.info.resolution;
this->cropped_map.header = this->global_costmap.header;
this->cropped_map_no_static.info = this->cropped_map.info ;
this->cropped_map_no_static.header = this->cropped_map.header ;



this->show_cropped_area();
}

void obstacle_velocity::show_cropped_area()
{
	std_srvs::Empty srv;
	this->costmap_pub.publish(this->cropped_map_no_static);
	map_msgs::OccupancyGridUpdate update ;
	update.header=this->cropped_map.header;
	update.x=this->cropped_map.info.origin.position.x;
	update.y=this->cropped_map.info.origin.position.y;
	update.width=this->cropped_map.info.width;
	update.height=this->cropped_map.info.height;
	update.data = this->cropped_map_no_static.data ;
	this->costmap_update.publish(update);
	this->cropped_map.data.clear();
	this->cropped_map_no_static.data.clear();
	this->map_client.call(srv);
}
geometry_msgs::Point obstacle_velocity::world_to_map(float x , float y)
{
	geometry_msgs::Point point ;
	
	point.x = (int)((x - this->global_costmap.info.origin.position.x) / this->global_costmap.info.resolution);
	point.y = (int)((y - this->global_costmap.info.origin.position.y) / this->global_costmap.info.resolution);
	
	return(point) ;
}

geometry_msgs::Point obstacle_velocity::map_to_world(int x , int y)
{
	geometry_msgs::Point point ;
	
	point.x = (float) (x * this->global_costmap.info.resolution + this->global_costmap.info.origin.position.x);
	point.y = (float) (y * this->global_costmap.info.resolution + this->global_costmap.info.origin.position.y);
	
	return(point) ;
}


int obstacle_velocity::creat_area()
/*
l'orientation de la zone est proportionelle aux cordonnées de l'odometrie et point finale
0 --> la zone d'intêret est celle devant le robot 
1 --> la zone d'intêret est celle deriére le robot
2 --> la zone d'intêret est celle à droite du robot
3 --> la zone d'intêret est celle à gauche du robot

*/
{
	if ((this->yaw_angle > -1.0472) and (this->yaw_angle < 1.0472))
	{
		// crop_avant robot
		return(0);
	} 
	else if (std::abs(this->yaw_angle) > 2.0944)
	{
		//crop deriere robot
		return(1) ;
	}
	else if ((this->yaw_angle< -1.0472) and (this->yaw_angle > -2.0944))
	{
		/*if (this->w_odom > 0.6 )
		{
			return(0);
		}
		else if (this->w_odom < -0.6)
		{
			return(1);
		}
		else 
		{*/
			return(2);
		//}	
	}
	else if ((this->yaw_angle > 1.0472) and (this->yaw_angle < 2.0944))
	{
		/*if (this->w_odom > 0.6 )
		{
			return(1);
		}
		else if (this->w_odom < -0.6)
		{
			return(0);
		}
		else 
		{
			return(3);
		}*/
		return(3);
	}
}
/// ShIT is about to Become REal 
/*
*****************************************||||||||||||||*****************************************
*****************************************||||||||||||||*****************************************
*/
std::vector<point> obstacle_velocity::return_points()
{
	std::vector<point> area ;
	point pt ;
	float x ,y , x1 , x4 , y1 , y4;
	//x1 , y1 
	x = this->x_odom + std::cos((pi/2) - this->yaw_angle) * 1.5 ;
	y = this->y_odom - std::sin((pi/2) - this->yaw_angle) * 1.5 ;
	x1 = x;
	y1 = y ;
	pt.x = x ;
	pt.y = y ;
	area.push_back(pt);
	//x4 , y4 
	x = this->x_odom + std::cos((pi/2) + this->yaw_angle) * 1.5 ;
	y = this->y_odom + std::sin((pi/2) + this->yaw_angle) * 1.5 ;
	x4 = x;
	y4 = y ;
	pt.x = x ;
	pt.y = y ;
	this->cropped_map.info.origin.position.x = x1   ;
	this->cropped_map.info.origin.position.y = y1;
	area.push_back(pt);
	//x2 , y2 
	x = x1 + std::cos(this->yaw_angle) * 6 ;
	y = y1+ std::sin(this->yaw_angle) * 6 ;
	pt.x = x ;
	pt.y = y ;
	area.push_back(pt);
	//x3 , y3 
	x = x4 + std::cos(this->yaw_angle) * 6 ;
	y = y4 + std::sin(this->yaw_angle) * 6 ;
	pt.x = x ;
	pt.y = y ;
	area.push_back(pt);
	for (std::vector<point>::iterator i = area.begin(); i != area.end(); ++i)
	{
		std::cout << i->x << "/" << i->y << std::endl ;
	}
	return (area);
}
std::vector<point> obstacle_velocity::return_points_area()
{
	std::vector<point> area ;
	point pt ;
	float x ,y , x1 , x4 , y1 , y4 , xo , yo ;
	yo = this->y_odom - std::sin((pi/2) - this->yaw_angle) * 1.5 ;
	xo = this->x_odom + std::cos((pi/2) + this->yaw_angle) * 1.5 ;
	//x1 , y1 
	x = this->x_odom + std::cos((pi/2) - this->yaw_angle) * 1.5 ;
	y = this->y_odom - std::sin((pi/2) - this->yaw_angle) * 1.5 ;
	x1 = x;
	y1 = y ;
	pt.x = x - xo ;
	pt.y = y - yo;
	//area.push_back(pt);
	//x4 , y4 
	x = this->x_odom + std::cos((pi/2) + this->yaw_angle) * 1.5 ;
	y = this->y_odom + std::sin((pi/2) + this->yaw_angle) * 1.5 ;
	x4 = x;
	y4 = y ;
	pt.x = x - xo ;
	pt.y = y - yo;
	this->cropped_map.info.origin.position.x = x1   ;
	this->cropped_map.info.origin.position.y = y1;
	//area.push_back(pt);
	//x2 , y2 
	x = x1 + std::cos(this->yaw_angle) * 6 ;
	y = y1+ std::sin(this->yaw_angle) * 6 ;
	pt.x = x - xo ;
	pt.y = y - yo;
	area.push_back(pt);
	//x3 , y3 
	x = x4 + std::cos(this->yaw_angle) * 6 ;
	y = y4 + std::sin(this->yaw_angle) * 6 ;
	pt.x = x - xo ;
	pt.y = y - yo;
	area.push_back(pt);
	std::cout << xo << "/" << yo << std::endl ;
	for (std::vector<point>::iterator i = area.begin(); i != area.end(); ++i)
	{
		std::cout << i->x << "/" << i->y << std::endl ;
	}
	return (area);
}

std::vector<point> obstacle_velocity::return_vertics(point pt1 , point pt2)
{
	std::vector<point> vertic ;
	float dx , dy ,a , b  ;
	float start , end , x , y ;
	point pt ;
	if (pt1.x == pt2.x)
	{
		if (pt1.y < pt2.y) 
		{ 
			start = pt1.y ;
			end = pt2.y ;
		}
		else 
		{
			start = pt2.y ;
			end = pt1.y ;
		}
		int i =0 ;
		while (start < end )
		{
			x = pt1.x ;
			y = start ; 
			pt = this->map_coo(x , y) ;
			vertic.push_back(pt); 
			start += 0.01 ;
			++i ;
		}
	} 
	else 
	{
		dy = std::abs((pt2.y - pt1.y)/(pt2.x - pt1.x)) ;
		dx = std::abs((pt2.x - pt1.x)/(pt2.y - pt1.y)) ;
		if ( dx > dy) 
		{
			a = (pt2.y - pt1.y)/(pt2.x - pt1.x) ;
			b = pt1.y - (pt1.x * a) ;

			int i =0 ;
			if (pt1.x < pt2.x) 
			{ 
				start = pt1.x ;
				end = pt2.x ;
			}
			else 
			{
				start = pt2.x ;
				end = pt1.x ;
			}


			while (start < end )
			{
				x = start ;
				y = a * x + b ; 
				pt = this->map_coo(x , y) ;
				vertic.push_back(pt); 
				start += this->global_costmap.info.resolution ;
				++i ;
			}
		}
		else 
		{
			a = (pt2.x - pt1.x)/(pt2.y - pt1.y) ;
			b = pt1.x - (pt1.y * a) ;

			int i =0 ;
			if (pt1.y < pt2.y) 
			{ 
				start = pt1.y ;
				end = pt2.y ;
			}
			else 
			{
				start = pt2.y ;
				end = pt1.y ;
			}


			while (start < end )
			{
				x = start ;
				y = a * x + b ; 
				pt = this->map_coo(y , x) ;
				vertic.push_back(pt); 
				start += this->global_costmap.info.resolution ;
				++i ;
			}
		}
	}
	return (vertic) ;	
}

std::vector<point> obstacle_velocity::return_vertics_std(point pt1 , point pt2)
{
	std::vector<point> vertic ;
	float dx , dy ,a , b  ;
	float start , end , x , y ;
	point pt ;
	if (pt1.x == pt2.x)
	{
		if (pt1.y < pt2.y) 
		{ 
			start = pt1.y ;
			end = pt2.y ;
		}
		else 
		{
			start = pt2.y ;
			end = pt1.y ;
		}
		int i =0 ;
		while (start < end )
		{
			x = pt1.x ;
			y = start ; 
			pt = this->std_map_coo(x , y) ;
			vertic.push_back(pt); 
			start += 0.01 ;
			++i ;
		}
	} 
	else 
	{
		dy = std::abs((pt2.y - pt1.y)/(pt2.x - pt1.x)) ;
		dx = std::abs((pt2.x - pt1.x)/(pt2.y - pt1.y)) ;
		if ( dx > dy) 
		{
			a = (pt2.y - pt1.y)/(pt2.x - pt1.x) ;
			b = pt1.y - (pt1.x * a) ;

			int i =0 ;
			if (pt1.x < pt2.x) 
			{ 
				start = pt1.x ;
				end = pt2.x ;
			}
			else 
			{
				start = pt2.x ;
				end = pt1.x ;
			}


			while (start < end )
			{
				x = start ;
				y = a * x + b ; 
				pt = this->std_map_coo(x , y) ;
				vertic.push_back(pt); 
				start += this->global_costmap.info.resolution ;
				++i ;
			}
		}
		else 
		{
			a = (pt2.x - pt1.x)/(pt2.y - pt1.y) ;
			b = pt1.x - (pt1.y * a) ;

			int i =0 ;
			if (pt1.y < pt2.y) 
			{ 
				start = pt1.y ;
				end = pt2.y ;
			}
			else 
			{
				start = pt2.y ;
				end = pt1.y ;
			}


			while (start < end )
			{
				x = start ;
				y = a * x + b ; 
				pt = this->std_map_coo(y , x) ;
				vertic.push_back(pt); 
				start += this->global_costmap.info.resolution ;
				++i ;
			}
		}
	}
	return (vertic) ;	
}

point obstacle_velocity::map_coo(float x , float y)
{
	point pt;
	pt.x = (int)((x - this->global_costmap.info.origin.position.x) / this->global_costmap.info.resolution);
	pt.y = (int)((y - this->global_costmap.info.origin.position.y) / this->global_costmap.info.resolution);
	//std::cout<<pt.x << " , " << pt.y << std::endl;
	return (pt);

}

point obstacle_velocity::std_map_coo(float x , float y )
{
	point pt;
	pt.x = (int)(x / this->global_costmap.info.resolution);
	pt.y = (int)(y / this->global_costmap.info.resolution);
	//std::cout<<pt.x << " , " << pt.y << std::endl;
	return (pt);

}

void obstacle_velocity::rect_tester()
{
	std::vector<point> area , area2 , A , B , C , D , A1 , B1 , C1 , D1;
	float x , y , dist ;
	area = this->return_points();
	area2 = this->return_points_area();
	this->cropped_map.info.width  =  (int) (std::max(std::abs(area2[1].x - area2[2].x) / this->global_costmap.info.resolution  , std::abs(area2[0].x - area2[3].x) / this->global_costmap.info.resolution));
	this->cropped_map.info.height = (int) (std::max(std::abs(area2[1].x - area2[3].x) /this->global_costmap.info.resolution  , std::abs(area2[0].x - area2[2].x) / this->global_costmap.info.resolution));
	A = return_vertics(area[0] , area[2]) ;
	B = return_vertics(area[3] , area[2]) ;
	C = return_vertics(area[3] , area[1]) ; 
	D = return_vertics(area[0] , area[1]) ;
	A1 = return_vertics_std(area2[0] , area2[2]) ;
	std::cout<<"2 !!" ;
	B1 = return_vertics_std(area2[3] , area2[2]) ;
	std::cout<<"3 !!" ;
	C1 = return_vertics_std(area2[3] , area2[1]) ; 
	std::cout<<"4 !!" ;
	D1 = return_vertics_std(area2[0] , area2[1]) ;
	std::cout<<"pushhh !!" ;
	area = this->fill_area(A,B,C,D) ;
	area2 = this->fill_area(A1,B1,C1,D1) ;

	/*std::sort(area.begin() , area.end() , [&](point pt1, point pt2) -> bool {
                return (pt1.x  > pt2.x );
            });*/
	std::cout<< this->cropped_map.info.width <<"****"<< this->cropped_map.info.height <<"||" ;
	/*for (int i = 0; i < this->cropped_map.info.width * this->cropped_map.info.height ; ++i)
	 {
	 	std::cout<<"pushhh !!" ;
	 	this->cropped_map.data.push_back(0);
	 }*/
	for (int i = 0; i < this->cropped_map.info.width *this->cropped_map.info.height; ++i)
	{
		this->cropped_map.data.push_back(-1);
	}
	for (int i = 0; i < area.size(); ++i)
	{
		//this->cropped_map.data.push_back(this->global_costmap.data[this->getIndex(area[i].x , area[i].y)] /*- this->global_static_map.data[this->getIndex(area[i].x , area[i].y)]*/ ) ;
		/*dist = sqrt(pow(area[i].x , 2) + pow(area[i].x , 2)) ;
		x = (int) (dist * std::cos(this->yaw_angle) + area[0].x );*/

		//this->cropped_map.data.push_back(this->global_costmap.data[this->getIndex(area[i].x , area[i].y)]) ;
		//this->global_costmap.data[this->getIndex(area[i].x , area[i].y)] = 60 ;
		if (i < area2.size())
			this->cropped_map.data[this->getIndex_area(area2[i].x , area2[i].y)] = this->global_costmap.data[this->getIndex(area[i].x , area[i].y)] ;
		//std::cout<< area2[i].x <<"."<<area2[i].y <<"||" ; 
	}
	std::cout<<"end" << std::endl; 
	//this->cropped_map.data = v ;
	this->cropped_map.header = this->global_costmap.header ;
	this->costmap_pub.publish(this->cropped_map) ;
	this->cropped_map.data.clear();

}

std::vector<point> obstacle_velocity::fill_area(std::vector<point> A , std::vector<point> B , std::vector<point> C , std::vector<point> D)
{
	std::vector<point> polygon_cells , polygons;
	std::copy(D.begin(), D.end(), std::back_inserter(polygon_cells));
	std::copy(A.begin(), A.end(), std::back_inserter(polygon_cells));
	std::copy(B.begin(), B.end(), std::back_inserter(polygon_cells));
	std::copy(C.begin(), C.end(), std::back_inserter(polygon_cells));

	// quick bubble sort to sort points by x
	point swap;
	unsigned int i = 0;
	while (i < polygon_cells.size() - 1)
	{
	  if (polygon_cells[i].x > polygon_cells[i + 1].x)
	  {
	    swap = polygon_cells[i];
	    polygon_cells[i] = polygon_cells[i + 1];
	    polygon_cells[i + 1] = swap;
	    if (i > 0)
	      --i;
	  }
	  else
	    ++i;
	}
	i = 0;
   point min_pt;
   point max_pt;
   unsigned int min_x = polygon_cells[0].x;
   unsigned int max_x = polygon_cells[polygon_cells.size() - 1].x;
 
   // walk through each column and mark cells inside the polygon
   for (unsigned int x = min_x; x <= max_x; ++x)
   {
     if (i >= polygon_cells.size() - 1)
       break;
 
     if (polygon_cells[i].y < polygon_cells[i + 1].y)
     {
       min_pt = polygon_cells[i];
       max_pt = polygon_cells[i + 1];
     }
     else
     {
       min_pt = polygon_cells[i + 1];
       max_pt = polygon_cells[i];
     }
 
     i += 2;
     while (i < polygon_cells.size() && polygon_cells[i].x == x)
     {
       if (polygon_cells[i].y < min_pt.y)
         min_pt = polygon_cells[i];
       else if (polygon_cells[i].y > max_pt.y)
         max_pt = polygon_cells[i];
       ++i;
     }
 
     point pt;
     // loop though cells in the column
     for (unsigned int y = min_pt.y; y < max_pt.y ; ++y)
     {
       pt.x = y;
       pt.y = x;
       polygons.push_back(pt);
     }
   }
   return (polygons);
}

int obstacle_velocity::getIndex( int mx, int my) 
	{
		return mx * this->global_costmap.info.width + my;
	}
int obstacle_velocity::getIndex_area( int mx, int my) 
	{
		return mx * this->cropped_map.info.width + my;
	}
bool obstacle_velocity::compare_index(point pt1 , point pt2)
{
	return (this->getIndex(pt1.x , pt1.y) < this->getIndex(pt1.x , pt1.y));
}
int main(int argc, char **argv)
{	
	ros::init(argc, argv, "obstacle_detector");
	obstacle_velocity robot1 = obstacle_velocity();
	ros::spin();
	return 0;
}

