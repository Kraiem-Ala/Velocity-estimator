#include<costmap_layer/custom_layer.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>


PLUGINLIB_EXPORT_CLASS(custom_layer_namespace::Customlayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace custom_layer_namespace
{

Customlayer::Customlayer() {}

void Customlayer::onInitialize()
{
  std::vector<long double> row;
  std::string line, word;
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &Customlayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  
  std::fstream file ("/home/kraiem/Desktop/PFE/simulation_workspace/src/costmap_layer/costmap/kde/array816hour66.csv", std::ios::in);
  if(file.is_open())
  {
    while(getline(file, line))
    {
      row.clear();
      std::stringstream str(line);
      while(getline(str, word, ','))
      {
        //std::cout<< word <<std::endl;
        row.push_back(std::stold(word) * 255);
      }
      content.push_back(row);
      
  ROS_INFO("Costmap LOADED");
  }
  file.close();
}
  else
  ROS_ERROR("Could not open the file");
 

}

void Customlayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void Customlayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void Customlayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
  unsigned int mx;
  unsigned int my;
  int n , m , i ,j , k ,l ,cost;
  n=m=i=j=k=l = 0;
  //ROS_ERROR("COSTMAP LAYER");
  for (j = 0; j < 816; j++)
  {
    k=0;
    for (i = 0; i < 450; i++)
    {
      int index = getIndex(i, j);
      if (i < 21)
        setCost(i, j, 255);
      else
      {
        if (k < 815)
        {
          cost = (int)((int)content[k][j] + (int)content[k+1][j])/2.5 ;
          if((int)content[k][j] != 0)  
            setCost(i, j, cost /*(int)content[i][l]*/ );
        }
          else 
            cost = (int)((int)content[814][j] + (int)content[815][j])/2.5 ;
        k+=2 ;
      }
      
      ++n;
      
      //std::cout<<k << std::endl;
    }
  }
  
  *min_x = std::min(*min_x, mark_x);
  *min_y = std::min(*min_y, mark_y);
  *max_x = std::max(*max_x, mark_x);
  *max_y = std::max(*max_y, mark_y);
}

void Customlayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
  {
  if (!enabled_)
    return;
  for (int j = min_j; j < max_j; j++)
    {
      for (int i = min_i; i < max_i; i++)
      {
        int index = getIndex(i, j);
        if (costmap_[index] == NO_INFORMATION)
          continue;
        if ((int)master_grid.getCost(i,j) < 20)
          master_grid.setCost(i, j, costmap_[index]); 
      }
    }  
  }

  } // end namespace