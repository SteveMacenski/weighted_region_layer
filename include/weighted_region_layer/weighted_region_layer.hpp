/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Steven Macenski
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Steven Macenski, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Steve Macenski (stevenmacenski@gmail.com)
 * Purpose: Allow for weighted regions for traversal to be easily used
 *********************************************************************/

#ifndef WEIGHTED_REGION_LAYER_H_
#define WEIGHTED_REGION_LAYER_H_

// ROS
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/footprint.h>
// STL
#include <vector>
#include <string>
#include <iostream>
#include <time.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
// msgs
#include <weighted_region_layer/LoadWeightedRegionFile.h>
#include <weighted_region_layer/SaveWeightedRegionFile.h>
#include <weighted_region_layer/data_serial.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>

namespace weighted_region_layer
{

class WeightedRegionLayer : public costmap_2d::CostmapLayer
{

public:
  WeightedRegionLayer();
  ~WeightedRegionLayer();

  // Necessary costmap_2d evils
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void matchSize();

private:
  // the "meat"
  void ChangeWeightedRegionsFile();

  // Callbacks
  void MapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  bool LoadFileService( \
                 weighted_region_layer::LoadWeightedRegionFile::Request& req, 
                 weighted_region_layer::LoadWeightedRegionFile::Response& resp);
  bool SaveFileService( \
                 weighted_region_layer::SaveWeightedRegionFile::Request& req, 
                 weighted_region_layer::SaveWeightedRegionFile::Response& resp);

  // IO
  void ReadFromFile(const std::string& filename);
  void WriteToFile(const std::string& filename);
  inline bool IsFileValid(const std::string& name)
  {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0); 
  }

  ros::Subscriber _map_sub;
  ros::NodeHandle _nh;
  ros::ServiceServer _save, _load;
  std::string _map_topic, _wrl_parameter_name, _wrl_file_name, _global_frame, _map_frame;
  bool _enable_param_updates, _got_map;
  double _width, _height;
};

} // end namespace

#endif
