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
 * Purpose: Allow for weighted regions to be visually defined
 *********************************************************************/

#ifndef MARKINGTOOL_H
#define MARKINGTOOL_H

// ROS
#include <ros/ros.h>
#include <costmap_2d/cost_values.h>
// msgs
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/Header.h>
#include <weighted_region_layer/LoadWeightedRegionFile.h>
#include <weighted_region_layer/SaveWeightedRegionFile.h>
// RVIZ
#include <rviz/tool.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
// boost
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
// STL
#include <vector>
#include <utility>
#include <queue>

 /*
process:
OFFLINE
  1 start map server with map
  2 load plugin
  3 start tool
  4 mark all relavent things
  5 clear if needed
  6 set level if needed
  7 save file
  8 in costmap_2d layer set file to load
  9 load and use

ONLINE
  1 start robot 
  2 make new map
  3 share URI
  4 edit
  5 save
  6 load
 */

namespace weighted_region_layer
{

class MarkingTool: public rviz::Tool
{
Q_OBJECT
public:
  MarkingTool();
  ~MarkingTool(); // called when '-' key is selected on Tool
  virtual void onInitialize();
  virtual void activate(); // called with selecting tool or hotbutton
  virtual void deactivate(); // called when a new tool is selected or hotkey out
  virtual int processMouseEvent( rviz::ViewportMouseEvent& event ); // main function
  bool GetWeightedMap(nav_msgs::OccupancyGrid& weighted_map);
  bool ResetWeightedMap();
  bool SetMarkLevel(const int& level);
  bool SetSize(const double& size);
  void PublishWeightedMap();

private:
  void AddToWeighedMap( const Ogre::Vector3& position );
  void NewMapCallback(const nav_msgs::OccupancyGrid& msg);

  Ogre::SceneNode* brush_node_;
  std::string mesh_;

  ros::Publisher _weighed_map_pub;
  ros::Subscriber _map_sub;
  ros::Time _last_right;
  int _num_wait, _level;
  double _size;
  bool _got_map;
  nav_msgs::OccupancyGrid* _occ_map;
  nav_msgs::MapMetaData _map_meta;
  std_msgs::Header _map_header;

  boost::mutex _map_lock;
};

} // end namespace

#endif // PLANT_FLAG_TOOL_H
