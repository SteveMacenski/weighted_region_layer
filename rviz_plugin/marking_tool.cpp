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
 *********************************************************************/

#include "marking_tool.hpp"

namespace weighted_region_layer
{

/*****************************************************************************/
MarkingTool::MarkingTool() : brush_node_( NULL ), _num_wait(0), 
                             _occ_map(new nav_msgs::OccupancyGrid()), 
                             _level(255), _size(1.0), _got_map(false)
/*****************************************************************************/
  
{
  _last_right = ros::Time(0);
  shortcut_key_ = 'm';
  ROS_INFO("MarkingTool: Initialized with short cut key 'm' selected.");
  ros::NodeHandle nh;
  _map_sub = nh.subscribe("/map", 1, &MarkingTool::NewMapCallback, this);
  _weighed_map_pub = nh.advertise<nav_msgs::OccupancyGrid>( \
                           "/weighted_region_layer/marking_tool/map", 1, true);

  //TEST CASE WITHOUT MAP
  _occ_map->info.resolution = 0.05;
  _occ_map->info.width = 1000;
  _occ_map->info.height = 1000;
  _occ_map->info.origin.position.x = 0.0;
  _occ_map->info.origin.position.y = 0.0;
  _occ_map->info.origin.orientation.w = 1.0;
  _occ_map->data.resize(1000*1000);
  _occ_map->header.frame_id = "/map";
  _got_map = true;
  _map_meta = _occ_map->info;
  _map_header = _occ_map->header;
}

/*****************************************************************************/
MarkingTool::~MarkingTool()
/*****************************************************************************/
{
  scene_manager_->destroySceneNode( brush_node_);
  brush_node_ = NULL;
  delete _occ_map;
  _occ_map = NULL;
}

/*****************************************************************************/
void MarkingTool::NewMapCallback(const nav_msgs::OccupancyGrid& msg)
/*****************************************************************************/
{
  if (!_got_map)
  {
    _got_map = true;
  }
  _map_meta = msg.info;
  _map_header = msg.header;
  _occ_map->info = _map_meta;
  _occ_map->header = _occ_map->header;
  _occ_map->data.resize(_occ_map->info.width * _occ_map->info.height);

  ResetWeightedMap();
  PublishWeightedMap();
}

/*****************************************************************************/
void MarkingTool::onInitialize()
/*****************************************************************************/
{
  mesh_ = "package://rviz/ogre_media/models/rviz_sphere.mesh";

  if( rviz::loadMeshFromResource( mesh_ ).isNull() )
  {
    ROS_ERROR("MarkingTool: failed to load model '%s'.", mesh_.c_str());
    return;
  }

  brush_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( mesh_ );
  brush_node_->attachObject( entity );
  brush_node_->setVisible( false );
}

/*****************************************************************************/
void MarkingTool::activate()
/*****************************************************************************/
{
  if( brush_node_ )
  {
    brush_node_->setVisible( true );

  }
}

/*****************************************************************************/
void MarkingTool::deactivate()
/*****************************************************************************/
{
  if( brush_node_ )
  {
    brush_node_->setVisible( false );
  }
}

/*****************************************************************************/
int MarkingTool::processMouseEvent( rviz::ViewportMouseEvent& event )
/*****************************************************************************/
{
  if( !brush_node_ || !_got_map )
  {
    return Render;
  }
  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
  if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                         ground_plane,
                                         event.x, event.y, intersection ))
  {
    brush_node_->setVisible( true );
    brush_node_->setPosition( intersection );

    if( event.left() )
    {
      AddToWeighedMap(intersection);

      _num_wait++;
      if (_num_wait % 10)
      {
        PublishWeightedMap();
      }
    }
    if (event.rightDown() )
    {
      const ros::Time curr_time = ros::Time::now();
      if (curr_time - _last_right < ros::Duration(1.))
      {
        ROS_INFO("MarkingTool: Clearing weighted regions!");
        ResetWeightedMap();
      }
      _last_right = ros::Time::now();
    }
  }
  else
  {
    brush_node_->setVisible( false );
  }
  return Render;
}

/*****************************************************************************/
void MarkingTool::AddToWeighedMap( const Ogre::Vector3& position )
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(_map_lock);

  // find center pose in the occupancy grid frame
  int mx = (int)((position.x - _map_meta.origin.position.x) / _map_meta.resolution);
  int my = (int)((position.y - _map_meta.origin.position.y) / _map_meta.resolution);

  if (mx < 0 || mx > _map_meta.width || my < 0 || my > _map_meta.height)
  {
    return;
  }

  // Find all occupancy grid cells inside radius
  std::vector<std::pair<int, int> > cells;
  cells.push_back(std::pair<int,int>(mx,my));

  const int leg_length = floor(_size / (2.*_map_meta.resolution));
  const double leg_length_2 = (double)(leg_length*leg_length);
  const int start_cell = my * _map_meta.width + mx;

  // find bottom corner
  int start_of_col = start_cell - leg_length;
  int bottom_corner = start_of_col - leg_length*_map_meta.width;

  // get all in width row
  for (int j=0;j!=2.*leg_length;j++)
  {
    for (int i=0;i!=2.*leg_length;i++)
    {
      int cell = bottom_corner + i + j*_map_meta.width; 
      if (true) // TODO check in valid space
      {
        int my_t = cell / _map_meta.width;
        cells.push_back(std::pair<int,int>(cell - (my_t * _map_meta.width), my_t));
      }
      else
      {
        break;
      }
    }
  }

  for (int i=0;i!=cells.size();i++) 
  {
    int x = cells[i].first;
    int y = cells[i].second;
    if ((mx-x)*(mx-x) + (my-y)*(my-y) < leg_length_2)
    { 
      // take max of current value and _level
      const int index = cells[i].second * _map_meta.width + cells[i].first;
      if (_level > _occ_map->data[index])
      {
        _occ_map->data[index] = _level;
      }
    }
  }
}

/*****************************************************************************/
void MarkingTool::PublishWeightedMap()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(_map_lock);
  _weighed_map_pub.publish(*_occ_map);
}

/*****************************************************************************/
bool MarkingTool::GetWeightedMap(nav_msgs::OccupancyGrid& weighted_map)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(_map_lock);
  weighted_map = *_occ_map;
  return true;
}

/*****************************************************************************/
bool MarkingTool::ResetWeightedMap()
/*****************************************************************************/
{
  _map_lock.lock();
  _occ_map = new nav_msgs::OccupancyGrid();
  _occ_map->info = _map_meta;
  _occ_map->header = _map_header;
  _occ_map->header.stamp = ros::Time::now();
  _occ_map->data.resize(_map_meta.width * _map_meta.height);
  _map_lock.unlock();
  return true;
}

/*****************************************************************************/
bool MarkingTool::SetMarkLevel(const int& level)
/*****************************************************************************/
{
  _level = level;
  return true;
}

/*****************************************************************************/
bool MarkingTool::SetSize(const double& size)
/*****************************************************************************/
{
  _size = size;
  brush_node_->setScale(size, size, size);
  brush_node_->setVisible( true );

  return true;
}

} // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(weighted_region_layer::MarkingTool,rviz::Tool )
