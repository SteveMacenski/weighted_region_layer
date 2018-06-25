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
                             _level(255), _size(1.0)
/*****************************************************************************/
  
{
  _last_right = ros::Time(0);
  shortcut_key_ = 'm';
  ROS_INFO("MarkingTool: Initialized with short cut key 'm' selected.");
  ros::NodeHandle nh;
  _map_sub = nh.subscribe("/map", 1, &MarkingTool::NewMapCallback, this);
  _weighed_map_pub = nh.advertise<nav_msgs::OccupancyGrid>( \
                           "/weighted_region_layer/marking_tool/map", 1, true);
}

/*****************************************************************************/
MarkingTool::~MarkingTool()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(_map_lock);
  scene_manager_->destroySceneNode( brush_node_);
  brush_node_ = NULL;
  delete _occ_map;
  _occ_map = NULL;
}

/*****************************************************************************/
void MarkingTool::NewMapCallback(const nav_msgs::OccupancyGrid& msg)
/*****************************************************************************/
{
  ResetWeightedMap();
  PublishWeightedMap();
}

/*****************************************************************************/
void MarkingTool::onInitialize()
/*****************************************************************************/
{
  mesh_ = "package://rviz/ogre_media/models/rviz_sphere.mesh"; //TODO color

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
  if( !brush_node_ )
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
      ROS_INFO("pose: %f %f %f", intersection.x, intersection.y, intersection.z);
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
  // add to off grid TODO

  // use _level, _size

  // take max went over


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
  boost::mutex::scoped_lock lock(_map_lock);
  _occ_map = new nav_msgs::OccupancyGrid();
  _occ_map->info = _map_meta;
  _occ_map->header = _map_header;
  _occ_map->header.stamp = ros::Time::now();
  PublishWeightedMap();
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
