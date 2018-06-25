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

#ifndef COMMAND_PANEL_H
#define COMMAND_PANEL_H

// rviz / QT
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif
#include <QPushButton>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <std_msgs/Bool.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>
#include <nav_msgs/OccupancyGrid.h>
//STL
#include <ctime>
#include <stdlib.h>
#include <stdio.h>
// weighted zones layer
#include <weighted_region_layer/LoadWeightedRegionFile.h>
#include <weighted_region_layer/SaveWeightedRegionFile.h>
#include "marking_tool.hpp"

class QLineEdit;
class QSpinBox;
class QComboBox;

namespace weighted_region_layer
{

class MarkingPanel : public rviz::Panel
{
  Q_OBJECT
public:

  MarkingPanel(QWidget *parent = 0);
  ~MarkingPanel();

public Q_SLOTS:
protected Q_SLOTS:
  void Start();
  void Clear();
  void Toggle();
  void Save();
  void SetLevel();
  void Load();
  void SetSize();

protected:
  QPushButton *btn_clear_;
  QPushButton *btn_toggle_;
  QPushButton *btn_save_;
  QPushButton *btn_load_;
  QPushButton *btn_setLevel_;
  QPushButton *btn_setsize_;
  QLineEdit   *name_;
  QLineEdit   *load_;
  QComboBox   *level_;
  QComboBox   *size_;

  rviz::Tool  *tool_;
  MarkingTool *tool;

  QHBoxLayout* hlayout1;
  QHBoxLayout* hlayout2;
  QHBoxLayout* hlayout3;
  QHBoxLayout* hlayout4;
  QVBoxLayout* layout1;

  ros::ServiceClient _save, _load;
};

}  // end namespace

#endif
