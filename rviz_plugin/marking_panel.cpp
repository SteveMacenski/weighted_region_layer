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

#include <stdio.h>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QSpinBox>
#include "marking_panel.hpp"
#include "src/serialization.cpp"


namespace weighted_region_layer
{

/*****************************************************************************/
MarkingPanel::MarkingPanel(QWidget* parent) : rviz::Panel(parent), tool_(NULL)
/*****************************************************************************/
{
  // Create a push button
  btn_clear_ = new QPushButton(this);
  btn_clear_->setText("Clear Marked");
  connect(btn_clear_, SIGNAL(clicked()), this, SLOT(Clear()));

  btn_toggle_ = new QPushButton(this);
  btn_toggle_->setText("Toggle Tool");
  connect(btn_toggle_, SIGNAL(clicked()), this, SLOT(Toggle()));

  btn_save_ = new QPushButton(this);
  btn_save_->setText("Save Weighted Map");
  connect(btn_save_, SIGNAL(clicked()), this, SLOT(Save()));

  btn_setLevel_ = new QPushButton(this);
  btn_setLevel_->setText("Set Mark Level");
  connect(btn_setLevel_, SIGNAL(clicked()), this, SLOT(SetLevel()));

  btn_setsize_ = new QPushButton(this);
  btn_setsize_->setText("Set Brush Size");
  connect(btn_setsize_, SIGNAL(clicked()), this, SLOT(SetSize()));

  btn_load_ = new QPushButton(this);
  btn_load_->setText("Load File");
  connect(btn_load_, SIGNAL(clicked()), this, SLOT(Load()));

  level_ = new QComboBox(this);
  QList<QString> stringsList;
  stringsList.append("0");  
  stringsList.append("30");  
  stringsList.append("50");  
  stringsList.append("100");  
  stringsList.append("125");  
  stringsList.append("150");  
  stringsList.append("200");  
  stringsList.append("255");  
  level_->addItems(stringsList);
  level_->setCurrentIndex(-1);

  size_ = new QComboBox(this);
  stringsList.clear();
  stringsList.append("0.1");  
  stringsList.append("0.5");  
  stringsList.append("1.0");  
  stringsList.append("1.5");  
  stringsList.append("2.0");  
  stringsList.append("2.5");  
  stringsList.append("3");  
  stringsList.append("10");  
  size_->addItems(stringsList);
  size_->setCurrentIndex(-1);

  // Create a line edit
  name_ = new QLineEdit(this);
  load_ = new QLineEdit(this);

  // Horizontal Layout 
  hlayout1 = new QHBoxLayout;
  hlayout1->addWidget(btn_toggle_);
  hlayout1->addWidget(btn_clear_);

  hlayout2 = new QHBoxLayout;
  hlayout2->addWidget(btn_save_);
  hlayout2->addWidget(name_);

  hlayout3 = new QHBoxLayout;
  hlayout3->addWidget(btn_setLevel_);
  hlayout3->addWidget(level_);
  hlayout3->addWidget(btn_setsize_);
  hlayout3->addWidget(size_);

  hlayout4 = new QHBoxLayout;
  hlayout4->addWidget(btn_load_);
  hlayout4->addWidget(load_);

  layout1 = new QVBoxLayout;
  layout1->addLayout(hlayout1);
  layout1->addLayout(hlayout3);
  layout1->addLayout(hlayout2);
  layout1->addLayout(hlayout4);

  setLayout(layout1);

  ros::NodeHandle nh;
  _save = nh.serviceClient<weighted_region_layer::SaveWeightedRegionFile>( \
                  "/move_base/global_costmap/weighted_region_layer/save_file"); // TODO thse are p wrong
  _load = nh.serviceClient<weighted_region_layer::LoadWeightedRegionFile>( \
                  "/move_base/global_costmap/weighted_region_layer/save_file");
}

/*****************************************************************************/
MarkingPanel::~MarkingPanel()
/*****************************************************************************/
{
  delete tool; 
  delete tool_; 
  tool = NULL; 
  tool_ = NULL;
};

/*****************************************************************************/
void MarkingPanel::Start()
/*****************************************************************************/
{
  if (tool_ == NULL)
  {
    tool_ = \
  vis_manager_->getToolManager()->addTool("weighted_region_layer/MarkingTool");
  tool = dynamic_cast<MarkingTool*>(tool_);
  }
  return;
}

/*****************************************************************************/
void MarkingPanel::Clear()
/*****************************************************************************/
{
  Start();
  tool->ResetWeightedMap();
}

/*****************************************************************************/
void MarkingPanel::SetLevel()
/*****************************************************************************/
{
  Start();

  std::string level_s = level_->itemText(level_->currentIndex()).toStdString();
  level_->setCurrentIndex(-1);
  int level = atoi(level_s.c_str());
  tool->SetMarkLevel(level);

}

/*****************************************************************************/
void MarkingPanel::SetSize()
/*****************************************************************************/
{
  Start();

  std::string size_s = size_->itemText(size_->currentIndex()).toStdString();
  size_->setCurrentIndex(-1);
  double size = atof(size_s.c_str());
  tool->SetSize(size);
}

/*****************************************************************************/
void MarkingPanel::Toggle()
/*****************************************************************************/
{
  Start();

  rviz::ToolManager* toolManager = vis_manager_->getToolManager();
  if (toolManager->getCurrentTool()->getName().toStdString()
      == std::string("Marking Tool"))
  {
    toolManager->setCurrentTool(toolManager->getDefaultTool());
  } 
  else
  {
    toolManager->setCurrentTool(tool_);
  }
}

/*****************************************************************************/
void MarkingPanel::Save()
/*****************************************************************************/
{
  Start();

  nav_msgs::OccupancyGrid grid;
  tool->GetWeightedMap(grid);
  
  weighted_region_layer::SaveWeightedRegionFile srv;
  srv.request.filename = name_->text().toStdString();
  srv.request.overwrite = true;
  srv.request.grid = grid;

  if(!_save.call(srv))
  {
    ROS_WARN("MarkingPanel: Could not save using layer service, "
             "is it running? No worries, I'll just save locally instead.");
    std::string name = name_->text().toStdString() + std::string(".wrl");
    weighted_region_layer::data_serial msg;
    msg.data = grid.data;
    serialization::Write(name, msg);
  }
}

/*****************************************************************************/
void MarkingPanel::Load()
/*****************************************************************************/
{
  Start();

  weighted_region_layer::LoadWeightedRegionFile srv;
  srv.request.filename = load_->text().toStdString();
  if(!_load.call(srv))
  {
     ROS_ERROR("MarkingPanel: Failed to load from file, is service running?");
  }
}

}  // end namespace 

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(weighted_region_layer::MarkingPanel, rviz::Panel)
