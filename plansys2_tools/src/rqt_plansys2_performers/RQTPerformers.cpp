// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <unistd.h>

#include <QDebug>
#include <QTime>
#include <QTimer>
#include <QPushButton>
#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <QCheckBox>

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <memory>
#include <utility>
#include <string>

#include "rqt_plansys2_performers/RQTPerformers.hpp"

#include <ros/ros.h>


namespace rqt_plansys2_performers
{

using std::placeholders::_1;

RQTPerformers::RQTPerformers()
: rqt_gui_cpp::Plugin(),
  widget_(0)
{
  setObjectName("RQTPerformers");

  sub_ = nh_.subscribe("message", 1, &RQTPerformers::messageCallback, this);
}

void RQTPerformers::initPlugin(qt_gui_cpp::PluginContext & context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(
      widget_->windowTitle() + " (" +
      QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);
  connect(ui_.pushButton, SIGNAL(pressed()), this, SLOT(buttonCB()));

  controller_spin_timer_ = new QTimer(this);
  connect(controller_spin_timer_, SIGNAL(timeout()), this, SLOT(spin_loop()));
  controller_spin_timer_->start(100);
  
}

void RQTPerformers::shutdownPlugin()
{
}

void
RQTPerformers::saveSettings(
  qt_gui_cpp::Settings & plugin_settings, qt_gui_cpp::Settings & instance_settings) const
{
  (void)plugin_settings;
  (void)instance_settings;
}

void
RQTPerformers::spin_loop()
{
  std::cerr << "*";
}

void
RQTPerformers::restoreSettings(
  const qt_gui_cpp::Settings & plugin_settings, const qt_gui_cpp::Settings & instance_settings)
{
  (void)plugin_settings;
  (void)instance_settings;
}

void
RQTPerformers::messageCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Message: [%s]", msg->data.c_str());
  ui_.label->setText(QString::fromStdString(msg->data));
}

void 
RQTPerformers::buttonCB()
{
   ROS_INFO("Pressed button=================");
}

}  // namespace rqt_plansys2_performers

PLUGINLIB_EXPORT_CLASS(rqt_plansys2_performers::RQTPerformers, rqt_gui_cpp::Plugin)
