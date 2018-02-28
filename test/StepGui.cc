/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/*
 * Author: Jennifer Buehler
 * Date: December 2017
 */

#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "StepGui.hh"

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(StepGui)

QSize maxHeightAddWidth(const QSize &s1, const QSize &s2,
                        float wFact = 1, float hFact = 1.0)
{
  return QSize((s1.width() + s2.width())*wFact,
               std::max(s1.height(), s2.height())*hFact);
}


/////////////////////////////////////////////////
StepGui::StepGui()
  : GUIPlugin()
{
  // Set the frame background and foreground colors
  this->setStyleSheet
    ("QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame & layout to hold all the buttons
  QFrame *bckFwdFrame = new QFrame();
  QHBoxLayout *bckFwdLayout = new QHBoxLayout();

  // Create the push buttons and connect to OnButton* functions
  QPushButton * buttonPrev = new QPushButton("<<");
  QPushButton * buttonNext = new QPushButton(">>");
  buttonPrev->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  buttonPrev->resize(buttonPrev->sizeHint());
  buttonNext->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  buttonNext->resize(buttonNext->sizeHint());

  connect(buttonPrev, SIGNAL(clicked()), this, SLOT(OnButtonPrev()));
  connect(buttonNext, SIGNAL(clicked()), this, SLOT(OnButtonNext()));

  minSize = maxHeightAddWidth(buttonPrev->sizeHint(), buttonNext->sizeHint());

  // Add the buttons to the frame's layout
  bckFwdLayout->addWidget(buttonPrev);
  bckFwdLayout->addWidget(buttonNext);

  // set layout and add widget
  bckFwdFrame->setLayout(bckFwdLayout);
  mainLayout->addWidget(bckFwdFrame);

  // Remove margins to reduce space
  bckFwdLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  sizePolicy.setHorizontalStretch(0);
  sizePolicy.setVerticalStretch(0);
  this->setSizePolicy(sizePolicy);
  this->move(10, 80);
  this->resize(minSize);

  // Set up transportation system
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init();
  const std::string TOPIC="/test/cmd";
  this->pub = this->node->Advertise<gazebo::msgs::Any>(TOPIC);
}

/////////////////////////////////////////////////
StepGui::~StepGui()
{
}

/////////////////////////////////////////////////
void StepGui::OnButtonNext()
{
  gazebo::msgs::Any m;
  m.set_type(gazebo::msgs::Any::INT32);
  // "Next"
  m.set_int_value(1);
  this->pub->Publish(m);
}


/////////////////////////////////////////////////
void StepGui::OnButtonPrev()
{
  gazebo::msgs::Any m;
  m.set_type(gazebo::msgs::Any::INT32);
  // "Prev"
  m.set_int_value(-1);
  this->pub->Publish(m);
}
