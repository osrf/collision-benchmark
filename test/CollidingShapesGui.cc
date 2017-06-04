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
#include "CollidingShapesGui.hh"
#include <sstream>
#include <gazebo/msgs/msgs.hh>

using collision_benchmark::test::CollidingShapesGui;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(CollidingShapesGui)


QSize maxHeightAddWidth(const QSize& s1, const QSize& s2,
                        float wFact=1, float hFact=1.0)
{
  return QSize((s1.width() + s2.width())*wFact,
               std::max(s1.height(), s2.height())*hFact);
}


/////////////////////////////////////////////////
CollidingShapesGui::CollidingShapesGui()
  : GUIPlugin()
{
  // Set the frame background and foreground colors
  this->setStyleSheet
    ("QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame & layout to hold all the buttons
  QFrame *switchWorldsFrame = new QFrame();
  QHBoxLayout *collidingShapesLayout = new QHBoxLayout();

  // Create the push buttons and connect to OnButton* functions
  QPushButton * buttonPrev = new QPushButton("Gigi");
  QPushButton * buttonNext = new QPushButton("Gaga");
  buttonPrev->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  buttonPrev->resize(buttonPrev->sizeHint());
  buttonNext->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  buttonNext->resize(buttonNext->sizeHint());

//  connect(buttonPrev, SIGNAL(clicked()), this, SLOT(OnButtonPrev()));
//  connect(buttonNext, SIGNAL(clicked()), this, SLOT(OnButtonNext()));

/*  minSize = maxHeightAddWidth(buttonPrev->sizeHint(), buttonNext->sizeHint());

  // Create label to sit in-between buttons ad display world name
  labelName = new QLabel("<...>");
  connect(this, SIGNAL(TriggerNameChange(const std::string&)),
          this, SLOT(OnNameChange(const std::string&)));*/

  // Add the buttons to the frame's layout
  collidingShapesLayout->addWidget(buttonPrev);
  // collidingShapesLayout->addWidget(labelName);
  collidingShapesLayout->addWidget(buttonNext);

  switchWorldsFrame->setLayout(collidingShapesLayout);
  mainLayout->addWidget(switchWorldsFrame);

  // Remove margins to reduce space
  collidingShapesLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  sizePolicy.setHorizontalStretch(0);
  sizePolicy.setVerticalStretch(0);
  this->setSizePolicy(sizePolicy);
  this->move(100, 10);
//  QSize totalSize = maxHeightAddWidth(scrollBar->sizeHint(), minSize, 1.1);
//  this->resize(totalSize);

  // Set up transportation system
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init();
  std::string TOPIC="collide_shapes_test/test";

  std::cout<<"Waiting for connection to topic "<<TOPIC<<std::endl;
  this->mirrorWorldPub->WaitForConnection();
  std::cout<<"Received."<<std::endl;

}

/////////////////////////////////////////////////
CollidingShapesGui::~CollidingShapesGui()
{
}

/////////////////////////////////////////////////
void CollidingShapesGui::OnValueChanged(int val)
{
  // Send the model to the gazebo server
  gazebo::msgs::Any m;
  m.set_type(gazebo::msgs::Any::INT32);
  m.set_int_value(1); // "Next" world
  this->mirrorWorldPub->Publish(m);
}
