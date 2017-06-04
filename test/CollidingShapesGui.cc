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

#include <QScrollBar>
#include <QSlider>

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
  QFrame *collidingShapesFrame = new QFrame();
  QHBoxLayout *collidingShapesLayout = new QHBoxLayout();

#if 1
  // Create the push buttons and connect to OnButton* functions
  QPushButton * buttonNext = new QPushButton("Gaga");
  buttonNext->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  buttonNext->resize(buttonNext->sizeHint());
//  connect(buttonNext, SIGNAL(clicked()), this, SLOT(OnButtonNext()));
  collidingShapesLayout->addWidget(buttonNext);
#endif

#if 0
  QScrollBar * scrollbar = new QScrollBar();
  // scrollbar->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  connect(scrollbar, SIGNAL(valueChanged(int)),
          this, SLOT(OnValueChanged(int)));
  collidingShapesLayout->addWidget(scrollbar);
#endif

#if 1
  QSlider * slider = new QSlider(Qt::Horizontal);
  slider->setMinimum(-100);
  slider->setMaximum(100);
  slider->resize(300,20);
//  slider->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  connect(slider, SIGNAL(valueChanged(int)),
          this, SLOT(OnValueChanged(int)));
  collidingShapesLayout->addWidget(slider);
#endif

  // set the layout and add the frame as widget
  collidingShapesFrame->setLayout(collidingShapesLayout);
  mainLayout->addWidget(collidingShapesFrame);

  // Remove margins to reduce space
  collidingShapesLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  sizePolicy.setHorizontalStretch(0);
  sizePolicy.setVerticalStretch(0);
  this->setSizePolicy(sizePolicy);
  this->move(10, 100);

  QSize minSize = maxHeightAddWidth(buttonNext->size(), slider->size());
  this->resize(minSize);

  // Set up transportation system
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init();
  std::string TOPIC="collide_shapes_test/test";
  this->mirrorWorldPub =
    this->node->Advertise<gazebo::msgs::Any>(TOPIC);
}

/////////////////////////////////////////////////
CollidingShapesGui::~CollidingShapesGui()
{
}

/////////////////////////////////////////////////
void CollidingShapesGui::OnValueChanged(int val)
{
  std::cout << "Value changed! " << val << std::endl;
  // this->mirrorWorldPub->WaitForConnection();
  // Send the model to the gazebo server
  gazebo::msgs::Any m;
  m.set_type(gazebo::msgs::Any::INT32);
  m.set_int_value(1); // "Next" world
  this->mirrorWorldPub->Publish(m);
}
