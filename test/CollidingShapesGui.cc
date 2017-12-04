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
#include "colliding_shapes.pb.h"
#include <sstream>

#include "CollidingShapesGui.hh"
#include "CollidingShapesParams.hh"

using collision_benchmark::test::CollidingShapesGui;
using collision_benchmark::test::CollidingShapesParams;
using collision_benchmark::test::msgs::CollidingShapesMsg;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(CollidingShapesGui)

QSize maxHeightAddWidth(const QSize &s1, const QSize &s2,
                        float wFact = 1, float hFact = 1.0)
{
  return QSize((s1.width() + s2.width())*wFact,
               std::max(s1.height(), s2.height())*hFact);
}

/////////////////////////////////////////////////
CollidingShapesGui::CollidingShapesGui()
  : GUIPlugin(),
    slider(NULL)
{
  // Set the frame background and foreground colors
  this->setStyleSheet
    ("QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  // QGridLayout *mainLayout = new QGridLayout;
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame and layout to hold all perpendicular move elements
  //////////////////////

  QFrame *perpMoveShapesFrame = new QFrame();
  QHBoxLayout *perpMoveShapesLayout = new QHBoxLayout();
  this->dial = new QDial();
  this->dial->setMinimum(0);
  this->dial->setMaximum(CollidingShapesParams::MaxSliderVal);
  this->dial->setValue(CollidingShapesParams::MaxSliderVal);
  this->dial->resize(300, 20);
  connect(this->dial, SIGNAL(valueChanged(int)),
          this, SLOT(OnPerpValueChanged(int)));
  perpMoveShapesLayout->addWidget(this->dial);
  // Button up
  QPushButton * buttonDecPerp = new QPushButton("^");
  buttonDecPerp->resize(buttonDecPerp->sizeHint());
  connect(buttonDecPerp, SIGNAL(clicked()), this, SLOT(OnButtonPerpInc()));
  perpMoveShapesLayout->addWidget(buttonDecPerp);
  // Button down
  QPushButton * buttonIncPerp = new QPushButton("v");
  buttonIncPerp->resize(buttonIncPerp->sizeHint());
  connect(buttonIncPerp, SIGNAL(clicked()), this, SLOT(OnButtonPerpDec()));
  perpMoveShapesLayout->addWidget(buttonIncPerp);
  // set the layout
  perpMoveShapesFrame->setLayout(perpMoveShapesLayout);

  // Create the frame and layout to hold all other control elements
  //////////////////////

  QFrame *collidingShapesFrame = new QFrame();
  QHBoxLayout *collidingShapesLayout = new QHBoxLayout();
  // Button to the left
  QPushButton * buttonDec = new QPushButton("<");
  buttonDec->resize(buttonDec->sizeHint());
  connect(buttonDec, SIGNAL(clicked()), this, SLOT(OnButtonDec()));
  collidingShapesLayout->addWidget(buttonDec);
  // slider
  this->slider = new QSlider(Qt::Horizontal);
  this->slider->setMinimum(0);
  this->slider->setMaximum(CollidingShapesParams::MaxSliderVal);
  this->slider->setValue(CollidingShapesParams::MaxSliderVal);
  this->slider->resize(20, 20);
  connect(this->slider, SIGNAL(valueChanged(int)),
          this, SLOT(OnValueChanged(int)));
  collidingShapesLayout->addWidget(this->slider);
  // Button to the right
  QPushButton * buttonInc = new QPushButton(">");
  buttonInc->resize(buttonInc->sizeHint());
  connect(buttonInc, SIGNAL(clicked()), this, SLOT(OnButtonInc()));
  collidingShapesLayout->addWidget(buttonInc);
  // auto collide button
  QPushButton * buttonAutoCollide = new QPushButton("AutoCollide");
  buttonAutoCollide->resize(buttonAutoCollide->sizeHint());
  connect(buttonAutoCollide, SIGNAL(clicked()), this,
          SLOT(OnButtonAutoCollide()));
  collidingShapesLayout->addWidget(buttonAutoCollide);
  // save button
  QPushButton * buttonSave = new QPushButton("Save config");
  buttonSave->resize(buttonSave->sizeHint());
  connect(buttonSave, SIGNAL(clicked()), this,
          SLOT(OnButtonSaveConfig()));
  collidingShapesLayout->addWidget(buttonSave);
  // set the layout
  collidingShapesFrame->setLayout(collidingShapesLayout);

  // add frame widgets to main frame and do resizing operations
  //////////////////

  mainLayout->addWidget(collidingShapesFrame);
  mainLayout->addWidget(perpMoveShapesFrame);

  // Remove margins to reduce space
  collidingShapesLayout->setContentsMargins(0, 0, 0, 0);
  perpMoveShapesLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  this->setLayout(mainLayout);

  // Resize this widget. Stupidly, the collision slider is resized if this
  // is not done manually :(
  QSize buttonsSize = maxHeightAddWidth(buttonInc->size(), buttonDec->size());
  buttonsSize = maxHeightAddWidth(buttonsSize, buttonSave->size());
  buttonsSize = maxHeightAddWidth(buttonsSize, buttonAutoCollide->size());
  QSize totalCollSize = maxHeightAddWidth(buttonsSize, this->slider->size());
  QSize perpButtonsSize = maxHeightAddWidth(buttonIncPerp->size(),
                                            buttonDecPerp->size());
  QSize totalPerpSize = maxHeightAddWidth(perpButtonsSize, this->dial->size());
  QSize totalSize = maxHeightAddWidth(totalCollSize, totalPerpSize);
  this->resize(totalSize);

  // Set up transportation system
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init();
  std::string pub_topic="collide_shapes_test/control";
  this->pub = this->node->Advertise<CollidingShapesMsg>(pub_topic);

  std::string sub_topic="collide_shapes_test/feedback";
  this->sub = this->node->Subscribe(sub_topic,
                      &CollidingShapesGui::receiveFeedbackMsg, this);
}

/////////////////////////////////////////////////
CollidingShapesGui::~CollidingShapesGui()
{
}

/////////////////////////////////////////////////
void CollidingShapesGui::Load(sdf::ElementPtr /*_sdf*/)
{
  QWidget * parentWidget = this->parentWidget();
  if (!parentWidget)
  {
    gzerr << "CollidingShapesGui should have a parent widget." << std::endl;
    return;
  }
  // align the widget to the bottom-right of the render window
  int width = parentWidget->size().width();
  int height = parentWidget->size().height();
  this->move(width - this->width() - 20, height - this->height() - 20);

  parentWidget->installEventFilter(this);
}

/////////////////////////////////////////////////
bool CollidingShapesGui::eventFilter(QObject *obj, QEvent *event)
{
  if (event->type() != QEvent::Resize)
  {
    return QObject::eventFilter(obj, event);
  }
/*  QResizeEvent * resizeEvent = dynamic_cast<QResizeEvent*>(event);
  if (!resizeEvent)
  {
    gzerr << "Failed to dynamic cast resizeEvent. " << std::endl;
    return QObject::eventFilter(obj, event);
  }
  std::cout << "New size: " << resizeEvent->size().width()
            << ", " << resizeEvent->size().height() << std::endl;
  */

  QWidget * parentWidget = this->parentWidget();
  if (!parentWidget)
  {
    gzerr << "CollidingShapesGui should have a parent widget." << std::endl;
    return gazebo::GUIPlugin::eventFilter(obj, event);
  }

  // only move widget if it was the parent which moved
  // (which will always be the case unless another event filter is installed)
  if (obj != parentWidget)
  {
    return QObject::eventFilter(obj, event);
  }

  // Re-align the widget to the bottom-right of the render window.
  // the parentWidget will already have its new size.
  int width = parentWidget->size().width();
  int height = parentWidget->size().height();
  this->move(width - this->width() - 20, height - this->height() - 20);

  return gazebo::GUIPlugin::eventFilter(obj, event);
}

/////////////////////////////////////////////////
void CollidingShapesGui::OnValueChanged(int val)
{
  // std::cout << "Value changed! " << val << std::endl;
  CollidingShapesMsg m;
  m.set_type(CollidingShapesMsg::INT32);
  m.set_int_value(val);
  this->pub->Publish(m);
}

/////////////////////////////////////////////////
void CollidingShapesGui::OnPerpValueChanged(int val)
{
  std::cout << "Value changed! " << val << std::endl;
  /*CollidingShapesMsg m;
  m.set_type(CollidingShapesMsg::INT32);
  m.set_int_value(val);
  this->pub->Publish(m);*/
}

/////////////////////////////////////////////////
void CollidingShapesGui::OnButtonAutoCollide()
{
  CollidingShapesMsg m;
  m.set_type(CollidingShapesMsg::BOOLEAN);
  m.set_bool_value(true);
  this->pub->Publish(m);
}

/////////////////////////////////////////////////
void CollidingShapesGui::OnButtonSaveConfig()
{
  char * home = std::getenv("HOME");
  std::string dir;
  if (home) dir = std::string(home);
  QString qConfigFile = QFileDialog::getSaveFileName(this,
    tr("Save configuration"), dir.c_str(),
    tr("CollidingShapes test config (*.cstc);;All Files (*)"));

  if (qConfigFile.isEmpty())
  {
    std::cout << "CollidingShapesGui: Cancelled saving config." << std::endl;
    return;
  }
  std::string configFile = qConfigFile.toStdString();
  if (configFile.length() < 5)
  {
    configFile += ".cstc";
  }
  else
  {
    std::string ending = configFile.substr(configFile.length() - 5, 5);
    if (ending != ".cstc")
      configFile += ".cstc";
  }
  // std::cout << "Chosen to save as file " << configFile << std::endl;
  CollidingShapesMsg m;
  m.set_type(CollidingShapesMsg::STRING);
  m.set_string_value(configFile);
  this->pub->Publish(m);
}

/////////////////////////////////////////////////
void CollidingShapesGui::OnButtonInc()
{
  if (!this->slider) return;
  this->slider->setValue(slider->value() + 1);
}
/////////////////////////////////////////////////
void CollidingShapesGui::OnButtonDec()
{
  if (!this->slider) return;
  this->slider->setValue(slider->value() - 1);
}

/////////////////////////////////////////////////
void CollidingShapesGui::OnButtonPerpInc()
{
  std::cout << "UP" << std::endl;
}
/////////////////////////////////////////////////
void CollidingShapesGui::OnButtonPerpDec()
{
  std::cout << "DOWN" << std::endl;
}

/////////////////////////////////////////////////
void CollidingShapesGui::receiveFeedbackMsg(ConstAnyPtr &_msg)
{
  // std::cout << "GUI FEEDBACK! " << _msg->DebugString();
  switch (_msg->type())
  {
    case CollidingShapesMsg::INT32:
      {
        // std::cout << "GUI feedback: Moved shapes to "
        //           << _msg->int_value() << std::endl;
        this->slider->setValue(_msg->int_value());
        break;
      }

    default:
      std::cerr << "Unsupported AnyMsg type" << std::endl;
  }
}

