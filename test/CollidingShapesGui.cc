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
#include <gazebo/msgs/msgs.hh>
#include <sstream>

#include <QScrollBar>
#include <QSlider>

#include "CollidingShapesGui.hh"
#include "CollidingShapesParams.hh"

using collision_benchmark::test::CollidingShapesGui;
using collision_benchmark::test::CollidingShapesParams;

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
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame and layout to hold all elements
  QFrame *collidingShapesFrame = new QFrame();
  QHBoxLayout *collidingShapesLayout = new QHBoxLayout();

  // Button to the left
  QPushButton * buttonDec = new QPushButton("<");
  buttonDec->resize(buttonDec->sizeHint());
  connect(buttonDec, SIGNAL(clicked()), this, SLOT(OnButtonDec()));
  collidingShapesLayout->addWidget(buttonDec);

  slider = new QSlider(Qt::Horizontal);
  slider->setMinimum(0);
  slider->setMaximum(CollidingShapesParams::MaxSliderVal);
  slider->setValue(CollidingShapesParams::MaxSliderVal);
  slider->resize(300, 20);
  connect(slider, SIGNAL(valueChanged(int)),
          this, SLOT(OnValueChanged(int)));
  collidingShapesLayout->addWidget(slider);

  // Button to the right
  QPushButton * buttonInc = new QPushButton(">");
  buttonInc->resize(buttonInc->sizeHint());
  connect(buttonInc, SIGNAL(clicked()), this, SLOT(OnButtonInc()));
  collidingShapesLayout->addWidget(buttonInc);

  QPushButton * buttonAutoCollide = new QPushButton("AutoCollide");
  buttonAutoCollide->resize(buttonAutoCollide->sizeHint());
  connect(buttonAutoCollide, SIGNAL(clicked()), this,
          SLOT(OnButtonAutoCollide()));
  collidingShapesLayout->addWidget(buttonAutoCollide);

  QPushButton * buttonSave = new QPushButton("Save config");
  buttonSave->resize(buttonSave->sizeHint());
  connect(buttonSave, SIGNAL(clicked()), this,
          SLOT(OnButtonSaveConfig()));
  collidingShapesLayout->addWidget(buttonSave);

  // set the layout and add the frame as widget
  collidingShapesFrame->setLayout(collidingShapesLayout);
  mainLayout->addWidget(collidingShapesFrame);

  // Remove margins to reduce space
  collidingShapesLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  this->setLayout(mainLayout);

  // Resize this widget
  QSize buttonsSize = maxHeightAddWidth(buttonInc->size(), buttonDec->size());
  buttonsSize = maxHeightAddWidth(buttonsSize, buttonSave->size());
  buttonsSize = maxHeightAddWidth(buttonsSize, buttonAutoCollide->size());
  QSize totalSize = maxHeightAddWidth(buttonsSize, slider->size());
  this->resize(totalSize);

  // Set up transportation system
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init();
  std::string pub_topic="collide_shapes_test/control";
  this->pub = this->node->Advertise<gazebo::msgs::Any>(pub_topic);

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
  gazebo::msgs::Any m;
  m.set_type(gazebo::msgs::Any::INT32);
  m.set_int_value(val);
  this->pub->Publish(m);
}

/////////////////////////////////////////////////
void CollidingShapesGui::OnButtonAutoCollide()
{
  gazebo::msgs::Any m;
  m.set_type(gazebo::msgs::Any::BOOLEAN);
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
  gazebo::msgs::Any m;
  m.set_type(gazebo::msgs::Any::STRING);
  m.set_string_value(configFile);
  this->pub->Publish(m);
}

/////////////////////////////////////////////////
void CollidingShapesGui::OnButtonInc()
{
  if (!slider) return;
  slider->setValue(slider->value() + 1);
}
/////////////////////////////////////////////////
void CollidingShapesGui::OnButtonDec()
{
  if (!slider) return;
  slider->setValue(slider->value() - 1);
}


/////////////////////////////////////////////////
void CollidingShapesGui::receiveFeedbackMsg(ConstAnyPtr &_msg)
{
  // std::cout << "GUI FEEDBACK! " << _msg->DebugString();
  switch (_msg->type())
  {
    case gazebo::msgs::Any::INT32:
      {
        // std::cout << "GUI feedback: Moved shapes to "
        //           << _msg->int_value() << std::endl;
        slider->setValue(_msg->int_value());
        break;
      }

    default:
      std::cerr << "Unsupported AnyMsg type" << std::endl;
  }
}

