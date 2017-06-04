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
#ifndef COLLISION_BENCHMARK_COLLIDINGSHAPESGUI_H
#define COLLISION_BENCHMARK_COLLIDINGSHAPESGUI_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

namespace collision_benchmark
{
namespace test
{

/**
 * Creates a GUI with a slider to move the two objects of
 * CollidingShapesTestFramework towards or apart from each other.
 * Also provides a button to save the configuration.
 *
 * Communication with the server works via gazebo::Any messages.
 * An integer indicates the change of the slider, a string gives the
 * filename to save the configuration to.
 *
 * \author Jennifer Buehler
 * \date May 2017
 */
class GAZEBO_VISIBLE CollidingShapesGui : public gazebo::GUIPlugin
{
  Q_OBJECT

  /// \brief Constructor
  /// \param[in] _parent Parent widget
  public: CollidingShapesGui();

  /// \brief Destructor
  public: virtual ~CollidingShapesGui();

  /// \brief Callback trigged when the slider is changed
  protected slots: void OnValueChanged(int value);

  /// \brief Callback trigged when the button "<" is pressed.
  protected slots: void OnButtonDec();

  /// \brief Callback trigged when the button ">" is pressed.
  protected slots: void OnButtonInc();


  /// \brief Node used to establish communication with gzserver.
  private: gazebo::transport::NodePtr node;

  /// \brief Publisher for the events
  private: gazebo::transport::PublisherPtr mirrorWorldPub;

  /// \brief minimum size of the widget, set by the buttons width and height
  private: QSize minSize;
};

}  // namespace test
}  // namespace collision_benchmark

#endif
