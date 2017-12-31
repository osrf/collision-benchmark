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
 */
#ifndef COLLISION_BENCHMARK_TEST_STEPGUI_H
#define COLLISION_BENCHMARK_TEST_STEPGUI_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

#include <string>

/**
 * \brief Creates a GUI with "Next" and "Prev" button
 * for debugging between update steps.
 *
 * Communication with the server works via gazebo::Any messages.
 * An integer of -1 is sent for "Prev", an integer of 1 for "Next".
 *
 * \author Jennifer Buehler
 * \date December 2017
 */
class GAZEBO_VISIBLE StepGui : public gazebo::GUIPlugin
{
  Q_OBJECT

  /// \brief Constructor
  /// \param[in] _parent Parent widget
  public: StepGui();

  /// \brief Destructor
  public: virtual ~StepGui();

  /// \brief Callback trigged when the button "Prev" is pressed.
  protected slots: void OnButtonPrev();

  /// \brief Callback trigged when the button "Next" is pressed.
  protected slots: void OnButtonNext();

  /// \brief Node used to establish communication with gzserver.
  private: gazebo::transport::NodePtr node;

  /// \brief Publisher for "next" and "prev" world setting
  private: gazebo::transport::PublisherPtr pub;

  /// \brief minimum size of the widget, set by the buttons width and height
  private: QSize minSize;
};
#endif
