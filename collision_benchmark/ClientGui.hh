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
#ifndef COLLISION_BENCHMARK_CLIENTGUI_H
#define COLLISION_BENCHMARK_CLIENTGUI_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

namespace collision_benchmark
{
  /**
   * Creates a GUI with "Next" and "Prev" button to swith between worlds to be
   * displayed. A label displaying the name of the current worls is fitted
   * between the buttons.
   *
   * Communication with the server works via gazebo::Any messages.
   * An integer of -1 is sent for "Prev", an integer of 1 for "Next",
   * a 0 for no change and simply request the world name to be sent,
   * and a string is received for the world name.
   *
   * \author Jennifer Buehler
   * \date December 2016
   */
  class GAZEBO_VISIBLE ClientGui : public gazebo::GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor
    /// \param[in] _parent Parent widget
    public: ClientGui();

    /// \brief Destructor
    public: virtual ~ClientGui();

    /// \brief Callback trigged when the button "Prev" is pressed.
    protected slots: void OnButtonPrev();

    /// \brief Callback trigged when the button "Next" is pressed.
    protected slots: void OnButtonNext();

    /// \brief Callback triggered upon reception of the mirrored world info
    private: void receiveWorldMsg(ConstAnyPtr &_msg);

    /// changing the name of the label has to be done via a signal/slot
    /// in order to be handled by the Qt event loop. Connect this signal
    /// to OnNameChange.
    signals: void TriggerNameChange(const std::string &n);
    /// Slot for TriggerNameChange.
    private slots: void OnNameChange(const std::string &n);

    /// \brief Label to display current world name
    private: QLabel * labelName;

    /// \brief Node used to establish communication with gzserver.
    private: gazebo::transport::NodePtr node;

    /// \brief Publisher for "next" and "prev" world setting
    private: gazebo::transport::PublisherPtr mirrorWorldPub;

    /// \brief Subscriber for receiving information of the world displayed
    private: gazebo::transport::SubscriberPtr mirrorWorldSub;

    /// \brief minimum size of the widget, set by the buttons width and height
    private: QSize minSize;
  };
}
#endif
