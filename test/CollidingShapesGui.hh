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

#include <QDial>
#include <QSlider>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

namespace collision_benchmark
{
namespace test
{

namespace msgs
{
  // forward declaration of message
  class CollidingShapesMsg;
}

typedef const boost::shared_ptr<const msgs::CollidingShapesMsg>
        ConstCollidingShapesMsgPtr;


/**
 * Creates a GUI with a slider to move the two objects of
 * CollidingShapesTestFramework towards or apart from each other.
 * Also provides buttons to auto-collide and to save the configuration.
 *
 * Communication with the server works via the message
 * collision_benchmark::test::msgs::CollidingShapesMsg.
 *
 * Publication:
 * - When COLLISION_SLIDER is set as type, an integer indicates the slider
 *   position (0..MaxSliderVal).
 * - When AUTO_COLLIDE is set as type, the auto-collide has been triggered.
 * - When SAVE_CONFIG is set as type,  a string gives the filename to save the
 *   configuration to.
 * - And additional test functionality provides
 *      - When PERPENDICULAR_VALUE is set, an int value indicates the number of
 *        steps a model should move away from the collision axis, on an axis
 *        perpendicular to the collision axis.
 *      - When PERPENDICULAR_ANGLE is set, a double angle [0..2*M_PI] is returned
 *        which indicates the rotation of the shape around the collision axis
 *
 * Subscription:
 * With the type COLLISION_SLIDER set, an integer can be sent to indicate when
 * the shapes have been moved along the collision axis (given in step sizes of
 * the slider bar, 0..MaxSliderVal).
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

  public: virtual void Load(sdf::ElementPtr /*_sdf*/);

  /// \brief Callback trigged when the slider is changed
  protected slots: void OnValueChanged(int value);

  /// \brief Callback trigged when the slider is changed
  protected slots: void OnDialValueChanged(int value);

  /// \brief Callback trigged when the button "<" is pressed.
  protected slots: void OnButtonDec();

  /// \brief Callback trigged when the button ">" is pressed.
  protected slots: void OnButtonInc();

  /// \brief Callback trigged when the button "<" is pressed
  /// for perpendicular move.
  protected slots: void OnButtonPerpDec();

  /// \brief Callback trigged when the button ">" is pressed
  /// for perpendicular move.
  protected slots: void OnButtonPerpInc();

  /// \brief Callback trigged when the button "AutoCollide" is pressed.
  protected slots: void OnButtonAutoCollide();

  /// \brief Callback trigged when the button "Save config" is pressed.
  protected slots: void OnButtonSaveConfig();

  protected: bool eventFilter(QObject *obj, QEvent *event);

  /// \brief receives feedback from the test
  private: void receiveFeedbackMsg(ConstCollidingShapesMsgPtr &_msg);

  /// \brief Node used to establish communication with gzserver.
  private: gazebo::transport::NodePtr node;

  /// \brief Publisher for the events
  private: gazebo::transport::PublisherPtr pub;
  /// \brief Subscriber for feedback
  private: gazebo::transport::SubscriberPtr sub;

  /// \brief the slider object for colliding the models
  private: QSlider * slider;

  /// \brief the slider object for moving objects perpendicular
  private: QDial * dial;
};

}  // namespace test
}  // namespace collision_benchmark

#endif
