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
#ifndef COLLISION_BENCHMARK_SIGNALRECEIVER_H
#define COLLISION_BENCHMARK_SIGNALRECEIVER_H

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <string>
#include <map>
#include <mutex>

namespace collision_benchmark
{
  /**
   * \brief Convenience class to receive signals from keyboard or via message
   *
   * Signals can be sent to this receiver as string or int values.
   * One example is via a msgs::Any message, which is the only message
   * type implemented so far.
   *
   * \author Jennifer Buehler
   * \date December 2017
   */
  class GAZEBO_VISIBLE SignalReceiver
  {
    /// \brief Constructor
    public: SignalReceiver();

    /// \brief Destructor
    public: virtual ~SignalReceiver();

    // \brief Received signals since the last call of ClearReceivedSignals()
    // or WaitForSignal()
    public: std::set<int> GetReceivedSignals() const;

    // \brief Clear received signals
    public: void ClearReceivedSignals();

    // \brief Checks whether this signal was received
    public: bool ReceivedSignal(const int sig);

    // \brief Returns all signals in \e sig which have been received
    // since the last call of ClearReceivedSignals() or WaitForSignal()
    public: std::set<int> ReceivedSignal(const std::set<int> &sig);

    // \brief Waits for this signal arriving from the time this was called
    // and then removes it from the received signals
    public: void WaitForSignal(const int sig);

    // \brief Waits for one of the signals in \e sig
    // and returns the signals in \e sigs which have happened.
    // All signals \e sigs are first removed from the received signals list,
    // so it waits for the arrival of a new signal in \e sigs.
    public: std::set<int> WaitForSignal(const std::set<int> &sigs);

    // \brief initialize topic to receive msgs::Any messages on
    public: void InitAnyMsg(const std::string& topic);


    // Additional callbacks can be used to determine whether there
    // has been a signal.
    // Currently, this callbacks are only checked in the WaitForSignal()
    // functions.
    // \param[in] sigID signal ID to use for this signal. Will be returned
    //    with GetReceivedSignals after a signal was indicated by the callback.
    public: void AddCallback(const int sigID,
                             const std::function<bool(void)>& fct);

    // \brief add a string signal
    // \param[in] sigID signal ID to use for this signal. Will be returned
    //    with GetReceivedSignals after a signal of \e strVal was received.
    // \param[in] strVal the string value for this signal.
    //    For example, each time an Any message arrives with this string,
    //    a signal of \e sigID will be returned with GetReceivedSignals().
    public: void AddStringSignal(const int sigID, const std::string &strVal);

    // \brief like AddStringSignal(), but for int values
    public: void AddIntSignal(const int sigID, const int intVal);

    /// \brief Callback triggered upon reception of an Any message
    private: void ReceiveAnyMsg(ConstAnyPtr &msg);

    /// \brief Helper which checks all callbacks registered with AddCallback()
    private: void CheckCallbacks();

    /// \brief table of all registered string signals and their signal IDs
    private: std::map<std::string, int> stringSignals;

    /// \brief table of all registered int signals and their signal IDs
    private: std::map<int, int> intSignals;

    /// \brief set of received signals
    private: std::set<int> receivedSignals;
    /// \brief mutex for receivedSignals
    private: mutable std::mutex receivedSignalsMtx;

    /// \brief Node used to establish communication with gzserver.
    private: gazebo::transport::NodePtr node;

    /// \brief Subscriber for receiving msgs::Any messages.
    private: gazebo::transport::SubscriberPtr anySub;

    /// \brief Callbacks for signals
    private: std::map<int, std::function<bool(void)>> callbacks;
  };
}
#endif
