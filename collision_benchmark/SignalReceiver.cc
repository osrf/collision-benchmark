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
 * Date: December 2016
 */

#include "SignalReceiver.hh"
#include <algorithm>

using collision_benchmark::SignalReceiver;

/////////////////////////////////////////////////
SignalReceiver::SignalReceiver()
{
}

/////////////////////////////////////////////////
SignalReceiver::~SignalReceiver()
{
}

/////////////////////////////////////////////////
void SignalReceiver::InitAnyMsg(const std::string& topic)
{
  if (!this->node)
  {
    // Set up transportation system. Don't do this in constructor as it is
    // possible the constructor is called before transport is initialized.
    this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->node->Init();
  }

  this->anySub =
    this->node->Subscribe(topic, &SignalReceiver::ReceiveAnyMsg, this);
}


/////////////////////////////////////////////////
void SignalReceiver::AddCallback(const int sigID,
                                 const std::function<bool(void)>& fct)
{
  callbacks[sigID]=fct;
}

/////////////////////////////////////////////////
void SignalReceiver::AddStringSignal(const int sigID, const std::string &strVal)
{
  stringSignals[strVal] = sigID;
}

/////////////////////////////////////////////////
void SignalReceiver::AddIntSignal(const int sigID, const int intVal)
{
  intSignals[intVal] = sigID;
}

/////////////////////////////////////////////////
std::set<int> SignalReceiver::GetReceivedSignals() const
{
  std::lock_guard<std::mutex> lock(receivedSignalsMtx);
  return receivedSignals;
}

/////////////////////////////////////////////////
void SignalReceiver::ClearReceivedSignals()
{
  std::lock_guard<std::mutex> lock(receivedSignalsMtx);
  receivedSignals.clear();
}

/////////////////////////////////////////////////
bool SignalReceiver::ReceivedSignal(const int sig)
{
  std::lock_guard<std::mutex> lock(receivedSignalsMtx);
  return receivedSignals.find(sig) != receivedSignals.end();
}

/////////////////////////////////////////////////
std::set<int> SignalReceiver::ReceivedSignal(const std::set<int> &sig)
{
  std::lock_guard<std::mutex> lock(receivedSignalsMtx);
  std::set<int> intr;
  std::set_intersection(sig.begin(), sig.end(),
                        receivedSignals.begin(), receivedSignals.end(),
                        std::inserter(intr, intr.end()));
  return intr;
}

/////////////////////////////////////////////////
void SignalReceiver::ReceiveAnyMsg(ConstAnyPtr &msg)
{
  // std::cout << "Any msg: " << msg->DebugString();
  if (msg->has_string_value())
  {
    std::string str = msg->string_value();
    std::map<std::string, int>::const_iterator it = stringSignals.find(str);
    if (it != stringSignals.end())
    {
      std::lock_guard<std::mutex> lock(receivedSignalsMtx);
      receivedSignals.insert(it->second);
    }
  }
  if (msg->has_int_value())
  {
    int i = msg->int_value();
    std::map<int, int>::const_iterator it = intSignals.find(i);
    if (it != intSignals.end())
    {
      std::lock_guard<std::mutex> lock(receivedSignalsMtx);
      receivedSignals.insert(it->second);
    }
  }
}

/////////////////////////////////////////////////
void SignalReceiver::CheckCallbacks()
{
  for (std::map<int, std::function<bool(void)>>::const_iterator
       it = callbacks.begin(); it != callbacks.end(); ++it)
  {
    const std::function<bool(void)> &cb = it->second;
    if (cb())
    {
      std::lock_guard<std::mutex> lock(receivedSignalsMtx);
      receivedSignals.insert(it->first);
    }
  }
}

/////////////////////////////////////////////////
void SignalReceiver::WaitForSignal(const int sig)
{
  // we have to wait for a new signal (not an old one), so erase the signal
  {
    std::lock_guard<std::mutex> lock(receivedSignalsMtx);
    receivedSignals.erase(sig);
  }
  while (!ReceivedSignal(sig))
  {
    gazebo::common::Time::MSleep(100);
    CheckCallbacks();
  }
  std::lock_guard<std::mutex> lock(receivedSignalsMtx);
  receivedSignals.erase(sig);
}


/////////////////////////////////////////////////
std::set<int> SignalReceiver::WaitForSignal(const std::set<int> &sigs)
{
  // we have to wait for a new signal (not an old one), so erase
  // all signals.
  {
    std::lock_guard<std::mutex> lock(receivedSignalsMtx);
    for (std::set<int>::const_iterator it = sigs.begin();
         it != sigs.end(); ++it)
      receivedSignals.erase(*it);
  }
  std::set<int> receivedSigs;
  while ((receivedSigs=ReceivedSignal(sigs)).empty())
  {
    gazebo::common::Time::MSleep(100);
    CheckCallbacks();
  }
  std::lock_guard<std::mutex> lock(receivedSignalsMtx);
  for (std::set<int>::const_iterator it = receivedSigs.begin();
       it != receivedSigs.end(); ++it)
    receivedSignals.erase(*it);
  return receivedSigs;
}

/////////////////////////////////////////////////
std::set<int> SignalReceiver::WaitForAnySignal()
{
  // we have to wait for a new signal (not an old one), so erase
  // all signals.
  {
    std::lock_guard<std::mutex> lock(receivedSignalsMtx);
    receivedSignals.clear();
  }
  std::set<int> receivedSigs;
  while ((receivedSigs=GetReceivedSignals()).empty())
  {
    gazebo::common::Time::MSleep(100);
    CheckCallbacks();
  }
  std::lock_guard<std::mutex> lock(receivedSignalsMtx);
  receivedSignals.clear();
  return receivedSigs;
}
