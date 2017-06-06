/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <iostream>

int g_receivedPosesStamped = 0;
void ReceivePosesStampedMsgCounter(ConstPosesStampedPtr &/*_msg*/)
{
  ++g_receivedPosesStamped;
}

int main(int argc, char** argv)
{
  gazebo::setupServer(argc, argv);

  // register a namespace
  gazebo::transport::TopicManager::Instance()->RegisterTopicNamespace("transp");

  gazebo::msgs::PosesStamped msg;
  gazebo::msgs::Init(msg, "test");

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // create the publisher
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::PosesStamped>("~/test");

  std::cout << "Run the following command in a new terminal:" << std::endl;
  std::cout << "gz topic -e /gazebo/transp/test" << std::endl;

  // wait for the remote connection
  // IMPORTANT: This error will only be triggered with remote
  // connections, because transport::Publication::Publish() returns
  // the number of remote connections only.
  pub->WaitForConnection();

  // connection received, continue.
  std::cout << "Received remote connection." << std::endl;

  // now make an additional subscriber which will count the
  // number of messages arrived and trigger the dead-end for messages.
  g_receivedPosesStamped = 0;
  gazebo::transport::SubscriberPtr sceneSub =
    node->Subscribe("~/test", &ReceivePosesStampedMsgCounter);

  // pub->WaitForConnection();

  // the dead-end happens randomly due to the multi-threading.
  // Send a large numer of messages to increase the chance this happens.
  int numMsgs = 1000;

  for (int i = 0; i < numMsgs; ++i)
  {
    std::cout << "SENDING MESSAGE! " << i << "..." << std::endl;
    gazebo::msgs::Set(msg.mutable_time(), gazebo::common::Time(i));
    // do a direct publishing in which the message should
    // be written out right away
    pub->Publish(msg, true);
  }

  std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  // if this is skipped, then the test will fail, as some
  // left-over messages which could not be sent immediately
  // will remain in the message queue in transport::Publisher.
  while (pub->GetOutgoingCount() > 0)
  {
    std::cout << "Getting out last messages, got "
      <<pub->GetOutgoingCount() << " left." << std::endl;
     pub->SendMessage();
    gazebo::common::Time::MSleep(200);
  }

  static const int timeoutSecs = 10;
  int sleepTime = 0;
  while ((g_receivedPosesStamped != numMsgs) && (sleepTime < timeoutSecs))
  {
    std::cout << "SLEEP " << sleepTime << std::endl;
    // not very nice but sleep just to make sure
    // enough time has passed for all messages to arrive
    gazebo::common::Time::Sleep(1);
    sleepTime += 1;
  }

  if (g_receivedPosesStamped != numMsgs)
    std::cerr << "ERROR: Have not received all messages! Sent " <<numMsgs
              << " and received " << g_receivedPosesStamped << std::endl;
  else
    std::cout << "SUCCESS" << std::endl;
}
