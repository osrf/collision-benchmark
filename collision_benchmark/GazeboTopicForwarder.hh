/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef COLLISION_BENCHMARK_GAZEBOTOPICFORWARDER_H
#define COLLISION_BENCHMARK_GAZEBOTOPICFORWARDER_H

#include <collision_benchmark/MirrorWorld.hh>
#include <collision_benchmark/TypeHelper.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/transport/Subscriber.hh>

#include <string>
#include <iostream>
#include <memory>

namespace collision_benchmark
{

/**
 * \brief Strategy pattern to filter messages
 * \author Jennifer Buehler
 * \date February 2017
 */
template<typename Msg_>
class MessageFilter
{
  public: typedef Msg_ Msg;
  private: typedef  MessageFilter<Msg> Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  // Determines whether a message is filtered out
  // \return true if the filter applies to the message
  public: virtual bool Filter(const boost::shared_ptr<Msg const> &_msg) const = 0;
};

/**
 * \brief Strategy pattern which will be applied when a topic changes.
 * To be used with GazeboTopicForwarder.
 * \author Jennifer Buehler
 * \date February 2017
 */
class TopicTrigger
{
  public: typedef std::shared_ptr<TopicTrigger> Ptr;
  public: typedef std::shared_ptr<const TopicTrigger> ConstPtr;

  // Called when a topic changes from \e _oldTopic to \e _newTopic.
  // \param _incoming If true it is the incoming topic
  // (subscribed to) that changes. Otherwise, it is the outgoing (published to)
  public: virtual void Trigger(const std::string& _oldTopic,
                               const std::string& _newTopic,
                               bool _incoming) const = 0;
};



/**
 * \brief forwards messages from one topic to another
 * \author Jennifer Buehler
 * \date February 2017
 */
template<typename Msg_>
class GazeboTopicForwarder
{
  public: typedef Msg_ Msg;
  private: typedef GazeboTopicForwarder<Msg> Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;
  public: typedef typename MessageFilter<Msg>::Ptr MessageFilterPtr;
  public: typedef TopicTrigger::Ptr TopicTriggerPtr;
  // Constructor
  // \param _from topic to dedirect from
  // \param _to topic to redirect to
  // \param _node the node to use for creating publishers and subscribers
  // \param _pubQueueLimit limit of the queue for the re-publisher
  // \param _pubHzRate update rate for publisher (0=fastest possible)
  // \param _subLatching latch for latest incoming message of
  //      subscriber to topic to be forwarded
  // \param _filter filters out all messages to which the filter applies
  //      and does not republish them. If NULL, does not filter any messages.
  // \param _topicTrigger method TopicTrigger::Trigger() will be called
  //      each time the topic changes (except the first time when it
  //      wasn't subscribed yet).
  public: GazeboTopicForwarder(const std::string &_from,
                               const std::string &_to,
                               const gazebo::transport::NodePtr &_node,
                               unsigned int _pubQueueLimit = 1000,
                               double _pubHzRate = 0,
                               const bool _subLatching=true,
                               const MessageFilterPtr &_filter=nullptr,
                               const TopicTriggerPtr &_topicTrigger=nullptr,
                               const bool _verbose=false):
          msgFilter(_filter),
          topicTrigger(_topicTrigger),
          verbose(_verbose)
          {
            Init(_to,_node,_pubQueueLimit,_pubHzRate);
            RedirectFrom(_from, _node, _subLatching);
          }

  // Constructor
  // \param _from topic to dedirect from
  // \param _node the node to use for creating publishers and subscribers
  // \param _pubQueueLimit limit of the queue for the re-publisher
  // \param _pubHzRate update rate for publisher (0=fastest possible)
  // \param _filter filters out all messages to which the filter applies
  //      and does not republish them. If NULL, does not filter any messages.
  // \param _topicTrigger method TopicTrigger::Trigger() will be called
  //      each time the topic changes (except the first time when it
  //      wasn't subscribed yet).
 public: GazeboTopicForwarder(const std::string& _from,
                              const gazebo::transport::NodePtr& _node,
                              unsigned int _pubQueueLimit = 1000,
                              double _pubHzRate = 0,
                              const MessageFilterPtr _filter=nullptr,
                              const TopicTriggerPtr &_topicTrigger=nullptr,
                              const bool _verbose=false):
          msgFilter(_filter),
          verbose(_verbose)
          {
            Init(_from,_node,_pubQueueLimit,_pubHzRate);
          }
  public: virtual ~GazeboTopicForwarder(){}

  // Disconnects the subscribers
  public: void DisconnectSubscriber()
          {
            if (this->sub) this->sub->Unsubscribe();
          }

  // Sets the topic to redirect from
  // \param _from topic to dedirect from
  // \param _subLatching latch for latest incoming message of
  //      subscriber to topic to be forwarded
  // \param _node the node to use for creating the subscriber
  public: void RedirectFrom(const std::string& _from,
                            const gazebo::transport::NodePtr& _node,
                            const bool _subLatching=true)
          {
            if (this->sub)
            {
              this->sub->Unsubscribe();
              if (this->topicTrigger)
                this->topicTrigger->Trigger(this->sub->GetTopic(),_from, true);
            }
            this->sub = _node->Subscribe(_from, &GazeboTopicForwarder::OnMsg,
                                         this, _subLatching);
          }

  public: gazebo::transport::PublisherPtr GetPublisher() const { return this->pub; }
  public: gazebo::transport::SubscriberPtr GetSubscriber() const { return this->sub; }

  // \param _to topic to redirect to
  // \param _node the node to use for creating the publisher
  // \param _pubQueueLimit limit of the queue for the re-publisher
  // \param _pubHzRate update rate for publisher (0=fastest possible)
  private: void Init(const std::string& _to,
                     const gazebo::transport::NodePtr& _node,
                     unsigned int _pubQueueLimit = 1000,
                     double _pubHzRate = 0)
           {
              this->pub = _node->Advertise<Msg>(_to, _pubQueueLimit, _pubHzRate);
           }

  private: void OnMsg(const boost::shared_ptr<Msg const> &_msg)
  {
    if (this->verbose)
    {
      std::cout<<"Debug: Got message of type "
        <<GetTypeName<Msg>();
      if (this->sub) std::cout<<" on topic "<<this->sub->GetTopic();
      else std::cout<<" Subscriber NULL?!";
      if (this->pub) std::cout<<" to topic "<<this->pub->GetTopic();
      else std::cout<<" Publisher NULL?!";
      std::cout<<std::endl;
    }
    assert(_msg);
    if (msgFilter && msgFilter->Filter(_msg))
    {
      if (this->verbose)
        std::cout<<"Debug: Rejected message of type "
          <<GetTypeName<Msg>()<<std::endl;
      return;
    }
    this->pub->Publish(*_msg);
  }

  /// \brief Publisher for forwarding messages.
  private: gazebo::transport::PublisherPtr pub;
  /// \brief Subscriber to get the messages to forward.
  private: gazebo::transport::SubscriberPtr sub;

  private: MessageFilterPtr msgFilter;
  private: TopicTriggerPtr topicTrigger;
  /// \brief for debugging
  private: bool verbose;
};

}  // namespace collision_benchmark
#endif
