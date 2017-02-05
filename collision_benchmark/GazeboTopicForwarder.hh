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
#include <collision_benchmark/Exception.hh>

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
/*class TopicTrigger
{
  public: typedef std::shared_ptr<TopicTrigger> Ptr;
  public: typedef std::shared_ptr<const TopicTrigger> ConstPtr;

  // Called when a topic changes from \e _oldTopic to \e _newTopic.
  // \param _incoming If true it is the incoming topic
  // (subscribed to) that changes. Otherwise, it is the outgoing (published to)
  public: virtual void Trigger(const std::string& _oldTopic,
                               const std::string& _newTopic,
                               bool _incoming) const = 0;
};*/



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

  // \brief Constructor
  // \param _to topic to forward to
  // \param _node the node to use for creating publishers and subscribers
  // \param _pubQueueLimit limit of the queue for the re-publisher
  // \param _pubHzRate update rate for publisher (0=fastest possible)
  // \param _filter filters out all messages to which the filter applies
  //      and does not republish them. If NULL, does not filter any messages.
 public: GazeboTopicForwarder(const std::string &_to,
                              const gazebo::transport::NodePtr &_node,
                              unsigned int _pubQueueLimit = 1000,
                              double _pubHzRate = 0,
                              const MessageFilterPtr _filter=nullptr,
                              const bool _verbose=false):
          msgFilter(_filter),
          verbose(_verbose)
          {
            ForwardTo(_to,_node,_pubQueueLimit,_pubHzRate);
          }
  public: virtual ~GazeboTopicForwarder(){}

  // \brief Disconnects the subscribers
  public: void DisconnectSubscriber()
          {
            if (this->sub) this->sub->Unsubscribe();
          }

  // \brief Initiates the forwarding
  // \param _from topic to forward from
  // \param _subLatching latch for latest incoming message of
  //      subscriber to topic to be forwarded
  // \param _node the node to use for creating the subscriber
  public: void ForwardFrom(const std::string &_from,
                            const gazebo::transport::NodePtr &_node,
                            const bool _subLatching=true)
          {
            if (this->sub)
              this->sub->Unsubscribe();
            this->sub = _node->Subscribe(_from, &GazeboTopicForwarder::OnMsg,
                                         this, _subLatching);
            std::cout<<"Forwarding messages of type "
                      <<GetTypeName<Msg>()<<" from topic "<<_from<<" to topic ";
            if (this->pub) std::cout<<this->pub->GetTopic()<<std::endl;
            else std::cout<<"<none>"<<std::endl;
          }

  public: gazebo::transport::PublisherPtr GetPublisher() const { return this->pub; }
  public: gazebo::transport::SubscriberPtr GetSubscriber() const { return this->sub; }

  // \brief Initializes the destination topic and publisher
  // \param _to topic to forward to
  // \param _node the node to use for creating the publisher
  // \param _pubQueueLimit limit of the queue for the re-publisher
  // \param _pubHzRate update rate for publisher (0=fastest possible)
  private: void ForwardTo(const std::string& _to,
                     const gazebo::transport::NodePtr& _node,
                     unsigned int _pubQueueLimit = 1000,
                     double _pubHzRate = 0)
           {
             // std::cout<<"Forwarding messages of type "
             //         <<GetTypeName<Msg>()<<" to topic "<<_to<<std::endl;
             try
             {
                this->pub = _node->Advertise<Msg>(_to, _pubQueueLimit, _pubHzRate);
             } catch (gazebo::common::Exception &e)
             {
               THROW_EXCEPTION("Could not create forwarder to "<<_to<<" with message type "<<GetTypeName<Msg>()<<": "<<e);
             }
           }

  private: void OnMsg(const boost::shared_ptr<Msg const> &_msg)
  {
    if (this->verbose)
    {
      std::cout<<"Debug: Got message of type "
        <<GetTypeName<Msg>();
      if (this->sub) std::cout<<" on topic "<<this->sub->GetTopic();
      else std::cout<<" Subscriber NULL?!";
      if (this->pub) std::cout<<" forwarded to topic "<<this->pub->GetTopic();
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

    // this->pub->WaitForConnection();
    this->pub->Publish(*_msg);
  }

  /// \brief Publisher for forwarding messages.
  private: gazebo::transport::PublisherPtr pub;
  /// \brief Subscriber to get the messages to forward.
  private: gazebo::transport::SubscriberPtr sub;

  private: MessageFilterPtr msgFilter;
  /// \brief for debugging
  private: bool verbose;
};



/**
 * \brief Forwards requests and responses from one request/response pair to another.
 *
 * This can forward requests and responses from one source service to another destination service:
 *
 * ```
 * ---request---> <SourceService>  ----request forward---->  <DestinationService>
 * <--response--- <SourceService>  <---response forward----  <DestinationService>
 * ```
 * \author Jennifer Buehler
 * \date February 2017
 */
class GazeboServiceForwarder
{
  private: typedef GazeboServiceForwarder Self;
  private: typedef GazeboTopicForwarder<gazebo::msgs::Request> RequestForwarder;
  private: typedef GazeboTopicForwarder<gazebo::msgs::Response> ResponseForwarder;

  public: typedef typename MessageFilter<gazebo::msgs::Request>::Ptr RequestMessageFilterPtr;
  public: typedef typename MessageFilter<gazebo::msgs::Response>::Ptr ResponseMessageFilterPtr;

  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  // \brief Constructor
  // \param _pubQueueLimit limit of the queue for the re-publisher
  //      of messages to be published
  // \param _pubHzRate update rate for re-publisher of messages
  //      (0=fastest possible)
  // \param _requestFilter filters out all request messages coming from
  //    the source service to which the filter
  //    applies and does not forward them.
  //    If NULL, does not filter messages.
 public: GazeboServiceForwarder(const RequestMessageFilterPtr &_requestFilter=nullptr,
                                unsigned int _pubQueueLimit = 1000,
                                double _pubHzRate = 0,
                                const bool _verbose=false):
          pubQueueLimit(_pubQueueLimit),
          pubHzRate(_pubHzRate),
          requestFilter(_requestFilter),
          verbose(_verbose)
          {}

  // Constructor
  // \param _requestSourceTopic the source service request topic
  // \param _responseSourceTopic the source service response topic
  // \param _pubQueueLimit limit of the queue for the re-publisher
  //      of messages to be published
  // \param _pubHzRate update rate for re-publisher of messages
  //      (0=fastest possible)
  // \param _requestFilter filters out all request messages coming from
  //    the source service to which the filter
  //    applies and does not forward them.
  //    If NULL, does not filter messages.
 public: GazeboServiceForwarder(const std::string &_requestSourceTopic,
                                const std::string &_responseSourceTopic,
                                const RequestMessageFilterPtr &_requestFilter=nullptr,
                                unsigned int _pubQueueLimit = 1000,
                                double _pubHzRate = 0,
                                const bool _verbose=false):
          requestSourceTopic(_requestSourceTopic),
          responseSourceTopic(_responseSourceTopic),
          pubQueueLimit(_pubQueueLimit),
          pubHzRate(_pubHzRate),
          requestFilter(_requestFilter),
          verbose(_verbose)
          {}



  public: virtual ~GazeboServiceForwarder(){}

  // \brief Disconnects the subscribers
  public: void Disconnect()
          {
            if (reqFwd) reqFwd->DisconnectSubscriber();
            if (resFwd) resFwd->DisconnectSubscriber();
          }

  // Initiates the fowarding with the source topics which were already
  // set (previously, or in constructor)
  // \param _requestDestTopic the destination service request topic
  // \param _responseDestTopic the destination service response topic
  public: void Forward(const std::string &_requestDestTopic,
                        const std::string &_responseDestTopic,
                        const gazebo::transport::NodePtr& _node)
          {
            if (requestSourceTopic.empty() || responseSourceTopic.empty())
            {
              THROW_EXCEPTION("Source topics must have been previously set");
            }
            Forward(requestSourceTopic, responseSourceTopic,
                    _requestDestTopic, _responseDestTopic, _node);
          }


  // \brief Initiates the fowarding
  // \param _requestSourceTopic the source service request topic
  // \param _responseSourceTopic the source service response topic
  // \param _requestDestTopic the destination service request topic
  // \param _responseDestTopic the destination service response topic
  public: void Forward(const std::string &_requestSourceTopic,
                        const std::string &_responseSourceTopic,
                        const std::string &_requestDestTopic,
                        const std::string &_responseDestTopic,
                        const gazebo::transport::NodePtr& _node)
          {
            requestSourceTopic=_requestSourceTopic;
            responseSourceTopic=_responseSourceTopic;
            requestDestTopic=_requestDestTopic;
            responseDestTopic=_responseDestTopic;
            if (reqFwd) reqFwd->DisconnectSubscriber();
            if (resFwd) resFwd->DisconnectSubscriber();
            reqFwd.reset(new RequestForwarder(_requestDestTopic, _node, pubQueueLimit, pubHzRate, requestFilter, verbose));
            resFwd.reset(new ResponseForwarder(_responseSourceTopic, _node, pubQueueLimit, pubHzRate, nullptr, verbose));
            bool latching=false;
            reqFwd->ForwardFrom(_requestSourceTopic, _node, latching);
            resFwd->ForwardFrom(_responseDestTopic, _node, latching);
          }

  // \brief the source service request topic.
  private: std::string requestSourceTopic;
  // \brief the source service response topic.
  private: std::string responseSourceTopic;

  // \brief the destination service request topic.
  private: std::string requestDestTopic;
  // \brief the destination service response topic.
  private: std::string responseDestTopic;


  /// \brief Forwarder for requests
  private: RequestForwarder::Ptr reqFwd;
  /// \brief Forwarder for responses
  private: ResponseForwarder::Ptr resFwd;

  // \brief limit of the queue for the re-publisher
  //      of messages to be published
  private: unsigned int pubQueueLimit;
  // \brief update rate for re-publisher of messages (0=fastest possible)
  private: double pubHzRate;
  // \brief filters out all request messages to which the filter
  //      applies and does not forward them. If NULL, does not filter messages.
  private: const RequestMessageFilterPtr requestFilter;

  /// \brief for debugging
  private: bool verbose;

};



}  // namespace collision_benchmark
#endif
