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
#include <list>

namespace collision_benchmark
{

/**
 * \brief Strategy pattern to filter messages.
 * Messages may be simply filtered out, or modified by the filter.
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

  // Determines whether a message is filtered out. If it is not filtered
  // out, it may have been modified.
  // \return NULL if the filter does not apply
  //    to the message, otherwise the message itself (or a modification of it).
  public: virtual boost::shared_ptr<Msg const> Filter
          (const boost::shared_ptr<Msg const> &_msg) const = 0;
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
  public: typedef typename MessageFilter<Msg>::ConstPtr MessageFilterConstPtr;

  // \brief Constructor
  // \param _filter filters out all messages to which the filter applies
  //      and does not republish them. If NULL, does not filter any messages.
 public: GazeboTopicForwarder(const MessageFilterConstPtr _filter=nullptr,
                              const bool _verbose=false):
          msgFilter(_filter),
          verbose(_verbose)
          {
          }

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
                              const MessageFilterConstPtr _filter=nullptr,
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

  public: void Forward(const std::string &_from,
                       const std::string &_to,
                       const gazebo::transport::NodePtr &_node,
                       const bool _subLatching=true,
                       unsigned int _pubQueueLimit = 1000,
                       double _pubHzRate = 0)
          {
            ForwardTo(_to, _node, _pubQueueLimit, _pubHzRate);
            ForwardFrom(_from, _node, _subLatching);
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
            // std::cout<<"Subscribing to topic "<<_from<<"."<<std::endl;

            std::lock_guard<std::mutex> lock(transportMutex);

            if (!this->pub)
            {
              gzwarn<<"Publisher is not initialized, should call \
                GazeboTopicForwarder::ForwardTo first!\n";
            }

            if (this->sub)
            {
              this->sub->Unsubscribe();
            }

            this->sub = _node->Subscribe(_from, &GazeboTopicForwarder::OnMsg,
                                         this, _subLatching);


            if (this->verbose)
            {
              std::cout << "Forwarding messages (verbose: "
                        << (verbose ? "true" : "false")<<") of type "
                        << GetTypeName<Msg>()<<" from topic "
                        << _from<<" to topic ";
              if (this->pub) std::cout<<this->pub->GetTopic()<<std::endl;
              else std::cout<<"<none>"<<std::endl;
            }
          }

//  public: gazebo::transport::PublisherPtr GetPublisher() const
//          { return this->pub; }
//  public: gazebo::transport::SubscriberPtr GetSubscriber() const
//          { return this->sub; }

  // \brief Initializes the destination topic and publisher.
  // Should always be set before ForwardFrom() is called, so that the first
  // arriving messages (asynchronous) will be forwarded.
  // \param _to topic to forward to
  // \param _node the node to use for creating the publisher
  // \param _pubQueueLimit limit of the queue for the re-publisher
  // \param _pubHzRate update rate for publisher (0=fastest possible)
  public: void ForwardTo(const std::string &_to,
                     const gazebo::transport::NodePtr &_node,
                     unsigned int _pubQueueLimit = 1000,
                     double _pubHzRate = 0)
           {
             // std::cout<<"Forwarding messages of type "
             //         <<GetTypeName<Msg>()<<" to topic "<<_to<<std::endl;
             try
             {
               std::lock_guard<std::mutex> lock(transportMutex);
               this->pub =
                 _node->Advertise<Msg>(_to, _pubQueueLimit, _pubHzRate);
             } catch (gazebo::common::Exception &e)
             {
               THROW_EXCEPTION("Could not create forwarder to "
                               << _to <<" with message type "
                               << GetTypeName<Msg>() << ": " << e);
             }
           }

  private: void OnMsg(const boost::shared_ptr<Msg const> &_msg)
  {
    assert(_msg);

    if (this->verbose)
      std::cout<<"Debug: Got message of type "<<GetTypeName<Msg>()<<std::endl;

    std::lock_guard<std::mutex> lock(transportMutex);
    if (!this->sub) THROW_EXCEPTION("Inconsistency: Subscriber is NULL, "
                                    << "can't be as we are in the callback!");

    if (this->verbose)
    {
      std::cout<<" - on topic "<<this->sub->GetTopic();
      if (this->pub) std::cout<<" forwarded to topic "<<this->pub->GetTopic();
      else std::cout<<" <null>";
      std::cout<<std::endl;
    }

    if (!this->pub)
    {
      // can't re-publish message anyhow as publisher not set (yet)
      return;
    }

    boost::shared_ptr<Msg const> msgToFwd(nullptr);
    if (!msgFilter) msgToFwd = _msg;
    else msgToFwd = msgFilter->Filter(_msg);
    if (!msgToFwd)
    {
      if (this->verbose)
        std::cout<<"Debug: Rejected message of type "
          <<GetTypeName<Msg>()<<std::endl;
      return;
    }

    // this->pub->WaitForConnection();
    this->pub->Publish(*msgToFwd);
  }

  /// \brief Publisher for forwarding messages.
  private: gazebo::transport::PublisherPtr pub;

  /// \brief Subscriber to get the messages to forward.
  private: gazebo::transport::SubscriberPtr sub;

  /// \brief Mutex for the publisher and subscriber
  private: std::mutex transportMutex;

  /// \brief the message filter (optional)
  private: MessageFilterConstPtr msgFilter;

  /// \brief for debugging
  private: bool verbose;
};



/**
 * \brief Forwards requests and responses
 * from one request/response pair to another.
 *
 * This can forward requests and responses from one source service to
 * another destination service:
 *
 * ```
 * --request--> <SourceService>  ---request forward--->  <DestinationService>
 * <--response-- <SourceService>  <--response forward---  <DestinationService>
 * ```
 * \author Jennifer Buehler
 * \date February 2017
 */
class GazeboServiceForwarder
{
  private: typedef GazeboServiceForwarder Self;
  private: typedef GazeboTopicForwarder<gazebo::msgs::Request>
                     RequestForwarder;
  private: typedef GazeboTopicForwarder<gazebo::msgs::Response>
                     ResponseForwarder;

  public: typedef typename MessageFilter<gazebo::msgs::Request>::ConstPtr
                     RequestMessageFilterConstPtr;

  private: typedef boost::shared_ptr<gazebo::msgs::Request const>
                     RequestConstPtr;

  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  // \brief Constructor
  // \param _pubQueueLimit limit of the queue for the re-publisher
  //      of messages to be published
  // \param _pubHzRate update rate for re-publisher of messages
  //      (0=fastest possible)
 public: GazeboServiceForwarder(unsigned int _pubQueueLimit = 1000,
                                double _pubHzRate = 0,
                                const bool _verbose=false):
          pubQueueLimit(_pubQueueLimit),
          pubHzRate(_pubHzRate),
          verbose(_verbose)
          {}

  // Constructor
  // \param _requestSourceTopic the source service request topic
  // \param _responseSourceTopic the source service response topic
  // \param _pubQueueLimit limit of the queue for the re-publisher
  //      of messages to be published
  // \param _pubHzRate update rate for re-publisher of messages
  //      (0=fastest possible)
 public: GazeboServiceForwarder(const std::string &_requestSourceTopic,
                                const std::string &_responseSourceTopic,
                                unsigned int _pubQueueLimit = 1000,
                                double _pubHzRate = 0,
                                const bool _verbose=false):
          requestSourceTopic(_requestSourceTopic),
          responseSourceTopic(_responseSourceTopic),
          pubQueueLimit(_pubQueueLimit),
          pubHzRate(_pubHzRate),
          verbose(_verbose)
          {}

  // \brief Disconnects the subscribers
  public: void Disconnect()
          {
            if (reqFwd) reqFwd->DisconnectSubscriber();
            if (resFwd) resFwd->DisconnectSubscriber();
            UnsubscribeBufferRequests();
          }



  public: virtual ~GazeboServiceForwarder(){}

  // Buffers incoming request messages *before* Forward() is called
  // with the right topics (in case the destination topic is only known later)
  // As soon as Forward() is called, this feature is disabled again.
  // After enabling this feature, the requests which were buffered in between
  // the call of this function and the subsequent call of Forward() will
  // be forwarded first, before any new incoming requests are forwarded.
  // \param latching if true, the last arrived request will also be considered
  public: void BufferRequests(const std::string &_requestSourceTopic,
                              const gazebo::transport::NodePtr &_node,
                              const bool latching=true)
          {
            std::lock_guard<std::mutex> lock(bufferedRequestsMutex);
            if (this->bufferedRequestsSub)
              this->bufferedRequestsSub->Unsubscribe();
            this->bufferedRequestsSub =
              _node->Subscribe(_requestSourceTopic,
                               &GazeboServiceForwarder::OnRequest,
                               this, latching);
          }


  // Initiates the fowarding with the source topics which were already
  // set (previously, or in constructor)
  // \param _requestDestTopic the destination service request topic
  // \param _responseDestTopic the destination service response topic
  // \param _requestFilter filters out all request messages coming from the
  //    source service to which the filter applies and does not forward them.
  //    If NULL, does not filter messages.
  public: void Forward(const std::string &_requestDestTopic,
                        const std::string &_responseDestTopic,
                        const RequestMessageFilterConstPtr &_requestFilter,
                        const gazebo::transport::NodePtr &_node)
          {
            if (requestSourceTopic.empty() || responseSourceTopic.empty())
            {
              THROW_EXCEPTION("Source topics must have been previously set");
            }
            Forward(requestSourceTopic, responseSourceTopic,
                    _requestDestTopic, _responseDestTopic,
                    _requestFilter, _node);
          }


  // \brief Initiates the fowarding
  // \param _requestSourceTopic the source service request topic
  // \param _responseSourceTopic the source service response topic
  // \param _requestDestTopic the destination service request topic
  // \param _responseDestTopic the destination service response topic
  //  \param _requestFilter filters out all request messages coming from the
  //    source service to which the filter applies and does not forward them.
  //    If NULL, does not filter messages.
  public: void Forward(const std::string &_requestSourceTopic,
                        const std::string &_responseSourceTopic,
                        const std::string &_requestDestTopic,
                        const std::string &_responseDestTopic,
                        const RequestMessageFilterConstPtr &_requestFilter,
                        const gazebo::transport::NodePtr &_node)
          {
            // first, forward all requests which have arrived before we
            // called this and stop the buffering.
            StopBufferRequests(_requestDestTopic, _node, _requestFilter);
            requestSourceTopic=_requestSourceTopic;
            responseSourceTopic=_responseSourceTopic;
            requestDestTopic=_requestDestTopic;
            responseDestTopic=_responseDestTopic;
            if (reqFwd) reqFwd->DisconnectSubscriber();
            if (resFwd) resFwd->DisconnectSubscriber();
            reqFwd.reset(new RequestForwarder(_requestDestTopic, _node,
                                              pubQueueLimit, pubHzRate,
                                              _requestFilter, verbose));
            resFwd.reset(new ResponseForwarder(_responseSourceTopic, _node,
                                               pubQueueLimit, pubHzRate,
                                               nullptr, verbose));
            bool latching=false;
            reqFwd->ForwardFrom(_requestSourceTopic, _node, latching);
            resFwd->ForwardFrom(_responseDestTopic, _node, latching);
          }

  // Stop the buffer requests reception started with BufferRequests()
  // and send them off to _requestDestTopic
  private: void StopBufferRequests(const std::string &_requestDestTopic,
                                   const gazebo::transport::NodePtr &_node,
                                   const RequestMessageFilterConstPtr &_reqFltr)
           {
             gazebo::transport::PublisherPtr pubTmp
               = _node->Advertise<gazebo::msgs::Request>(_requestDestTopic,
                                                         pubQueueLimit,
                                                         pubHzRate);
             UnsubscribeBufferRequests();
             std::lock_guard<std::mutex> lock(bufferedRequestsMutex);
             while (!this->bufferedRequests.empty())
             {
               RequestConstPtr msg=this->bufferedRequests.front();
               this->bufferedRequests.pop_front();

               assert(msg);

               if (this->verbose)
               {
                 std::cout<<"Forwarding request: "<<msg->DebugString()
                          <<" to "<<_requestDestTopic<<std::endl;
               }

               RequestConstPtr msgToFwd(nullptr);
               if (!_reqFltr) msgToFwd = msg;
               else msgToFwd = _reqFltr->Filter(msg);
               if (!msgToFwd)
               {
                 if (this->verbose)
                   std::cout<<"Debug: Filtered out request"<<std::endl;
                 continue;
               }

               // pubTmp->WaitForConnection();
               pubTmp->Publish(*msgToFwd);
             }
           }

  private: void UnsubscribeBufferRequests()
           {
             std::lock_guard<std::mutex> lock(bufferedRequestsMutex);
             if (this->bufferedRequestsSub)
             {
               this->bufferedRequestsSub->Unsubscribe();
               this->bufferedRequestsSub.reset();
             }
           }

  private: void OnRequest(const RequestConstPtr &_request)
           {
             assert(_request);
             // std::cout << "Buffering request: "
             //           << _request->DebugString()<<std::endl;
             std::lock_guard<std::mutex> lock(bufferedRequestsMutex);
             this->bufferedRequests.push_back(_request);
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

  // \brief buffered requests (before the destination topic is known)
  private: std::list<RequestConstPtr> bufferedRequests;
  /// \brief Subscriber to fill bufferedRequests
  private: gazebo::transport::SubscriberPtr bufferedRequestsSub;
  /// \brief Mutex for bufferedRequests and bufferedRequestsSub
  private: std::mutex bufferedRequestsMutex;

  /// \brief for debugging
  private: bool verbose;

};

class GazeboTopicBlockPrinterInterface
{
  private: typedef GazeboTopicBlockPrinterInterface Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

};

/**
 * \brief Receives messages of this type on a topic
 * and prints an error that they are not supported.
 * Can also use a MessageFilter to print the blocking message only
 * certain messages.
 *
 * \author Jennifer Buehler
 * \date February 2017
 */
template<typename Msg_>
class GazeboTopicBlockPrinter:
  public GazeboTopicBlockPrinterInterface
{
  public: typedef Msg_ Msg;
  private: typedef GazeboTopicBlockPrinter<Msg> Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;
  public: typedef typename MessageFilter<Msg>::Ptr MessageFilterConstPtr;

  // \brief Constructor
  // \param _printPrefix a string that is printed first
  //    with every message
  // \param _topic topic to receive this message on
  // \param _node node to use for subscription
  public: GazeboTopicBlockPrinter(const std::string &_printPrefix,
                                  const std::string &_topic,
                                  const gazebo::transport::NodePtr &_node,
                                  const MessageFilterConstPtr _filter=nullptr):
          printPrefix(_printPrefix),
          msgFilter(_filter)
          {
            Init(_topic,_node, false);
          }
  public: virtual ~GazeboTopicBlockPrinter(){}

  // \brief Disconnects the subscribers
  public: void DisconnectSubscriber()
          {
            if (this->sub) this->sub->Unsubscribe();
          }

  private: void Init(const std::string &_topic,
                     const gazebo::transport::NodePtr &_node,
                     const bool _subLatching=false)
          {
            if (this->sub)
              this->sub->Unsubscribe();
            this->sub =
              _node->Subscribe(_topic, &GazeboTopicBlockPrinter::OnMsg,
                               this, _subLatching);
//            std::cout << "Catching messages of type "
//                      << GetTypeName<Msg>()<<" from topic "
//                      << _topic<<std::endl;
          }

  private: void OnMsg(const boost::shared_ptr<Msg const> &_msg)
  {
    assert(_msg);
    if (!msgFilter || msgFilter->Filter(_msg))
    {
      /* std::cout<<printPrefix<<": Blocked message of type "<<GetTypeName<Msg>();
      if (this->sub) std::cout<<" on topic "<<this->sub->GetTopic();
      else std::cout<<" <null>";
      std::cout<<std::endl;*/
    }
  }

  private: std::string printPrefix;

  /// \brief Subscriber to get the messages to forward.
  private: gazebo::transport::SubscriberPtr sub;

  /// \brief the message filter (optional)
  private: MessageFilterConstPtr msgFilter;
};



}  // namespace collision_benchmark
#endif
