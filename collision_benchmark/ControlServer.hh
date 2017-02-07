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
 * Date: February 2017
 */
#ifndef COLLISION_BENCHMARK_CONTROLSERVER_H
#define COLLISION_BENCHMARK_CONTROLSERVER_H

#include <collision_benchmark/Exception.hh>
#include <collision_benchmark/BasicTypes.hh>

#include <string>
#include <iostream>
#include <functional>
#include <vector>
#include <memory>

namespace collision_benchmark
{

/**
 * \brief Implements certain controls that can be applied to a world.
 * Detaches the implementation of how the controls are received from
 * the application to the worlds. Application to the worlds can be
 * done via callbacks which need to be registered via this interface.
 *
 * \param _ModelID the identifier for a specific model
 * \author Jennifer Buehler
 * \date February 2017
 */
template<class _ModelID>
class ControlServer
{
  public: typedef _ModelID ModelID;
  public: typedef std::shared_ptr<ControlServer> Ptr;
  public: typedef std::shared_ptr<const ControlServer> ConstPtr;

  // function to set a paused flag
  public: typedef std::function<void(const bool)> NotifyPauseFct;

  // function to trigger an update of the world with a
  // number of steps/iterations
  public: typedef std::function<void(const int)> NotifyUpdateFct;

  // funciton to select one of several worlds (identified by
  // and index) as the currently selected world.
  public: typedef std::function<std::string(const int)>
            NotifySelectWorldFct;

  // function to set the position, rotation and scale of a model.
  public: typedef std::function<void(const ModelID&, const BasicState&)>
            NotifySetModelStateFct;

  /// Constructor.
  public:  ControlServer() {}
  public: ControlServer(const ControlServer& o):
          pauseCallbacks(o.pauseCallbacks),
          updateCallbacks(o.updateCallbacks),
          modelStateCallbacks(o.modelStateCallbacks) {}
  public:  virtual ~ControlServer() {}

  // register a pause callback which will be called
  // when the world paused state is to be set to \e _flag
  public: void RegisterPauseCallback(const NotifyPauseFct &_fct)
          {
            pauseCallbacks.push_back(_fct);
          }

  // register an update callback, called when the world is
  // to be updated by \e _numSteps steps.
  // \param _numSteps number of steps to run the
  //  world for. If 0, run indefinitely.
  public: void RegisterUpdateCallback(const NotifyUpdateFct &_fct)
          {
            updateCallbacks.push_back(_fct);
          }

  // register model change callback, called when the state of the model
  // is to be updated.
  public: void RegisterSetModelStateCallback(const NotifySetModelStateFct &_fct)
          {
            modelStateCallbacks.push_back(_fct);
          }

  // register callback for requests that the world with this index
  // is to be chosen as the currently selected world.
  // Only one callback can be registered as this is a service
  // which returns a value (the name of the world).
  // \param _fct the callback function. Must return the name of
  // the world at this index is returned, or an empty string if the
  // index is out of range.
  public: void RegisterSelectWorldService(const NotifySelectWorldFct &_fct)
          {
            if (selectWorldCallback)
            {
              THROW_EXCEPTION("The callback to set the selected world was already set");
            }
            selectWorldCallback.reset(new NotifySelectWorldFct(_fct));
          }


  // must be called by subclasses when the world paused
  // state is to be set to \e _flag
  protected: void NotifyPause(const bool _flag)
             {
                std::vector<NotifyPauseFct>::iterator it;
                for (it=pauseCallbacks.begin();
                     it!=pauseCallbacks.end(); ++it)
                {
                  (*it)(_flag);
                }
             }

  // must be called by subclasses when the world is
  // to be updated by \e _numSteps steps.
  // \param _numSteps number of steps to run the
  //  world for. If 0, run indefinitely.
  protected: void NotifyUpdate(const int _numSteps)
             {
                std::vector<NotifyUpdateFct>::iterator it;
                for (it=updateCallbacks.begin();
                     it!=updateCallbacks.end(); ++it)
                {
                  (*it)(_numSteps);
                }
             }

  // must be called by subclasses when the model is to be changed
  protected: void NotifySetModelState(const ModelID& id,
                                      const BasicState &_state)
             {
                typename std::vector<NotifySetModelStateFct>::iterator it;
                for (it=modelStateCallbacks.begin();
                     it!=modelStateCallbacks.end(); ++it)
                {
                  (*it)(id,_state);
                }
             }

  // must be called by subclasses when the world with this
  // index is to be chosen as the selected world.\
  // Will call the callback function registered with
  // RegisterSelectWorldCallback(const NotifySelectWorldFct &_fct)
  // and return its value, or the empty string if the callback
  // was not set. The subclass should then return the return value
  // (the world name) as the service response.
  protected: std::string CallSelectWorld(const int _index)
           {
             if (selectWorldCallback)
              return (*selectWorldCallback)(_index);
             return std::string();
           }

  private: std::vector<NotifyPauseFct> pauseCallbacks;
  private: std::vector<NotifyUpdateFct> updateCallbacks;
  private: std::vector<NotifySetModelStateFct> modelStateCallbacks;
  private: std::shared_ptr<NotifySelectWorldFct> selectWorldCallback;


};

}  // namespace collision_benchmark
#endif
