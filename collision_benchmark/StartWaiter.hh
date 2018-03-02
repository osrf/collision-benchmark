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
/*
 * Author: Jennifer Buehler
 * Date: May 2017
 */
#ifndef COLLISION_BENCHMARK_START_WAITER_HH
#define COLLISION_BENCHMARK_START_WAITER_HH

#include <gazebo/common/Time.hh>
#include <thread>
#include <atomic>
#include <vector>

namespace collision_benchmark
{
/**
 * \brief Simple helper class to wait for the "start" signal to a program.
 *
 * Uses the terminal to either wait for a press of [Enter], or allows
 * to register a callback function to trigger the start signal alternatively
 * to pressing [Enter].
 *
 * \author Jennifer Buehler
 * \date May 2017
 */
class StartWaiter
{
  public: StartWaiter():
    unpaused(false),
    keypressed(false)
  {}

  // waits for either unpaused is set to
  // true or until enter key was pressed.
  public: void WaitForUnpause()
  {
    keypressed = false;
    std::thread * t =
      new std::thread(std::bind(&StartWaiter::WaitForEnter, this));
    t->detach();  // detach so it can be terminated
    while (!unpaused && !keypressed)
    {
      gazebo::common::Time::MSleep(100);
      bool triggered = false;
      for (std::vector<std::function<bool(void)> >::iterator
           it = unpausedCallbacks.begin(); it != unpausedCallbacks.end(); ++it)
      {
        std::function<bool(void)> func = *it;
        if (func && func())
        {
          triggered = true;
          break;
        }
      }
      if (triggered)
        break;
    }
    delete t;
  }

  // callback to trigger the pause state to \e pause.
  // One condition of WaitForUnpause() to stop waiting is when this callback
  // is called with \e pause being true.
  public: void PauseCallback(bool pause)
  {
    unpaused = !pause;
  }

  // Additional callbacks can be used to additionally determine whether there
  // has been an 'unpause'.
  // \param fct returns true if unpaused
  public: void AddUnpausedCallback(const std::function<bool(void)>& fct)
  {
    unpausedCallbacks.push_back(fct);
  }

  // waits until enter has been pressed and sets keypressed to true
  private: void WaitForEnter()
  {
    // wait for key to be pressed
    getchar();
    keypressed = true;
  }

  // test is paused or not
  private: std::atomic<bool> unpaused;
  private: std::atomic<bool> keypressed;
  private: std::vector<std::function<bool(void)> > unpausedCallbacks;
};
}
#endif  // COLLISION_BENCHMARK_START_WAITER_HH
