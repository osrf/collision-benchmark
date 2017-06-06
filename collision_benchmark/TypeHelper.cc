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
 * Date: December 2016
 */
#include <collision_benchmark/TypeHelper.hh>

///////////////////////////////
#ifdef __GNUG__

#include <cstdlib>
#include <memory>
#include <cxxabi.h>
#include <bits/unique_ptr.h>

std::string collision_benchmark::Demangle(const char* name)
{
  int status = -4;  // some arbitrary value to eliminate the compiler warning

  // enable c++11 by passing the flag -std = c++11 to g++
  std::unique_ptr<char, void(*)(void*)> res
  {
    abi::__cxa_demangle(name, NULL, NULL, &status),
    std::free
  };

  return (status == 0) ? res.get() : name;
}

///////////////////////////////
#else

// does nothing if not g++
std::string collision_benchmark::Demangle(const char *name)
{
  return name;
}

///////////////////////////////
#endif

