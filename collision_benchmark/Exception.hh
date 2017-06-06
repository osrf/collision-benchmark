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
 * Date: November 2016
 */

#ifndef COLLISION_BENCHMARK_EXCEPTION
#define COLLISION_BENCHMARK_EXCEPTION

#include <stdexcept>
#include <string>
#include <sstream>

#define THROW_EXCEPTION(stream) \
{ \
    std::stringstream str; \
    str << stream<< ": " << __FILE__ << ", " << __LINE__; \
    throw collision_benchmark::Exception(str.str()); \
}

namespace collision_benchmark
{
class Exception : public std::runtime_error
{
  public:
    // Constructor.
    explicit Exception(const std::string &description):
      std::runtime_error(description) {}
};
}

#endif   // COLLISION_BENCHMARK_EXCEPTION
