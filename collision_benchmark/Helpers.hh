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
 */
#ifndef COLLISION_BENCHMARK_HELPERS
#define COLLISION_BENCHMARK_HELPERS

#include <string>

namespace collision_benchmark
{
// checks if \e path is a directory, or a potential path to a
// non-existing directory
bool isDirectory(const std::string &path);

// Creates the directory given in \e dPath if it does not already exist.
// \return false on error, true if directory already exists or has been
//    successfully created.
bool makeDirectoryIfNeeded(const std::string &dPath);
}  // namespace

#endif  // COLLISION_BENCHMARK_HELPERS
