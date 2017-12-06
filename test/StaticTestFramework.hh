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
#ifndef COLLISION_BENCHMARK_TEST_STATICTESTFRAMEWORK_H
#define COLLISION_BENCHMARK_TEST_STATICTESTFRAMEWORK_H

#include <test/MultipleWorldsTestFramework.hh>
#include <test/TestUtils.hh>
#include <collision_benchmark/Shape.hh>

#include <string>
#include <vector>

/**
 * \brief Framework for the static test.
 *
 * Main implementation can be found in AABBTestWorldsAgreement().
 * Is meant to be used with the gtest framework.
 *
 * \author Jennifer Buehler
 * \date April 2017
 */
class StaticTestFramework : public MultipleWorldsTestFramework
{
 protected:
  typedef GzWorldManager::PhysicsWorldContactInterfaceT::ContactInfo
            GzContactInfo;
  typedef GzContactInfo::Ptr GzContactInfoPtr;

  StaticTestFramework():
    MultipleWorldsTestFramework()
  {}
  virtual ~StaticTestFramework()
  {}

  // Two models, which must already have been loaded, are moved relative to
  // each other by iterating through states in which their AABBs intersect.
  // All engines have to agree on the collision state (boolean collision).
  // You need to use a loading function such as LoadShapes() before.
  //
  // Model 1 will remain stationary, while model 2 will
  // be moved along the 3D grid which is formed by the AABB of model 1,
  // expanded by half the dimensions of the AABB of model 2.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  //
  // \param[in] cellSizeFactor the proportion of the 3D grid
  //    that will be used to determine the cell size, which is the size
  //    of cells that the models will be moved through.
  // \param[in] bbTol tolerance for comparison of bounding box sizes. The min
  //    and max coordinates (per x,y,z) are allowed to vary by this much in
  //    the worlds, otherwise an exception will be thrown.
  // \param[in] zeroDepthTol tolerance to accept contacts as zero depth
  //    contacts - all contacts as close to this tolerance to zero will be
  //    considered "just touching" contacts, and no disagreement of the
  //    engines will be considered.
  // \param[in] minAgree minimum agreement of engines that has to be reached,
  //    value in range [0..1].
  // \param[in] interactive if true, the test will be run interactively,
  //    which means the user gets the chance to start gzclient, and each
  //    test failure the test will be paused so they can look at the result.
  // \param[in] outputBasePath if not empty, this should be a writable path to
  //    a directory into which the failure results will be written. If emtpy,
  //    no failure results will be written to file. In this directory,
  //    the directory structure \e outputSubdir will be created, and the
  //    results are placed there. Resources which are written to file and
  //    referenced from the world file, e.g. meshes, may be referenced
  //    relative path \e outputSubdir, so not containing
  //    \e outputBasePath.
  // \param outputSubdir subdirectory of \e outputBasePath where the result
  //    files will be written to. Resource references use this relative path.
  //    If \e outputBasePath is emtpy, this parameter will have no effect.
  void AABBTestWorldsAgreement(const std::string &modelName1,
                const std::string &modelName2,
                const float cellSizeFactor = 0.1,
                const double minAgree = 0.999,
                const double bbTol = 5e-02,
                const double zeroDepthTol = 5e-02,
                const bool interactive = false,
                const std::string &outputBasePath = "",
                const std::string &outputSubdir = "");
};

#endif  // COLLISION_BENCHMARK_TEST_STATICTESTFRAMEWORK_H
