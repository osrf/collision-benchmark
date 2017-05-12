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

class StaticTestFramework : public MultipleWorldsTestFramework {
protected:
  typedef GzWorldManager::PhysicsWorldContactInterfaceT::ContactInfo
            GzContactInfo;
  typedef GzContactInfo::Ptr GzContactInfoPtr;

  StaticTestFramework():
    MultipleWorldsTestFramework()
  {}
  virtual ~StaticTestFramework()
  {}


  // \brief Initializes the framework and creates the world manager, but no
  // worlds are added to it.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void Init();

  // \brief Calls Init() and loads the empty world with all the given engines.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void InitMultipleEngines(const std::vector<std::string>& engines);

  // \brief Calls Init() and loads the empty world \e numWorld times
  // with the given engine by calling LoadOneEngine().
  // Each world can be accessed in the world
  // manager given the index [0..numWorlds-1].
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void InitOneEngine(const std::string& engine,
                     const unsigned int numWorlds);


  // \brief Loads the empty world \e numWorld times with the given engine.
  // If the world manager previously had \e n engines, then it will then have
  // \e n + \e numWorld empty worlds loaded after this call.
  // Each world can be accessed in the world
  // manager given the index ``[n-1..n+numWorlds-1]``.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void LoadOneEngine(const std::string& engine,
                     const unsigned int numWorlds);

  // \brief Loads a shape into *all* worlds.
  // You must call Init(), InitMultipleEngines() or InitOneEngine()
  // before you can use this.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void LoadShape(const collision_benchmark::Shape::Ptr& shape,
                 const std::string& modelName);


  // \brief Loads a shape into the worlds at the given index \e worldIdx.
  // You must call Init(), InitMultipleEngines() or InitOneEngine()
  // before you can use this.
  //
  // Throws gtest assertions so needs to be called from top-level
  // test function (nested function calls will not work correctly)
  void LoadShape(const collision_benchmark::Shape::Ptr& shape,
                 const std::string& modelName,
                 const unsigned int worldIdx);

  // Two models, which must already have been loaded, are moved relative to
  // each other by iterating through states in which their AABBs intersect.
  // Alll engines have to agree on the collision state (boolean collision).
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
  void AABBTestWorldsAgreement(const std::string& modelName1,
                const std::string& modelName2,
                const float cellSizeFactor = 0.1,
                const double minAgree = 0.999,
                const double bbTol = 5e-02,
                const double zeroDepthTol = 5e-02,
                const bool interactive = false,
                const std::string& outputBasePath = "",
                const std::string& outputSubdir = "");

private:

  // checks that AABB of model 1 and 2 are the same in all worlds and
  // returns the two AABBs
  // \param bbTol tolerance for comparison of bounding box sizes. The min/max
  //    coordinates (per x,y,z) are allowed to vary by this much in the worlds.
  // \return true if worlds are consistent, falsle otherwise
  bool GetAABBs(const std::string& modelName1,
                const std::string& modelName2,
                const double bbTol,
                collision_benchmark::GzAABB& m1,
                collision_benchmark::GzAABB& m2);

};

#endif  // COLLISION_BENCHMARK_TEST_STATICTESTFRAMEWORK_H
