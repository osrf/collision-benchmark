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
 * Desc: Collection of basic types to ease protability of data
 * Author: Jennifer Buehler
 * Date: February 2017
 */
#ifndef COLLISION_BENCHMARK_BASICTYPES_H
#define COLLISION_BENCHMARK_BASICTYPES_H

#include <collision_benchmark/Exception.hh>

#include <string>
#include <iostream>
#include <functional>
#include <vector>
#include <memory>

namespace collision_benchmark
{

// simple 3D vector data with no functionality,
// just for portability of data
class Vector3
{
  public: Vector3(double _x, double _y, double _z):
    x(_x), y(_y), z(_z) {}
  public: Vector3(const Vector3& o):
    x(o.x), y(o.y), z(o.z) {}
  public: double x,y,z;
};

// simple quaternion data with no functionality,
// just for portability of data
class Quaternion: public Vector3
{
  public: Quaternion(double _x, double _y, double _z, double _w):
          Vector3(_x,_y,_z), w(_w) {}
  public: Quaternion(const Quaternion& o):
          Vector3(o), w(o.w) {}
  public: double w;
};

// simple state of an object
// Not all values are mandatory.
// If disabled, assumes current values are kept.
class BasicState
{
  // constructor which enables only fields which are not NULL
  public: BasicState(const Vector3 *_position,
                     const Quaternion *_rotation=NULL,
                     const Vector3 *_scale=NULL):
    position(_position ? *_position : Vector3(0,0,0)),
    rotation(_rotation ? *_rotation : Quaternion(0,0,0,0)),
    scale(_scale ? *_scale : Vector3(0,0,0)),
    posEnabled(_position ? true : false),
    rotEnabled(_rotation ? true : false),
    scaleEnabled(_scale ? true : false) {}
  public: BasicState(const BasicState& o):
    position(o.position),
    rotation(o.rotation),
    scale(o.scale),
    posEnabled(o.posEnabled),
    rotEnabled(o.rotEnabled),
    scaleEnabled(o.scaleEnabled) {}

  public: Vector3 position;
  public: Quaternion rotation;
  public: Vector3 scale;
  public: bool posEnabled, rotEnabled, scaleEnabled;
};


}  // namespace
#endif  // COLLISION_BENCHMARK_BASICTYPES_H
