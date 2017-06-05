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
  // helper struct to allow external boost serialization
  public: struct access;

  public: Vector3(double _x=0, double _y=0, double _z=0):
    x(_x), y(_y), z(_z) {}
  public: Vector3(const Vector3& o):
    x(o.x), y(o.y), z(o.z) {}

  public: friend std::ostream&
          operator<<(std::ostream& _o, const Vector3& _v)
          {
            _o<<"["<<_v.x<<", "<<_v.y<<", "<<_v.z<<"]";
            return _o;
          }
  public: double x,y,z;
};

// simple quaternion data with no functionality,
// just for portability of data
class Quaternion: public Vector3
{
  // helper struct to allow external boost serialization
  public: struct access;
  public: Quaternion(double _x=0, double _y=0, double _z=0, double _w=0):
          Vector3(_x,_y,_z), w(_w) {}
  public: Quaternion(const Quaternion& o):
          Vector3(o), w(o.w) {}

  public: friend std::ostream&
          operator<<(std::ostream& _o, const Quaternion& _q)
          {
            _o<<"["<<_q.x<<", "<<_q.y<<", "<<_q.z<<", "<<_q.w<<"]";
            return _o;
          }
  public: double w;
};

/**
 * \brief Simple state of an object
 * Not all values are mandatory.
 * If disabled, assumes current values are kept.
 * \author Jennifer Buehler
 * \date 2017
 */
class BasicState
{
  // helper struct to allow external boost serialization
  public: struct access;
  // constructor which enables only fields which are not NULL
  public: BasicState(const Vector3 *_position=NULL,
                     const Quaternion *_rotation=NULL,
                     const Vector3 *_scale=NULL):
    position(_position ? *_position : Vector3(0,0,0)),
    rotation(_rotation ? *_rotation : Quaternion(0,0,0,0)),
    scale(_scale ? *_scale : Vector3(1,1,1)),
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

  public: void SetPosition(double x, double y, double z)
          {
            position=Vector3(x,y,z);
            posEnabled=true;
          }
  public: void SetPosition(const Vector3 &_pos)
          {
            SetPosition(_pos.x, _pos.y, _pos.z);
          }
  public: void SetRotation(double x, double y, double z, double w)
          {
            rotation=Quaternion(x,y,z,w);
            rotEnabled=true;
          }
  public: void SetRotation(const Quaternion &_q)
          {
            SetRotation(_q.x, _q.y, _q.z, _q.w);
          }

  public: void SetScale(double x, double y, double z)
          {
            scale=Vector3(x,y,z);
            scaleEnabled=true;
          }
  public: void SetScale(const Vector3 &_scale)
          {
            SetScale(_scale.x, _scale.y, _scale.z);
          }

  public: friend std::ostream&
          operator<<(std::ostream& _o, const BasicState& _s)
          {
            if (_s.posEnabled) _o<<"Position: "<<_s.position<<" ";
            if (_s.rotEnabled) _o<<"Rotation: "<<_s.rotation<<" ";
            if (_s.scaleEnabled) _o<<"Scale: "<<_s.scale;
            return _o;
          }

  public: bool PosEnabled() const { return posEnabled; }
  public: bool RotEnabled() const { return rotEnabled; }
  public: bool ScaleEnabled() const { return scaleEnabled; }

  public: Vector3 position;
  public: Quaternion rotation;
  public: Vector3 scale;
  private: bool posEnabled, rotEnabled, scaleEnabled;
};


}  // namespace
#endif  // COLLISION_BENCHMARK_BASICTYPES_H
