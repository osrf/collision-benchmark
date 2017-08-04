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
 * Date: October 2016
 */

#ifndef COLLISION_BENCHMARK_CONTACTINFO
#define COLLISION_BENCHMARK_CONTACTINFO

#include <vector>
#include <memory>
#include <iostream>
#include <limits>

namespace collision_benchmark
{
/**
 * \brief Basic data for a contact point between two bodies
 * \author Jennifer Buehler
 * \date October 2016
 */
template<class Vector3Impl, class WrenchImpl>
class Contact
{
  public: typedef Vector3Impl Vector3;
  public: typedef WrenchImpl Wrench;
  private: typedef Contact<Vector3, Wrench> Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  public: Contact() {}
  public: Contact(const Vector3& position_,
                  const Vector3& normal_,
                  const Wrench &wrench_,
                  const double depth_)
        : position(position_),
          normal(normal_),
          wrench(wrench_),
          depth(depth_) {}

  public: Contact(const Contact &c)
        : position(c.position),
          normal(c.normal),
          wrench(c.wrench),
          depth(c.depth) {}

  public: virtual ~Contact() {}

  public: friend std::ostream &operator << (std::ostream &o, const Self &c)
  {
    o << "{";
    // o <<"Position: " << c.position << " ";
    // o <<"normal: " << c.normal << " ";
    o <<"force: b1=" << c.wrench.body1Force
      << " b2=" << c.wrench.body2Force << " ";
    o <<"torque: b1=" << c.wrench.body1Torque
      << " b2=" << c.wrench.body2Torque << " ";
    o <<"depth: " << c.depth;
    o << "}";
    return o;
  }
  public: Vector3 position;
  public: Vector3 normal;
  public: Wrench wrench;
  // penetration depth: depth to which the two bodies inter-penetrate each
  // other. If zero, tho bodies "just touch". However due to inaccuracies
  // this will rarely be exactly zero.
  // If negative, the bodies don't touch and the contact should be considered
  // invalid.
  public: double depth;
};

/**
 * \brief Simple summary of basic contact point information
 * compatible with a variety of physics engines.
 * Can be derived by specific physics engine implementations to
 * supplement information.
 * The purpose of this base class is to allow compare contact points between
 * different physics engines, e.g. all of the Gazebo engines vs. another
 * contact point calculation implementation.
 *
 * Each contact between two bodies may consist of several contact points.
 *
 * Template parameters:
 * - ContactImpl any instantiated type of collision_benchmark::Contact
 * - ModelIdImpl ID type used to identify models in the world
 * - ModelPartIdImpl ID type to identify individual parts of a model, e.g. links
 *
 * \author Jennifer Buehler
 * \date October 2016
 */
template<class ContactImpl, typename ModelIdImpl, typename ModelPartIdImpl>
class ContactInfo
{
  public: typedef ContactImpl Contact;
  public: typedef typename Contact::Vector3 Vector3;
  public: typedef ModelIdImpl ModelID;
  public: typedef ModelPartIdImpl ModelPartID;

  private: typedef ContactInfo<Contact, ModelID, ModelPartID> Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  public: ContactInfo() {}
  /// constructor which automatically swaps model1 and model2
  /// if necessary according to their lexicographicall order
  /// (model1 before model2)
  public: ContactInfo(const ModelID &model1_,
                      const ModelPartID &modelPart1_,
                      const ModelID &model2_,
                      const ModelPartID &modelPart2_)
  {
    if (model1_ < model2_)
    {
      model1 =  model1_;
      modelPart1 =  modelPart1_;
      model2 =  model2_;
      modelPart2 =  modelPart2_;
    }
    else
    {
      model1 =  model2_;
      modelPart1 =  modelPart2_;
      model2 =  model1_;
      modelPart2 =  modelPart1_;
    }
  }
  public: ContactInfo(const ContactInfo &c)
        : contacts(c.contacts),
          model1(c.model1),
          modelPart1(c.modelPart1),
          model2(c.model2),
          modelPart2(c.modelpart2) {}
  public: virtual ~ContactInfo() {}

  // checks for validity of the contact configuration
  public: bool isValid() const { return model1 < model2; }

  // returns minimum depth amongst all contacts in \e min.
  // \return false if contacts are empty
  public: bool minDepth(double &min) const
  {
    if (contacts.empty()) return false;
    min = std::numeric_limits<double>::max();
    for (typename std::vector<Contact>::const_iterator
         cit = contacts.begin(); cit != contacts.end(); ++cit)
    {
      const Contact &c = *cit;
      if (c.depth < min) min = c.depth;
    }
    return true;
  }
  // returns maximum depth amongst all contacts in \e max.
  // \return false if contacts are empty
  public: bool maxDepth(double &max) const
  {
    if (contacts.empty()) return false;
    max = std::numeric_limits<double>::min();
    for (typename std::vector<Contact>::const_iterator
         cit = contacts.begin(); cit != contacts.end(); ++cit)
    {
      const Contact &c = *cit;
      if (c.depth > max) max = c.depth;
    }
    return true;
  }

  public: friend std::ostream &operator << (std::ostream &o, const Self &c)
  {
    o << "(Model1: " << c.model1<<"/" << c.modelPart1<<". Model2: "
      << c.model2<<"/" << c.modelPart2;
    o << "; Contacts: ";
    for (typename std::vector<Contact>::const_iterator it = c.contacts.begin();
         it != c.contacts.end(); ++it)
    {
      if (it != c.contacts.begin()) o << ", ";
      o << *it;
    }
    o << ")";
    return o;
  }

  // all contacts which happen between the models.
  public: std::vector<Contact> contacts;

  // first model which is part of the contact.
  // Is always lexicographically 'smaller' than model2.
  public: ModelID model1;
  public: ModelPartID modelPart1;
  // second model which is part of the contact
  public: ModelID model2;
  public: ModelPartID modelPart2;
};
}  // namespace

#endif  // COLLISION_BENCHMARK_CONTACTINFO
