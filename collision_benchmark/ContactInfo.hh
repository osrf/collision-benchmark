#ifndef COLLISION_BENCHMARK_CONTACTINFO
#define COLLISION_BENCHMARK_CONTACTINFO

#include <vector>
#include <memory>
#include <iostream>

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

  public: Contact(){}
  public: Contact(const Vector3& position_,
                  const Vector3& normal_,
                  const Wrench& wrench_,
                  const double depth_):
        position(position_),
        normal(normal_),
        wrench(wrench_),
        depth(depth_) {}

  public: Contact(const Contact& c):
        position(c.position),
        normal(c.normal),
        wrench(c.wrench),
        depth(c.depth) {}

  public: virtual ~Contact() {}

  public: friend std::ostream& operator<<(std::ostream& o, const Self& c)
  {
    o << "{Position: " << c.position << ", depth: " << c.depth << "}";
    return o;
  }
  public: Vector3 position;
  public: Vector3 normal;
  public: Wrench wrench;
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

  public: ContactInfo(){}
  /// constructor which automatically swaps model1 and model2
  /// if necessary according to their lexicographicall order
  /// (model1 before model2)
  public: ContactInfo(const ModelID& model1_,
                      const ModelPartID& modelPart1_,
                      const ModelID& model2_,
                      const ModelPartID& modelPart2_)
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
  public: ContactInfo(const ContactInfo& c):
        contacts(c.contacts),
        model1(c.model1),
        modelPart1(c.modelPart1),
        model2(c.model2),
        modelPart2(c.modelpart2){}
  public: virtual ~ContactInfo() {}

  // checks for validity of the contact configuration
  public: bool isValid() const { return model1 < model2; }

  // returns minimum depth amongst all contacts or positive
  // value if contacts are empty (the deepest penetration)
  public: double minDepth() const
  {
    double min=1;
    for (typename std::vector<Contact>::const_iterator
         cit = contacts.begin(); cit != contacts.end(); ++cit)
    {
      const Contact& c = *cit;
      if (c.depth < min) min = c.depth;
    }
    return min;
  }

  public: friend std::ostream& operator<<(std::ostream& o, const Self& c)
  {
    o << "(Model1: "<<c.model1<<"/"<<c.modelPart1<<". Model2: "
      << c.model2<<"/"<<c.modelPart2;
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
