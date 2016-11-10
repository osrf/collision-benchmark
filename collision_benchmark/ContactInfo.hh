#ifndef COLLISION_BENCHMARK_CONTACTINFO
#define COLLISION_BENCHMARK_CONTACTINFO

#include <vector>

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

   public: Contact(){}
  private: Contact(const Contact& c):
        position(c.position),
        normal(c.normal),
        wrench(c.wrench),
        depth(c.depth) {}
  public: virtual ~Contact();

  public: Vector3 position;
  public: Vector3 normal;
  public: Wrench wrench;
  public: double depth;
};

/**
 * \brief Simple summary of basic contact point information compatible with a variety of physics engines.
 * Can be derived by specific physics engine implementations to supplement information.
 * The purpose of this base class is to allow compare contact points between different physics
 * engines, e.g. all of the Gazebo engines vs. a brute-force contact point calculation.
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

  public: ContactInfo(){}
  private: ContactInfo(const ContactInfo& c):
        contacts(c.contacts),
        model1(c.model1),
        modelPart1(c.modelPart1),
        model2(c.model2),
        modelPart2(c.modelpart2){}
  public: virtual ~ContactInfo();

  public: std::vector<Contact> contacts;

  public: ModelID model1;
  public: ModelPartID modelPart1;
  public: ModelID model2;
  public: ModelPartID modelPart2;

};

}  // namespace

#endif  // COLLISION_BENCHMARK_CONTACTINFO
