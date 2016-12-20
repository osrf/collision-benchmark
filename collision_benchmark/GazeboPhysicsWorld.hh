#ifndef COLLISION_BENCHMARK_GAZEBOPHYSICSWORLD
#define COLLISION_BENCHMARK_GAZEBOPHYSICSWORLD

#include <collision_benchmark/PhysicsWorld.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Contact.hh>


namespace collision_benchmark
{

struct GazeboPhysicsWorldTypes
{
  /// ID type used to identify models in the world
  typedef std::string ModelID;
  /// ID type to identify individual parts of a model
  typedef std::string ModelPartID;

  /// Describes a state of the world
  typedef gazebo::physics::WorldState WorldState;

  /// Math 3D vector implementation
  typedef gazebo::math::Vector3 Vector3;

  /// Math wrench implementation
  typedef gazebo::physics::JointWrench Wrench;
};

struct GazeboPhysicsEngineWorldTypes
{
  typedef gazebo::physics::Model Model;
  typedef gazebo::physics::Contact Contact;
  typedef gazebo::physics::PhysicsEngine PhysicsEngine;
  typedef gazebo::physics::World World;
};

/**
 * \brief implementation of a PhysicsEngineWorld for Gazebo. Wraps a gazebo::physics::World object.
 *
 * \author Jennifer Buehler
 * \date November 2016
 */
class GazeboPhysicsWorld: public collision_benchmark::PhysicsEngineWorld<GazeboPhysicsWorldTypes, GazeboPhysicsEngineWorldTypes>
{
  protected: typedef PhysicsEngineWorld<GazeboPhysicsWorldTypes, GazeboPhysicsEngineWorldTypes> ParentClass;

  public: typedef typename ParentClass::ModelID ModelID;
  public: typedef typename ParentClass::ModelPartID ModelPartID;
  public: typedef typename ParentClass::WorldState WorldState;
  public: typedef typename ParentClass::ContactInfo ContactInfo;
  public: typedef typename ParentClass::ContactInfoPtr ContactInfoPtr;
  public: typedef typename ParentClass::Shape Shape;
  public: typedef typename ParentClass::OpResult OpResult;
  public: typedef typename ParentClass::RefResult RefResult;
  public: typedef typename ParentClass::ModelLoadResult ModelLoadResult;


  public: typedef typename ParentClass::Model Model;
  public: typedef typename ParentClass::Contact Contact;
  public: typedef typename ParentClass::PhysicsEngine PhysicsEngine;
  public: typedef typename ParentClass::World World;
  public: typedef typename ParentClass::ModelPtr ModelPtr;
  public: typedef typename ParentClass::ContactPtr ContactPtr;
  public: typedef typename ParentClass::PhysicsEnginePtr PhysicsEnginePtr;
  public: typedef typename ParentClass::WorldPtr WorldPtr;

  public: GazeboPhysicsWorld();
  public: GazeboPhysicsWorld(const GazeboPhysicsWorld& w){}
  public: virtual ~GazeboPhysicsWorld();

  public: virtual bool SupportsSDF() const;

  public: virtual OpResult LoadFromSDF(const sdf::ElementPtr& sdf);

  public: virtual OpResult LoadFromFile(const std::string& filename);

  public: virtual OpResult LoadFromString(const std::string& str);

  public: virtual ModelLoadResult AddModelFromFile(const std::string& filename);

  public: virtual ModelLoadResult AddModelFromString(const std::string& str);

  public: virtual ModelLoadResult AddModelFromSDF(const sdf::ElementPtr& sdf);

  public: virtual bool SupportsShapes() const;

  public: virtual ModelLoadResult AddModelFromShape(const ShapePtr& shape, const ShapePtr * collShape=NULL);

  public: virtual std::vector<ModelID> GetAllModelIDs() const;

  public: virtual bool RemoveModel(const ModelID& id);

  public: virtual void Clear();

  public: virtual WorldState GetWorldState() const;

  public: virtual WorldState GetWorldStateDiff(const WorldState& other) const;

  public: virtual OpResult SetWorldState(const WorldState& state, bool isDiff);

  public: virtual void Update(int steps=1);

  public: virtual bool SupportsContacts() const;

  public: virtual std::vector<ContactInfoPtr> GetContactInfo() const;

  public: virtual std::vector<ContactInfoPtr> GetContactInfo(const ModelID& m1, const ModelID& m2) const;

  public: virtual const std::vector<ContactPtr>& GetContacts() const;

  public: virtual const std::vector<ContactPtr> GetContacts(const ModelID& m1, const ModelID& m2) const;

  public: virtual bool IsAdaptor() const;

  public: virtual RefResult SetWorld(WorldPtr& world);

  public: virtual WorldPtr GetWorld() const;

  public: virtual ModelPtr GetModel(const ModelID& model) const;

  public: virtual PhysicsEnginePtr GetPhysicsEngine() const;

  private: gazebo::physics::WorldPtr world;
};  // class GazeboPhysicsWorld

/// \def GazeboPhysicsWorldPtr
/// \brief Boost shared pointer to a GazeboPhysicsWorld object
typedef std::shared_ptr<GazeboPhysicsWorld> GazeboPhysicsWorldPtr;

}  // namespace

#endif  // COLLISION_BENCHMARK_GAZEBOPHYSICSWORLD
