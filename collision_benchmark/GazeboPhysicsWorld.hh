#ifndef COLLISION_BENCHMARK_GAZEBOPHYSICSWORLD
#define COLLISION_BENCHMARK_GAZEBOPHYSICSWORLD

#include <collision_benchmark/PhysicsWorld.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Contact.hh>

#ifndef CONTACTS_ENFORCABLE
//#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#endif

namespace collision_benchmark
{

struct GazeboPhysicsWorldTypes
{
  /// Describes a state of the world
  typedef gazebo::physics::WorldState WorldState;

  /// ID type used to identify models in the world
  typedef std::string ModelID;

  /// ID type to identify individual parts of a model
  typedef std::string ModelPartID;

  /// Math 3D vector implementation
  typedef ignition::math::Vector3d Vector3;

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
 * \brief implementation of a PhysicsEngineWorld for Gazebo.
 * Wraps a gazebo::physics::World object.
 *
 * \author Jennifer Buehler
 * \date November 2016
 */
class GazeboPhysicsWorld:
  public collision_benchmark::PhysicsEngineWorld<GazeboPhysicsWorldTypes,
                                                 GazeboPhysicsEngineWorldTypes>
{
  protected: typedef PhysicsEngineWorld<GazeboPhysicsWorldTypes,
                                        GazeboPhysicsEngineWorldTypes>
                                          ParentClass;

  public: typedef typename ParentClass::ModelID ModelID;
  public: typedef typename ParentClass::ModelPartID ModelPartID;
  public: typedef typename ParentClass::WorldState WorldState;
  public: typedef typename ParentClass::ContactInfo ContactInfo;
  public: typedef typename ParentClass::ContactInfoPtr ContactInfoPtr;
  public: typedef typename ParentClass::Shape Shape;
  public: typedef typename ParentClass::ModelLoadResult ModelLoadResult;


  public: typedef typename ParentClass::Model Model;
  public: typedef typename ParentClass::NativeContact NativeContact;
  public: typedef typename ParentClass::NativeContactPtr NativeContactPtr;
  public: typedef typename ParentClass::PhysicsEngine PhysicsEngine;
  public: typedef typename ParentClass::World World;
  public: typedef typename ParentClass::ModelPtr ModelPtr;
  public: typedef typename ParentClass::ContactPtr ContactPtr;
  public: typedef typename ParentClass::PhysicsEnginePtr PhysicsEnginePtr;
  public: typedef typename ParentClass::WorldPtr WorldPtr;

  // set to true (default) to wait for the namespace for be loaded in
  // the Load* methods. Max wait time can be set in \e OnLoadMaxWaitForNamespace
  // and \e OnLoadMaxWaitForNamespaceSleep
  public: static constexpr bool OnLoadWaitForNamespace = true;
  // if \e OnLoadWaitForNamespace, then this is the maximum
  // time (seconds) to wait for
  public: static constexpr float OnLoadMaxWaitForNamespace = 10;
  // if \e OnLoadWaitForNamespace, sleep time in-between checks to wait for
  // whether the namespace has been loaded
  public: static constexpr float OnLoadWaitForNamespaceSleep = 1;

  // \param enforceContactComputation by default, contacts in Gazebo are only
  //  computed if there is at least one subscriber to the contacts topic.
  //  Use this flag to enforce contacts computation in any case.
  public: GazeboPhysicsWorld(bool enforceContactComputation=false);
  public: GazeboPhysicsWorld(const GazeboPhysicsWorld& w){}
  public: virtual ~GazeboPhysicsWorld();

  public: virtual bool SupportsSDF() const;

  public: virtual OpResult LoadFromSDF(const sdf::ElementPtr& sdf,
                                       const std::string& worldname="");

  public: virtual OpResult LoadFromFile(const std::string& filename,
                                        const std::string& worldname="");

  public: virtual OpResult LoadFromString(const std::string& str,
                                          const std::string& worldname="");

  public: virtual ModelLoadResult
                    AddModelFromFile(const std::string& filename,
                                     const std::string& modelname="");

  public: virtual ModelLoadResult
                    AddModelFromString(const std::string& str,
                                       const std::string& modelname="");

  public: virtual ModelLoadResult
                  AddModelFromSDF(const sdf::ElementPtr& sdf,
                                  const std::string& modelname="");

  public: virtual bool SupportsShapes() const;

  public: virtual ModelLoadResult
                  AddModelFromShape(const std::string& modelname,
                                    const Shape::Ptr& shape,
                                    const Shape::Ptr collShape=Shape::Ptr());

  public: virtual std::vector<ModelID> GetAllModelIDs() const;

  public: virtual bool RemoveModel(const ModelID& id);

  public: virtual bool GetAABB(const ModelID& id,
                               Vector3& min, Vector3& max) const;

  public: virtual void Clear();

  public: virtual WorldState GetWorldState() const;

  public: virtual WorldState GetWorldStateDiff(const WorldState& other) const;

  public: virtual OpResult SetWorldState(const WorldState& state, bool isDiff);

  public: virtual void Update(int steps=1, bool force=false);

  public: virtual void SetPaused(bool flag);

  public: virtual bool IsPaused() const;

  public: virtual std::string GetName() const;

  public: virtual void SetDynamicsEnabled(const bool flag);

  public: virtual bool SupportsContacts() const;

  public: virtual std::vector<ContactInfoPtr> GetContactInfo() const;

  public: virtual std::vector<ContactInfoPtr>
                  GetContactInfo(const ModelID& m1, const ModelID& m2) const;

  /// Current warning for Gazebo implementation: Returned shared pointers
  /// are flakey, they will be deleted as soon as
  /// Gazebo ContactManager deletes them. This will be resolved as soon as
  /// gazebo::physics::Contact instances are managed
  /// as shared pointers by Gazebo as well.
  public: virtual std::vector<NativeContactPtr> GetNativeContacts() const;

  /// Current warning for Gazebo implementation: Returned shared pointers are
  /// flakey, they will be deleted as soon as Gazebo ContactManager deletes
  /// them. This will be resolved as soon as gazebo::physics::Contact
  /// instances are managed as shared poitners by Gazebo as well.
  public: virtual std::vector<NativeContactPtr>
                  GetNativeContacts(const ModelID& m1, const ModelID& m2) const;

  public: virtual bool IsAdaptor() const;

  public: virtual RefResult SetWorld(const WorldPtr& world);

  public: virtual WorldPtr GetWorld() const;

  public: virtual ModelPtr GetModel(const ModelID& model) const;

  public: virtual PhysicsEnginePtr GetPhysicsEngine() const;

  // Set enforcement of contact computation. Gazebo by default only computes
  // contacts when there is a subscriber to the contacts topic.
  // See also constructor parameter.
  public: void SetEnforceContactsComputation(bool flag);

  public: virtual bool SetBasicModelState(const ModelID& id,
                                          const BasicState& state);

  public: virtual bool GetBasicModelState(const ModelID& id,
                                          BasicState& state);

  /// wait for the namespace of this world
  private: bool WaitForNamespace(const gazebo::physics::WorldPtr& gzworld,
                                 float maxWait, float waitSleep);

  // called after a world has been loaded
  private: void PostWorldLoaded();

  private: gazebo::physics::WorldPtr world;
  // by default, contacts in Gazebo are only computed if
  // there is at least one subscriber to the contacts topic.
  // This flag to enforce contact computation.
  private: bool enforceContactComputation;

#ifndef CONTACTS_ENFORCABLE
  /// \brief Callback when a Contact message is received
  /// \param[in] _msg The Contact message
  private: void OnContact(ConstContactsPtr &_msg);
  private: gazebo::transport::NodePtr node;
  private: gazebo::transport::SubscriberPtr contactsSub;
#endif

  // flag whether the world has been paused. Paused in
  // gazebo::World has a slightly different effect because
  // we can still call Step() and Update() etc. while the
  // world is paused. So the paused state has to be managed
  // separately.
  private: bool paused;

};  // class GazeboPhysicsWorld

/// \def GazeboPhysicsWorldPtr
/// \brief Boost shared pointer to a GazeboPhysicsWorld object
typedef std::shared_ptr<GazeboPhysicsWorld> GazeboPhysicsWorldPtr;

}  // namespace

#endif  // COLLISION_BENCHMARK_GAZEBOPHYSICSWORLD
