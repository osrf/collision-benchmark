#ifndef COLLISION_BENCHMARK_PHYSICSWORLD
#define COLLISION_BENCHMARK_PHYSICSWORLD
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

#include <collision_benchmark/ContactInfo.hh>
#include <collision_benchmark/Shape.hh>
#include <sdf/sdf.hh>

#include <memory>
// #include <boost/shared_ptr.hpp>

namespace collision_benchmark
{

/**
 * \brief Minimal pure virtual interface for various physics world implementations
 *
 * This interface aims to be minimal in order to support a variety of
 * underlying physics engine world implementations.
 * An "underlying implementation" can be an adaptor to, or a complete implementation
 * of a **world used for physics engines**.
 *
 * If possible, all implementations should derive from the subclass PhysicsEngineWorld.
 * This pure virtual base class only guarantees a minimal common subset of functionality between implementations.
 *
 * Adding and removing of models, lights, or anything part of a world *has to be*
 * supported via the general SetWorldState(). This is to keep things fairly general in-between implementations.
 *
 * \param WorldStateImpl The WorldState can be used to retrieve all sorts of information about the world, including the
 * model states. Most of the functionality offered via this interface is accessible via the
 * world state.
 * There is no specification on what the world state type may be, in this interface they are just needed to define the API.
 *
 * \author Jennifer Buehler
 * \date October 2016
 */
template<class WorldStateImpl>
class PhysicsWorldBase
{
  /// NOT_SUPPORTED: depending on the context of the function, this means
  ///   something about what the method does is not supported (eg. file format not supported)
  public: typedef enum _OpResult {FAILED, NOT_SUPPORTED, SUCCESS} OpResult;

  /// Describes a state of the world
  public: typedef WorldStateImpl WorldState;

  public: PhysicsWorldBase(){}
  public: PhysicsWorldBase(const PhysicsWorldBase& w){}
  public: virtual ~PhysicsWorldBase(){}

  /// Clears the world of all models, lights and anything else that can be
  /// in the world implementation.
  /// A new world can be built with SetWorldState() and/or the Add* functions.
  public: virtual void Clear()=0;

  /// Get the current state of the world
  public: virtual WorldState GetWorldState() const=0;

  /// Gets the difference to the world state in \e other as differential state.
  /// Which means, if the returned state is applied on the current world, it will
  /// be in the state of \e other (this includes adding or removing of models and any
  /// other entities in the underlying world)
  public: virtual WorldState GetWorldStateDiff(const WorldState& other) const=0;

  /// Set the current state of the world. It can be used to *update* the state, like model poses etc.,
  /// and also to *add and remove* models, lights, or whichever entities the underlying world supports.
  /// A \e state can also be a "differential" state (when param \e isDiff is true)
  /// which is applied on the *existing* world, as opposed to resetting the world to the exact state \e state.
  /// \param isDiff if true, this state is a "diff" state which is to be applied/added onto the
  ///     current state of the world. If false, the world is reset to *exactly* this state.
  /// \retval NOT_SUPPORTED this combination of \e state and \e isDiff is not supported, eg.
  ///   \e state as differential state can't be applied to the current world.
  ///   This could happen when trying to add a model which already exists in the current state.
  ///   In general, this value is returned when the error is about the combination of
  ///   \e isDiff and \e state, ie. the parameters not being compatible for whichever reason.
  /// \retval FAILED Failure for other reasons than \e NOT_SUPPORTED
  public: virtual OpResult SetWorldState(const WorldState& state, bool isDiff)=0;

  /// Does \e steps subsequent update calls to the world. **This call blocks**.
  /// \param steps number of iterations to run the world. If 0, runs forever.
  public: virtual void Update(int steps=1)=0;
};



/**
 * \brief Pure virtual extension of PhysicsWorldBase adding functionality to load world and models and contact information.
 *
 * This interface can act as general interface to different physics
 * engines worlds, e.g. all of the Gazebo-integrated engines or any other implementation
 * (eg. "brute force"). It is designed with the aim of comparing results of the engines, e.g. contact points.
 *
 * If possible, all implementations of PhysicsWorld should derive from the subclass PhysicsEngineWorld.
 * This pure virtual interface only guarantees a minimal common subset of functionality between implementations.
 *
 * Adding and removing of models, lights, or anything part of a world *has to be*
 * supported via the general SetWorldState(), as well as via the Add* functions and RemoveModel().
 *
 * \param PhysicsWorldTypes any struct which defines the following typedefs. There is no specification
 * on what these types may be, in this interface they are just needed to define the API.
 * - WorldState: Describes a state of the world
 * - ModelID: ID type used to identify models in the world
 * - ModelPartID: ID type to identify individual parts of a model
 * - Vector3: Math 3D vector implementation
 * - Wrench: Math wrench implementation
 *
 * \author Jennifer Buehler
 * \date October 2016
 */
template<class PhysicsWorldTypes>
class PhysicsWorld: public PhysicsWorldBase<typename PhysicsWorldTypes::WorldState>
{

  /// Describes a state of the world
  public: typedef typename PhysicsWorldTypes::WorldState WorldState;
  protected: typedef PhysicsWorldBase<typename PhysicsWorldTypes::WorldState> BaseClass;

  public: typedef typename BaseClass::OpResult OpResult;

  /// ID type used to identify models in the world
  public: typedef typename PhysicsWorldTypes::ModelID ModelID;
  /// ID type to identify individual parts of a model
  public: typedef typename PhysicsWorldTypes::ModelPartID ModelPartID;

  private: typedef collision_benchmark::Contact<
              typename PhysicsWorldTypes::Vector3,
              typename PhysicsWorldTypes::Wrench> Contact;
  public: typedef collision_benchmark::ContactInfo<Contact, ModelID, ModelPartID> ContactInfo;
  public: typedef std::shared_ptr<ContactInfo> ContactInfoPtr;

  public: typedef collision_benchmark::Shape Shape;


  /// Results returned for loading models
  public: typedef struct _ModelLoadResult
      {
        OpResult opResult;
        /// on success, this will contain the ID given
        /// to the loaded model
        ModelID modelID;
      } ModelLoadResult;


  public: PhysicsWorld(){}
  public: PhysicsWorld(const PhysicsWorld& w){}
  public: virtual ~PhysicsWorld(){}

  /// \return true if SDF related methods are supported
  public: virtual bool SupportsSDF() const = 0;

  /// Like gazebo::physics::World::Load(), loads from SDF
  /// Some implementations may not support directly reading from SDF,
  /// in which case an exception is thrown (see also SupportsSDF()).
  /// \retval NOT_SUPPORTED the type of model specified in the SDF is not supported
  /// \retval FAILED Loading failed for any other reason
  public: virtual OpResult LoadFromSDF(const sdf::ElementPtr& sdf)=0;

  /// Loads a world from a file. The format of the file has to be
  /// supported by the implementation.
  /// \retval NOT_SUPPORTED the file type, or the world specified within is not supported
  /// \retval FAILED Loading failed for any other reason
  public: virtual OpResult LoadFromFile(const std::string& filename)=0;

  /// Loads a world from a string \e str. The format of the file has to be
  /// supported by the implementation.
  /// \retval NOT_SUPPORTED the format of the string, or the world specified within is not supported
  /// \retval FAILED Loading failed for any other reason
  public: virtual OpResult LoadFromString(const std::string& str)=0;

  /// Loads a model from a file and adds it to the world
  /// To subsequently set the pose of the model, use SetWorldState(),
  /// or specific methods of the subclass implementation.
  /// \retval NOT_SUPPORTED the file type, or the model specified within is not supported
  /// \retval FAILED Loading failed for any other reason
  public: virtual ModelLoadResult AddModelFromFile(const std::string& filename)=0;

  /// Loads a model from a string and adds it to the world
  /// To subsequently set the pose of the model, use SetWorldState(),
  /// or specific methods of the subclass implementation.
  /// \retval NOT_SUPPORTED the format of the string, or the model specified within is not supported
  /// \retval FAILED Loading failed for any other reason
  public: virtual ModelLoadResult AddModelFromString(const std::string& str)=0;

  /// Loads a model from a SDF specification and adds it to the world.
  /// Some implementations may not support directly reading from SDF,
  /// in which case an exception is thrown (see also SupportsSDF()).
  /// To subsequently set the pose of the model, use SetWorldState(),
  /// or specific methods of the subclass implementation.
  public: virtual ModelLoadResult AddModelFromSDF(const sdf::ElementPtr& sdf)=0;

  /// \return true if AddModelFromShape() is supported
  public: virtual bool SupportsShapes() const = 0;

  /// Adds this shape to the world and converts it to whichever representation
  /// is required in the implementation. The shape will become a model which can be identified
  /// with \e ModelID as well.
  /// \param shape the shape to be used for visualization (if underlying implementation supports
  ///   separate visualization shapes), and unless \e collShape is specified, this shape
  ///   will be used for collisions as well.
  /// \param collShape optionally, a representation of \e shape to use for collision computation
  /// \return in case the underlying implementation does not support shapes, this throws and
  ///   exception (see also SupportsShapes()). Instead, the AddModel*() methods have to be used.
  public: virtual ModelLoadResult AddModelFromShape(const ShapePtr& shape, const ShapePtr * collShape=NULL)=0;

  public: virtual std::vector<ModelID> GetAllModelIDs() const=0;

  /// removes a model from the world
  /// \retval false the model was not in the world
  public: virtual bool RemoveModel(const ModelID& id)=0;

  /// \return false if the underlying implementation does not compute contact points, true otherwise.
  public: virtual bool SupportsContacts() const=0;

  /// In the current state of the world, get all contact points between models.
  /// The returned vector will be empty if no models collide.
  /// This method may be less efficient than methods specific to the physics engine, because it requires
  /// constructiong the shared data structure ContactInfo. Use this method only to compare
  /// different implementations, and stick to the engine-specific ones in subclasses otherwise.
  ///
  /// Throws an exception if the underlying implementation does not support calculation
  /// of contact points (SupportContacts() returns false).
  public: virtual std::vector<ContactInfoPtr> GetContactInfo() const=0;

  /// Works as GetContactInfo() but only returns the contact points between models \e m1 and \e m2.
  public: virtual std::vector<ContactInfoPtr> GetContactInfo(const ModelID& m1, const ModelID& m2) const=0;
};


/**
 * \brief A more engine-specific pure virtual interface of PhysicsWorld which adds a broader access to physics engine functionality
 *
 * If applicable, all implementations of PhysicsWorld should derive from this class, while the base
 * class(es) only guarantees a minimal common subset of functionality between implementations.
 *
 * \param PhysicsWorldTypes see PhysicsWorld
 * \param PhysicsEngineWorldTypes has to be a struct with the following typedefs:
 * - Model: The model class type
 * - Contact: Class for engine-specific implementation of a contact point
 * - PhysicsEngine: Class for the physics engine, if there is any (set to void* if there is none)
 * - World: Class type of the world, if there is any (set to void* if there is none).
 *
 * \author Jennifer Buehler
 * \date October 2016
 */
template<class PhysicsWorldTypes, class PhysicsEngineWorldTypes>
class PhysicsEngineWorld: public PhysicsWorld<PhysicsWorldTypes>
{
  protected: typedef PhysicsWorld<PhysicsWorldTypes> ParentClass;
  public: typedef typename ParentClass::ModelID ModelID;
  public: typedef typename ParentClass::ModelPartID ModelPartID;
  public: typedef typename ParentClass::WorldState WorldState;
  public: typedef typename ParentClass::ContactInfoPtr ContactInfoPtr;

  public: typedef typename PhysicsEngineWorldTypes::Model Model;
  public: typedef typename PhysicsEngineWorldTypes::Contact Contact;
  public: typedef typename PhysicsEngineWorldTypes::PhysicsEngine PhysicsEngine;
  public: typedef typename PhysicsEngineWorldTypes::World World;

  public: typedef std::shared_ptr<Model> ModelPtr;
  public: typedef std::shared_ptr<Contact> ContactPtr;
  public: typedef std::shared_ptr<PhysicsEngine> PhysicsEnginePtr;
  public: typedef std::shared_ptr<World> WorldPtr;

  public: typedef enum _RefResult {FAILED, NOT_SUPPORTED, COPIED, REFERENCED} RefResult;

  public: PhysicsEngineWorld(){}
  public: PhysicsEngineWorld(const PhysicsEngineWorld& w){}
  public: virtual ~PhysicsEngineWorld(){}

  /// In the current state of the world, get all contact points between models.
  /// The returned vector will be empty if no models collide.
  /// This method is more engine-specific than GetContactInfo(), and therefore
  /// more specific to the engine used. However for that reason this function is not
  /// suitable to compare different contact point implementations.
  ///
  /// Throws an exception if the underlying implementation does not support calculation
  /// of contact points (SupportContacts() returns false).
  public: virtual const std::vector<ContactPtr>& GetContacts() const=0;

  /// Works as GetContact() but only returns the contact points between models \e m1 and \e m2.
  public: virtual const std::vector<ContactPtr> GetContacts(const ModelID& m1, const ModelID& m2) const=0;

  /// \retval true if this is an adaptor to another world (either of type
  ///   \e PhysicsWorld or of \e WorldPtr). This means \e GetWorld() will
  ///   not return a reference to this instance.
  /// \retval false if this type is a self-contained implementation of a world.
  public: virtual bool IsAdaptor() const = 0;

  /// Set the specific underlying world. This will clear out
  /// any possibly already loaded world.
  /// If IsAdaptor() returns false,
  /// the whole state of \e world will be *copied* to this world.
  /// Otherwise, the given pointer will be used as adapted world.
  /// \retval NOT_SUPPORTED this implementation does not support setting the
  ///   state directly from another world. Try SetWorldState instead.
  /// \retval COPIED if the state of \e world was copied.
  ///   In this case, the result of GetWorld() will NOT return a pointer to \e world.
  /// \retval REFERENCED if \e world was taken as local reference to the world.
  ///   In this case, the result of GetWorld() will return a pointer to \e world.
  /// \retval FAILED Error copying the state of \e world.
  public: virtual RefResult SetWorld(WorldPtr& world) = 0;

  /// Returns the underlying world, or a pointer to this instance if this is
  /// a self-contained implementation (not an adaptor to another world).
  /// This method only makes sense to use if it is known that this implementation
  /// is an adaptor (IsAdaptor() returns true), *and*
  /// the specific type is known (eg. to call specific functions on it).
  public: virtual WorldPtr GetWorld() const = 0;

  public: virtual ModelPtr GetModel(const ModelID& model) const=0;

  /// Get underlying physics engine to use for more specific tests on the
  /// current state of the world. Returns NULL pointer type if there
  /// is no underlying physics engine for this implementation.
  public: virtual PhysicsEnginePtr GetPhysicsEngine() const=0;

};  // class PhysicsEngineWorld

}  // namespace

#endif  // COLLISION_BENCHMARK_PHYSICSWORLD
