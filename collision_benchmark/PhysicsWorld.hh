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
#include <collision_benchmark/BasicTypes.hh>
#include <sdf/sdf.hh>

#include <memory>

namespace collision_benchmark
{

/// Type to indicate success or failure of an operation
/// NOT_SUPPORTED: depending on the context of the function, this means
///   something about what the method does is not supported
///   (eg. file format not supported)
typedef enum _OpResult {FAILED, NOT_SUPPORTED, SUCCESS} OpResult;

// Type to indicate the result of setting a field using an object.
// CLONED: the object was deep copied
// SHALLOW_COPIED: the object was shallow copied
// REFERENCED: the object was referenced
typedef enum _RefResult {ERROR, CLONED, SHALLOW_COPIED, REFERENCED} RefResult;

/**
 * \brief Minimal pure virtual interface for physics world implementations
 *
 * This interface aims to be minimal in order to support a variety of
 * underlying physics engine world implementations.
 * An "underlying implementation" can be an adaptor to, or a complete
 * implementation of a **world used for physics engines**.
 *
 * This pure virtual base class only guarantees a minimal common subset of
 * functionality between implementations.
 * If possible, all implementations should agree on a common basic interface,
 * such as PhysicsEngineWorld.
 *
 * \author Jennifer Buehler
 * \date October 2016
 */
class PhysicsWorldBaseInterface
{

  private: typedef PhysicsWorldBaseInterface Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  public: PhysicsWorldBaseInterface(){}
  public: virtual ~PhysicsWorldBaseInterface(){}

  /// Clears the world of all models, lights and anything else that can be
  /// in the world implementation.
  /// A new world can be built with SetWorldState() and/or the Add* functions.
  public: virtual void Clear() = 0;

  /// Does \e steps subsequent update calls to the world. **This call blocks**.
  /// \param steps number of iterations to run the world. If 0, runs forever.
  /// \param force if set to true, the update is forced even if the
  ///   world is paused. This is like temporarily unpausing the world,
  ///   but it will only have an effect for this calling thread
  ///   (if other threads try to call this with force set to false
  ///   the world will not update for the call from the other thread).
  public: virtual void Update(int steps=1, bool force=false) = 0;

  /// Pauses or "freezes" the world simulation in the current state.
  /// If the world is paused, any calls of Update() will have no effect.
  public: virtual void SetPaused(bool flag) = 0;

  /// returns whether the world is paused.
  public: virtual bool IsPaused() const = 0;

  /// Returns the name of the world
  public: virtual std::string GetName() const = 0;

  /// \return true if SDF is supported to load worlds and models.
  public: virtual bool SupportsSDF() const = 0;

  /// Loads a world from a SDF element.
  /// Some implementations may not support directly reading from SDF,
  /// in which case an exception is thrown (see also SupportsSDF()).
  /// \param worldname set to non-empty string to override world
  ///        name given in SDF
  /// \retval NOT_SUPPORTED the type of model specified in the SDF
  ///         is not supported
  /// \retval FAILED Loading failed for any other reason
  public: virtual OpResult LoadFromSDF(const sdf::ElementPtr& sdf,
                                       const std::string& worldname="") = 0;

  /// Loads a world from a file. The format of the file has to be
  /// supported by the implementation.
  /// \param worldname set to non-empty string to override world
  ///        name given in the file
  /// \retval NOT_SUPPORTED the file type, or the world specified
  ///         within is not supported
  /// \retval FAILED Loading failed for any other reason
  public: virtual OpResult LoadFromFile(const std::string& filename,
                                        const std::string& worldname="") = 0;

  /// Loads a world from a string \e str. The format of the string has to be
  /// supported by the implementation.
  /// \param worldname set to non-empty string to override world name
  ///        given in the string
  /// \retval NOT_SUPPORTED the format of the string, or the world specified
  ///         within is not supported
  /// \retval FAILED Loading failed for any other reason
  public: virtual OpResult LoadFromString(const std::string& str,
                                          const std::string& worldname="") = 0;

  /// Saves the world to the file.
  /// \param[in] filename the filename (absolute path).
  /// \param[in] resourceDir If not empty, relevant
  ///   resource files referenced by the world are copied to
  ///   ``resourceDir/resourceSubdir``, and the reference in the written file
  ///   \e filename to the resource files is changed to the new file location.
  ///   The purpose of \e resourceSubdir is that if the file format references
  ///   files with a relative path, this is the part of the full path
  ///   ``resourceDir/resourceSubdir`` that the resource file will be
  ///   referenced with, meaning that ``resourceDir`` is the part of the path
  ///   which is specific to the hard drive used at the time of writing to file.
  ///   When copying \e resourceSubdir to a different location on a harddrive,
  ///   the references would then still be valid relative to the new base
  ///   directory. If this path is empty, it has no effect, so resource files
  ///   won't be copied then.
  /// \param[in] resourceSubdir see description of \e resourceDir. If empty,
  ///   has no effect.
  /// \return success or not
  public: virtual bool SaveToFile(const std::string& filename,
                                  const std::string& resourceDir = "",
                                  const std::string& resourceSubdir = "") = 0;

  /// Set the dynamics engine to enabledl or disabled. If disabled, the objects
  /// won't react to physics laws, but objects can be maintained in the world
  /// and collision states / contact points between them checked.
  public: virtual void SetDynamicsEnabled(const bool flag) = 0;
};

/**
 * \brief Extension to the PhysicsWorldBaseInterface
 * offering functionality relating to a world state.
 *
 * Adding and removing of models, lights, or anything part of a world
 * *has to be* supported via the general SetWorldState().
 *
 * \param WorldStateImpl The WorldState can be used to retrieve all sorts
 *        of information about the world, including the
 *        model states. Most of the functionality offered via this interface
 *        is accessible via the world state. There is no specification on
 *        what the world state type may be, in this interface they
 *        are just needed to define the API.
 *
 * \author Jennifer Buehler
 * \date February 2016
 */
template<class WorldStateImpl>
class PhysicsWorldStateInterface
{
  private: typedef PhysicsWorldStateInterface<WorldStateImpl> Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  /// Describes a state of the world
  public: typedef WorldStateImpl WorldState;

  public: PhysicsWorldStateInterface(){}
  public: virtual ~PhysicsWorldStateInterface(){}

  /// Get the current state of the world
  public: virtual WorldState GetWorldState() const = 0;

  /// Gets the difference to the world state in \e other as differential state.
  /// Which means, if the returned state is applied on the current world,
  /// it will be in the state of \e other (this includes adding or removing
  /// of models and any other entities in the underlying world)
  public: virtual WorldState
                  GetWorldStateDiff(const WorldState& other) const = 0;

  /// Set the current state of the world. It can be used to *update* the state,
  /// like model poses etc., and also to *add and remove* models, lights,
  /// or whichever entities the underlying world supports.
  /// A \e state can also be a "differential" state (when param \e isDiff
  /// is true) which is applied on the *existing* world, as opposed to
  /// resetting the world to the exact state \e state.
  /// \param isDiff if true, this state is a "diff" state which is to be
  ///        applied/added onto the current state of the world. If false,
  ///        the world is reset to *exactly* this state.
  /// \retval NOT_SUPPORTED this combination of \e state and \e isDiff is
  ///         not supported, eg. \e state as differential state can't be
  ///         applied to the current world.
  ///         This could happen when trying to add a model which already
  ///         exists in the current state. In general, this value is returned
  ///         when the error is about the combination of
  ///         \e isDiff and \e state, ie. the parameters not being compatible
  ///         for whichever reason.
  /// \retval FAILED Failure for other reasons than \e NOT_SUPPORTED
  public: virtual OpResult SetWorldState(const WorldState& state,
                                         bool isDiff=false) = 0;
};

/**
 * \brief Extension to PhysicsWorldBaseInterface
 * adding functionality to load and access models and shapes.
 *
 * Template parameters: There is no specification on what these types may be,
 * in this interface they are just needed to define the API.
 *
 * \param ModelID_ ID type used to identify models in the world
 * \param ModelPartID_ ID type to identify individual parts of a model
 * \param Vector3_ Math 3D vector implementation
 *
 * \author Jennifer Buehler
 * \date October 2016
 */
template<class ModelID_, class ModelPartID_, class Vector3_>
class PhysicsWorldModelInterface
{
  public: typedef ModelID_ ModelID;
  public: typedef ModelPartID_ ModelPartID;
  public: typedef Vector3_ Vector3;

  private: typedef PhysicsWorldModelInterface<ModelID, ModelPartID,
                                              Vector3> Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;
  public: typedef collision_benchmark::Shape Shape;

  /// Results returned for loading models
  public: typedef struct _ModelLoadResult
      {
        OpResult opResult;
        /// on success, this will contain the ID given
        /// to the loaded model
        ModelID modelID;
      } ModelLoadResult;


  public: PhysicsWorldModelInterface(){}
  public: virtual ~PhysicsWorldModelInterface(){}

  /// Loads a model from a file and adds it to the world.
  /// Doesn't set the model pose.
  /// \param modelname set to non-empty string to override world
  ///        name given in file
  /// \retval NOT_SUPPORTED the file type, or the model specified
  ///         within is not supported
  /// \retval FAILED Loading failed for any other reason
  public: virtual ModelLoadResult
                  AddModelFromFile(const std::string& filename,
                                   const std::string& modelname="") = 0;

  /// Loads a model from a string and adds it to the world.
  /// The format of the string must be supported by the implementation.
  /// To subsequently set the pose of the model, use SetWorldState(),
  /// or specific methods of the subclass implementation.
  /// \param modelname set to non-empty string to override world name given
  ///        in the string \e str
  /// \retval NOT_SUPPORTED the format of the string \e str, or the model
  ///         specified within is not supported
  /// \retval FAILED Loading failed for any other reason
  public: virtual ModelLoadResult
                  AddModelFromString(const std::string& str,
                                     const std::string& modelname="") = 0;

  /// Loads a model from a SDF specification and adds it to the world.
  /// Some implementations may not support directly reading from SDF,
  /// in which case an exception is thrown (see also SupportsSDF()).
  /// To subsequently set the pose of the model, use SetWorldState(),
  /// or specific methods of the subclass implementation.
  /// \param modelname set to non-empty string to override world
  ///        name given in SDF
  public: virtual ModelLoadResult
                  AddModelFromSDF(const sdf::ElementPtr& sdf,
                                  const std::string& modelname="") = 0;

  /// \return true if AddModelFromShape() is supported
  public: virtual bool SupportsShapes() const = 0;

  /// Adds this shape to the world and converts it to whichever representation
  /// is required in the implementation. The shape will become a model which
  /// can be identified with \e ModelID as well.
  /// \param modelname name to give to the model
  /// \param shape the shape to be used for visualization
  ///   (if underlying implementation supports
  ///   separate visualization shapes), and unless \e collShape is specified,
  ///   this shape will be used for collisions as well.
  /// \param collShape optionally, a representation of \e shape to use for
  ///       collision computation. If not given, no collision shape is added.
  /// \return in case the underlying implementation does not support shapes,
  ///         this throws an exception (see also SupportsShapes()).
  ///         Instead, the AddModel*() methods have to be used.
  public: virtual ModelLoadResult
                  AddModelFromShape(const std::string& modelname,
                                    const Shape::Ptr& shape,
                                    const Shape::Ptr& collShape
                                      = Shape::Ptr()) = 0;

  public: virtual std::vector<ModelID> GetAllModelIDs() const = 0;

  // If the underlying implementation offers integer IDs for models
  // in addition to the ModelID type, this function will return the
  // integer ID. Usually, negative numbers mean that the implementation
  // does not use integer IDs, or it would mean that the model
  // with this \e id was not found.
  public: virtual int GetIntegerModelID(const ModelID& id) const = 0;

  /// Removes a model from the world
  /// \retval false the model was not in the world
  public: virtual bool RemoveModel(const ModelID& id) = 0;

  /// Sets the pose and scale of a model.
  /// \retval false the model was not in the world
  public: virtual bool SetBasicModelState(const ModelID& id,
                                          const BasicState& state) = 0;

  /// Gets the pose and scale of a model.
  /// \retval false the model was not in the world
  public: virtual bool GetBasicModelState(const ModelID& id,
                                          BasicState& state) = 0;

  /// \brief Get axis aligned bounding box of the model.
  /// \param[out] inLocalFrame true if the AABB is expressed in the
  ///   local coordinate system of the model, false if it's in global.
  /// \retval false the model was not in the world
  public: virtual bool GetAABB(const ModelID& id,
                               Vector3& min, Vector3& max,
                               bool& inLocalFrame) const = 0;
};

/**
 * \brief Extension to PhysicsWorldBaseInterface
 * adding functionality related to contact points.
 *
 * Template parameters: There is no specification on what these types may be,
 * in this interface they are just needed to define the API.
 *
 * \param ModelID_ ID type used to identify models in the world
 * \param ModelPartID_ ID type to identify individual parts of a model
 * \param Vector3_ Math 3D vector implementation
 * \param Wrench_ Math wrench implementation
 *
 * \author Jennifer Buehler
 * \date October 2016
 */
template<class ModelID_, class ModelPartID_, class Vector3_, class Wrench_>
class PhysicsWorldContactInterface
{
  public: typedef ModelID_ ModelID;
  public: typedef ModelPartID_ ModelPartID;
  public: typedef Vector3_ Vector3;
  public: typedef Wrench_ Wrench;

  private: typedef PhysicsWorldContactInterface<ModelID, ModelPartID,
                                                Vector3, Wrench> Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  public: typedef collision_benchmark::Contact<Vector3,Wrench> Contact;
  public: typedef typename Contact::Ptr ContactPtr;

  public: typedef collision_benchmark::ContactInfo<Contact, ModelID,
                                                   ModelPartID> ContactInfo;
  public: typedef typename ContactInfo::Ptr ContactInfoPtr;

  public: PhysicsWorldContactInterface(){}
  public: virtual ~PhysicsWorldContactInterface(){}

  /// \return false if the underlying implementation does not
  ///         compute contact points, true otherwise.
  public: virtual bool SupportsContacts() const = 0;

  /// In the current state of the world, get all contact points between models.
  /// The returned vector will be empty if no models collide.
  ///
  /// Throws an exception if the underlying implementation does not
  /// support calculation of contact points (SupportsContacts() returns false).
  public: virtual std::vector<ContactInfoPtr> GetContactInfo() const = 0;

  /// Works as GetContactInfo() but only returns the contact points between
  /// models \e m1 and \e m2.
  public: virtual std::vector<ContactInfoPtr>
                  GetContactInfo(const ModelID& m1,
                                 const ModelID& m2) const = 0;
};

/**
 * \brief Extension of PhysicsWorldBaseInterface
 * which provides more engine-specific functionality.
 *
 * \param ModelID_ ID to use for identifying a model
 * \param Model_ The model class type
 * \param NativeContact_ Class for engine-specific implementation of a
 *        contact point
 * \param PhysicsEngine_ Class for the physics engine, if there is any
 *        (set to void* if there is none)
 * \param World_ Class type of the world, if there is any (set to void*
 *        if there is none).
 *
 * \author Jennifer Buehler
 * \date October 2016
 */
template<class ModelID_, class Model_, class NativeContact_,
         class PhysicsEngine_, class World_>
class PhysicsEngineWorldInterface
{
  public: typedef ModelID_ ModelID;
  public: typedef Model_ Model;
  public: typedef NativeContact_ NativeContact;
  public: typedef PhysicsEngine_ PhysicsEngine;
  public: typedef World_ World;

  private: typedef PhysicsEngineWorldInterface<
              ModelID, Model, NativeContact,
              PhysicsEngine, World> Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  public: typedef std::shared_ptr<Model> ModelPtr;
  public: typedef std::shared_ptr<NativeContact> NativeContactPtr;
  public: typedef std::shared_ptr<PhysicsEngine> PhysicsEnginePtr;
  public: typedef std::shared_ptr<World> WorldPtr;

  public: PhysicsEngineWorldInterface(){}
  public: virtual ~PhysicsEngineWorldInterface(){}

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
  /// \retval COPIED if the state of \e world was copied.
  ///   In this case, the result of GetWorld() will NOT return a
  ///   pointer to \e world.
  /// \retval REFERENCED if \e world was taken as local reference to the world.
  ///   In this case, the result of GetWorld() will return a pointer
  ///   to \e world.
  /// \retval ERROR Error copying the state of \e world.
  public: virtual RefResult SetWorld(const WorldPtr& world) = 0;

  /// Returns the underlying world, or a pointer to this instance if this is
  /// a self-contained implementation (not an adaptor to another world).
  /// This method only makes sense to use if it is known that this
  /// implementation is an adaptor (IsAdaptor() returns true), *and*
  /// the specific type is known (eg. to call specific functions on it).
  public: virtual WorldPtr GetWorld() const = 0;

  public: virtual ModelPtr GetModel(const ModelID& model) const = 0;

  /// Get underlying physics engine to use for more specific tests on the
  /// current state of the world. Returns NULL pointer type if there
  /// is no underlying physics engine for this implementation.
  public: virtual PhysicsEnginePtr GetPhysicsEngine() const = 0;

  /// In the current state of the world, get all contact points between models.
  /// The returned vector will be empty if no models collide.
  /// This method is more engine-specific than GetContactInfo(), and therefore
  /// more specific to the engine used. However for that reason this
  /// function is not suitable to compare different contact point
  /// implementations.
  /// It is more efficient than GetContactInfo() though because there is no
  /// need to copy information over to the ContactInfo struct.
  ///
  /// Throws an exception if the underlying implementation does not support
  /// calculation of contact points (SupportsContacts() returns false).
  public: virtual std::vector<NativeContactPtr> GetNativeContacts() const = 0;

  /// Works as GetNativeContact() but only returns the contact points
  /// between models \e m1 and \e m2.
  public: virtual std::vector<NativeContactPtr>
                  GetNativeContacts(const ModelID& m1,
                                    const ModelID& m2) const = 0;

};  // class PhysicsEngineWorld


/**
 * \brief Common interface for all physics worlds
 * combining basic, model and contact points functionality.
 *
 * If possible, all implementations of PhysicsWorld should derive from
 * the subclass PhysicsEngineWorld.
 * This pure virtual interface only guarantees a minimal common subset of
 * functionality between implementations.
 *
 * There is no specification on what the template types may be, in this
 * interface they are just needed to define the API.
 *
 * \param _WorldState describes the state of a world.
 * \param _ModelID the identifier for a specific model
 * \param _ModelPartID the identifier for a part of a model
 * \param _Vector3 Math 3D vector implementation
 * \param _Wrench Math wrench implementation
 *
 * \author Jennifer Buehler
 * \date October 2016
 */
template<class _WorldState, class _ModelID,
         class _ModelPartID, class _Vector3, class _Wrench>
class PhysicsWorld:
  public PhysicsWorldBaseInterface,
  public PhysicsWorldStateInterface<_WorldState>,
  public PhysicsWorldModelInterface<_ModelID, _ModelPartID, _Vector3>,
  public PhysicsWorldContactInterface<_ModelID, _ModelPartID,
                                      _Vector3, _Wrench>
{
  public: typedef _WorldState WorldState;
  public: typedef _ModelID ModelID;
  public: typedef _ModelPartID ModelPartID;
  public: typedef _Vector3 Vector3;
  public: typedef _Wrench Wrench;

  private: typedef PhysicsWorld<WorldState, ModelID, ModelPartID,
                                Vector3, Wrench> Self;

  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  private: typedef PhysicsWorldBaseInterface PhysicsWorldBaseParent;
  private: typedef PhysicsWorldStateInterface<WorldState>
                PhysicsWorldStateParent;
  private: typedef PhysicsWorldModelInterface<ModelID, ModelPartID,
              Vector3> PhysicsWorldModelParent;
  private: typedef PhysicsWorldContactInterface<ModelID, ModelPartID,
              Vector3, Wrench> PhysicsWorldContactParent;

  public: typedef typename PhysicsWorldModelParent::Shape Shape;

  public: typedef typename PhysicsWorldContactParent::Contact Contact;
  public: typedef typename Contact::Ptr ContactPtr;

  public: typedef typename PhysicsWorldContactParent::ContactInfo ContactInfo;
  public: typedef typename ContactInfo::Ptr ContactInfoPtr;
};


/**
 * \brief Specialization of PhysicsWorld
 * which additionally implements PhysicsEngineWorldInterface
 *
 * All implementations of PhysicsWorld should derive from this class,
 * while the base class(es) only guarantees a minimal common subset of
 * functionality between implementations.
 *
 * \param PhysicsWorldTypes_ has to be a struct with the
 *        following typedefs:
 *        - WorldState describes the state of a world.
 *        - ModelID the identifier for a specific model
 *        - ModelPartID the identifier for a part of a model
 *        - Vector3 Math 3D vector implementation
 *        - Wrench Math wrench implementation
 * \param PhysicsEngineWorldTypes_ has to be a struct with the
 *        following typedefs:
 *        - Model: The model class type
 *        - Contact: Class for engine-specific implementation of a contact point
 *        - PhysicsEngine: Class for the physics engine, if there is any
 *            (set to void* if there is none)
 *        - World: Class type of the world, if there is any
 *            (set to void* if there is none).
 *
 * \author Jennifer Buehler
 * \date October 2016
 */
template<class PhysicsWorldTypes_, class PhysicsEngineWorldTypes_>
class PhysicsEngineWorld:
  public PhysicsWorld<typename PhysicsWorldTypes_::WorldState,
                      typename PhysicsWorldTypes_::ModelID,
                      typename PhysicsWorldTypes_::ModelPartID,
                      typename PhysicsWorldTypes_::Vector3,
                      typename PhysicsWorldTypes_::Wrench>,
  public PhysicsEngineWorldInterface<
              typename PhysicsWorldTypes_::ModelID,
              typename PhysicsEngineWorldTypes_::Model,
              typename PhysicsEngineWorldTypes_::Contact,
              typename PhysicsEngineWorldTypes_::PhysicsEngine,
              typename PhysicsEngineWorldTypes_::World>
{
  public: typedef PhysicsWorldTypes_ PhysicsWorldTypes;
  public: typedef PhysicsEngineWorldTypes_ PhysicsEngineWorldTypes;

  private: typedef PhysicsWorld<typename PhysicsWorldTypes_::WorldState,
                      typename PhysicsWorldTypes_::ModelID,
                      typename PhysicsWorldTypes_::ModelPartID,
                      typename PhysicsWorldTypes_::Vector3,
                      typename PhysicsWorldTypes_::Wrench> PhysicsWorldParent;

  private: typedef PhysicsEngineWorldInterface<
             typename PhysicsWorldTypes_::ModelID,
             typename PhysicsEngineWorldTypes_::Model,
             typename PhysicsEngineWorldTypes_::Contact,
             typename PhysicsEngineWorldTypes_::PhysicsEngine,
             typename PhysicsEngineWorldTypes_::World> PhysicsEngineWorldParent;


  private: typedef PhysicsEngineWorld<PhysicsWorldTypes,
                                      PhysicsEngineWorldTypes> Self;
  public: typedef std::shared_ptr<Self> Ptr;
  public: typedef std::shared_ptr<const Self> ConstPtr;

  public: typedef typename PhysicsWorldTypes::WorldState WorldState;
  public: typedef typename PhysicsWorldTypes::ModelID ModelID;
  public: typedef typename PhysicsWorldTypes::ModelPartID ModelPartID;
  public: typedef typename PhysicsWorldTypes::Vector3 Vector3;
  public: typedef typename PhysicsWorldTypes::Wrench Wrench;

  public: typedef typename PhysicsEngineWorldTypes::Model Model;
  public: typedef typename PhysicsEngineWorldTypes::Contact NativeContact;
  public: typedef typename PhysicsEngineWorldTypes::PhysicsEngine PhysicsEngine;
  public: typedef typename PhysicsEngineWorldTypes::World World;

  public: typedef std::shared_ptr<Model> ModelPtr;
  public: typedef std::shared_ptr<NativeContact> NativeContactPtr;
  public: typedef std::shared_ptr<PhysicsEngine> PhysicsEnginePtr;
  public: typedef std::shared_ptr<World> WorldPtr;
};

}  // namespace

#endif  // COLLISION_BENCHMARK_PHYSICSWORLD
