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
/**
 * Author: Jennifer Buehler
 * Date: November 2016
 */
#include <collision_benchmark/GazeboPhysicsWorld.hh>
#include <collision_benchmark/GazeboWorldState.hh>
#include <collision_benchmark/GazeboStateCompare.hh>
#include <collision_benchmark/GazeboHelpers.hh>
#include <collision_benchmark/GazeboWorldLoader.hh>
#include <collision_benchmark/Helpers.hh>
#include <collision_benchmark/boost_std_conversion.hh>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/SystemPaths.hh>

#include <boost/filesystem.hpp>
#include <algorithm>

using collision_benchmark::GazeboPhysicsWorld;
using collision_benchmark::Contact;
using collision_benchmark::ContactInfo;

//////////////////////////////////////////////////////////////////////////////
GazeboPhysicsWorld::GazeboPhysicsWorld(bool _enforceContactComputation)
  : enforceContactComputation(_enforceContactComputation),
    paused(false)
{
}

//////////////////////////////////////////////////////////////////////////////
GazeboPhysicsWorld::~GazeboPhysicsWorld()
{
}

//////////////////////////////////////////////////////////////////////////////
bool GazeboPhysicsWorld::SupportsSDF() const
{
  return true;
}

//////////////////////////////////////////////////////////////////////////////
bool GazeboPhysicsWorld::WaitForNamespace
      (const gazebo::physics::WorldPtr &gzworld, float maxWait, float waitSleep)
{
  std::string worldNamespace = gzworld->Name();

  // wait for namespace to be loaded, to make sure the order of
  // namespaces maintained in the transport system eventually will correspond
  // to the same order of the worlds
  if (!collision_benchmark::WaitForNamespace(worldNamespace,
                                             maxWait, waitSleep))
  {
    std::cerr << "Namespace of world '" << worldNamespace
              << "' was not loaded" << std::endl;
    return false;
  }
  return true;
}


//////////////////////////////////////////////////////////////////////////////
collision_benchmark::OpResult
GazeboPhysicsWorld::LoadFromSDF(const sdf::ElementPtr &sdf,
                                const std::string &worldname)
{
  gazebo::physics::WorldPtr gzworld =
    collision_benchmark::LoadWorldFromSDF(sdf, worldname);

  if (!gzworld)
    return collision_benchmark::FAILED;

  if (OnLoadWaitForNamespace &&
      !WaitForNamespace(gzworld, OnLoadMaxWaitForNamespace,
                        OnLoadWaitForNamespaceSleep))
    return collision_benchmark::FAILED;

  SetWorld(collision_benchmark::to_std_ptr<gazebo::physics::World>(gzworld));
  return collision_benchmark::SUCCESS;
}

//////////////////////////////////////////////////////////////////////////////
collision_benchmark::OpResult
GazeboPhysicsWorld::LoadFromFile(const std::string &filename,
                                 const std::string &worldname)
{
  gazebo::physics::WorldPtr gzworld =
    collision_benchmark::LoadWorldFromFile(filename, worldname);

  if (!gzworld)
    return collision_benchmark::FAILED;

  if (OnLoadWaitForNamespace &&
      !WaitForNamespace(gzworld, OnLoadMaxWaitForNamespace,
                        OnLoadWaitForNamespaceSleep))
    return collision_benchmark::FAILED;

  SetWorld(collision_benchmark::to_std_ptr<gazebo::physics::World>(gzworld));
  return collision_benchmark::SUCCESS;
}

//////////////////////////////////////////////////////////////////////////////
collision_benchmark::OpResult
GazeboPhysicsWorld::LoadFromString(const std::string &str,
                                   const std::string &worldname)
{
  gazebo::physics::WorldPtr gzworld =
    collision_benchmark::LoadWorldFromSDFString(str, worldname);

  if (!gzworld)
    return collision_benchmark::FAILED;

  if (OnLoadWaitForNamespace &&
      !WaitForNamespace(gzworld, OnLoadMaxWaitForNamespace,
                        OnLoadWaitForNamespaceSleep))
    return collision_benchmark::FAILED;

  SetWorld(collision_benchmark::to_std_ptr<gazebo::physics::World>(gzworld));
  return collision_benchmark::SUCCESS;
}

//////////////////////////////////////////////////////////////////////////////
bool CopyAllResourcesHelper(const sdf::ElementPtr &elem,
                            const std::list<std::string>& parentElemNames,
                            const std::string &destinationBase,
                            const std::string &destinationSubdir)
{
  if (!elem) return false;
  for (std::list<std::string>::const_iterator it = parentElemNames.begin();
       it != parentElemNames.end(); ++it)
  {
    std::string name = *it;
    if (elem->HasElement(name))
    {
      // std::cout << "Found a '" << name << "' element in element "
      //          << elem->GetName() << std::endl;
      sdf::ElementPtr subElem = elem->GetElement(name);
      if (subElem->HasElement("uri"))
      {
        sdf::ElementPtr uriElem = subElem->GetElement("uri");
        if (!uriElem->GetValue() ||
            (uriElem->GetValue()->GetTypeName() != "string"))
        {
          std::cerr << "URI has no value or is not of expected string type"
                    << std::endl;
          continue;
        }

        std::string uri = uriElem->GetValue()->GetAsString();
        int index = uri.find("://");
        std::string prefix = uri.substr(0, index);
        boost::filesystem::path relPath =
          uri.substr(index + 3, uri.size() - index - 3);
        // the URI should be changed to:
        boost::filesystem::path uriDest =
          boost::filesystem::path(destinationSubdir) / relPath;
        // the absolute path to the directory where to move the file to:
        boost::filesystem::path fullDestinationDir =
          boost::filesystem::path(destinationBase) / uriDest.parent_path();
        // make the directory
        if (!collision_benchmark::makeDirectoryIfNeeded
                                                 (fullDestinationDir.native()))
        {
          std::cerr << "Could not create directory "
                    << fullDestinationDir << std::endl;
          return false;
        }
        // find the file in the existing GAZEBO_RESOURCE_PATH
        boost::filesystem::path filename = gazebo::common::find_file(uri);
        if (filename.empty())
        {
          std::cerr << "Could not find file " << uri << " in gazebo paths. "
                    << std::endl;
          return false;
        }
        // full filename path in destination:
        boost::filesystem::path fullDestinationFile =
          fullDestinationDir / filename.filename();
        // copy the file and print a warning if it already existed.
        // We could use boost::filesystem::copy_option::overwrite_if_exists
        // instead to force overwriting, maybe add this as a parameter later
        // on. For now, we're conservative and keep existing files.
        std::cout << "Copy file from " << filename.string()
                  << " to " << fullDestinationFile << std::endl;
        boost::system::error_code err;
        boost::filesystem::copy_file(filename, fullDestinationFile,
                        boost::filesystem::copy_option::fail_if_exists, err);
        if (err.value() != boost::system::errc::success)
        {
          bool exists = err.value() == boost::system::errc::file_exists;
          if (exists)
            std::cerr << "WARNING: ";
          else
            std::cerr << "ERROR: ";
          std::cerr << "Could not copy file from " << filename.string()
                    << " to " << fullDestinationFile;
          if (exists)
          {
            std::cerr << " because file exists. Keeping existing file. "
                      << std::endl;
          }
          else
          { // fatal error
            std::cerr << ". Error: " << err.message() << std::endl;
            return false;
          }
        }
        // std::cout <<"Setting uri to " << prefix << "://"
        //           << uriDest.string() << std::endl;
        uriElem->GetValue()->Set(prefix + "://" +uriDest.string());
      }
    }
  }

  // recurse into children
  for (sdf::ElementPtr childElem = elem->GetFirstElement(); childElem;
        childElem = childElem->GetNextElement())
  {
    // std::cout << "Has child "
    //          << childElem->GetName() << std::endl;
    CopyAllResourcesHelper(childElem, parentElemNames, destinationBase,
                           destinationSubdir);
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////
bool GazeboPhysicsWorld::CopyAllModelResources(const sdf::ElementPtr &elem,
                                const std::list<std::string>& parentElemNames,
                                const std::string &destinationBase,
                                const std::string &destinationSubdir)
{
  // replace the resources for all models...
  for (sdf::ElementPtr modelElem = elem->GetElement("model"); modelElem;
       modelElem = modelElem->GetNextElement("model"))
  {
    // std::cout << "Found model "
    //          << modelElem->GetAttribute("name")->GetAsString() << std::endl;

    if (!CopyAllResourcesHelper(modelElem, parentElemNames,
                                destinationBase, destinationSubdir))
    {
      std::cerr << "Could not replace URI resource in model "
                << modelElem->GetAttribute("name")->GetAsString()
                << std::endl;
      return false;
    }
    // recurse into nested models
    if (modelElem->HasElement("model") &&
        !CopyAllModelResources(modelElem, parentElemNames,
                               destinationBase, destinationSubdir))
    {
      std::cerr << "Could not replace URI resource in nested model "
                << modelElem->GetAttribute("name")->GetAsString()
                << std::endl;
      return false;
    }
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////
bool GazeboPhysicsWorld::SaveToFile(const std::string &filename,
                                    const std::string &resourceDir,
                                    const std::string &resourceSubdir)
{
  if (!world) return false;

  // we could use world->Save(filename), however this would not include
  // saving mesh files in the same directory to the resource target directory.

  std::ofstream out(filename.c_str(), std::ios::out);
  if (!out)
  {
    // try to make directory first
    boost::filesystem::path fpath(filename);
    fpath = fpath.parent_path();
    if (!collision_benchmark::makeDirectoryIfNeeded(fpath.string()))
    {
      std::cerr << "Unable to make directory for " << filename << std::endl;
      return false;
    }
    out.open(filename.c_str(), std::ios::out);
    if (!out.is_open())
    {
      std::cerr << "Unable to write file " << filename << std::endl;
      return false;
    }
  }

  sdf::ElementPtr worldSdf = world->SDF();
  if (!worldSdf)
  {
    std::cerr << "Could not get SDF of world" << std::endl;
    return false;
  }

  if (worldSdf->GetName() != "world")
  {
    std::cerr << "Missing SDF 'world' element" << std::endl;
    return false;
  }

  sdf::ElementPtr sdf = sdf::SDF::WrapInRoot(worldSdf->Clone());

  if (!resourceDir.empty())
  {
    sdf::ElementPtr worldElem = sdf->GetElement("world");
    std::list<std::string> parentElemNames;
    parentElemNames.push_back("mesh");
    CopyAllModelResources(worldElem, parentElemNames,
                          resourceDir, resourceSubdir);
  }
  // std::cout << "Writing world " << worldElem->ToString("") << std::endl;
  // std::cout << "Saving world to file " << filename << std::endl;

  std::string sdfString = sdf->ToString("");
  out << "<?xml version ='1.0'?>\n";
  out << sdfString;
  out.close();
  return true;
}

//////////////////////////////////////////////////////////////////////////////
GazeboPhysicsWorld::ModelLoadResult
GazeboPhysicsWorld::AddModelFromFile(const std::string &filename,
                                     const std::string &modelname)
{
  sdf::ElementPtr sdfRoot =
    collision_benchmark::GetSDFElementFromFile(filename, "model", modelname);
  ModelLoadResult ret;
  ret.opResult = FAILED;
  if (!sdfRoot)
  {
    std::cerr<< " Could not get SDF for model in " << filename << std::endl;
    return ret;
  }

  ret = AddModelFromSDF(sdfRoot);
  return ret;
}

//////////////////////////////////////////////////////////////////////////////
GazeboPhysicsWorld::ModelLoadResult
GazeboPhysicsWorld::AddModelFromString(const std::string &str,
                                       const std::string &modelname)
{
  ModelLoadResult ret;
  ret.opResult = FAILED;

  std::string useStr = str;
  int checkSDF = collision_benchmark::isProperSDFString(useStr);
  if (checkSDF < 0)
  {
    if (checkSDF == -2)
    {
      collision_benchmark::wrapSDF(useStr);
    }
    else
    {
      std::cerr << "SDF is not proper so cannot load model from the string. "
               <<"Value: " << checkSDF << std::endl;
      return ret;
    }
  }

  sdf::ElementPtr sdfRoot =
    collision_benchmark::GetSDFElementFromString(useStr, "model", modelname);
  if (!sdfRoot)
  {
    std::cerr<< " Could not get SDF for model." << std::endl;
    return ret;
  }

  ret = AddModelFromSDF(sdfRoot);
  return ret;
}

//////////////////////////////////////////////////////////////////////////////
GazeboPhysicsWorld::ModelLoadResult
GazeboPhysicsWorld::AddModelFromSDF(const sdf::ElementPtr &sdf,
                                    const std::string &modelname)
{
  gazebo::physics::ModelPtr model =
    collision_benchmark::LoadModelFromSDF(sdf, world, modelname);
  ModelLoadResult ret;
  if (!model)
  {
    ret.opResult = FAILED;
    return ret;
  }
  ret.opResult = SUCCESS;
  ret.modelID = model->GetName();
  return ret;
}

//////////////////////////////////////////////////////////////////////////////
bool GazeboPhysicsWorld::SupportsShapes() const
{
  return true;
}

//////////////////////////////////////////////////////////////////////////////
std::string
GazeboPhysicsWorld::GetMeshOutputPath(std::string &outputSubdir) const
{
  std::string outputPath =
    (boost::filesystem::path(gazebo::common::SystemPaths::Instance()->TmpPath())
     / boost::filesystem::path(".gazebo/models")).native();

  outputSubdir = "meshes";
  return outputPath;
}


//////////////////////////////////////////////////////////////////////////////
GazeboPhysicsWorld::ModelLoadResult
GazeboPhysicsWorld::AddModelFromShape(const std::string &modelname,
                                      const Shape::Ptr &shape,
                                      const Shape::Ptr &collShape)
{
  ModelLoadResult ret;
  ret.opResult = FAILED;

  if (modelname.empty())
  {
    std::cerr << "World " << GetName() << ": Must specify model name"
              << std::endl;
    return ret;
  }

  // Build the SDF
  sdf::ElementPtr root(new sdf::Element());
  root->SetName("model");
  root->AddAttribute("name", "string", modelname, true, "model name");

  sdf::ElementPtr pose = shape->GetPoseSDF();
  root->InsertElement(pose);

  sdf::ElementPtr link(new sdf::Element());
  link->SetName("link");
  link->AddAttribute("name", "string", "link", true, "link name");
  root->InsertElement(link);

  // use a temporary gazebo models directory to put the mesh file
  // there. Make sure the path is added to the gazebo file paths as well, so
  // that Gazebo will be able to find it.
  std::string outputSubdir;
  std::string outputPath = GetMeshOutputPath(outputSubdir);

  std::list<std::string> gzSysPaths =
    gazebo::common::SystemPaths::Instance()->GetGazeboPaths();
  if (std::find(gzSysPaths.begin(), gzSysPaths.end(), outputPath)
        == gzSysPaths.end())
  {
    std::cout << "Adding path " << outputPath << " to Gazebo paths"
              << std::endl;
    gazebo::common::SystemPaths::Instance()->AddGazeboPaths(outputPath);
  }

  sdf::ElementPtr shapeGeom =
    shape->GetShapeSDF(true, outputPath, outputSubdir);
  sdf::ElementPtr visual(new sdf::Element());
  visual->SetName("visual");
  visual->AddAttribute("name", "string", "visual", true, "visual name");
  visual->InsertElement(shapeGeom);
  link->InsertElement(visual);

  sdf::ElementPtr shapeColl;
  if (collShape)
  {
    shapeColl = collShape->GetShapeSDF(true, outputPath, outputSubdir);
  }
  else
  {
    // build collision shape out of the visual shape
    if (shape->SupportLowRes())
      shapeColl = shape->GetShapeSDF(false, outputPath, outputSubdir);
    else
      shapeColl = shapeGeom;
  }

  if (!shapeColl)
  {
    std::cerr << "Could not construct collision shape SDF" << std::endl;
    return ret;
  }

  sdf::ElementPtr collision(new sdf::Element());
  collision->SetName("collision");
  collision->AddAttribute("name", "string", "collision",
                          true, "collision name");
  collision->InsertElement(shapeColl);
  link->InsertElement(collision);

  return AddModelFromSDF(root);
}

//////////////////////////////////////////////////////////////////////////////
bool GazeboPhysicsWorld::HasModel(const ModelID &id) const
{
  return world->ModelByName(id) != nullptr;
}

//////////////////////////////////////////////////////////////////////////////
std::vector<GazeboPhysicsWorld::ModelID>
GazeboPhysicsWorld::GetAllModelIDs() const
{
  std::vector<GazeboPhysicsWorld::ModelID> names;
  int numModels = world->ModelCount();
  for (int i = 0; i < numModels; ++i)
  {
    gazebo::physics::ModelPtr m = world->ModelByIndex(i);
    names.push_back(m->GetName());
  }
  return names;
}

//////////////////////////////////////////////////////////////////////////////
int GazeboPhysicsWorld::GetIntegerModelID(const ModelID &id) const
{
  gazebo::physics::ModelPtr m = world->ModelByName(id);
  if (!m) return -1;
  return m->GetId();
}

//////////////////////////////////////////////////////////////////////////////
bool GazeboPhysicsWorld::RemoveModel(const ModelID &id)
{
  gazebo::physics::ModelPtr m = world->ModelByName(id);
  if (!m) return false;
  world->RemoveModel(m);
  return true;
}

//////////////////////////////////////////////////////////////////////////////
void GazeboPhysicsWorld::Clear()
{
  collision_benchmark::ClearModels(world);
}

//////////////////////////////////////////////////////////////////////////////
GazeboPhysicsWorld::WorldState GazeboPhysicsWorld::GetWorldState() const
{
  gazebo::physics::WorldState s(world);
  return s;
}

//////////////////////////////////////////////////////////////////////////////
GazeboPhysicsWorld::WorldState
GazeboPhysicsWorld::GetWorldStateDiff(const WorldState &other) const
{
  gazebo::physics::WorldState diffState = other - GetWorldState();
  // std::cout << "Diff state: " << std::endl << diffState << std::endl;
  return diffState;
}

//////////////////////////////////////////////////////////////////////////////
collision_benchmark::OpResult
GazeboPhysicsWorld::SetWorldState(const WorldState &state, bool isDiff)
{
  collision_benchmark::SetWorldState(world, state);

#ifdef DEBUG
  gazebo::physics::WorldState _currentState(world);
  GazeboStateCompare::Tolerances t =
    GazeboStateCompare::Tolerances::CreateDefault(1e-03);
  if (!world->PhysicsEnabled()) t.CheckDynamics = false;
  if (!GazeboStateCompare::Equal(_currentState, state, t))
  {
    std::cerr << "Target state was not set as supposed to!!" << std::endl;
  }
#endif

  return collision_benchmark::SUCCESS;
}


//////////////////////////////////////////////////////////////////////////////
bool GazeboPhysicsWorld::SetBasicModelState(const ModelID  &_id,
                                            const BasicState &_state)
{
  gazebo::physics::ModelPtr m = world->ModelByName(_id);
  if (!m)
  {
    std::cerr << "World " << GetName() << ": Model " << _id
              << " could not be found" << std::endl;
    return false;
  }
  ignition::math::Pose3d pose = m->WorldPose();
  if (_state.PosEnabled()) pose.Pos().Set(_state.position.x,
                                          _state.position.y,
                                          _state.position.z);
  if (_state.RotEnabled()) pose.Rot().Set(_state.rotation.w,
                                          _state.rotation.x,
                                          _state.rotation.y,
                                          _state.rotation.z);

  // std::cout << "Setting world state " << _state << std::endl;

  // Fix / HACK: Because World::posePub is throttled for publishing the
  // pose, space the publishing of poses apart to make sure they are
  // actually published.
  // Get the current time
  gazebo::common::Time currentTime = gazebo::common::Time::GetWallTime();
  static const double updatePeriod = 1.0 / 60.0;
  // Skip publication if the time difference is less than the update period.
  double timeDiff = (currentTime - this->prevPoseSetTime).Double();
  if (timeDiff < updatePeriod)
  {
    /* std::cout << "WARNING: Throttling the setting of the pose due to the "
              << "gazebo::physics::World throttle on pose publishing. "
              << __FILE__ << std::endl;*/
    gazebo::common::Time::Sleep((updatePeriod - timeDiff) + 1e-03);
  }
  this->prevPoseSetTime = currentTime;

  m->SetWorldPose(pose);

  if (_state.ScaleEnabled())
  {
    ignition::math::Vector3d s(_state.scale.x,
                               _state.scale.y,
                               _state.scale.z);
    m->SetScale(s);
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////
bool GazeboPhysicsWorld::GetBasicModelState(const ModelID  &_id,
                                            BasicState &_state) const
{
  gazebo::physics::ModelPtr m = world->ModelByName(_id);
  if (!m)
  {
    std::cerr << "World " << GetName() << ": Model " << _id
              << " could not be found" << std::endl;
    return false;
  }
  ignition::math::Pose3d pose = m->WorldPose();
  _state.SetPosition(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
  _state.SetRotation(pose.Rot().X(), pose.Rot().Y(),
                     pose.Rot().Z(), pose.Rot().W());
  ignition::math::Vector3d scale = m->Scale();
  _state.SetScale(scale.X(), scale.Y(), scale.Z());
  return true;
}

// This flag sets the behaviour of GazeboPhysicsWorld::Update().
// If defined, physics::World::Step(int) is called every loop, which is
// the behavior we would like.
// If not defined, gazebo::runWorld() is used instead.
//
// See also issue #2509
// https://bitbucket.org/osrf/gazebo/issues/2509/
#define UPDATE_CALLS_STEP

//////////////////////////////////////////////////////////////////////////////
void GazeboPhysicsWorld::PostWorldLoaded()
{
  GZ_ASSERT(world, "World is null, must have been loaded!");
#ifdef UPDATE_CALLS_STEP
  // Run the world in paused mode so that the main
  // loop of the gazebo world is working and calls of
  // Update() will do a World::Step().
  world->SetPaused(true);
  world->Run(0);
#endif
}


//////////////////////////////////////////////////////////////////////////////
void GazeboPhysicsWorld::Update(int steps, bool force)
{
  // std::cout << "Running " << steps << " steps for world "
  //          <<world->Name() << ", physics engine: "
  //          <<world->Physics()->GetType() << std::endl;
#ifdef UPDATE_CALLS_STEP
  if (!force && IsPaused()) return;

  // if the world is not paused, it is updating itself already
  // automatically (started in PostWorldLoaded().
  // We should either return and don't call
  // Step(), or first pause the world.
  if (!world->IsPaused())
  {
    static bool printOnce = true;
    if (printOnce)
    {
      std::cout << "WARNING: The Gazebo world is not paused. "
        <<"In GazeboPhysicsWorld::Update(), we operate it in "
        <<"paused mode and rely on manually doing the updates "
        <<"instead of letting the gazebo world update "
        <<"itself continuously." << std::endl;
      printOnce = false;
    }
    world->SetPaused(true);
  }

  // Step() only works if the world is paused.
  // It advances the state despite the paused state.
  world->Step(steps);
#else
  // This method calls world->RunBlocking();
  gazebo::runWorld(world, steps);
  // iterations is always 1 if it has been set with steps != 0
  // in call above. Also, start time will be reset every time (see
  // World::RunLoop()).
  // std::cout << "Iterations: " << world->Iterations() << std::endl;
#endif
}

//////////////////////////////////////////////////////////////////////////////
void GazeboPhysicsWorld::SetPaused(bool flag)
{
#ifdef UPDATE_CALLS_STEP
  paused = flag;
#else
  world->SetPaused(flag);
#endif
}

//////////////////////////////////////////////////////////////////////////////
bool GazeboPhysicsWorld::IsPaused() const
{
#ifdef UPDATE_CALLS_STEP
  return paused;
#else
  return world->IsPaused();
#endif
}

//////////////////////////////////////////////////////////////////////////////
std::string GazeboPhysicsWorld::GetName() const
{
  return world->Name();
}

//////////////////////////////////////////////////////////////////////////////
bool GazeboPhysicsWorld::SupportsContacts() const
{
  return true;
}

//////////////////////////////////////////////////////////////////////////////
// helper function which can be used to get contact info of either
// all models (m1 and m2 set to NULL), or for one model
// (m1 = NULL and m2 = NULL) or for two models (m1 != NULL and m2 != NULL).
std::vector<GazeboPhysicsWorld::ContactInfoPtr>
GetContactInfoHelper(const gazebo::physics::WorldPtr &world,
                     const GazeboPhysicsWorld::ModelID * m1 = NULL,
                     const GazeboPhysicsWorld::ModelID * m2 = NULL)
{
  std::vector<GazeboPhysicsWorld::ContactInfoPtr> ret;
  const gazebo::physics::ContactManager* contactManager =
    world->Physics()->GetContactManager();
  GZ_ASSERT(contactManager, "Contact manager has to be set");
  const std::vector<gazebo::physics::Contact*>& contacts =
    contactManager->GetContacts();
  // std::cout << "World has " << contacts.size() << "contacts." << std::endl;
  for (int cIdx = 0; cIdx < contactManager->GetContactCount(); ++cIdx)
  {
    if (cIdx >= contacts.size())
    {
      THROW_EXCEPTION("Contact count not consistent with vector size, idx="
                      << cIdx << ", size = " << contacts.size());
    }
    const gazebo::physics::Contact * c = contacts[cIdx];
    GZ_ASSERT(c->collision1->GetModel(), "Model of collision1 must be set");
    GZ_ASSERT(c->collision1->GetLink(), "Link of collision1 must be set");
    GZ_ASSERT(c->collision2->GetModel(), "Model of collision2 must be set");
    GZ_ASSERT(c->collision2->GetLink(), "Link of collision2 must be set");

    std::string m1Name = c->collision1->GetModel()->GetName();
    std::string m2Name = c->collision2->GetModel()->GetName();

    if (m1)
    {
      if (m2)
      { // m1Name and m2Name do not correspond to m1 and m2
        if ((*m1 != m1Name || *m2 != m2Name) &&
            (*m1 != m2Name || *m2 != m1Name)) continue;
      }
      else
      { // m1 has to be m1Name or m2Name to continue
        if (*m1 != m1Name && *m1 != m2Name) continue;
      }
    }

    if (c->count == 0)
    {
      // for BULLET, it can happen quite frequently that a contact is given
      // while there is no actual contact information.
      // See also this issue:
      // https://bitbucket.org/osrf/gazebo/issues/2222/bullet-contact-points-with-positive
      // For now, don't print this warning for bullet.
      if (world->Physics()->GetType() != "bullet")
      {
        std::cerr << "CONSISTENCY GazeboPhysicsWorld: With no contacts, "
                  << "there should be no collision!! World: " << world->Name()
                  << " Models: " << m1Name << ", " << m2Name << std::endl;
      }
      continue;
    }

    GazeboPhysicsWorld::ContactInfoPtr
      cInfo(new GazeboPhysicsWorld::ContactInfo
            (m1Name, c->collision1->GetLink()->GetName(),
             m2Name, c->collision2->GetLink()->GetName()));
    for (int i = 0; i < c->count; ++i)
    {
      if (c->depths[i] < 0)
      {
        // negative depths shoudl be considered invalid if they
        // are far beyond 0
        static double tol = 1e-03;
        if (c->depths[i] < -tol)
        {
          std::cout << "DEBUG-INFO: Negative contact distance found in world "
                    << world->Name() <<", depth = " << c->depths[i]
                    << ". Skipping contact. " << std::endl;
          continue;
        }
      }
      cInfo->contacts.push_back
        (GazeboPhysicsWorld::Contact(c->positions[i], c->normals[i],
                                     c->wrench[i], c->depths[i]));
    }

    if (cInfo->contacts.empty())
    {
     std::cout << "WARNING: All contact points gotten from models "
               << m1Name << " / " << c->collision1->GetLink()->GetName() << ", "
               << m2Name << " / " << c->collision2->GetLink()->GetName()
               << " world " << world->Name() <<" skipped. " << std::endl;
    }
    else
    {
      ret.push_back(cInfo);
    }
  }
  return ret;
}

//////////////////////////////////////////////////////////////////////////////
// deleter which does nothing, to be used for
// std::shared_ptr with extreme caution!
template<typename Type>
void null_deleter(Type *)
{}

//////////////////////////////////////////////////////////////////////////////
// helper function which can be used to get contact info of either
// all models (m1 and m2 set to NULL), or for one model
// (m1 = NULL and m2 = NULL) or for two models (m1 != NULL and m2 != NULL).
std::vector<GazeboPhysicsWorld::NativeContactPtr>
GetNativeContactsHelper(const gazebo::physics::WorldPtr &world,
                        const GazeboPhysicsWorld::ModelID * m1 = NULL,
                        const GazeboPhysicsWorld::ModelID * m2 = NULL)
{
  std::vector<GazeboPhysicsWorld::NativeContactPtr> ret;

  const gazebo::physics::ContactManager* contactManager
    = world->Physics()->GetContactManager();
  GZ_ASSERT(contactManager, "Contact manager has to be set");
  const std::vector<gazebo::physics::Contact*>& contacts =
    contactManager->GetContacts();
  for (std::vector<gazebo::physics::Contact*>::const_iterator
       it = contacts.begin(); it != contacts.end(); ++it)
  {
      gazebo::physics::Contact * c=*it;
      GZ_ASSERT(c->collision1->GetModel(), "Model of collision1 must be set");
      GZ_ASSERT(c->collision1->GetLink(), "Link of collision1 must be set");
      GZ_ASSERT(c->collision2->GetModel(), "Model of collision2 must be set");
      GZ_ASSERT(c->collision2->GetLink(), "Link of collision2 must be set");

      std::string m1Name = c->collision1->GetModel()->GetName();
      std::string m2Name = c->collision2->GetModel()->GetName();

      if (m1)
      {
        if (m2)
        { // m1Name and m2Name do not correspond to m1 and m2
          if ((*m1 != m1Name || *m2 != m2Name) &&
              (*m1 != m2Name || *m2 != m1Name)) continue;
        }
        else
        { // m1 has to be m1Name or m2Name to continue
          if (*m1 != m1Name && *m1 != m2Name) continue;
        }
      }

      // XXX HACK -> Also remove warning in header documentation of
      // GetNativeContacts() when this is resolved!
      // While Gazebo doesn't manage contacts as shared pointers, unfortunately
      // we will need to return the std::shared_ptr<gazebo::physics::Contact>
      // pointers without deleter. This may lead to awful segfaults if the
      // contacts are used beyond their lifetime in Gazebo.
      // However it is expected (?) that soon Gazebo will use shared pointers
      // for this as well, so keep this flakey solution for now.
      GazeboPhysicsWorld::NativeContactPtr
        gzContact(c, &null_deleter<GazeboPhysicsWorld::NativeContact>);
      ret.push_back(gzContact);
  }
  return ret;
}


//////////////////////////////////////////////////////////////////////////////
std::vector<GazeboPhysicsWorld::ContactInfoPtr>
GazeboPhysicsWorld::GetContactInfo() const
{
  return GetContactInfoHelper(world);
}

//////////////////////////////////////////////////////////////////////////////
std::vector<GazeboPhysicsWorld::ContactInfoPtr>
GazeboPhysicsWorld::GetContactInfo(const ModelID &m1, const ModelID &m2) const
{
  return GetContactInfoHelper(world, &m1, &m2);
}

//////////////////////////////////////////////////////////////////////////////
std::vector<GazeboPhysicsWorld::NativeContactPtr>
GazeboPhysicsWorld::GetNativeContacts() const
{
  return GetNativeContactsHelper(world);
}

//////////////////////////////////////////////////////////////////////////////
std::vector<GazeboPhysicsWorld::NativeContactPtr>
GazeboPhysicsWorld::GetNativeContacts(const ModelID &m1,
                                      const ModelID &m2) const
{
  return GetNativeContactsHelper(world, &m1, &m2);
}


//////////////////////////////////////////////////////////////////////////////
bool GazeboPhysicsWorld::IsAdaptor() const
{
  return true;
}

//////////////////////////////////////////////////////////////////////////////
#ifndef CONTACTS_ENFORCABLE
void GazeboPhysicsWorld::OnContact(ConstContactsPtr &_msg)
{
//  std::cout << "DEBUG: Got contact!" << std::endl;
}
#endif

//////////////////////////////////////////////////////////////////////////////
void GazeboPhysicsWorld::SetEnforceContactsComputation(bool flag)
{
  enforceContactComputation = flag;
#ifndef CONTACTS_ENFORCABLE
  if (enforceContactComputation)
  {
    if (!node)
    {
      node = gazebo::transport::NodePtr(new gazebo::transport::Node());
      node->Init(GetName());
    }
    contactsSub = node->Subscribe("~/physics/contacts",
                                  &GazeboPhysicsWorld::OnContact, this);
  }
  else
  {
    contactsSub.reset();
  }
#else
  assert(world->Physics() && world->Physics()->GetContactManager());
  world->Physics()->GetContactManager()->SetNeverDropContacts(flag);
#endif
}

//////////////////////////////////////////////////////////////////////////////
collision_benchmark::RefResult
GazeboPhysicsWorld::SetWorld(const WorldPtr &_world)
{
  world = collision_benchmark::to_boost_ptr<World>(_world);
  SetEnforceContactsComputation(enforceContactComputation);
  PostWorldLoaded();
  return collision_benchmark::REFERENCED;
}

//////////////////////////////////////////////////////////////////////////////
GazeboPhysicsWorld::WorldPtr GazeboPhysicsWorld::GetWorld() const
{
  return collision_benchmark::to_std_ptr<World>(world);
}

//////////////////////////////////////////////////////////////////////////////
GazeboPhysicsWorld::ModelPtr
GazeboPhysicsWorld::GetModel(const ModelID &model) const
{
  gazebo::physics::ModelPtr m = world->ModelByName(model);
  return collision_benchmark::to_std_ptr<gazebo::physics::Model>(m);
}


//////////////////////////////////////////////////////////////////////////////
bool GazeboPhysicsWorld::GetAABB(const ModelID &id,
                                 Vector3& min, Vector3& max,
                                 bool &inLocalFrame) const
{
  gazebo::physics::ModelPtr m = world->ModelByName(id);
  if (!m) return false;
  ignition::math::Box box = m->BoundingBox();
  min = Vector3(box.Min().X(), box.Min().Y(), box.Min().Z());
  max = Vector3(box.Max().X(), box.Max().Y(), box.Max().Z());
  inLocalFrame = false;
  return true;
}

//////////////////////////////////////////////////////////////////////////////
GazeboPhysicsWorld::PhysicsEnginePtr
GazeboPhysicsWorld::GetPhysicsEngine() const
{
  if (world)
    return collision_benchmark::to_std_ptr<GazeboPhysicsWorld::PhysicsEngine>
            (world->Physics());
  return PhysicsEnginePtr();
}

//////////////////////////////////////////////////////////////////////////////
void GazeboPhysicsWorld::SetDynamicsEnabled(const bool flag)
{
  if (world) world->SetPhysicsEnabled(flag);
}
