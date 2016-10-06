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


#include <collision_benchmark/WorldLoader.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

/// Waits for the namespace \e worldNamespace to appear in the Gazebo list of namespaces. 
/// Repeatedly waits for new incoming namespaces (waiting *maximum* \e sleepTime seconds at each attempt)
/// and checks whether the given \e worldNamespace is in the list of namespaces.
/// \param maxWaitTime waits for this maximum time (seconds)
bool WaitForNamespace(std::string worldNamespace, float maxWaitTime = 10, float sleepTime = 1)
{
    gazebo::transport::TopicManager * topicManager = gazebo::transport::TopicManager::Instance();
    if (!topicManager)
    {
        std::cerr << "No topic manager instance" << std::endl;
        return false;
    }

    gazebo::common::Timer timer;
    timer.Start();

    bool found = false;
    while (!found && (timer.GetElapsed().Float() < maxWaitTime))
    {
        std::cout << "Waiting for namespace '" << worldNamespace << " 'to be loaded." << std::endl;
        // found at most 1 second for namespaces to appear.
        if (gazebo::transport::waitForNamespaces(gazebo::common::Time(sleepTime, 0)))
        {
            std::list<std::string> namespaces;
            topicManager->GetTopicNamespaces(namespaces);

            /*std::cout<<"Namespaces:"<<std::endl;
            for (std::list<std::string>::iterator it=namespaces.begin(); it!=namespaces.end(); ++it)
                std::cout<<*it<<std::endl;*/

            std::list<std::string>::iterator ns = std::find(namespaces.begin(), namespaces.end(), worldNamespace);
            bool loaded = (ns != namespaces.end()) && (*ns == worldNamespace);
            if (loaded)
            {
                std::cout << "Namespace '" << worldNamespace << "' received." << std::endl;
                found = true;
            }
            else
            {
                std::cout << "Namespace '" << worldNamespace << "' not received yet." << std::endl;
            }
        }
        else
        {
            std::cout << "No namespaces received yet.\n";
        }
    }
    return found;
}

sdf::ElementPtr collision_benchmark::GetWorldFromSDF(const std::string& filename, const std::string& name)
{
    sdf::ElementPtr sdfRoot;

    // Load the world file
    sdf::SDFPtr sdf(new sdf::SDF);
    if (!sdf::init(sdf))
    {
        std::cerr << "Unable to initialize sdf\n";
        return sdfRoot;
    }

    // Find the file.
    std::string fullFile = gazebo::common::find_file(filename);

    if (fullFile.empty())
    {
        std::cerr << "Unable to find file[" << filename << "]\n";
        return sdfRoot;
    }

    if (!sdf::readFile(fullFile, sdf))
    {
        std::cerr << "Unable to read sdf file[" << "empty.world" << "]\n";
        return sdfRoot;
    }

    sdfRoot = sdf->Root()->GetElement("world");

    if (!name.empty())
    {
        sdf::ParamPtr sdfWorldName = sdfRoot->GetAttribute("name");
        std::cout << "Replacing world name: '" << sdfWorldName->GetAsString() << "' with '" << name << "'" << std::endl;
        sdfWorldName->SetFromString(name);
    }

    return sdfRoot;
}

gazebo::physics::WorldPtr collision_benchmark::LoadWorldFromSDF(const sdf::ElementPtr& sdfRoot, const std::string& name)
{
    gazebo::physics::WorldPtr world;
    try
    {
        // XXX this just creates a new world object, all worls objects are
        // kept in a static list and can be retrieved. No Physics engine
        // is created yet - this is done in World::Load (line 257), called
        // from load_world: The physics engine specified **in the SDF** is
        // loaded.
        world = gazebo::physics::create_world(name);

        if (!name.empty())
        {
            sdf::ParamPtr sdfWorldName = sdfRoot->GetAttribute("name");
            if (sdfWorldName->GetAsString() != name)
            {
                std::cout << "INFO: Need to replace world name in SDF: '" << sdfWorldName->GetAsString() << "' with '" << name << "'" << std::endl;
                sdfWorldName->SetFromString(name);
            }
        }
        std::cout<<"Loading world..."<<std::endl;
        if (world) gazebo::physics::load_world(world, sdfRoot);

        std::cout<<"Initializing world..."<<std::endl;
        // call to world->init
        if (world) gazebo::physics::init_world(world);
    }
    catch (gazebo::common::Exception& e)
    {
        std::cerr << " Exception ocurred when loading world. " << e.GetErrorStr() << std::endl;
        return gazebo::physics::WorldPtr();
    }
    assert(world);    
    std::cout<<"World loaded."<<std::endl;

    return world;
}

gazebo::physics::WorldPtr collision_benchmark::LoadWorldFromFile(const std::string &_worldFile, const std::string& name)
{
    gazebo::physics::WorldPtr world;
    sdf::ElementPtr sdfRoot = GetWorldFromSDF(_worldFile, name);
    if (!sdfRoot)
    {
        std::cerr << "Could not load world" << std::endl;
        return world;
    }
    return LoadWorldFromSDF(sdfRoot, name);
}

gazebo::physics::WorldPtr collision_benchmark::LoadWorld(const std::string& worldfile, const std::string& name)
{
    gazebo::physics::WorldPtr world;
    try
    {
        // Load a world
        std::cout << "Loading world " << worldfile;
        if (!name.empty()) std::cout << " (to be named '" << name << "')";
        std::cout << std::endl;

        world = LoadWorldFromFile(worldfile, name);
    }
    catch (gazebo::common::Exception& e)
    {
        std::cerr << " Exception ocurred when loading world. " << e.GetErrorStr() << std::endl;
        return gazebo::physics::WorldPtr();
    }
    if (!world)
    {
        std::cerr << "Could not load world." << std::endl;
        return gazebo::physics::WorldPtr();
    }

    std::string worldNamespace = world->GetName();

    // wait for namespace to be loaded, to make sure the order of namespaces maintained
    // in the transport system eventually will correspond to the same order of the worlds
    if (!WaitForNamespace(worldNamespace))
    {
        std::cerr << "Namespace of world '" << worldNamespace << "' was not loaded" << std::endl;
        return gazebo::physics::WorldPtr();
    }
    return world;
}

bool collision_benchmark::LoadWorlds(const std::vector<std::string>& worldfiles,
                const std::vector<std::string>& worldNames,
                std::vector<gazebo::physics::WorldPtr>& worlds)
{
    // -- load all worlds --
    if (worldfiles.size() != worldNames.size())
    {
        std::cerr << "Have to specify a name for each world, even if it's an empty string" << std::endl;
        return false;
    }
    int i = 0;
    for (std::vector<std::string>::const_iterator w = worldfiles.begin();
            w != worldfiles.end(); ++w, ++i)
    {
        // std::cout << "Loading world " << *w << " (named as '" << worldNames[i] << "')" << std::endl;
        gazebo::physics::WorldPtr world = LoadWorld(*w, worldNames[i]);
        if (!world)
        {
            std::cerr << "Could not load world " << *w << std::endl;
            return false;
        }
        worlds.push_back(world);
    }
    return true;
}

#if 0
/// Convenience function to call gazebo::runWorld() on several worlds
bool RunWorlds(int iter, int steps, const std::vector<gazebo::physics::WorldPtr>& worlds)
{
    for (unsigned int i = 0; i < iter; ++i)
    {
        std::cout << "Re-running world(s), " << i << std::endl;
        for (std::vector<gazebo::physics::WorldPtr>::const_iterator w = worlds.begin();
                w != worlds.end(); ++w)
        {
            gazebo::physics::WorldPtr world = *w;
            // Run simulation for given number of steps.
            // XXX this method only calls world->RunBlocking(_iterations);
            gazebo::runWorld(world, steps);
        }
    }
    return true;
}

void fixSDF(std::vector<std::string>& states)
{
    for (std::vector<std::string>::iterator it = states.begin(); it != states.end(); ++it)
    {
        std::stringstream mod;
        // mod<<"<?xml version='1.0'?>";
        mod << "<sdf version='1.6'>" << *it << "</sdf>";
        *it = std::string(mod.str());
    }
}


void GetNewEntities(const gazebo::physics::WorldState& _state1,
                    const gazebo::physics::WorldState& _state2, std::vector<gazebo::physics::ModelState>& models,
                    std::vector<gazebo::physics::LightState>& lights)
{
    const gazebo::physics::ModelState_M& _modelStates1 = _state1.GetModelStates();
    for (gazebo::physics::ModelState_M::const_iterator iter =
                _modelStates1.begin(); iter != _modelStates1.end(); ++iter)
    {
        if (!_state2.HasModelState(iter->second.GetName()))
        {
            models.push_back(iter->second);
        }
    }

    const gazebo::physics::LightState_M& _lightStates1 = _state1.LightStates();
    for (const auto & light : _lightStates1)
    {
        if (!_state2.HasLightState(light.second.GetName()))
        {
            lights.push_back(light.second);
        }
    }
}

// XXX TODO REMOVE: Flags for testing
#define FORCE_TARGET_TIME_VALUES
//#define FORCE_KEEP_TIME_VALUES
//#define DEBUGMIRR
void SetWorldState(gazebo::physics::WorldPtr& world, const gazebo::physics::WorldState& targetState)
{
    bool pauseState = world->IsPaused();
    world->SetPaused(true);
    gazebo::physics::WorldState currentState(world);

#ifdef DEBUGMIRR
    std::cout << "Setting world state. " << std::endl;
    std::cout << "Target state: " << std::endl << targetState << std::endl;
    std::cout << "Current state: " << std::endl << currentState << std::endl;
#endif

    //re-set mirror state times
    // XXX TODO CHECK: should reset all times of the state as well,
    // because the *difference* is going to be added to it
    // Not sure yet how to best handle the times.
    /*    currentState.SetWallTime(common::Time(0));
        currentState.SetRealTime(common::Time(0));
        currentState.SetIterations(0);*/

    // Handle all insertions/deletions of models in a
    // differential state. The result will have the name of
    // origState, adding all the models/lights/etc from origState which are
    // not in emptyState (which is empty so it will be all models).
    gazebo::physics::WorldState diffState = targetState - currentState;
    gazebo::physics::WorldState newState = currentState + diffState;

#ifdef DEBUGMIRR
    std::cout << "Diff state: " << std::endl << diffState << std::endl;
#endif

    // Force the new state to use certain iteration/time values
    // in order to maintain consistency within the world

#ifdef FORCE_TARGET_TIME_VALUES
    // the + operator doesn't add iterations and times
    newState.SetIterations(targetState.GetIterations());
    newState.SetWallTime(targetState.GetWallTime());
    newState.SetRealTime(targetState.GetRealTime());
    newState.SetSimTime(targetState.GetSimTime());
#endif

#ifdef FORCE_KEEP_TIME_VALUES
    // the + operator doesn't add iterations and times
    newState.SetIterations(currentState.GetIterations());
    newState.SetWallTime(currentState.GetWallTime());
    newState.SetRealTime(currentState.GetRealTime());
    newState.SetSimTime(currentState.GetSimTime());
#endif

    std::vector<std::string> insertions = diffState.Insertions();
    fixSDF(insertions);
    newState.SetInsertions(insertions);
    std::vector<std::string> deletions = diffState.Deletions();
    newState.SetDeletions(deletions);
    fixSDF(deletions);

    // the insertions still have one drawback: They don't contain
    // the current model pose. So subsequently, the current pose
    // is only published as message in the *next* world update *only if*
    // the pose has changed (from within Model::OnPoseChange by call of World::SetWorldPose).
    // But if the pose within the target state world has not changed,
    // no message is published. So we need to force publishing the poses
    // of the newly inserted models.
    std::vector<gazebo::physics::ModelState> models;
    std::vector<gazebo::physics::LightState> lights;
    GetNewEntities(targetState, currentState, models, lights);

    // apply the state to the mirror world
    world->SetState(newState);

    // now, update the poses of the new models and lights
    for (const auto & model : models)
    {
        // std::cout<<"New model: "<<model.GetName()<<std::endl;
        gazebo::physics::ModelPtr m = world->GetModel(model.GetName());
        if (!m)
        {
            throw new gazebo::common::Exception(__FILE__, __LINE__,
                                                "Model not found though it should have been inserted.");
        }
        m->SetState(model);
    }

    for (const auto & light : lights)
    {
        // std::cout<<"New light: "<<light.GetName()<<std::endl;
        gazebo::physics::LightPtr l = world->Light(light.GetName());
        if (!l)
        {
            throw new gazebo::common::Exception(__FILE__, __LINE__,
                                                "Light not found though it should have been inserted.");
        }
        l->SetState(light);
    }

#ifdef DEBUGMIRR
    std::cout << "State set to:" << std::endl;
    gazebo::physics::WorldState _currentState(world);
    std::cout << _currentState << std::endl;
#endif

    world->SetPaused(pauseState);
}

void ClearModels(gazebo::physics::WorldPtr& world)
{
    throw new gazebo::common::Exception(__FILE__,__LINE__,"Implement me");
    bool pauseState = world->IsPaused();
    world->SetPaused(true);
    // XXX TODO CHECK: this does not clear the lights, but they should be
    // added/removed/changed in the subsequent diff state?!
    world->ClearModels();
    // need to reset physics engine in order to stop it
    // from publishing old contact points?
    gazebo::physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
    if (physics) physics->GetContactManager()->Clear();
    world->SetPaused(pauseState);
}

#endif
