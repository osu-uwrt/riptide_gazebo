/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef MODEL_PLUGIN_MOVEENTITY_HH_
#define MODEL_PLUGIN_MOVEENTITY_HH_

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <gz/sim/System.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <gz/math.hh>
// It's good practice to use a custom namespace for your project.
namespace move_entity
{
  // This is the main plugin's class. It must inherit from System and at least
  // one other interface.
  // Here we use `ISystemPostUpdate`, which is used to get results after
  // physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
  // plugins that want to send commands.
  class MoveEntity:
    public gz::sim::System,
    public gz::sim::ISystemPostUpdate,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    private: gz::sim::Entity entity;

    //these are stored seperately because of issues the gz::msgs::pose - fix later?
    public: 
    gz::msgs::Vector3d  current_position;
    gz::msgs::Quaternion current_orentation;

    // Plugins inheriting ISystemPostUpdate must implement the PostUpdate
    // callback. This is called at every simulation iteration after the physics
    // updates the world. The _info variable provides information such as time,
    // while the _ecm provides an interface to all entities and components in
    // simulation.
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    public: void Configure(const gz::sim::Entity &_id,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          gz::sim::EntityComponentManager &_ecm,
                          gz::sim::EventManager &_eventMgr) final;  

    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    public: void PositionSubCallback(const gz::msgs::Vector3d &_msg);
    public: void OrentationSubCallback(const gz::msgs::Quaternion &_msg);

    private: gz::transport::Node gz_node;

    
  };
}
#endif
