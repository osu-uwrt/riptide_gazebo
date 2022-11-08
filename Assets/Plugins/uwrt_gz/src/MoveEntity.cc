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

// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <iostream>
#include <memory>
#include <gz/common/Console.hh>
#include <gz/sim/components/Pose.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "MoveEntity.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    move_entity::MoveEntity,
    gz::sim::System,
    move_entity::MoveEntity::ISystemPostUpdate,
    move_entity::MoveEntity::ISystemConfigure,
    move_entity::MoveEntity::ISystemPreUpdate)

using namespace move_entity;

// Here we implement the PostUpdate function, which is called at every
// iteration.
void MoveEntity::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &/*_ecm*/){
}

void MoveEntity::PositionSubCallback(const gz::msgs::Vector3d &_msg){
  //update position
  current_position = _msg;
}

void MoveEntity::OrentationSubCallback(const gz::msgs::Quaternion &_msg){
  //update orentation
  current_orentation = _msg;
}

void MoveEntity::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &/*_ecm*/,
    gz::sim::EventManager &/*_eventMgr*/)
{
  this->current_position.set_x(0);
  current_position.set_y(0);
  current_position.set_z(0);

  this->entity = _entity;

  //configure ros2 subscriber
  std::string position_sub_topic = "/bridge/tempest/position"; // the topic to subcribe to get position
  std::string orentation_sub_topic = "/bridge/tempest/orientation"; // the topic to subcribe to get position

  //create sub callback pointer
  void (MoveEntity::*position_cb)(const gz::msgs::Vector3d&);
  position_cb = &MoveEntity::PositionSubCallback;
  
  void (MoveEntity::*orentation_cb)(const gz::msgs::Quaternion&);
  orentation_cb = &MoveEntity::OrentationSubCallback;

  //subsrcibe to the topics
  if(!gz_node.Subscribe(position_sub_topic, position_cb, this)){
    std::cerr << "Error creating subscriber to " << position_sub_topic << std::endl;
  } else {
    std::cout << "Successfully subscribed to " << position_sub_topic << std::endl;
  }

  // //sub to orentation
  if(!gz_node.Subscribe(orentation_sub_topic, orentation_cb, this)){
    std::cerr << "Error creating subscriber to " << orentation_sub_topic << std::endl;
  } else {
    std::cout << "Successfully subscribed to " << orentation_sub_topic << std::endl;
  }
}

void MoveEntity::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
    gz::math::Pose3d targetPose = gz::math::Pose3d(current_position.x(),current_position.y(),current_position.z(),current_orentation.x(),current_orentation.y(),current_orentation.z(), current_orentation.w());

    auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->entity);
    *poseComp = gz::sim::v7::components::Pose(targetPose);

    _ecm.SetChanged(this->entity, gz::sim::components::Pose::typeId, gz::sim::ComponentState::OneTimeChange);
};



