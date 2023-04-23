#ifndef MODEL_PLUGIN_MOVEENTITY_HH_
#define MODEL_PLUGIN_MOVEENTITY_HH_

#include <gz/sim/System.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <gz/math.hh>

namespace move_entity
{
  class MoveEntity:
    public gz::sim::System,
    public gz::sim::ISystemPostUpdate,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    private: gz::sim::Entity entity;
    private: gz::transport::Node gz_node;

    //these are stored seperately because of issues the gz::msgs::pose - fix later?
    public: 
    gz::msgs::Vector3d  current_position;
    gz::msgs::Quaternion current_orentation;

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
  };
}
#endif
