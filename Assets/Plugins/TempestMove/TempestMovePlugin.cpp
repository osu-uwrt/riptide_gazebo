#ifndef _TEMPEST_MOVE_PLUGIN_
#define _TEMPEST_MOVE_PLUGIN_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo{
    class TempestMovePlugin : public ModelPlugin{
        public: TempestMovePlugin() : ModelPlugin(){
            printf("This is the tempest move plugin!")
        }

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/){
            this->model = _parent;

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this))
        }

        public: void OnUpdate(){

        }
    }
}

#endif