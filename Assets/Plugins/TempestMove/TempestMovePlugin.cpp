#ifndef _TEMPEST_MOVE_PLUGIN_HH_
#define _TEMPEST_MOVE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo{
    class TempestMovePlugin : public ModelPlugin{
        public: TempestMovePlugin() : ModelPlugin(){
            printf("This is the tempest move plugin!");
        }

        public: void OnUpdate(){

        }
    };
}

#endif