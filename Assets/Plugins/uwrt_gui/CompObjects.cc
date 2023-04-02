#include <gz/common/Util.hh>
#include <gz/common/Console.hh>
#include "CompObjectsPlugin.hh"

using namespace gz;
using namespace sim;


/////////////////////////////////////////////////
constexpr const char * axeSdf = R"(<?xml version="1.0" ?> <sdf version="1.9"> <model name="axe"> <static>true</static> <link name="link"> <pose>0 0 0 0 0 0</pose> <must_be_base_link>1</must_be_base_link> <collision name="collision"> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/axe/model.dae</uri> </mesh> </geometry> </collision> <visual name="visual"> <cast_shadows>false</cast_shadows> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/axe/model.dae</uri> </mesh> </geometry> </visual> </link> </model> </sdf>)";
constexpr const char * badgeSdf = R"(<?xml version="1.0" ?> <sdf version="1.9"> <model name="badge"> <static>true</static> <link name="link"> <pose>0 0 0 0 0 0</pose> <must_be_base_link>1</must_be_base_link> <collision name="collision"> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/badge/model.dae</uri> </mesh> </geometry> </collision> <visual name="visual"> <cast_shadows>false</cast_shadows> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/badge/model.dae</uri> </mesh> </geometry> </visual> </link> </model> </sdf>)";
constexpr const char * binBarrelSdf = R"(<?xml version="1.0" ?> <sdf version="1.9"> <model name="binBarrel"> <static>true</static> <link name="link"> <pose>0 0 0 0 0 0</pose> <must_be_base_link>1</must_be_base_link> <collision name="collision"> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/binBarrel/model.dae</uri> </mesh> </geometry> </collision> <visual name="visual"> <cast_shadows>false</cast_shadows> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/binBarrel/model.dae</uri> </mesh> </geometry> </visual> </link> </model> </sdf>)";
constexpr const char * binPhoneSdf = R"(<?xml version="1.0" ?> <sdf version="1.9"> <model name="binPhone"> <static>true</static> <link name="link"> <pose>0 0 0 0 0 0</pose> <must_be_base_link>1</must_be_base_link> <collision name="collision"> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/binPhone/model.dae</uri> </mesh> </geometry> </collision> <visual name="visual"> <cast_shadows>false</cast_shadows> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/binPhone/model.dae</uri> </mesh> </geometry> </visual> </link> </model> </sdf>)";
constexpr const char * bootleggerSdf = R"(<?xml version="1.0" ?> <sdf version="1.9"> <model name="bootlegger"> <static>true</static> <link name="link"> <pose>0 0 0 0 0 0</pose> <must_be_base_link>1</must_be_base_link> <collision name="collision"> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/bootlegger/model.dae</uri> </mesh> </geometry> </collision> <visual name="visual"> <cast_shadows>false</cast_shadows> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/bootlegger/model.dae</uri> </mesh> </geometry> </visual> </link> </model> </sdf>)";
constexpr const char * bootleggerTorpedoSdf = R"(<?xml version="1.0" ?> <sdf version="1.9"> <model name="bootleggerTorpedo"> <static>true</static> <link name="link"> <pose>0 0 0 0 0 0</pose> <must_be_base_link>1</must_be_base_link> <collision name="collision"> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/bootleggerTorpedo/model.dae</uri> </mesh> </geometry> </collision> <visual name="visual"> <cast_shadows>false</cast_shadows> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/bootleggerTorpedo/model.dae</uri> </mesh> </geometry> </visual> </link> </model> </sdf>)";
constexpr const char * cashSdf = R"(<?xml version="1.0" ?> <sdf version="1.9"> <model name="cash"> <static>true</static> <link name="link"> <pose>0 0 0 0 0 0</pose> <must_be_base_link>1</must_be_base_link> <collision name="collision"> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/cash/model.dae</uri> </mesh> </geometry> </collision> <visual name="visual"> <cast_shadows>false</cast_shadows> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/cash/model.dae</uri> </mesh> </geometry> </visual> </link> </model> </sdf>)";
constexpr const char * gmanSdf = R"(<?xml version="1.0" ?> <sdf version="1.9"> <model name="gman"> <static>true</static> <link name="link"> <pose>0 0 0 0 0 0</pose> <must_be_base_link>1</must_be_base_link> <collision name="collision"> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/gman/model.dae</uri> </mesh> </geometry> </collision> <visual name="visual"> <cast_shadows>false</cast_shadows> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/gman/model.dae</uri> </mesh> </geometry> </visual> </link> </model> </sdf>)";
constexpr const char * gmanTorpedoSdf = R"(<?xml version="1.0" ?> <sdf version="1.9"> <model name="gmanTorpedo"> <static>true</static> <link name="link"> <pose>0 0 0 0 0 0</pose> <must_be_base_link>1</must_be_base_link> <collision name="collision"> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/gmanTorpedo/model.dae</uri> </mesh> </geometry> </collision> <visual name="visual"> <cast_shadows>false</cast_shadows> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/gmanTorpedo/model.dae</uri> </mesh> </geometry> </visual> </link> </model> </sdf>)";
constexpr const char * tempestSdf = R"(<?xml version="1.0" ?> <sdf version="1.9"> <model name="tempest"> <static>true</static> <link name="link"> <pose>0 0 0 0 0 0</pose> <must_be_base_link>1</must_be_base_link> <collision name="collision"> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/tempest/model.dae</uri> </mesh> </geometry> </collision> <visual name="visual"> <cast_shadows>false</cast_shadows> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/tempest/model.dae</uri> </mesh> </geometry> </visual> </link> </model> </sdf>)";
constexpr const char * tommyGunSdf = R"(<?xml version="1.0" ?> <sdf version="1.9"> <model name="tommyGun"> <static>true</static> <link name="link"> <pose>0 0 0 0 0 0</pose> <must_be_base_link>1</must_be_base_link> <collision name="collision"> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/tommyGun/model.dae</uri> </mesh> </geometry> </collision> <visual name="visual"> <cast_shadows>false</cast_shadows> <geometry> <mesh> <uri>model:../../../../riptide_gui/riptide_meshes/meshes/tommyGun/model.dae</uri> </mesh> </geometry> </visual> </link> </model> </sdf>)";
//StartSdfDef




/////////////////////////////////////////////////
//Dictionary
std::string gz::sim::getCompObjectshape(const CompObjectshape &_type)
{
  switch(_type)
  {
case CompObjectshape::axeSdf: return axeSdf;
case CompObjectshape::badgeSdf: return badgeSdf;
case CompObjectshape::binBarrelSdf: return binBarrelSdf;
case CompObjectshape::binPhoneSdf: return binPhoneSdf;
case CompObjectshape::bootleggerSdf: return bootleggerSdf;
case CompObjectshape::bootleggerTorpedoSdf: return bootleggerTorpedoSdf;
case CompObjectshape::cashSdf: return cashSdf;
case CompObjectshape::gmanSdf: return gmanSdf;
case CompObjectshape::gmanTorpedoSdf: return gmanTorpedoSdf;
case CompObjectshape::tempestSdf: return tempestSdf;
case CompObjectshape::tommyGunSdf: return tommyGunSdf;
    //StartCases


    
    // case CompObjectshape::kGate:
    //   return kBoxSdf;
    // case CompObjectshape::kBouey:
    //   return kBoueySdf;
    // case CompObjectshape::kOctagon:
    //   return kOctagonSdf;
    // case CompObjectshape::kTorpedo:
    //   return kTorpedoSdf;
    default:
      return "";
  }
}


/////////////////////////////////////////////////
std::string gz::sim::getCompObject(const std::string &_typeName)
{
  std::string type = common::lowercase(_typeName);
if(type=="axe") return getCompObjectshape(CompObjectshape::axeSdf);
if(type=="badge") return getCompObjectshape(CompObjectshape::badgeSdf);
if(type=="binBarrel") return getCompObjectshape(CompObjectshape::binBarrelSdf);
if(type=="binPhone") return getCompObjectshape(CompObjectshape::binPhoneSdf);
if(type=="bootlegger") return getCompObjectshape(CompObjectshape::bootleggerSdf);
if(type=="bootleggerTorpedo") return getCompObjectshape(CompObjectshape::bootleggerTorpedoSdf);
if(type=="cash") return getCompObjectshape(CompObjectshape::cashSdf);
if(type=="gman") return getCompObjectshape(CompObjectshape::gmanSdf);
if(type=="gmanTorpedo") return getCompObjectshape(CompObjectshape::gmanTorpedoSdf);
if(type=="tempest") return getCompObjectshape(CompObjectshape::tempestSdf);
if(type=="tommyGun") return getCompObjectshape(CompObjectshape::tommyGunSdf);
  //StartGetCompObject
  printf("Did not request valid object");


  // if (type == "gate")
  //   return getCompObjectshape(CompObjectshape::kBox);
  // else if (type == "bouey")
  //   return getCompObjectshape(CompObjectshape::kSphere);
  // else if (type == "octagon")
  //   return getCompObjectshape(CompObjectshape::kCylinder);
  // else if (type == "torpedo")
  //   return getCompObjectshape(CompObjectshape::kCapsule);

  // gzwarn << "Invalid model string " << type << "\n";
  // gzwarn << "The valid options are:\n";
  // gzwarn << " - gate\n";
  // gzwarn << " - bouey\n";
  // gzwarn << " - octagon\n";
  // gzwarn << " - torpedo\n";
  return "";
}