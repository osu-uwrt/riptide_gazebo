#include <gz/common/Util.hh>
#include <gz/common/Console.hh>
#include "CompObjects.hh"

using namespace gz;
using namespace sim;


/////////////////////////////////////////////////
constexpr const char * kNAMESdf = R"(<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="NAME">
    <static>true</static>
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      <must_be_base_link>1</must_be_base_link>
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://buoy/meshes/NAME.dae</uri>   //REPLACE WITH LOCATION OF .DAE
          </mesh>
        </geometry>
      </collision>
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry>
          <mesh>
            <uri>model://buoy/meshes/NAME.dae</uri>   //REPLACE WITH LOCATION OF .DAE
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>)";

/////////////////////////////////////////////////
constexpr const char * kBoxSdf = R"(<?xml version="1.0"?>
<sdf version="1.9">
  <model name="box">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="box_link">
      <inertial>
        <inertia>
          <ixx>0.16666</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.16666</iyy>
          <iyz>0</iyz>
          <izz>0.16666</izz>
        </inertia>
        <mass>1.0</mass>
      </inertial>
      <collision name="box_collision">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </collision>
      <visual name="box_visual">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>1 1 1 1</specular>
        </material>
     </visual>
    </link>
  </model>
</sdf>
)";

/////////////////////////////////////////////////
constexpr const char * kSphereSdf = R"(<?xml version="1.0"?>
<sdf version="1.9">
  <model name="sphere">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="sphere_link">
      <inertial>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.1</iyy>
          <iyz>0</iyz>
          <izz>0.1</izz>
        </inertia>
        <mass>1.0</mass>
      </inertial>
      <collision name="sphere_collision">
        <geometry>
          <sphere>
            <radius>0.5</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name="sphere_visual">
        <geometry>
          <sphere>
            <radius>0.5</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>1 1 1 1</specular>
        </material>
     </visual>
    </link>
  </model>
</sdf>
)";

/////////////////////////////////////////////////
constexpr const char * kCylinderSdf = R"(<?xml version="1.0"?>
<sdf version="1.9">
  <model name="cylinder">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="cylinder_link">
      <inertial>
        <inertia>
          <ixx>0.1458</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.1458</iyy>
          <iyz>0</iyz>
          <izz>0.125</izz>
        </inertia>
        <mass>1.0</mass>
      </inertial>
      <collision name="cylinder_collision">
        <geometry>
          <cylinder>
            <radius>0.5</radius>
            <length>1.0</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="cylinder_visual">
        <geometry>
          <cylinder>
            <radius>0.5</radius>
            <length>1.0</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>1 1 1 1</specular>
        </material>
     </visual>
    </link>
  </model>
</sdf>
)";


/////////////////////////////////////////////////
constexpr const char * kCapsuleSdf = R"(<?xml version="1.0"?>
<sdf version="1.9">
  <model name="capsule">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="capsule_link">
      <inertial>
        <inertia>
          <ixx>0.074154</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.074154</iyy>
          <iyz>0</iyz>
          <izz>0.018769</izz>
        </inertia>
        <mass>1.0</mass>
      </inertial>
      <collision name="capsule_collision">
        <geometry>
          <capsule>
            <radius>0.2</radius>
            <length>0.6</length>
          </capsule>
        </geometry>
      </collision>
      <visual name="capsule_visual">
        <geometry>
          <capsule>
            <radius>0.2</radius>
            <length>0.6</length>
          </capsule>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>1 1 1 1</specular>
        </material>
     </visual>
    </link>
  </model>
</sdf>
)";

/////////////////////////////////////////////////
constexpr const char *kEllipsoidSdf = R"(<?xml version="1.0"?>
<sdf version="1.9">
  <model name="ellipsoid">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="ellipsoid_link">
      <inertial>
        <inertia>
          <ixx>0.068</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.058</iyy>
          <iyz>0</iyz>
          <izz>0.026</izz>
        </inertia>
        <mass>1.0</mass>
      </inertial>
      <collision name="ellipsoid_collision">
        <geometry>
          <ellipsoid>
            <radii>0.2 0.3 0.5</radii>
          </ellipsoid>
        </geometry>
      </collision>
      <visual name="ellipsoid_visual">
        <geometry>
          <ellipsoid>
            <radii>0.2 0.3 0.5</radii>
          </ellipsoid>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>1 1 1 1</specular>
        </material>
     </visual>
    </link>
  </model>
</sdf>
)";

/////////////////////////////////////////////////
//Dictionary
std::string gz::sim::getCompObjectshape(const CompObjectshape &_type)
{
  switch(_type)
  {
    case CompObjectshape::kGate:
      return kBoxSdf;
    case CompObjectshape::kBouey:
      return kBoueySdf;
    case CompObjectshape::kOctagon:
      return kOctagonSdf;
    case CompObjectshape::kTorpedo:
      return kTorpedoSdf;
    default:
      return "";
  }
}


/////////////////////////////////////////////////
std::string gz::sim::getCompObject(const std::string &_typeName)
{
  std::string type = common::lowercase(_typeName);

  if (type == "gate")
    return getCompObjectshape(CompObjectshape::kBox);
  else if (type == "bouey")
    return getCompObjectshape(CompObjectshape::kSphere);
  else if (type == "octagon")
    return getCompObjectshape(CompObjectshape::kCylinder);
  else if (type == "torpedo")
    return getCompObjectshape(CompObjectshape::kCapsule);

  gzwarn << "Invalid model string " << type << "\n";
  gzwarn << "The valid options are:\n";
  gzwarn << " - gate\n";
  gzwarn << " - bouey\n";
  gzwarn << " - octagon\n";
  gzwarn << " - torpedo\n";
  return "";
}