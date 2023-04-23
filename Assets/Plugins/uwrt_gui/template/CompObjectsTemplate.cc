#include <gz/common/Util.hh>
#include <gz/common/Console.hh>
#include "CompObjectsPlugin.hh"

using namespace gz;
using namespace sim;


/////////////////////////////////////////////////
//StartSdfDef




/////////////////////////////////////////////////
//Dictionary
std::string gz::sim::getCompObjectshape(const CompObjectshape &_type)
{
  switch(_type)
  {
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
  //StartGetCompObject


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