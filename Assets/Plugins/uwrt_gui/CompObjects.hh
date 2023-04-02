#ifndef GZ_SIM_CompObjects_HH_
#define GZ_SIM_CompObjects_HH_

#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>

#include <string>

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {

    /// \brief Enumeration of available primitive shape types
    enum class GZ_SIM_VISIBLE CompObjectshape
    {
axeSdf,
badgeSdf,
binBarrelSdf,
binPhoneSdf,
bootleggerSdf,
bootleggerTorpedoSdf,
cashSdf,
gmanSdf,
gmanTorpedoSdf,
tempestSdf,
tommyGunSdf,
      //StartNamedObjects
      // kGate,
      // kBouey,
      // kOctagon,
      // kTorpedo,
    };

    /// \brief Return an SDF string of one of the available primitive
    /// shape types
    /// \param[in] _type Type of shape to retrieve
    /// \return String containing SDF description of primitive shape
    /// Empty string if the _type is not supported.
    std::string GZ_SIM_VISIBLE
    getCompObjectshape(const CompObjectshape &_type);

    /// \brief Return an SDF string of one of the available primitive shape or
    /// light types.
    /// \param[in] _typeName Type name of the of shape or light to retrieve.
    /// Must be one of: box, sphere, cylinder, capsule, ellipsoid, directional,
    /// point, or spot.
    /// \return String containing SDF description of primitive shape or light.
    /// Empty string if the _typeName is invalid.
    std::string GZ_SIM_VISIBLE
    getCompObject(const std::string &_typeName);
    }
  }  // namespace sim
}  // namespace gz


#endif  // GZ_SIM_CompObjects_HH_