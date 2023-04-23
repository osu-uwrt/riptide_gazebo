#ifndef GZ_SIM_CUSTOM_OBJECTS_GUI_HH_
#define GZ_SIM_CUSTOM_OBJECTS_GUI_HH_

#include <memory>

#include <gz/gui/Plugin.hh>

namespace gz
{
namespace sim
{
  class CompObjectsPluginPrivate;

  /// \brief Provides buttons for adding a box, sphere, or cylinder
  /// to the scene
  class CompObjectsPlugin : public gz::gui::Plugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: CompObjectsPlugin();

    /// \brief Destructor
    public: ~CompObjectsPlugin() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    /// \brief Callback in Qt thread when mode changes.
    /// \param[in] _mode New transform mode
    public slots: void OnMode(const QString &_mode);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<CompObjectsPluginPrivate> dataPtr;
  };
}
}

#endif