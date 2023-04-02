
#include "CompObjectsPlugin.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <algorithm>
#include <iostream>
#include <string>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>

#include <gz/sim/Primitives.hh>

namespace gz::sim
{
  class CompObjectsPluginPrivate
  {
  };
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
CompObjectsPlugin::CompObjectsPlugin()
  : gz::gui::Plugin(),
  dataPtr(std::make_unique<CompObjectsPluginPrivate>())
{
}

/////////////////////////////////////////////////
CompObjectsPlugin::~CompObjectsPlugin() = default;

/////////////////////////////////////////////////
void CompObjectsPlugin::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Competition Objects";

  // For CompObjectsPlugin requests
  gz::gui::App()->findChild
    <gz::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
void CompObjectsPlugin::OnMode(const QString &_mode)
{
  std::string modelSdfString = _mode.toStdString();
  modelSdfString = getPrimitive(modelSdfString);

  if (!modelSdfString.empty())
  {
    gz::gui::events::SpawnFromDescription event(modelSdfString);
    gz::gui::App()->sendEvent(
        gz::gui::App()->findChild<gz::gui::MainWindow *>(),
        &event);
  }
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::CompObjectsPlugin,
                    gz::gui::Plugin)