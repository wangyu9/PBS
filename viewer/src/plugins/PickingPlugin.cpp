#include "PickingPlugin.h"

// Declare the single entity of the plugin
// This should be in all the plugin headers
PickingPlugin  PickingPluginInstance;
PickingPlugin&  PickingPluginInstance_() { return PickingPluginInstance; }

PickingPlugin& PickingPlugin::GetReference()
{
	return PickingPluginInstance;
}
