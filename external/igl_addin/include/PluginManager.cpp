// IGL Viewer - Copyright (c) 2013 ETH Zurich. All rights reserved.

// Class that handles loading of plugins.
// To add a new plugin DO NOT EDIT this file, see readme.txt

#include "PluginManager.h"

_PluginManager_*  __PluginManager_instance = 0;

_PluginManager_& PluginManager()
{
  if (!__PluginManager_instance)  __PluginManager_instance = new _PluginManager_();
  return *__PluginManager_instance;
}
