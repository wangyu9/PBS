#include "PickingPluginMG.h"

// Declare the single entity of the plugin
// This should be in all the plugin headers
PickingPluginMG  PickingPluginMGInstance;

PickingPluginMG& PickingPluginMG::GetReference()
{
	return PickingPluginMGInstance;
}


#include <DeformMulti.h>
#include <DeformARAP.h>
#include "DeformMultiPhys.h"
void PickingPluginMG::selection_changed()
{
	if (!PickingPluginBase::m_enable_mouse_response)
		return;
	// do nothing
}

void PickingPluginMG::selected_vertices_moved()
{
	if (!PickingPluginBase::m_enable_mouse_response)
		return;
	DeformMultiUI::GetReference().getConstraintFromPickingPlugin();
	DeformARAPUI::GetReference().getConstraintFromPickingPlugin();
	DeformMultiPhysUI::GetReference().getConstraintFromPickingPlugin();
}

void PickingPluginMG::constraint_changed()
{
	if (!PickingPluginBase::m_enable_mouse_response)
		return;
	DeformMultiUI::GetReference().getConstraintFromPickingPlugin();
	DeformARAPUI::GetReference().getConstraintFromPickingPlugin();
}