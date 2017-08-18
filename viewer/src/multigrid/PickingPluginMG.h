#pragma once

#include <viewer/PickingPluginBase.h>


class PickingPluginMG : public PickingPluginBase
{
public:

	// External Call
	const Eigen::MatrixXd& GetConstrainedPos()
	{
		return m_constrained_vertex_positions;
	}

	static PickingPluginMG& GetReference();

	virtual void selection_changed();
	virtual void selected_vertices_moved();
	virtual void constraint_changed();//

};