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

	// replace existing virtual functions.
	virtual void selection_changed();
	void selected_vertices_moved();
	void constraint_changed();
};