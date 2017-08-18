#ifndef PICKING_PLUGIN_H
#define PICKING_PLUGIN_H

#include <viewer/PickingPluginBase.h>


class PickingPlugin : public PickingPluginBase
{
public:

	// External Call
	const Eigen::MatrixXd& GetConstrainedPos()
	{
		return m_constrained_vertex_positions;
	}

	static PickingPlugin& GetReference();

	
};

#endif