#ifndef PICKING_VERTICES_BASE_H
#define PICKING_VERTICES_BASE_H

#include <Eigen/Core>

class VerticesBlock
{
private:
	Eigen::VectorXi indices;
	Eigen::MatrixXd pos;
	Eigen::MatrixXd color;
public:
	VerticesBlock(const Eigen::VectorXi& indices, const Eigen::MatrixXd& pos);
	void set_pos(const Eigen::MatrixXd& pos);
	void set_color(const Eigen::MatrixXd& color);
	const Eigen::MatrixXd& get_pos() const;
	const Eigen::MatrixXd& get_color() const;
};

#endif // PICKING_VERTICES_BASE_H

