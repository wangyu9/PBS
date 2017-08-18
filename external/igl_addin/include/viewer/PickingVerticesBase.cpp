#include "PickingVerticesBase.h"

VerticesBlock::VerticesBlock(const Eigen::VectorXi& indices, const Eigen::MatrixXd& pos)
{
	this->indices = indices;
	this->pos = pos;
}

void VerticesBlock::set_pos(const Eigen::MatrixXd& pos)
{
	//if (pos.rows() != this->pos.rows() || pos.cols() != this->pos.cols())
	//	return false;
	this->pos = pos;
	return;
}

void VerticesBlock::set_color(const Eigen::MatrixXd& color)
{
	this->color = color;
}

const Eigen::MatrixXd& VerticesBlock::get_pos() const
{
	return pos;
}
const Eigen::MatrixXd& VerticesBlock::get_color() const
{
	return color;
}