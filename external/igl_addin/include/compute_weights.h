#ifndef COMPUTE_WEIGHTS_H
#define COMPUTE_WEIGHTS_H

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <igl/igl_inline.h>
#include <vector>

template<
	typename DerivedV,
	typename DerivedT,
	typename SType,
	typename DerivedW >
	bool bilaplacian_coordinates(
	const Eigen::PlainObjectBase<DerivedV>& V,
	const Eigen::PlainObjectBase<DerivedT>& T,
	const std::vector<std::vector<SType> > & S,
	Eigen::PlainObjectBase<DerivedW>& W);

template<
	typename DerivedV,
	typename DerivedT,
	typename Derivedb,
	typename DerivedW >
	bool bilaplacian_coordinates(
		const Eigen::PlainObjectBase<DerivedV>& V,
		const Eigen::PlainObjectBase<DerivedT>& T,
		const Eigen::PlainObjectBase<Derivedb>& b,
		Eigen::PlainObjectBase<DerivedW>& W);

template<
	typename DerivedW,
	typename DerivedV,
	typename DerivedT,
	typename SType >
	 Eigen::PlainObjectBase<DerivedW> bilaplacian_coordinates(
		const Eigen::PlainObjectBase<DerivedV>& V,
		const Eigen::PlainObjectBase<DerivedT>& T,
		const std::vector<std::vector<SType> > & S)
{
	Eigen::PlainObjectBase<DerivedW> W;
	bilaplacian_coordinates(V, T, S, W);
	return W;
}

template<
	typename DerivedW,
	typename DerivedV,
	typename DerivedT,
	typename Derivedb >
	 Eigen::PlainObjectBase<DerivedW> bilaplacian_coordinates(
		const Eigen::PlainObjectBase<DerivedV>& V,
		const Eigen::PlainObjectBase<DerivedT>& T,
		const Eigen::PlainObjectBase<Derivedb>& b)
{
	Eigen::PlainObjectBase<DerivedW> W;
	bilaplacian_coordinates(V, T, b, W);
	return W;
}

template<
	typename Derivedb,
	typename DerivedW >
	Eigen::SparseMatrix<DerivedW> reorder_matrix(
		const Eigen::PlainObjectBase<Derivedb>& b)
{
	Eigen::PlainObjectBase<DerivedW> W;
	reorder_matrix(b, W);
	return W;
}

template<
	typename Derivedb,
	typename DerivedW >
	void reorder_matrix(
		const Eigen::PlainObjectBase<Derivedb>& b,
		Eigen::SparseMatrix<DerivedW>& W);

//bool bilaplacian_coordinates(const Eigen::MatrixXd& V, const Eigen::MatrixXd& TF, const)
//{
//
//}













#endif /*COMPUTE_WEIGHTS_H*/