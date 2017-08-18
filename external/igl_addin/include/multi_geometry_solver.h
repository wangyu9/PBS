#pragma once

#include <geometrySolver.h>
#include <vector>

class SubspaceInfo {
public:
	Eigen::VectorXi b;
	Eigen::MatrixXd bases;
	Eigen::MatrixXd var;
	Eigen::VectorXi rotation_group;

	Eigen::MatrixXd bases_bypass;

	Eigen::SparseMatrix<double> bases_sparse;
	Eigen::SparseMatrix<double> bases_bypass_sparse;
};

void generate_subspace_info(const Eigen::MatrixXd& V, const Eigen::MatrixXi& TF, const Eigen::VectorXi& b0, std::vector<SubspaceInfo*>& infos);

class MultiGeometrySolver {
private:
	const int numLayers;
	int dim;
	std::vector<GeometrySolver*> solvers;
	std::vector<SubspaceInfo> subspaceInfos;
	bool has_inited;
public:
	MultiGeometrySolver();// (int N);
	~MultiGeometrySolver();
	void init(const Eigen::MatrixXd& V, const Eigen::MatrixXi& TF, const std::vector<SubspaceInfo*> infos);
	void update(int update_id=0);
	void setConstraint(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos);

	void getV(Eigen::MatrixXd& VV) const ;// this could have 2D or 3D columns
	void getV(int layer, Eigen::MatrixXd& VV) const;
	
	void getV3d(Eigen::MatrixXd& VV) const;
	void getV3d(int layer, Eigen::MatrixXd& VV) const;

	int getNumLayer() const { return numLayers; }

	void update00();
	void update01(int start_level);
	void update02();
	void update03();
	void update04();
};
