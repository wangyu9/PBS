#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>

class LinearSolver {
public:
	virtual bool analyzePattern(const Eigen::SparseMatrix<double>& A) = 0;
	virtual bool factorize(const Eigen::SparseMatrix<double>& A) = 0;
	virtual bool solve(const Eigen::MatrixXd& b, Eigen::MatrixXd& x) = 0;
};

class EigenSolverLLT: public LinearSolver {
public:
	Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> llt;
	bool analyzePattern(const Eigen::SparseMatrix<double>& A);
	bool factorize(const Eigen::SparseMatrix<double>& A);
	bool solve(const Eigen::MatrixXd& b, Eigen::MatrixXd& x);
};

class FiniteElementSolverBase {

public:

	FiniteElementSolverBase();
	bool init(const Eigen::MatrixXd& V, const Eigen::MatrixXi& TF);
	void update();

	void getV3d(Eigen::MatrixXd& VV);

	void setConstraint(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos);

private:

	void compute_gradient(Eigen::MatrixXd& gradient) const;
	void compute_hessian(Eigen::SparseMatrix<double>& hessian) const;

	void compute_gradient_per_element(const Eigen::MatrixXd& ele0, const Eigen::MatrixXd& ele, Eigen::MatrixXd& gradient) const;

	void compute_gradient_per_element_in_matrix_from(const Eigen::MatrixXd& ele0, const Eigen::MatrixXd& ele, Eigen::MatrixXd& gradient) const;

	Eigen::MatrixXd piola_kirchhoff_stress_tensor(const Eigen::MatrixXd& F) const;

	//virtual void compute_hessian_per_element(const Eigen::MatrixXd& ele0, const Eigen::MatrixXd& ele, Eigen::MatrixXd& hessian) const = 0;
	//virtual void compute_gradient_per_element(const Eigen::MatrixXd& F, Eigen::MatrixXd& gradient) const = 0;


	bool has_init;
	int dim;

	Eigen::MatrixXi TF;
	Eigen::MatrixXd V0;//Initial vertices positions
	Eigen::MatrixXd V;//Current vertices positions

	Eigen::MatrixXd Var0;//Initial values of Varibles.
	Eigen::MatrixXd Var;

	Eigen::MatrixXd Var_last;
	Eigen::MatrixXd Var_last_last;

	Eigen::VectorXi known;
	Eigen::MatrixXd knownPos;

	bool should_refactor;


	EigenSolverLLT linearSolver;
};



class GeometricElasticSolver: public FiniteElementSolverBase
{
public:
	//virtual void compute_gradient_per_element(const Eigen::MatrixXd& F, Eigen::MatrixXd& gradient) const;
//	virtual void compute_hessian_per_element(const Eigen::MatrixXd& ele0, const Eigen::MatrixXd& ele, Eigen::MatrixXd& hessian) const;
};