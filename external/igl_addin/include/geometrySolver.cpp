#include "geometrySolver.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/SparseExtra>

#ifdef USING_IGL_HEADER_ONLY_MODE
#define IGL_HEADER_ONLY 
#endif


#include <igl/polar_svd3x3.h> // new path //old path in libigl #include <igl/svd3x3/polar_svd3x3.h>
#include <cotangent.h>//new igl removed this, has it in igl addin


#ifdef USE_ALEC_STYLE_ARAP_IMPLEMENTATION
#include <igl/cotmatrix.h>
#include <igl/arap_rhs.h>
#include <igl/columnize.h>
#include <igl/covariance_scatter_matrix.h>
#endif

#include <igl/fit_rotations.h> //#include <igl/svd3x3/fit_rotations.h>
#include <igl/cat.h>

#include <igl/polar_svd3x3.h> //#include <igl/svd3x3/polar_svd3x3.h>
#include <igl/repmat.h>
#include <igl/verbose.h>
//#include <igl/polar_dec.h>
#include <igl/polar_svd.h>


#define  DO_TIMING
#undef DO_TIMING

#ifdef DO_TIMING
	#include "TimerWrapper.h"
#endif


GeometrySolver::GeometrySolver()
{
	//set to unconstrained
	S.resize(V.rows());
	S.setConstant(-1);
}	

void GeometrySolver::init(const ARAP_para& para)
{
	if (para.with_subspace_ARAP)
	{
		//ARAP_rotation_group = m_preview3d->vertex_group.cast<int>();
		if (para.dim == 2)
		{
			set_mesh(para.mesh.F, para.mesh.V);
		}
		else
		{
			set_mesh(para.mesh.T, para.mesh.V);
		}
		if (para.use_sparse_weights)
		{
			setSubspaceWeights(para.subspace_M_sparse, para.rotation_group, para.var);
		} 
		else
		{
			setSubspaceWeights(para.subspace_M.sparseView(0.05), para.rotation_group, para.var);
		}		
	}
	else
	{
		if (para.dim == 2)
		{
			set_mesh(para.mesh.F, para.mesh.V);//This is not clear yet!
		}
		else
		{//dim ==3
			set_mesh(para.mesh.T, para.mesh.V);//This is not clear yet!
		}
	}

	setPerGroupWeight(para.per_group_weight); // this not only work for subspace, but also non-subspace

	if (para.with_dynamics)
	{

		precomputation_with_dynamics(para.phys_para.time_step, para.phys_para.mu, para.phys_para.rho);
	}
	else
	{
		precomputation_no_dynamics();
	}
}

void GeometrySolver::set_mesh(const Eigen::MatrixXi& TT, const Eigen::MatrixXd& VV, const Eigen::MatrixXd& VV0)
{
	assert(VV.rows()==VV0.rows());

	T = TT;
	V = VV;
	V0 = VV0;

	Vel.resize(V.rows(),V.cols());
	Vel.setZero();

	S.resize(V.rows());
	S.setConstant(-1);


	quadSolver.Init();

	bool tet_is_one_index = false;
	if (tet_is_one_index)
	{
		printf("Mannally substruct 1 for all tets' vertex indexing.\n");
		T = T - Eigen::MatrixXi::Ones(T.rows(),T.cols());
	}

	with_subspace_weights = false;
}

bool GeometrySolver::need_restart_when_constrant_change()
{
	// geometrySolver do not need to restart when contrained vertex index changed
	return true;
}

void GeometrySolver::setConstraints(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos)
{
	const Eigen::MatrixXd Meq(0, Varibles.rows());
	const Eigen::MatrixXd Peq(0, Varibles.cols());
	setConstraints(known, knownPos, Meq, Peq);
}

void GeometrySolver::setConstraints(const Eigen::VectorXi& known, const Eigen::MatrixXd& knownPos, const Eigen::MatrixXd& Meq, const Eigen::MatrixXd& Peq)
{
	// TODO: current implementation is stupid. improve later.

	Eigen::VectorXi SS(Varibles.rows());
	Eigen::MatrixXd SV = Eigen::MatrixXd::Zero(SS.rows(), Varibles.cols());
	SS.setConstant(-1);
	for (int i = 0; i < known.rows(); i++)
	{
		int idx = known(i);
		if (idx >= SS.rows())
			std::cerr << "GeometrySolver::setConstraints(): idx exceeds variable size!" << std::endl;		
		SS(idx) = 1;

		for (int d = 0; d < Varibles.cols(); d++)
			SV(idx, d) = knownPos(i, d);
	}
	setConstraintsIndicator(SS, SV, Meq, Peq);
}

void GeometrySolver::setConstraintsIndicator(const Eigen::VectorXi& SS, const Eigen::MatrixXd& Var_new, const Eigen::MatrixXd& Meq, const Eigen::MatrixXd& Peq)
{
	assert(SS.rows() == Var_new.rows() );

	bool constrained_vertex_index_changed = false;//!(S.rows()==SS.rows());
	if (S.rows()!=SS.rows())
	{
		constrained_vertex_index_changed = true;
	}
	else
	{
		for (int i=0; i<SS.rows(); i++)
		{
			if (S(i)!=SS(i))
			{
				constrained_vertex_index_changed = true;
			}
		}
	}
	S = SS;

	if (constrained_vertex_index_changed)
	{
		printf("Constraint is set!\n");
		Varibles = Var_new;
		Varibles_last = Varibles_last_last = Varibles; // Important: only do this when constrained variables are changed, if not the velocities in the update will be incorrect!

	}

	if (Varibles.rows()!=Var_new.rows())
	{
		printf("Error: GeometrySolver::setConstraints(): this should never happens!\n");
	}
	else
	{
		for (int i=0; i<S.rows(); i++)
		{
			if(S(i)>=0)//constrained vertices
			{
				Varibles.row(i) = Var_new.row(i);
			}
		}
	}

	quadSolver.setAeqKnownAndY(S,Varibles,Meq,Peq);

}

template <typename Scalar>
inline void Rotation_Group_Project(
	const Eigen::VectorXi RG,
	Eigen::SparseMatrix<Scalar>& RGP)
{
	using namespace igl;
	using namespace std;
	using namespace Eigen;

	int num_rots = RG.maxCoeff() + 1;
	printf("Number of rotation group: %d", num_rots);

	//DynamicSparseMatrix<Scalar> 
	RGP.resize(num_rots, RG.rows());

	RGP.reserve(RG.rows());

	vector<Triplet<Scalar> > IJV;
	IJV.reserve(RG.rows());

	for (int i = 0; i < RG.rows(); i++)
	{
		IJV.push_back(Triplet<Scalar>(RG(i), i, 1));
	}

	RGP.setFromTriplets(IJV.begin(),IJV.end());
}

// This is same as Arap_L_and_K_reduced, by allowing one more scale per rotation group
template <typename DerivedV, typename DerivedF, typename DerivedS, typename Scalar>
inline void Asap_L_and_K_reduced(
	const Eigen::PlainObjectBase<DerivedV> & V, 
	const Eigen::PlainObjectBase<DerivedF> & F,
	Eigen::SparseMatrix<Scalar>& L,
	Eigen::SparseMatrix<Scalar>& K_reduced,
	Eigen::PlainObjectBase<DerivedS> & Scale_Weight_reduced,
	Eigen::VectorXi RG,//RG is rotation group
	Eigen::VectorXd GW)//GW is per_group_weight
{

	using namespace igl;
	using namespace Eigen;
	Eigen::DynamicSparseMatrix<double> foo;

	int dim = V.cols();

	int nr = F.rows();//assert(dim==3);
	assert(RG.rows()==nr);
	int num_rots = RG.maxCoeff()+1;
	if (RG.minCoeff() != 0)
		printf("Warning: the rotation group index does not start from 0, it starts from %d to %d.\n", RG.minCoeff(), RG.maxCoeff());//should be 0 indexed	
	printf("Number of rotation group: %d",num_rots);
	Eigen::VectorXi rg_num;
	rg_num.resize(num_rots);
	rg_num.setZero();

	Scale_Weight_reduced.resize(num_rots,1);
	Scale_Weight_reduced.setZero();

	DynamicSparseMatrix<Scalar, RowMajor> dyn_L (V.rows(), V.rows());
	DynamicSparseMatrix<Scalar> dyn_K_reduced (dim*num_rots, V.rows());


	Matrix<int,Dynamic,2> edges;
	int simplex_size = F.cols();
	// 3 for triangles, 4 for tets
	assert(simplex_size == 3 || simplex_size == 4);
	if(simplex_size == 3)
	{
		// This is important! it could decrease the comptuation time by a factor of 2
		// Laplacian for a closed 2d manifold mesh will have on average 7 entries per
		// row
		dyn_L.reserve(7*V.rows());
		edges.resize(3,2);
		edges << 
			1,2,
			2,0,
			0,1;
	}else if(simplex_size == 4)
	{
		dyn_L.reserve(17*V.rows());
		edges.resize(6,2);
		edges << 
			1,2,
			2,0,
			0,1,
			3,0,
			3,1,
			3,2;
	}else
	{
		return;
	}
	// Gather cotangents
	Matrix<Scalar,Dynamic,Dynamic> C;
	cotangent(V,F,C);
	C = C*2;// because cotangent gives half cotangents 

	if (GW.rows()!=(RG.maxCoeff()+1) )
	{
		if (GW.rows() != 0)
			printf("Warning: Per Group Weight has the incorrect size, treat all rotation group the same!\n");
	}
	else
	{
		for (int i = 0; i < C.rows(); i++)
		{
			C.row(i) *= GW(RG(i));
		}
	}

	//SparseMatrix<Scalar> V_sparse_trans = (V.transpose()).sparseView();
	MatrixXd V_trans = V.transpose();

	// Loop over triangles
	for(int i = 0; i < F.rows(); i++)
	{

		int r = RG(i);
		assert(r<num_rots);
		rg_num(r) = rg_num(r)+1;

		// loop over edges of element
		for(int e = 0;e<edges.rows();e++)
		{
			int source = F(i,edges(e,0));
			int dest = F(i,edges(e,1));
			//NOTE: the following is different from that of libigl, wangyu
			dyn_L.coeffRef(source,dest) += -C(i,e);
			dyn_L.coeffRef(dest,source) += -C(i,e);
			dyn_L.coeffRef(source,source) += C(i,e);
			dyn_L.coeffRef(dest,dest) += C(i,e);

			// Do sparse matrix multiplication manually
			for (int c=0; c<dim; c++)
			{
				// CODE without RG
				//dyn_K.coeffRef(i*dim+c,source) += V_trans(c,source)*C(i,e)
				//	+ V_trans(c,dest)*(-C(i,e));
				//dyn_K.coeffRef(i*dim+c,dest) += V_trans(c,source)*(-C(i,e))
				//	+ V_trans(c,dest)*C(i,e);

				// CODE with RG
				dyn_K_reduced.coeffRef(r*dim+c,source) += V_trans(c,source)*C(i,e)
					+ V_trans(c,dest)*(-C(i,e));
				dyn_K_reduced.coeffRef(r*dim+c,dest) += V_trans(c,source)*(-C(i,e))
					+ V_trans(c,dest)*C(i,e);

				Scale_Weight_reduced(r) += (V_trans(c,source)-V_trans(c,dest))
					* (V_trans(c,source)-V_trans(c,dest)) * C(i,e);
			}
		}

	}
	// Corner indices of this triangle
	L = SparseMatrix<Scalar>(dyn_L);
	K_reduced = SparseMatrix<Scalar>(dyn_K_reduced);
}


// This could be used for both arap and asap, not different.
template <typename DerivedV, typename DerivedF, typename Scalar>
inline void ASRap_L_and_K(
	const Eigen::PlainObjectBase<DerivedV> & V,
	const Eigen::PlainObjectBase<DerivedF> & F,
	Eigen::SparseMatrix<Scalar>& L,
	Eigen::SparseMatrix<Scalar>& K_right)
{
	using namespace igl;
	using namespace Eigen;
	Eigen::DynamicSparseMatrix<double> foo;

	int dim = V.cols();

	DynamicSparseMatrix<Scalar, RowMajor> dyn_L(V.rows(), V.rows());
	DynamicSparseMatrix<Scalar> dyn_K_right(F.rows()*dim, V.rows());

	Matrix<int, Dynamic, 2> edges;
	int simplex_size = F.cols();
	// 3 for triangles, 4 for tets
	assert(simplex_size == 3 || simplex_size == 4);
	if (simplex_size == 3)
	{
		// This is important! it could decrease the comptuation time by a factor of 2
		// Laplacian for a closed 2d manifold mesh will have on average 7 entries per
		// row
		dyn_L.reserve(7 * V.rows());
		edges.resize(3, 2);
		edges <<
			1, 2,
			2, 0,
			0, 1;
	}
	else if (simplex_size == 4)
	{
		dyn_L.reserve(17 * V.rows());
		edges.resize(6, 2);
		edges <<
			1, 2,
			2, 0,
			0, 1,
			3, 0,
			3, 1,
			3, 2;
	}
	else
	{
		return;
	}
	// Gather cotangents
	Matrix<Scalar, Dynamic, Dynamic> C;
	cotangent(V, F, C);
	C = C * 2;

	// Loop over triangles
	for (int i = 0; i < F.rows(); i++)
	{
		// loop over edges of element
		for (int e = 0; e < edges.rows(); e++)
		{
			int source = F(i, edges(e, 0));
			int dest = F(i, edges(e, 1));
			//NOTE: the following is different from that of libigl, wangyu
			dyn_L.coeffRef(source, dest) += -C(i, e);
			dyn_L.coeffRef(dest, source) += -C(i, e);
			dyn_L.coeffRef(source, source) += C(i, e);
			dyn_L.coeffRef(dest, dest) += C(i, e);

			//dyn_K_right.coeffRef(i*V.rows() + source, dest) += -C(i, e);
			//dyn_K_right.coeffRef(i*V.rows() + dest, source) += -C(i, e);
			//dyn_K_right.coeffRef(i*V.rows() + source, source) += C(i, e);
			//dyn_K_right.coeffRef(i*V.rows() + dest, dest) += C(i, e);

			for (int c = 0; c < dim; c++)
			{
				// CODE without RG
				dyn_K.coeffRef(i*dim+c,source) += V_trans(c,source)*C(i,e)
					+ V_trans(c,dest)*(-C(i,e));
				dyn_K.coeffRef(i*dim+c,dest) += V_trans(c,source)*(-C(i,e))
					+ V_trans(c,dest)*C(i,e);

				// CODE with RG
				//dyn_K_reduced.coeffRef(r*dim + c, source) += V_trans(c, source)*C(i, e)
				//	+ V_trans(c, dest)*(-C(i, e));
				//dyn_K_reduced.coeffRef(r*dim + c, dest) += V_trans(c, source)*(-C(i, e))
				//	+ V_trans(c, dest)*C(i, e);

				Scale_Weight_reduced(r) += (V_trans(c, source) - V_trans(c, dest))
					* (V_trans(c, source) - V_trans(c, dest)) * C(i, e);
			}
		}
	}
	// Corner indices of this triangle
	L = SparseMatrix<Scalar>(dyn_L);
	K_right = SparseMatrix<Scalar>(dyn_K_right);
}


//This works for 2D also, but less efficient than Solve_for_Rotations_2D
template <typename DerivedS, typename DerivedD>
inline void Solve_for_Rotations(
	const Eigen::PlainObjectBase<DerivedS> & S,
	Eigen::PlainObjectBase<DerivedD> & R)
{
	using namespace igl;

	using namespace std;
	const int dim = S.cols();
	const int nr = S.rows()/dim;
	assert(nr * dim == S.rows());

	// resize output
	R.resize(dim,dim*nr); // hopefully no op (should be already allocated)

	//std::cout<<"S=["<<std::endl<<S<<std::endl<<"];"<<std::endl;
	//MatrixXd si(dim,dim);
	Eigen::Matrix<typename DerivedS::Scalar,3,3> si = Eigen::Matrix3d::Identity();
	// loop over number of rotations we're computing
	for(int r = 0;r<nr;r++)
	{
		// build this covariance matrix
		for(int i = 0;i<dim;i++)
		{
			for(int j = 0;j<dim;j++)
			{
				si(i,j) = S(r*dim+i,j);//Note: here is different from Alec's implementation! wangyu 
			}
		}
		// We implicitly have this in order to make use of svd3x3 for dim = 2
		if (dim==2)
		{
			si(2,2) = 1;// 
		}
		Eigen::Matrix<typename DerivedD::Scalar,3,3> ri;//here should always be 3 even for dim = 2
		Eigen::Matrix<typename DerivedD::Scalar,3,3> ti;
		//polar_dec(si,ri,ti);
		//polar_svd(si,ri,ti);
		polar_svd3x3(si, ri);
		assert(ri.determinant() >= 0);
#ifndef FIT_ROTATIONS_ALLOW_FLIPS
		// Check for reflection
		if(ri.determinant() < 0)
		{
			cerr<<"Error: Warning: flipping is wrong..."<<endl;
			assert(false && "This is wrong. Need to flip column in U and recompute R = U*V'");
			// flip sign of last row
			ri.row(dim) *= -1;//ri.row(2) *= -1;
			if (dim==2)
			{
				if (abs(ri(2,0))>0.001||abs(ri(2,1))>0.001)
				{
					printf("Using svd3x3 for dim=2 gets wrong result here.\n");
				}
			}
			
		}
		assert(ri.determinant() >= 0);
#endif  
		// Alec:Not sure why polar_dec computes transpose... 
		// wangyu:this is correct since polar_svd3x3 returns UV'=UQ, while R=Q'U'
		R.block(0,r*dim,dim,dim) = ri.block(0,0,dim,dim).transpose();
	}
}

template <typename DerivedS, typename DerivedD>
inline void Solve_for_Rotations_2D(
	const Eigen::PlainObjectBase<DerivedS> & S,
	Eigen::PlainObjectBase<DerivedD> & R)
{ 
	using namespace std;
	const int dim = S.cols();
	const int nr = S.rows()/dim;
	assert(dim == 2 && "_planar input should be 2D");
	assert(nr * dim == S.rows());

	// resize output
	R.resize(dim,dim*nr); // hopefully no op (should be already allocated)

	Eigen::Matrix<typename DerivedS::Scalar,2,2> si;
	// loop over number of rotations we're computing
	for(int r = 0;r<nr;r++)
	{
		// build this covariance matrix
		for(int i = 0;i<dim;i++)
		{
			for(int j = 0;j<dim;j++)
			{
				si(i,j) = S(r*dim+i,j);//Note: here is different from Alec's implementation! wangyu 
			}
		}
		typedef Eigen::Matrix<typename DerivedD::Scalar,2,2> Mat2;
		typedef Eigen::Matrix<typename DerivedD::Scalar,2,1> Vec2;
		Mat2 ri,ti,ui,vi;
		Vec2 _;
		igl::polar_svd(si,ri,ti,ui,_,vi);
#ifndef FIT_ROTATIONS_ALLOW_FLIPS
		// Check for reflection
		if(ri.determinant() < 0)
		{
			vi.col(1) *= -1.;
			ri = ui * vi.transpose();
		}
		assert(ri.determinant() >= 0);
#endif  

		// Not sure why polar_dec computes transpose...
		R.block(0,r*dim,2,2) = ri.transpose();
	}
}

void GeometrySolver::setSubspaceWeights(const Eigen::SparseMatrix<double>& m, const Eigen::VectorXi& rg, const Eigen::MatrixXd& var0)
{
	with_subspace_weights = true;
	M = m;
	rotation_group = rg;
	Varibles0 = var0;
	Varibles = Varibles0;
	Varibles_last = Varibles_last_last = Varibles;
}

int num_in_selection_copy_in_gs(const Eigen::VectorXi & S)
{
	int count = 0;
	for(int v = 0;v<S.rows(); v++)
	{
		if(S(v) >= 0)
		{
			count++;
		}
	}
	return count;
}

#include <igl/massmatrix.h>
template <typename DerivedV, typename DerivedF, typename Scalar>
inline void Mass_Dynamics(
	const Eigen::PlainObjectBase<DerivedV> & V,
	const Eigen::PlainObjectBase<DerivedF> & TF,
	double rho,
	Eigen::SparseMatrix<Scalar>& Mass)
{
	igl::massmatrix(V, TF, igl::MASSMATRIX_TYPE_BARYCENTRIC, Mass);
	Mass = Mass * rho;
}


void GeometrySolver::precomputation_no_dynamics()
{
	Eigen::SparseMatrix<double> Mass;
	Mass_Dynamics(V0, T, 1., Mass);
	precomputation(false, 0, 0, Mass);
}

void GeometrySolver::precomputation_with_dynamics(double dt, double mu, double rho)// The mass matrix used for dymanics simulation.);
{
	Eigen::SparseMatrix<double> Mass;
	Mass_Dynamics(V0, T, rho, Mass);
	precomputation(true, dt, mu, Mass);
}

const Eigen::MatrixXd GeometrySolver::getV() const
{
	if (with_subspace_weights)
	{
		return M * Varibles;
	} 
	else
	{
		return V;
	}
}

void GeometrySolver::setVar(const Eigen::MatrixXd& var)
{
	Varibles = var;
}

#include <eigen_helper.h>
//Constraints should be set before precomputation
void GeometrySolver::precomputation(bool wd, double dt, double mu, const Eigen::SparseMatrix<double>& Mass)
{

	printf("Pre-computation of ARAP, with %d vertices, %d handles (%d are active).\n", V.rows(), Varibles.rows(), num_in_selection_copy_in_gs(S) );

	// set dynamics
	{
		with_dynamics = wd;
		time_step = dt;
		this->mu = mu;
		this->Mass = Mass;
	}

	int dim;
	if (T.cols()==3)
	{
		dim = 2;
	} 
	else if(T.cols()==4)
	{
		dim = 3;

	}
	else
	{
		printf("Unknown Dimension for ARAP.\n");
		return;
	}

	if (V.rows()!=M.rows())
	{
		printf("Warning: GeometrySolver::precomputation() M is not set, making it identity!\n");
		M = Eigen::SparseMatrix<double>(V.rows(), V.rows());
		M.setIdentity();
	}
	if (T.rows()!=rotation_group.size())
	{
		printf("Warning: GeometrySolver::precomputation() Rotation Group is not set, making it natural group!\n");
		rotation_group = NaturalSeq(T.rows());
	}

	assert(T.cols()==4 || T.cols()==3);
	Eigen::SparseMatrix<double> L;
	//NOTE Q should be 1/2*L instead of L! Howver //min_quad_with_fixed is changed to 0.5 x'Ax

#ifdef USE_ALEC_STYLE_ARAP_IMPLEMENTATION

	igl::cotmatrix(V,T,L);
	// Note: Alec's L is the -L in the FAST paper
	//Q = (-0.5*L).eval();//eval returns a reference of the matrix to avoid copy
	Q = (-L).eval();//min_quad_with_fixed is changed to 0.5 x'Ax

	igl::ARAPEnergyType eff_energy = igl::ARAP_ENERGY_TYPE_ELEMENTS;

	//This K is different from what is in the FAST paper!
	igl::arap_rhs(V,T,eff_energy,K_alec);

	//CSM is dim*r by dim*n  (r=#tets for dim=3 and tet based arap energy)
	igl::covariance_scatter_matrix(V,T,eff_energy,CSM);

#else

#if 0
	if(false){
		int dim = 3;
		Eigen::SparseMatrix<double> CSM;
		//TimerWrapper timerWrapper0;
		//timerWrapper0.Tic();
		igl::covariance_scatter_matrix(V,T,igl::ARAP_ENERGY_TYPE_ELEMENTS,CSM);
		//timerWrapper0.Toc();
		//printf("Alec CSM time :%f\n",timerWrapper0.Duration());
		//timerWrapper0.Tic();
		Eigen::MatrixXd S = CSM * V.replicate(dim,1);
		//printf("CSM(%d,%d)\n",CSM.rows(),CSM.cols());
		//timerWrapper0.Toc();
		//printf("Alec S time :%f\n",timerWrapper0.Duration());
	}
#endif

	//Actually the following two functions could be merged into one
	//Arap_L_and_K_right(V,T,L,K_right);
	//Compute_K_from_K_right(K_right,V,dim,K_in_paper);
#ifdef DO_TIMING
	TimerWrapper timerWrapper;
	timerWrapper.Tic();
#endif

	//Arap_L_and_K_reduced(V,T,L,K_reduced,rotation_group);
	Asap_L_and_K_reduced(V,T,L,K_reduced,Scale_Weight_reduced,rotation_group,per_group_weight);// This could be used for Both ARAP and ASAP
	//ASRap_L_and_K(V, T, L, K_right);
	//Rotation_Group_Project(rotation_group, RGProject);

	K_reduced_hat = K_reduced*M;
	/*****************************/

	//For debug purpose, only one group, which works not too bad.
	printf("Rotation Group Size:(%d,%d)\n",rotation_group.rows(),rotation_group.cols());
	//std::cout<< rotation_group.block(0,0,2,1)<<std::endl;
	//rotation_group.resize(nr);
	//rotation_group.setZero();
	/*****************************/


	//int nr = T.rows();assert(dim==3);

	//assert(rotation_group.rows()==nr);
	//int num_rots = rotation_group.maxCoeff()+1;
	//assert(rotation_group.minCoeff()==0);//should be 0 indexed	
	//printf("Number of rotation group: %d",num_rots);
	//if(!with_subspace_weights)
	//{
	//	K_reduced = K_in_paper;
	//}else
	//{
	//	K_reduced.resize(dim*num_rots, V.rows());

	//	Eigen::VectorXi rg_num;
	//	rg_num.resize(num_rots);
	//	rg_num.setZero();

	//	for (int i=0; i<nr; i++)
	//	{
	//		int r = rotation_group(i);
	//		assert(r<num_rots);
	//		//r = 0;
	//		K_reduced.block(dim*r,0,dim,V.rows()) = 
	//			K_reduced.block(dim*r,0,dim,V.rows())+ 
	//			K_in_paper.block(dim*i,0,dim,V.rows());
	//		rg_num(r) = rg_num(r)+1;
	//	}
	//}


	////For debug purpose, only one group, which works not too bad.
	//printf("Rotation Group Size:(%d,%d)\n",rotation_group.rows(),rotation_group.cols());
	////std::cout<< rotation_group.block(0,0,2,1)<<std::endl;
	////rotation_group.resize(nr);
	////rotation_group.setZero();

#ifdef DO_TIMING
	timerWrapper.Toc();
	printf("ARAP L and K time:%f\n",timerWrapper.Duration());
#endif

	//Q = (0.5*L).eval();
	Eigen::SparseMatrix<double> Q = (L).eval(); //min_quad_with_fixed is changed to 0.5 x'Ax

#endif

	if(with_subspace_weights)
	{
		Q = M.transpose()*Q*M;

#ifdef USE_ALEC_STYLE_ARAP_IMPLEMENTATION
		//This is not clear yet...
		assert(false && "Do not support alec's implementation with weights yet");	
#else
		//K_in_paper = K_in_paper*M;
#endif
		//Varibles and Varibles0 should have been set in setSubspaceWeight
	}
	else
	{
		Varibles0 = V0;
		Varibles = V;
	}

	if (with_dynamics)
	{
		Q = mu * Q;
		Mass_reduced = M.transpose()*Mass*M;
		Q = Q + Mass_reduced / (time_step*time_step);
	}

	if (true)// with external forces
	{
		Eigen::MatrixXd tmp = Mass.diagonal().transpose();
		// Testing code, Mass.diagonal() returns a n by 1 vector, stacking all diagonal elements of the matrix.

		Eigen::MatrixXd ones = Eigen::MatrixXd::Ones(1,Mass.rows());
		Eigen::SparseMatrix<double> sparse_ConstantMassM = ones.sparseView();
		sparse_ConstantMassM = sparse_ConstantMassM * Mass * M;
		ConstantMassM = sparse_ConstantMassM.toDense();
	}

	//Eigen::SparseMatrix<double> Aeq;// Zero Constraints
	quadSolver.precompute(Q);

	//igl::min_quad_with_fixed_precompute( 
	//	Q,b,Eigen::SparseMatrix<double>(),true,precomp_data);

}

void linear_multiply( const Eigen::SparseMatrix<double>& A, const Eigen::SparseMatrix<double>& b, Eigen::MatrixXd& m)
{
	Eigen::SparseMatrix<double> m_s = (A*b);
	;
	//m.resize(m_s.rows(),m_s.cols());
	//m.setZero();
	//for(int i=0; i<m_s.rows(); i++)
	//{
	//	for (int j=0; j<m_s.cols(); j++)
	//	{
	//		m(i,j) = m_s.coeffRef(i,j);
	//	}
	//}
	m = Eigen::MatrixXd(m_s);
}

//Turned off now 
//#define ENABLE_TIMING_GEO_SOLVER 

#ifdef ENABLE_TIMING_GEO_SOLVER
#include <TimerWrapper.h>
#endif

double GeometrySolver::update(int arap_iters, int phys_iter, bool is_arap, bool fakeUpdate, double * gravity, double damping)
{
	double energy = 0;
	for (int i = 0; i < phys_iter; i++)
	{

#ifdef ENABLE_TIMING_GEO_SOLVER
		TimerWrapper timerWrapper;
		timerWrapper.Tic();
#endif

		energy = phys_update_once(arap_iters, is_arap, fakeUpdate, gravity, damping);

#ifdef ENABLE_TIMING_GEO_SOLVER
		timerWrapper.Toc();
		printf("Per ARAP iteration time: %f.\n", timerWrapper.Duration()*1.0/arap_iters);
#endif

	}
	return energy;
}

double GeometrySolver::phys_update_once(int arap_iters, bool is_arap, bool fakeUpdate, double * gravity, double damping)
{
	int dim = V.cols();

	int n_var = Varibles.rows();

	double energy = 0;

#ifdef DO_TIMING
	TimerWrapper timerWrapper;
#endif

	for (int arap_iter=0; arap_iter<arap_iters; arap_iter++)
	{
		//iteration

		energy = 0;


		// NOTE: all definition here is exactly following the FAST paper.
		// R = [R1,R2,...,Rr], R is dim by dim*r, r is same as 
		// K = [V^TA1C1A1^T;
		//		V^TA2C2A2^T;
		//		...
		//		V^TArCrAr^T;]
		// S = KT = [S1;S2;...;Sr], T=V if no rotation clusters introduced.

		//Local step

		// S is dim*r by dim, which is the same as the FAST paper defination.
		
		#ifdef DO_TIMING
		timerWrapper.Tic();
		#endif

		//Eigen::SparseMatrix<double> Var_sparse = Varibles.sparseView(0.0001);
		Eigen::MatrixXd S = K_reduced_hat * Varibles;//K_in_paper * (M * Varibles);// This order of multiplication is good if no rotation cluster is used.
		

		// Copied this from Alec's implementation: THIS NORMALIZATION IS IMPORTANT TO GET SINGLE PRECISION SVD CODE TO WORK CORRECTLY.
		S /= S.array().abs().maxCoeff();

		#ifdef DO_TIMING
		timerWrapper.Toc();
		printf("ARAP step1 time:%f\n",timerWrapper.Duration());
		timerWrapper.Tic();
		#endif

		//Eigen::MatrixXd S_alec = CSM * V.replicate(dim,1);
		//std::cout << "Alec's Implemetation of CSM: " << CSM << std::endl;
		//std::cout << "Wangyu's Implemetation of K: " << K_in_paper << std::endl;
		
		//std::cout << "Alec's Implemetation of CSM_1: " << CSM.block(0,0,V.rows(),V.rows()) << std::endl;
		//std::cout << "Wangyu's Implemetation of K_1: " << K_in_paper.block(0,0,V.rows(),V.rows()) << std::endl;

		/*std::cout << "Alec's Implemetation of S_1: " << S_alec.block(0,0,dim,dim) << std::endl;
		std::cout << "Wangyu's Implemetation of S_1: " << S.block(0,0,dim,dim) << std::endl;
		std::cout << "Alec's Implemetation of S_2: " << S_alec.block(dim,0,dim,dim) << std::endl;
		std::cout << "Wangyu's Implemetation of S_2: " << S.block(dim,0,dim,dim) << std::endl;*/


		#ifdef DO_TIMING
		timerWrapper.Toc();
		printf("ARAP step2 time:%f\n",timerWrapper.Duration());
		timerWrapper.Tic();
		#endif

		Eigen::MatrixXd R_eff(dim,S.rows());
		//Eigen::MatrixXd R;

		//std::cout<<"S_eff=["<<std::endl<<S_eff<<std::endl<<"];"<<std::endl;

		// This local step is still necessary even for fakeUpdate, but one arap_iter is enough.
		if (dim==3)
		{
			Solve_for_Rotations(S,R_eff);// the indexing of S is different form 
		} 
		else
		{
			Solve_for_Rotations_2D(S,R_eff);
			// CANNOT use alec's fit_rotation function they are DIFFERENT igl::fit_rotations_planar(S,R_eff);
		}
		
		// Set Identity Matrix for debug purpose
		//for (int i=0; i<R_eff.cols()/dim; i++)
		//{
		//	R_eff.block(0,dim*i,dim,dim)=Eigen::MatrixXd::Identity(dim,dim);
		//}
		
		if (is_arap)
		{// ARAP
			// add the constant part of the energy.
			for (int i=0; i<Scale_Weight_reduced.rows(); i++)
			{
				energy +=  0.5 * Scale_Weight_reduced(i);
			}

			// Do nothing
		}
		else
		{// ASAP
			// Gather multiplication of each block of S and R_eff
			Eigen::VectorXd Scale_reduced(Scale_Weight_reduced.rows());// This is used for ASAP energy. It is a rots by 1 vector. rots is number of rotation group.
			Scale_reduced.setZero();
			for (int i=0; i<Scale_reduced.rows(); i++)
			{
				for (int c=0; c<dim; c++)
				{
					Scale_reduced(i) += R_eff(c,i*dim+c) * S(i*dim+c,c);
				}
				Scale_reduced(i) /= Scale_Weight_reduced(i);
				R_eff.block(0,i*dim,dim,dim) *= Scale_reduced(i);

				energy +=  0.5 * Scale_Weight_reduced(i) * Scale_reduced(i) * Scale_reduced(i);
			}
		}
		

		// Cout is slow without Optimization!
		//std::cout<<"S:"<<std::endl;
		//std::cout<<S.block(0,0,dim,dim) <<std::endl;
		//std::cout<<"R_eff:"<<std::endl;
		//std::cout<<R_eff.block(0,0,dim,dim) <<std::endl;
		
		// Number of rotations: #vertices or #elements
		//int num_rots = K.cols()/dim/dim;

		//Global Step

		#ifdef DO_TIMING
		timerWrapper.Toc();
		printf("ARAP step3 time:%f\n",timerWrapper.Duration());
		#endif


		#ifdef DO_TIMING
		timerWrapper.Tic();
		#endif

		Eigen::MatrixXd B = -R_eff*K_reduced_hat;

		if (with_dynamics)
		{
			// damping: 1 means no damping and 0 means full damping.
			B = B - ( Varibles_last + damping * Varibles_last - damping * Varibles_last_last).transpose() * Mass_reduced / (time_step*time_step);
		}

		if (true)
		{
			Eigen::MatrixXd Fext_t = Eigen::MatrixXd(dim, ConstantMassM.cols());
			for (int c = 0; c < dim; c++)
			{
				Fext_t.row(c) = ConstantMassM * gravity[c];
			}
			B = B - Fext_t;
		}


		if (fakeUpdate)
		{
			energy += quadSolver.fakeSolve(
				B.transpose(), 
				Varibles);

			if (arap_iters>1)
			{
				printf("Warning: arap iteration is only executed once for fake solving!\n");
			}
			return energy;// fake update return here.
		} 
		else
		{
			energy += quadSolver.solve(
				B.transpose(), 
				Varibles);
		}



		//for (int c=0; c<dim; c++)
		//{
		//	Eigen::VectorXd V_c;
		//	Eigen::VectorXd B_c = R_eff_K_reduced_hat.row(c).transpose();//Bcol.block(0,c,n_var,1);
		//	Eigen::VectorXd knownY_c = quadSolver.knownY.col(c);
		//	Eigen::VectorXd Beq_c;// Zero constraints
		//	quadSolver.solve(B_c, knownY_c, Beq_c, V_c);
		//	Varibles.col(c) = V_c;
		//}

		#ifdef DO_TIMING
		timerWrapper.Toc();
		printf("ARAP min_quad time:%f\n",timerWrapper.Duration());
		#endif



		if (false)
		{
			// Positional Collision Handling
			// By now only supporting points varibles
			if (Varibles.cols() == 3)
			{
				for (int i = 0; i < Varibles.rows(); i++)
				{
					if (Varibles(i, 2) < -2)
						Varibles(i, 2) = -2;
				}
			}
		}


	}

	// the following is only for non-fake solve:

	if (with_subspace_weights)
	{
		//DO NOT update V here since this is very costly!!
		//Need to do this outside the solver!!
		//linear_multiply(M,Varibles.sparseView(),V);
		//V = M*Varibles;
		//Eigen::SparseMatrix<double> Varibles_sparse = Varibles.sparseView();
		//V = M*Varibles_sparse;
	}
	else
	{
		V = Varibles;
	}

	Varibles_last_last = Varibles_last;// order is important!
	Varibles_last = Varibles;

	return energy;
}

void GeometrySolver::setPerGroupWeight(const Eigen::VectorXd& gw)
{
	per_group_weight = gw;
}

void GeometrySolver::perElementEnergy(Eigen::VectorXd& energy)
{
	int n = V.rows();
	energy.resize(n);
}

void GeometrySolver::perGroupEnergy(Eigen::VectorXd& energy)
{
	int r = rotation_group.maxCoeff()+1;
	energy.resize(r);
}
