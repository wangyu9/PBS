#ifndef DEFORM_SKINNING_BASE_H
#define DEFORM_SKINNING_BASE_H

#include <string>

#include "ViewerPlugin.h"

//#include "DeformerPicking.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>


//#include "FAST/skinning/Animation.h"
#include <Animation.h>
#include "utils/TimerWrapper.h"

//#define USE_MATLAB_ENGINE
#ifdef USE_MATLAB_ENGINE
#include "engine.h"

#ifdef USING_IGL_HEADER_ONLY_MODE
#define IGL_HEADER_ONLY 
#include "igl/matlab/matlabinterface.h"
#undef IGL_HEADER_ONLY
#else
#include "igl/matlab/matlabinterface.h"
#endif

#endif

#include "virtual_function_default.h"

typedef enum { LBS, DQLBS, PBS, HS, None } SkinningType;
typedef enum {
	BilaplacianCoordinate = 0,
	ThinPlateSpline = 1,
	StandardBilaplacian = 2,
	LeastSquareMesh = 3,
	BoundedBiharmonicWeights = 4,
	BiharmonicWeights = 5,
	FacetBiharmonicWeights = 6,
	ConstraintBilaplacianCoordinate = 7,
	TrilaplacianCoordinate = 8,
	TestSlot2 = 9,
	TestSlot3 = 10
} WeightsType;


inline SkinningType str2_skinning_type(const std::string st);
inline WeightsType str2_weights_type(const std::string wt);

typedef enum { NaiveNormalUpdate, SkinningNormalUpdate, NoNormalUpdate } NormalUpdateType;
//Notice: this should match the definition in TwBar definition in the init function

class WeightsFrame
{
public:
	std::string name;
	Eigen::MatrixXd Weights;
	WeightsFrame(std::string n, Eigen::MatrixXd W) :name(n), Weights(W){}
};


class DeformSkinningBase
{
public:
	//public interface


	void SetUpdateSkinning()
	{
		update_skinning = true;
	}

	void SetHasLoadHandle(bool v = true)
	{
		hasLoadHandles = v;
	}

	bool GetHasLoadHandle()
	{
		return hasLoadHandles;
	}

	bool GetHasLoadWeights()
	{
		return hasLoadWeights;
	}

	void ApplySkinning()
	{
		applySkinning();
	}

	// This function should call from deform phys to send data in 
	// input: dim 2 or 3
	// output Var
	void SendVarToDeformPhys(const int dim, Eigen::MatrixXd& Var);

	void SendBasisToDeformPhys(const int dim, Eigen::MatrixXd& ARAP_M);
	const Eigen::MatrixXd& HandlesToViewer()
	{
		return Handles();
	}
	void set_M_hs_2nd(const Eigen::MatrixXd& new_M3d_hs_2nd, const Eigen::MatrixXd& new_M2d_hs_2nd)
	{
		M3d_hs_2nd = new_M3d_hs_2nd;
		M2d_hs_2nd = new_M2d_hs_2nd;
	}
	//void setM(const Eigen::MatrixXd& new_M3d_hs, const Eigen::MatrixXd& new_M2d_hs)
	//{
	//	M3d_hs = new_M3d_hs;
	//	M2d_hs = new_M2d_hs;
	//}
	const Eigen::MatrixXd& GetM(int dim)
	{
		switch (skinningType)
		{
		case LBS:
			return GetM_lbs(dim);
		case HS:
			return GetM_hs(dim);
		case DQLBS:
		case None:
		case PBS:
			return GetM_hs(dim);
		}
	}
	const Eigen::MatrixXd& GetM_hs(int dim)
	{
		//set_M_lbs_matrix(3);
		return (dim == 3) ? M3d_hs : M2d_hs;
	}
	const Eigen::MatrixXd& GetM_lbs(int dim)
	{
		//set_M_lbs_matrix(3);
		return (dim == 3) ? M3d_lbs : M2d_lbs;
	}
	//void update_handles_in_picking();
	SkinningType getSkinningType(){
		return skinningType;
	}
private:
	Eigen::MatrixXd GetVar(int dim) const
	{
		switch (skinningType){
		case LBS:
			return HandleTrans(dim);
			break;
		case HS:
			return HandleVars(dim);
			break;
		case PBS:
			return Handles(dim);
			break;
		}
	}


	/****** Virtual Functions defined here *******/
protected:
	virtual void copy_skinned_mesh_back() const VIRTUAL_VOID_DEFAULT_NO_DEFINATION(copy_skinned_mesh_back)
private:
	// TODO make all of these to be reference.
		virtual Eigen::MatrixXd RestHandles(int dim) const VIRTUAL_EIGEN_MATRIXXD_DEFAULT_NO_DEFINATION(RestHandles)
		virtual Eigen::MatrixXd Handles(int dim = 3) const VIRTUAL_EIGEN_MATRIXXD_DEFAULT_NO_DEFINATION(Handles)//TODO remove this default value
		virtual Eigen::MatrixXd HandleTrans(int dim) const VIRTUAL_EIGEN_MATRIXXD_DEFAULT_NO_DEFINATION(HandleTrans)
		virtual Eigen::MatrixXd HandleVars(int dim) const VIRTUAL_EIGEN_MATRIXXD_DEFAULT_NO_DEFINATION(HandleVars)
		virtual Eigen::VectorXi HandleIndices() const VIRTUAL_EIGEN_VECTORXI_DEFAULT_NO_DEFINATION(HandleIndices)
public:
	bool enable_deform_skinning;

	DeformSkinningBase();
	~DeformSkinningBase();


	// initialization (runs every time a mesh is loaded or cleared)
	void init(Preview3D *preview);

	void UpdateConstraintVertexPositions(const std::vector<IndexType>& constraintVertices, const Eigen::MatrixXd& positions, bool update_skinning);
	void UpdatePositionalConstraints(const std::vector<IndexType>& constraintVertices, bool update_skinning);

	bool recompute_normal_updated;

	//Eigen::MatrixXd*
	Eigen::MatrixXd* pV;
	//Eigen::MatrixXi*
	Eigen::MatrixXi* pF;

	bool wantTetMesh;//whether do we want the mesh to be tethedronlize
	std::string currentWeightsFileName;


	Eigen::MatrixXd W;
	Eigen::MatrixXi WI;

	Eigen::MatrixXd M3d_lbs;
	Eigen::MatrixXd M2d_lbs;
	Eigen::MatrixXd M3d_hs;
	Eigen::MatrixXd M2d_hs;

	Eigen::MatrixXd M3d_hs_2nd;
	Eigen::MatrixXd M2d_hs_2nd;

	// Deform Factors which used for Effecient Lighting
	Eigen::MatrixXd alphaFactors;	// Same Dimension as Weights
	Eigen::MatrixXd betaFactors;	// Same Dimension as Weights
	// Rest pose dir
	Eigen::MatrixXd t_dir; // Note the dimesnsion is (3,V.rows()) by now!!
	Eigen::MatrixXd b_dir; // Note the dimesnsion is (3,V.rows()) by now!!
	// Updated pose dir
	Eigen::MatrixXd tang_dir;
	Eigen::MatrixXd binormal_dir;
	void draw_tangents();

	Eigen::MatrixXd sorted_alphaFactors;
	Eigen::MatrixXd sorted_betaFactors;
	void computeDeformFactors();

	bool draw_weights_to_mesh(const std::vector<std::pair<int, Eigen::VectorXd>>& cols_to_draw);

	bool set_skinning_type(const std::string stn);
	bool set_weights_type(const std::string wtn);

	void SetSkinning(SkinningType st)
	{
		skinningType = st;
	}

	SkinningType GetSkinning() const
	{
		return skinningType;
	}

	void SetWeightsType(WeightsType wt)
	{
		weightsType = wt;
	}
	WeightsType GetWeightsType() const
	{
		return weightsType;
	}
	void SetNormalUpdateType(NormalUpdateType nt)
	{
		normalUpdateType = nt;
	}
	NormalUpdateType GetNormalUpdateType() const
	{
		return normalUpdateType;
	}

protected:

	Preview3D *m_preview3d;

	void sortWeights();

	int numElements;


	bool isMeshLoaded;
	bool showInvertedElements;
	bool drawVertexTangents;
	bool runSolver;
	bool isTetMesh;
	SkinningType skinningType;
	bool hasLoadHandles;
	bool hasLoadWeights;

	Eigen::MatrixXd Weights;
	//bool showHandles;
public:
#ifdef USE_MATLAB_ENGINE
	Engine **matlabEngine;
#endif	
	Eigen::MatrixXd GetWeights() const
	{
		return Weights;
	}
protected:


	/*********************** Weights, Deform Factors **********************/
	void set_M_lbs_matrix(int dim = 3);

public:
	bool set_weights(const Eigen::MatrixXd& newW, const std::string name);
	bool load_weights_from_file(const char* weights_file_name);
	bool load_sparse_weights_from_file(const char* sparse_weights_file_nameconst, const char* sparse_weight_indices_file_name);
	bool load_deform_factors_from_file(const char* alpha_factors_file_nameconst, const char* beta_factors_file_nameconst);
	bool save_deform_factors_to_file(const char* alpha_factors_file_nameconst, const char* beta_factors_file_nameconst);

protected:

	bool save_weights_to_file(const char* weights_file_name);



	WeightsType weightsType;
	bool enable_draw_weights_on_mesh;

	void reproject_sparse_weights();

	void reproject_deform_factors();

	virtual void do_this_when_weights_loaded();
	virtual void do_this_when_only_M_loaded();
	/************************************************************/

	NormalUpdateType normalUpdateType;

	/*************************** Skinning ***************************/
	bool use_full_weights;//for skinning
	bool use_full_factors;//for normal updating
	bool num_sparse_weight_slots;
	bool num_sparse_factor_slots;
public:
	bool update_skinning;
	void applySkinning();
protected:

	void applyPointBasedSkinning(Eigen::Matrix<double, Eigen::Dynamic, 3>* verticesMatrix);
	void applyLinearBlendSkinning(Eigen::MatrixXd* verticesMatrix);
	void applyHybridSkinning(Eigen::MatrixXd* verticesMatrix);

public:
	bool workSpaceClearedBeforeUse;

	//key frame
	int current_weights_frame_index;
	std::vector<WeightsFrame> weightsFrames;
};

class DeformSkinningBaseUI : public DeformSkinningBase, public PreviewPlugin
{
protected:
	// Pointer to the tweak bar
	//igl::ReTwBar* bar; wangyu moved this to ViewerPlugin

public:
	DeformSkinningBaseUI();
	~DeformSkinningBaseUI();

	// implement Serializable interface
	bool Serialize(tinyxml2::XMLDocument* doc, tinyxml2::XMLElement* element);
	bool Deserialize(tinyxml2::XMLDocument* doc, const tinyxml2::XMLElement* element);

	// initialization (runs every time a mesh is loaded or cleared)
	void init(Preview3D *preview);

	// keyboard callback
	bool keyDownEvent(unsigned char key, int modifiers, int mouse_x, int mouse_y);

	//mouse callback
	bool mouseDownEvent(int mouse_x, int mouse_y, int button, int modifiers);
	bool mouseUpEvent(int mouse_x, int mouse_y, int button, int modifiers);
	bool mouseMoveEvent(int mouse_x, int mouse_y);
	bool mouseScrollEvent(int mouse_x, int mouse_y, float delta);

	//stuff that is drawn by the plugin before the previewer has displayed the mesh
	//first draw 3d, then 2d
	void preDraw(int currentTime);
	//stuff that is drawn by the plugin after the previewer has displayed the mesh
	//first draw 3d, then 2d
	void postDraw(int currentTime);


protected:
	//static void TW_CALL draw_dialog_weights(void *clientData);


	static void TW_CALL SetSkinningCB(const void *value, void *clientData)
	{
		static_cast<DeformSkinningBaseUI*>(clientData)->SetSkinning(*static_cast<const SkinningType *>(value));
	}
	static void TW_CALL GetSkinningCB(void *value, void *clientData)
	{
		*static_cast<SkinningType *>(value) = static_cast<const DeformSkinningBaseUI*>(clientData)->GetSkinning();
	}
	static void TW_CALL open_dialog_weights(void *clientData);
	static void TW_CALL open_dialog_sparse_weights(void *clientData);
	static void TW_CALL open_dialog_deform_factors(void *clientData);
	static void TW_CALL save_dialog_deform_factors(void *clientData);

	static void TW_CALL save_dialog_weights(void *clientData);
	static void TW_CALL SetWeightsTypeCB(const void *value, void *clientData)
	{
		static_cast<DeformSkinningBaseUI*>(clientData)->SetWeightsType(*static_cast<const WeightsType *>(value));
	}
	static void TW_CALL GetWeightsTypeCB(void *value, void *clientData)
	{
		*static_cast<WeightsType *>(value) = static_cast<const DeformSkinningBaseUI*>(clientData)->GetWeightsType();
	}

	/******************************************************/
	static void TW_CALL ReprojectSparseWeights(void *clientData)
	{
		static_cast<DeformSkinningBaseUI*>(clientData)->reproject_sparse_weights();
	}
	static void TW_CALL ReprojectDeformFactors(void *clientData)
	{
		static_cast<DeformSkinningBaseUI*>(clientData)->reproject_deform_factors();
	}
	static void TW_CALL SetNormalUpdateTypeCB(const void *value, void *clientData)
	{
		static_cast<DeformSkinningBaseUI*>(clientData)->SetNormalUpdateType(*static_cast<const NormalUpdateType *>(value));
	}
	static void TW_CALL GetNormalUpdateTypeCB(void *value, void *clientData)
	{
		*static_cast<NormalUpdateType *>(value) = static_cast<const DeformSkinningBaseUI*>(clientData)->GetNormalUpdateType();
	}

};


#endif /*DEFORM_SKINNING_BASE_H*/