



//inc cstd lib
#include <cstdlib>
#include <string>

//#include <GL/gl.h>// by wangyu
#include <GL/glut.h>

//inc AntTweakBar
#include "AntTweakBar.h"
//inc Eigen
#include <Eigen/Dense>
//inc targaImage
#include "targaImage.h"
//inc tetgen
//#include <tetgen.h>
//inc libigl


#define IGL_HEADER_ONLY
#include <igl/readObj.h>
#include <igl/adjacency_list.h>
#include <igl/cat.h>
#include <igl/tetgen/tetrahedralize.h>
#include <igl/writeMESH.h>
#include <igl/writeDMAT.h>
#include <igl/mosek/bbw.h>
//inc c++ std lib
#include <iostream>
#include <vector>
#include <queue>
#include <fstream>
//basic data types in geometry and character animation
#include "./math/dualQuat.h"
#include "./math/types.h"

using namespace std;


int window_height = 600;
int window_width = 800;
float rotx=0,roty=0;
float transx=-5,transy=-5,transz=0;
float ori_x = 0;
float ori_y = 0;
bool RightMouseDown = false;
bool LeftMouseDown  = false;

float deltaTime = 0.33f;

GLfloat mat_diff_bone[]= { 0.8f, 0.8f, 0.8f, 1.0f };
GLfloat mat_diff_skin[]= { 1.0f, 0.9f, 0.2f, 1.0f };
GLfloat mat_spec1[]= { 0.6f, 0.6f, 0.6f, 1.0f };
GLfloat mat_shine1[]={ 128.0f };

//mesh data
Eigen::MatrixXd V_bone;
Eigen::MatrixXi F_bone;
Eigen::MatrixXd CN_bone;
Eigen::MatrixXi FN_bone;
Eigen::MatrixXd TC_bone;
Eigen::MatrixXi FTC_bone;

Eigen::MatrixXd V_skin;
Eigen::MatrixXi F_skin;
Eigen::MatrixXd CN_skin;
Eigen::MatrixXi FN_skin;
Eigen::MatrixXd TC_skin;
Eigen::MatrixXi FTC_skin;
Eigen::MatrixXd Deformed_V_skin;

//bone data
vector<int> vtBoneId;
int numOfBones;
vector<vector<int> > boneVtSegments;
vector<vector<int> > boneFcSegments;
vector<vector<float> > vertexWeights;
vector<vector<float> > vertexWeights_backup;
float bendAng = 0.0f;
float twistAng = 0.0f;

//mesh loaded
bool bone_loaded = false;
bool skin_loaded = false;
//mesh rendering
bool render_skin = false;
bool render_bone = false;
bool skin_in_wire = true;
//minCoordCfg related
struct SFuncInfo{
	int _jntId;
	int _dofId;

	std::vector<double>	_cubicSplineXs; //in degree
	std::vector<double>	_cubicSplineYs; //in degree

	int _coordId;
	DualQuaterniond _dqDerivative;

	double evalFunc(double X){
		//linear function for now
		return (X - _cubicSplineXs[0])*(_cubicSplineYs[1] - _cubicSplineYs[0])/(_cubicSplineXs[1] - _cubicSplineXs[0]) + _cubicSplineYs[0];
	}
	double evalDerivative(double X){
		//linear function for now
		return (_cubicSplineYs[1] - _cubicSplineYs[0])/(_cubicSplineXs[1] - _cubicSplineXs[0]);
	};

};

struct SCoordInfo{
	double _previous;
	double _current;

	double _rangeMin;
	double _rangeMax;

	std::vector<SFuncInfo> _funcs;
};

struct STreeNode{
	int _bId;
	STreeNode* _parent;
	std::vector<STreeNode*>	_children;

	Vector3 _restAxis0;
	Vector3 _restAxis1;
	Vector3 _restAxis2;
	Vector3 _pivot;
	Vector3 _EAngles; //in degree
	DualQuaterniond _curDQ;

	Quatd _relRot;
	Vector3 _relPivot;
	DualQuaterniond _relDQ;
	DualQuaterniond _curJntDQ;

	STreeNode():_bId(-1),
		_parent(NULL){
			_curDQ.setIdentity();
	}
};

//min coord data
vector<SCoordInfo> m_coords;
vector<vector<int> > m_boneGrps;
std::vector<STreeNode> m_treeNodes;
vector<DualQuaterniond>	m_boneDQs;
STreeNode* m_pRoot;

//weight bound info
Vector3 radiusS = Vector3(9.63, -2.00, -2.0);    // from lifeng 9.63 -2.00 -2.0
Vector3 radiusE = Vector3(10.02, -10.94, 0.27);  // from lifeng 10.02 -10.94 0.27
Vector3 ulnaS   = Vector3(8.71, -2.00, -2.0);    // from lifeng 8.71 -2.00 -2.0
Vector3 ulnaE   = Vector3(9.246, -10.94, 0.27);  // from lifeng 9.246 -10.94 0.27
bool show_weight_bound = false;
float m_splineCtrlParam = 2.0f;

Vector3 normalizingDQXformVt(const DualQuaterniond& dq, const Vector3& vt)
{
	const double sqrLen1 = dq.PrimalQuat().squaredNorm();
	//MYRELEASEASSERT(sqrLen1>1e-8);
	const double invSqrLen1 = 1.0/sqrLen1;

	const double& w1 = dq.PrimalQuat().w(); double w12 = w1*w1;
	const double& x1  = dq.PrimalQuat().x(); double x12 = x1*x1;
	const double& y1  = dq.PrimalQuat().y(); double y12 = y1*y1;
	const double& z1  = dq.PrimalQuat().z(); double z12 = z1*z1;
	const double& w2 = dq.DualQuat().w();
	const double& x2 = dq.DualQuat().x();
	const double& y2 = dq.DualQuat().y();
	const double& z2 = dq.DualQuat().z();
	const double& v1 = vt[0];
	const double& v2 = vt[1];
	const double& v3 = vt[2];

	Vector3 ret;

	ret[0] = x12*v1+2*y1*z2-2*y2*z1+2*x1*v2*y1+2*w1*x2-2*w2*x1+2*w1*v3*y1-2*w1*v2*z1-y12*v1-z12*v1+2*x1*v3*z1+w12*v1;
	ret[0] *= invSqrLen1;
	ret[1] = 2*z1*x2-2*z2*x1-v2*x12-v2*z12+2*y1*v1*x1+2*z1*v1*w1-2*w1*v3*x1+2*y1*v3*z1+y12*v2-2*y1*w2+2*y2*w1+w12*v2;
	ret[1] *= invSqrLen1;
	ret[2] = -2*z1*v1*x1-2*w1*v2*x1-2*y1*v2*z1+2*y1*v1*w1-w12*v3+x12*v3+2*y1*x2-2*y2*x1+2*z1*w2-2*z2*w1+y12*v3-z12*v3;
	ret[2] *= -invSqrLen1;

	return ret;
}

void applySkinning_DQS()
{
	DualQuaterniond summedDQ;
	int bid;
	int m_numOfSkinVts = V_skin.rows();

	for (int vi=0; vi<m_numOfSkinVts; vi++)
	{
		//summing up bone dqs
		summedDQ.setZero();
		for (int ibgp=0; ibgp<m_boneGrps.size(); ibgp++)
		{
			vector<int> &boneGrp = m_boneGrps[ibgp];
			for (int ibpt=0; ibpt<boneGrp.size(); ibpt++)
			{
				int bonePart = boneGrp[ibpt];
				summedDQ += vertexWeights[vi][bonePart] * m_boneDQs[ibgp]; //DQ are the same in one group
			}
		}// for bone groups

		//transform vt with unified bone dqs
		const Vector3& restV = V_skin.row(vi);
		Deformed_V_skin.row(vi) = normalizingDQXformVt(summedDQ, restV);
	}// for m_numOfSkinVts
}

Quatd sequenceAngle2Quat(const Vector3& euAngles, int dofi)
{
	double half = DEGREE2RADIAN(euAngles[dofi])*0.5;
	double sinH = sin(half);
	double cosH = cos(half);

	Vector3 axis = Vector3::Zero();
	axis[dofi] = 1.0;

	Quatd ret(cosH, sinH*axis[0], sinH*axis[1], sinH*axis[2]);
	return ret;
}

DualQuaterniond getJntDQ(STreeNode* pNode)
{
	Vector3 transP = pNode->_pivot;
	Quatd rot[3];
	DualQuaterniond dq[3];
	for (int ii=0; ii<3; ++ii)
	{
		rot[ii] = sequenceAngle2Quat(pNode->_EAngles, ii);
		rot[ii] = pNode->_relRot * rot[ii];
		dq[ii].FromTransRot(transP, rot[ii]);
		dq[ii] = dq[ii] * pNode->_relDQ;
	}

	pNode->_curDQ = dq[0] * dq[1] * dq[2];
	return pNode->_curDQ;
}

void doFK(STreeNode* pNode)
{
	//	cout<<pNode->_bId<<"\n";
	DualQuaterniond& curFrm = m_boneDQs[pNode->_bId];

	if(pNode->_parent!=NULL){
		getJntDQ(pNode);
		curFrm = m_boneDQs[pNode->_parent->_bId] * pNode->_curDQ;
	}else{
		//root
		curFrm = getJntDQ(pNode);
	}
	for (size_t ci=0; ci<pNode->_children.size(); ++ci){
		doFK(pNode->_children[ci]);
	}
}

void setCoord(int coordId, double value)
{
	if(coordId<0 || coordId>=(int)m_coords.size()) return;

	SCoordInfo& curCoord = m_coords[coordId];
	curCoord._previous = curCoord._current;
	curCoord._current = value;
	if(curCoord._current<curCoord._rangeMin) curCoord._current = curCoord._rangeMin;
	if(curCoord._current>curCoord._rangeMax) curCoord._current = curCoord._rangeMax;


	for (size_t fi=0; fi<curCoord._funcs.size(); ++fi){
		SFuncInfo& curFunc = curCoord._funcs[fi];
		STreeNode* curNode = &m_treeNodes[curFunc._jntId];

		if(curFunc._dofId<3)
			curNode->_EAngles[curFunc._dofId] = curFunc.evalFunc(curCoord._current);
		else
			curNode->_pivot[curFunc._dofId] = curFunc.evalFunc(curCoord._current);
	}

	doFK(m_pRoot);
	applySkinning_DQS();
}

void segmentBone()
{
	vector<vector<int> >boneAdjMatrix;
	igl::adjacency_list(F_bone, boneAdjMatrix);
	// compute each vertex bone part id, initially assigned to -1
	int nV = V_bone.rows();
	vector<int> boneVerPartID(nV, -1);
	int seed;
	queue<int> toBeVisited;
	int curBoneVerPartId = 0;

	while (true)
	{
		seed = 0;
		for (; seed<nV; ++seed)
		{
			if (boneVerPartID[seed] == -1)
				break;
		}//for(seed)

		if (seed < nV)
			toBeVisited.push(seed);
		else
			break;
		// BFS search
		while(!toBeVisited.empty())
		{
			seed = toBeVisited.front();
			toBeVisited.pop();
			if(boneVerPartID[seed]==-1){
				boneVerPartID[seed] = curBoneVerPartId;

				vector<int>& neiVts = boneAdjMatrix[seed];
				for (size_t ni=0; ni<neiVts.size(); ++ni) if(boneVerPartID[neiVts[ni]]==-1){
					toBeVisited.push(neiVts[ni]);
				}
			} //if
		}// while
		++curBoneVerPartId;

	}//while(true)

	vtBoneId = boneVerPartID;
	numOfBones = curBoneVerPartId;
	boneVtSegments.resize(numOfBones);
	boneFcSegments.resize(numOfBones);

	for (int vi=0; vi<nV; ++vi){
		boneVtSegments[boneVerPartID[vi]].push_back(vi);
	}

	int nF = F_bone.rows();
	for (int fi=0; fi<nF; ++fi)
		boneFcSegments[boneVerPartID[F_bone(fi, 0)]].push_back(fi);

	cout<<numOfBones<<" bones in total\n";

	int vcount = 0;
	int fcount = 0;
	for (int bpid=0; bpid<numOfBones; bpid++)
	{
		vcount += boneVtSegments[bpid].size();
		fcount += boneFcSegments[bpid].size();
	}
	cout << "vcount " << vcount << endl << "fcount " << fcount << endl;
}

void readBone()
{
	igl::readOBJ("../data/geo/LArm_full.obj",V_bone,F_bone,CN_bone,FN_bone,TC_bone,FTC_bone);
	bone_loaded = true;
	render_bone = true;
	segmentBone();
}

void readSkin()
{
	igl::readOBJ("../data/geo/skin.obj",V_skin,F_skin,CN_skin,FN_skin,TC_skin,FTC_skin);
	skin_loaded = true;
	render_skin = true;
	Deformed_V_skin = V_skin;
}

void readMinCoordsCfg()
{
	// open weight file ========
	ifstream minCfgFile;
	minCfgFile.open("../data/geo/minCoordArmS6.txt");
	if (!minCfgFile)
		cout << "error in open minCoordsCfg file" << endl;
	else
		cout << "minCoordsCfg file is opened " << endl;

	//temporary variable for reading info from cfg files
	int tmpI, tmpI2;

	// read min coords info
	minCfgFile>>tmpI;
	m_coords.resize(tmpI);
	for (size_t ci=0; ci<m_coords.size(); ++ci)
	{
		SCoordInfo& curC = m_coords[ci];
		minCfgFile>>tmpI;
		minCfgFile>>curC._rangeMin>>curC._rangeMax;
		curC._previous = curC._current = 0.0;

		minCfgFile>>tmpI;
		curC._funcs.resize(tmpI);
		for (size_t fi=0; fi<curC._funcs.size(); ++fi){
			SFuncInfo& curF = curC._funcs[fi];
			curF._coordId = (int)ci;
			minCfgFile>>curF._jntId>>curF._dofId;

			minCfgFile>>tmpI;
			curF._cubicSplineXs.resize(tmpI);
			curF._cubicSplineYs.resize(tmpI);
			for (int ki=0; ki<tmpI; ++ki){
				minCfgFile>>curF._cubicSplineXs[ki]>>curF._cubicSplineYs[ki];
			}
		}
	}//min coords info

	//read group info
	minCfgFile>>tmpI;
	m_boneGrps.resize(tmpI);
	for (size_t gi=0; gi<m_boneGrps.size(); ++gi)
	{
		minCfgFile>>tmpI;
		vector<int>& curGrp = m_boneGrps[gi];
		curGrp.resize(tmpI);

		for (int bi=0; bi<tmpI; ++bi)
			minCfgFile>>curGrp[bi];
	}// group info

	//read link info
	minCfgFile>>tmpI;
	m_treeNodes.resize(tmpI);
	for (size_t li=0; li<m_treeNodes.size(); ++li)
	{
		minCfgFile>>tmpI>>tmpI2;
		STreeNode* curNode = &m_treeNodes[li];
		curNode->_bId = tmpI2;
		if(tmpI>=0){
			STreeNode* pParentNode = NULL;
			for (size_t lj=0; lj<m_treeNodes.size(); ++lj) if(m_treeNodes[lj]._bId==tmpI){
				pParentNode = &m_treeNodes[lj];
				break;
			}

			curNode->_parent = pParentNode;
			pParentNode->_children.push_back(curNode);

		}else{
			curNode->_parent = NULL;
			m_pRoot = curNode;
		}

		//now the rest axis
		minCfgFile>>curNode->_restAxis0[0]>>curNode->_restAxis0[1]>>curNode->_restAxis0[2];
		minCfgFile>>curNode->_restAxis1[0]>>curNode->_restAxis1[1]>>curNode->_restAxis1[2];
		minCfgFile>>curNode->_restAxis2[0]>>curNode->_restAxis2[1]>>curNode->_restAxis2[2];
		minCfgFile>>curNode->_pivot[0]>>curNode->_pivot[1]>>curNode->_pivot[2];
		minCfgFile>>curNode->_EAngles[0]>>curNode->_EAngles[1]>>curNode->_EAngles[2];

		Matrix33 f;
		f.col(0) = curNode->_restAxis0;
		f.col(1) = curNode->_restAxis1;
		f.col(2) = curNode->_restAxis2;
		curNode->_relRot = f;
		curNode->_relRot.normalize();
		curNode->_relPivot = curNode->_pivot;

		curNode->_relDQ.FromTransRot(curNode->_relPivot, curNode->_relRot);
		curNode->_relDQ = curNode->_relDQ.Conjugate();
	}// link info

	// init m_boneDQs
	m_boneDQs.resize(m_boneGrps.size());
	for (int i=0; i<m_boneGrps.size(); i++)
	{
		m_boneDQs[i].setIdentity();
	}
}

void readBoneWeights()
{
	// open weight file ========
	ifstream weightFile;
	weightFile.open("../data/geo/tetBBW_old.vwfs");
	if (!weightFile)
		cout << "error in open weight file" << endl;
	else
		cout << "weight file is opened " << endl;

	// read number of bones ========
	int numBones;
	weightFile >> numBones;
	cout << "number of bones is " << numBones << endl;

	// read weight data ========
	float weight;
	int vW_count = 0;
	vector<float> vW;
	while (!weightFile.eof())
	{
		if (vW_count == numBones)
		{
			vertexWeights.push_back(vW);
			vW.clear();
			vW_count = 0;
		}

		weightFile >> weight;
		vW.push_back(weight);
		vW_count++;
	}// while

	// backup the weights for resetting
	vertexWeights_backup = vertexWeights;
}

///////////////////////////////////////////////////////////////////////////////////

void testbbw()
{
	// tetgen inputs
	//vector<vector<REAL> > tetInputVertices; // include skin vertices and bone vertices
	Eigen::MatrixXd V_bone_top2 = V_bone.topRows(2);
	Eigen::MatrixXd V_skin_bone = igl::cat(1,V_skin,V_bone_top2);
	//tetgen_flags: "pq1.414a0.01" , "qa0.01" , "pq2Y"
	const string tetgen_flags = "pq2Y";
	// tetgen outputs
	Eigen::MatrixXd VV;
	Eigen::MatrixXi TT;
	Eigen::MatrixXi FF;
	// generate tet
	cout <<"============tetgen begin()============"<<endl;
	int tet_status = igl::tetrahedralize( V_skin_bone,F_skin,tetgen_flags,VV,TT,FF);
	cout <<"============ tetgen end() ============"<<endl;
	cout << endl << endl << " ============ output mesh =============" << endl;
	igl::writeMESH("output.mesh", VV, TT, FF);

	cout << endl << endl << "============ compute BBW ============"  << endl;
	//setting boundary
	int nVtsSkin = V_skin.rows();
	Eigen::VectorXi b(2);
	Eigen::MatrixXd bc = Eigen::MatrixXd::Zero(2,2);

	for (int i=0; i<2; i++)
	{
		b[i] = i+nVtsSkin;
		bc.coeffRef(i,i) = 1.0;
	}// for bone groups

	// setting bbw solver
	igl::BBWData bbw_data;  // bbw data and flags
	Eigen::MatrixXd W; //weights

	cout << "============= start computing bbw =============" << endl;
	int bbw_status = bbw(VV, TT, b, bc, bbw_data, W);
	if (!bbw_status)
	{
		cout << "============= bbw computing false =============" << endl;
	}
	else
	{
		cout << "============= writting to dmat file =============" << endl;
		igl::writeDMAT("output.dmat", W);
	}
}


///////////////////////////////////////////////////////////////////////////////////
void TW_CALL genTet_BBW(void * param)
{
	// tetgen inputs
	//vector<vector<REAL> > tetInputVertices; // include skin vertices and bone vertices
	Eigen::MatrixXd V_skin_bone = igl::cat(1,V_skin,V_bone);
	//tetgen_flags: "pq1.414a0.01" , "qa0.01" , "pq2Y"
	const string tetgen_flags = "pq2Y";
	// tetgen outputs
	Eigen::MatrixXd VV;
	Eigen::MatrixXi TT;
	Eigen::MatrixXi FF;
	// generate tet
	cout <<"============tetgen begin()============"<<endl;
	int status = igl::tetrahedralize( V_skin_bone,F_skin,tetgen_flags,VV,TT,FF);
	cout <<"============ tetgen end() ============"<<endl;
	cout << endl << endl << " ============ output mesh =============" << endl;
	igl::writeMESH("output.mesh", VV, TT, FF);

	cout << endl << endl << "============ compute BBW ============"  << endl;
	//setting boundary
	int nVtsSkin = V_skin.rows();
	int nVtsBone = V_bone.rows();
	Eigen::VectorXi b(nVtsBone);
	Eigen::MatrixXd bc = Eigen::MatrixXd::Zero(nVtsBone,numOfBones);

	for (int ibgp=0; ibgp<m_boneGrps.size(); ibgp++)
	{
		vector<int> &boneGrp = m_boneGrps[ibgp];
		for (int ibpt=0; ibpt<boneGrp.size(); ibpt++)
		{
			int bonePart = boneGrp[ibpt];
			int nV_in_bonePart = boneVtSegments[bonePart].size();
			vector<int> &bPVt = boneVtSegments[bonePart];

			for (int i=0; i<nV_in_bonePart; i++)
			{
				int boneVtId = bPVt[i];
				int boneVtOffset = boneVtId + nVtsSkin;
				b[boneVtId] = boneVtOffset;
				bc.coeffRef(boneVtId,bonePart) = 1.0;
			}// for vertices in the bone part
		}// for bone parts
	}// for bone groups

	// setting bbw solver
	igl::BBWData bbw_data;  // bbw data and flags
	Eigen::MatrixXd W; //weights

	cout << "============= start computing bbw =============" << endl;
	int bbw_status = bbw(VV, TT, b, bc, bbw_data, W);

	if (!bbw_status)
	{
		cout << "============= bbw computing false =============" << endl;
	}
	else
	{
		cout << "============= writting to dmat file =============" << endl;
		igl::writeDMAT("output.dmat", W);
		ofstream weightFile("output.weight");
		weightFile << numOfBones << "\n";
		for (int i=0; i<nVtsSkin; i++)
		{
			for (int j=0; j<numOfBones; j++)
			{
				weightFile << W.coeff(i,j) << " ";
			}// for bone parts
			weightFile << "\n";
		} // for vertices of skin
	}
}

void AdvanceFrame(int value)
{
	glutTimerFunc(deltaTime*10,AdvanceFrame,0);
	glutPostRedisplay();
}

void init()
{
	glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
	GLfloat light_amb[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat light_diff[]= { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat light_spec[]= { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat light_pos[] = { 2.0f, 2.0f, 4.0f, 1.0f };

	glShadeModel(GL_SMOOTH);
	//	glEnable(GL_DEPTH_TEST);
	glEnable (GL_DEPTH_TEST);
	glEnable (GL_CULL_FACE);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_amb);
	glLightfv( GL_LIGHT0, GL_DIFFUSE, light_diff);
	glLightfv( GL_LIGHT0, GL_SPECULAR, light_spec);
	glLightfv( GL_LIGHT0, GL_POSITION, light_pos);
}

void draw_bone()
{
	int nf = F_bone.rows();
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat_diff_bone);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_spec1);
	glMaterialfv(GL_FRONT, GL_SHININESS,mat_shine1);

	glPolygonMode(GL_FRONT, GL_FILL);
	glBegin(GL_TRIANGLES);
	for (int i=0; i<nf; i++)
	{
		for (int j=0; j<3; j++)
		{
			glNormal3d(CN_bone(FN_bone(i,j),0),CN_bone(FN_bone(i,j),1),CN_bone(FN_bone(i,j),2));
			glVertex3d(V_bone(F_bone(i,j),0),V_bone(F_bone(i,j),1),V_bone(F_bone(i,j),2));
		}
	}
	glEnd();
}

void draw_weight_bound()
{
	glPushMatrix();
	glTranslatef(radiusS[0], radiusS[1], radiusS[2]);
	glutSolidSphere(0.5f, 12, 20);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(radiusE[0], radiusE[1], radiusE[2]);
	glutSolidSphere(0.5f, 12, 20);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(ulnaS[0], ulnaS[1], ulnaS[2]);
	glutSolidSphere(0.5f, 12, 20);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(ulnaE[0], ulnaE[1], ulnaE[2]);
	glutSolidSphere(0.5f, 12, 20);
	glPopMatrix();
}

void setBoneMaterial(int id)
{
	GLfloat mat_diff_boneParts_1[]= { 0.6f, 0.2f, 0.7f, 1.0f };
	GLfloat mat_diff_boneParts_2[]= { 1.0f, 0.2f, 0.2f, 1.0f };
	GLfloat mat_diff_boneParts_3[]= { 0.2f, 0.2f, 1.0f, 1.0f };
	GLfloat mat_diff_boneParts_4[]= { 1.0f, 0.8f, 0.2f, 1.0f };
	GLfloat mat_diff_boneParts_5[]= { 0.2f, 1.0f, 0.3f, 1.0f };
	GLfloat mat_diff_boneParts_default[]= { 0.8f, 0.8f, 0.8f, 1.0f };

	int selection;
	selection = id % 5;

	switch (selection)
	{
	case 0:
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat_diff_boneParts_1);
		break;
	case 1:
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat_diff_boneParts_2);
		break;
	case 2:
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat_diff_boneParts_3);
		break;
	case 3:
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat_diff_boneParts_4);
		break;
	case 4:
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat_diff_boneParts_5);
		break;
	default:
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat_diff_boneParts_default);
	}
}

void draw_bone_parts()
{
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_spec1);
	glMaterialfv(GL_FRONT, GL_SHININESS,mat_shine1);

	glPolygonMode(GL_FRONT, GL_FILL);
	for (int ibgp=0; ibgp<m_boneGrps.size(); ibgp++)
	{
		Vector3 trans;
		Quatd rot;
		m_boneDQs[ibgp].ToTransRot(trans, rot);

		Matrix33 f = rot.toRotationMatrix();
		Vector3 m_axis[3];
		m_axis[0] = f.col(0);
		m_axis[1] = f.col(1);
		m_axis[2] = f.col(2);
		double m[16];
		m[0] = m_axis[0][0];
		m[1] = m_axis[0][1];
		m[2] = m_axis[0][2];
		m[3] = 0.0;

		m[4] = m_axis[1][0];
		m[5] = m_axis[1][1];
		m[6] = m_axis[1][2];
		m[7] = 0.0;

		m[8] = m_axis[2][0];
		m[9] = m_axis[2][1];
		m[10] = m_axis[2][2];
		m[11] = 0.0;

		m[12] = trans[0];
		m[13] = trans[1];
		m[14] = trans[2];
		m[15] = 1.0;

		glPushMatrix();
		glMultMatrixd((double *)m);

		vector<int> &boneGrp = m_boneGrps[ibgp];
		for (int ibpt=0; ibpt<boneGrp.size(); ibpt++)
		{
			int bonePart = boneGrp[ibpt];
			int nF_in_bonePart = boneFcSegments[bonePart].size();
			setBoneMaterial(bonePart);
			vector<int> &bPFc = boneFcSegments[bonePart];

			glBegin(GL_TRIANGLES);
			for (int i=0; i<nF_in_bonePart; i++)
			{
				for (int j=0; j<3; j++)
				{
					glNormal3d(CN_bone(FN_bone(bPFc[i],j),0),CN_bone(FN_bone(bPFc[i],j),1),CN_bone(FN_bone(bPFc[i],j),2));
					glVertex3d(V_bone(F_bone(bPFc[i],j),0),V_bone(F_bone(bPFc[i],j),1),V_bone(F_bone(bPFc[i],j),2));
				} // for three vertex in triangle
			}// for triangles in the bone part
			glEnd();
		}// for bone parts in the same bone group
		glPopMatrix();
	}// for bone groups
}

void draw_skin()
{
	int nf = F_skin.rows();
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat_diff_skin);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_spec1);
	glMaterialfv(GL_FRONT, GL_SHININESS,mat_shine1);

	if(skin_in_wire)
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	}
	else
	{
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}

	glBegin(GL_TRIANGLES);
	for (int i=0; i<nf; i++)
	{
		for (int j=0; j<3; j++)
		{
			//glNormal3d(CN_skin(FN_skin(i,j),0),CN_skin(FN_skin(i,j),1),CN_skin(FN_skin(i,j),2));
			//glVertex3d(V_skin(F_skin(i,j),0),V_skin(F_skin(i,j),1),V_skin(F_skin(i,j),2));
			glVertex3d(Deformed_V_skin(F_skin(i,j),0),Deformed_V_skin(F_skin(i,j),1),Deformed_V_skin(F_skin(i,j),2));
		}
	}
	glEnd();
}

void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();

	gluLookAt(10.0f, 0.0f, 25.0f,   0.0f, 0.0f, 0.0f,   0.0f, 1.0f, 0.0f);
	glTranslatef(transx, -transy, transz);
	glRotatef(roty, 1.0f, 0.0f, 0.0f);
	glRotatef(rotx, 0.0f, 1.0f, 0.0f);

	if (bone_loaded && render_bone)
		draw_bone_parts();
	if (skin_loaded && render_skin)
		draw_skin();
	if (show_weight_bound)
		draw_weight_bound();

	glPopMatrix();
	TwDraw();
	glutSwapBuffers();
}

void reshape(int w, int h)
{
	glViewport(0,0,w,h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(65.0f, w/(double)h, 1.0f, 1000.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	TwWindowSize(w, h);
}

void keyboard(unsigned char key, int x, int y)
{
	if (!TwEventSpecialGLUT(key, x, y)){
		switch (key) {
		case 27:
			exit(0);
			break;
		case 'w':
		case 'W':
			transz += 1.0f;
			break;
		case 's':
		case 'S':
			transz -= 1.0f;
			break;
		case 'f':
		case 'F':
			break;
		case 'b':
		case 'B':
			break;
		}
	}// end if (!TwEventSpecialGLUT)
}
void motion(int x,int y)
{
	if ( !TwEventMouseMotionGLUT(x,y) )
	{
		if (LeftMouseDown)
		{
			//roty=(x-winWidth/2)*360.0/winWidth;
			//rotx=(y-winHeight/2)*360.0/winHeight;
			rotx += x-ori_x;
			if (rotx > 360.) rotx -= 360.;
			else if (rotx < -360.) rotx += 360.;
			roty += y-ori_y;
			if (roty > 360.) roty -= 360.;
			else if (roty < -360.) roty += 360.;
			ori_x = x; ori_y = y;
			glutPostRedisplay();
		}

		if (RightMouseDown)
		{
			transx += (x-ori_x)/50.;
			transy += (y-ori_y)/50.;
			ori_x = x; ori_y = y;
			glutPostRedisplay();
		}
	}//end if(!TwEventMouseMotionGLUT)
}

void idle()
{
}

void mouse(int button, int state, int x,int y)
{
	if (!TwEventMouseButtonGLUT(button, state, x, y))
	{
		if(state == GLUT_DOWN)
		{
			if (button == GLUT_LEFT_BUTTON)
			{
				LeftMouseDown = true;
				motion(ori_x=x,ori_y=y);
			}
			else if (button == GLUT_RIGHT_BUTTON)
			{
				RightMouseDown = true;
				motion(ori_x=x,ori_y=y);
			}
		}
		else if(state == GLUT_UP)
		{
			if (button == GLUT_LEFT_BUTTON)
			{
				LeftMouseDown = false;
			}
			else if (button == GLUT_RIGHT_BUTTON)
			{
				RightMouseDown = false;
			}
		}//end else if (GLUT_UP)
	}// end if (!TwEventMouseButtonGLUT)
}

void TW_CALL readObjs(void * param)
{
	readBone();
	readSkin();
	readBoneWeights();
	readMinCoordsCfg();
}

double unitSpline(double data, double lvl/* =1.0 */)
{
	//	MYRELEASEASSERT(data>=0.0 && data<=1.0);

	if(data <=0.5){
		return pow(data*2.0, lvl) * 0.5;
	}else{
		return 1.0 - pow((1.0-data)*2.0, lvl) * 0.5;
	}
}

void TW_CALL redistributeWeights_TW(void * param)
{
	Vector3 gradElbow = (radiusS + ulnaS)*0.5;
	Vector3 gradWrist = (radiusE + ulnaE)*0.5;
	Vector3 dir = gradWrist - gradElbow;
	double len = dir.norm();
	dir /= len;

	int nV = V_skin.rows();

	//summing up ulna and radius weights
	double localCoord, curWtSum;
	float curBBWRadius;
	float curBBWulna;

	for (int i=0; i<nV; ++i){
		const Vector3& curGeo = V_skin.row(i);
		localCoord = (curGeo - gradElbow).dot(dir);
		localCoord = localCoord / len;

		double lc;
		if( localCoord < 0.0){
			lc = 0.0;
		}else if(localCoord<=1.0){
			lc = unitSpline(localCoord, m_splineCtrlParam);
			//lc = localCoord;
		}else{
			lc = 1.0;
		}

		curBBWRadius = vertexWeights[i][1];
		curBBWulna = vertexWeights[i][2];
		curWtSum = curBBWRadius + curBBWulna;
		curBBWRadius = curWtSum * lc;
		curBBWulna = curWtSum * (1.0-lc);
		vertexWeights[i][1] = curBBWRadius;
		vertexWeights[i][2] = curBBWulna;
	}
}

void resetWeights()
{
	vertexWeights = vertexWeights_backup;
}

void TW_CALL resetWeights_TW(void * param)
{
	resetWeights();
}

void TW_CALL setBendAngle(const void *value, void *clientData)
{
	bendAng = *(const float *) value;
	setCoord(0, bendAng);
}

void TW_CALL getBendAngle(void *value, void *clientData)
{
	*(float *)value = bendAng;
}

void TW_CALL setTwistAngle(const void *value, void *clientData)
{
	twistAng = *(const float *) value;
	setCoord(1, twistAng);
}

void TW_CALL getTwistAngle(void *value, void *clientData)
{
	*(float *)value = twistAng;
}

void myTwGuiInit()
{
	TwInit(TW_OPENGL, NULL);

	// Set GLUT event callbacks
	glutPassiveMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT);
	glutSpecialFunc((GLUTspecialfun)TwEventSpecialGLUT);
	TwGLUTModifiersFunc(glutGetModifiers);

	// AntTweakBar instance and its items
	TwBar *simpleCtrlBar = TwNewBar("SimpleCtrlBar");
	TwAddButton(simpleCtrlBar, "Load_Bone_Arm", readObjs, NULL, "");
	TwAddButton(simpleCtrlBar, "Redistribute Weights", redistributeWeights_TW, NULL, "");
	TwAddButton(simpleCtrlBar, "Reset Weights", resetWeights_TW, NULL, "");
	TwAddVarRW(simpleCtrlBar, "Show Arm",TW_TYPE_BOOLCPP, &render_skin,"");
	TwAddVarRW(simpleCtrlBar, "Show Bone",TW_TYPE_BOOLCPP, &render_bone,"");
	TwAddVarRW(simpleCtrlBar, "Skin in Wire",TW_TYPE_BOOLCPP, &skin_in_wire,"");
	TwAddVarRW(simpleCtrlBar, "Show Weight Bound",TW_TYPE_BOOLCPP, &show_weight_bound,"");
	TwAddVarCB(simpleCtrlBar, "bendAngle", TW_TYPE_FLOAT, setBendAngle, getBendAngle, NULL,"");
	TwAddVarCB(simpleCtrlBar, "twistAngle", TW_TYPE_FLOAT, setTwistAngle, getTwistAngle, NULL,"");
	TwAddButton(simpleCtrlBar, "genTet_BBW", genTet_BBW, NULL, "");
}

int main(int argc, char ** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGBA );
	glutInitWindowPosition(100,100);
	glutInitWindowSize(window_width, window_height);
	glutCreateWindow("Skinning Viewer");
	glutTimerFunc(deltaTime*10,AdvanceFrame,0);

	init();
	myTwGuiInit();

	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutReshapeFunc(reshape);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutIdleFunc(idle);

	glutMainLoop();
	TwTerminate();
	return 0;
}
