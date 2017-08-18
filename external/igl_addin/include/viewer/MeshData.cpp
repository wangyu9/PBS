#include "MeshData.h"

#include "FileDialog.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <sstream>
#include <iomanip>

#include <algorithm>
using namespace std;

#include "igl/readOBJ.h"//#include "libigl_removed/readOBJ.h"//#include "igl/readOBJ.h"
#include "igl/readOFF.h"
#include "igl/readMESH.h"//wangyu
#include "igl/readDMAT.h"//wangyu

#ifndef _NOMATLAB_
#include "matlabIO.h"
#endif


#include "igl/writeOBJ.h"
#include "igl/writeOFF.h"
#include "igl/writeMESH.h"//wangyu
#include "igl/writeDMAT.h"//wangyu


MeshData::MeshData()
{
	mesh_alpha = 1.;//added by wangyu

	number_of_vertices = 0;
	number_of_faces = 0;
	number_of_tets = 0;

	vertices = new Eigen::MatrixXd(0, 3);
	faces = new Eigen::MatrixXi(0, 3);
	tets = new Eigen::MatrixXi(0, 3);
	face_colors = new Eigen::MatrixXd(0, 3);
	vertex_colors = new Eigen::MatrixXd(0, 3);
	vertex_normals = new Eigen::MatrixXd(0, 3);
	face_normals = new Eigen::MatrixXd(0, 3);

	face_property = new VectorX(0, 1);
	vertex_property = new VectorX(0, 1);



	corner_normals = new Eigen::MatrixXd(0, 3);

	fNormIndices = new Eigen::MatrixXi(0, 3);

	texCoords = new Eigen::MatrixXd(0, 2);

	fTexIndices = new Eigen::MatrixXi(0, 3);

	filename = std::string("");
}

MeshData::~MeshData()
{
	if (vertices)
	{
		delete vertices;
		vertices = 0;
	}
	if (faces)
	{
		delete faces;
		faces = 0;
	}
	if (face_normals)
	{
		delete face_normals;
		face_normals = 0;
	}
	if (vertex_normals)
	{
		delete vertex_normals;
		vertex_normals = 0;
	}
	if (fNormIndices)
	{
		delete fNormIndices;
		fNormIndices = 0;
	}
	if (vertex_colors)
	{
		delete vertex_colors;
		vertex_colors = 0;
	}
	if (face_colors)
	{
		delete face_colors;
		face_colors = 0;
	}
	if (vertex_property)
	{
		delete vertex_property;
		vertex_property = 0;
	}
	if (face_property)
	{
		delete face_property;
		face_property = 0;
	}
}

#include <file_helper.h>

bool MeshData::load_texture_coordinates_from_file(const char* file_name)
{
	std::string mesh_file_name_string = std::string(file_name);
	std::string extension;

	if (!extract_extension(mesh_file_name_string, extension))
		return false;

	Eigen::MatrixXd V, N;
	Eigen::MatrixXd TC;
	Eigen::MatrixXi F, FTC, FN;

	//Eigen::MatrixXd V, TC, N;
	//Eigen::MatrixXi F, FTC, FN;

	if (extension == "obj" || extension == "OBJ")
	{// this is necessary since the else will match the closet if
		//if (!(igl::readOBJ(mesh_file_name_string, *vertices, *texCoords, *corner_normals, *faces, *fTexIndices, *fNormIndices)))
		if (!(igl::readOBJ(mesh_file_name_string, V, TC, N, F, FTC, FN)))
			return false;
	}
	else
	{
		// unrecognized file type
		printf("Error: %s is not a recognized texture coordinates file type.\n", extension.c_str());
		return false;
	}

	if (F.rows()!=faces->rows())
	{
		printf("Error: loaded texture size does not match existing face size!\n");
		return false;
	}

	// TODO: maybe also deal with the remaining ones: *texCoords, *corner_normals, *faces, *fTexIndices, *fNormIndices
	*texCoords = TC;
	*fTexIndices = FTC;
	*fNormIndices = FN;

	return true;
}

bool MeshData::load_mesh_from_file(const char* mesh_file_name)
{
	std::string mesh_file_name_string = std::string(mesh_file_name);
	std::string extension;

	if (!extract_extension(mesh_file_name_string, extension))
	{
		return false;
	}

	filename = mesh_file_name;
	clear_mesh();

	if (extension == "off" || extension == "OFF")
	{
		if (!igl::readOFF(mesh_file_name_string, *vertices, *faces))
		{
			return false;
		}
	}
	else if (extension == "mesh" || extension == "MESH")
	{//added by wangyu
		if (!igl::readMESH(mesh_file_name_string, *vertices, *tets, *faces))
		{
			return false;
		}
	}
	else if (extension == "obj" || extension == "OBJ")
	{
		if (!(igl::readOBJ(mesh_file_name_string, *vertices, *texCoords, *corner_normals, *faces, *fTexIndices, *fNormIndices)))
		//old use of readOBJ, note now different order!! if (!(igl::readOBJ(mesh_file_name_string, *vertices, *faces, *corner_normals, *fNormIndices, *texCoords, *fTexIndices)))
		{
#if 0
			vector< vector< IndexType > > faces_poly;
			if (igl::readOBJPoly(mesh_file_name_string, *vertices, faces_poly, *corner_normals, *fNormIndices, *texCoords, *fTexIndices))
			{
				// Triangulate all faces
				vector<vector<int> > tri;
				vector< Vector3 > tri_normals;

				for (unsigned i = 0; i < faces_poly.size(); ++i)
				{
					vector<IndexType> f = faces_poly[i];
					int iter = f.size() - 2;

					Vector3 e1 = vertices->row(f[0 + 2]) - vertices->row(f[0]);
					Vector3 e2 = vertices->row(f[0 + 1]) - vertices->row(f[0]);
					Vector3 n = e2.cross(e1);
					n.normalize();

					for (int j = 0; j < iter; ++j)
					{
						vector<int> t(3);
						t[0] = f[0];
						t[1] = f[1];
						t[2] = f[2];
						f.erase(f.begin() + 1);
						tri.push_back(t);
						tri_normals.push_back(n);
					}
					assert(f.size() == 2);

				}

				faces->resize(tri.size(), 3);
				face_normals->resize(tri.size(), 3);
				for (unsigned i = 0; i < tri.size(); ++i)
				{
					(*faces)(i, 0) = tri[i][0];
					(*faces)(i, 1) = tri[i][1];
					(*faces)(i, 2) = tri[i][2];
					face_normals->row(i) = tri_normals[i];
				}

				// Add the polygonal edges
				//        lines.clear();
				for (unsigned i = 0; i < faces_poly.size(); ++i)
				{
					vector<IndexType> f = faces_poly[i];
					for (unsigned j = 0; j < f.size(); ++j)
					{
						vector<double> t(9);
						t[0] = (*vertices)(f[j], 0);
						t[1] = (*vertices)(f[j], 1);
						t[2] = (*vertices)(f[j], 2);
						t[3] = (*vertices)(f[(j + 1) % f.size()], 0);
						t[4] = (*vertices)(f[(j + 1) % f.size()], 1);
						t[5] = (*vertices)(f[(j + 1) % f.size()], 2);
						t[6] = 1; t[7] = 0; t[8] = 0;
						lines.push_back(t);
					}
				}
			}
			else
#endif
				return false;
		}
	}
	else if (extension == "mp" || extension == "MP")
	{
		// delete the opened file on exit
		bool delete_on_exit = true;
		MatlabIO mio;
		mio.deserialize(mesh_file_name);
		if (mio.error)
		{
			return false;
		}
		else
		{
			vertices->resize(mio.V.size(), 3);
			for (unsigned int i = 0; i < mio.V.size(); ++i)
				vertices->row(i) << mio.V[i][0], mio.V[i][1], mio.V[i][2];

			if (mio.F.size() == 0)
				faces->resize(mio.F.size(), 3);
			else
				faces->resize(mio.F.size(), mio.F[0].size());

			for (unsigned int i = 0; i < mio.F.size(); ++i)
				for (unsigned int j = 0; j < mio.F[0].size(); ++j)
					(*faces)(i, j) = mio.F[i][j];

			vertex_colors->resize(mio.VC.size(), 3);
			for (unsigned int i = 0; i < mio.VC.size(); ++i)
				vertex_colors->row(i) << mio.VC[i][0], mio.VC[i][1], mio.VC[i][2];

			vertex_property->resize(mio.VP.size(), 1);
			for (unsigned int i = 0; i < mio.VP.size(); ++i)
				(*vertex_property)(i, 0) = mio.VP[i][0];

			texCoords->resize(mio.TC.size(), 3);
			for (unsigned int i = 0; i < mio.TC.size(); ++i)
				texCoords->row(i) << mio.TC[i][0], mio.TC[i][1];

			std::vector<vector<double> > tf = mio.TF;
			if (tf.size())
			{
				fUseTexture.clear();
				fUseTexture.resize(faces->rows(), true);
				for (int fi = 0; fi < faces->rows(); ++fi)
					for (int vit = 0; vit < faces->cols(); ++vit)
					{
						fUseTexture[fi] = fUseTexture[fi] && (tf[(*faces)(fi, vit)][0] > 0);
					}
			}

			face_colors->resize(mio.FC.size(), 3);
			for (unsigned int i = 0; i < mio.FC.size(); ++i)
				face_colors->row(i) << mio.FC[i][0], mio.FC[i][1], mio.FC[i][2];

			face_property->resize(mio.FP.size(), 1);
			for (unsigned int i = 0; i < mio.FP.size(); ++i)
				(*face_property)(i, 0) = mio.FP[i][0];


			vertex_normals->resize(mio.VN.size(), 3);
			for (unsigned int i = 0; i < mio.VN.size(); ++i)
				vertex_normals->row(i) << mio.VN[i][0], mio.VN[i][1], mio.VN[i][2];

			face_normals->resize(mio.FN.size(), 3);
			for (unsigned int i = 0; i < mio.FN.size(); ++i)
				face_normals->row(i) << mio.FN[i][0], mio.FN[i][1], mio.FN[i][2];

			lines = mio.L;
			points = mio.P;
			textp = mio.TEXTP;
			texts = mio.TEXTS;

			if (delete_on_exit)
				remove(mesh_file_name);
		}
	}
	else
	{
		// unrecognized file type
		printf("Error: %s is not a recognized file type.\n", extension.c_str());
		return false;
	}

	//added by wangyu
	rest_vertices = *vertices;

	number_of_vertices = vertices->rows();
	number_of_faces = faces->rows();
	number_of_tets = tets->rows();

	return true;
}

bool MeshData::save_mesh_to_file(const char* mesh_file_name)
{
	std::string mesh_file_name_string(mesh_file_name);
	size_t last_dot = mesh_file_name_string.rfind('.');
	if (last_dot == std::string::npos)
	{
		// No file type determined
		printf("Error: No file extension found in %s\n", mesh_file_name);
		return false;
	}
	std::string extension = mesh_file_name_string.substr(last_dot + 1);
	if (extension == "off" || extension == "OFF")
	{
		return igl::writeOFF(mesh_file_name_string, *vertices, *faces);
	}
	else if (extension == "obj" || extension == "OBJ")
	{
		return igl::writeOBJ(mesh_file_name_string, *vertices, *faces, *corner_normals, *fNormIndices, *texCoords, *fTexIndices);
	}
	else if (extension == "mesh" || extension == "MESH")//added by wangyu
	{
		return igl::writeMESH(mesh_file_name_string, *vertices, *tets, *faces);

	}
	else
	{
		// unrecognized file type
		printf("Error: %s is not a recognized file type.\n", extension.c_str());
		return false;
	}
	return true;
}

void MeshData::save_tets_to_file(const char* mesh_file_name)
{
	igl::writeDMAT(mesh_file_name, *tets);
}

bool MeshData::load_pose_mesh_from_file(const char* mesh_file_name)
{
	std::string mesh_file_name_string = std::string(mesh_file_name);
	filename = mesh_file_name;
	size_t last_dot = mesh_file_name_string.rfind('.');
	if (last_dot == std::string::npos)
	{
		// No file type determined
		printf("Error: No file extension found in %s\n", mesh_file_name);
		return false;
	}

	Eigen::MatrixXd pose_vertices;
	Eigen::MatrixXi pose_faces;

	Eigen::MatrixXd pose_corner_normals;
	Eigen::MatrixXi pose_fNormIndices;
	Eigen::MatrixXd pose_texCoords;
	Eigen::MatrixXi pose_fTexIndices;

	std::string extension = mesh_file_name_string.substr(last_dot + 1);
	if (extension == "off" || extension == "OFF")
	{
		if (!igl::readOFF(mesh_file_name_string, pose_vertices, pose_faces))
		{
			return false;
		}
	}
	else if (extension == "obj" || extension == "OBJ")
	{
		if (!igl::readOBJ(mesh_file_name_string, pose_vertices, pose_texCoords, pose_corner_normals, pose_faces, pose_fTexIndices, pose_fNormIndices))
		{
			return false;
		}

		//if(!(igl::readOBJ(mesh_file_name_string, *vertices, *faces, *corner_normals, *fNormIndices, *texCoords, *fTexIndices)))
		//{
		//	vector< vector< IndexType > > faces_poly;
		//	if(igl::readOBJPoly(mesh_file_name_string, *vertices, faces_poly, *corner_normals, *fNormIndices, *texCoords, *fTexIndices))
		//	{
		//		// Triangulate all faces
		//		vector<vector<int> > tri;
		//		vector< Vector3 > tri_normals;

		//		for (unsigned i=0; i<faces_poly.size(); ++i)
		//		{
		//			vector<IndexType> f = faces_poly[i];
		//			int iter = f.size()-2;

		//			Vector3 e1 = vertices->row(f[0+2]) - vertices->row(f[0]);
		//			Vector3 e2 = vertices->row(f[0+1]) - vertices->row(f[0]);
		//			Vector3 n = e2.cross(e1);
		//			n.normalize();

		//			for (int j=0; j<iter; ++j)
		//			{
		//				vector<int> t(3);
		//				t[0] = f[0];
		//				t[1] = f[1];
		//				t[2] = f[2];
		//				f.erase(f.begin()+1);
		//				tri.push_back(t);
		//				tri_normals.push_back(n);
		//			}
		//			assert(f.size() == 2);

		//		}

		//		faces->resize(tri.size(),3);
		//		face_normals->resize(tri.size(),3);
		//		for (unsigned i=0; i < tri.size();++i)
		//		{
		//			(*faces)(i,0) = tri[i][0];
		//			(*faces)(i,1) = tri[i][1];
		//			(*faces)(i,2) = tri[i][2];
		//			face_normals->row(i) = tri_normals[i];
		//		}

		//		// Add the polygonal edges
		//		//        lines.clear();
		//		for (unsigned i=0; i<faces_poly.size(); ++i)
		//		{
		//			vector<IndexType> f = faces_poly[i];
		//			for (unsigned j=0; j<f.size(); ++j)
		//			{
		//				vector<double> t(9);
		//				t[0] = (*vertices)(f[j  ],0);
		//				t[1] = (*vertices)(f[j  ],1);
		//				t[2] = (*vertices)(f[j  ],2);
		//				t[3] = (*vertices)(f[(j+1)%f.size()],0);
		//				t[4] = (*vertices)(f[(j+1)%f.size()],1);
		//				t[5] = (*vertices)(f[(j+1)%f.size()],2);
		//				t[6] = 1; t[7] = 0; t[8] = 0;
		//				lines.push_back(t);
		//			}
		//		}
		//	}
		//	else
		//		return false;
		//}
	}
	else
	{
		// unrecognized file type
		printf("Error: %s is not a recognized file type.\n", extension.c_str());
		return false;
	}


	if (pose_vertices.rows() == vertices->rows()
		&& pose_vertices.cols() == vertices->cols()
		&& pose_faces.rows() == faces->rows()
		&& pose_faces.cols() == faces->cols())
	{
		// a more careful check should be that pose_faces should equal to faces. Ignore it here
		*vertices = pose_vertices;
	}
	else
	{
		printf("Error Open_pose_mesh(): the pose mesh has different size with the existing mesh, pose mesh is not set!");
		return false;
	}


	//	number_of_vertices = vertices->rows();
	//	number_of_faces = faces->rows();
	//
	//	show_faces = true;
	//	normals_type = PER_VERTEX;//wangyu PER_FACE
	//



	//	get_scale_and_shift_to_fit_mesh(vertices,zoom,g_Translation);
	//	radius = 1/zoom;
	//
	//	if (face_normals->rows() == 0 || face_normals->cols() != 3)
	//		recompute_face_normals();
	//
	//	if (vertex_normals->rows() == 0  || vertex_normals->cols() != 3)
	//		recompute_vertex_normals();
	//
	//	std::vector<std::vector<IndexType> > vfi;
	//	igl::vf(*vertices, *faces, vertex_to_faces, vfi);
	//	igl::adjacency_list(*faces, vertex_to_vertices);
	//	if (corner_normals->rows() == 0  || corner_normals->cols() != 3 || fNormIndices->size() != faces->size())
	//		recompute_corner_normals();
	//
	//	if (face_colors->rows() == 0  || face_colors->cols() != 3)
	//		compute_face_colors(face_property, face_colors);
	//	if (vertex_colors->rows() == 0 || vertex_colors->cols() != 3)
	//		compute_vertex_colors(vertex_property, vertex_colors);
	//
	//	if(vertex_property->rows())
	//		compute_isolevels();
	//
	//#ifndef PREVIEW3D_NO_SHADERS
	//	if (face_colors->rows() > 0)
	//	{
	//		shader_mode = OFF;
	//		shader_id = 0;
	//	}
	//#endif  
	//
	//	if(fTexIndices->size() != faces->size())
	//	{
	//		fTexIndices->resize(faces->rows(),faces->cols());
	//		for (int fi = 0; fi<faces->rows(); ++fi)
	//			fTexIndices->row(fi) = faces->row(fi);
	//		has_wedge_texture = false;
	//	}
	//	else
	//		has_wedge_texture = (*faces != *fTexIndices);
	//
	//	if(texCoords->rows()&&bFlipYCoord)//wangyu
	//		flipCoord(texCoords,1,texCoords);
	//	resetTexCoords(!texCoords->rows(), !fUseTexture.size());
	//
	//	// compute mesh dimension properties
	//	compute_bounding_box(vertices, aabb_min, aabb_max);
	//	compute_centroid(vertices, aabb_center);
	//	diameter = (aabb_min - aabb_max).norm();
	//	avg_edge_length = 0;
	//	for(int f=0;f<faces->rows();f++)
	//	{
	//		for(int b=0, e=2;b<3;b++,e=b)
	//		{
	//			Eigen::Matrix<ScalarType,1,3> begin = vertices->row(faces->row(f)[b]);
	//			Eigen::Matrix<ScalarType,1,3> end = vertices->row(faces->row(f)[e]);
	//			avg_edge_length += (begin - end).norm();
	//		}
	//	}
	//	avg_edge_length /= faces->rows()*3;
	//
	//	for (unsigned int i = 0; i<PluginManager().plugin_list_.size(); ++i)
	//		PluginManager().plugin_list_[i]->init(this);

	return true;
}

void MeshData::clear_mesh()
{
	*vertices = Eigen::MatrixXd(0, 3);
	*faces = Eigen::MatrixXi(0, 3);
	*tets = Eigen::MatrixXi(0, 4);//wangyu
	*face_colors = Eigen::MatrixXd(0, 3);
	*vertex_colors = Eigen::MatrixXd(0, 3);
	*vertex_normals = Eigen::MatrixXd(0, 3);
	*face_normals = Eigen::MatrixXd(0, 3);

	*face_property = VectorX(0, 1);
	*vertex_property = VectorX(0, 1);

	*corner_normals = Eigen::MatrixXd(0, 3);

	*fNormIndices = Eigen::MatrixXi(0, 3);

	*texCoords = Eigen::MatrixXd(0, 2);

	*fTexIndices = Eigen::MatrixXi(0, 3);

	fUseTexture.clear();

	//disabled by wangyu //for (unsigned int i = 0; i<PluginManager().plugin_list_.size(); ++i)
	//  PluginManager().plugin_list_[i]->init(this);

	lines.clear();
	points.clear();
	textp.clear();
}

const Eigen::MatrixXd MeshData::GetV() const
{
	return *vertices;
}

const Eigen::MatrixXi MeshData::GetT() const
{
	return tets->cast<int>();
}

const Eigen::MatrixXi MeshData::GetF() const
{
	return faces->cast<int>();
}

const Eigen::MatrixXd MeshData::GetVR() const
{
	return rest_vertices.cast<double>();
}

void MeshData::SetV(const Eigen::MatrixXd& VV)
{
	*vertices = VV;
}



//void MeshData::SetF(const Eigen::MatrixXi& FF)
//{
//	*faces = FF;
//}