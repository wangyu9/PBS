// IGL Viewer - Copyright (c) 2013 ETH Zurich. All rights reserved.
// This file contains a reader for the text file format exported by matlab/meshplot.m
#ifndef MATLABIO_H
#define MATLABIO_H

#include <iostream>
/*
 For the file format description see meshplot.m
 */

#include <string>
#include <vector>
#include <fstream>
#include <iostream>

#ifdef MATLAB_LINK
#include <mat.h>
#endif

using namespace std;

class MatlabIO
{
public:
	
	typedef vector<vector<double> > vvd;
  //	typedef vector<double > vd;
	typedef vector<vector<size_t> > vvi;
  
	vvd V; // Point coordinates
	vvd F; // Face indices
	vvd VC; // Vertex Colors
	vvd TC; // Vertex Texture Coordinates
	vvd TF; // Vertex Texture Flag (valid or not)
	vvd VP; // Vertex Scalar Property
	vvd FC; // Face Colors
	vvd FP; // Face Scalar Property
	vvd VN; // Vertex Colors
	vvd FN; // Face Colors
	vvd L; // lines (p1x,p1y,p1z,p2x,p2y,p2z,r,g,b)
	vvd P; // points (x,y,z,radius,r,g,b)
	vvd TEXTP; // anchor for text
	vector<string> TEXTS; // text attached to TEXTP
	
	MatlabIO()
	{
		error = true;
	}
	
#ifdef MATLAB_LINK
	int readMAT(const mxArray* mesh)
	{
		// Read vertices positions
		{
			mxArray* mat = mxGetField(mesh, 0, "V");
			if (mat)
			{
				double* ptr = mxGetPr(mat);
				
				int m = mxGetM(mat);
				int n = mxGetN(mat);
				
				V.clear();
				V.resize(m);
				for(int j=0;j<n;j++)
					for(int i=0;i<m;++i)
						V[i].push_back(*(ptr++));
			}
		}
		
		// Read faces indices
		{
			mxArray* mat = mxGetField(mesh, 0, "F");
			if (mat)
			{
				double* ptr = mxGetPr(mat);
				
				int m = mxGetM(mat);
				int n = mxGetN(mat);
				
				F.clear();
				F.resize(m);
				for(int j=0;j<n;j++)
					for(int i=0;i<m;++i)
						F[i].push_back((size_t)(*(ptr++)-1));
			}
		}
		
		// Read vertices colors
		{
			mxArray* mat = mxGetField(mesh, 0, "VC");
			if (mat)
			{
				double* ptr = mxGetPr(mat);
				
				int m = mxGetM(mat);
				int n = mxGetN(mat);
				
				VC.clear();
				VC.resize(m);
				for(int j=0;j<n;j++)
					for(int i=0;i<m;++i)
						VC[i].push_back(*(ptr++));
			}
		}
		
		// Read vertices texture coordinates
		{
			mxArray* mat = mxGetField(mesh, 0, "TC");
			if (mat)
			{
				double* ptr = mxGetPr(mat);
				
				int m = mxGetM(mat);
				int n = mxGetN(mat);
				
				TC.clear();
				TC.resize(m);
				for(int j=0;j<n;j++)
					for(int i=0;i<m;++i)
						TC[i].push_back(*(ptr++));
			}
		}
		
		
		// Read vertices texture flags
		{
			mxArray* mat = mxGetField(mesh, 0, "TF");
			if (mat)
			{
				double* ptr = mxGetPr(mat);
				
				int m = mxGetM(mat);
				int n = mxGetN(mat);
				
				TF.clear();
				TF.resize(m);
				for(int j=0;j<n;j++)
					for(int i=0;i<m;++i)
						TF[i].push_back((size_t)(*(ptr++)));
			}
		}
    
    // Read vertices property
		{
			mxArray* mat = mxGetField(mesh, 0, "VP");
			if (mat)
			{
				double* ptr = mxGetPr(mat);
				
				int m = mxGetM(mat);
				int n = mxGetN(mat);
				
				VP.clear();
				VP.resize(m);
				for(int j=0;j<n;j++)
					for(int i=0;i<m;++i)
						VP[i].push_back(*(ptr++));
			}
		}
		
    // Read face colors
		{
			mxArray* mat = mxGetField(mesh, 0, "FC");
			if (mat)
			{
				double* ptr = mxGetPr(mat);
				
				int m = mxGetM(mat);
				int n = mxGetN(mat);
				
				FC.clear();
				FC.resize(m);
				for(int j=0;j<n;j++)
					for(int i=0;i<m;++i)
						FC[i].push_back(*(ptr++));
			}
		}
    
    // Read face property
		{
			mxArray* mat = mxGetField(mesh, 0, "FP");
			if (mat)
			{
				
				double* ptr = mxGetPr(mat);
				
				int m = mxGetM(mat);
				int n = mxGetN(mat);
				
				FP.clear();
				FP.resize(m);
				for(int j=0;j<n;j++)
					for(int i=0;i<m;++i)
						FP[i].push_back(*(ptr++));
			}
		}
    
		// Read vertices normals
		{
			mxArray* mat = mxGetField(mesh, 0, "VN");
			if (mat)
			{
				double* ptr = mxGetPr(mat);
				
				int m = mxGetM(mat);
				int n = mxGetN(mat);
				
				VN.clear();
				VN.resize(m);
				for(int j=0;j<n;j++)
					for(int i=0;i<m;++i)
						VN[i].push_back(*(ptr++));
			}
		}
		
		// Read faces normals
		{
			mxArray* mat = mxGetField(mesh, 0, "FN");
			if (mat)
			{
				double* ptr = mxGetPr(mat);
				
				int m = mxGetM(mat);
				int n = mxGetN(mat);
				
				FN.clear();
				FN.resize(m);
				for(int j=0;j<n;j++)
					for(int i=0;i<m;++i)
						FN[i].push_back(*(ptr++));
			}
		}
		
		// Read lines
		{
			mxArray* mat = mxGetField(mesh, 0, "L");
			if (mat)
			{
				double* ptr = mxGetPr(mat);
				
				int m = mxGetM(mat);
				int n = mxGetN(mat);
				
				L.clear();
				L.resize(m);
				for(int j=0;j<n;j++)
					for(int i=0;i<m;++i)
						L[i].push_back(*(ptr++));
			}
		}
		
		// Read points
		{
			mxArray* mat = mxGetField(mesh, 0, "P");
			if (mat)
			{
				double* ptr = mxGetPr(mat);
				
				int m = mxGetM(mat);
				int n = mxGetN(mat);
				
				P.clear();
				P.resize(m);
				for(int j=0;j<n;j++)
					for(int i=0;i<m;++i)
						P[i].push_back(*(ptr++));
			}
		}
		
		// Read text positions
		{
			mxArray* mat = mxGetField(mesh, 0, "TEXTP");
			if (mat)
			{
				double* ptr = mxGetPr(mat);
				
				int m = mxGetM(mat);
				int n = mxGetN(mat);
				
				TEXTP.clear();
				TEXTP.resize(m);
				for(int j=0;j<n;j++)
					for(int i=0;i<m;++i)
						TEXTP[i].push_back(*(ptr++));
			}
		}
		
		// Read text labels
		{
			mxArray* mat = mxGetField(mesh, 0, "TEXTS");
			if (mat)
			{
				int m = mxGetM(mat);
				
				TEXTS.clear();
				TEXTS.resize(m);
				for(int i=0;i<m;++i)
				{
					char temp[1000];
					mxArray* c = mxGetCell(mat,i);
					mxGetString(c,temp,1000);
					TEXTS[i] = string(temp);
				}
				
			}
		}
	}
	
	int readMAT(const char* file) // require to link with matlab!
	{
		/*
		 * Open file
		 */
		MATFile* pmat = matOpen(file, "r");
		if (pmat == NULL) {
			printf("Error reopening file %s\n", file);
			return true;
		}
		
		/*
		 * Read the structures that contains the mesh
		 */
		mxArray* mesh = matGetVariable(pmat, "mesh");
		if (mesh == NULL) {
			printf("Error reading the Mesh structure\n");
			return true;
		}
		
		readMAT(mesh);
		
		/* clean up before exit */
		mxDestroyArray(mesh);
		
		if (matClose(pmat) != 0) {
			printf("Error closing file %s\n",file);
			error = true;
			return true;
		}
		error = false;
		printf("Done\n");
		return false;
	}
#endif
  
	
	int serialize(const char* file)
	{
		ofstream s(file);
    
		// matrixdouble matrixint vectorstring
		int m, v;
		m = 13;
		v = 1;
		s << m << " " << v << endl;
		saveMatrix("V", V, s);
		saveMatrix("F", F, s);
		saveMatrix("VC", VC, s);
		saveMatrix("TC", TC, s);
		saveMatrix("TF", TF, s);
		saveMatrix("FC", FC, s);
		saveMatrix("VN", VN, s);
		saveMatrix("FN", FN, s);
		saveMatrix("L", L, s);
		saveMatrix("P", P, s);
		saveMatrix("TEXTP", TEXTP, s);
		saveMatrix("VP", VP, s);
		saveMatrix("FP", FP, s);
		saveVectorStrings("TEXTS",TEXTS, s);
		
		s.close();
    
		return 1;
	}
	
	void loadMatrix(ifstream& s)
	{
		string name;
		vvd v;
		loadMatrix(name, v, s);
    cerr << "Read " << name << endl;
		if (name.compare("V") == 0) V = v;
		if (name.compare("F") == 0) F = v;
		if (name.compare("VC") == 0) VC = v;
		if (name.compare("TC") == 0) TC = v;
		if (name.compare("TF") == 0) TF = v;
		if (name.compare("FC") == 0) FC = v;
		if (name.compare("VN") == 0) VN = v;
		if (name.compare("FN") == 0) FN = v;
		if (name.compare("L") == 0) L = v;
		if (name.compare("P") == 0) P = v;
		if (name.compare("TEXTP") == 0) TEXTP = v;
    if (name.compare("VP") == 0) VP = v;
    if (name.compare("FP") == 0) FP = v;
	}
  
	void loadVectorStrings(ifstream& s)
	{
		string name;
		vector<string> v;
		loadVectorStrings(name, v, s);
		if (name.compare("TEXTS") == 0) TEXTS = v;
	}
  
	int deserialize(const char* file)
	{
		ifstream s(file);
    // Check that file was actually opened, if not then set error flag and return false
    if (s.is_open() == false)
    {
      error = true;
      return false;
    }
    
    
		int m, v;
		s >> m >> v;
		
		for(int i=0; i<m; ++i)
			loadMatrix(s);
    
		for(int i=0; i<v; ++i)
			loadVectorStrings(s);
		s.close();
		error = false;
    
		return 1;
	}
	
	template<class T>
	static void saveMatrix(string name, vector<vector<T> >& v, ofstream& s)
  {
		int m = v.size();
		if (m == 0)
		{
			s << name << " " << 0 << " " << 0 << endl;
			return;
		}
		
		int n = v[0].size();
		if (n==0)
		{
			s << name << " " << 0 << " " << 0 << endl;
			return;
		}
		
    s << name << " " << m << " " << n << endl;
    for (int i = 0; i < m; ++i)
		{
			for (int j = 0; j < n; ++j)
				s << v[i][j] << " ";
			s << endl;
		}
    s << endl;
  }
	
	template<class T>
	static void loadMatrix(string& name, vector<vector<T> >& v, ifstream& s)
  {
		int m;
		int n;
		
		s >> name >> m >> n;
		
		v.clear();
		v.resize(m);
		
    for (int i = 0; i < m; ++i)
		{
			for (int j = 0; j < n; ++j)
			{
				T temp;
				s >> temp;
				v[i].push_back(temp);
			}
		}
  }
	
	
	template<class T>
	static void loadVector(string& name, vector<T >& v, ifstream& s)
  {
		int m;
		
		s >> name >> m;
		
		cout << name << endl;
		
		v.clear();
		v.resize(m);
		
    for (int i = 0; i < m; ++i)
		{
			T temp;
			s >> temp;
			v[i] = temp;
		}
  }
	
  static void saveVectorStrings(string name, vector<string>& v, ofstream& s)
  {
		int m = v.size();
		if (m == 0)
		{
			s << name << " " << 0 << endl;
			return;
		}
		
    s << name << " " << m << endl;
    for (int i = 0; i < m; ++i)
		{
			s << v[i] << endl;
		}
    s << endl;
  }
	
	static void loadVectorStrings(string& name, vector<string>& v, ifstream& s)
  {
		int m;
		
		s >> name >> m;
    
    // Throw away the current line
    string temp;
    getline(s,temp);
    
		cout << "Read Vector String:" << name << endl;
		v.clear();
		v.resize(m);
		
    for (int i = 0; i < m; ++i)
		{
			getline(s,temp);
			v[i] = temp;
      
			cout << "Printing value " << i << " ---> " << v[i] << endl;
		}
  }
	
	bool error;
};

#endif
