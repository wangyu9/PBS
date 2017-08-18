#include "unique_plus.h"

#include <igl/sort.h>
#include <igl/IndexComparison.h>
#include <igl/SortableRow.h>
#include <igl/sortrows.h>
#include <igl/list_to_matrix.h>

#include <algorithm>
#include <iostream>
#include <map>

template <typename DerivedA, typename DerivedIA, typename DerivedIC>
IGL_INLINE void unique_rows_plus(
	const Eigen::PlainObjectBase<DerivedA>& A,
	Eigen::PlainObjectBase<DerivedA>& C,
	Eigen::PlainObjectBase<DerivedIA>& IA,
	Eigen::PlainObjectBase<DerivedIC>& IC)
{
	using namespace std;
	using namespace igl;
	using namespace Eigen;
	VectorXi IM;
	Eigen::PlainObjectBase<DerivedA> sortA;
	sortrows(A,true,sortA,IM);


	vector<int> vIA(sortA.rows());
	for(int i=0;i<(int)sortA.rows();i++)
	{
		vIA[i] = i;
	}
	vIA.erase(
		std::unique(
		vIA.begin(),
		vIA.end(),
		igl::IndexRowEquals<const Eigen::PlainObjectBase<DerivedA> &>(sortA)),vIA.end());

	IC.resize(A.rows(),1);
	{
		int j = 0;
		for(int i = 0;i<(int)sortA.rows();i++)
		{
			if(sortA.row(vIA[j]) != sortA.row(i))
			{
				j++;
			}
			IC(IM(i,0),0) = j;
		}
	}
	C.resize(vIA.size(),A.cols());
	IA.resize(vIA.size(),1);
	// Reindex IA according to IM
	for(int i = 0;i<(int)vIA.size();i++)
	{
		IA(i,0) = IM(vIA[i],0);
		C.row(i) = A.row(IA(i,0));
	}
}


