


template <typename DerivedX, typename DerivedIX>
IGL_INLINE void igl::sortrows(
	const Eigen::PlainObjectBase<DerivedX>& X,
	const int dim,
	const bool ascending,
	Eigen::PlainObjectBase<DerivedX>& Y,
	Eigen::PlainObjectBase<DerivedIX>& IX)
{
	// get number of rows (or columns)
	int num_inner = (dim == 1 ? X.rows() : X.cols() );
	// Special case for swapping
	if(num_inner == 2)
	{
		return igl::sort2(X,dim,ascending,Y,IX);
	}
	using namespace Eigen;
	// get number of columns (or rows)
	int num_outer = (dim == 1 ? X.cols() : X.rows() );
	// dim must be 2 or 1
	assert(dim == 1 || dim == 2);
	// Resize output
	Y.resize(X.rows(),X.cols());
	IX.resize(X.rows(),X.cols());
	// idea is to process each column (or row) as a std vector
	// loop over columns (or rows)
	for(int i = 0; i<num_outer;i++)
	{
		// Unsorted index map for this column (or row)
		std::vector<size_t> index_map(num_inner);
		std::vector<double> data(num_inner);
		for(int j = 0;j<num_inner;j++)
		{
			if(dim == 1)
			{
				data[j] = (double) X(j,i);
			}else
			{
				data[j] = (double) X(i,j);
			}
		}
		// sort this column (or row)
		igl::sort( data, ascending, data, index_map);
		// Copy into Y and IX
		for(int j = 0;j<num_inner;j++)
		{
			if(dim == 1)
			{
				Y(j,i) = data[j];
				IX(j,i) = index_map[j];
			}else
			{
				Y(i,j) = data[j];
				IX(i,j) = index_map[j];
			}
		}
	}
}