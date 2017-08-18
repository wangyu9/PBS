#ifndef SORT_WEIGHTS_H
#define SORT_WEIGHTS_H
#include <Eigen/Dense>
// Sort skinning weights. We want that W contains up to k weights, first being
// made up of *non-zero* original weights in descending order and then extra
// weights in descending order
// Templates:
//   T  should be a eigen matrix primitive type like int or double
// Inputs:
//   OW  #V by #OW original weights
//   EW  #V by #EW extra weights (may be empty)
//   k  maximum number of weights per handle (may be inf)
// Outputs:
//   W  #V by min(#OW+#EW,k)  sorted weights
//   WI  #V by min(#OW+#EW,k)  sorted weight indices such that if OEW = [OW EW]
//     then W(:,i) = OEW(:,WI(i))
template <typename T>
inline void sort_weights(
  const Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> & OW,
  const Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> & EW,
  const int k,
  const double epsilon,
  Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> & W,
  Eigen::MatrixXi & WI);

// Implementation
#include <igl/EPS.h>
#include <igl/sort.h>

// %Equivalent Matlab code:
// OEW = [OW+(OW>0)*max(EW(:)) EW];
// [W,WI] = sort(OEW,2,'descend');
// W(WI<=size(OW,2) & W>0) = W(WI<=size(OW,2) & W>0) - max(EW(:));
// W = W(:,1:min(k,size(W,2)));
// WI = WI(:,1:min(k,size(W,2)));
template <typename T>
inline void sort_weights(
  const Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> & OW,
  const Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> & EW,
  const int k,
  const double epsilon,
  Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> & W,
  Eigen::MatrixXi & WI)
{
  assert(k > 0);
  // number of domain positions
  int n = OW.rows();
  assert(n == EW.rows());
  // number of original weights
  int no = OW.cols();
  // number of extra weights
  int ne = EW.cols();

  // Find maximum in EW
  T max_ew = 0;
  if(ne > 0)
  {
    max_ew = EW.maxCoeff();
  }

  // Add max_ew to all significant original weights 
  // OEW = [OW+(OW>0)*max(EW(:)) EW];
  Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> OEW(n,no+ne);
  OEW.block(0,0,n,no) = 
    (OW.array() > epsilon).select(OW.array()+max_ew,OW.array());
  // Fill in EW
  if(ne > 0)
  {
    OEW.block(0,no,n,ne) = EW;
  }

  igl::sort(OEW,2,false,W,WI);

  // Set all original weights in W their value without max_ew
  W = 
    (WI.array() < no).select(
      (W.array() > epsilon).select(
        W.array()-max_ew, W.array() ), W.array());
  // only keep k weights
  if(k < W.cols())
  {
    W = W.leftCols(k).eval();
    WI = WI.leftCols(k).eval();
  }
}

// added by wangyu
// 
template <typename T>
inline void sort_abs_weights(
	const Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> & OW,
	const Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> & EW,
	const int k,
	Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> & W,
	Eigen::MatrixXi & WI)
{
	assert(k > 0);
	// number of domain positions
	int n = OW.rows();
	assert(n == EW.rows());
	// number of original weights
	int no = OW.cols();
	// number of extra weights
	int ne = EW.cols();

	// Find maximum in EW
	T max_ew = 0;
	if(ne > 0)
	{
		max_ew = EW.maxCoeff();
	}

	// Add max_ew to all significant original weights 
	// OEW = [OW+(OW>0)*max(EW(:)) EW];
	// Then it is different! wangyu

	Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> OEW(n,no);
	OEW.block(0,0,n,no) = 
		(OW.array() > 0).select(OW.array(),-OW.array());
	for (int i=0; i<n; i++)
	{
		for (int j=0; j<no; j++)
		{
			if(OEW(i,j)!=abs(OW(i,j)))
			{
				printf("Error0 in ABS(weithgs) detected!");
			}
		}
	}
	


	igl::sort(OEW,2,false,W,WI);// wangyu W value is incorrect since OEW=abs(OW)>=0

	for (int i=0; i<WI.rows(); i++)
	{
		for (int j=0; j<WI.cols(); j++)
		{
			int k = WI(i,j);
			if ( abs(W(i,j)) != OEW(i,k) )
			{
				printf("Error1 in weights sorting detected!");
			}
			W(i,j) = OW(i,k);
			if (j>0)
			{
				if ( abs(W(i,j)) > abs(W(i,j-1)) )
				{
					printf("Error2 in weights sorting detected!");
				}
			}
			
		}
	}
	
	// only keep k weights
	if(k < W.cols())
	{
		W = W.leftCols(k).eval();
		WI = WI.leftCols(k).eval();
	}
}

#endif
