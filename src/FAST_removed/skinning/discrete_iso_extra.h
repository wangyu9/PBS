#ifndef DISCRETE_ISO_EXTRA_H
#define DISCRETE_ISO_EXTRA_H

#include <Eigen/Dense>
#include "Ease.h"

// DISCRETE_ISO_EXTRA Use initial weights as embedding for isotropic "bump" weights
//
// Inputs:
//  W  weights for existing handles, #V by #W matrix of weights
//  dist   distance between uniform samples
//  ease  type of ease filter to use to make bumps, see Ease.h
// Outputs:
//  EW  #V by k matrix of extra weights
// 
void discrete_iso_extra(
	const Eigen::MatrixXd & W,
	const double dist,	
	Ease ease,
	Eigen::MatrixXd & EW);

// Implementation 
#include <colon.h>
#include <slice.h>
#include <functional>
#include "partition.h"
#include "uniform_sample.h"

class MatrixXi_comparator {
public:
	bool operator()(const Eigen::MatrixXi &m1, const Eigen::MatrixXi &m2)
	{
		assert(m1.rows() == 1 && m2.rows() == 1);
		assert(m1.cols() == m2.cols());

		for (int i=0; i<m1.cols(); i++)
		{
			if (m1(0, i) < m2(0, i)) return true;
			else if (m1(0, i) > m2(0, i)) return false;
		}
		
		return false; // equal
	}
};

////tst
//Eigen::MatrixXi tst1(1, 5);
//tst1 << 1, 2, 3, 2, 7;
//Eigen::MatrixXi tst2(1, 5);
//tst2 << 1, 2, 3, 2, 6;
//
////bool t = tst1 < tst2;
//
//std::vector<Eigen::MatrixXi> tv;
//tv.push_back(tst1);
//tv.push_back(tst2);
//
//MatrixXi_comparator comp;
//std::sort(tv.begin(), tv.end(), comp);
//
//int hu = 1;
////eof tst
//

void discrete_uniform_sample(
	const Eigen::MatrixXd & W,
	int numCuts,
	Eigen::MatrixXd & WS)
{
	using namespace std;

	const int n= W.rows();
	const int dim = W.cols();

	//cout<<"sw=["<<endl<<W<<endl<<"];"<<endl;

	// discretize all coordinates except the first one:
	Eigen::MatrixXi snapped(n, dim-1);
	for (int i=0; i<n; i++)
	{
		Eigen::MatrixXd Wrow(1, dim);
		for (int d=0; d<dim; d++) Wrow(0, d) = W(i, d);

		// possibly the dirtiest piece of code I've ever written...
		int it = 0;
		for (;;it++)
		{
			if (it == 100)
			{
				printf("Kill me... (iteration count in discrete_uniform_sample exceeded)\n");
				assert(false);
			}
			int chkSum = 0;
			for (int d=1; d<dim; d++)
			{
				const int value = floor( Wrow(0, d) * numCuts + 0.5);
				snapped(i, d-1) = value;
				chkSum += value;
			}
			if (chkSum <= numCuts) break;

			Wrow *= 0.98;
		}
	}

	//cout<<"sw=["<<endl<<snapped<<endl<<"];"<<endl;

	// discard duplicates:
	std::vector<Eigen::MatrixXi> rowVec; rowVec.resize(n);
	for (int i=0; i<n; i++)
	{
		rowVec[i].resize(1, dim-1); // = MatrixXi(1, dim-1);
		for (int d=0; d<dim-1; d++) rowVec[i](0, d) = snapped(i, d);
	}
	MatrixXi_comparator comp;
	std::sort(rowVec.begin(), rowVec.end(), comp);

	std::list<Eigen::MatrixXi> rowList;
	for (int i=0; i<n; i++) 
	{
		const Eigen::MatrixXi &row = rowVec[i];
		// filter away centers that are corner or are immediately adjacent to it:
		bool closeToCorner = false;
		int sum = 0;
		for (int d=0; d<dim-1; d++)
		{
			const int v = row(0, d);			
			if (v == numCuts || v == numCuts - 1) closeToCorner = true;

			sum += v;
		}
		if (sum <= 1) closeToCorner = true;
		if (!closeToCorner)
		{
			rowList.push_back(rowVec[i]);
		}		
	}
	rowList.unique();	
	
	// return back to weight space:
	const int k = rowList.size();
	WS.resize(k, dim);

	for (int i=0; i<k; i++)
	{
		Eigen::MatrixXi discreteCenter = rowList.front();
		rowList.pop_front();
		int sum = 0;
		for (int d=0; d<dim-1; d++) sum += discreteCenter(0, d);

		int first = numCuts - sum;
		assert(first >= 0);

		WS(i, 0) = (double)first / numCuts;
		for (int d=1; d<dim; d++) WS(i, d) = (double)discreteCenter(0, d-1) / numCuts;
	}

	cout<<"WS=["<<endl<<WS<<endl<<"];"<<endl;
}


void discrete_iso_extra(
	const Eigen::MatrixXd & W,
	int numCuts,
	const double push,
	Ease ease,
	Eigen::MatrixXd & EW)
{
	using namespace Eigen;
	using namespace igl;
	using namespace std;
	assert(numCuts >= 2);
	// number of domain vertices
	const int n = W.rows();
	// number of original weights
	const int nc = W.cols();

	printf("discrete_iso_extra start\n");

	// Get weights of seeds
	MatrixXd WS;
	discrete_uniform_sample(W,numCuts,WS);

	const int k = WS.rows();
	printf("discrete_iso_extra generated %i extra weights\n", k);

	//assert(WS.rows() == k);

#ifdef EXTREME_VERBOSE
	cout<<"WS=["<<endl<<WS<<endl<<"];"<<endl;
#endif

	// Get weights of originals (corners of weight space)
	MatrixXd I = MatrixXd::Identity(W.cols(),W.cols());

	// resize output
	EW.resize(n,k);
	// loop over new handles
	int added_i = 0;
	for(int i = 0;i < k;i++)
	{ 
		Matrix<double,1,Dynamic> sw = WS.row(i);
#ifdef EXTREME_VERBOSE
		cout<<"sw=["<<endl<<sw<<endl<<"];"<<endl;
#endif
		// find closest original seed and use distance as potential radius
		double r =
			sqrt((I.rowwise()-sw).array().pow(2).rowwise().sum().minCoeff());
#ifdef EXTREME_VERBOSE
		cout<<"r=["<<endl<<r<<endl<<"];"<<endl;
#endif
		// Get weights of other new seeds
		MatrixXd Wother(k-1,W.cols());
		for(int j = 0;j<i;j++)
		{
			Wother.row(j) = WS.row(j);
		}
		for(int j = i+1;j<k;j++)
		{
			Wother.row(j-1) = WS.row(j);
		}
#ifdef EXTREME_VERBOSE
		cout<<"Wother=["<<endl<<Wother<<endl<<"];"<<endl;
#endif
		if(Wother.size() > 0)
		{
			// find closest new seed and use distance as potential radius
			double r_new =
				sqrt((Wother.rowwise()-sw).array().pow(2).rowwise().sum().minCoeff());
#ifdef EXTREME_VERBOSE
			cout<<"r_new=["<<endl<<r_new<<endl<<"];"<<endl;
#endif
			// radius is minimum of pushed closest new and closest original
			if(r<r_new*push)
			{
				//continue;
				verbose("r_original (%f) < (%f)*push\n",r,r_new);
			}
			r = (r<r_new*push?r:r_new*push);
		}
#ifdef EXTREME_VERBOSE
		cout<<"r=["<<endl<<r<<endl<<"];"<<endl;
#endif
		assert(r>0);
		r = 2.0 / numCuts;
		printf("%i: r = %f\n", i, r);
		for(int j = 0;j<n;j++)
		{
			// get distance to this seed
			EW(j,added_i) = sqrt((W.row(j)-sw).array().pow(2).sum());
			// clamp to r
			EW(j,added_i) = EW(j,added_i) > r ? r : EW(j,added_i);
			// divide by r, reverse and run through ease filter
			EW(j,added_i) = ::ease(ease,1.0-EW(j,added_i)/r);
		}
		added_i++;
	}
	EW.conservativeResize(EW.rows(),added_i);
}

#endif
