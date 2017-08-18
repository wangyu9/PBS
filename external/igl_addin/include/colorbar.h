#ifndef IGL_ADDIN_COLORBAR_h
#define IGL_ADDIN_COLORBAR_h

#include "types.h"

enum ColorBarType 
{
	COLORBAR_IGL_DEFAULT,
	COLORBAR_ZERO_ONE,
	COLORBAR_HIGHLIGHT_NEG,
	COLORBAR_ZERO_ONE_GREY_RED,
	COLORBAR_ZERO_ONE_NORMAL,
	COLORBAR_HIGHLIGHT_NEG_NORMAL,
	COLORBAR_ZERO_ONE_GREY_RED_NORMAL
};
#define NUM_OF_COLORBAR_TYPE 7

/********************* IGL_DEFAULT ************************/

inline void per_attrib_colors_igl_default(
	const VectorX *vertex_property, 
	PointMatrixType *vertex_colors)
{

	if (vertex_property->rows()<=0)
		return;
	double min_val = vertex_property->minCoeff();
	double max_val = vertex_property->maxCoeff();
	vertex_colors->resize(vertex_property->rows(),3);

	double im, m, mim, nmim;
	bool x1, x2, x3, x4, x5;
	//slope
	m = 1/(1.0/4);

	for (int i = 0; i<vertex_property->rows(); ++i)
	{
		im = ((*vertex_property)(i,0) - min_val) / (max_val - min_val);

		if ((max_val - min_val)<=0)
			im = 0.5;
		else    
			im = ((*vertex_property)(i,0) - min_val) / (max_val - min_val);
		//precomputation
		mim = m*im;
		nmim = -mim;
		x1 = im<1./8;
		x2 = im>=1./8 & im<3./8;
		x3 = im>=3./8 & im<5./8;
		x4 = im>=5./8 & im<7./8;
		x5 = im>=7./8;
		(*vertex_colors)(i,0) = x3*(mim - (3./8)*m)   + x4*1 + x5*(nmim + (9./8)*m);
		(*vertex_colors)(i,1) = x2*(mim - (1./8)*m)   + x3*1 + x4*(nmim + (7./8)*m);
		(*vertex_colors)(i,2) = x1*(mim + 0.5);

	}    
}


/********************* ZeroOne ************************/
// Assume most of the properties are distributed between 0 and 1 like weights
// It is allowed to be out of [0,1]
inline void per_attrib_colors_zero_one(
	const VectorX *vertex_property, 
	PointMatrixType *vertex_colors)
{

	if (vertex_property->rows()<=0)
		return;
	double min_val = vertex_property->minCoeff();
	double max_val = vertex_property->maxCoeff();
	vertex_colors->resize(vertex_property->rows(),3);

	double im;
	bool x1, x2, x3, x4, x5;

	Eigen::MatrixXd deep_red(1,3);
	deep_red << 0.5, 0, 0;
	Eigen::MatrixXd red(1,3);
	red << 1, 0, 0;
	Eigen::MatrixXd yellow(1,3);
	yellow << 1, 1, 0;
	Eigen::MatrixXd cyan(1,3);
	cyan << 0, 1, 1;
	Eigen::MatrixXd blue(1,3);
	blue << 0, 0, 1;
	Eigen::MatrixXd deep_blue(1,3);
	deep_blue << 0, 0, 0.56;
	for (int i = 0; i<vertex_property->rows(); ++i)
	{
		if ((max_val - min_val)<=0)
			im = 0.5;
		else    
			im = (*vertex_property)(i,0);
		//precomputation
		if(im>7./6)
		{
			im = 7./6;
		}
		if (im<-1./6)
		{
			im = -1./6;
		}

		if (im<0){
			double w = (im+1./6)*6.;
			vertex_colors->row(i) = (1-w)*deep_blue + w*blue;
		} 
		else if (im<1./3){
			double w = (im)*3.;
			vertex_colors->row(i) = (1-w)*blue + w*cyan;
		} 
		else if (im<2./3){
			double w = (im-1./3)*3.;
			vertex_colors->row(i) = (1-w)*cyan + w*yellow;
		}
		else if (im<1.){
			double w = (im-2./3)*3.;
			vertex_colors->row(i) = (1-w)*yellow + w*red;
		}
		else{//x5 must be true
			double w = (im-1.)*6.;
			vertex_colors->row(i) = (1-w)*red + w*deep_red;
		}
	}    
}

/********************* Heightlight Neg ************************/
// Assume most of the properties are distributed between 0 and 1 like weights
// It is allowed to be out of [0,1]
inline void per_attrib_colors_highlight_neg(
	const VectorX *vertex_property, 
	PointMatrixType *vertex_colors)
{

	if (vertex_property->rows()<=0)
		return;
	double min_val = vertex_property->minCoeff();
	double max_val = vertex_property->maxCoeff();
	vertex_colors->resize(vertex_property->rows(),3);

	double im;
	bool x1, x2, x3, x4, x5;

	Eigen::MatrixXd deep_red(1,3);
	deep_red << 0.5, 0, 0;
	Eigen::MatrixXd red(1,3);
	red << 1, 0, 0;
	Eigen::MatrixXd yellow(1,3);
	yellow << 1, 1, 0;
	Eigen::MatrixXd cyan(1,3);
	cyan << 0, 1, 1;
	Eigen::MatrixXd blue(1,3);
	blue << 0, 0, 1;
	Eigen::MatrixXd deep_blue(1,3);
	deep_blue << 0, 0, 0.56;

	Eigen::MatrixXd black(1,3);
	black << 0, 0, 0;

	for (int i = 0; i<vertex_property->rows(); ++i)
	{
		if ((max_val - min_val)<=0)
			im = 0.5;
		else    
			im = (*vertex_property)(i,0);
		//precomputation
		if(im>4./3)
		{
			im = 4./3;
		}
		if (im<-1./3)
		{
			im = -1./3;
		}

		if (im<0){
			double w = (im+1./3)*3.;
			vertex_colors->row(i) = (1-w)*black + w*deep_blue;
		} 
		else if (im<1./3){
			double w = (im)*3.;
			vertex_colors->row(i) = (1-w)*blue + w*cyan;
		} 
		else if (im<2./3){
			double w = (im-1./3)*3.;
			vertex_colors->row(i) = (1-w)*cyan + w*yellow;
		}
		else if (im<1.){
			double w = (im-2./3)*3.;
			vertex_colors->row(i) = (1-w)*yellow + w*red;
		}
		else{//x5 must be true
			double w = (im-1.)*3.;
			vertex_colors->row(i) = (1-w)*red + w*deep_red;
		}
	}    
}

/********************* ZeroOne ************************/
// Assume most of the properties are distributed between 0 and 1 like weights
// It is allowed to be out of [0,1]
inline void per_attrib_colors_zero_one_grey_red(
	const VectorX *vertex_property,
	PointMatrixType *vertex_colors)
{

	if (vertex_property->rows() <= 0)
		return;
	double min_val = vertex_property->minCoeff();
	double max_val = vertex_property->maxCoeff();
	vertex_colors->resize(vertex_property->rows(), 3);

	double im;
	bool x1, x2, x3, x4, x5;

	Eigen::MatrixXd deep_red(1, 3);
	deep_red << 0.5, 0, 0;
	Eigen::MatrixXd red(1, 3);
	red << 1, 0, 0;
	Eigen::MatrixXd yellow(1, 3);
	yellow << 1, 1, 0;
	Eigen::MatrixXd cyan(1, 3);
	cyan << 0, 1, 1;
	Eigen::MatrixXd grey(1, 3);
	grey << 1.0, 1.0, 1.0;
	Eigen::MatrixXd deep_blue(1, 3);
	deep_blue << 0, 0, 0.56;
	for (int i = 0; i < vertex_property->rows(); ++i)
	{
		if ((max_val - min_val) <= 0)
			im = 0.5;
		else
			im = (*vertex_property)(i, 0);
		//precomputation
		if (im > 7. / 6)
		{
			im = 7. / 6;
		}
		if (im < -1. / 6)
		{
			im = -1. / 6;
		}

		if (im < 0){
			double w = (im + 1. / 6)*6.;
			vertex_colors->row(i) = grey;//(1 - w)*deep_blue + w*blue;
		}
		else if (im < 1.){
			double w = im;
			vertex_colors->row(i) = (1 - w)*grey + w*red;
		}
		else{//x5 must be true
			double w = (im - 1.)*6.;
			vertex_colors->row(i) = (1 - w)*red + w*deep_red;
		}
	}
}

/********************** Wrapper ***************************/

inline void per_attrib_colors(
	const VectorX *attrib_property, 
	PointMatrixType *attrib_colors,
	ColorBarType colorBarType = COLORBAR_IGL_DEFAULT
	)
{
	const VectorX na = attrib_property->rows() ?  (attrib_property->array() - attrib_property->minCoeff()) / (attrib_property->maxCoeff() - attrib_property->minCoeff() + 1e-6)
		: VectorX(0, 1);
	switch(colorBarType)
	{
	case COLORBAR_IGL_DEFAULT:
		per_attrib_colors_igl_default(attrib_property,attrib_colors);
		break;
	case COLORBAR_ZERO_ONE:
		per_attrib_colors_zero_one(attrib_property,attrib_colors);
		break;
	case COLORBAR_HIGHLIGHT_NEG:
		per_attrib_colors_highlight_neg(attrib_property,attrib_colors);
		break;
	case COLORBAR_ZERO_ONE_GREY_RED:
		per_attrib_colors_zero_one_grey_red(attrib_property, attrib_colors);
		break;
	case COLORBAR_ZERO_ONE_NORMAL:
		per_attrib_colors_zero_one(&na, attrib_colors);
		break;
	case COLORBAR_HIGHLIGHT_NEG_NORMAL:
		per_attrib_colors_highlight_neg(&na, attrib_colors);
		break;
	case COLORBAR_ZERO_ONE_GREY_RED_NORMAL:
		per_attrib_colors_zero_one_grey_red(&na, attrib_colors);
		break;
	}
}


#endif /*IGL_ADDIN_COLORBAR_h*/