#ifndef VIRTUAL_FUNCTION_DEFAULT_H
#define VIRTUAL_FUNCTION_DEFUALT_H

#define VIRTUAL_EIGEN_MATRIXXD_DEFAULT_NO_DEFINATION(XX) { std::cerr << "Error. You should define (XX)() in derived class." << std::endl; return Eigen::MatrixXd::Zero(0,0);}
#define VIRTUAL_EIGEN_VECTORXI_DEFAULT_NO_DEFINATION(XX) { std::cerr << "Error. You should define (XX)() in derived class." << std::endl; return Eigen::VectorXi::Zero(0);}

#define VIRTUAL_VOID_DEFAULT_NO_DEFINATION(XX) { std::cerr << "Error. You should define (XX)() in derived class." << std::endl; }

#endif /*VIRTUAL_FUNCTION_DEFAULT_H*/