/*
 * GaussianMixtureModel.h
 *
 *  Created on: 2010/10/28
 *      Author: umakatsu
 */

#ifndef GMM_H_
#define GMM_H_

#include <string.h>
#include <vector>
//#include <TooN/numerics.h>
#include <TooN/numhelpers.h>
#include <TooN/helpers.h>
//#include "GMMwithEM.h"
/* Data , Thetaの定義を行っているヘッダ */
#include "parameter.h"
/* 共通のパラメータの設定ヘッダ 2010/11/15*/
#include "common.h"

namespace PTAMM{

class GMM{

private:
	int cla; //  the number of class
	int dim;	//  Dimention
	unsigned long int colsize;
	unsigned long int num;	//  the number of sample
	char *filename;
//	GMMwithEM * emgm;
	SCALAR *x;
	SCALAR *p;
	Theta **pTheta;

public:
	GMM();
//	GMM(const int k , const int d , const int size , const std::vector< TooN::Vector< 3 > > col , const std::vector< TooN::Vector< 2 > > crd);
	GMM(const int k , const int d , const int size );
	~GMM( void );
	void Run( const char* , const char*);
	void Run( const std::vector< TooN::Vector< 3 > > col , const std::vector< TooN::Vector< 2 > > crd);
	void setParameter(GaussDistribution ** gauss);
    int getCla() const
    {
        return cla;
    }

    void setCla(int cla)
    {
        this->cla = cla;
    }
};
}
#endif /* GMM_H_ */
