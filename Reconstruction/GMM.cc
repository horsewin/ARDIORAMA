/*
 * GaussianMixtureModel.cc
 *
 *  Created on: 2010/10/28
 *      Author: umakatsu
 */

#include "Reconstruction/GMM.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>

#include <opencv/ml.h>
#include <opencv/highgui.h>
namespace PTAMM{

using namespace std;

GMM::GMM(void){}

/*
 *
 */
GMM::GMM(const int k , const int d , const int size
//		, const vector< TooN::Vector< 3 > > col , const vector< TooN::Vector< 2 > > crd
): cla(k) , dim(d) , num(size)
{
//	emgm = new GMMwithEM(cla , dim , num);
	x = new SCALAR[dim];
	p = new SCALAR[dim - 1];
//	colsize = col.size();
	pTheta = new Theta*[cla];
	for(int ik = 0; ik < cla; ik++) {
		pTheta[ik] = new Theta(dim);
	}
}

GMM::~GMM(){
	delete[] x;
	delete[] p;
	for(int ik = 0; ik < cla; ik++) {
		delete pTheta[ik];
	}
	delete[] pTheta;
//	delete emgm;
}

void GMM::Run(const char *outputname1 , const char *outputname2){
//	emgm->learn(iterate);
//	emgm->dprint(outputname1 , outputname2);
}

void GMM::Run( const std::vector< TooN::Vector< 3 > > col , const std::vector< TooN::Vector< 2 > > crd ){
	/*
	 * OpenCVのEMアルゴリズムをもう一度試みる 2011.4.14
	 */
	colsize = col.size();
	CvEM em_model;
	CvEMParams params;
	CvMat * samples;
	samples = cvCreateMat( colsize , dim , CV_32FC1);	// 必要：デストラクタ
	for(unsigned int i=0; i<colsize; i++){
		TooN::Vector< 3 > col_tmp 	= col.at(i);
		TooN::Vector< 2 > crd_tmp 	= crd.at(i);
		REP(id , dim){
			x[id] = col_tmp[id];
			cvmSet( samples , i , id , x[id]);
		}
	}
    // モデルパラメータの初期化
    params.covs      = NULL;
    params.means     = NULL;
    params.weights   = NULL;
    params.probs     = NULL;
    params.nclusters = cla;
    params.cov_mat_type       = CvEM::COV_MAT_DIAGONAL;
    params.start_step         = CvEM::START_AUTO_STEP;
    params.term_crit.max_iter = iterate;
    params.term_crit.epsilon  = EPSILON;
    params.term_crit.type     = CV_TERMCRIT_ITER|CV_TERMCRIT_EPS;
    // EMアルゴリズムのトレーニング
	em_model.train( samples , 0 , params);
	// まどろっこしい手法
	// 平均・共分散行列とその逆行列をメンバ変数に格納
	const CvMat *means	= em_model.get_means();
	const CvMat *weights = em_model.get_weights();
	const CvMat ** covs  = em_model.get_covs();
	CvMat ** inversecovs = new CvMat*[cla];
	REP(i,cla){
		inversecovs[i] = cvCreateMat( dim , dim , CV_32FC1);
	}
	REP(i,cla){
		REP(j,dim){
			pTheta[i]->Mu[j] = cvGetReal2D( means , i , j);
			REP(k,dim){
				pTheta[i]->S[j*dim + k] = cvGetReal2D( covs[i] , j , k );
				cvmSet( inversecovs[i] , j , k , pTheta[i]->S[j*dim + k]);
			}
		}
		pTheta[i]->detS = cvmDet(covs[i]);
		pTheta[i]->weight = cvGetReal2D( weights , 0 , i);
//		cout << pTheta[i]->weight << " ";
	}
//	cout << endl;
	REP(i,cla){
		cvmInvert( inversecovs[i] , inversecovs[i]);
		REP(j,dim) REP(k,dim)
			pTheta[i]->invS[j*dim + k] = cvGetReal2D( inversecovs[i] , j , k );
	}
}

void GMM::setParameter(GaussDistribution ** gauss){
	assert(gauss);
	REP(i,cla){	//各クラスに対して繰り返し
		assert(gauss[i]);
		REP(j,dim){
			/* 平均パラメータの設定 */
			gauss[i]->Mu[j] = pTheta[i]->Mu[j];
			REP(k,dim){
				/* 共分散行列の逆行列のパラメータ設定 */
				gauss[i]->invS[j*D + k] = pTheta[i]->invS[j*D + k];
			}
		}
		/* 共分散行列の行列式のパラメータ設定 */
		gauss[i]->detS = pTheta[i]->detS;
		/* set the weight parameter */
		gauss[i]->weight = pTheta[i]->weight;
	}
}
}
