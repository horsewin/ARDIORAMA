/*
 * GraphCut.h
 *
 *  Created on: 2010/11/03
 *      Author: umakatsu
 */

#ifndef GRAPHCUT_H_
#define GRAPHCUT_H_

#include "Reconstruction/graph.h"
#include "Reconstruction/GMM.h"
#include "Image/VideoData.h"
#include "common.h"
#include <vector>
//#include <TooN/numerics.h>
#include <TooN/numhelpers.h>

using namespace PTAMM;
struct TLink;
struct NLink;
typedef Graph<double,double,double> GraphType;

/* Graph-Cut */
//t-linkの影響をかなり強くしてみる 2011.2.21
const double	SIGMA = 3;
const double 	LAMBDA = 0.01; //t-linkの係数
const double	alpha	= 1;

using namespace std;

class GraphCut{
public:
	GraphCut( const int & w , const int & h , const int & k , VideoData * video);
	~GraphCut( void );
	void initFirst(GaussDistribution ** gmm_fore , GaussDistribution ** gmm_back);
	void initRegular(GaussDistribution ** gmm_fore , GaussDistribution ** gmm_back , double *Pr_o , double *Pr_b);
	void firstRun(char **image_map);
	void regularRun(char **image_map);

private:
  /* private member function */
  void initConstant(GaussDistribution ** gmm_fore , GaussDistribution ** gmm_back);

  /* Member variable */
	int width;
	int height;
	int K;
	GraphType * g;
	VideoData * vd;
	/* 尤度を求めるための各種パラメータ */
	boost::array < Vector < D > , CLASSNUMBER> mu_fore;
	boost::array < Vector < D > , CLASSNUMBER> mu_back;
	boost::array< TooN::Matrix < D > , CLASSNUMBER > covariance_inv_fore;
	boost::array< TooN::Matrix < D > , CLASSNUMBER > covariance_inv_back;
	Vector < CLASSNUMBER >	det_covariance_fore;
	Vector < CLASSNUMBER >	det_covariance_back;
	Vector < CLASSNUMBER > constpart_fore; //同時確率密度関数の定数部
	Vector < CLASSNUMBER > constpart_back; //同時確率密度関数の定数部
//	double weight[CLASSNUMBER];			//各ガウス分布に当てる重み
	double weight_fore[CLASSNUMBER];			//各ガウス分布に当てる重み
	double weight_back[CLASSNUMBER];			//各ガウス分布に当てる重み
	/* 2回目以降のGrapu Cutsに用いる */
	vector<int> check;
};

#endif /* GRAPHCUT_H_ */
