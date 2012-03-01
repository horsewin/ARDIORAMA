/*
 * GraphCut.cc
 *
 *  Created on: 2010/11/03
 *      Author: umakatsu
 *      行列演算のところを見直す必要あり
 */
#define _USE_MATH_DEFINES

/* 共通のパラメータの設定ヘッダ 2010/11/15*/
#include "common.h"

#include <stdio.h>
#include <cmath>
#include <vector>
#include <TooN/helpers.h>
#include <float.h>
#include <time.h>

#include <boost/array.hpp>
#include "Reconstruction/GraphCut.h"

/* Data , Thetaの定義を行っているヘッダ */
#include "parameter.h"

//const definition
#define DEBUG 0
#define CHECK 0
#define ISMARCUT 0
//namespace definition
using namespace std;
using namespace TooN;

namespace{
	template< int S >
	Vector< S > Calc(const Vector< S > v1 , const Matrix< S > v2){
		Vector< S > ret;
		ret = Zeros;
		for(int i=0; i<S; i++){
			for(int j=0; j<S; j++){
				ret[i] += v1[j] * v2[j][i];
			}
		}
		return ret;
	}

	//Vectorクラスのユーグリッド距離
	template < int Size>
	double dist(const Vector< Size > & v1 , const Vector< Size > & v2){
		double ret = 0;
		for(int i=0; i < Size; i++){
			ret += (v2[i] - v1[i]) * (v2[i] - v1[i]);
		}
		ret = sqrt(ret);
		return ret;
	}

	/* 4近傍ピクセルの座標値を返すメソッド */
	vector <Vector< 2 > > Neighbor(Vector< 2 > p){
		vector< Vector< 2 > > q;	//隣接ピクセルの定義
		q.clear();
		for(int i=-1; i<=1; i++){
			for(int j=-1; j<=1; j++){
				if( (i==0 && j==0)
				||	(i==-1 && j==-1)
				||	(i==-1 && j== 1)
				||	(i== 1 && j== 1)
				||	(i== 1 && j== -1)
				);
				else{
					if( (p[0] + i ) >= 0
					&&	(p[0] + i ) < WIDTH
					&&  (p[1] + j ) >= 0
					&&  (p[1] + j ) < HEIGHT){
						Vector< 2 > tmp;
						tmp[0] = p[0] + i;
						tmp[1] = p[1] + j;
						q.push_back(tmp);
					}
				}
			}
		}
		return q;
	}

	inline void doubleCheck(double & data){
		if(isnan(data) || isinf(data)) data = DBL_MAX;
	}
}

struct TLink{
	int sigma;
	int width;
	int height;
	double _K;
	double** rgb_data;

	TLink(int s , int w , int h) : sigma(s) , width(w) , height(h) , _K(1.0) {}
	~TLink(){}

	void setRGBData(double **rgb){ rgb_data = rgb; }
	inline double neighborEdgeCost(Vector< 3 > RGB1 , Vector< 3 > RGB2 , Vector< 2 > tmp1 , Vector< 2 > tmp2){
		double color_diff = (RGB1 - RGB2) * (RGB1 - RGB2);
		double cost = exp(-color_diff / (2*sigma*sigma));
		if( isnan(cost) || isinf(cost)) cost = DBL_MAX;
		cost /= dist(tmp1 , tmp2);
		return cost;
	}

	inline void objectCost( Vector< 3 > rgb , Vector< 2 > p){
		vector< Vector< 2 > > neighbors = Neighbor(p);
		/* objectに属しているピクセルの中で最大のものを見つけるための一時変数 */
		double K_tmp = 1.0;
		REP( i, neighbors.size()){
			Vector< 2 > q = neighbors.at(i);
			Vector < D > iq;
			unsigned long int num = q[0] + q[1]*width;
			REP( id , D ) iq[id] = rgb_data[num][id];
			K_tmp += neighborEdgeCost(rgb , iq , p , q);
		}
		if( _K < K_tmp ) _K = K_tmp;
	}

	inline void remainTLinkObject(vector< Vector < 2 > > remain , GraphType *graph){
		int remainsize = remain.size();
		REP( i , remainsize){
			Vector< 2 > r = remain.at(i);
			/* 背景(Source)t-link:0 , 前景(Sink)t-link: _K */
			graph->add_tweights(r[0]+r[1]*WIDTH , 0 , _K);
		}
	}

	inline void remainTLinkBackground(vector< Vector < 2 > > remain , GraphType *graph){
		int remainsize = remain.size();
		REP( i , remainsize){
			Vector< 2 > r = remain.at(i);
			/* 背景(Source)t-link:_K , 前景(Sink)t-link: 0 */
			graph->add_tweights(r[0]+r[1]*WIDTH , _K , 0);
		}
	}

};

struct NLink{
	double sigma;
	Vector< 2 > p1;
	Vector< 2 > p2;
	Vector< 3 > RGB1;
	Vector< 3 > RGB2;

	NLink(int s ,
			Vector< 2 > tmp1 , Vector< 2 > tmp2 ,
			Vector< 3 > rgb1 , Vector< 3 > rgb2)
	: sigma(s) , p1(tmp1) , p2(tmp2) , RGB1(rgb1) , RGB2(rgb2){}
	~NLink(){}
	double setNLinkCost(){
		double color_diff = (RGB1 - RGB2) * (RGB1 - RGB2);
		double cost = exp(-color_diff / (2*sigma*sigma)) * alpha ;
		if( isnan(cost) || isinf(cost)) cost = DBL_MAX;
//		cost /= dist(p1 , p2);
		return cost;
	}
};


/*
 * コンストラクタ : GraphCut
 * GMMからのグラフカットのためのコンストラクタ
 */
GraphCut::GraphCut(const int & w , const int & h , const int & k , VideoData *video)
: width(w) , height(h) , K(k)
{
	g = new GraphType(/*estimated # of nodes*/ 2, /*estimated # of edges*/ 1);
	REP(i,K){
		weight_fore[i] = (double)1 / K;
		weight_back[i] = (double)1 / K;
	}
	vd = video;
}

GraphCut::~GraphCut( void ){
	delete g;
}


void GraphCut::initFirst( GaussDistribution ** gmm_fore , GaussDistribution ** gmm_back)
{
	char *image_map = vd->getImg();
	initConstant(gmm_fore , gmm_back);
//	ofstream fo("InitialGraphCuts.txt");
	/* ピクセル番号に関連したRGB情報を取得 */
	double **rgb_data = vd->getRGB();
	/* ストロークによってFOREGROUNDに属していると判定されているピクセルのstore */
	vector< Vector< 2 > > remain;
	remain.clear();
	/* TLinkクラスの作成 */
	TLink *tlink = new TLink(SIGMA , width , height);
	tlink->setRGBData(rgb_data);
	REP(i,height){
		REP(j,width){
#if ISMARCUT
			/* ISMAR */
			double likelihood_fore = 1.0;
			double likelihood_back = 1.0;
#else
			/* Graph Cuts */
			double likelihood_fore = 0.0;
			double likelihood_back = 0.0;
#endif
			/* 計算対象であるピクセルのRGBデータ取得 */
			Vector< D > ip;
			REP(id , D) ip[id] = rgb_data[i*width + j][id];
			/* 計算対象であるピクセルの座標データ取得 */
			Vector< 2 > p;
			p[0] = j; p[1] = i;

			g->add_node();
//			fo << "(" << p[0] << "," << p[1] << ")" << endl;
			/* 重み付けの対象となるノード番号 */
			int number = i*width + j;
			/* 前景と背景のどちらかで尤度計算 */
			if(image_map[j + i * width] == FOREGROUND){
				/* t-linkを前景はまとめて設定する */
				remain.push_back(p);
				tlink->objectCost( ip , p);
			}else{
				REP(k,K){
					Vector<D> tmp = Calc( (ip - mu_fore[k]) , covariance_inv_fore[k]);
					double tmp2 = exp( ((tmp * (ip - mu_fore[k]) ))*(-0.5) );
					if(isinf(tmp2) || isnan(tmp2) ) tmp2 = DBL_MAX;
#if ISMARCUT
					likelihood_fore += -log(weight[k] * constpart_fore[k] * tmp2);
#else
					likelihood_fore += constpart_fore[k] * tmp2;
#endif
				}
				REP(k,K){
					Vector<D> tmp = Calc( (ip - mu_back[k]) , covariance_inv_back[k]);
					double tmp2 = exp( ((tmp * (ip - mu_back[k]) ))*(-0.5) );
					if(isinf(tmp2) || isnan(tmp2) ) tmp2 = DBL_MAX;
#if ISMARCUT
					likelihood_back += -log(weight[k] * constpart_back[k] * tmp2);
#else
					likelihood_back += constpart_back[k] * tmp2;
#endif
				}
				/* t-linkへのエッジの重み付け */
#if ISMARCUT == 0
				likelihood_fore = -log(likelihood_fore);
				likelihood_back = -log(likelihood_back);
#endif
				g->add_tweights( number ,
						/* capacities */  likelihood_fore * LAMBDA , likelihood_back * LAMBDA);
//				fo << "  T-link: F="<< likelihood_fore << " , " << "B=" << likelihood_back << endl;
			}
			/* n-linkへのエッジの重み付け */
			vector< Vector< 2 > > neighbors = Neighbor(p);
			REP( i_n , neighbors.size()){
				Vector < 2 > q = neighbors.at(i_n);
				/* 追加されていないノードのエッジの重みは飛ばす */
				if( (p[0] < q[0]) || (p[1] < q[1]));
				else {
					Vector < D > iq;
					unsigned long int n = q[0] + q[1]*width;
					REP( id , D ) iq[id] = rgb_data[n][id];
					NLink *nlink = new NLink( SIGMA , p , q , ip , iq);
					double cost = nlink->setNLinkCost();
					g -> add_edge( number , n , /* capacities */  cost , cost );
//					fo << "  N-link to (" << q[0] << "," << q[1] << ") : " << cost << endl;
					delete nlink;
				}
			}
		}
	}
	/* FOREGROUNDに属していた残りのピクセルに固定の重みを持つt-linkを行う */
	tlink->remainTLinkObject(remain , g);
	//memory release
	delete tlink;
//	fo.close();
}

void GraphCut::initRegular(GaussDistribution ** gmm_fore , GaussDistribution ** gmm_back , double *Pr_o , double *Pr_b)
{
	initConstant(gmm_fore , gmm_back);
//	ofstream fo("GraphCuts.txt");
	// ビデオイメージから2Dのセグメントマップを取得
	// ラベル名:CHECKREGION,FOREGROUNDの部分のみをGraph Cuts対象として用いる
	char *img = vd->getImg();
	check.clear();
	REP(i,width*height){
//		if(img[i] == CHECKREGION || img[i] == FOREGROUND ) check.push_back(i);
		if(img[i] == CHECKREGION) check.push_back(i);
	}

//	/* ストロークによってFOREGROUNDに属していると判定されているピクセルのstore */
//	vector< Vector< 2 > > remain;
//	remain.clear();
	/* ピクセル番号に関連したRGB情報を取得 */
	double **rgb_data = vd->getRGB();
	/* TLinkクラスの作成 */
	TLink *tlink = new TLink(SIGMA , width , height);
	tlink->setRGBData(rgb_data);
	//Set edge weight to each objective pixels
	REP(n,check.size()){
	    /* 対象領域の1次元座標値の取得 */
		int address = check.at(n);
		/* 尤度の初期化 */
		double likelihood_fore = 0.0;
		double likelihood_back = 0.0;
		/* 計算対象であるピクセルのRGBデータ取得 */
		Vector< D > ip;
		REP(id , D) ip[id] = rgb_data[address][id];
		/* 計算対象であるピクセルの2次元座標データ取得 */
		Vector< 2 > p;
		p[0] = address % width;
		p[1] = address / width;
		/* ノード追加 */
		g->add_node();
//		if(img[address] == FOREGROUND ){
//			/* t-linkを前景はまとめて設定する */
//			tlink->objectCost( ip , p);
////			remain.push_back(p);
//			/* BACKGROUNDはノードとして追加されていないので工夫してノード位置を格納 */
//			Vector< 2 > obj;
//			obj[0] = n; obj[1] = 0;
//			remain.push_back(obj);
//		}else if(img[address] == CHECKREGION ){
	/* GMMのパラメータを用いた尤度計算 */
		REP(k , K){
			Vector<D> tmp = Calc( (ip - mu_fore[k]) , covariance_inv_fore[k]);
			double tmp2 = exp( ((tmp * (ip - mu_fore[k]) ))*(-0.5) );
			if(isinf(tmp2) || isnan(tmp2) ) tmp2 = DBL_MAX;
#if ISMARCUT
			likelihood_fore += -log(weight[k] * constpart_fore[k] * tmp2 * Pr_o[address]);
#else
			likelihood_fore += weight_fore[k]*constpart_fore[k] * tmp2;
#endif
			tmp = Calc( (ip - mu_back[k]) , covariance_inv_back[k]);
			tmp2 = exp( ((tmp * (ip - mu_back[k]) ))*(-0.5) );
			if(isinf(tmp2) || isnan(tmp2) ) tmp2 = DBL_MAX;
#if ISMARCUT
			likelihood_back += -log(weight[k] * constpart_back[k] * tmp2 * Pr_b[address]);
#else
			likelihood_back += weight_back[k]*constpart_back[k] * tmp2;
#endif
		}
		/* エッジ部分の確率に重み付けをする */
#if ISMARCUT == 0
		//			likelihood_fore *= Pr_o[address];
		//			likelihood_back *= Pr_b[address];
		if(isinf(likelihood_fore) || isnan(likelihood_fore) )  likelihood_fore= DBL_MAX;
		if(isinf(likelihood_back) || isnan(likelihood_back) )  likelihood_back= DBL_MAX;
		likelihood_fore = -log(likelihood_fore)*Pr_b[address]*LAMBDA;
		likelihood_back = -log(likelihood_back)*Pr_o[address]*LAMBDA;
#endif
		/* t-linkへのエッジの重み付け */
		g->add_tweights( n ,
				/* capacities */  likelihood_fore, likelihood_back);

		/* n-linkへのエッジの重み付け */
		vector< Vector< 2 > > neighbors = Neighbor(p);
		REP( i_n , neighbors.size()){
			Vector < 2 > q = neighbors.at(i_n);
			int qn = q[0] + q[1]*width; //隣接ノードの画像ピクセルの位置
			/* 探索対象ピクセルの隣接ノード以外は飛ばす */
	//			if( img[qn] == CHECKREGION || img[qn] == FOREGROUND ){
			if( img[qn] == CHECKREGION ){
			  /* 追加されていないノードのエッジの重みは飛ばす */
			  if( (p[0] < q[0]) || (p[1] < q[1]));
			  else {
				  Vector < D > iq;
				  REP( id , D ) iq[id] = rgb_data[qn][id];
				  NLink *nlink = new NLink( SIGMA , p , q , ip , iq);
				  double cost = nlink->setNLinkCost();
				  /* 対象となっている隣接ノードの通し番号(ノード番号)の探索 */
				  int seek;
				  for( seek = n; seek >= 0; seek--){
					  if(check.at(seek) == qn) break;
				  }
				  assert(seek >= 0);
				  /* n-linkをはる */
				  g -> add_edge( n , seek , /* capacities */  cost , cost );
				  delete nlink;
			  }
			}
		}
	}
	/* FOREGROUNDに属していた残りのピクセルに固定の重みを持つt-linkを行う */
//	tlink->remainTLinkObject(remain , g);
//	fo.close();
	//memory release
	delete tlink;
}
//		fprintf(stderr,"Cannot Detect silhouette!!\n");
//		assert(first);

void GraphCut::firstRun(char **image_map){
#if CHECK
	double start = TIME();
#endif
	double flow = g -> maxflow();
	/* グラフカットの結果を観測するための処理(PPMファイルに出力) */
//	double **rgb_data = vd->getRGB();
//	FILE *fi = fopen("origin_graphcut.ppm","w");
//	FILE *fw = fopen("result_graphcut.ppm","w");
//	fprintf(fi,"P3\n%d\t%d\n255\n",width,height);
//	fprintf(fw,"P3\n%d\t%d\n255\n",width,height);
	REP(i , height - bottom_buf){
		REP(j , width){
//			REP(data,D) fprintf(fi,"%lf\t",rgb_data[i*width+j][data]);
//			fprintf(fi,"\n");
			if (g->what_segment( i*width + j) == GraphType::SOURCE){
//				fprintf(fw,"0.0\t0.0\t0.0\n");
				image_map[j][i] = BACKGROUND;
			}else{
//				REP(data,D) fprintf(fw,"%lf\t",rgb_data[i*width+j][data]);
//				fprintf(fw,"\n");
				image_map[j][i] = FOREGROUND;
			}
		}
	}
//	for(int i=height - bottom_buf; i<height; i++){
//		REP(j , width){
//			REP(data,D) fprintf(fi,"%lf\t",rgb_data[i*width+j][data]);
//			fprintf(fi,"\n");
//			fprintf(fw,"0.0\t0.0\t0.0\n");
//		}
//	}
//	fclose(fw);
//	fclose(fi);
#if CHECK
	if( isnan(flow) || isinf(flow)){
		cout << "  Initial Graph Cuts End : Flow Error " << endl;
	}else{
		cout << "  Initial Graph Cuts End : " << PROCESSTIME(start) << endl;
	}
#endif
//	return *g;
}

void GraphCut::regularRun(char **image_map){
#if CHECK
	double start = TIME();
#endif
	double flow = g -> maxflow();
	REP(n,check.size()){
		int i = check.at(n) % width;	// pixel width
		int j = check.at(n) / width;	// pixel height
		if (g->what_segment(n) == GraphType::SOURCE){
			image_map[i][j] = BACKGROUND;
		}else{
			image_map[i][j] = FOREGROUND;
		}
	}
#if CHECK
	if( isnan(flow) || isinf(flow)){
		cout << "  Regular Graph Cuts End: Flow Error " << endl;
	}else{
		cout << "  Regular Graph Cuts : " << PROCESSTIME(start) << endl;
	}
#endif
//	return *g;
}


/* 尤度の定数部のみを先に計算しておく */
void GraphCut::initConstant(GaussDistribution ** gmm_fore , GaussDistribution ** gmm_back){
	REP(i,K){	//各クラスに対して繰り返し
		covariance_inv_fore[i] = Zeros;
		covariance_inv_back[i] = Zeros;
		weight_fore[i] = gmm_fore[i]->weight;
		weight_back[i] = gmm_back[i]->weight;
		REP(j,D){
			/* 平均パラメータの設定 */
			mu_fore[i][j] = gmm_fore[i]->Mu[j];
			mu_back[i][j] = gmm_back[i]->Mu[j];
		}
		REP(j,D){
			REP(k,D){
				/* 共分散行列の逆行列のパラメータ設定 */
				covariance_inv_fore[i][j][k] = gmm_fore[i]->invS[j*D + k];
				covariance_inv_back[i][j][k] = gmm_back[i]->invS[j*D + k];
			}
		}
		/* 共分散行列のdeterminantのパラメータ設定 */
		det_covariance_fore[i] = gmm_fore[i]->detS;
		det_covariance_back[i] = gmm_back[i]->detS;
		/* 同時確率密度関数の定数部を先に計算しておく */
		constpart_fore[i] = weight_fore[i]/( 2*M_PI*sqrt(2*M_PI)*sqrt(det_covariance_fore[i]) );
		constpart_back[i] = weight_back[i]/( 2*M_PI*sqrt(2*M_PI)*sqrt(det_covariance_back[i]) );
//		constpart_fore[i] = weight_fore[i]/( 2*M_PI* sqrt(2*M_PI) * sqrt(det_covariance_fore[i]) );
//		constpart_back[i] = weight_back[i]/( 2*M_PI* sqrt(2*M_PI) * sqrt(det_covariance_back[i]) );
//		constpart_fore[i] = weight[i]/( sqrt(det_covariance_fore[i]) );
//		constpart_back[i] = weight[i]/( sqrt(det_covariance_back[i]) );
		doubleCheck(constpart_fore[i]);
		doubleCheck(constpart_back[i]);
//		cout << "FOREGROUND : " << constpart_fore[i] << " , BACKGROUND : " << constpart_back[i] << endl;
	}
}

