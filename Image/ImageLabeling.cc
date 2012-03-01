/*
 * ImageLabeing.cc
 *
 *  Created on: 2010/11/18
 *      Author: umakatsu
 *      2-dimensional binary imageに対してのラベリングを行うためのクラス
 */

#include "Image/ImageLabeling.h"
#include <stdio.h>
#include <fstream>
#include <vector>

namespace PTAMM{
//const definition
#define DEBUG 0

using namespace std;

namespace{
	template< class T > short int Filtering(int i , int j , T **src , T filter[3][3])
//	short int Filtering(int i , int j , short int **src, short filter[3][3])
	{
		return(src[i-1][j-1]*filter[0][0]+src[i-1][j]*filter[1][0]+src[i-1][j+1]*filter[2][0]+src[i][j-1]*filter[0][1]+src[i][j]*filter[1][1]+src[i][j+1]*filter[2][1]+src[i+1][j-1]*filter[0][2]+src[i+1][j]*filter[1][2]+src[i+1][j+1]*filter[2][2]);
	}
}

ImageLabeling::ImageLabeling( void ){}

/**
 * @param input : binary image
 */
ImageLabeling::ImageLabeling(int size_x , int size_y , unsigned char *input)
: w(size_x) , h(size_y) , src(input)
{
	det 	= new short[ w * h ];
}

ImageLabeling::~ImageLabeling( void )
{
	if(det) delete[] det;
}

/*
 * ラプラシアンを行うメソッド
 * 輪郭抽出のために用いる
 */
//template <class T>
//void ImageLabeling::laplacian( T **result , T **region){
void ImageLabeling::laplacian( short int **result , short int **region){
	short int filter[3][3]={{1,1,1},{1,-8,1},{1,1,1}};	//ラプラシアンのフィルタ
	for(int y=1; y<(h-1); y++){
		for(int x=1; x<(w-1); x++){
			if(Filtering( x , y , result , filter) >= 0){
				region[x][y] = 0;
			}
			else{
				region[x][y] = 255;
			}
		}
	}
}

void ImageLabeling::run( void ){

	// 連結領域抽出の実行 /////////////////////////////////////////////////////
	labeling.Exec( src, det , w, h, true, 0 );

	// 引数は順に
	// 入力画像の先頭アドレス(unsigned char *)
	// 出力画像の先頭アドレス(short *)
	// 画像の幅(int)
	// 画像の高さ(int)
	// 領域の大きさ順にソートするか(bool) - true:する false:しない
	// 消去する小領域の最大サイズ(int) - これ以下のサイズの領域を消去する

	// 処理結果は det(が指す画像バッファ)に格納される。

	// また、各連結領域の情報は labeling 内に保持されており、
	// メソッドを通じて取り出すことができる。この情報は
	// 次の画像のラベリングを実行すると、消去される。
}
}
