/*
 * common.h
 *
 *  Created on: 2010/11/16
 *      Author: umakatsu
 *      共通のパラメータ変数を設定するヘッダ
 */

#ifndef COMMON_H_
#define COMMON_H_

#include "define.h"
#include "OpenGL.h"
#include "Image/BmpFileIO.h"
#include <string>

// OpenCV画像を既存タイプの配列に変換するヘッダ
#include "myCVmethod.h"
// Vectorクラスの処理関数を使うヘッダ
#include "myVector.h"

/* 全体 */
const int D = 3;	// dimension
const int size = 128;	//The size of object voxel
const ulint ALLVOXELS = (size+1)*(size+1)*(size+1);
const GLint LINEWIDTH = 5;
const GLint INPUTWIDTH = 10;

enum state{NEW , INIT , READY, RUN , DYNAMIC , INCLUDE , EXCLUDE , IDLE}; //State of Reconstruction Process
enum quality{ BAD, DODGY, GOOD};	//State of Tracking
enum mState{TRANS , ROTATE , ENABLE , COPY , DISABLE };	//State of A Model
enum mode{MAPINIT , RECONSTRUCT , AUTHORING , COMMAND, INTERACTION};	//State of System Mode 2011.3.16


const GLdouble RED[]    = {1.0 , 0.0 , 0.0};
const GLdouble GREEN[]  = {0.0 , 1.0 , 0.0};
const GLdouble BLUE[]   = {0.0 , 0.0 , 1.0};
const GLdouble YELLOW[] = {1.0 , 1.0 , 0.0};
const GLdouble CIAN[]   = {0.0 , 1.0 , 1.0};
const GLdouble PURPLE[] = {1.0 , 0.0 , 1.0};
const GLdouble WHITE[]  = {1.0 , 1.0 , 1.0};

#define WIDTH 640
#define HEIGHT 480

/* 前景と背景のラベル付け */
#define FOREGROUND 	'f'
#define BACKGROUND 	'b'
#define CHECKREGION	'c'
#define NOLABEL		'n'

//占有率
const double OCCUPIED = 1.0;
const double EMPTY    = 0.0;

/* ビデオ画像の上下のバッファ */
const int top_buf = 0;
const int bottom_buf = 0;

/* GMM関連 */
const int 		CLASSNUMBER = 8;	//混合数
const double  EPSILON = (1.0/65536.0); //収束範囲
const int 		iterate = 10; //反復回数

struct GaussDistribution{
	SCALAR *	Mu;
	SCALAR *	invS;
	SCALAR 	detS;
	SCALAR		weight;
	GaussDistribution(int DIM){
		Mu		= new SCALAR[DIM];
		invS	= new SCALAR[DIM * DIM];
		detS	= 0.0;
		weight = 0.0;
	}
	~GaussDistribution(){
		delete[] Mu;
		delete[] invS;
	}
};

std::string inline getMessageState(const state & rhs){
	switch(rhs){
		case NEW:		return "No Data";
		case INIT: 	return "Init";
		case DYNAMIC: return "Dynamic";
		case EXCLUDE: return "Exclusion";
		case INCLUDE: return "Inclusion";
		case READY:	return "Ready";
		case RUN:		return "Update";
		case IDLE:		return "Idle";
		default:		return "ERROR";
	}
}

/*
 * モノクロのビットマップファイルを生成.
 */
void inline writeMonoBitmapfile(const char *filename, const int & image_width, const int & image_height, uchar*result )
{
	/* RGB形式で書き込み */
	unsigned char	*result_image = new unsigned char[ image_width * image_height * 3 ];
	REP(j,image_height){
		REP(i,image_width){
			float	r, g, b;
			int n = i + (image_height-1-j)*image_width;
			if ( result[ i + j*image_width ] == 255) {
				r = g = b = 1.0f;
			} else {
				r = g = b = 0.0f;
			}
			result_image[ n * 3 ] = ( r * 255.0f );
			result_image[ n * 3 + 1 ] = ( g * 255.0f );
			result_image[ n * 3 + 2 ] = ( b * 255.0f );
		}
	}

	BmpFileIO	b;
	b.WriteRGB( filename, image_width, image_height, result_image );
	delete [] result_image;
}

/*
 * カラーのビットマップファイルを生成.
 */
void inline writeColorBitmapfile(const char *filename, const int & image_width, const int & image_height, uchar*result )
{
	/* RGB形式で書き込み */
	unsigned char	*result_image = new unsigned char[ image_width * image_height * 3 ];
	REP(j,image_height){
		REP(i,image_width){
			float	c[3];
			int n = i + (image_height-1-j)*image_width;
			REP(k,3){
				c[k] = 1 / static_cast< float >(result[ i + j*image_width + k] );
				result_image[ n * 3 + k] = ( c[k] * 255.0f );
			}
		}
	}

	BmpFileIO	b;
	b.WriteRGB( filename, image_width, image_height, result_image );
	delete [] result_image;
}
#endif /* COMMON_H_ */
