/*
 * BrushAction.cc
 *
 *  Created on: 2010/12/14
 *      Author: umakatsu
 */

#include "Reconstruction/BrushAction.h"
#include "Reconstruction/GMM.h"
/* created by Umakatsu 2010/11/3  */
#include "Reconstruction/GraphCut.h"
/* created by Umakatsu 2010/11/12 */
#include "Image/VideoData.h"
/* created by Umakatsu 2010/11/18 */
#include "Image/ImageLabeling.h"
/* OpenCV画像を既存タイプの配列 に変換するヘッダ */
#include "cvConverter.h"
#include "myCVmethod.h"

namespace PTAMM{

BrushAction::BrushAction(int width , int height , VideoData *video)
: w(width) , h(height) , vd(video)
{
	binary = new unsigned char[w * h];
	result = new short int[w * h];
}

BrushAction::~BrushAction(){
//	delete[] binary;
	delete[] result;
}

void BrushAction::excludeAction(const TooN::SE3<> & currentPose , Stroke *silhouette , ATANCamera & camera){
	/* Extracting stroke part */
	/* シルエット領域を以前計算した世界座標系から画像平面上に逆投影する */
	unsigned char * sbuf = new unsigned char[w*h];
	vd->BackProjectionFromWorldToImage(sbuf , currentPose , silhouette , camera);
	IplImage * d2 =mycvConvertIplImage(sbuf ,	w , h , 1);
	//雑音除去
	cvDilate(d2 , d2 );
	cvErode(d2 , d2 );
	mycvConvertRegularType(d2 , sbuf , 1);

	vd->replaceLabel((unsigned char)(255) , sbuf , FOREGROUND , CHECKREGION);

	char *img = vd->getImg();
	/* creating the binary image */
	REP(i , w*h){
		if(img[i] == FOREGROUND)	binary[i] = 255;
		else						binary[i] = 0;
	}

	/* labeling run */
	ImageLabeling *imglabel = new ImageLabeling(w , h , binary);
	imglabel->run();
	result = imglabel->getDet();
	REP(i,w*h)
		if(result[i] == 1 )	binary[i] = 255;
		else 					binary[i] = 0;
	// calculate the gravity of the region have max area
	IplImage *src_img =mycvConvertIplImage(binary , w , h , 1);
	mycvGetGravity(src_img , foreground_coord_x , foreground_coord_y);
	//memory release
//	delete imglabel;
	delete[] sbuf;
	cvReleaseImage(&src_img);
	cvReleaseImage(&d2);
}

void BrushAction::includeAction(const TooN::SE3<> & currentPose , Stroke *silhouette , ATANCamera & camera){
	/* Extracting stroke part */
	/* シルエット領域を以前計算した世界座標系から画像平面上に逆投影する */
	unsigned char * sbuf = new unsigned char[w * h];
	vd->BackProjectionFromWorldToImage(sbuf , currentPose , silhouette , camera );
	IplImage * d2 =mycvConvertIplImage(sbuf ,	w , h , 1);
	cvDilate(d2 , d2);
	cvErode(d2 , d2);
	mycvConvertRegularType(d2 , sbuf , 1);
	vd->replaceLabel((unsigned char)(255) , sbuf , FOREGROUND , NOLABEL);
	char *img = vd->getImg();
	/* creating the binary image */
	REP(i , w*h){
		if(sbuf[i] == 255 )	binary[i] = 255;
		else 					binary[i] = 0;
		if(img[i] == CHECKREGION){
			binary[i] = 255;
		}
	}

	/* 前景の重心座標を抽出 */
	IplImage *src_img =mycvConvertIplImage(binary , w , h , 1);
	mycvGetGravity(src_img , foreground_coord_x , foreground_coord_y);
	//labeling run
	ImageLabeling *imglabel = new ImageLabeling(w , h , binary);
	imglabel->run();
	short int * maxarea = imglabel->getDet();
	REP(i,w*h){
		result[i] = maxarea[i];
	}
	//memory release
	delete imglabel;
	delete[] sbuf;
	cvReleaseImage(&src_img);
	cvReleaseImage(&d2);

}
}
